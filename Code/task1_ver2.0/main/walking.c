#include "walking.h"
#include "motor.h"
#include "ultrasonic.h"
#include <math.h>
#include <stdlib.h>

/* ------------------------ 现场标定区 ------------------------
 * 轮子命名按参考底盘：L=左轮、R=右轮、B=后轮。
 * 实车已确认：A=左轮、C=右轮、B=后轮；三路电机逆时针均为正。
 * 如果后续重新接线或改装轮位，只改下面两组映射，不改控制逻辑。
 */
#define WHEEL_TO_MOTOR_L   MOTOR_A
#define WHEEL_TO_MOTOR_R   MOTOR_C
#define WHEEL_TO_MOTOR_B   MOTOR_B

static const int s_motor_sign[MOTOR_NUM] = { +1, +1, +1 };

/*
 * 同一 PWM 下 C 轮实测略快于 A 轮。这里是电机输出标定，不改变运动学方向；
 * 后续如仍有直行偏差，只在 0.85~1.00 间微调 C 的系数。
 */
static const float s_motor_gain[MOTOR_NUM] = { 1.00f, 1.00f, 0.90f };

/*
 * 正前进（车头方向）的现场标定：中心线实测旧组合 A 正、C 反会后退，
 * 因此仅翻转前进分量。保留 turn 分量不变，避免改变已经单独标定的转向方向。
 * forward > 0 时：A 反转、C 正转、B 停止。
 */
#define FORWARD_SIGN_L     (-1)
#define FORWARD_SIGN_R     (+1)

/*
 * 实车循迹验证：黑线在右侧时旧 B 转向分量会使车继续向右偏。
 * A/C 的转向分量保持不变，仅翻转后轮 B 的转向分量。
 */
#define TURN_SIGN_L        (+1)
#define TURN_SIGN_R        (+1)
#define TURN_SIGN_B        (-1)

/* 后轮只承担转向；0.65 倍介于原 0.60 与死区补偿后的等效 0.70 之间的实测折中。 */
#define TURN_B_GAIN         0.68f

/* 若误差为正（黑线在车体右侧）时实车反而向左修正，把它改成 -1。 */
#define STEERING_SIGN       (+1)

/* ---- 平移（横移）标定 v3（2026-08-27 按实车几何与 MODE=5 现象修正）----
 * 实车结构（已确认）：三轮全向底盘，三驱动轮朝外互成 120°；
 *   B 在【正后方】，A 在【右前方】，C 在【左前方】。
 *
 * 运动学要点（s_i 为第 i 轮沿切向的等效转速）：
 *   - 自转角速度 ω ∝ (s_A+s_B+s_C)：纯平移必须令三轮有效转速之和为零；
 *   - 纯左移要求 s_A=s_C=+v/2、s_B=-v（B 转速大小 = A+C 之和，比值 1:1:-2）。
 *
 * ⚠️ 关键发现：B 电机的"正转"方向约定与 A/C 相反（安装镜像所致）。
 *   证据：MODE=5 旧参数下左移脉冲强顺时针自旋、右移脉冲强逆时针自旋，
 *   三轮有效转速全部同号——只有 B 符号约定与 A/C 相反才能同时解释两拍现象，
 *   并与"前进需 A 反 C 正"、"转向翻转 B 分量后弯道可过"两项已验证事实相容。
 *
 * 由此，strafe>0（向左平移）需要三条电机指令同为负：
 *   m_A=-v/2, m_C=-v/2, m_B=-v  →  三个 SIGN 全取 -1，B_GAIN≈2。
 *
 * GAIN 取 1.75 而非理想 2.0 的原因：混控归一化后 B 占空比 100%、
 * A/C 仅约 57%/51%，电机低占空段出力不足（死区/亚线性），若按 2.0 执行，
 * B 实际转速会超过 A+C 之和 → 残余自旋。按死区模型
 *     g = 190/(100 + 死区%)   （已计入 C 轮 0.90 全局增益）
 * 死区 8%~12% 对应 g≈1.70~1.76，取 1.75。
 *
 * 微调指引（MODE=5 着地短脉冲观察）：
 *  - 左移脉冲仍带顺时针自旋 → 减小 STRAFE_B_GAIN（每次 -0.05）
 *  - 带逆时针自旋           → 增大 STRAFE_B_GAIN（每次 +0.05）
 *  - 平移方向整体反了       → 三个 STRAFE_SIGN_* 同时取反
 *
 * 迭代记录：1.75 实测仍有轻微逆时针自旋（B 相对偏慢）→ 上调 1.80。 */
#define STRAFE_SIGN_L       (-1)    /* A：右前轮 */
#define STRAFE_SIGN_R       (-1)    /* C：左前轮 */
#define STRAFE_SIGN_B       (-1)    /* B：正后轮（符号约定与 A/C 相反！） */
#define STRAFE_B_GAIN       1.80f

/* ------------------------ 控制参数区 ------------------------ */
#define LINE_BASE_SPEED_PCT       13
#define LINE_MIN_SPEED_PCT         8
#define LINE_MAX_TURN_PCT         10
#define LINE_KP                    3.0f
#define LINE_KD                    0.5f  /* 少量微分阻尼，抑制直线来回扭动 */
#define LINE_TURN_SLEW_PCT         3  /* 每个 20ms 控制周期最大转向变化量 */
#define LINE_ERROR_FILTER_ALPHA  0.30f /* 数字红外的低通系数，越小越稳但越慢 */
#define LINE_CENTER_DEADBAND      0.45f /* 小误差不转向，避免直线来回扭动 */
#define LINE_SEARCH_SIDE_MIN_ERROR 0.25f /* 参与方向投票所需的最小原始偏差 */
#define LINE_SIDE_ACCUM_WINDOW     10  /* 方向投票窗口（拍数，20ms/拍 ≈ 200ms） */
#define LINE_SIDE_FLIP_MIN          2  /* 累计票达到该值才允许翻转找线方向 */
#define LINE_LOST_GRACE_MS        60u
#define LINE_LOST_FORWARD_PCT      7
#define LINE_SEARCH_TIMEOUT_MS     0u /* 0=只红外测试时持续找线，直到重新见线 */
#define LINE_SEARCH_TURN_PCT       12
#define LINE_CORNER_SPEED_PCT      7
#define LINE_CORNER_TURN_PCT      12
/*
 * 锐角弯专用找出口参数：
 * - 结束条件始终是确认重新见线，不是定时结束；
 * - 重试时间只用于判断首选方向很可能错误，届时仅反向尝试一次。
 */
#define LINE_CORNER_EXIT_CONFIRM_SAMPLES  3u
#define LINE_CORNER_RETRY_MS           1000u
/* 反向重试后仍找不到出口的总兜底时间：超时进故障停车，避免无限原地空转。 */
#define LINE_CORNER_TIMEOUT_MS         5000u

#define OBSTACLE_TRIGGER_CM       15u
#define OBSTACLE_CLEAR_CM         21u
#define OBSTACLE_CONFIRM_SAMPLES   2u
#define OBSTACLE_SAMPLE_GAP_MS    350u

/* ---- 终点 T 线停车参数（2026-08-27 新增）----
 * 避障完成回到巡线后，四路全黑连续 FINISH_CONFIRM_SAMPLES 拍即判定
 * 到达赛道末尾 T 线（横杆宽度足以同时压住四个探头），刹车结束任务。
 * 赛道前半段该检测不生效（未武装），不会误停。 */
#define FINISH_CONFIRM_SAMPLES     3u

/* ============================================================
 * 避障时序参数 —— 赛道标定接口（2026-08-27）
 *
 * 本赛道（docs/避障图.JPG）：车沿与木板垂直的直线接近木板，
 * 从【左侧】绕过木板，进入板内的黑线区域，随后恢复原巡线。
 * 序列：停车 → 左移 → 前进（越过木板）→ 找回黑线 → RUN。
 *
 * ⚠️ 当前数值为"凭感觉"的初值（2026-08-27），联调时按现象只改对应参数：
 *   ┌────────────────────────┬──────────────────────────────────┐
 *   │ 联调现象               │ 只改这个参数（本文件内搜索名字）  │
 *   ├────────────────────────┼──────────────────────────────────┤
 *   │ 左移不够/过头          │ AVOID_STRAFE_DIST_CM（±5cm 步进）│
 *   │ 前进撞板/出板太远      │ AVOID_FORWARD_DIST_CM（±5cm）    │
 *   │ 平移太慢/太快          │ STRAFE_SPEED_CM_S（实测后填真值）│
 *   │ 避障前进太慢/太快      │ FORWARD_SPEED_CM_S（实测后填）   │
 *   │ 找线扫错方向           │ AVOID_SCAN_TURN_SIGN（±1 翻转）  │
 *   │ 找线超时进 FAILSAFE    │ 加大 AVOID_FIND_LINE_MS 或       │
 *   │                        │ AVOID_SCAN_TURN_PCT              │
 *   │ 平移带自旋             │ 文件顶部 STRAFE_B_GAIN           │
 *   └────────────────────────┴──────────────────────────────────┘
 * ============================================================ */

/* [标定#6] 电机死区下限：低于该值轮子可能不转。MODE=5 实测平移/前进均能
 * 正常启动，说明当前三档速度(12/15/10%)高于死区，暂按 8% 保留。 */
#define MOTOR_DEADZONE_PCT       8

/* [标定#1/#2/#4] 平移方向与纯度 —— 已标定通过（见文件顶部 v3 区）：
 * SIGN 全 -1、B_GAIN=1.80。此处不再重复定义。 */

/* [标定#3] 平移速度：初值按 MODE=5 的 12% 档估 15cm/s。
 * 联调前建议用卷尺量一次真实值替换（左移位移÷时间）。 */
#define STRAFE_SPEED_CM_S   10.0f   /* 初值，待实测校准 */

/* [标定#5] 前进速度：初值按 15% 档估 20cm/s。 */
#define FORWARD_SPEED_CM_S  10.0f   /* 初值，待实测校准 */

/* ---- [赛道几何 #14~#17] 凭感觉的初值（单位 cm），联调时按上表调整----
 * 参照 docs/避障图.JPG 目测：木板横向遮挡约 40cm、纵深约 15cm，
 * 超声 18cm 触发时车头距板约 18cm。
 *   左移 = 触发点横距(~20) + 板宽一半(~20) + 余量(5) ≈ 45cm；
 *   前进 = 剩余距离(~18) + 纵深(~15) + 出板余量(10) ≈ 43cm。 */
#define AVOID_STRAFE_DIST_CM    12.0f   /* 初值：左移总距离 */
#define AVOID_FORWARD_DIST_CM   14.0f   /* 初值：前进越过距离 */

/* 由上面的距离/速度自动换算出各阶段时长（勿手改，改距离或速度即可）。 */
#define AVOID_STRAFE_MS(dist_cm) \
    ((uint32_t)((dist_cm) / STRAFE_SPEED_CM_S * 1000.0f))
#define AVOID_FORWARD_CALC_MS \
    ((uint32_t)(AVOID_FORWARD_DIST_CM / FORWARD_SPEED_CM_S * 1000.0f))

/* ---- 固定时序（不依赖标定）---- */
#define AVOID_STOP_MS            120u   /* 确认障碍后的短暂停车 */

/* ---- 找线阶段（AVOID_FIND_LINE）----
 * 绕过木板后低速前进+小幅扫描找回黑线；见线即恢复 RUN。
 * [标定#17] 扫描偏置方向：初值 +1（左偏扫）；若试跑发现扫向错误一侧
 *   （远离 T 干线），翻转成 -1。 */
#define AVOID_FIND_LINE_MS      2500u   /* 找线总超时；超时进 FAILSAFE_STOP */
#define AVOID_STRAFE_SPEED_PCT   12     /* 平移速度档（须 > MOTOR_DEADZONE_PCT） */
#define AVOID_FORWARD_SPEED_PCT  15     /* 避障前进速度档 */
#define AVOID_FIND_SPEED_PCT     10     /* 找线前进速度档 */
#define AVOID_SCAN_TURN_PCT       5     /* 找线扫描转向幅度 */
#define AVOID_SCAN_TURN_SIGN    (+1)

static const char *const s_state_name[] = {
    "RUN",
    "LINE_SEARCH",
    "CORNER_SEARCH",
    "AVOID_STOP",
    "AVOID_STRAFE_LEFT",
    "AVOID_FORWARD",
    "AVOID_FIND_LINE",
    "FINISH_STOP",
    "FAILSAFE_STOP",
};

static car_state_t s_state;
static uint32_t s_state_enter_ms;
static uint32_t s_line_lost_since_ms;
static uint32_t s_last_distance_sample_ms;
static uint8_t s_obstacle_samples;
static uint8_t s_line_reacquire_samples;
static uint8_t s_corner_exit_samples;
static bool s_line_lost_timing;
static bool s_last_line_was_corner;
static bool s_corner_retry_used;
static bool s_output_enabled;
static float s_last_error;
/* ---- 终点 T 线停车（2026-08-27 新增）----
 * s_finish_armed：只有避障完成回到巡线后才置位，避免赛道前半段的
 * 宽线/路口全黑被误判成终点；s_finish_samples：全黑连续确认计数。 */
static bool s_finish_armed;
static uint8_t s_finish_samples;
/* +1=黑线最后在右侧，应使用正 turn；-1=最后在左侧，应使用负 turn。 */
static int s_last_line_side;
/* 方向投票计数器：正=近期线在右侧占优，负=左侧占优。 */
static int s_line_side_accum;
static int s_corner_turn_side;
static float s_filtered_error;
static bool s_error_filter_valid;
static walking_command_t s_last_command;

static int clamp_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms;  /* unsigned subtraction also handles wraparound */
}

static void enter_state(car_state_t state, uint32_t now_ms)
{
    s_state = state;
    s_state_enter_ms = now_ms;
    s_line_reacquire_samples = 0;
}

/*
 * 当前底盘三轴混控（forward / turn / strafe）：
 *   纯前进（已实车校正）：L=-v, R=+v, B=0
 *   原地旋转：左右轮同向转向分量、后轮 B 反向分量（0.65 倍补偿）
 *   平移（strafe>0=左）：L/R 按 STRAFE_SIGN、后轮 B 反向（增益待标定）
 * 循迹只用到 forward/turn（strafe=0），输出与旧版完全一致，不影响循迹。
 * 三轮比例超出 100% 时等比缩放，避免逐轮截断改变运动方向。
 */
static void apply_body_command(int forward, int turn, int strafe)
{
    s_last_command.forward_percent = forward;
    s_last_command.turn_percent = turn;
    s_last_command.strafe_percent = strafe;

    float wheel[MOTOR_NUM] = {0};
    wheel[WHEEL_TO_MOTOR_L] = (float)(FORWARD_SIGN_L * forward + TURN_SIGN_L * turn + STRAFE_SIGN_L * strafe);
    wheel[WHEEL_TO_MOTOR_R] = (float)(FORWARD_SIGN_R * forward + TURN_SIGN_R * turn + STRAFE_SIGN_R * strafe);
    wheel[WHEEL_TO_MOTOR_B] = (float)(TURN_SIGN_B * turn) * TURN_B_GAIN
                            + (float)(STRAFE_SIGN_B * strafe) * STRAFE_B_GAIN;

    float max_abs = 100.0f;
    for (int i = 0; i < MOTOR_NUM; i++) {
        const float value_abs = fabsf(wheel[i]);
        if (value_abs > max_abs) {
            max_abs = value_abs;
        }
    }

    if (!s_output_enabled) {
        return;
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        const int percent = (int)lroundf(wheel[i] * s_motor_gain[i] * 100.0f / max_abs);
        motor_set_speed(i, percent * s_motor_sign[i]);
    }
}

static void apply_brake(void)
{
    s_last_command.forward_percent = 0;
    s_last_command.turn_percent = 0;
    s_last_command.strafe_percent = 0;
    if (s_output_enabled) {
        motor_stop_all();
    }
}

/* 仅测试/标定用：绕过状态机直接驱动三轴（见 walking.h）。 */
void walking_test_drive(int forward, int turn, int strafe)
{
    apply_body_command(forward, turn, strafe);
}

/* 避免数字红外从中心跳到外侧时，转向指令在一个控制周期内突变。 */
static int limit_line_turn_slew(int requested_turn)
{
    const int previous_turn = s_last_command.turn_percent;
    return clamp_int(requested_turn,
                     previous_turn - LINE_TURN_SLEW_PCT,
                     previous_turn + LINE_TURN_SLEW_PCT);
}

/*
 * 外侧探头参与命中即视为接近直角/锐角弯：a&&!c 或 d&&!b
 * （覆盖外侧单点 a/d，也覆盖外侧+内侧 a&&b / c&&d）。
 * 该判定只负责两件事：锁定找线方向、置弯道标志（供全白后进 CORNER_SEARCH），
 * 不直接决定控制动作；动作区分见 run_line_controller。
 */
static bool infrared_is_sharp_corner(const infrared_pattern_t *pat)
{
    return (pat->a && !pat->c) || (pat->d && !pat->b);
}

/*
 * 外侧单探头命中、相邻内侧未命中（a&&!b / d&&!c）：马上就要丢线，
 * 只有这种图样才用固定大转向的弯道动作；a&&b / c&&d 只是偏得多，走正常 PD。
 */
static bool infrared_is_corner_imminent(const infrared_pattern_t *pat)
{
    return (pat->a && !pat->b) || (pat->d && !pat->c);
}

/*
 * 锐角弯出口只接受与转弯方向同侧的内侧探头：左转时线必须从左内(b)进入，
 * 右转时从右内(c)进入。对侧探头先见线说明转过头或扫到了身后的来路线，
 * 不能算出口；仅凭外侧探头的短暂命中也不算出口。
 */
static bool infrared_is_corner_exit_candidate(const infrared_pattern_t *pat,
                                              int turn_side)
{
    return (turn_side < 0) ? (pat->b != 0) : (pat->c != 0);
}

/*
 * 找线方向基于未经滤波的原始红外位置，并对最近若干拍做累计投票：
 * 单拍抖动只把计数拨动 1，翻不了方向；丢线前约 200ms 内的主导侧说了算。
 * 不能复用 s_last_error：直线防抖的死区会把小误差置零，导致全白后总落到默认方向。
 */
static void remember_last_line_side(float raw_error)
{
    if (raw_error > LINE_SEARCH_SIDE_MIN_ERROR) {
        if (s_line_side_accum < LINE_SIDE_ACCUM_WINDOW) {
            s_line_side_accum++;
        }
    } else if (raw_error < -LINE_SEARCH_SIDE_MIN_ERROR) {
        if (s_line_side_accum > -LINE_SIDE_ACCUM_WINDOW) {
            s_line_side_accum--;
        }
    }

    if (s_line_side_accum >= LINE_SIDE_FLIP_MIN) {
        s_last_line_side = +1;
    } else if (s_line_side_accum <= -LINE_SIDE_FLIP_MIN) {
        s_last_line_side = -1;
    }
}

static float filter_line_error(float raw_error)
{
    if (!s_error_filter_valid) {
        s_filtered_error = raw_error;
        s_error_filter_valid = true;
    } else {
        s_filtered_error += LINE_ERROR_FILTER_ALPHA * (raw_error - s_filtered_error);
    }

    return (fabsf(s_filtered_error) < LINE_CENTER_DEADBAND) ? 0.0f : s_filtered_error;
}

static bool state_accepts_obstacle_trigger(void)
{
    /*
     * 锐角原地找线期间不接受超声触发：定时长避障动作会打断转弯出口确认，
     * 且旋转中测到的“障碍”常是赛道边物，容易把弯道恢复引入 FAILSAFE_STOP。
     */
    return s_state == CAR_STATE_RUN ||
           s_state == CAR_STATE_LINE_SEARCH;
}

static bool obstacle_confirmed(uint16_t distance_cm, uint32_t now_ms)
{
    if (distance_cm == ULTRASONIC_INVALID) {
        return false;  /* 无新样本时不增加也不清空计数 */
    }

    if (elapsed_ms(now_ms, s_last_distance_sample_ms) > OBSTACLE_SAMPLE_GAP_MS) {
        s_obstacle_samples = 0;
    }
    s_last_distance_sample_ms = now_ms;

    if (distance_cm <= OBSTACLE_TRIGGER_CM) {
        if (s_obstacle_samples < OBSTACLE_CONFIRM_SAMPLES) {
            s_obstacle_samples++;
        }
    } else if (distance_cm >= OBSTACLE_CLEAR_CM) {
        s_obstacle_samples = 0;
    }

    return s_obstacle_samples >= OBSTACLE_CONFIRM_SAMPLES;
}

static void run_line_controller(const infrared_pattern_t *pat, uint32_t now_ms)
{
    float error = 0.0f;
    const bool has_line = infrared_get_error(pat, &error);

    if (!has_line) {
        if (!s_line_lost_timing) {
            s_line_lost_timing = true;
            s_line_lost_since_ms = now_ms;
        }

        /*
         * 锐角进入后出现全白，说明外侧探头已越过拐点。
         * 立刻停止前进，按最后确认的黑线侧原地转；不再用固定毫秒数盲目前冲。
         */
        if (s_last_line_was_corner) {
            s_corner_turn_side = s_last_line_side;
            s_corner_exit_samples = 0;
            s_corner_retry_used = false;
            enter_state(CAR_STATE_CORNER_SEARCH, now_ms);
            apply_body_command(0,
                               s_corner_turn_side * STEERING_SIGN * LINE_SEARCH_TURN_PCT, 0);
            return;
        }

        if (elapsed_ms(now_ms, s_line_lost_since_ms) < LINE_LOST_GRACE_MS) {
            const int held_turn = clamp_int(
                (int)lroundf(STEERING_SIGN * LINE_KP * s_last_error),
                -LINE_MAX_TURN_PCT, LINE_MAX_TURN_PCT);
            apply_body_command(LINE_LOST_FORWARD_PCT, limit_line_turn_slew(held_turn), 0);
        } else {
            enter_state(CAR_STATE_LINE_SEARCH, now_ms);
        }
        return;
    }

    s_line_lost_timing = false;

    /* 在滤波/死区前记录方向，供全白后的原地找线使用。 */
    remember_last_line_side(error);

    /* 全黑通常是较宽的线或路口：保持低速直行，不执行未经赛道验证的转向。 */
    if (infrared_all_black(pat)) {
        s_last_error = 0.0f;
        s_filtered_error = 0.0f;
        s_error_filter_valid = true;
        s_last_line_was_corner = false;
        apply_body_command(LINE_MIN_SPEED_PCT, 0, 0);
        return;
    }

    if (infrared_is_sharp_corner(pat)) {
        /*
         * 外侧探头命中是方向的最强证据：直接锁定找线方向并把投票计数
         * 置到对应侧，不等逐拍积累（20ms 采样可能只采到一两拍弯道图样）。
         * 方向锁定对全部弯道图样生效，与控制动作无关。
         */
        s_last_line_side = (error < 0.0f) ? -1 : +1;
        s_line_side_accum = s_last_line_side * LINE_SIDE_FLIP_MIN;
        s_last_line_was_corner = true;

        if (infrared_is_corner_imminent(pat)) {
            /* 外侧单点、马上要丢线：低速+固定大转向，抢在丢线前把车转过来。 */
            s_last_error = error;
            s_filtered_error = error;
            s_error_filter_valid = true;
            const int corner_turn = (error < 0.0f ? -1 : +1) *
                                    STEERING_SIGN * LINE_CORNER_TURN_PCT;
            apply_body_command(LINE_CORNER_SPEED_PCT, limit_line_turn_slew(corner_turn), 0);
            return;
        }
        /* a&&b / c&&d：只是偏得多，不落弯道动作，继续走下面的正常 PD。 */
    } else {
        s_last_line_was_corner = false;
    }

    const float filtered_error = filter_line_error(error);
    const float derivative = filtered_error - s_last_error;
    s_last_error = filtered_error;
    int turn = (int)lroundf(STEERING_SIGN *
                            (LINE_KP * filtered_error + LINE_KD * derivative));
    turn = clamp_int(turn, -LINE_MAX_TURN_PCT, LINE_MAX_TURN_PCT);
    turn = limit_line_turn_slew(turn);

    /* 转弯越急，前进速度越低，减少冲出赛道。 */
    int forward = LINE_BASE_SPEED_PCT - abs(turn) / 2;
    forward = clamp_int(forward, LINE_MIN_SPEED_PCT, LINE_BASE_SPEED_PCT);
    apply_body_command(forward, turn, 0);
}

void walking_init(void)
{
    motor_init();
    s_output_enabled = false;
    walking_reset(0);
}

void walking_reset(uint32_t now_ms)
{
    enter_state(CAR_STATE_RUN, now_ms);
    s_line_lost_since_ms = now_ms;
    s_last_distance_sample_ms = now_ms;
    s_obstacle_samples = 0;
    s_line_reacquire_samples = 0;
    s_corner_exit_samples = 0;
    s_line_lost_timing = false;
    s_last_line_was_corner = false;
    s_corner_retry_used = false;
    s_last_error = 0.0f;
    s_last_line_side = +1;
    s_line_side_accum = 0;
    s_corner_turn_side = +1;
    s_filtered_error = 0.0f;
    s_error_filter_valid = false;
    s_finish_armed = false;      /* 复位后需重新经避障武装终点检测 */
    s_finish_samples = 0;
    s_last_command = (walking_command_t){0};
    if (s_output_enabled) {
        motor_stop_all();
    }
}

void walking_set_output_enabled(bool enabled)
{
    if (!enabled) {
        motor_stop_all();
        motor_enable(false);
        s_output_enabled = false;
        return;
    }

    motor_enable(true);
    s_output_enabled = true;
}

void walking_update(const infrared_pattern_t *pat,
                    uint16_t distance_cm,
                    uint32_t now_ms)
{
    if (pat == NULL) {
        enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
        apply_brake();
        return;
    }

    if (state_accepts_obstacle_trigger() &&
        obstacle_confirmed(distance_cm, now_ms)) {
        s_obstacle_samples = 0;
        enter_state(CAR_STATE_AVOID_STOP, now_ms);
    }

    switch (s_state) {
    case CAR_STATE_RUN:
        /*
         * 终点 T 线检测（仅在避障完成后武装）：四路全黑连续 3 拍
         * 判定到达赛道末尾，进 FINISH_STOP 永久停车。
         */
        if (s_finish_armed && infrared_all_black(pat)) {
            if (s_finish_samples < FINISH_CONFIRM_SAMPLES) {
                s_finish_samples++;
            }
            if (s_finish_samples >= FINISH_CONFIRM_SAMPLES) {
                enter_state(CAR_STATE_FINISH_STOP, now_ms);
                apply_brake();
                break;
            }
        } else {
            s_finish_samples = 0;
        }

        run_line_controller(pat, now_ms);
        break;

    case CAR_STATE_LINE_SEARCH: {
        float error;
        if (infrared_get_error(pat, &error)) {
            if (++s_line_reacquire_samples >= 2) {
                /* 干净重启滤波器，不把丢线前的旧误差带进 RUN。 */
                s_last_error = 0.0f;
                s_filtered_error = 0.0f;
                s_error_filter_valid = false;
                s_line_lost_timing = false;
                enter_state(CAR_STATE_RUN, now_ms);
                run_line_controller(pat, now_ms);
                break;
            }
        } else {
            s_line_reacquire_samples = 0;
        }

#if LINE_SEARCH_TIMEOUT_MS > 0u
        if (elapsed_ms(now_ms, s_state_enter_ms) >= LINE_SEARCH_TIMEOUT_MS) {
            enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
            apply_brake();
            break;
        }
#endif

        /*
         * 普通丢线找线也不在第一次见线时刹车：继续低速原地转，
         * 直到两拍连续确认后才回到巡线，避免黑白临界处反复停转。
         */
        apply_body_command(0, s_last_line_side * STEERING_SIGN * LINE_SEARCH_TURN_PCT, 0);
        break;
    }

    case CAR_STATE_CORNER_SEARCH: {
        float error;
        const bool has_line = infrared_get_error(pat, &error);

        /*
         * 这里不会因满 1000ms 而结束转弯。它仅说明首选方向尚未找到可信出口，
         * 因而反向尝试一次；此后仍由“连续见线”决定何时恢复巡线。
         * 注意不再重置 s_state_enter_ms：总超时从进入本状态起算。
         */
        if (!s_corner_retry_used &&
            elapsed_ms(now_ms, s_state_enter_ms) >= LINE_CORNER_RETRY_MS) {
            s_corner_turn_side = -s_corner_turn_side;
            s_corner_retry_used = true;
            s_corner_exit_samples = 0;
        }

        if (has_line &&
            infrared_is_corner_exit_candidate(pat, s_corner_turn_side)) {
            if (s_corner_exit_samples < LINE_CORNER_EXIT_CONFIRM_SAMPLES) {
                s_corner_exit_samples++;
            }

            if (s_corner_exit_samples >= LINE_CORNER_EXIT_CONFIRM_SAMPLES) {
                /* 干净重启滤波器，不把弯道里的瞬时误差带进 RUN。 */
                s_last_error = 0.0f;
                s_filtered_error = 0.0f;
                s_error_filter_valid = false;
                s_line_lost_timing = false;
                s_last_line_was_corner = false;
                enter_state(CAR_STATE_RUN, now_ms);
                run_line_controller(pat, now_ms);
                break;
            }

            /*
             * 确认期间先停住转动，让探头停在线上完成连续采样；
             * 否则原地旋转稍快时线在探头下停留不足确认窗口，计数被反复清零。
             */
            apply_body_command(0, 0, 0);
        } else {
            s_corner_exit_samples = 0;
            apply_body_command(0,
                               s_corner_turn_side * STEERING_SIGN * LINE_SEARCH_TURN_PCT, 0);
        }

        /* 总兜底：首选+反向都没找到出口，进故障停车而不是无限空转。 */
        if (elapsed_ms(now_ms, s_state_enter_ms) >= LINE_CORNER_TIMEOUT_MS) {
            enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
            apply_brake();
        }
        break;
    }

    case CAR_STATE_AVOID_STOP:
        apply_brake();
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_STOP_MS) {
            enter_state(CAR_STATE_AVOID_STRAFE_LEFT, now_ms);
        }
        break;

    case CAR_STATE_AVOID_STRAFE_LEFT:
        /* strafe>0 = 左移（方向由 STRAFE_SIGN_* 标定）。
         * 时长 = AVOID_STRAFE_DIST_CM / STRAFE_SPEED_CM_S，见顶部标定接口。 */
        apply_body_command(0, 0, AVOID_STRAFE_SPEED_PCT);
        if (elapsed_ms(now_ms, s_state_enter_ms) >=
                AVOID_STRAFE_MS(AVOID_STRAFE_DIST_CM)) {
            enter_state(CAR_STATE_AVOID_FORWARD, now_ms);
        }
        break;

    case CAR_STATE_AVOID_FORWARD:
        /* 直行前进越过木板；结束时整车应已出板、位于板内 T 线附近。 */
        apply_body_command(AVOID_FORWARD_SPEED_PCT, 0, 0);
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_FORWARD_CALC_MS) {
            enter_state(CAR_STATE_AVOID_FIND_LINE, now_ms);
        }
        break;

    case CAR_STATE_AVOID_FIND_LINE:
        /*
         * 绕过木板后向【右】平移找回黑线（2026-08-27 按实车联调修改：
         * 原方案"前进+转向扫描"实测表现为向右转，不符合赛道需求）。
         * 平移方向由 STRAFE_SIGN_* 标定保证 strafe<0=向右；任何非全白
         * 即视为见线，恢复 RUN 回到原巡线模式（滤波器干净重启，
         * 不带避障期旧误差）。超时仍未见到线则进 FAILSAFE_STOP。
         */
        if (!infrared_all_white(pat)) {
            s_last_error = 0.0f;
            s_filtered_error = 0.0f;
            s_error_filter_valid = false;
            s_line_lost_timing = false;
            /* 避障完成、重新见线：武装终点 T 线检测（此后全黑=终点） */
            s_finish_armed = true;
            s_finish_samples = 0;
            enter_state(CAR_STATE_RUN, now_ms);
            run_line_controller(pat, now_ms);
            break;
        }
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_FIND_LINE_MS) {
            enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
            apply_brake();
            break;
        }
        /* strafe<0 = 向右平移（速度复用避障平移档） */
        apply_body_command(0, 0, -AVOID_STRAFE_SPEED_PCT);
        break;

    case CAR_STATE_FINISH_STOP:
        /*
         * 到达赛道末尾 T 线：永久刹车，任务结束。
         * 与 FAILSAFE_STOP 不同，本状态不自动恢复——终点停车是预期行为。
         */
        apply_brake();
        break;

    case CAR_STATE_FAILSAFE_STOP:
    default:
        apply_brake();
        /*
         * 故障停车不是死锁：连续 5 拍稳定见线后自动恢复巡线。
         * 恢复前同样重置滤波器和丢线计时，避免带入停车前的旧状态。
         */
        if (!infrared_all_white(pat)) {
            if (++s_line_reacquire_samples >= 5) {
                s_last_error = 0.0f;
                s_filtered_error = 0.0f;
                s_error_filter_valid = false;
                s_line_lost_timing = false;
                enter_state(CAR_STATE_RUN, now_ms);
                run_line_controller(pat, now_ms);
            }
        } else {
            s_line_reacquire_samples = 0;
        }
        break;
    }
}

car_state_t walking_get_state(void)
{
    return s_state;
}

bool walking_is_finished(void)
{
    return s_state == CAR_STATE_FINISH_STOP;
}

const char *walking_state_name(car_state_t state)
{
    if ((unsigned)state >= (sizeof(s_state_name) / sizeof(s_state_name[0]))) {
        return "?";
    }
    return s_state_name[state];
}

walking_command_t walking_get_last_command(void)
{
    return s_last_command;
}

float walking_get_last_error(void)
{
    return s_last_error;
}
