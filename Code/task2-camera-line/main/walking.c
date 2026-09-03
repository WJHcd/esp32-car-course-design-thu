#include "walking.h"
#include "motor.h"
#include "ultrasonic.h"
#include "ball_task.h"
#include "camera_line.h"
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
/* 找线右移专用平移增益（2026-09-01 新增）：死区非线性使平衡所需的后轮
 * 增益随指令大小变化——低速右移时前轮被死区压缩更多、B 相对偏快，
 * 按左移 12% 标定的全局 STRAFE_B_GAIN 不适用。本值只在 AVOID_FIND_LINE
 * 状态生效（enter_state 每次切换状态恢复全局值），左移段不受影响。
 * 实车数据点：右移 10% 时 G=1.80 与 1.75、9% 时 G=1.65 均逆时针自旋
 * → 方向为减小；2026-09-01 下调至 1.45（若转成顺时针，+0.05 回调）。
 * 调参（着地观察右移脉冲）：仍逆时针自旋 → 每次 -0.05；反向 → +0.05。 */
#define STRAFE_B_GAIN_FIND  1.45f

/* ------------------------ 控制参数区 ------------------------ */
/* 直线基准速度（2026-09-01 现场 16→15）；弯道前进按 15-|turn|/2 联动 */
#define LINE_BASE_SPEED_PCT       15.0f
#define LINE_MIN_SPEED_PCT         8
#define LINE_MAX_TURN_PCT         10
#define LINE_KP                    3.0f
#define LINE_KD                    0.8f  /* 微分阻尼（2026-09-01 0.5→0.8），抑制直线来回扭动 */
#define LINE_TURN_SLEW_PCT         3  /* 每个 20ms 控制周期最大转向变化量 */
#define LINE_ERROR_FILTER_ALPHA  0.30f /* 摄像头误差的低通系数，越小越稳但越慢 */
/* 摄像头误差范围是 [-1,+1]；沿用红外的 0.45 死区会几乎不修正。 */
#define CAM_CENTER_DEADBAND       0.08f
#define LINE_SEARCH_SIDE_MIN_ERROR 0.12f /* 参与方向投票所需的最小原始摄像头偏差 */
/* 以下计数只在“新稳定摄像头帧”到来时更新，约 3fps。 */
#define LINE_SIDE_ACCUM_WINDOW      4
#define LINE_SIDE_FLIP_MIN          2  /* 累计票达到该值才允许翻转找线方向 */
#define LINE_LOST_GRACE_MS        60u
#define LINE_LOST_FORWARD_PCT      7
/* 找线超时（A1 修复）：原值 0 表示永不超时，丢线后原地找线会无限转圈。
 * 超时进 FAILSAFE_STOP 刹车；车静止后摄像头容易满足稳定见线，可自动恢复。 */
/* 5000ms：换向周期 1.6s 下两相位（先猜错再扫回）也能覆盖锐角出口
 * 所需的 ~120° 单侧摆幅，另留静止确认的余量。 */
#define LINE_SEARCH_TIMEOUT_MS  5000u
/* 找线转速（2026-09-01 10%→12%）：实车反馈转弯力度不足；且 12% 使后轮
 * B 分量达 8.2%、脱离死区开始真正出力，旋转显著加快。 */
#define LINE_SEARCH_TURN_PCT      12
/* 找线扫描换向周期（A3 修复）：在"上次见线侧"与反方向之间交替，
 * 避免方向猜错时一直朝错误方向空转。
 * 1600ms：10% 转速下单相位约覆盖 130°+，锐角出口（相对丢线航向
 * ~120°）在首个相位内即可扫到；旧值 1200ms 只够直角弯。 */
#define LINE_SEARCH_SWEEP_MS    1600u
/* 弯道前进速度（2026-09-01 现场 7→6.5） */
#define LINE_CORNER_SPEED_PCT    6.5f
/* 弯道转向力度（2026-09-01 11→10.5）：11 略强，回调 0.5 档 */
#define LINE_CORNER_TURN_PCT   10.5f
/*
 * 锐角弯专用找出口参数（2026-09-01 重构）：
 * - 取消 1000ms 定时反向：找线转速下 1s 转不满锐角出口所需的 ~120°，
 *   到点反向只会把已转过的角度退回去，方向猜对反而必败。改为按锁定
 *   方向单向持续旋转——方向即使猜错，出口线绕一圈（~240°）后仍会从
 *   锁定侧进入扫描带，5s 兜底足够覆盖；
 * - 结束条件始终是"重新见线并静止确认"，不是定时结束。
 */
#define LINE_CORNER_EXIT_CONFIRM_SAMPLES  2u
/* 出口误差下限（现场标定 0.50→0.30）：已贴近顶点宽黑区质心常驻的
 * 中央区域（|err|<0.3），此档位基本是"同侧见线即接受"。若车在顶点
 * 反复凑近/折返，往 0.5~0.7 回调；若出口接不住的现象仍在，问题可能
 * 不在此参数（查方向锁定侧别或摄像头帧时机）。 */
#define LINE_CORNER_EXIT_MIN_ERR     0.30f
/* 锐角单向找出口的最长时间；超时后直接转入可换向的 LINE_SEARCH，
 * 不再先刹停 2 秒。 */
#define LINE_CORNER_TIMEOUT_MS         5000u

/* 第一帧候选会让整车静止等待确认；若下一帧确认失败，先用短促较大转向
 * 克服 B/C 轮静摩擦，再恢复 LINE_SEARCH_TURN_PCT 的常规慢速扫描。 */
#define SEARCH_RESTART_KICK_TURN_PCT     14
#define SEARCH_RESTART_KICK_MS          100u

/* 已经真正经历过无线路面后，重新出现在画面中央的黑线也可作为弯道出口。
 * 这避免车头已经对准黑线，却因 |error|<LINE_CORNER_EXIT_MIN_ERR 被拒绝。 */
#define LINE_CORNER_CENTER_EXIT_ERR    0.25f

/* FAILSAFE 自动重启（2026-09-01）：故障停车不再是终点——刹停满此时长
 * 仍未被动恢复（5 稳定帧见线），就重新进入 LINE_SEARCH 主动找线。
 * 覆盖巡线找线/弯道找出口/避障找线的全部超时路径，比赛里不再出现
 * 永久停死；真正的终点 FINISH_STOP 不受影响。 */
#define FAILSAFE_RESTART_MS      2000u

/*
 * 弯道出口起步助推（2026-09-01）：CORNER_SEARCH 停车确认出口后直接回
 * RUN，此时弯道混控(F=6.5,T=±9)给出 A≈2.5%、B≈6.1%、C≈15.5% 的不对称
 * 低速输出，A/B 低于死区无法从静止克服摩擦，车停在出口处需要人工轻推。
 * CORNER_EXIT_LAUNCH 现作为所有“停车确认后恢复巡线”的通用助推状态：
 * 第一阶段纯前进启动 A/C；若确认帧仍有明显横向误差，第二阶段短促带转向
 * 启动 B 并接上弯道，随后才进入 RUN。驱动层仍不做全局低速抬升。
 */
#define CORNER_EXIT_LAUNCH_SPEED_PCT  12.0f
#define CORNER_EXIT_LAUNCH_FORWARD_MS  100u
#define CORNER_EXIT_LAUNCH_TURN_FORWARD_PCT  8.0f
#define CORNER_EXIT_LAUNCH_TURN_PCT     13
#define CORNER_EXIT_LAUNCH_TURN_MS      90u
#define REACQUIRE_TURN_REQUIRED_ERR    0.30f

/* 现场标定（2026-09-01）：触发距离 15→13cm。
 * 必须连续两次有效测距都 <=13cm；任意一次 >13cm 或有效样本间隔过长
 * 都会清零计数，避免两个不相邻的近距离毛刺误触发避障。 */
#define OBSTACLE_TRIGGER_CM       13u
#define OBSTACLE_CONFIRM_SAMPLES   2u
#define OBSTACLE_SAMPLE_GAP_MS    350u

/* ---- 终点 T 线停车参数（2026-08-27 新增）----
 * 避障完成回到巡线后，摄像头连续 FINISH_CONFIRM_SAMPLES 帧看到足够多的
 * 宽黑行即判定到达 T 线横杆并刹车。赛道前半段检测未武装，不会误停。 */
#define FINISH_CONFIRM_SAMPLES     2u

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

/* 电机低速处理在 motor.c：<1% 指令按停止清理噪声，其余按原幅值输出
 * （不做全局抬升——那会把巡线小修角时后轮 B 的微小修正放大，直线画龙）。
 * 弯道出口的起步问题改由 CORNER_EXIT_LAUNCH 状态解决，见下方宏说明。 */

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
#define AVOID_STRAFE_DIST_CM    14.0f   /* 现场标定 13→14：左移总距离 */
#define AVOID_FORWARD_DIST_CM   14.0f   /* 初值：前进越过距离 */

/* 由上面的距离/速度自动换算出各阶段时长（勿手改，改距离或速度即可）。 */
#define AVOID_STRAFE_MS(dist_cm) \
    ((uint32_t)((dist_cm) / STRAFE_SPEED_CM_S * 1000.0f))
#define AVOID_FORWARD_CALC_MS \
    ((uint32_t)(AVOID_FORWARD_DIST_CM / FORWARD_SPEED_CM_S * 1000.0f))

/* ---- 固定时序（不依赖标定）---- */
#define AVOID_STOP_MS            120u   /* 确认障碍后的短暂停车 */

/* ---- 硬编码调试行为（2026-09-01，已启用）----
 * 避障完整走完、回到巡线（AVOID_FIND_LINE 退出）满 2s 后直接刹停并进
 * FINISH_STOP（不自动恢复），用于实车观察避障后的巡线表现。
 * 置 0 关闭；调试完成后删除本段与 walking_update_cam 中的对应检查。 */
#define AVOID_HARD_STOP_ENABLED   1
#define AVOID_HARD_STOP_MS       1000u

/* ---- 找线阶段（AVOID_FIND_LINE）----
 * 绕过木板后低速前进+小幅扫描找回黑线；见线即恢复 RUN。
 * [标定#17] 扫描偏置方向：初值 +1（左偏扫）；若试跑发现扫向错误一侧
 *   （远离 T 干线），翻转成 -1。 */
#define AVOID_FIND_LINE_MS      2500u   /* 找线总超时；超时进 FAILSAFE_STOP */
#define AVOID_FIND_CENTER_ERR   0.35f   /* 目标线进入画面中央后停车确认 */
#define AVOID_FIND_CONFIRM_SAMPLES 2u   /* 首帧停车，静止确认第二帧 */
#define AVOID_STRAFE_SPEED_PCT   12     /* 平移速度档（高于 motor.c 最低有效出力） */
/* 找线右移速度（2026-09-01 10→11）：现场要求提速 1 档；自旋由
 * STRAFE_B_GAIN_FIND 专用增益抑制（速度变化后如自旋方向/强度变化，
 * 对该值做 ±0.05 微调即可） */
#define AVOID_FIND_STRAFE_PCT    11
#define AVOID_FORWARD_SPEED_PCT  15     /* 避障前进速度档 */
#define AVOID_FIND_SPEED_PCT     10     /* 找线前进速度档 */
#define AVOID_SCAN_TURN_PCT       5     /* 找线扫描转向幅度 */
#define AVOID_SCAN_TURN_SIGN    (+1)

static const char *const s_state_name[] = {
    "RUN",
    "LINE_SEARCH",
    "CORNER_SEARCH",
    "CORNER_EXIT_LAUNCH",
    "AVOID_STOP",
    "AVOID_STRAFE_LEFT",
    "AVOID_FORWARD",
    "AVOID_FIND_LINE",
    "FINISH_STOP",
    "FAILSAFE_STOP",
    "BALL_TASK",
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
static bool s_output_enabled;
static float s_last_error;
/* ---- 终点 T 线停车（2026-08-27 新增）----
 * s_finish_armed：只有避障完成回到巡线后才置位，避免赛道前半段的
 * 宽线/路口全黑被误判成终点；s_finish_samples：全黑连续确认计数。 */
static bool s_finish_armed;
static uint8_t s_finish_samples;
static uint32_t s_last_cam_frame_seq;
static uint32_t s_avoid_find_line_start_seq;
/* 避障完成回到巡线的时刻（AVOID_FIND_LINE 退出），供 AVOID_HARD_STOP_MS
 * 硬编码停车计时 */
static uint32_t s_avoid_run_entry_ms;
static bool s_cam_new_frame;
/* 原始新帧（A2 修复）：不要求 stable，仅用于找线/弯道恢复计数。
 * 旋转找线时凑不满"连续 3 帧一致"的稳定条件，若恢复也要求 stable
 * 会永远找不回线（原地转圈自锁）。 */
static bool s_cam_new_raw_frame;
static uint32_t s_last_cam_raw_frame_seq;
/* +1=黑线最后在右侧，应使用正 turn；-1=最后在左侧，应使用负 turn。 */
static int s_last_line_side;
/* 方向投票计数器：正=近期线在右侧占优，负=左侧占优。 */
static int s_line_side_accum;
static int s_corner_turn_side;
static float s_filtered_error;
static bool s_error_filter_valid;
static walking_command_t s_last_command;
/* 搜索候选确认失败后的短促旋转起步，以及通用恢复助推上下文。 */
static bool s_search_restart_kick_active;
static uint32_t s_search_restart_kick_start_ms;
static bool s_corner_saw_no_line;
/* 硬编码：赛道第一个锐角为右转（2026-09-01 现场确认），首次弯道检测
 * 直接锁定右侧；置位后恢复正常的逐帧侧别判定 */
static bool s_first_corner_locked;
static bool s_launch_turn_required;
static int s_launch_turn_side;
/* 把摄像头结果包装成巡线控制器需要的上下文。 */
static camera_line_result_t s_cam;
static camera_line_result_t s_launch_confirmed_cam;
/* 当前生效的平移后轮增益：全局 STRAFE_B_GAIN，仅找线状态临时替换为
 * STRAFE_B_GAIN_FIND（进入新状态时由 enter_state 恢复全局值） */
static float s_strafe_b_gain = STRAFE_B_GAIN;

static int clamp_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float clamp_float(float value, float low, float high)
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
    s_search_restart_kick_active = false;
    if (state == CAR_STATE_CORNER_SEARCH) {
        s_corner_saw_no_line = false;
    }
    s_strafe_b_gain = STRAFE_B_GAIN;   /* 切换状态恢复全局平移增益 */
}

/*
 * 当前底盘三轴混控（forward / turn / strafe）：
 *   纯前进（已实车校正）：L=-v, R=+v, B=0
 *   原地旋转：左右轮同向转向分量、后轮 B 反向分量（0.65 倍补偿）
 *   平移（strafe>0=左）：L/R 按 STRAFE_SIGN、后轮 B 反向（增益待标定）
 * 循迹只用到 forward/turn（strafe=0），输出与旧版完全一致，不影响循迹。
 * 三轮比例超出 100% 时等比缩放，避免逐轮截断改变运动方向。
 */
static void apply_body_command(float forward, float turn, float strafe)
{
    /* 遥测结构体仍是整数：四舍五入显示，PWM 按 float 落地 */
    s_last_command.forward_percent = (int)lroundf(forward);
    s_last_command.turn_percent = (int)lroundf(turn);
    s_last_command.strafe_percent = (int)lroundf(strafe);

    float wheel[MOTOR_NUM] = {0};
    wheel[WHEEL_TO_MOTOR_L] = FORWARD_SIGN_L * forward + TURN_SIGN_L * turn + STRAFE_SIGN_L * strafe;
    wheel[WHEEL_TO_MOTOR_R] = FORWARD_SIGN_R * forward + TURN_SIGN_R * turn + STRAFE_SIGN_R * strafe;
    wheel[WHEEL_TO_MOTOR_B] = TURN_SIGN_B * turn * TURN_B_GAIN
                            + STRAFE_SIGN_B * strafe * s_strafe_b_gain;

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
        const float percent = wheel[i] * s_motor_gain[i] * 100.0f / max_abs;
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

static void reset_line_control_history(void)
{
    s_last_error = 0.0f;
    s_filtered_error = 0.0f;
    s_error_filter_valid = false;
    s_line_lost_timing = false;
    s_last_line_was_corner = false;
}

/* 所有“停车确认后恢复巡线”的路径统一从这里进入助推状态。 */
static void enter_reacquire_launch(const camera_line_result_t *cam,
                                   bool force_turn,
                                   int preferred_turn_side,
                                   uint32_t now_ms)
{
    if (cam != NULL && cam->has_line) {
        s_cam = *cam;
    }
    s_launch_confirmed_cam = s_cam;

    reset_line_control_history();
    enter_state(CAR_STATE_CORNER_EXIT_LAUNCH, now_ms);

    s_launch_turn_required = force_turn ||
                             (s_cam.has_line &&
                              fabsf(s_cam.error) >= REACQUIRE_TURN_REQUIRED_ERR);
    if (preferred_turn_side != 0) {
        s_launch_turn_side = preferred_turn_side < 0 ? -1 : +1;
    } else if (s_cam.has_line && fabsf(s_cam.error) >= CAM_CENTER_DEADBAND) {
        s_launch_turn_side = s_cam.error < 0.0f ? -1 : +1;
    } else {
        s_launch_turn_side = s_last_line_side < 0 ? -1 : +1;
    }

    /* 同一控制周期立即输出，避免状态切换后多空等 20ms。 */
    apply_body_command(CORNER_EXIT_LAUNCH_SPEED_PCT, 0, 0);
}

static void start_search_restart_kick(uint32_t now_ms)
{
    s_search_restart_kick_active = true;
    s_search_restart_kick_start_ms = now_ms;
}

static bool search_restart_kick_active(uint32_t now_ms)
{
    if (!s_search_restart_kick_active) {
        return false;
    }
    if (elapsed_ms(now_ms, s_search_restart_kick_start_ms) >=
        SEARCH_RESTART_KICK_MS) {
        s_search_restart_kick_active = false;
        return false;
    }
    return true;
}

/* 仅测试/标定用：绕过状态机直接驱动三轴（见 walking.h）。 */
void walking_test_drive(int forward, int turn, int strafe)
{
    apply_body_command(forward, turn, strafe);
}

/* 供 ball_task 模块下发三轴混控指令（走正常混控与输出使能）。 */
void walking_direct_command(float forward, float turn, float strafe)
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
 * 找线方向基于未经滤波的原始误差，并对最近若干拍做累计投票：
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

    return (fabsf(s_filtered_error) < CAM_CENTER_DEADBAND) ? 0.0f : s_filtered_error;
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
        /* 短暂测量失败不破坏连续性；但长时间没有有效样本必须清零，
         * 避免一次旧的近距离毛刺永久留在确认计数里。 */
        if (elapsed_ms(now_ms, s_last_distance_sample_ms) > OBSTACLE_SAMPLE_GAP_MS) {
            s_obstacle_samples = 0;
        }
        return false;
    }

    if (elapsed_ms(now_ms, s_last_distance_sample_ms) > OBSTACLE_SAMPLE_GAP_MS) {
        s_obstacle_samples = 0;
    }
    s_last_distance_sample_ms = now_ms;

    if (distance_cm <= OBSTACLE_TRIGGER_CM) {
        if (s_obstacle_samples < OBSTACLE_CONFIRM_SAMPLES) {
            s_obstacle_samples++;
        }
    } else {
        /* 触发前要求真正连续的近距离样本。旧逻辑在 14~18cm 区间既不
         * 累加也不清零，两个相隔很久的 13cm 毛刺也会误触发避障。 */
        s_obstacle_samples = 0;
    }

    return s_obstacle_samples >= OBSTACLE_CONFIRM_SAMPLES;
}

/* ============================================================
 * 任务二：摄像头巡线适配层
 *
 * 摄像头输出的是连续误差 [-1,+1]（黑线质心），没有红外四路的
 * a/b/c/d 图样。为最大限度复用任务一已验证的控制逻辑，这里做
 * 语义映射：
 *   - has_line            ↔ 红外"至少一路见线"
 *   - |error| > CAM_ERR_BIG ↔ 外侧探头命中（接近弯道）
 *   - 全帧大面积黑        ↔ 四路全黑（T 线/宽线）
 * 误差量纲差异由 CAM_ERR_SCALE 折算进 PD 增益。
 * ============================================================ */
#define CAM_ERR_BIG       0.45f   /* |err| 超过此值视为"外侧探头命中"（弯道前兆） */
#define CAM_ERR_SCALE     4.0f   /* 归一化摄像头误差转换为可用的转向量 */
/* 终点横杆必须有至少三条“横向黑色覆盖 >= CAM_WIDE_ROW_PCT%”的采样行。 */
#define CAM_T_LINE_MIN_WIDE_ROWS  3u

static void run_line_controller_cam(uint32_t now_ms);

static bool cam_is_sharp_corner(void)
{
    return fabsf(s_cam.error) > CAM_ERR_BIG;
}

static bool cam_all_black(void)
{
    /* 普通纵向黑线也会穿过很多扫描行，不能仅依赖 black_rows。
     * 终点横杆的区分特征是：连续多行在横向上大面积（>=65%）变黑。 */
    return s_cam.has_line && s_cam.wide_black_rows >= CAM_T_LINE_MIN_WIDE_ROWS;
}

static bool cam_is_finish_marker(const camera_line_result_t *cam)
{
    return cam != NULL && cam->has_line &&
           cam->wide_black_rows >= CAM_T_LINE_MIN_WIDE_ROWS;
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
    s_last_error = 0.0f;
    s_last_line_side = +1;
    s_line_side_accum = 0;
    s_corner_turn_side = +1;
    s_corner_saw_no_line = false;
    s_first_corner_locked = false;
    s_search_restart_kick_active = false;
    s_search_restart_kick_start_ms = now_ms;
    s_launch_turn_required = false;
    s_launch_turn_side = +1;
    s_launch_confirmed_cam = (camera_line_result_t){0};
    s_filtered_error = 0.0f;
    s_error_filter_valid = false;
    s_finish_armed = false;
    s_finish_samples = 0;
    s_last_cam_frame_seq = 0;
    s_last_cam_raw_frame_seq = 0;
    s_avoid_find_line_start_seq = 0;
    s_avoid_run_entry_ms = now_ms;
    ball_task_reset(now_ms);
    s_cam_new_frame = false;
    s_cam_new_raw_frame = false;
    s_cam = (camera_line_result_t){0};
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

void walking_update_cam(const camera_line_result_t *cam,
                        uint16_t distance_cm,
                        uint32_t now_ms)
{
    if (cam == NULL) {
        enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
        apply_brake();
        return;
    }
    /* 主循环为 20ms、摄像头约为 3fps。区分两种"新帧"事件：
     * 稳定新帧（要求 stable）→ 更新控制器视觉量 s_cam，供巡线/T线/侧别投票；
     * 原始新帧（不要求 stable）→ 仅供找线/弯道恢复计数。
     * 两者都按 frame_seq 去重，同一张旧图不会被重复消费。 */
    s_cam_new_raw_frame = cam->frame_seq != s_last_cam_raw_frame_seq;
    s_cam_new_frame = cam->stable && s_cam_new_raw_frame;
    if (s_cam_new_raw_frame) {
        s_last_cam_raw_frame_seq = cam->frame_seq;
    }
    if (s_cam_new_frame) {
        s_cam = *cam;
        s_last_cam_frame_seq = cam->frame_seq;
    }

    if (state_accepts_obstacle_trigger() &&
        obstacle_confirmed(distance_cm, now_ms)) {
        s_obstacle_samples = 0;
        enter_state(CAR_STATE_AVOID_STOP, now_ms);
    }

#if AVOID_HARD_STOP_ENABLED
    /* 硬编码衔接点：避障回到巡线满 1s 停下，随后开始找球/推球任务
     * （见 AVOID_HARD_STOP_MS 与 ball_task 模块说明）。 */
    if (s_state == CAR_STATE_RUN && s_finish_armed &&
        elapsed_ms(now_ms, s_avoid_run_entry_ms) >= AVOID_HARD_STOP_MS) {
        ball_task_start(now_ms);
        enter_state(CAR_STATE_BALL_TASK, now_ms);
        apply_brake();
    }
#endif

    bool finish_candidate_cleared = false;

    switch (s_state) {
    case CAR_STATE_RUN:
        /*
         * 终点 T 线检测（仅在避障完成后武装）：使用原始新帧，第一帧
         * 宽黑候选立即停车，静止确认第二帧。旧逻辑边以 8% 前进边等
         * 3 个稳定帧，横杆较窄时会在确认完成前驶过终点。
         */
        if (s_finish_armed) {
            if (s_cam_new_raw_frame) {
                if (cam_is_finish_marker(cam)) {
                    if (s_finish_samples < FINISH_CONFIRM_SAMPLES) {
                        s_finish_samples++;
                    }
                } else {
                    finish_candidate_cleared = s_finish_samples > 0u;
                    s_finish_samples = 0;
                }
            }

            if (s_finish_samples >= FINISH_CONFIRM_SAMPLES) {
                enter_state(CAR_STATE_FINISH_STOP, now_ms);
                apply_brake();
                break;
            }
            if (s_finish_samples > 0u) {
                apply_brake();  /* 首帧候选后保持静止，等待下一张原始帧 */
                break;
            }
            if (finish_candidate_cleared && cam->has_line) {
                /* 假终点候选曾让整车刹停；恢复巡线也要经过通用起步助推。 */
                enter_reacquire_launch(cam, false, 0, now_ms);
                break;
            }
        }

        run_line_controller_cam(now_ms);   /* 内部改用 s_cam */
        break;

    case CAR_STATE_LINE_SEARCH: {
        /*
         * 恢复判定用原始新帧（A2 修复）：连续 2 帧见线即恢复，不要求
         * stable——旋转找线中凑不满稳定条件，按 stable 恢复会永远转圈。
         * 恢复时用最新原始帧刷新 s_cam，否则 s_cam 仍停留在丢线旧数据，
         * 回到 RUN 后会立刻再判丢线、弹回本状态。
         *
         * 首帧见线立即停车（2026-09-01）：旧逻辑边转边数 2 帧，转速稍快
         * 线就滑出扫描带、计数清零，表现为"明明扫到还继续原地转"。现在
         * 首帧见线即静止，画面不动，下一帧确认几乎必然命中；确认失败
         * （单帧误检）则清零继续扫描。
         */
        if (s_cam_new_raw_frame) {
            if (cam->has_line) {
                if (++s_line_reacquire_samples >= 2) {
                    enter_reacquire_launch(cam, false, 0, now_ms);
                    break;
                }
                /* 首帧见线：落到下方静止等待确认帧，不再继续转 */
            } else {
                if (s_line_reacquire_samples > 0u) {
                    start_search_restart_kick(now_ms);
                }
                s_line_reacquire_samples = 0;
            }
        }

#if LINE_SEARCH_TIMEOUT_MS > 0u
        if (elapsed_ms(now_ms, s_state_enter_ms) >= LINE_SEARCH_TIMEOUT_MS) {
            /* 普通找线超时不再刹停 2 秒；翻转优先侧并立即开始新一轮扫描。 */
            s_last_line_side = -s_last_line_side;
            enter_state(CAR_STATE_LINE_SEARCH, now_ms);
            start_search_restart_kick(now_ms);
            apply_body_command(0,
                               s_last_line_side * STEERING_SIGN *
                                   SEARCH_RESTART_KICK_TURN_PCT,
                               0);
            break;
        }
#endif

        if (s_line_reacquire_samples > 0u) {
            apply_body_command(0, 0, 0);   /* 见线后静止，等下一帧确认 */
        } else {
            /* 周期性换向扫描（A3 修复）：先朝上次见线侧转，再反向交替，
             * 方向猜错时也能自行摆回来。 */
            const int sweep_sign =
                ((elapsed_ms(now_ms, s_state_enter_ms) / LINE_SEARCH_SWEEP_MS) & 1u) ? -1 : +1;
            const int turn_pct = search_restart_kick_active(now_ms)
                                     ? SEARCH_RESTART_KICK_TURN_PCT
                                     : LINE_SEARCH_TURN_PCT;
            apply_body_command(0,
                               s_last_line_side * sweep_sign * STEERING_SIGN * turn_pct,
                               0);
        }
        break;
    }

    case CAR_STATE_CORNER_SEARCH: {
        /* 出口判定用最新原始帧（A2 修复）：旋转中 s_cam 只随稳定帧更新，
         * 会长期停留在旧数据上；cam 为最近一帧结果，按帧到来时才计数。 */
        const bool has_line = cam->has_line;
        if (s_cam_new_raw_frame && !has_line) {
            s_corner_saw_no_line = true;
        }

        /*
         * 出口判定（2026-09-01 重构）：见线且线出现在当前转向一侧、幅值
         * 达到现场标定阈值 LINE_CORNER_EXIT_MIN_ERR（含义见参数区说明）。
         * 原条件还要求同侧且 |err|>0.225，而旋转中出口线从扫描带边缘
         * 切入（误差接近满偏），顶点宽黑区质心又常停在中央附近。方向
         * 同侧门控同时天然排除入口旧线段：车头朝锁定侧旋转时，它只会
         * 向转向反侧滑出视野。
         * 不再定时反向：单向持续旋转下方向猜错也能扫到出口（见参数区
         * 说明），到点反向反而会退掉已转过的角度。
         */
        const bool same_side_exit =
            (s_corner_turn_side < 0) ? (cam->error < -LINE_CORNER_EXIT_MIN_ERR)
                                     : (cam->error >  LINE_CORNER_EXIT_MIN_ERR);
        const bool centered_exit = s_corner_saw_no_line &&
                                   fabsf(cam->error) <= LINE_CORNER_CENTER_EXIT_ERR;
        const bool exit_candidate = has_line && (same_side_exit || centered_exit);

        if (exit_candidate) {
            if (s_cam_new_raw_frame && s_corner_exit_samples < LINE_CORNER_EXIT_CONFIRM_SAMPLES) {
                s_corner_exit_samples++;
            }

            if (s_corner_exit_samples >= LINE_CORNER_EXIT_CONFIRM_SAMPLES) {
                /* 锐角出口恢复必须带第二阶段转向助推，启动此前静止的 B 轮。 */
                enter_reacquire_launch(cam, true, s_corner_turn_side, now_ms);
                break;
            }

            apply_body_command(0, 0, 0);   /* 候选帧立即停车，静止画面下确认最可靠 */
        } else {
            if (s_cam_new_raw_frame) {
                if (s_corner_exit_samples > 0u) {
                    start_search_restart_kick(now_ms);
                }
                s_corner_exit_samples = 0;
            }
            /* 按锁定方向持续慢转，直到见线确认或总超时 */
            const int turn_pct = search_restart_kick_active(now_ms)
                                     ? SEARCH_RESTART_KICK_TURN_PCT
                                     : LINE_SEARCH_TURN_PCT;
            apply_body_command(0,
                               s_corner_turn_side * STEERING_SIGN * turn_pct,
                               0);
        }

        if (elapsed_ms(now_ms, s_state_enter_ms) >= LINE_CORNER_TIMEOUT_MS) {
            /* 锁定方向未找到出口时，直接切换到反侧优先的双向扫描。 */
            s_last_line_side = -s_corner_turn_side;
            enter_state(CAR_STATE_LINE_SEARCH, now_ms);
            start_search_restart_kick(now_ms);
            apply_body_command(0,
                               s_last_line_side * STEERING_SIGN *
                                   SEARCH_RESTART_KICK_TURN_PCT,
                               0);
        }
        break;
    }

    case CAR_STATE_CORNER_EXIT_LAUNCH:
        /*
         * 通用恢复助推：先纯前进启动 A/C；若黑线仍明显偏侧，再用短促
         * 带方向助推启动 B。只采用新的有效见线帧，避免瞬时丢线覆盖
         * 进入本状态前已经确认过的 s_cam。
         */
        if (elapsed_ms(now_ms, s_state_enter_ms) <
            CORNER_EXIT_LAUNCH_FORWARD_MS) {
            apply_body_command(CORNER_EXIT_LAUNCH_SPEED_PCT, 0, 0);
            break;
        }

        if (s_launch_turn_required &&
            elapsed_ms(now_ms, s_state_enter_ms) <
                CORNER_EXIT_LAUNCH_FORWARD_MS + CORNER_EXIT_LAUNCH_TURN_MS) {
            apply_body_command(CORNER_EXIT_LAUNCH_TURN_FORWARD_PCT,
                               s_launch_turn_side * STEERING_SIGN *
                                   CORNER_EXIT_LAUNCH_TURN_PCT,
                               0);
            break;
        }

        if (s_cam_new_raw_frame && cam->has_line) {
            s_cam = *cam;
        } else if (s_launch_confirmed_cam.has_line) {
            s_cam = s_launch_confirmed_cam;
        }
        reset_line_control_history();
        enter_state(CAR_STATE_RUN, now_ms);
        run_line_controller_cam(now_ms);
        break;

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
            /* 找线阶段按原始帧判断，起始序号也必须使用原始帧序号。 */
            s_avoid_find_line_start_seq = s_last_cam_raw_frame_seq;
            enter_state(CAR_STATE_AVOID_FIND_LINE, now_ms);
        }
        break;

    case CAR_STATE_AVOID_FIND_LINE:
        /*
         * 绕过木板后向【右】平移找回黑线（2026-08-27 按实车联调修改：
         * 原方案"前进+转向扫描"实测表现为向右转，不符合赛道需求）。
         * 平移方向由 STRAFE_SIGN_* 标定保证 strafe<0=向右。目标线进入
         * 画面中央范围后，第一张原始帧立即停车，静止确认第二帧再恢复
         * RUN；避免边横移边等待 stable 的 600~900ms 延迟导致越过黑线。
         * 中央门控同时排除仍留在画面边缘的入口旧线段。
         */
        if (s_cam_new_raw_frame && cam->frame_seq > s_avoid_find_line_start_seq) {
            const bool centered_line = cam->has_line &&
                                       fabsf(cam->error) <= AVOID_FIND_CENTER_ERR;
            if (centered_line) {
                if (s_line_reacquire_samples < AVOID_FIND_CONFIRM_SAMPLES) {
                    s_line_reacquire_samples++;
                }
            } else {
                s_line_reacquire_samples = 0;
            }
        }

        if (s_line_reacquire_samples >= AVOID_FIND_CONFIRM_SAMPLES) {
            s_finish_armed = true;   /* 武装终点 T 线检测 */
            s_finish_samples = 0;
            s_avoid_run_entry_ms = now_ms;   /* 回巡线时刻（硬编码停车计时起点） */
            enter_reacquire_launch(cam, false, 0, now_ms);
            break;
        }
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_FIND_LINE_MS) {
            enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
            apply_brake();
            break;
        }
        if (s_line_reacquire_samples > 0u) {
            apply_brake();  /* 首帧进入中央范围，静止等第二帧确认 */
        } else {
            /* strafe<0 = 向右平移；套用找线专用后轮增益，抵消低速下
             * 前轮死区压缩带来的失配自旋（调参见 STRAFE_B_GAIN_FIND）。 */
            s_strafe_b_gain = STRAFE_B_GAIN_FIND;
            apply_body_command(0, 0, -AVOID_FIND_STRAFE_PCT);
        }
        break;

    case CAR_STATE_FINISH_STOP:
        /*
         * 到达赛道末尾 T 线：永久刹车，任务结束。
         * 与 FAILSAFE_STOP 不同，本状态不自动恢复——终点停车是预期行为。
         * （找球任务完成后也进本状态收尾。）
         */
        apply_brake();
        break;

    case CAR_STATE_BALL_TASK: {
        /*
         * 找球/推球任务（接在避障→巡线→1s 停车之后）：由 ball_task
         * 状态机接管电机（经 walking_direct_command 走正常三轴混控）。
         * 双球推送完成或全任务超时后收尾进 FINISH_STOP。
         */
        ball_field_result_t balls;
        camera_line_get_ball(&balls);
        ball_task_update(cam, &balls, now_ms);
        if (ball_task_is_done()) {
            enter_state(CAR_STATE_FINISH_STOP, now_ms);
            apply_brake();
        }
        break;
    }

    case CAR_STATE_FAILSAFE_STOP:
    default:
        apply_brake();
        if (s_cam_new_frame && s_cam.has_line) {
            if (++s_line_reacquire_samples >= 5) {
                enter_reacquire_launch(&s_cam, false, 0, now_ms);
            }
        } else if (s_cam_new_frame) {
            s_line_reacquire_samples = 0;
        }

        /*
         * 自动重启（FAILSAFE_RESTART_MS）：刹停静置满时长仍未恢复，
         * 重新主动找线——车头背对线时被动恢复永远凑不齐，主动扫描
         * 是唯一出路。找到线后经 LINE_SEARCH 的确认流程回 RUN。
         */
        if (elapsed_ms(now_ms, s_state_enter_ms) >= FAILSAFE_RESTART_MS) {
            s_line_lost_timing = false;
            enter_state(CAR_STATE_LINE_SEARCH, now_ms);
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

/*
 * 任务二巡线控制器（摄像头版）：替代任务一的 run_line_controller。
 * 输入：s_cam（黑线质心误差 [-1,+1]，左负右正）
 * 逻辑与任务一一致：丢线宽限→原地找线；弯道大误差→固定大转向；
 * 正常→PD 巡线（CAM_ERR_SCALE 折算量纲）。
 */
static void run_line_controller_cam(uint32_t now_ms)
{
    const bool has_line = s_cam.has_line;
    float error = has_line ? s_cam.error : 0.0f;

    if (!has_line) {
        if (!s_line_lost_timing) {
            s_line_lost_timing = true;
            s_line_lost_since_ms = now_ms;
        }

        /*
         * 弯道处丢线（此前 |err| 大）：立刻停止前进，按最后确认的
         * 黑线侧原地转，等内侧重新见线。
         */
        if (s_last_line_was_corner) {
            s_corner_turn_side = s_last_line_side;
            s_corner_exit_samples = 0;
            enter_state(CAR_STATE_CORNER_SEARCH, now_ms);
            apply_body_command(0,
                               s_corner_turn_side * STEERING_SIGN * LINE_SEARCH_TURN_PCT, 0);
            return;
        }

        if (elapsed_ms(now_ms, s_line_lost_since_ms) < LINE_LOST_GRACE_MS) {
            const int held_turn = clamp_int(
                (int)lroundf(STEERING_SIGN * LINE_KP * CAM_ERR_SCALE * s_last_error),
                -LINE_MAX_TURN_PCT, LINE_MAX_TURN_PCT);
            apply_body_command(LINE_LOST_FORWARD_PCT, limit_line_turn_slew(held_turn), 0);
        } else {
            enter_state(CAR_STATE_LINE_SEARCH, now_ms);
        }
        return;
    }

    s_line_lost_timing = false;
    if (s_cam_new_frame) {
        remember_last_line_side(error);
    }

    /* T 线/宽黑区：低速直行（终点判定在 walking_update_cam 的 RUN 分支做） */
    if (cam_all_black()) {
        s_last_error = 0.0f;
        s_filtered_error = 0.0f;
        s_error_filter_valid = true;
        s_last_line_was_corner = false;
        apply_body_command(LINE_MIN_SPEED_PCT, 0, 0);
        return;
    }

    /* 大误差=接近弯道边缘：锁定方向并减速大转向 */
    if (cam_is_sharp_corner()) {
        /* 硬编码（2026-09-01 现场确认）：赛道第一个锐角为【右】转，首次
         * 进入弯道检测直接锁定右侧，防侧别误锁原地反绕。 */
        if (!s_first_corner_locked) {
            s_first_corner_locked = true;
            s_last_line_side = +1;
        } else {
            s_last_line_side = (error < 0.0f) ? -1 : +1;
        }
        s_line_side_accum = s_last_line_side * LINE_SIDE_FLIP_MIN;
        s_last_line_was_corner = true;

        s_last_error = error;
        s_filtered_error = error;
        s_error_filter_valid = true;
        const int corner_turn = (error < 0.0f ? -1 : +1) *
                                STEERING_SIGN * LINE_CORNER_TURN_PCT;
        apply_body_command(LINE_CORNER_SPEED_PCT, limit_line_turn_slew(corner_turn), 0);
        return;
    }

    s_last_line_was_corner = false;

    const float filtered_error = filter_line_error(error);
    const float derivative = filtered_error - s_last_error;
    s_last_error = filtered_error;
    int turn = (int)lroundf(STEERING_SIGN *
                            (LINE_KP * CAM_ERR_SCALE *
                             (filtered_error + LINE_KD / LINE_KP * derivative)));
    turn = clamp_int(turn, -LINE_MAX_TURN_PCT, LINE_MAX_TURN_PCT);
    turn = limit_line_turn_slew(turn);

    const float forward = clamp_float(LINE_BASE_SPEED_PCT - abs(turn) / 2.0f,
                                      LINE_MIN_SPEED_PCT, LINE_BASE_SPEED_PCT);
    apply_body_command(forward, turn, 0);
}
