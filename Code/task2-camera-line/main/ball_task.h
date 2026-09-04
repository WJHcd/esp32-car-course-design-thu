#pragma once
#include <stdint.h>
#include "camera_line.h"

/* ============================================================
 * 找球/推球任务状态机（科目三，接在"避障→巡线→1s 停车"之后）
 *
 * 任务：场地内有两个球（红、蓝，2026-09-03 原黑球换成红球），把它们
 * 分别推到场地末端的两个绿色角标处（角标识别见 ball_detect.h）。
 * 云台抬到扫视角后【全程固定】，不再远近切换。
 *
 * v3 三点一线法（2026-09-03 重构；同日晚间去掉 APPROACH，对准吸收接近）：
 *   SURVEY 静止等球（人工把球摆到镜头前；BALL_SURVEY_SPIN_ENABLE=1
 *   可恢复转圈搜索）→ ALIGN 利用车-球-角标三点共线的图像几何：
 *   【共线 ⇔ 球与角标在画面中 x 重合】，且该差值 e=x球−x角标 对纯
 *   旋转不变、只对平移收敛——故用全向底盘【横移】消差，再用原地踢转
 *   把已对齐的目标对居中（旋转不破坏共线）；角标不可见时不连续直奔，
 *   改为【脉冲式前进】接近球（走一步停一步看帧），球到近处仍无角标
 *   则直推兜底 → PUSH 直线撞球，球自行滚向角标；同帧贴近 + 静止两帧
 *   确认到位 → RETURN 按推进时长盲退回近似原位 → SURVEY 推下一球。
 *
 * 流程：GIMBAL_UP → SURVEY → ALIGN → PUSH → RETURN → …
 *
 * 电机指令经 walking_direct_command() 下发（走正常三轴混控）。
 * ============================================================ */

typedef enum {
    BALL_TASK_GIMBAL_UP = 0,   /* 抬云台并静置 */
    BALL_TASK_SURVEY,          /* 静止等摆球（或转圈搜索，见 SPIN_ENABLE） */
    BALL_TASK_ALIGN,           /* 对准：平移+旋转共线；无角标则脉冲前进 */
    BALL_TASK_PUSH,            /* 直线撞球推进 */
    BALL_TASK_RETURN,          /* 按推进时长盲退回近似原位 */
    BALL_TASK_DONE,            /* 双球完成（或超时兜底） */
} ball_task_state_t;

/* ---- 现场标定参数 ---- */
#define BALL_TASK_TILT_SURVEY_DEG   35     /* 云台仰角（全程恒定，抬起后不动） */
#define BALL_GIMBAL_DWELL_MS        600u   /* 抬云台后的静置时长 */

/* ---- SURVEY：静止等摆球 / 转圈搜索 ----
 * 当前测试流程人工摆球，默认不转圈（SPIN_ENABLE=0）；正式赛道需要
 * 自主搜索时置 1，恢复"短转-静止"间歇扫视（BURST/HOLD/TURN_PCT）。 */
#define BALL_SURVEY_SPIN_ENABLE     0
#define BALL_SURVEY_CONFIRM         2      /* 每个球需确认的帧数 */
#define BALL_SURVEY_TIMEOUT_MS      20000u /* 等待超时（静止等摆球给足时间） */
/* 静止等球模式下（SPIN=0）：开局静置一小段让人工摆球确认；仍找不到
 * 待推球就缓慢向前蠕进（画面里没有球时边走边找），直到确认到球。 */
#define BALL_SURVEY_CREEP_DELAY_MS  2000u  /* 静置时长，超过即开始蠕进 */
#define BALL_SURVEY_CREEP_PCT       12     /* 蠕进速度：11% 时 C 轮（×0.90 全局增益）
                                            * 仅 9.9% 贴死区上沿（反馈文档分析），
                                            * 提到 12 让 C 轮 10.8%；9% 是物理下限 */
#define BALL_SURVEY_TURN_PCT       12      /* 搜索短转转速（SPIN=1 时） */
#define BALL_SURVEY_BURST_MS       250u    /* 短转时长 */
#define BALL_SURVEY_HOLD_MS        450u    /* 静止看帧时长 */

/* ---- ALIGN：对准（含无角标时的脉冲前进接近）----
 * 有角标：平移消差 + 踢转居中，共线即进 PUSH（从远处直接推，推进中
 * 持续微修）。无角标：脉冲式前进接近球（走一步停一步看帧），球到
 * 近处（y≥BALL_NEAR_Y_TARGET）仍无角标则直推兜底。 */
#define BALL_ALIGN_FWD_PCT         13      /* 脉冲前进速度 */
#define BALL_ALIGN_FWD_MS          180u    /* 每个前进脉冲时长（走一步停一步） */
/* 球"已近"判定（原 APPROACH 到位线；车体遮挡底部 30%，可见区 y≤0.70）：
 * 无角标脉冲前进到此刻转直推；PUSH 丢球倒车重捕获也按此分流。 */
#define BALL_NEAR_Y_TARGET         0.65f   /* 球心 y ≥ 此值 = 球已到车头近前 */
#define BALL_NEAR_Y_ARRIVE_LOST    0.55f   /* 球出画时最后 y ≥ 此值 = 视为已近 */
#define BALL_ALIGN_DEADBAND        0.06f   /* |e| 死区：以内视为三点共线 */
#define BALL_ALIGN_STRAFE_PCT      12      /* 对准横移速度 */
#define BALL_ALIGN_STRAFE_SIGN     (+1)
/* 横移采用"移一脚-看一帧"（2026-09-03 实车：帧间连续横移步距过大、
 * 收敛线附近过冲）：每帧只按 |e| 比例移一小段，停住等下一帧再评估。 */
#define BALL_ALIGN_STRAFE_SCALE_MS 500u    /* 脉冲时长 ≈ |e| × 此值 */
#define BALL_ALIGN_STRAFE_MIN_MS   80u     /* 最小脉冲 */
#define BALL_ALIGN_STRAFE_MAX_MS   250u    /* 最大脉冲（帧率约 3fps，留看帧余量） */
#define BALL_ALIGN_CENTER_BAND     0.08f   /* 目标对居中死区（踢转消除） */
#define BALL_ALIGN_EDGE_X          0.12f   /* 目标距画面边 <此值：先居中再横移 */
#define BALL_ALIGN_KICK_TURN_PCT   15      /* 踢转转速：15×0.68=10.2%（B 轮）明显
                                            * 越过 9% 死区下限；14 及以下 B 轮
                                            * 起不了转，实车只响不动 */
#define BALL_ALIGN_KICK_SCALE_MS   380u    /* 踢转时长 ≈ |偏差| × 此值 */
#define BALL_ALIGN_KICK_MIN_MS     100u    /* 最小踢转：25ms 电流刚建立脉冲就结束，
                                            * 克服不了静摩擦（2026-09-03 反馈文档） */
#define BALL_ALIGN_KICK_MAX_MS     240u    /* 最大踢转 */
#define BALL_ALIGN_TIMEOUT_MS      20000u  /* 对准总超时（回 SURVEY；含远处对准
                                            * 与脉冲前进接近，比纯对准耗时） */
#define BALL_ALIGN_LOST_WAIT_MS    2000u   /* 对准中球不可见的等待上限（防对车
                                            * 底下的球干等；到点回 SURVEY） */

/* ---- PUSH：直线撞球 ---- */
#define BALL_MIN_EFFECTIVE_PCT     10.0f   /* 前轮有效出力下限（死区 9% + 余量）。
                                            * 球任务内差速限幅用：turn 过大时
                                            * 较慢一侧前轮会跌进死区"别住不走"，
                                            * 故限 |turn| ≤ speed - 此值 */
#define BALL_REALIGN_BACKUP_MS     400u    /* 撞球后球贴车头出画：先倒车一小段
                                            * 把球倒回视野再重新衔接（直接对准
                                            * 会对着车底干等，是"卡住"主因） */
#define BALL_CTRL_PERIOD_MS         20u    /* ball_task_update 调用周期 */
#define BALL_PUSH_SPEED_PCT         12     /* 巡航推速（2026-09-03 实车 13 过快→12，
                                            * C 轮 10.8%；电池电压低时会更无力） */
#define BALL_PUSH_LAUNCH_PCT        22     /* 起步强推档（防死区闷熄；20 无力→22） */
#define BALL_PUSH_LAUNCH_MS         350u   /* 强推时长 */
#define BALL_PUSH_RAMP_MS           600u   /* 强推→巡航线性缓降时长 */
#define BALL_PUSH_STEER_GAIN        10.0f  /* 转向修正：x 偏差 → 转向 %
                                            * （2026-09-03 实车 8 转弯无力→10） */
#define BALL_PUSH_ALIGN_STRAFE_PCT  8      /* 推进中 e 偏差的横移微修力度 */
#define BALL_PUSH_BLIND_MS          300u   /* 球滑出车头视野后的盲走时长
                                            * （800→500→300：实车推球距离过远，
                                            * 撞击后让球自己滚，车少跟进） */
#define BALL_PUSH_MAX_DRIVE_MS      5000u  /* 单球"实际推进指令"累计上限
                                            * （2026-09-04 新增：略微调小推球距离
                                            * 的兜底——到顶按完成本次尝试处理，
                                            * 原来只靠 15s 墙钟超时，推得过远） */
#define BALL_PUSH_TIMEOUT_MS        15000u /* 单球推球总超时 */
#define BALL_TASK_DONE_CONFIRM_WAIT_MS 2000u /* 到位待确认超时（防摄像头卡死） */
#define BALL_DONE_MIN_DRIVE_MS     800u   /* 到位判定前置条件：PUSH 已实际下发
                                            * 推进 ≥ 此时长，防透视/框抖导致
                                            * 刚进 PUSH 就"贴近"误判完成 */
#define BALL_DONE_DIST_EPS         0.10f  /* 静止确认两帧间允许的距离增量。
                                            * 2026-09-03 0.02→0.10：球被撞后是
                                            * 滚动的，滚过角标瞬间距离增大，
                                            * 旧守卫会漏判完成 → 车一路推过头 */
#define BALL_MARKER_ADJACENT_ERR_X  0.15f  /* 球心与角标水平贴近 = 到位 */
#define BALL_MARKER_ADJACENT_ERR_Y  0.15f  /* 纵向（深度）贴近；同帧双可见才判 */

/* ---- 角标锁定（2026-09-03 反馈文档 P1）----
 * 双角标同视野时若每帧重选"离球最近"，检测抖动会让目标来回切换、
 * e 跳变变号。进入 ALIGN 后锁定首个可见角标（按方位匹配跟踪），
 * 连续 BALL_MARKER_HOLD_LOST_FRAMES 帧不可见才允许重新选择。 */
#define BALL_MARKER_MATCH_TOL      0.15f  /* 锁定角标的方位匹配容差 */
#define BALL_MARKER_HOLD_LOST_FRAMES 6    /* 锁定角标连续丢失帧数上限（≈2s） */

/* ---- RETURN：退回原位 ----
 * 编码器里程计退回（2026-09-04）：PUSH 进入时里程清零，期间双前轮
 * （A/C）累计脉冲即实际推进里程；RETURN 倒车到"累计脉冲 ≥ 推进里程
 * ×系数"为止——轮子卡死/死区空转不计里程，会自动多倒补偿，比开环
 * 计时准。计时仍保留为上限兜底（编码器失效时退回纯计时逻辑）。
 * 注意：里程从 PUSH 入口起算，退回到的是"对准完成、开始撞球"的位置。 */
#define BALL_RETURN_SPEED_PCT       14     /* 盲退速度（2026-09-03 实车 11 倒不动
                                            * ——C 轮 0.90 增益下仅 9.9% 贴死区，
                                            * 与前进不起步同机制；14 → C 轮 12.6%） */
#define BALL_RETURN_PULSE_SCALE     1.0f   /* 退回里程 = 推进脉冲 × 此系数 */
#define BALL_RETURN_TIME_SCALE      1.1f   /* 计时上限 = 推进时长 × 此系数 */
#define BALL_RETURN_MAX_MS          8000u  /* 盲退上限（推进时长异常兜底） */

#define BALL_TASK_TOTAL_MS          120000u/* 全任务兜底超时 */

/**
 * @brief 启动找球任务（抬起云台并进入 GIMBAL_UP）
 */
void ball_task_start(uint32_t now_ms);

/**
 * @brief 每控制周期调用（20ms）；内部按新帧处理识别结果并下发电机指令
 * @param cam   最近巡线结果（仅用 frame_seq 判新帧）
 * @param balls 最近一帧三色识别结果（camera_line_get_ball 的输出）
 */
void ball_task_update(const camera_line_result_t *cam,
                      const ball_field_result_t *balls, uint32_t now_ms);

/**
 * @brief 双球任务是否已结束（含超时兜底），walking 据此进 FINISH_STOP
 */
bool ball_task_is_done(void);

/** @brief walking_reset 联动复位 */
void ball_task_reset(uint32_t now_ms);

/** @brief 当前状态名（串口显示用） */
const char *ball_task_state_name(void);
