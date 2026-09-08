#pragma once
#include <stdint.h>
#include "camera_line.h"

/* ============================================================
 * 找球/推球任务状态机（科目三，接在"避障→巡线→1s 停车"之后）
 *
 * 任务：场地内有两个球（红、蓝，2026-09-03 原黑球换成红球），把它们
 * 分别推到场地末端的两个黑色终点块（识别见 ball_detect.h）。
 * 云台抬到扫视角后【全程固定】，不再远近切换。
 *
 * v3 三点一线法（2026-09-03 重构；同日晚间去掉 APPROACH，对准吸收接近）：
 *   SURVEY 静止等球（人工把球摆到镜头前；BALL_SURVEY_SPIN_ENABLE=1
 *   可恢复转圈搜索）→ ALIGN 利用车-球-角标三点共线的图像几何：
 *   【共线 ⇔ 球与角标在画面中 x 重合】，且该差值 e=x球−x角标 对纯
 *   旋转不变、只对平移收敛——故用全向底盘【横移】消差，再用原地踢转
 *   把目标对居中。只有球和黑色终点同帧稳定可见才允许移动；所有短动作
 *   都完整经历“动作→刹停→等待新帧”，杜绝用旧画面连续下令。
 *   PUSH 也分段推进并逐帧复核；同帧贴近 + 停车后的下一帧仍贴近才算
 *   到位 → RETURN 退回近似原位 → SURVEY 推下一球。限距和超时只
 *   进入 ABORT，不再冒充任务成功。
 *
 * 流程：GIMBAL_UP → SURVEY → ALIGN → PUSH → RETURN → …
 *
 * 电机指令经 walking_direct_command() 下发（走正常三轴混控）。
 * ============================================================ */

typedef enum {
    BALL_TASK_GIMBAL_UP = 0,   /* 抬云台并静置 */
    BALL_TASK_SURVEY,          /* 静止等摆球（或转圈搜索，见 SPIN_ENABLE） */
    BALL_TASK_ALIGN,           /* 对准：球+终点同帧可见后平移/旋转共线 */
    BALL_TASK_PUSH,            /* 直线撞球推进 */
    BALL_TASK_RETURN,          /* 按推进时长盲退回近似原位 */
    BALL_TASK_DONE,            /* 仅视觉确认双球完成 */
    BALL_TASK_ABORT,           /* 超时/失联/限距等安全中止，不冒充成功 */
} ball_task_state_t;

/* ---- 现场标定参数 ---- */
#define BALL_TASK_TILT_SURVEY_DEG   35     /* 云台仰角（全程恒定，抬起后不动） */
#define BALL_GIMBAL_DWELL_MS        600u   /* 抬云台后的静置时长 */

/* ---- SURVEY：静止等摆球 / 转圈搜索 ----
 * 当前测试流程人工摆球，默认不转圈（SPIN_ENABLE=0）；正式赛道需要
 * 自主搜索时置 1，恢复"短转-静止"间歇扫视（BURST/HOLD/TURN_PCT）。 */
#define BALL_SURVEY_SPIN_ENABLE     0
#define BALL_SURVEY_CREEP_ENABLE    0      /* 安全测试默认禁用无目标自动蠕进 */
#define BALL_SURVEY_CONFIRM         3      /* 连续且位置相近的不同帧确认 */
#define BALL_SURVEY_TRACK_JUMP_MAX  0.15f  /* 相邻确认帧质心最大跳变量 */
#define BALL_SURVEY_TIMEOUT_MS      20000u /* 等待超时（静止等摆球给足时间） */
/* 仅在显式启用 CREEP 后生效；默认静止测试不会无目标前行。 */
#define BALL_SURVEY_CREEP_DELAY_MS  2000u  /* 静置时长，超过即开始蠕进 */
#define BALL_SURVEY_CREEP_PCT       12     /* 蠕进速度：11% 时 C 轮（×0.90 全局增益）
                                            * 仅 9.9% 贴死区上沿（反馈文档分析），
                                            * 提到 12 让 C 轮 10.8%；9% 是物理下限 */
#define BALL_SURVEY_TURN_PCT       12      /* 搜索短转转速（SPIN=1 时） */
#define BALL_SURVEY_BURST_MS       250u    /* 短转时长 */
#define BALL_SURVEY_HOLD_MS        450u    /* 静止看帧时长 */

/* ---- ALIGN：球和黑色终点同帧稳定可见后，对准并逐动作看帧 ---- */
/* 球"已近"判定（车体遮挡底部 30%，可见区 y≤0.70）：仅用于球丢失
 * 后选择一次短倒车重捕获，不能作为无终点直推的依据。 */
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
#define BALL_ALIGN_KICK_TURN_PCT   16      /* 踢转转速：16×0.68=10.9%（B 轮）。
                                            * 2026-09-04 用户要求加力 15→16 */
#define BALL_ALIGN_KICK_SCALE_MS   260u    /* 踢转时长 ≈ |偏差| × 此值 */
#define BALL_ALIGN_KICK_MIN_MS     100u    /* 最小踢转：25ms 电流刚建立脉冲就结束，
                                            * 克服不了静摩擦（2026-09-03 反馈文档） */
#define BALL_ALIGN_KICK_MAX_MS     160u    /* 最大踢转，动作后必须停车复拍 */
#define BALL_ACTION_SETTLE_MS      100u    /* 每个短动作后先刹停，再等新帧 */
#define BALL_ACTION_WAIT_MAX_MS    600u    /* 等新帧上限：相机卡死时强制交还
                                            * 决策权刹停，防 keep_drive 盲走 */
#define BALL_CAMERA_STALE_MS       500u    /* 超过此时长无新视觉结果立即停车 */
#define BALL_LOST_CONFIRM_FRAMES     4     /* 连续不同帧丢球才进入恢复 */
/* 丢球恢复（2026-09-04 用户定稿）：先连续后退 1.5s（后退中见球即恢复
 * 推球），退完仍无球 → 顺时针脉冲自转找球（转 250ms、隔 1.5s）。 */
#define BALL_ALIGN_BACKUP_MS       1200u   /* 丢球后连续后退时长（2026-09-04
                                            * 用户要求 1.5s→1.2s） */
#define BALL_ALIGN_BWD_PCT         17      /* 后退速度 */
#define BALL_ALIGN_SEARCH_TURN_MS  400u    /* 自转一次的幅度：400ms（普通脉冲
                                            * 250ms；2026-09-04 用户要求幅度
                                            * 稍大，1.5s 停顿间隔随循环删除） */
#define BALL_ALIGN_RECAPTURE_MAX     1     /* 球近车头时最多一次短倒车找回 */
#define BALL_ALIGN_RECAPTURE_PCT    12
#define BALL_ALIGN_RECAPTURE_MS    220u
#define BALL_ALIGN_TIMEOUT_MS      20000u  /* 对准总超时：停车回 SURVEY */

/* ---- PUSH：直线撞球 ---- */
#define BALL_MIN_EFFECTIVE_PCT     10.0f   /* 前轮有效出力下限（死区 9% + 余量）。
                                            * 球任务内差速限幅用：turn 过大时
                                            * 较慢一侧前轮会跌进死区"别住不走"，
                                            * 故限 |turn| ≤ speed - 此值 */
#define BALL_REALIGN_BACKUP_MS     400u    /* 撞球后球贴车头出画：先倒车一小段
                                            * 把球倒回视野再重新衔接（直接对准
                                            * 会对着车底干等，是"卡住"主因） */
#define BALL_PUSH_SPEED_PCT         12     /* 巡航推速（2026-09-03 实车 13 过快→12，
                                            * C 轮 10.8%；电池电压低时会更无力） */
#define BALL_PUSH_LAUNCH_PCT        22     /* 起步强推档（防死区闷熄；20 无力→22） */
#define BALL_PUSH_LAUNCH_MS         350u   /* 强推时长 */
#define BALL_PUSH_RAMP_MS           600u   /* 强推→巡航线性缓降时长 */
#define BALL_PUSH_STEER_GAIN        10.0f  /* 转向修正：x 偏差 → 转向 %
                                            * （2026-09-03 实车 8 转弯无力→10） */
#define BALL_PUSH_ALIGN_STRAFE_PCT 12      /* 纯横移短动作；须越过后轮死区 */
#define BALL_PUSH_STEP_MS          400u    /* 每次推进一段；段末不刹停滚入等帧窗口，
                                            * 保住动量（实测 200ms 段+刹停车起不了步） */
#define BALL_PUSH_BLIND_STEP_MS    150u    /* 近车头刚丢球时最多补推一次 */
#define BALL_PUSH_MAX_DRIVE_MS      5000u  /* 单球实际推进指令累计上限；到顶 ABORT */
#define BALL_PUSH_TIMEOUT_MS        15000u /* 单球墙钟超时；到点 ABORT */
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
#define BALL_RETURN_SPEED_PCT       15     /* 盲退速度（2026-09-04 用户要求加力
                                            * 14→15；C 轮 ×0.90 = 13.5%） */
#define BALL_RETURN_PULSE_SCALE     1.3f   /* 退回里程 = 推进脉冲 × 此系数。
                                            * 2026-09-04 实车 1.0 退得不够 → 1.3 */
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

/** @brief 终态是否为视觉确认成功；ABORT 返回 false。 */
bool ball_task_succeeded(void);

/** @brief walking_reset 联动复位 */
void ball_task_reset(uint32_t now_ms);

/** @brief 当前状态名（串口显示用） */
const char *ball_task_state_name(void);
