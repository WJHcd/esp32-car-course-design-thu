#include "test.h"
#include "walking.h"
#include "ultrasonic.h"
#include <stdio.h>

static const infrared_pattern_t CENTER = { .a = 0, .b = 1, .c = 1, .d = 0 };
static const infrared_pattern_t LEFT   = { .a = 1, .b = 0, .c = 0, .d = 0 };
static const infrared_pattern_t RIGHT  = { .a = 0, .b = 0, .c = 0, .d = 1 };
static const infrared_pattern_t RIGHT_INNER = { .a = 0, .b = 0, .c = 1, .d = 0 };
static const infrared_pattern_t WHITE  = { .a = 0, .b = 0, .c = 0, .d = 0 };
static const infrared_pattern_t BLACK  = { .a = 1, .b = 1, .c = 1, .d = 1 };

static void print_step(uint32_t now_ms, car_state_t before)
{
    const car_state_t after = walking_get_state();
    const walking_command_t cmd = walking_get_last_command();
    if (after != before) {
        printf("[%4lu ms] %-16s -> %-16s | F=%+d T=%+d err=%+.2f\n",
               (unsigned long)now_ms,
               walking_state_name(before), walking_state_name(after),
               cmd.forward_percent, cmd.turn_percent,
               walking_get_last_error());
    }
}

static void step_once(const infrared_pattern_t *pat,
                      uint16_t distance_cm,
                      uint32_t now_ms)
{
    const car_state_t before = walking_get_state();
    walking_update(pat, distance_cm, now_ms);
    print_step(now_ms, before);
}

void test_walking(void)
{
    walking_init();
    walking_set_output_enabled(false);
    walking_reset(0);

    printf("\n========== 行走状态机确定性自检 ==========\n");
    printf("电机 STBY 保持低；使用模拟毫秒时间，不等待真实电机动作。\n\n");

    uint32_t now = 0;

    /* 基本循迹误差方向与全黑处理。 */
    step_once(&CENTER, ULTRASONIC_INVALID, now += 20);
    printf("中心线: error=%+.2f\n", walking_get_last_error());
    step_once(&LEFT, ULTRASONIC_INVALID, now += 20);
    printf("左侧线: error=%+.2f\n", walking_get_last_error());
    step_once(&RIGHT, ULTRASONIC_INVALID, now += 20);
    printf("右侧线: error=%+.2f\n", walking_get_last_error());
    step_once(&BLACK, ULTRASONIC_INVALID, now += 20);
    printf("全黑低速直行: F=%+d T=%+d\n",
           walking_get_last_command().forward_percent,
           walking_get_last_command().turn_percent);

    /* 全白超过宽限时间后进入搜索；连续两拍重新见线后恢复 RUN。 */
    for (int i = 0; i < 10; i++) {
        step_once(&WHITE, ULTRASONIC_INVALID, now += 20);
    }
    step_once(&CENTER, ULTRASONIC_INVALID, now += 20);
    step_once(&CENTER, ULTRASONIC_INVALID, now += 20);

    /*
     * 锐角弯：外侧单探头后全白，应立即进入专用原地找出口状态；
     * 对侧内探见线（扫到旧线/转过头）不能算出口，
     * 同侧内侧/中央连续三拍才恢复 RUN。
     */
    step_once(&LEFT, ULTRASONIC_INVALID, now += 20);
    step_once(&WHITE, ULTRASONIC_INVALID, now += 20);
    printf("锐角全白: state=%s F=%+d T=%+d\n",
           walking_state_name(walking_get_state()),
           walking_get_last_command().forward_percent,
           walking_get_last_command().turn_percent);
    /* 左转中对侧内探(c)见线：应继续 CORNER_SEARCH，不得当作出口。 */
    step_once(&RIGHT_INNER, ULTRASONIC_INVALID, now += 20);
    printf("错侧见线: state=%s（期望仍为 CORNER_SEARCH）\n",
           walking_state_name(walking_get_state()));
    step_once(&LEFT, ULTRASONIC_INVALID, now += 20);
    step_once(&CENTER, ULTRASONIC_INVALID, now += 20);
    step_once(&CENTER, ULTRASONIC_INVALID, now += 20);
    step_once(&CENTER, ULTRASONIC_INVALID, now += 20);

    /* 两个相隔 100ms 的独立近距离样本触发避障，旧样本不会重复计数。 */
    step_once(&CENTER, 10, now += 100);
    for (int i = 0; i < 4; i++) {
        step_once(&CENTER, ULTRASONIC_INVALID, now += 20);
    }
    step_once(&CENTER, 11, now += 20);

    /* 推进模拟时间，覆盖停车、后退、转出、绕行、转回和找线。 */
    bool recovered = false;
    for (int i = 0; i < 160; i++) {
        const car_state_t before = walking_get_state();
        const infrared_pattern_t *pat =
            (before == CAR_STATE_AVOID_FIND_LINE) ? &CENTER : &WHITE;
        step_once(pat, ULTRASONIC_INVALID, now += 20);
        if (before == CAR_STATE_AVOID_FIND_LINE &&
            walking_get_state() == CAR_STATE_RUN) {
            recovered = true;
            break;
        }
    }

    printf("\n最终状态=%s（期望 RUN） | %s\n",
           walking_state_name(walking_get_state()),
           recovered ? "避障流程完成" : "避障流程未按预期完成");
    printf("自检结束；电机仍保持禁用。\n");
}
