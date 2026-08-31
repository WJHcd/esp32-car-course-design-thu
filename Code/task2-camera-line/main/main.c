#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "ultrasonic.h"
#include "walking.h"
#include "camera_line.h"
#include "ascii_preview.h"
#include "camera_follow_test.h"
#include "encoder.h"
#include "tft_st7735.h"
#include "test.h"

/* ============================================================
 * 任务二 · 综合主程序（摄像头巡线 + 超声波避障）
 *
 * 与任务一的区别：巡线误差来源由四路红外改为 USB 摄像头
 * （UVC 采集 → JPEG 解码 → 黑线质心），walking.c 控制逻辑不变。
 *
 * ⚠️ 摄像头占用 GPIO19/20（板载 USB Host），
 *    下载/监视需用外部 USB 转 TTL 接 UART0（GPIO43/44）。
 *
 * ⚠️ 运行模式切换：改下面的 TASK2_MODE 后重新编译烧录
 *   0 = 综合模式（摄像头巡线 + 超声波避障，正式跑车用）
 *   1 = 摄像头巡线测试（打印黑线误差，不驱动电机）
 *   2 = 超声检测测试   （test_ultrasonic 标定统计）
 *   3 = 平移方向自检   （驱动电机；着地观察平移/前进方向）
 *   4 = TFT 面板自检   （仅屏幕，红绿蓝黑循环）
 *   5 = 摄像头画面预览（摄像头画面实时显示到 TFT，确认摄像头好坏）
 *   6 = 摄像头ASCII预览（每秒打印 64x24 字符画到串口，'#'=判黑像素）
 *   7 = 摄像头纯巡线实车测试（无超声/避障/找线旋转；丢线只停车）
 * ============================================================ */
#define TASK2_MODE  7

static const char *TAG = "task2";

/*
 * 摄像头巡线测试模式：只初始化摄像头和屏幕显示，不驱动电机、不读超声。
 * 用于标定黑线阈值（CAM_BIN_THRESHOLD）和确认误差方向。
 * 信息显示在小车 TFT 屏幕上（无需电脑串口）：
 *   A/B/C 行：三路电机 RPM（未接编码器脉冲时为 0）
 *   D 行：超声距离（本模式未初始化超声，显示 ---）
 *   E 行：摄像头巡线状态 E:+0.12 / E:LOST
 */
static void app_camera_line_test(void)
{
    printf("\n========== 摄像头巡线测试模式 ==========\n");
    printf("黑线误差显示在小车 TFT 屏幕第 5 行（E:xx）。\n");
    printf("把车对准黑线前后移动/左右平移，观察 E 值变化（左负右正）。\n\n");

    tft_st7735_init();
    tft_st7735_start_telemetry();

    if (!camera_line_init()) {
        ESP_LOGE(TAG, "摄像头初始化失败");
        return;
    }

    while (1) {
        camera_line_result_t res;
        camera_line_get(&res);
        /* 把巡线状态推给屏幕显示 */
        tft_st7735_set_line(res.has_line, res.error, res.stable);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*
 * 平移方向标定模式：轮子着地、短促脉冲，直接观察车体实际位移方向。
 * 注意：架空轮子看不出是"平移"还是"原地旋转"（两者轮子转向几乎相同），
 * 必须着地短脉冲实测。判读：
 *   - 左移脉冲车体实际向右 → 翻转 STRAFE_SIGN_L/R（B 不变）
 *   - 左移脉冲变成原地旋转 → 翻转 STRAFE_SIGN_B
 *   - 平移带明显旋转/斜走 → 微调 STRAFE_B_GAIN
 */
static void app_strafe_test(void)
{
#define STRAFE_TEST_SPEED_PCT  12
#define STRAFE_TEST_FWD_PCT    15
#define STRAFE_TEST_MS         400   /* 短脉冲，位移几厘米，安全 */
#define STRAFE_TEST_PAUSE_MS   300

    walking_init();
    walking_set_output_enabled(true);

    printf("\n========== 平移方向自检（标定用）==========\n");
    printf("请在宽敞地面/桌面【着地】运行（勿架空）；车会循环：\n");
    printf("  左移 0.4s → 停 0.3s → 右移 0.4s → 停 0.3s → 前进 0.5s → 停 0.3s\n");
    printf("判读：左移脉冲车体应【向左平移】（不旋转、不前/后）。\n");
    printf("  走反(向右)   → 翻转 walking.c 的 STRAFE_SIGN_L/R（B 不变）\n");
    printf("  原地旋转     → 翻转 STRAFE_SIGN_B\n");
    printf("  斜走/带旋转  → 微调 STRAFE_B_GAIN\n\n");

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle = 0;

    while (1) {
        const uint32_t t1 = (uint32_t)(esp_timer_get_time() / 1000);
        printf("[%6lu ms] 左移(+%d)\n", (unsigned long)t1, STRAFE_TEST_SPEED_PCT);
        walking_test_drive(0, 0, STRAFE_TEST_SPEED_PCT);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_MS));
        walking_test_drive(0, 0, 0);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_PAUSE_MS));

        const uint32_t t2 = (uint32_t)(esp_timer_get_time() / 1000);
        printf("[%6lu ms] 右移(-%d)\n", (unsigned long)t2, STRAFE_TEST_SPEED_PCT);
        walking_test_drive(0, 0, -STRAFE_TEST_SPEED_PCT);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_MS));
        walking_test_drive(0, 0, 0);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_PAUSE_MS));

        const uint32_t t3 = (uint32_t)(esp_timer_get_time() / 1000);
        printf("[%6lu ms] 前进(+%d)\n", (unsigned long)t3, STRAFE_TEST_FWD_PCT);
        walking_test_drive(STRAFE_TEST_FWD_PCT, 0, 0);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
        walking_test_drive(0, 0, 0);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_PAUSE_MS));

        printf("[%6lu ms] cycle %lu 完成\n",
               (unsigned long)(esp_timer_get_time() / 1000), (unsigned long)cycle);
        cycle++;
    }
}

/* 仅 TFT 面板诊断：不初始化红外、超声、电机或编码器。 */
static void app_tft_panel_test(void)
{
    printf("\n========== TFT 独立面板测试 ==========\n");
    printf("将循环显示：红 -> 绿 -> 蓝 -> 黑，每种 1 秒。\n");
    tft_st7735_init();
    tft_st7735_panel_test();
}

/*
 * 摄像头画面预览模式：把 UVC 摄像头画面实时显示到 TFT 屏幕，
 * 用于确认摄像头是否出帧/成像正常。只初始化摄像头和屏幕，
 * 不驱动电机、不读超声。全屏刷新约 3fps（SPI 1MHz 限制）。
 */
static void app_camera_preview(void)
{
    printf("\n========== 摄像头画面预览模式 ==========\n");
    printf("摄像头画面实时显示在 TFT 屏幕（约 3fps）。\n");
    printf("屏幕有画面 = 摄像头正常；仍白屏 = 检查接线/供电。\n\n");

    tft_st7735_init();              /* 含红绿蓝黑上电自检 */
    if (!camera_line_init()) {
        ESP_LOGE(TAG, "摄像头初始化失败");
        return;
    }
    camera_line_set_frame_cb(tft_st7735_show_camera_frame);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*
 * 摄像头 ASCII 串口预览模式：不依赖电脑，每秒把摄像头画面
 * 降采样成 64x24 字符画打印到串口监视器（80 列终端可完整显示），
 * 同时把黑线状态同步显示到小车 TFT 屏幕（E 行）。
 *   '#' = 低于 CAM_BIN_THRESHOLD 判黑的像素（即算法看到的黑线）
 *   其他字符 = 亮度渐变（空格=全白，*=高亮）
 * 用于确认摄像头出帧/成像/黑线阈值是否合理。
 */
static void app_camera_ascii_preview(void)
{
    printf("\n========== 摄像头 ASCII 串口预览 ==========\n");
    printf("每 %d 秒打印一帧 %dx%d 字符画：\n",
           ASCII_PREVIEW_INTERVAL_MS / 1000,
           ASCII_PREVIEW_WIDTH, ASCII_PREVIEW_HEIGHT);
    printf("  '#' = 判为黑线的像素（阈值 CAM_BIN_THRESHOLD=%d）\n", CAM_BIN_THRESHOLD);
    printf("  其余字符按亮度渐变：空格=全白，*=高亮\n");
    printf("判读：无画面/全空 = 检查摄像头接线与供电；\n");
    printf("      黑线应显示为一串 '#'；画面颠倒/镜像 = 调整摄像头安装。\n");
    printf("小车屏幕 E 行同步显示：E:+0.12 OK=见线稳定，E:+0.12 !=见线未稳，E:LOST=丢线。\n\n");

    tft_st7735_init();
    tft_st7735_start_telemetry();
    if (!camera_line_init()) {
        ESP_LOGE(TAG, "摄像头初始化失败");
        return;
    }
    camera_line_set_stats_log(false);   /* 关周期日志，保持字符画完整 */
    camera_line_set_frame_cb(ascii_preview_on_frame);

    while (1) {
        camera_line_result_t res;
        camera_line_get(&res);
        tft_st7735_set_line(res.has_line, res.error, res.stable);
        printf("状态: %s%s err=%+.2f rows=%u wide=%u frame=%lu\n",
               res.has_line ? "见线" : "丢线",
               res.stable ? "(稳)" : "(抖)",
               res.error, res.black_rows, res.wide_black_rows,
               (unsigned long)res.frame_seq);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*
 * 综合模式：摄像头巡线 + 超声波避障，20ms 控制周期。
 * 巡线误差来自 camera_line（黑线质心，[-1,+1]），
 * walking.c 的 PD 增益通过 CAM_ERR_SCALE 适配该量纲。
 */
void app_integrated(void)
{
    ultrasonic_init();
    walking_init();
    encoder_init();
    tft_st7735_init();
    tft_st7735_start_telemetry();
    if (!camera_line_init()) {
        ESP_LOGE(TAG, "摄像头初始化任务创建失败；电机保持禁用");
        return;
    }

    printf("\n========== 任务二综合模式：摄像头巡线 + 超声波避障 ==========\n");
    printf("等待摄像头稳定见线后才使能电机，避免开机枚举期间自主找线。\n\n");

    while (true) {
        camera_line_result_t cam;
        camera_line_get(&cam);
        tft_st7735_set_line(cam.has_line, cam.error, cam.stable);
        if (camera_line_ready() && camera_line_stable() && cam.has_line) {
            const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            walking_reset(now_ms);
            walking_set_output_enabled(true);
            printf("摄像头已稳定见线（frame=%lu, err=%+.2f），电机输出已使能。\n",
                   (unsigned long)cam.frame_seq, cam.error);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    uint32_t tick = 0;
    uint16_t last_distance = ULTRASONIC_INVALID;
    car_state_t last_state = walking_get_state();
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        /* 摄像头巡线：取最近一帧的黑线误差 */
        camera_line_result_t cam;
        camera_line_get(&cam);
        tft_st7735_set_line(cam.has_line, cam.error, cam.stable);

        /* 每 5 拍（100ms）测一次超声 */
        uint16_t distance_sample = ULTRASONIC_INVALID;
        if (tick % 5 == 0) {
            distance_sample = ultrasonic_measure_cm();
            last_distance = distance_sample;
            if (distance_sample != ULTRASONIC_INVALID) {
                tft_st7735_set_distance(distance_sample);
            }
        }

        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        walking_update_cam(&cam, distance_sample, now_ms);

        const car_state_t state = walking_get_state();
        const bool state_changed = (state != last_state);
        last_state = state;

        if (state_changed || tick % 25 == 0) {
            walking_command_t cmd = walking_get_last_command();
            printf("[%6lu ms] state=%-16s %s err=%+.2f "
                   "cmd(F=%+d,T=%+d,S=%+d) 距离=%s%u%s\n",
                   (unsigned long)now_ms,
                   walking_state_name(state),
                   cam.has_line ? "见线" : "丢线",
                   cam.error,
                   cmd.forward_percent, cmd.turn_percent, cmd.strafe_percent,
                   last_distance == ULTRASONIC_INVALID ? "无效" : "",
                   last_distance == ULTRASONIC_INVALID ? 0 : last_distance,
                   last_distance == ULTRASONIC_INVALID ? "" : "cm");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
        tick++;
    }
}

void app_main(void)
{
#if   TASK2_MODE == 1
    app_camera_line_test();
#elif TASK2_MODE == 2
    test_ultrasonic();
#elif TASK2_MODE == 3
    app_strafe_test();
#elif TASK2_MODE == 4
    app_tft_panel_test();
#elif TASK2_MODE == 5
    app_camera_preview();
#elif TASK2_MODE == 6
    app_camera_ascii_preview();
#elif TASK2_MODE == 7
    app_camera_follow_test();
#else
    app_integrated();
#endif
}
