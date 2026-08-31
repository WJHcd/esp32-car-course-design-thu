#include "camera_follow_test.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "camera_line.h"
#include "walking.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------------- 摄像头纯巡线测试标定区 ----------------
 * 测试版故意低速、无自动找线：识别不到黑线时立即停止，见线后再继续。
 * 黑线在右侧(error>0)时应给正 turn；若实车确实修正到反方向，只把 SIGN 改 -1。
 */
#define CAM_TEST_FORWARD_PCT       15
#define CAM_TEST_MIN_FORWARD_PCT   11
#define CAM_TEST_TURN_GAIN         13.0f
#define CAM_TEST_MAX_TURN_PCT      12
#define CAM_TEST_STEERING_SIGN      (+1)
#define CAM_TEST_CENTER_DEADBAND    0.06f
/* 滤波系数按帧率标定：EMA 时间常数 ≈ 帧间隔/α。帧率从 ~4fps 提到 ~7.4fps
 * 后 α 相应减半，保持同样的平滑时间常数，避免帧间抖动直接进入转向。 */
#define CAM_TEST_FILTER_ALPHA       0.22f
/* 转向斜率限幅：相邻帧转向指令最大变化量。高帧率下无限制的转向可
 * 以 ~8Hz 正反交替，超出机械惯性响应能力，实车表现为画龙/乱跑。 */
#define CAM_TEST_TURN_SLEW_PCT      5
#define CAM_TEST_FRAME_TIMEOUT_MS   900u

static const char *TAG = "cam_follow_test";

static int clamp_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void app_camera_follow_test(void)
{
    printf("\n========== MODE 7：摄像头纯巡线实车测试 ==========\n");
    printf("仅启用摄像头和电机；不初始化超声、编码器、TFT。\n");
    printf("丢线时只停车等待，绝不会进入原地旋转找线或避障。\n");
    printf("等待连续稳定见线后才使能电机，请先把车头对准黑色直线。\n\n");

    walking_init();                  /* 初始化后 STBY 保持关闭 */
    if (!camera_line_init()) {
        ESP_LOGE(TAG, "摄像头初始化失败；电机保持禁用");
        return;
    }

    camera_line_result_t cam = {0};
    while (1) {
        camera_line_get(&cam);
        if (camera_line_ready() && cam.stable && cam.has_line) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    walking_set_output_enabled(true);
    uint32_t last_frame_seq = cam.frame_seq;
    uint32_t last_frame_ms = now_ms();
    float filtered_error = cam.error;
    bool stopped_for_loss = false;
    int last_turn = 0;

    ESP_LOGI(TAG, "稳定见线 frame=%lu err=%+.3f，开始低速巡线",
             (unsigned long)cam.frame_seq, cam.error);

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        camera_line_get(&cam);
        const uint32_t current_ms = now_ms();

        if (cam.frame_seq != last_frame_seq) {
            last_frame_seq = cam.frame_seq;
            last_frame_ms = current_ms;

            if (!cam.has_line) {
                /* 测试版不猜方向、不找线：当前帧丢线就立刻停。 */
                walking_test_drive(0, 0, 0);
                last_turn = 0;   /* 重新起步时转向从 0 平滑爬升 */
                if (!stopped_for_loss) {
                    ESP_LOGW(TAG,
                             "丢线，已停车等待：frame=%lu rows=%u stable=%d",
                             (unsigned long)cam.frame_seq, cam.black_rows, cam.stable);
                    stopped_for_loss = true;
                }
            } else {
                filtered_error += CAM_TEST_FILTER_ALPHA * (cam.error - filtered_error);
                const float control_error =
                    fabsf(filtered_error) < CAM_TEST_CENTER_DEADBAND ? 0.0f : filtered_error;
                int turn = (int)lroundf(CAM_TEST_STEERING_SIGN *
                                        CAM_TEST_TURN_GAIN * control_error);
                turn = clamp_int(turn, -CAM_TEST_MAX_TURN_PCT, CAM_TEST_MAX_TURN_PCT);
                /* 斜率限幅：本帧转向相对上一帧最多变化 CAM_TEST_TURN_SLEW_PCT */
                turn = clamp_int(turn,
                                 last_turn - CAM_TEST_TURN_SLEW_PCT,
                                 last_turn + CAM_TEST_TURN_SLEW_PCT);
                last_turn = turn;
                int forward = CAM_TEST_FORWARD_PCT - abs(turn) / 3;
                forward = clamp_int(forward,
                                    CAM_TEST_MIN_FORWARD_PCT,
                                    CAM_TEST_FORWARD_PCT);

                walking_test_drive(forward, turn, 0);
                stopped_for_loss = false;
                ESP_LOGI(TAG,
                         "frame=%lu %s raw=%+.3f filt=%+.3f rows=%u wide=%u -> F=%d T=%+d",
                         (unsigned long)cam.frame_seq,
                         cam.stable ? "稳" : "原始",
                         cam.error, filtered_error,
                         cam.black_rows, cam.wide_black_rows,
                         forward, turn);
            }
        } else if (current_ms - last_frame_ms >= CAM_TEST_FRAME_TIMEOUT_MS) {
            /* 摄像头卡住/掉线时不允许沿旧命令继续跑。 */
            walking_test_drive(0, 0, 0);
            last_turn = 0;
            if (!stopped_for_loss) {
                ESP_LOGE(TAG, "超过 %ums 没有新画面，已停车",
                         CAM_TEST_FRAME_TIMEOUT_MS);
                stopped_for_loss = true;
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}
