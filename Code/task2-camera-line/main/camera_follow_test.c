#include "camera_follow_test.h"

#include <stdint.h>
#include <stdio.h>

#include "camera_line.h"
#include "ultrasonic.h"
#include "walking.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 摄像头超过该时间没有新帧时禁用电机；恢复稳定见线后自动重新使能。 */
#define CAM_TEST_FRAME_TIMEOUT_MS  700u

static const char *TAG = "cam_follow_test";

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void app_camera_follow_test(void)
{
    printf("\n========== MODE 7：摄像头纯巡线 + 弯道恢复测试 ==========\n");
    printf("只初始化摄像头和电机；不初始化、不读取超声，也不执行避障。\n");
    printf("直线使用 PD 控制；外侧见线时减速大转，丢线后按最后侧别原地找线。\n");
    printf("等待连续稳定见线后才使能电机，请先把车头对准黑线。\n\n");

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

    uint32_t current_ms = now_ms();
    walking_reset(current_ms);
    walking_set_output_enabled(true);

    uint32_t last_frame_seq = cam.frame_seq;
    uint32_t last_frame_ms = current_ms;
    car_state_t last_state = walking_get_state();
    bool camera_watchdog_stopped = false;

    ESP_LOGI(TAG, "稳定见线 frame=%lu err=%+.3f，电机已使能",
             (unsigned long)cam.frame_seq, cam.error);

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        camera_line_get(&cam);
        current_ms = now_ms();
        const bool new_frame = cam.frame_seq != last_frame_seq;
        if (new_frame) {
            last_frame_seq = cam.frame_seq;
            last_frame_ms = current_ms;
        }

        if (current_ms - last_frame_ms >= CAM_TEST_FRAME_TIMEOUT_MS) {
            if (!camera_watchdog_stopped) {
                walking_set_output_enabled(false);
                ESP_LOGE(TAG, "超过 %ums 没有新画面，已禁用电机",
                         CAM_TEST_FRAME_TIMEOUT_MS);
                camera_watchdog_stopped = true;
            }
        } else if (camera_watchdog_stopped) {
            /* 掉线恢复时必须重新稳定见线，避免刚恢复一帧就突然动作。 */
            if (new_frame && cam.stable && cam.has_line) {
                walking_reset(current_ms);
                walking_set_output_enabled(true);
                last_state = walking_get_state();
                camera_watchdog_stopped = false;
                ESP_LOGI(TAG, "摄像头恢复稳定见线，电机重新使能");
            }
        } else {
            /* ULTRASONIC_INVALID 永远不会累计障碍确认，因此仍是纯摄像头测试；
             * 复用 walking 的 RUN / CORNER_SEARCH / LINE_SEARCH 弯道逻辑。 */
            walking_update_cam(&cam, ULTRASONIC_INVALID, current_ms);
        }

        const car_state_t state = walking_get_state();
        const bool state_changed = state != last_state;
        if (state_changed) {
            ESP_LOGW(TAG, "状态切换：%s -> %s",
                     walking_state_name(last_state), walking_state_name(state));
            last_state = state;
        }

        if (new_frame || state_changed) {
            const walking_command_t cmd = walking_get_last_command();
            ESP_LOGI(TAG,
                     "frame=%lu %s%s err=%+.3f rows=%u pat=%c%c%c%c state=%s -> F=%+d T=%+d",
                     (unsigned long)cam.frame_seq,
                     cam.has_line ? "见线" : "丢线",
                     cam.stable ? "(稳)" : "(抖)",
                     cam.error, cam.black_rows,
                     (cam.line_pattern >> 3) & 1 ? '1' : '0',
                     (cam.line_pattern >> 2) & 1 ? '1' : '0',
                     (cam.line_pattern >> 1) & 1 ? '1' : '0',
                     cam.line_pattern & 1 ? '1' : '0',
                     walking_state_name(state),
                     cmd.forward_percent, cmd.turn_percent);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}
