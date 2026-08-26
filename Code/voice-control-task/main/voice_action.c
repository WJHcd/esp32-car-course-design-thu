#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "voice_action.h"
#include "motion.h"

/* ============================================================
 * 语音动作执行器
 *
 * 独立任务循环接收命令队列，映射到 motion 抽象接口：
 *   - 前进/倒退：差速直行（左=右）
 *   - 左转/右转：原地旋转（正=逆时针）
 *   - 调头：原地旋转固定时长后自动停
 * 新命令到达时先清空旧命令，立即生效。
 *
 * ⚠️ 速度/时长参数为暂定值，上电后需按实际底盘标定：
 *    - ACTION_SPEED 建议 30~60（过大易打滑/过冲）
 *    - ACTION_UTURN_MS 以“实测转 180°”为准
 * ============================================================ */

#define ACTION_SPEED      50      /* 前进/倒退速度（-100~100） */
#define ACTION_ROT_SPEED  60      /* 旋转速度（-100~100） */
#define ACTION_UTURN_MS   1100    /* 调头 180° 时长（ms，需标定） */
#define ACTION_QUEUE_LEN  4

static const char *TAG = "voice_action";
static QueueHandle_t s_cmd_queue = NULL;

static void action_task(void *arg)
{
    while (1) {
        voice_cmd_t cmd;
        if (xQueueReceive(s_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        ESP_LOGI(TAG, "执行: %s", voice_cmd_to_str(cmd));

        switch (cmd) {
        case VOICE_CMD_FORWARD:
            motion_set_left_right(ACTION_SPEED, ACTION_SPEED);   /* 持续前进 */
            break;
        case VOICE_CMD_BACKWARD:
            motion_set_left_right(-ACTION_SPEED, -ACTION_SPEED); /* 持续倒退 */
            break;
        case VOICE_CMD_LEFT:
            motion_rotate(ACTION_ROT_SPEED);                     /* 持续左转 */
            break;
        case VOICE_CMD_RIGHT:
            motion_rotate(-ACTION_ROT_SPEED);                    /* 持续右转 */
            break;
        case VOICE_CMD_UTURN:
            /* 一次性动作：转 180° 后自动停 */
            motion_rotate(ACTION_ROT_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ACTION_UTURN_MS));
            motion_stop();
            break;
        case VOICE_CMD_STOP:
        default:
            motion_stop();
            break;
        }
    }
}

void voice_action_init(void)
{
    if (s_cmd_queue) {
        return;   /* 已初始化 */
    }
    s_cmd_queue = xQueueCreate(ACTION_QUEUE_LEN, sizeof(voice_cmd_t));
    motion_init();
    if (xTaskCreatePinnedToCore(action_task, "voice_action", 4096, NULL, 6, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "action 任务创建失败");
    }
}

void voice_action_execute(voice_cmd_t cmd, void *ctx)
{
    (void)ctx;
    if (!s_cmd_queue) {
        return;
    }
    /* 清空排队中的旧命令，保证新命令立即生效（可打断调头/前进） */
    xQueueReset(s_cmd_queue);
    xQueueSend(s_cmd_queue, &cmd, 0);
}
