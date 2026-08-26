/*
 * 语音控制小车 —— 创新功能（自主题）
 *
 * 基于 ESP32-S3 + ESP-SR 的离线中文语音控制：
 *   说“嗨 乐鑫”唤醒 → 说命令词（前进/倒退/左转/右转/调头/停止）→ 小车执行
 *
 * 模块划分：
 *   audio_in_i2s.c   I2S 麦克风采集（16kHz/16bit/mono）
 *   voice_cmd.c      ESP-SR：唤醒词 + MultiNet 中文命令词识别
 *   voice_action.c   命令 → 动作执行（队列 + 独立任务）
 *   motion.c         电机接口（当前为打印占位，可替换为真实运动学）
 *
 * 参考：esp-skainet/examples/cn_speech_commands_recognition
 * 依赖：flash "model" 分区（唤醒词+命令词模型），menuconfig 选择唤醒词
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "voice_cmd.h"
#include "voice_action.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "语音控制小车启动（离线语音识别：ESP-SR）");

    /* 1. 动作执行器（内部含 motion_init） */
    voice_action_init();

    /* 2. 语音识别（失败不影响运行，仅禁用语音控制） */
    esp_err_t ret = voice_cmd_init(voice_action_execute, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "语音识别初始化失败: %s", esp_err_to_name(ret));
    }

    /* 3. 心跳打印 */
    while (1) {
        ESP_LOGI(TAG, "运行中：说“嗨 乐鑫”唤醒后下达命令（前进/倒退/左转/右转/调头/停止）");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
