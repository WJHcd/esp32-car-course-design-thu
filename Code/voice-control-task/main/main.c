/*
 * 语音控制小车 —— 创新功能（自主题）
 *
 * 语音模块 = “一个接口，通向三个功能”（免唤醒：直接说命令）：
 *
 *   功能选择（默认）下直接说出功能名切换：
 *     “蓝牙遥控” → 蓝牙遥控模式（BLE，手机遥控，已实现）
 *     “表情”     → 表情模式（TFT 彩屏，接口占位）
 *     “语音遥控” → 语音遥控模式（已实现：前进/后退/左转/右转/停止）
 *   任意模式说“选择功能” → 返回功能选择
 *
 * 模块划分：
 *   audio_in_uac.c    USB UAC 麦克风采集（摄像头三合一，16kHz/16bit/mono）
 *   voice_cmd.c       ESP-SR 识别核心：免唤醒连续命令识别 → 事件回调
 *   app_menu.c        功能选择状态机：模式切换/事件裁决/刹停
 *   bt_remote.c       蓝牙遥控（BLE MyCar-BLE，已实现）
 *   expression.c      表情（接口占位，ST7735 未接入）
 *   voice_remote.c    语音遥控（已实现，命令 → motion 动作）
 *   motion.c          D24A 真实电机驱动（三轮全向混控）
 *
 * 参考：esp-skainet/examples/cn_speech_commands_recognition
 * 依赖：flash "model" 分区（唤醒词+命令词模型；免唤醒模式下
 *       唤醒词模型保留加载但已 disable，命令词模型 mn7_cn 全程监听）
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "voice_cmd.h"
#include "app_menu.h"
#include "status_disp.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "语音控制小车启动（离线语音识别：ESP-SR）");

    /* 1. 功能选择菜单（内部完成 motion_init + 三个功能模块 init） */
    app_menu_init();

    /* 2. 语音识别（失败不影响运行，仅禁用语音控制） */
    esp_err_t ret = voice_cmd_init(app_menu_on_voice_evt, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "语音识别初始化失败: %s", esp_err_to_name(ret));
    }

    /* 3. 状态屏（模式/BLE/电机/麦克风/动作回显；FACE 模式自动让屏） */
    status_disp_init();

    /* 3. 心跳打印 */
    while (1) {
        ESP_LOGI(TAG, "运行中：模式=%s ｜ 直接说命令（免唤醒连续识别）",
                 app_menu_mode_name(app_menu_get_mode()));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
