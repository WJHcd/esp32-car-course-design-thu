/*
 * 攻城槌小车 —— 创新任务（CreativeTask）
 *
 * 在语音功能选择（voice-control-task）基础上，将"表情"功能替换为
 * "攻城槌"模式（语音说"攻城槌"进入，"选择功能"退出）：
 *
 *   ① 目标查找：小幅度脉冲自旋 + 搜索表情动画 + 搜索音效，
 *      UVC 摄像头持续识别二维码（箱贴 "1"/"2"，"1" 为撞击目标）
 *   ② 撞击：识别到 "1" → 撞击表情动画 + 撞击音效，
 *      对准二维码缓慢前进，超声波判撞 → 后退 → 循环撞击
 *   ③ 目标丢失自动退回查找；任意时刻"选择功能"返回菜单
 *
 * 模块：
 *   camera_task.c  UVC 摄像头 + quirc 二维码识别（结果共享）
 *   usb_audio.c    UAC 麦克风（语音输入）+ 喇叭（音效播放，同摄像头 USB）
 *   voice_cmd.c    ESP-SR 离线命令词识别（"攻城槌"等）
 *   app_menu.c     功能选择状态机（蓝牙遥控 / 攻城槌 / 语音遥控）
 *   ram_mode.c     攻城槌状态机（查找→撞击循环）
 *   display.c      ILI9163 TFT + 双 GIF 表情动画
 *   motion.c       D24A 电机三轮混控（task2 标定增益）
 *   ultrasonic.c   HC-SR04 测距（撞击判定）
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "camera_task.h"
#include "app_menu.h"
#include "voice_cmd.h"
#include "status_disp.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "攻城槌小车启动（语音功能选择 + 二维码找目标撞击）");

    /* 1. 摄像头管线（usb_host_install 在此完成，UAC 依赖它；二维码持续识别） */
    esp_err_t ret = camera_task_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "摄像头管线启动失败: %s", esp_err_to_name(ret));
    }

    /* 2. 功能选择菜单（内部完成 motion_init + 三个功能模块 init） */
    app_menu_init();

    /* 3. 语音识别（免唤醒：直接说 蓝牙遥控/攻城槌/语音遥控） */
    ret = voice_cmd_init(app_menu_on_voice_evt, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "语音识别初始化失败: %s", esp_err_to_name(ret));
    }

    /* 4. 状态屏（模式/提示/麦克风/电机状态；攻城槌模式自动让屏给动画） */
    status_disp_init();

    /* 5. 心跳打印 */
    while (1) {
        ESP_LOGI(TAG, "运行中：模式=%s", app_menu_mode_name(app_menu_get_mode()));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
