# voice-control-task —— 语音控制小车（自主题创新功能）

基于 **ESP32-S3 + ESP-SR** 的**完全离线**中文语音控制：
说 **"嗨 乐鑫"** 唤醒 → 说命令词 → 小车执行动作。

## 支持的语音命令

| 命令词 | 动作 | 说明 |
|--------|------|------|
| 前进 | 持续前进 | 直到新命令或"停止" |
| 倒退 | 持续后退 | 同上 |
| 左转 | 原地左转 | 同上 |
| 右转 | 原地右转 | 同上 |
| 调头 | 原地转 180° | 转完后自动停 |
| 停止 | 全停 | 打断任何动作 |

所有命令词均通过 MultiNet 中文模型**离线识别**，无需联网、无需训练模型。

## 模块结构

```
main/
├── main.c            主程序：初始化语音识别 + 心跳打印
├── audio_in.h/.c     I2S 麦克风采集抽象层（16kHz/16bit/mono）
├── voice_cmd.h/.c    ESP-SR 核心：唤醒词 + 命令词识别
├── voice_action.h/.c 命令→动作执行器（队列 + 独立任务）
└── motion.h/.c       电机接口（当前为打印占位，可替换为真实运动学）
```

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

> 烧录包含两部分：应用固件 `voice-control-task.bin` + 语音模型 `srmodels.bin`（自动烧入 model 分区）。

## 依赖与配置

- **ESP-SR**（`espressif/esp-sr ^2.0.0`，组件管理器自动下载，已适配 v2.5.1 API）
- **模型**（`sdkconfig.defaults`）：
  - 唤醒词：`CONFIG_SR_WN_WN9_HILEXIN=y`（Hi,乐鑫）
  - 中文命令词：`CONFIG_SR_MN_CN_MULTINET7_QUANT=y`（MultiNet7）
- **分区表**：自定义 `partitions.csv`，新增 8MB `model` 分区存放语音模型
- **PSRAM**：已启用（语音识别内存占用大，必须）

## 硬件接线（待定，先跑通逻辑）

I2S 数字麦克风（如 INMP441）默认占用 GPIO 4/5/6（避开 USB Host 19/20）：

| 引脚 | GPIO | 接麦克风 |
|------|------|----------|
| BCLK | GPIO5 | SCK/BCLK |
| WS   | GPIO4 | WS/LRCK |
| DIN  | GPIO6 | DOUT/SD  |

> INMP441 的 L/R 脚接 GND 选左声道。

## 后续接入真实小车

`motion.c` 目前是打印占位（与 camera-task 接口一致）。接入真实运动学时
替换 `motion.c` 实现即可，`voice_action` 无需改动。动作参数（速度/调头时长）
见 `voice_action.c` 顶部宏，需按底盘实测标定。
