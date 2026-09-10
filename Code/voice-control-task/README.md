# voice-control-task —— 语音功能选择 · 攻城槌小车（自主题创新）

基于 **ESP32-S3** 的一体化演示固件：**ESP-SR 离线语音** + **BLE 手机遥控** + **D24A 真实电机**
+ **攻城槌模式（二维码找目标撞击 + GIF 表情动画 + 音效）**。

语音模块 = **“一个接口，通向三个功能”**（免唤醒，直接说命令词）：

```
                    ┌──“蓝牙遥控”→ 蓝牙遥控模式（BLE + 手机 App，已实现）
  功能选择(默认) ────┼──“攻城槌”  → 攻城槌模式（二维码找目标撞击，已实现）
                    └──“语音遥控”→ 语音遥控模式（前进/后退/左转/右转/停止，已实现）

   任意模式 ——“选择功能”→ 返回功能选择（先刹停；离开运动模式即关闭电机使能）
```

> 功能命名沿用 `feature/creative-ram` 分支（djj 的 CreativeTask 工程）的定稿：
> 原“表情”功能槽位升级为**攻城槌**模式，语音词 **“攻城槌”(gong cheng chui)**。
> 攻城槌模式说明：① 目标查找——小幅脉冲自旋 + 搜索表情动画 + 搜索音效，
> UVC 摄像头持续识别场地二维码（"1"=撞击目标，"2"=干扰）；② 撞击——识别到
> "1" 切撞击动画 + 音效，对准缓慢前进，超声波判撞 → 后退 → 循环撞击；
> ③ 目标丢失自动回查找。任意时刻说“选择功能”返回菜单。

## 语音命令表（全量注册，当前模式生效的才响应）

| 命令词 | 事件 | 生效模式 |
|--------|------|----------|
| 蓝牙遥控 | 进入蓝牙遥控 | 功能选择 |
| 攻城槌 | 进入攻城槌 | 功能选择 |
| 语音遥控 | 进入语音遥控 | 功能选择 |
| 前进 | 定时直行（到时自动停） | 语音遥控 |
| 后退（“倒退”同义） | 定时直行（到时自动停） | 语音遥控 |
| 左转 / 右转 | 定时原地转（到时自动停） | 语音遥控 |
| 停止 | 急刹（打断进行中动作） | 语音遥控 |
| 选择功能 | 返回功能选择 | 所有模式 |

所有命令词通过 MultiNet 中文模型**离线识别**，拼音词表见 `voice_cmd.c`
（MultiNet 要求拼音格式）。动作时长/速度宏在 `voice_remote.c` 顶部。

## 模块结构

```
main/
├── main.c            主程序：camera_task_start（USB Host）→ app_menu → 语音 → 状态屏
├── audio_in.h        音频输入接口（voice_cmd 用）
├── usb_audio.c/h     UAC 麦克风（语音输入）+ 喇叭（攻城槌音效），官方 usb_host_uac
├── voice_cmd.c/h     ESP-SR 识别核心：免唤醒连续识别，拼音词表 → voice_evt_t
├── app_menu.c/h      功能选择状态机：模式切换 / 事件裁决 / 刹停
├── bt_remote.c/h     蓝牙遥控：NimBLE NUS，MyCar-BLE，广播门控
├── ble_control.c/h   蓝牙协议/安全状态机（纯逻辑，宿主测试 tests/test_control.c）
├── voice_remote.c/h  语音遥控：定时动作（前进1s/转向0.5s，自动停）
├── ram_mode.c/h      攻城槌状态机：查找（自旋+动画+音效）→ 撞击循环
├── camera_task.c/h   UVC 摄像头采集 + esp_jpeg 解码 + quirc 二维码识别（usb_host_install 在此）
├── qr_detect.c/h     二维码检测（连通域裁剪窗，防看门狗）
├── display.c/h       ILI9163 TFT 自绘驱动 + 双 GIF 表情动画（搜索/撞击）
├── ultrasonic.c/h    HC-SR04 测距（39/40，撞击判定）
├── motion.c/h        D24A 电机驱动 + 三轮全向混控（task2 实车标定）
└── gif_*.bin / snd_*.wav   嵌入式资源（EMBED_FILES）
```

## 构建与烧录

Windows（本机 EIM 环境）：日常 `build-fast.bat`（增量），`flash.bat [COM口]` 烧录。
改 `sdkconfig.defaults` 或切 target 才用 `build.bat`（全量）。

> 烧录含：bootloader + 分区表 + 应用固件（~3MB）+ 语音模型 srmodels.bin（自动入 model 分区）。
> 分区表已扩容：factory 5MB / model 6MB（固件 2.94MB，原 3MB 分区余量不足）。

## 依赖（idf_component.yml，组件管理器自动拉取）

- `espressif/esp-sr` ^2.0.0：唤醒词 hilexin（保留加载、已禁用）+ MultiNet7 中文命令词
- `espressif/usb_host_uac` ^1.0.2：UAC 麦克风（16k/1ch/16bit 原生直通）+ 喇叭音效
- `usb_host_uvc` ^2.5.2 + `espressif/esp_jpeg` + `espressif/quirc`：摄像头二维码链路
- **PSRAM Octal** 必须启用（ESP-SR）；`USB_HOST_CONTROL_TRANSFER_MAX_SIZE=16384`
  （摄像头配置描述符实测 16330 字节）；CPU0 空闲任务不挂看门狗（quirc 病态耗时兜底）

## 硬件接线（与 task2 完全一致，零改动）

- 电机 D24A：STBY=1，PWM=2/6/9，方向=4/5/7/8/10/11
- TFT（ILI9163 1.8"）：SCK=13 / MOSI=15 / DC=17，CS 接地、RST 接 EN
- 超声波 HC-SR04：TRIG=39 / ECHO=40（攻城槌撞击判定）
- 摄像头 USB 三合一：GPIO19/20（视频+麦克风+喇叭同线）
- UART0 下载/监视：43/44（外接 USB-TTL）
- BLE 不占引脚

## 开发备忘

- **词表修改**：`voice_cmd.h` 枚举 + `voice_cmd.c` 词表（拼音）→ 重烧；
  模式内生效与否由 `app_menu.c` 裁决。
- **协议回归**：`gcc -o tc tests/test_control.c main/ble_control.c -I main && ./tc`
- **电机标定**：混控增益在 `motion.c` 顶部；turn 正=右转（与 task3 一致）。
- **源码来源**：攻城槌相关模块（ram_mode/camera_task/display/qr_detect/
  ultrasonic/usb_audio/资源与词表定稿）来自 `feature/creative-ram` 分支
  `Code/CreativeTask`（74de4a7），其完整独立工程保留在该分支；
  框架部分（motion/bt_remote/ble_control/voice_remote）与本工程同源。
