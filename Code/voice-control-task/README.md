# voice-control-task —— 语音功能选择 · 三合一控制小车（自主题创新）

基于 **ESP32-S3** 的一体化演示固件：**ESP-SR 离线语音** + **BLE 手机遥控** + **D24A 真实电机驱动**。

语音模块 = **“一个接口，通向三个功能”**：说 **"嗨 乐鑫"** 唤醒后，
在功能选择处说出功能名即可切换模式，任意模式说 **"选择功能"** 返回菜单：

```
                    ┌──“蓝牙遥控”→ 蓝牙遥控模式（BLE + 手机 App，已实现）
  功能选择(默认) ────┼──“表情”   → 表情模式（TFT 彩屏表情，接口占位）
                    └──“语音遥控”→ 语音遥控模式（前进/后退/左转/右转/停止，已实现）

   任意模式 ——“选择功能”→ 返回功能选择（先刹停；离开运动模式即关闭电机使能）
```

## 语音命令表（全量注册，当前模式生效的才响应）

| 命令词 | 事件 | 生效模式 |
|--------|------|----------|
| 蓝牙遥控 | 进入蓝牙遥控 | 功能选择 |
| 表情 | 进入表情 | 功能选择 |
| 语音遥控 | 进入语音遥控 | 功能选择 |
| 前进 | 前向直行（持续） | 语音遥控 |
| 后退（“倒退”同义） | 前向直行（持续） | 语音遥控 |
| 左转 / 右转 | 原地旋转（持续） | 语音遥控 |
| 停止 | 全停 | 语音遥控 |
| 选择功能 | 返回功能选择 | 所有模式 |

所有命令词通过 MultiNet 中文模型**离线识别**。每次命令前需说“嗨 乐鑫”重新唤醒
（识别窗口一次命令）。

## 蓝牙遥控模式（已实现，由 task3-ble-control 移植）

- **NimBLE 外设**，广播名 `MyCar-BLE`，服务为 Nordic UART Service (NUS)。
  只有**进入蓝牙遥控模式才广播**；退出模式自动断开连接并关闭电机使能。
- 推荐客户端 **Adafruit Bluefruit Connect** → Controller → Control Pad：
  | 按键 | 功能 |
  |---|---|
  | 1 | 使能 ARM（连接后先按它） |
  | 2 | 急停（需重新按 1） |
  | ↑/↓ | 按住前进/后退，松开停 |
  | ←/→ | 按住左转/右转，松开停 |
  | 3/4 | 按住左/右平移，松开停 |
- 兼容 **nRF Toolbox / nRF Connect** 文本命令：`ARM / X / S / F / B / L / R / Q / E / M,f,t,s / ?`
  （文本点动 300ms 命令租约；按键模式 10s 硬上限防丢松开——详见
  `ble_control.c` 与 task3-ble-control/README.md）。
- 电机使能 = `模式激活 && connected && armed`（STBY 总开关）。

## 模块结构

```
main/
├── main.c            主程序：app_menu_init + voice_cmd_init + 心跳打印
├── audio_in.h/.c     USB UAC 麦克风输入：摄像头 USB 三合一（16kHz/16bit/mono）
│                     （备选 audio_in_i2s.c：独立 I2S 数字麦，未参与构建）
├── voice_cmd.h/.c    ESP-SR 识别核心：唤醒 + 命令词表 → voice_evt_t 事件
├── app_menu.h/.c     功能选择状态机：模式切换 / 事件裁决 / 刹停（语音模块的"接口"）
├── bt_remote.h/.c    蓝牙遥控【已实现】NimBLE NUS：广播/生命周期/控制环
├── ble_control.h/.c  蓝牙遥控协议/安全状态机（纯逻辑，移植自 task3，宿主测试通过）
├── expression.h/.c   表情模式【接口占位】ST7735 未接入，TODO 见文件头
├── voice_remote.h/.c 语音遥控【已实现】命令 → motion_drive(f/t/s)
└── motion.h/.c       电机驱动【真实 D24A】三轮全向混控（task2 实车标定增益）
```

## 构建与烧录

Windows（本机 EIM 环境）：双击 `build.bat`（或 `flash.bat [COM口]` 烧录）。

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

> 烧录含两部分：应用固件 `voice-control-task.bin` + 语音模型 `srmodels.bin`（自动烧入 model 分区）。
> 改 `sdkconfig.defaults` 后需删除 `sdkconfig` 重新生成（本分支已提交一份带 BLE 的 sdkconfig）。

## 依赖与配置（sdkconfig.defaults）

- **ESP-SR** v2.5.1：唤醒词 `wn9_hilexin` + 中文命令词 `mn7_cn`（离线，无需训练）
- **BLE**：`CONFIG_BT_NIMBLE_ENABLED=y`（外设角色，1 连接），`CONFIG_FREERTOS_HZ=1000`
- **分区表**：自定义，8MB `model` 分区；**PSRAM** 已启用（ESP-SR 必需）

## 硬件接线

### 电机 D24A（真实驱动，与 task2 同接法）

| D24A | ESP32-S3 | 说明 |
|---|---|---|
| STBY | GPIO1 | 全局使能（高有效，软件总开关） |
| PWMA/B/C | GPIO2/6/9 | LEDC 20kHz 10bit |
| AIN1/2 BIN1/2 CIN1/2 | GPIO4/5 7/8 10/11 | 方向 |

### 麦克风 —— 走摄像头 USB 线（UAC），零接线改动

语音输入用 **JQ-CAM12-720P 三合一摄像头板载麦克风**：音频与视频共用同一根
USB 线（UVC+UAC 复合设备，2026-09-09 修正，同型号已在其他组实现）。
ESP32-S3 USB Host 枚举后以 UAC isoc 捕获 → 48k/32k→16k 降采样进 ESP-SR。

> 因此 **task2 的所有接线（电机、屏幕 13/15/17、编码器 12/14/16、超声波、
> 舵机、摄像头）全部原样保留，一根线都不用动**。唯一要求：摄像头 USB 线
> 必须接着（麦克风供电/数据都在这根线上），跑语音功能时不要拔。
>
> 备选：`audio_in_i2s.c`（独立 I2S 数字麦，如 INMP441）已保留但**未参与构建**，
> 需要时按 `main/CMakeLists.txt` 注释切换；其引脚宏（当前 21/41/42，避开
> 屏幕/编码器）按实际空闲脚调整。

### 表情 TFT（待接入）

ST7735 1.8"（7Pin）：SCK=21 SDI=18 CS=42 DC=41 RST=40（同 feature/qr 分支 Code/QR 工程），
实现时在 `expression.c` 里移植 display 驱动即可（与 12~17 无冲突，可共存）。

## 开发备忘

- **新增语音命令词**：`voice_cmd.h` 枚举 + `voice_cmd.c` 词表各加一项 → 重烧；
  是否在当前模式生效在 `app_menu.c` 裁决，功能模块私有命令由对应模块处理。
- **协议逻辑回归**：`tests/test_control.c`（宿主 gcc 直接编译运行，不依赖 IDF）：
  `gcc -o tc tests/test_control.c main/ble_control.c -I main && ./tc`
- **电机标定**：混控公式/增益（TURN_B 0.68、STRAFE_B 1.80、MOTOR_C 0.90、
  轮速上限 30%）在 `motion.c` 顶部，为 task2 实车标定值，改前先架空验证方向。
- 源码来源：BLE/协议/电机驱动移植自 `feature/ble-phone-control` 分支的
  `task3-ble-control`（该分支含过时的 task2 巡线代码，未并入本工程）。
