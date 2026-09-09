# task3：iPhone BLE 遥控小车

独立 ESP-IDF 5.4.4 工程，目标 ESP32-S3。广播名 `MyCar-BLE`，兼容 Nordic UART Service（NUS）。
复用 task2 本地已标定的电机方向与增益，motor.c/h 为独立快照。无需蓝牙模块或 LQ6050V3。
本固件只运行蓝牙遥控；不初始化摄像头、屏幕、超声、编码器或舵机。烧录后屏幕没有画面是正常现象，摄像头/屏幕可保留原接线。

## 推荐：Bluefruit Connect 按住运动、松开停止（新增）

iPhone App Store 安装 **Bluefruit Connect / Bluefruit LE Connect**，开发者 **Adafruit**。
先在 nRF Toolbox 断开小车连接（固件最多一个连接），再打开 Bluefruit，允许蓝牙权限。
依次操作：扫描列表 → MyCar-BLE → Connect → Controller → Control Pad。
此页面自带方向键和 1/2/3/4 按钮，无需编辑按钮或输入文字；功能映射由小车固件定义。
如扫描看不到，请取消仅显示 UART/名称等不合适的过滤条件并检查广播日志；不用在 iOS 设置里先配对。

| 控制板按键 | 功能 |
|---|---|
| 1 | 使能，等同 ARM；本身不移动 |
| 2 | 锁定停止，等同 X；再次按 1 才恢复 |
| ↑ / ↓ | 按住前进 / 后退，松开停止 |
| ← / → | 按住左转 / 右转，松开停止 |
| 3 / 4 | 按住左平移 / 右平移，松开停止 |

烧录新版后先架空：按一下 1，按住 ↑ 两秒再松开，应连续转动约两秒后停。
不需要开启 Accelerometer / Gyro / Quaternion 等传感器数据流；本版只解析按键包，其他二进制包会停止当前运动并报错。
只用一个运动键：同时按多个时最后按下的键生效，松开该键停止，不自动恢复之前仍按着的方向。
App 的按下/松开数据包是 `!B + 按钮编号 + 1或0 + 校验字节`，固件校验完整五字节包。

**两种控制方式有不同的失联边界：** Bluefruit 只发按下和松开事件，没有持续心跳，所以不能再以 300 ms 无命令判定按住结束。
按键模式单次最长 `BUTTON_HOLD_MAX_MS=10000`（10秒），到时停止，松开重新按下才继续；重复的同键按下包不会续期。
收到松开/停止包后在下一个 20 ms 控制周期撤去驱动；蓝牙栈报告断开或重置时撤销使能。
如果松开事件丢失、App 被杀掉或锁屏后连接暂时未断，车可能继续到断线被发现或 10 秒上限；不是 300 ms 停车保证。
因此运行中保持控制页面前台，退出/锁屏前先松手并按 2。不要通过把上限改成无限来规避停止。
若需要长距离连续遥控且要求 300 ms 内失联停车，应使用每 100 ms 发送状态的专用摇杆客户端，现有 M 命令接口可接入。

官方操作与协议：[Control Pad 说明](https://learn.adafruit.com/bluefruit-le-connect/controller)、[连接后进入 Controller](https://learn.adafruit.com/easy-no-solder-bluetooth-controlled-room-lights/bluefruit-app-controls)。

下面的 nRF Toolbox/nRF Connect 文本控制仍兼容，仍是 300 ms 点动。此前把 Toolbox 按钮编辑作为首选不够明确；当前 iOS 版本若没有该入口，直接使用上述预置 Control Pad。
旧 task 的源码和固件构建目录保留，但烧录本工程会替换芯片正在运行的应用；要恢复巡线/推球需重新烧录对应工程。

## 接线与供电

BLE 不占 GPIO。电机 D24A 接线如下（与 task2 相同）：

| D24A | ESP32-S3 |
|---|---|
| STBY | GPIO1 |
| PWMA / PWMB / PWMC | GPIO2 / GPIO6 / GPIO9 |
| AIN1 / AIN2 | GPIO4 / GPIO5 |
| BIN1 / BIN2 | GPIO7 / GPIO8 |
| CIN1 / CIN2 | GPIO10 / GPIO11 |
| GND | ESP32 GND，电机供电地也共地 |

电机仍用原电池/驱动板供电。ESP32 使用开发板允许的稳压供电，禁止把电池原始电压接入 3V3/5V；不要用 USB-TTL 的弱电源输出给电机供电。
无线使用时可以断开 TTL 信号线，但 ESP32 自身必须继续有电。主控天线区域远离银色车壳。

UART0 烧录：USB-TTL TX → GPIO44，RX → GPIO43，GND → GND；TTL 必须是 3.3V 逻辑。
摄像头保留接线时仍走 UART0 烧录，电脑原生 USB 不与摄像头同时占用 GPIO19/20。

## Windows 编译与烧录

在 PowerShell 运行（本机 EIM 环境）：

```powershell
cd D:\Mycar\Code\task3-ble-control
. .\activate-idf.ps1
idf.py build
idf.py -p COM7 -b 115200 flash monitor
```

`COM7` 是例子，请用设备管理器里的实际 USB-TTL COM 号。另一台电脑直接打开其 ESP-IDF 5.4.x 终端并进入本目录，不使用本机专用激活脚本。
首次 build 从 sdkconfig.defaults 选择 esp32s3；无需反复 set-target。若旧构建目标确实错误再执行 `idf.py set-target esp32s3`。

手动下载：电机电源关闭、ESP32 保持供电，按住 BOOT → 短按 EN → 松开 BOOT，执行 flash。
无自动复位线时，烧录结束短按 EN 才运行应用。串口应出现 `Advertising MyCar-BLE: rc=0`。
退出监视按 Ctrl+]。只编译不会让车移动，烧录后默认也不使能电机。
如 NVS 初始化报错，本程序不会自行擦除已有 NVS；先记录错误再处理，不必常规 erase-flash。

## iPhone：保留的 nRF Toolbox 文本点动方式

1. App Store 搜索 **nRF Toolbox**，确认开发者 Nordic Semiconductor；允许蓝牙访问并打开 iPhone 蓝牙。
2. 打开应用中的 **UART / Nordic UART**，在应用内扫描连接 `MyCar-BLE`，不用在系统蓝牙页面先配对。
3. 在 UART 的编辑/配置按钮界面设置下表的 ASCII/文本命令。不同版本界面位置不同；如果没有 UART 按钮界面，使用下节 nRF Connect。
4. 架空小车，接通电机电源；先发送 `ARM`，收到 `ARMED` 后点击方向按钮。
5. 每点一次方向，运动 300 ms 后自动停车。需要继续移动就再次点击。一般按钮不会自动重复，长按不保证连续运动。
6. `S` 立即停止当前运动；`X` 停止并锁定，之后必须重新发 `ARM`。断线/重连也需要重新 ARM。

| 按钮 | 发送文本 | 行为 |
|---|---|---|
| 使能 | ARM | 清零并允许后续运动，本命令本身不运动 |
| 前进 | F | 前进 18% |
| 后退 | B | 后退 18% |
| 左转 | L | 转向 -18% |
| 右转 | R | 转向 +18% |
| 左平移 | Q | 平移 +15% |
| 右平移 | E | 平移 -15% |
| 停止 | S | 三轴停止，保持使能 |
| 锁定停止 | X | 三轴停止并撤销使能 |

串口和 TX 通知返回 OK、ARMED、STOP、ESTOP 或 ERR。方向按钮每次只发送一个大写命令，可有尾部 CR/LF。
本工程沿用 task2 混控的正转向标定；首次必须架空确认方向。如果实车方向不符先停止并检查电机编号/接线。

## iPhone：nRF Connect 通用调试方法

安装 Nordic 的 **nRF Connect for Mobile** → 允许蓝牙 → Scan → `MyCar-BLE` → Connect。
服务 UUID：`6E400001-B5A3-F393-E0A9-E50E24DCCA9E`。

- 手机写入 RX：`6E400002-B5A3-F393-E0A9-E50E24DCCA9E`，选择 Write，文本/UTF-8，发送 `ARM`，再发送 `F`。
- 小车回复 TX：`6E400003-B5A3-F393-E0A9-E50E24DCCA9E`，启用 Notify 接收回复；Read 可读取 ARMED / DISARMED。
- 若界面只有 Hex：ARM=`41 52 4D`，F=`46`，B=`42`，L=`4C`，R=`52`，Q=`51`，E=`45`，S=`53`，X=`58`。
- Write with response 和 Write without response 都支持；调试优先前者。ATT 写入成功只表示传输成功，指令是否接受看回复/串口。

官方资料：[nRF Toolbox](https://www.nordicsemi.com/Products/Development-tools/nRF-Toolbox)、[iOS Toolbox 源码与说明](https://github.com/nordicsemi/IOS-nRF-Toolbox)、[NUS 收发教程](https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-4-bluetooth-le-data-exchange/topic/blefund-lesson-4-exercise-3/?version=v3.4.0)。

## 连续遥控与调速

支持 `M,前进,转向,平移`，例如 `M,18,0,0`、`M,-18,0,0`、`M,0,0,15`、`M,0,0,0`。
每轴范围 -25～25，超范围/格式错误停止当前运动并返回错误。每次 GATT 写入必须是一个完整命令，不支持分包拼接或一包多个命令。
连续摇杆客户端应每 100 ms 重发当前 M 命令，松手发 `M,0,0,0`，离开控制页停止重发。300 ms 没收到有效运动命令即停车。
本次提供的是通过现成 iPhone App 使用的按钮遥控固件，没有另行开发摇杆 App；不要把普通 UART 按钮的长按误认为有自动心跳。
`?` 只查询使能状态，不刷新运动有效期；ARM 会清零，不能作为运动心跳。

调参集中在 `main/control.h`：

| 参数 | 默认 | 作用 |
|---|---|---|
| DRIVE_PCT | 18 | F/B 速度 |
| TURN_PCT | 18 | L/R 速度 |
| STRAFE_PCT | 15 | Q/E 速度 |
| COMMAND_LEASE_MS | 300 | 最后有效 ASCII 运动命令后的最大运动时长 |
| BUTTON_HOLD_MAX_MS | 10000 | Bluefruit 单次长按的最大持续时长 |
| AXIS_MAX_PCT | 25 | M 命令单轴上限 |
| WHEEL_MAX_PCT | 30 | 混控后三轮统一比例限幅 |
| MOTOR_C_GAIN | 0.90 | C 轮校准 |
| TURN_B_GAIN | 0.68 | B 轮转向增益 |
| STRAFE_B_GAIN | 1.80 | B 轮平移增益 |

不是 RPM 闭环，低于静摩擦区仍可能不动。先使用默认 18%，必要时小步调高；不要为了连续行驶大幅增加命令超时。
停止响应在 20 ms 控制循环内执行；断线被蓝牙栈发现有延迟。ASCII 模式由 300 ms 命令有效期兜底，Bluefruit 长按模式由 10秒单次上限兜底。轮子/车体惯性使实际停稳不是零时间。
只有主控制循环写电机；BLE 回调更新受临界区保护的命令，断线清空命令并撤销使能，旧命令不会跨连接恢复。
演示版允许附近任意 BLE 客户端连接，ARM 只是防误动使能，不是身份认证。现场由操作者保管连接，正式扩展可增加认证配对。

## 验收与排错

先架空：未 ARM 发 F 不动；ARM 后 F/B 各点一次方向相反；S 立即停止；X 后 F 不动；重新 ARM 恢复；断开重连后仍不动直到 ARM。
落地：ASCII 每次点动约 0.3 秒；Bluefruit 按住持续运动、松开停止。平移可能因负载需要重新标定 B 增益。锁屏/离开 App 前先按停止；异常退出时按上文对应模式的失联边界验收。
找不到设备：确认已复位运行、广播日志 rc=0、App 蓝牙权限、没有被另一台手机占用（最多一个连接）。
收到 OK 但不动：检查电池、STBY、电机供电、共地；检查是否只点动太短或 PWM 太低；不要改蓝牙协议解决机械起步问题。
只有串口打印而屏幕黑：本工程不运行 TFT，这是预期行为。

主机逻辑测试（有 MinGW GCC 时）：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -I main tests/test_control.c main/control.c -o build/test_control.exe
.\build\test_control.exe
```

测试覆盖使能、命令解析、300 ms 停车、急停、断线重连以及三轮混控限幅；不能代替 iPhone BLE 和实车测试。

2026-09-09 初版验证：ESP-IDF v5.4.4 / esp32s3 构建及 MinGW 主机测试通过；用户已反馈 nRF Toolbox 文本命令可控制实车。
新增 Control Pad 版：ESP-IDF 5.4.4 构建通过，应用 0x76680 字节，1 MiB 应用分区剩余 54%。主机测试通过长按两秒、松开停车、不同键交叠释放、校验错误停车、急停锁定、10秒上限、断线重连及文本控制回归。本次尚未烧录新版或进行 Bluefruit iPhone 实测。
