# 二维码识别表情+跳舞小车 · 工程说明文档

> 基于 ESP32-S3-WROOM-2 + USB 三合一摄像头（UVC）+ D24A 电机驱动 + **1.8寸 TFT 显示屏（LQ_TFT18SPIV33 / ST7735）**。
> 核心目标：**摄像头识别二维码 → 根据码内容在 TFT 屏显示不同表情 + 执行不同舞蹈编排（原地转圈 + 扭动），动作结束自动停下等待下一个二维码。**

---

## 1. 项目概述

### 1.1 目标与验收标准

| # | 验收项 | 判定 |
|---|---|---|
| 1 | 摄像头识别二维码并输出内容 | 串口打印解码文本 |
| 2 | 不同二维码 → 不同"表情 + 舞蹈" | 贴 `1` 显示笑脸+转圈；贴 `2` 显示哭脸+扭动；贴 `3` 显示惊讶+混合 |
| 3 | 防重：同一码持续出现不重复触发 | 贴同一个码不动，只跳一次 |
| 4 | 舞蹈自动结束回待命 | 到点 `motion_stop` + 屏回待机，等待下一个码 |
| 5 | TFT 显示表情 | 屏上显示可辨识的表情（如笑脸/哭脸/惊讶/眨眼） |

### 1.2 术语

- **QR 码（二维码）**：内容为指令文本，本工程用作"表情+舞蹈"索引。
- **表情（Face）**：TFT 屏上绘制的卡通表情（圆脸 + 眼睛 + 嘴型），不同码对应不同表情。
- **舞蹈（Dance）**：电机动作编排，由**原地转圈**与**左右扭动**交替组成。
- **歌舞组合（Routine）**：一个二维码内容对应的 `{表情编号, 舞蹈参数}`。

---

## 2. 硬件说明

### 2.1 器件清单

| 器件 | 型号/说明 | 状态 |
|---|---|---|
| 主控 | ESP32-S3-DevKitC-1（WROOM-2 自带 PSRAM） | ✅ 已装车 |
| 摄像头 | JQ-CAM12-720P USB 三合一（1280×720，UVC，4Pin） | ✅ 已装车 |
| 电机驱动 | D24A 裸功率板（TB6612FNG，STBY+3×PWM+6×方向） | ✅ 已装车 |
| 电机 | 3× LQ25R8V6（带霍尔编码器，本工程不使用编码器） | ✅ 已装车 |
| 底盘 | 三轮 120° 全向轮（Kiwi） | ✅ 已装车 |
| **显示屏** | **LQ_TFT18SPIV33**：1.8寸 TFT，**128×160，ST7735，4线 SPI，3.3V** | ⚠️ **需外接** |
| 调试口 | 外部 USB 转 TTL（接 UART0） | ✅ |

### 2.2 接线表（本工程）

| 信号 | GPIO | 说明 |
|---|---|---|
| 摄像头 USB Host | **GPIO19 / GPIO20**（USB_D-/USB_D+） | 只接摄像头；下载/监视不能走板载 USB |
| 电机 STBY（全局使能） | GPIO1 | 高有效 |
| 电机 PWM | GPIO2(A) / GPIO6(B) / GPIO9(C) | LEDC，20kHz，10bit |
| 电机方向 | GPIO4/5(AIN1/2)、GPIO7/8(BIN1/2)、GPIO10/11(CIN1/2) | 普通输出 |
| **TFT SCK（时钟）** | **GPIO13** | SPI 时钟 |
| **TFT SDI（数据/MOSI）** | **GPIO15** | SPI 数据（只写，MISO 不接） |
| **TFT D/C（数据/命令）** | **GPIO17** | 数据/命令选择 |
| **TFT CS** | **硬件接 GND** | 总线上只有本屏，不占 GPIO |
| **TFT RST** | **接 ESP32 EN** | 随板上电复位，不占 GPIO |
| TFT VCC / GND | 3.3V / GND | 供电 |
| 调试串口 UART0 | GPIO43（TX）/ GPIO44（RX） | 外部 USB 转 TTL |
| 编码器（已接，本工程不用） | GPIO12/14/16（A 相） | task1/2 共用 |
| 超声波（已接，本工程不用） | GPIO39(TRIG) / GPIO40(ECHO) | task1/2 共用 |

> 引脚与 **WJHcd/esp32-car-course-design 仓库（task1_ver2.0 / task2-camera-line）**
> 的实车接线对齐（2026-09-08）；此前 SCK=21/SDI=18/CS=42/DC=41/RST=40 与
> 车上实际接线不符，是 TFT 白屏的直接原因之一。

### 2.3 引脚冲突检查（关键）

| 资源 | GPIO | 是否占用 |
|---|---|---|
| 摄像头 USB | 19, 20 | ✅ 占用，不可他用 |
| UART0 console | 43, 44 | ✅ 占用 |
| 板载 WS2812 | 38 | 避开（不可用） |
| Flash/PSRAM | 26~37 | 不可用 |
| Strapping | 0, 3, 45, 46 | 上电敏感，避开 |
| 编码器（其他任务） | 12, 14, 16 | 已接线，本工程不用但不冲突 |
| 超声波（其他任务） | 39, 40 | 已接线，本工程不用但不冲突 |
| 电机 | 1, 2, 4, 5, 6, 7, 8, 9, 10, 11 | 本工程占用 |
| **TFT 显示屏** | **13, 15, 17**（CS 接地、RST 接 EN 不占脚） | 本工程占用 |

> ⚠️ 摄像头占用原生 USB（GPIO19/20）做 USB Host，**下载与监视必须走外部 USB 转 TTL 接 UART0（GPIO43/44）**。

---

## 3. 系统架构

### 3.1 数据流

```
USB摄像头 (MJPEG)
   │  UVC (usb_host_uvc)
   ▼
frame_callback  —— 仅拷贝入队（FreeRTOS 队列，深度 2），不做耗时操作
   ▼
processing_task（图像处理 + 表情/舞蹈状态机）
   ├─ 1. esp_jpeg 软件解码 MJPEG → RGB888
   ├─ 2. RGB888 → 8bit 灰度（quirc 需要）
   ├─ 3. quirc 解码（节流：每 ~300ms 一次）→ 文本
   ├─ 4. qr_dance 状态机：内容变化 → 选表情+舞蹈 → 执行
   │        ├── display ：TFT 屏绘制表情
   │        └── motion  ：电机转圈/扭动
   └─ 5. 到点 → motion_stop + 屏回待机 → 回 WAIT_QR
```

### 3.2 FreeRTOS 任务划分

| 任务 | 优先级 | 栈 | 职责 |
|---|---|---|---|
| `usb_events` | 2 | 4096 | USB Host 库事件处理（设备枚举/释放） |
| `img_proc`（processing_task） | 5 | **32KB**（quirc 解码需要较大栈） | 收帧→解码→灰度→QR→状态机→驱动 display/motion |

### 3.3 模块划分

```
Code/QR/
├── CMakeLists.txt          顶层工程文件
├── sdkconfig.defaults      PSRAM / USB / Flash 默认配置
└── main/
    ├── CMakeLists.txt      组件注册（依赖 usb / esp_jpeg / quirc / esp_lcd / esp_timer）
    ├── idf_component.yml   组件依赖（usb_host_uvc / espressif/esp_jpeg / espressif/quirc）
    ├── main.c              UVC 采集 + 解码 + 灰度 + 状态机主流程
    ├── pins.h              引脚定义（电机 / 显示屏 / 摄像头）
    ├── qr_detect.h/.c      quirc 封装：灰度图 → 解码文本
    ├── qr_dance.h/.c       表情/舞蹈状态机 + 组合表
    ├── display.h/.c        TFT(ST7735) 驱动 + 表情绘制
    ├── motion.h/.c         电机动作：原地旋转 / 扭动 / 停
    └── motor.h/.c          电机底层驱动（移植自 task1）
```

---

## 4. 模块详细设计

### 4.1 摄像头采集与解码（main.c）

与 `Code/camera` 工程同构，差异点：

- **分辨率**：优先协商 `640x480@15`（二维码需要足够像素）；失败降级 `320x240@30`。
- **帧缓冲**：`MJPEG_BUF_SIZE` 由 64KB 提到 **128KB**（640×480 MJPEG 单帧可能更大）。
- **RGB 缓冲**：`FRAME_WIDTH*HEIGHT*3`（640×480 ≈ 921KB，PSRAM 分配）。
- **灰度缓冲**：`FRAME_WIDTH*HEIGHT`（640×480 ≈ 307KB，PSRAM 分配）。
- **处理任务栈**：`8192 → 32768`（quirc 解码需要）。

```c
#define FRAME_WIDTH    640
#define FRAME_HEIGHT   480
#define FRAME_FPS      15
#define FRAME_FORMAT   UVC_FRAME_FORMAT_MJPEG
#define MJPEG_BUF_SIZE (128 * 1024)
#define RGB_BUF_SIZE   (FRAME_WIDTH * FRAME_HEIGHT * 3)
#define GRAY_BUF_SIZE  (FRAME_WIDTH * FRAME_HEIGHT)
#define FRAME_QUEUE_LEN 2
```

### 4.2 二维码识别（qr_detect）

**依赖**：`espressif/quirc`（官方组件，`idf.py add-dependency "espressif/quirc^1.2.0"`）。

**接口**：

```c
typedef struct {
    bool   detected;      /* 本帧是否解出至少一个码 */
    char   text[32];      /* 解码文本（取第一个码） */
    float  x_ratio;       /* 码中心 X [0,1] */
    float  y_ratio;       /* 码中心 Y [0,1] */
    int    count;         /* 解出的码数量 */
} qr_result_t;

void qr_detect_init(int width, int height, int max_codes);
bool qr_detect_process_gray(const uint8_t *gray, qr_result_t *out);
```

**实现要点**：

1. **灰度转换**：`gray = (R + G + B) / 3`（在 main.c 内联做）。
2. **quirc 流程**：`quirc_new` → `quirc_resize(w,h)` → `quirc_begin` 取缓冲 → `memcpy` 灰度 → `quirc_end` → `quirc_count` → 逐个 `quirc_extract` + `quirc_decode`，取 UTF8 类型 payload。
3. **ECC 失败重试**：`quirc_decode` 返回 `QUIRC_ERROR_DATA_ECC` 时调用 `quirc_flip(q, i)` 翻转重试（反相码兜底）。
4. **极性**：quirc 认**黑码白底**，正常打印的二维码即可。
5. **节流**：`qr_detect_process_gray` 耗时几十~几百 ms，**调用方必须节流**（每 ~300ms 解一次）。

### 4.3 表情/舞蹈状态机（qr_dance）

**接口**：

```c
typedef struct {
    int  face_id;         /* 表情编号（display 表索引） */
    /* 舞蹈序列：转圈 + 扭动交替 */
    int  spin_speed;      /* 转圈速度 %（正=正转方向，见 motion 标定） */
    int  spin_ms;         /* 转圈时长 ms */
    int  spin_dir;        /* +1 / -1（顺/逆） */
    int  wiggle_speed;    /* 扭动速度 % */
    int  wiggle_ms;       /* 单次扭动时长 ms */
    int  wiggle_repeat;   /* 扭动来回次数 */
    int  sequence_loop;   /* "转圈+扭动"整组重复次数 */
} dance_routine_t;

void  qr_dance_init(void);
void  qr_dance_update(const qr_result_t *qr);  /* 每节流周期调用 */
bool  qr_dance_is_dancing(void);
dance_routine_t qr_dance_routine_for(const char *content); /* 内容→组合查表 */
```

**状态**：

```
WAIT_QR ──(识别到新内容，查表命中)──▶ DANCE ──(时长到)──▶ WAIT_QR
```

**行为**：

1. **WAIT_QR**：等待 `qr_result.detected`。识别到内容后与"上次触发内容"比较：**不同**→查表→进入 DANCE；**相同**→忽略（防重/防抖）。
2. **DANCE**：记录开始时间（`esp_timer`），依次执行：
   - `display_face(face_id)`：TFT 显示对应表情；
   - `motion` 执行舞蹈序列：**转圈 → 扭动 → 转圈 → …**（按 `sequence_loop` 交替）；
   - 到总时长 → `motion_stop()` + `display_idle()` → 回 WAIT_QR。
3. **未命中**（内容不在表内）：打印提示，忽略。

**组合表**（示例，可现场改）：

| QR 内容 | face_id | spin_speed | spin_ms | spin_dir | wiggle_speed | wiggle_ms | wiggle_repeat | loop |
|---|---|---|---|---|---|---|---|---|
| `1` | 0（笑脸） | 40 | 2000 | +1 | 30 | 300 | 6 | 2 |
| `2` | 1（哭脸） | 40 | 1500 | -1 | 30 | 400 | 4 | 2 |
| `3` | 2（惊讶） | 50 | 2500 | +1 | 40 | 250 | 8 | 1 |

### 4.4 TFT 显示与表情（display）

**接口**：

```c
void display_init(void);          /* SPI + ST7735 面板初始化 */
void display_face(int face_id);   /* 绘制指定表情（彩色背景 + 卡通脸） */
void display_idle(void);          /* 待机画面（纯色背景或"等待二维码"文字） */
```

**实现要点**：

1. **面板实为 ILI9163**（非 ST7735）：按 ST7735 序列配置电源参数（B1/C0/C5）
   不匹配，表现为**背光亮但全白**（已在实车复现并定位）。初始化序列取自
   龙邱官方 LQ_TFT18（spi_TFT18.c）的 ILI9163 版本，与 task2-camera-line
   的 `tft_st7735.c` 一致（实车验证过）。
2. **驱动**：自绘 SPI 底层（`spi_master` polling 事务 60 字节分块 + DC GPIO），
   不用 esp_lcd；SPI 时钟 1MHz 起步（杜邦线友好，稳定后可升），SPI mode 0，
   CS 硬件接地（`spics_io_num=-1`），RST 接 EN（上电自动复位）。
3. **颜色**：实测该屏 R/B 物理互换（MADCTL BGR 位已置仍互换），发送前交换
   RGB565 高低 5 位，上层颜色语义才正确。
4. **表情绘制**：帧缓冲图元（填色圆=脸、填色圆=眼睛、月牙/圆=嘴型），不同
   `face_id` 改眼睛/嘴型/背景色：笑脸、哭脸、惊讶、眨眼、爱心眼。
5. **内存**：帧缓冲 128×160×2 ≈ 40KB，优先内部 DMA 内存，不足退回 PSRAM
   （发送经内部 staging 缓冲分块，两种位置均可工作）。

### 4.5 电机动作（motion / motor）

**motor（移植自 task1）**：`motor_init()` / `motor_set_speed(id, percent)` / `motor_stop_all()`，LEDC 20kHz + 方向 GPIO + STBY 控制，换向 200us 滑行保护。引脚见 §2.2。

**motion**：

```c
void motion_init(void);
void motion_rotate(int speed);   /* 原地旋转 speed%（带符号），持续由状态机控制 */
void motion_stop(void);
```

**三轮运动学**（对齐 task1_ver2.0 walking.c 实车标定）：

| 动作 | 电机命令 |
|---|---|
| **原地旋转** | A=+turn、C=+turn、B=-0.68×turn（B 电机正转方向约定与 A/C 相反，0.68 为其实测增益 TURN_B_GAIN） |
| **扭动** | 左右小幅原地旋转交替（`motion_rotate(+v)` ↔ `motion_rotate(-v)` 交替 `repeat` 次） |

> ⚠️ **首次实车需标定**：确认"原地旋转"方向（`motion_rotate(+speed)` 顺时针/逆时针），如与预期相反把 `motion.c` 的 `MOTION_ROTATE_SIGN` 改成 -1。

### 4.6 主流程（main.c）

```
app_main
 ├─ usb_host_init()            USB Host 库
 ├─ xQueueCreate(frame_queue, 2)
 ├─ xTaskCreatePinnedToCore(img_proc, 32KB, prio5, core0)
 ├─ uvc_init / uvc_find / uvc_open
 ├─ 协商 640x480@15（失败 320x240@30）
 └─ uvc_start_streaming（frame_callback 每帧入队）

img_proc（循环）
 ├─ 收帧
 ├─ esp_jpeg_decode → RGB888
 ├─ RGB888 → 灰度
 ├─ 节流：距上次解码 <300ms → 跳过本次解码
 ├─ qr_detect_process_gray → qr
 └─ qr_dance_update(&qr)
```

---

## 5. 关键参数汇总（可现场标定）

| 参数 | 位置 | 默认值 |
|---|---|---|
| 分辨率 / 帧率 | main.c | 640×480@15（降级 320×240@30） |
| MJPEG 帧缓冲 | main.c | 128KB |
| 解码节流间隔 | main.c | 300ms |
| TFT SPI 引脚 | pins.h | SCK=13, SDI=15, D/C=17（CS 接地、RST 接 EN） |
| TFT SPI 时钟 | display.c | 1MHz（杜邦线友好，稳定后可升） |
| 面板控制器 | display.c | **ILI9163**（非 ST7735，初始化序列见 §4.4） |
| 舞蹈速度范围 | qr_dance.h | 0~100% |
| 舞蹈总时长 | qr_dance.h 组合表 | 8~15s |
| 表情表 | display.c | 笑脸/哭脸/惊讶/眨眼（可加） |

---

## 6. 构建、烧录与调试

```bash
# 工程在纯英文路径（如本仓库 Code\QR）
# 首次：设置目标芯片（USB Host 需要）
idf.py set-target esp32s3

# 添加组件依赖（quirc）
idf.py add-dependency "espressif/quirc^1.2.0"

# 编译（esp_lcd 为 ESP-IDF 内置组件，无需额外下载）
idf.py build

# 烧录（摄像头占板载 USB，必须用外部 TTL 接 UART0 的 COM 口）
idf.py -p COMx flash

# 监视
idf.py -p COMx monitor
```

> ⚠️ **下载/监视必须走外部 USB 转 TTL → UART0（GPIO43/44）**；板载 USB 已被摄像头占用（GPIO19/20）。

---

## 7. 测试与验收

### 7.1 模块自检（TEST_MODE 编译开关）

| TEST_MODE | 内容 | 目的 |
|---|---|---|
| 0 | 综合模式：二维码表情+跳舞 | 正式运行 |
| 1 | 纯解码测试 | 打印解码文本，验证摄像头+quirc |
| 2 | 显示屏测试 | 依次显示各表情，验证 TFT 接线/驱动/偏移 |
| 3 | 电机动作测试 | 依次转圈/扭动/停，验证 motion 方向与符号 |

### 7.2 实车验收流程

1. 贴 `1` 码在摄像头正前方约 30~50cm → 屏显示笑脸 + 转圈 → 自动停。
2. 换贴 `2` 码 → 屏显示哭脸 + 扭动为主 → 自动停。
3. 换贴 `3` 码 → 屏显示惊讶 + 混合编排 → 自动停。
4. 贴回 `1`（内容变化）→ 重新触发；同一码持续贴着不动 → 不重复触发。
5. 串口确认：`QR: "1"` → `DANCE face=0 ...` → `DANCE 结束`。

---

## 8. 进度

- [x] 工程说明文档
- [x] 工程骨架（CMakeLists / sdkconfig.defaults / main）
- [x] qr_detect（quirc 接入）
- [x] display（TFT ST7735 + 表情）
- [x] motor / motion（电机动作）
- [x] qr_dance 状态机
- [x] main.c 综合流程
- [x] 代码审查加固（2026-09-07）：解码节流前置省 CPU / 摄像头停滞时舞蹈仍到点停车 / TFT 帧缓冲优先内部 DMA / 解码尺寸边界校验 / quirc 失败可见化
- [x] 引脚与 WJHcd 仓库实车接线对齐（2026-09-08）：TFT 改 13/15/17（CS 接地、RST 接 EN）+ 面板更正为 ILI9163 自绘驱动 + 旋转运动学同步 task1_ver2.0 标定
- [ ] 实车联调（解码 / 电机方向）
