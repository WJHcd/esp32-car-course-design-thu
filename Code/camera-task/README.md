# 摄像头任务（科目三：找球）工程

> 基于 ESP32-S3-WROOM-2 + USB 三合一摄像头（UVC）+ esp_driver_jpeg 硬件解码。
> 核心目标：识别画面中的红/白色小球，输出"球是否在画面中 + 方位信息"接口。

---

## 1. 工程结构

```
camera-task/
├── CMakeLists.txt          # 顶层工程文件
├── sdkconfig.defaults      # PSRAM / USB / Flash 等默认配置
└── main/
    ├── CMakeLists.txt      # 组件注册（依赖 usb/esp_driver_jpeg/esp_psram 等）
    ├── idf_component.yml   # 组件依赖（usb_host_uvc）
    ├── main.c              # UVC 采集 + JPEG 解码 + 找球主流程
    ├── ball_detect.h       # 找球接口（核心：球是否在画面中 + 方位）
    └── ball_detect.c       # 颜色识别实现（红/白球）
```

## 2. 硬件要求

| 项 | 说明 |
| --- | --- |
| 主控 | ESP32-S3-WROOM-2（自带 PSRAM，需在配置中启用） |
| 摄像头 | USB 三合一摄像头（JQ-CAM12-720P，4Pin USB：5V/GND/D-/D+，UVC 协议） |
| 调试口 | ⚠️ 摄像头占用 GPIO19/20（USB Host），**下载/监视需改用外部 USB 转 TTL 接 UART0（GPIO43/44）** |

## 3. 关键设计

### 数据流

```
USB摄像头(MJPEG)
      │  UVC (usb_host_uvc)
      ▼
frame_callback（仅入队，不做耗时操作）
      │  FreeRTOS 队列
      ▼
处理任务：esp_driver_jpeg 硬件解码 MJPEG → RGB888
      │
      ▼
ball_detect_process()：颜色阈值识别红/白球 → 质心/面积/包围盒
      │
      ▼
ball_detect_result_t（是否检测到 + x/y 归一化方位 + 面积估距）
```

### 找球接口（ball_detect.h）

```c
typedef struct {
    bool detected;        // 画面中是否检测到球
    ball_color_t color;   // BALL_COLOR_RED / BALL_COLOR_WHITE
    float x_ratio;        // 球心 X 归一化 [0,1]，0.5 = 画面正中（供转向）
    float y_ratio;        // 球心 Y 归一化 [0,1]，0 = 画面顶部
    float area_ratio;     // 球像素面积 / 画面面积 [0,1]，越大球越近
    uint16_t width_px;    // 球包围盒宽度（像素）
    uint16_t height_px;   // 球包围盒高度（像素）
    uint32_t pixel_count; // 球像素总数
} ball_detect_result_t;
```

- `detected` + `color` → 球是否在画面中、什么颜色
- `x_ratio` → 左右方位（供差速转向）
- `area_ratio`/`pixel_count` → 远近估计（供前进/停止判断）

### 颜色标定

颜色阈值集中在 `ball_detect.h` 顶部宏定义，现场按实际球的颜色标定：

| 宏 | 含义 | 初始值 |
| --- | --- | --- |
| `BALL_RED_R_MIN` | 红色：R 通道最小值 | 100 |
| `BALL_RED_RG_DELTA` | 红色：R 需比 G 高出的差值 | 40 |
| `BALL_RED_RB_DELTA` | 红色：R 需比 B 高出的差值 | 40 |
| `BALL_WHITE_MIN` | 白色：三通道最低值 | 160 |
| `BALL_WHITE_TOL` | 白色：三通道最大差（接近白） | 30 |
| `BALL_MIN_PIXELS` | 最少像素数（滤除噪声） | 20 |
| `BALL_MAX_AREA_RATIO` | 最大面积占比（滤除全屏误判） | 0.6 |

## 4. 构建与烧录

```bash
# 首次：设置目标芯片（必须，USB Host 需要）
idf.py set-target esp32s3

# 编译
idf.py build

# 烧录（USB 转 TTL 接 UART0 时用对应 COM 口）
idf.py -p COMx flash

# 监视（通过 TTL 转 UART0）
idf.py -p COMx monitor
```

> ⚠️ 摄像头占用了板载 USB（GPIO19/20），所以下载/监视必须走外部 USB 转 TTL → UART0（GPIO43/44）。

## 5. 当前进度

- [x] 工程骨架
- [x] 找球接口 ball_detect（红/白球颜色识别）
- [ ] UVC 采集主流程（main.c）
- [ ] 实际硬件联调（帧率、阈值标定）
- [ ] 与运动学控制对接
