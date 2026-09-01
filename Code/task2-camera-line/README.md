# task2 摄像头巡线（ESP32-S3 + USB UVC 摄像头）

任务二：赛道与任务一相同，巡线改用 USB 摄像头（UVC）+ 黑线质心检测，避障仍用 HC-SR04。
接线见 [接线说明.md](接线说明.md)。

## 快速开始

| 操作 | 命令/方法 |
| --- | --- |
| 编译 | 双击 `build-fast.bat`（增量，**不重生成配置**） |
| 重新生成配置 | `build.bat`（内含 `set-target`，会按 `sdkconfig.defaults` 重建 sdkconfig） |
| 烧录 | 板子进下载模式（**按住 BOOT → 短按 EN → 松开**），然后在 `build/` 下执行下方 esptool 命令 |
| 监控 | 同一根 TTL 线，串口 115200（推荐 `idf.py monitor` 或任意串口工具） |

烧录命令（`build` 目录下，COM 口按实际改）：

```
python -m esptool --chip esp32s3 -p COM8 -b 115200 --before no_reset --after hard_reset write_flash @flash_args
```

运行模式在 `main.c` 的 `TASK2_MODE` 切换：`0` = 综合跑车（当前，摄像头巡线 + 超声避障 + T 线停车）、
`7` = 摄像头纯巡线实车测试、`1` = 静态巡线识别（不动电机）、`5` = TFT 预览、`6` = ASCII 预览，详见 `main.c` 头部注释。

## 摄像头验证现状（2026-08-28）

- UVC 枚举、480x320 MJPEG 采集、JPEG 解码（约 213ms/帧）、虚拟红外扫描带检测全部稳定运行。
- 串口每秒打印一帧 64x24 ASCII 字符画：`#` = 灰度低于 `CAM_BIN_THRESHOLD` 判黑的像素
  （与检测算法同阈值，可直接用于校准阈值），其余字符按亮度渐变。
- 每帧后附 `状态: 见线/丢线 err=… rows=…`；`perf:` 行打印解码耗时。

## 踩坑记录（重要心得，改这块之前先读）

### 1. PSRAM 必须开启，且只能改 `sdkconfig.defaults`

`camera_line.c` 的 MJPEG/RGB 缓冲全部从 PSRAM 分配。工程最初 `CONFIG_SPIRAM is not set`，
导致 `no mem for rgb buffer` —— 这曾是最早"摄像头完全不工作"的第一层原因。
板载 ESP32-S3 为 16MB Octal PSRAM（AP_1v8），必须 `CONFIG_SPIRAM_MODE_OCT=y`
（注意 IDF 5.4.4 里符号名是 `_OCT` 不是 `_OCTAL`，写错会被静默忽略并回退 QUAD）。

⚠️ **直接改 `sdkconfig` 会被冲掉**：`build.bat` 里的 `set-target` 会重新生成 sdkconfig。
所有自定义配置必须写进 `sdkconfig.defaults`；日常编译用 `build-fast.bat`（跳过 set-target）。

### 2. USB 控制传输缓冲要加大（4096）

UVC 摄像头的配置描述符普遍超过默认的 256 字节，会报
`ENUM: Configuration descriptor larger than control transfer max length` 枚举失败。
`CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=4096`（与官方 uvc 示例一致）。

### 3. usb_host_uvc 必须用 2.x，1.x（libuvc 封装）有崩溃 bug

1.0.3 在 `uvc_find_device → libusb_close → usb_host_device_close` 路径上断言崩溃
（`usbh_dev_close: num_ctrl_xfers_inflight == 0`），表现为反复重启死循环，无法规避。
已升级 `usb_host_uvc: ^2.5.2` 并按新 API（`uvc_host_*`）重写采集层：

- 驱动事件 `DEVICE_CONNECTED` 里枚举摄像头支持的格式（`uvc_host_get_frame_list`）再开流；
- 设备断开/传输错误只记日志不 abort（2.5.2 的修复点）；
- 帧回调里拷贝数据后立即 `return true` 归还缓冲，不做耗时处理。

**这颗 JQ-CAM12-720P 不支持 320x240**（支持 1280x720/800x480/640x480/480x320/480x854），
旧代码硬协商 320x240 永远不会成功。选档优先级：320x240 → 480x320 → 640x480 → 任意 MJPEG。

### 4. 解码是大户：限频 + 绑核，否则饿死系统

640x480 档 JPEG 解码实测 200~300ms/帧，若不限帧率会把单核占满，
IDLE 任务饿死触发 task watchdog（每 5s 告警）、低优先级任务（如 app_main 的打印）被饿死。
当前策略：

- `FRAME_PROC_INTERVAL_MS=150`：采集侧理论上限约 6.7fps（实际受 JPEG 解码耗时限制），队列满时丢旧帧避免控制过期画面；
- 解码任务 `img_proc` 固定在 **CPU1**，CPU0 留给 USB 主机和主任务；
- 摄像头分辨率 > 320x240 时解码输出按 1/2 缩放（480x320 → 240x160），
  缩放档不支持时逐级回退；黑线检测的窗口/阈值按实际分辨率自适应。

### 5. 烧录通道：摄像头占了 GPIO19/20（板载 USB 口）

- 板载 USB 口的数据脚就是 GPIO19/20，摄像头接着时**烧录必失败**（写完 bootloader 掉线、
  之后连不上），且即使拔下摄像头烧完，接回后串口也没输出可看——不要走这条路。
- 正确方式：**USB 转 TTL 接 UART0**（TX→GPIO44、RX→GPIO43、共地，交叉接线），
  烧录和监控共用这一根线，摄像头永远不用拔。
- 手动进下载模式：按住 BOOT → 短按 EN → 松开（TTL 没接 DTR/RTS，esptool 无法自动复位，
  用 `--before no_reset`，配合重试循环在按键后几秒内自动接上）。
- USB-TTL 芯片要有驱动：CH340 免驱概率大；**CP2102 需要 Silicon Labs 驱动**
  （设备管理器出现在"其他设备"就是没驱动，装好后才有 COM 口）。
  驱动包：[CP210x Universal Windows Driver](https://www.silabs.com/documents/public/software/CP210x_Universal_Windows_Driver.zip)，
  管理员权限 `pnputil /add-driver silabser.inf /install`。

### 6. 调试方法：串口 ASCII 预览代替屏幕

`ascii_preview.c` 把解码后的画面降采样成 64x24 字符画打印到串口（`#`=判黑像素），
不开屏幕、不动电机即可确认"摄像头出图 → 解码正确 → 阈值合理"整条链路。
现场标定 `CAM_BIN_THRESHOLD` 时直接看字符画上 `#` 是否贴合实际黑线。

## 注意事项汇总

- 改配置 → 写 `sdkconfig.defaults` → 用 `build.bat` 重建一次；日常编译用 `build-fast.bat`。
- 摄像头供电务必来自电池稳压 5V（同电机源），从电脑 USB 取电跑车会供电不稳。
- 烧录时若 esptool 报 `No serial data received`：九成是没进下载模式（重做 BOOT+EN），
  其次检查 TX/RX 是否交叉、TTL 驱动是否装好（设备管理器里有没有 COM 口）。
- MODE=7 启动前需要连续 3 帧稳定见线；运行中复用 `walking.c` 的 RUN / CORNER_SEARCH / LINE_SEARCH，但始终传 `ULTRASONIC_INVALID`，所以不会触发避障。
- 当前摄像头检测仿真任务一红外阵列：只扫描画面中一条宽 60%、高 8% 的水平带。弯角处目标线短暂离开扫描带是正常现象，控制器会停止前进并按最后侧别原地旋转，重新见线后恢复。
- 综合模式避障后向右平移时，只接受进入画面中央误差范围的目标线；首帧立即停车，静止确认第二帧后恢复巡线并武装 T 形终点检测。
- T 形终点首帧宽黑候选立即停车，静止确认第二帧后进入永久停车，避免边前进边确认而驶过横杆。
- 摄像头超过 700ms 没有新帧会禁用电机；恢复连续稳定见线后自动重新使能。

## 目录速览

```
main/
  camera_line.c/.h   UVC 采集(uvc_host 2.x) + JPEG 解码 + 虚拟红外扫描带 + 稳定性判定
  ascii_preview.c/.h 串口 ASCII 字符画预览（模式 6，验证摄像头用）
  camera_follow_test.c/.h 摄像头纯巡线测试（模式 7，无超声/避障，含弯道恢复状态）
  walking.c/.h       行走控制（沿用任务一，误差源改为摄像头）
  ultrasonic.c/.h    HC-SR04 超声避障（沿用任务一）
  motor.c / encoder.c / tft_st7735.c   底层驱动（沿用任务一）
  main.c             TASK2_MODE 模式选择
build-fast.bat       日常增量编译（不动配置）
build.bat            全量编译（set-target，会按 defaults 重建 sdkconfig）
sdkconfig.defaults   关键配置：SPIRAM(OCT) / USB 控制传输 4096 / FREERTOS_HZ=1000
接线说明.md          完整接线表
```
