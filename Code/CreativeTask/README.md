# CreativeTask —— 攻城槌小车（语音功能选择 + 二维码找目标撞击）

基于 **ESP32-S3** 的创新任务固件：**ESP-SR 离线语音功能选择**（移植自
voice-control-task）+ **攻城槌模式**（UVC 二维码找目标 + 超声波判撞 +
GIF 表情动画 + 摄像头喇叭音效）。

## 功能入口（免唤醒，直接说命令）

| 命令词 | 动作 | 生效模式 |
|---|---|---|
| **攻城槌** | 进入攻城槌模式 | 功能选择 |
| 蓝牙遥控 | 进入蓝牙遥控（BLE，手机 App） | 功能选择 |
| 语音遥控 | 进入语音遥控（前进/后退/左转/右转/停止） | 功能选择 |
| 选择功能 | 刹停并返回功能选择 | 任意模式 |

## 攻城槌模式流程

```
① 目标查找：小幅度脉冲自旋 + 搜索表情动画(IMG_1271) + 循环搜索音效
            UVC 摄像头持续识别二维码（箱贴 "1"/"2"，"1" 为撞击目标）
② 撞击：    识别到 "1" → 撞击表情动画(IMG_1272) + 撞击音效
            对准二维码缓慢前进（x_ratio 比例转向）；
            超声波判撞（<7cm 或前进中距离不再减小）→ 后退 650ms → 再撞击，循环
③ 目标丢失 >2.5s 自动退回查找；任意时刻说"选择功能"退出
```

## 构建 / 烧录

```bash
python _build.py set-target esp32s3     # 首次（Git Bash 用 _build.py 桥接）
python _build.py build
python _build.py -p COM9 -b 115200 flash        # 全量（含 8MB 语音模型，约 8 分钟）
python _build.py -p COM9 app-flash              # 只更应用（日常调试用）
```

> ⚠️ 必须全量 flash 过一次（语音模型 srmodels.bin 烧入 model 分区），
> 之后调试用 `app-flash` 快速迭代（`idf.py app` 只构建不烧录，别踩坑）。

## 资源再生成（tools/）

```bash
# GIF → 调色板+索引RLE 资源（fit=等比缩放居中补底色）
python tools/qr_gif_convert.py <in.gif> main/gif_search.bin  2 128 160 fit
python tools/qr_gif_convert.py <in.gif> main/gif_attack.bin  2 128 160 fit
# 音频 → 16k/48k 单声道 WAV（设备喇叭协商到哪个采样率就用哪套）
ffmpeg -i <in.m4a> -ar 16000 -ac 1 -c:a pcm_s16le main/snd_search_16k.wav
```

## 关键实现备忘

- **UVC+UAC 同线共存**：JQ-CAM12 为 UVC+UAC 复合设备，一根 USB 线同时
  跑视频（quirc 二维码）、麦克风（语音输入）、喇叭（音效）。
  `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=16384` 必须设置（配置描述符 16KB+）。
- **UVC 帧率**：从摄像头描述符选**该分辨率最低帧率档**（本机 640x480 仅 25fps），
  必须与描述符精确匹配（fps 不符 → ESP_ERR_NOT_FOUND）。
- **quirc 裁剪窗**：只喂画面中央 256×192。整帧洪水填充在复杂背景下可超 5s
  （CPU0 空闲任务看门狗饿死），裁剪后常规 ~20ms；并已关 CPU0 空闲 WDT 订阅。
- **JPEG 半分辨率**：`JPEG_IMAGE_SCALE_1_2`（320×240）解码，配合裁剪窗。
- **撞击判定**：超声波 <7cm，或前进中连续 5 次距离不再减小（顶住）。
- **音效协商**：喇叭流参数依次尝试 16k/1ch → 16k/2ch → 48k/1ch → 48k/2ch，
  双声道由软件升采样，WAV 嵌入 16k/48k 两套。

## 模块结构

```
main/
├── main.c           入口：camera_task → app_menu → voice_cmd
├── camera_task.c    UVC 采集 + JPEG/灰度 + quirc 二维码（结果共享）
├── usb_audio.c      UAC 麦克风（RX）+ 喇叭（TX）+ WAV 循环播放
├── voice_cmd.c/h    ESP-SR 免唤醒命令词识别（"攻城槌"等）
├── app_menu.c/h     功能选择状态机（蓝牙遥控/攻城槌/语音遥控）
├── ram_mode.c/h     攻城槌状态机（查找→撞击循环，动画+音效切换）
├── qr_detect.c/h    quirc 封装（灰度→文本+位置）
├── display.c/h      ILI9163 TFT + 双 GIF 动画资源
├── motion.c/h       D24A 三轮混控（task2 标定增益）
├── ultrasonic.c/h   HC-SR04 测距（GPIO39/40）
└── pins.h           引脚分配（电机 1~11 / TFT 13,15,17 / 超声波 39,40）
```
