#include "display.h"
#include "pins.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_log.h"
#include <inttypes.h>
#include <math.h>
#include <string.h>

/* ============================================================
 * TFT 显示屏实现（LQ_TFT18SPIV33，128x160，RGB565）
 *
 * ⚠️ 面板实为 ILI9163 控制器（非 ST7735）：按 ST7735 序列配置
 *    电源参数（B1/C0/C5）不匹配，表现为背光亮但全白。
 *    初始化序列取自龙邱官方 LQ_TFT18（spi_TFT18.c）的 ILI9163
 *    版本，与 WJHcd 仓库 task2-camera-line 的 tft_st7735.c 一致，
 *    为该屏实车验证过的序列。
 *
 * 引脚按车上实物接线（见 pins.h）：SCK=13 / SDI=15 / DC=17，
 * CS 硬件接地（不占 GPIO），RST 接 EN（上电自动复位）。
 *
 * 绘制方式：先把整屏画进帧缓冲，最后经 tft_set_window 一次性
 * 推出。帧缓冲优先内部 DMA 内存，不足退回 PSRAM。
 *
 * ⚠️ 颜色：实测该屏 R/B 物理互换（MADCTL BGR 位已置仍互换），
 *    push_fb 发送前交换 RGB565 高低 5 位，上层颜色语义才正确。
 * ============================================================ */

#define TFT_W  128
#define TFT_H  160

#define TFT_HOST         SPI2_HOST
/* 实测：1/2/4MHz 稳定。8MHz 全屏推屏 ~41ms，约 14fps（接近素材 16fps 采样上限）。
 * 若撕裂：回退 5MHz（80/5=16 整分频）→ 4MHz。 */
#define TFT_SPI_CLOCK_HZ (8 * 1000 * 1000)
/* polling 事务安全分块（字节）：无 DMA 缓冲时单事务上限约 64 字节，
 * 留余量取 60；tft_write 内部按此分块。 */
#define TFT_SPI_CHUNK_MAX 60u

static const char *TAG = "display";
static spi_device_handle_t s_spi = NULL;
static uint16_t *s_fb = NULL;   /* 128x160 RGB565 帧缓冲 */

/* ---- RGB565 颜色（语义值，发送时由 push_fb 做 R/B 互换） ---- */
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_YELLOW  0xFFE0
#define C_RED     0xF800
#define C_BLUE    0x001F
#define C_GREEN   0x07E0
#define C_LBLUE   0x7DFF   /* 淡蓝 */
#define C_BROWN   0x8800   /* 深棕（描边） */

/* ---------------- SPI 底层（移植自 task2-camera-line tft_st7735.c） ---------------- */

static void tft_write(bool is_data, const void *data, size_t length)
{
    if (length == 0) {
        return;
    }
    ESP_ERROR_CHECK(gpio_set_level(TFT_PIN_DC, is_data ? 1 : 0));
    const uint8_t *bytes = (const uint8_t *)data;
    while (length > 0) {
        const size_t chunk = length > TFT_SPI_CHUNK_MAX ? TFT_SPI_CHUNK_MAX : length;
        spi_transaction_t transaction = {
            .length = chunk * 8,
            .tx_buffer = bytes,
        };
        esp_err_t err = spi_device_polling_transmit(s_spi, &transaction);
        if (err != ESP_OK) {
            /* 写屏失败只影响显示，不允许拖垮整车控制：跳过本次剩余数据 */
            ESP_LOGE(TAG, "SPI 写入失败: %s", esp_err_to_name(err));
            return;
        }
        bytes += chunk;
        length -= chunk;
    }
}

static void tft_command(uint8_t command, const uint8_t *params, size_t param_len)
{
    tft_write(false, &command, 1);
    tft_write(true, params, param_len);
}

static void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    const uint8_t column[] = { (uint8_t)(x0 >> 8), (uint8_t)x0,
                               (uint8_t)(x1 >> 8), (uint8_t)x1 };
    const uint8_t row[] = { (uint8_t)(y0 >> 8), (uint8_t)y0,
                            (uint8_t)(y1 >> 8), (uint8_t)y1 };
    tft_command(0x2A, column, sizeof(column));  /* CASET */
    tft_command(0x2B, row, sizeof(row));         /* RASET */
    tft_command(0x2C, NULL, 0);                  /* RAMWR */
}

/* ---------------- 帧缓冲图元 ---------------- */

static inline void fb_set(int x, int y, uint16_t c)
{
    if (x < 0 || x >= TFT_W || y < 0 || y >= TFT_H) return;
    s_fb[y * TFT_W + x] = c;
}

static void fb_fill(uint16_t c)
{
    for (int i = 0; i < TFT_W * TFT_H; i++) s_fb[i] = c;
}

static void fb_fill_rect(int x, int y, int w, int h, uint16_t c)
{
    for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
            fb_set(i, j, c);
}

static void fb_fill_circle(int cx, int cy, int r, uint16_t c)
{
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)sqrtf((float)(r * r - dy * dy));
        fb_fill_rect(cx - dx, cy + dy, dx * 2 + 1, 1, c);
    }
}

/* 填充三角形（用于爱心/尖角嘴） */
static void fb_fill_triangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t c)
{
    /* 按 y 排序 */
    if (y0 > y1) { int t; t=x0;x0=x1;x1=t; t=y0;y0=y1;y1=t; }
    if (y1 > y2) { int t; t=x1;x1=x2;x2=t; t=y1;y1=y2;y2=t; }
    if (y0 > y1) { int t; t=x0;x0=x1;x1=t; t=y0;y0=y1;y1=t; }

    int pts[3][2] = { {x0,y0},{x1,y1},{x2,y2} };
    for (int y = y0; y <= y2; y++) {
        /* 对每条边 (p,q)，若 y 在区间内则插值出 x 交点 */
        float xs[2];
        int xn = 0;
        for (int e = 0; e < 3; e++) {
            int ax = pts[e][0], ay = pts[e][1];
            int bx = pts[(e+1)%3][0], by = pts[(e+1)%3][1];
            if (by == ay) continue;             /* 水平边跳过 */
            int lo = (ay < by) ? ay : by;
            int hi = (ay < by) ? by : ay;
            if (y >= lo && y <= hi) {
                float t = (float)(y - ay) / (by - ay);
                if (xn < 2) xs[xn++] = ax + t * (bx - ax);
            }
        }
        if (xn == 2) {
            int l = (int)xs[0], r = (int)xs[1];
            if (l > r) { int t = l; l = r; r = t; }
            fb_fill_rect(l, y, r - l + 1, 1, c);
        }
    }
}

/* ---------------- 表情图元 ---------------- */

/* 底色 + 脸圆 + 描边 + 上下色带 */
static void draw_base(uint16_t band)
{
    fb_fill(C_WHITE);
    fb_fill_rect(0, 0, TFT_W, 16, band);
    fb_fill_rect(0, TFT_H - 16, TFT_W, 16, band);
    fb_fill_circle(64, 76, 48, C_BROWN);   /* 外圈描边 */
    fb_fill_circle(64, 76, 45, C_YELLOW);  /* 脸 */
}

static void face_eyes_round(void)
{
    fb_fill_circle(43, 66, 7, C_BLACK);
    fb_fill_circle(85, 66, 7, C_BLACK);
    fb_fill_circle(45, 63, 2, C_WHITE);    /* 高光 */
    fb_fill_circle(87, 63, 2, C_WHITE);
}

static void face_eyes_dot(void)
{
    fb_fill_circle(43, 66, 4, C_BLACK);
    fb_fill_circle(85, 66, 4, C_BLACK);
}

static void face_eyes_wink(void)
{
    fb_fill_circle(43, 66, 7, C_BLACK);
    fb_fill_circle(45, 63, 2, C_WHITE);
    fb_fill_rect(78, 66, 16, 4, C_BLACK);  /* 右眼眯成线 */
}

static void face_eyes_surprised(void)
{
    fb_fill_circle(42, 66, 9, C_BLACK);
    fb_fill_circle(86, 66, 9, C_BLACK);
    fb_fill_circle(45, 62, 3, C_WHITE);
    fb_fill_circle(89, 62, 3, C_WHITE);
}

/* 嘴：在嘴位画黑圆后盖掉一半，得到月牙 */
static void face_mouth_half(int cx, int cy, int r, bool cover_top)
{
    fb_fill_circle(cx, cy, r, C_BLACK);
    if (cover_top) {
        fb_fill_rect(cx - r, cy - r, r * 2, r, C_YELLOW);  /* 盖上 → 微笑 U */
    } else {
        fb_fill_rect(cx - r, cy, r * 2, r, C_YELLOW);      /* 盖下 → 愁眉 ∩ */
    }
}

static void face_mouth_o(int cx, int cy, int r)
{
    fb_fill_circle(cx, cy, r, C_BLACK);   /* 惊讶张嘴 */
}

static void face_mouth_straight(void)
{
    fb_fill_rect(52, 106, 24, 4, C_BLACK);
}

static void face_tear(int ex)
{
    fb_fill_circle(ex, 84, 3, C_BLUE);
    fb_fill_circle(ex, 90, 2, C_BLUE);
}

static void face_heart(int cx, int cy)
{
    fb_fill_circle(cx - 4, cy - 3, 5, C_RED);
    fb_fill_circle(cx + 4, cy - 3, 5, C_RED);
    fb_fill_rect(cx - 4, cy - 4, 8, 5, C_RED);
    fb_fill_triangle(cx - 8, cy - 2, cx + 8, cy - 2, cx, cy + 9, C_RED);
}

/* ---------------- 对外接口 ---------------- */

void display_init(void)
{
    /* 幂等保护：SPI2_HOST 总线重复初始化会 abort 重启 */
    if (s_spi != NULL) {
        return;
    }

    gpio_config_t dc_cfg = {
        .pin_bit_mask = 1ULL << TFT_PIN_DC,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dc_cfg));

    const spi_bus_config_t bus_cfg = {
        .sclk_io_num = TFT_PIN_SCK,
        .mosi_io_num = TFT_PIN_SDI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TFT_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    const spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = TFT_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = -1,  /* TFT CS 已硬件接地 */
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(TFT_HOST, &dev_cfg, &s_spi));

    /* DC 在 spi_bus_add_device 之后才配置完成，保证不被外设复用覆盖；
     * RST 与 EN 同步，等待屏幕上电复位完成 */
    vTaskDelay(pdMS_TO_TICKS(120));

    /* ---- ILI9163 初始化序列（龙邱官方 LQ_TFT18，实车验证） ---- */
    tft_command(0x11, NULL, 0);      /* SLPOUT：关闭睡眠，振荡器工作 */
    vTaskDelay(pdMS_TO_TICKS(120));
    const uint8_t color_mode = 0x05; /* 0x3A COLMOD：16bit/像素 RGB565 */
    tft_command(0x3A, &color_mode, 1);
    const uint8_t gamset = 0x04;     /* 0x26 GAMSET */
    tft_command(0x26, &gamset, 1);
    const uint8_t f2 = 0x01;         /* 0xF2 Driver Output Control */
    tft_command(0xF2, &f2, 1);
    /* 0xE0 正伽马校正（官方参数） */
    const uint8_t gamma_pos[] = {
        0x3F, 0x25, 0x1C, 0x1E, 0x20, 0x12, 0x2A, 0x90,
        0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    /* 0xE1 负伽马校正（官方参数） */
    const uint8_t gamma_neg[] = {
        0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xA7,
        0x3D, 0x18, 0x25, 0x2A, 0x2B, 0x2B, 0x3A,
    };
    tft_command(0xE0, gamma_pos, sizeof(gamma_pos));
    tft_command(0xE1, gamma_neg, sizeof(gamma_neg));
    const uint8_t frmctr[] = { 0x00, 0x00 };   /* 0xB1 帧率（官方 DIVA/VPA） */
    tft_command(0xB1, frmctr, sizeof(frmctr));
    const uint8_t invcl = 0x07;      /* 0xB4 NLA/NLB/NLC 反转控制 */
    tft_command(0xB4, &invcl, 1);
    const uint8_t pwctr1[] = { 0x0A, 0x02 };   /* 0xC0 电源控制 1 */
    tft_command(0xC0, pwctr1, sizeof(pwctr1));
    const uint8_t pwctr2 = 0x02;     /* 0xC1 电源控制 2 */
    tft_command(0xC1, &pwctr2, 1);
    const uint8_t vmctrl[] = { 0x4F, 0x5A };   /* 0xC5 VCOM 控制 */
    tft_command(0xC5, vmctrl, sizeof(vmctrl));
    const uint8_t vcom_offset = 0x40;          /* 0xC7 VCOM offset */
    tft_command(0xC7, &vcom_offset, 1);
    /* 0x36 MADCTL：竖屏正显 + BGR 位（官方 type=1 实测修正） */
    const uint8_t madctl = 0xC8;
    tft_command(0x36, &madctl, 1);
    const uint8_t gctl = 0x00;       /* 0xB7 GCTL */
    tft_command(0xB7, &gctl, 1);
    tft_command(0x29, NULL, 0);      /* DISPON 开显示 */
    tft_command(0x2C, NULL, 0);      /* RAMWR */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 帧缓冲优先内部 DMA 内存（最稳）；不足再退回 PSRAM。
     * 发送经内部 staging 缓冲分块拷贝，两种位置均可工作。 */
    s_fb = heap_caps_malloc(TFT_W * TFT_H * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (s_fb == NULL) {
        s_fb = heap_caps_malloc(TFT_W * TFT_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_fb != NULL) {
            ESP_LOGW(TAG, "内部 DMA 内存不足，帧缓冲退回 PSRAM");
        }
    }
    if (s_fb == NULL) {
        ESP_LOGE(TAG, "帧缓冲分配失败（%d 字节），显示不可用", TFT_W * TFT_H * 2);
        return;
    }
    ESP_LOGI(TAG, "ILI9163 128x160 初始化完成（fb 在%s，SPI %d kHz）",
             esp_ptr_internal(s_fb) ? "内部 RAM" : "PSRAM", TFT_SPI_CLOCK_HZ / 1000);
}

static void push_region(int y0, int y1)
{
    if (!s_fb) return;
    /* 该屏 R/B 物理互换：发送前交换 RGB565 高低 5 位，屏幕颜色才正确 */
    static uint8_t stage[512];
    tft_set_window(0, y0, TFT_W - 1, y1);
    const uint16_t *px = s_fb + (size_t)y0 * TFT_W;
    size_t remain = (size_t)(y1 - y0 + 1) * TFT_W;
    while (remain > 0) {
        const size_t n = remain > 256 ? 256 : remain;
        for (size_t i = 0; i < n; i++) {
            uint16_t c = px[i];
            c = (uint16_t)(((c & 0x001F) << 11) | (c & 0x07E0) | ((c >> 11) & 0x001F));
            stage[i * 2]     = (uint8_t)(c >> 8);
            stage[i * 2 + 1] = (uint8_t)c;
        }
        tft_write(true, stage, n * 2);
        px += n;
        remain -= n;
    }
}

static void push_fb(void)
{
    push_region(0, TFT_H - 1);
}

void display_face(int face_id)
{
    if (!s_fb) return;
    switch (face_id) {
    case 0:  /* 笑脸 */
        draw_base(C_GREEN);
        face_eyes_round();
        face_mouth_half(64, 108, 12, true);
        break;
    case 1:  /* 哭脸 */
        draw_base(C_LBLUE);
        face_eyes_round();
        face_tear(47);
        face_tear(81);
        face_mouth_half(64, 108, 12, false);
        break;
    case 2:  /* 惊讶 */
        draw_base(C_LBLUE);
        face_eyes_surprised();
        face_mouth_o(64, 114, 8);
        break;
    case 3:  /* 眨眼 */
        draw_base(C_GREEN);
        face_eyes_wink();
        face_mouth_half(64, 108, 12, true);
        break;
    case 4:  /* 爱心眼 */
        draw_base(C_LBLUE);
        face_heart(43, 68);
        face_heart(85, 68);
        face_mouth_half(64, 108, 12, true);
        break;
    default:
        draw_base(C_GREEN);
        face_eyes_round();
        face_mouth_half(64, 108, 12, true);
        break;
    }
    push_fb();
}

void display_idle(void)
{
    if (!s_fb) return;
    draw_base(C_GREEN);
    face_eyes_dot();
    face_mouth_straight();
    push_fb();
}

/* ============================================================
 * GIF 表情动画（离线转换资源）
 * 转换器：tools/qr_gif_convert.py（全局 256 色调色板 + 索引 RLE）
 * 资源由 CMake EMBED_FILES 嵌入（main/gif_asset.bin）
 * ============================================================ */

extern const uint8_t gif_asset_start[] asm("_binary_gif_asset_bin_start");
extern const uint8_t gif_asset_end[]   asm("_binary_gif_asset_bin_end");

#define GIF_MAGIC   0x32534147  /* 'GAS2' */

typedef struct __attribute__((packed)) {
    uint32_t offset;
    uint32_t size;
    uint16_t delay_ms;
    uint16_t flags;      /* bit0 = RLE */
} gif_frame_entry_t;

static bool s_gif_ok;
static uint16_t s_gif_count, s_gif_w, s_gif_h;
static const uint16_t *s_gif_pal;          /* RGB565 调色板（语义色，push_fb 统一换 R/B） */
static const gif_frame_entry_t *s_gif_table;
static int s_gif_y0;                       /* 在屏上的纵向偏移（居中） */

static void gif_parse(void)
{
    if (s_gif_ok || s_fb == NULL) return;
    const uint8_t *p = gif_asset_start;
    uint32_t magic;
    uint16_t count, w, h, pal_count;
    memcpy(&magic, p, 4);
    memcpy(&count, p + 4, 2);
    memcpy(&w, p + 6, 2);
    memcpy(&h, p + 8, 2);
    memcpy(&pal_count, p + 10, 2);
    if (magic != GIF_MAGIC || count == 0 || w == 0 || h == 0 || pal_count != 256) {
        ESP_LOGE(TAG, "GIF 资源无效 (magic=%08" PRIx32 ")", magic);
        return;
    }
    s_gif_count = count;
    s_gif_w = w;
    s_gif_h = h;
    s_gif_pal = (const uint16_t *)(p + 12);
    s_gif_table = (const gif_frame_entry_t *)(p + 12 + 512);
    s_gif_y0 = (TFT_H - h) / 2;
    s_gif_ok = true;
    ESP_LOGI(TAG, "GIF 资源: %u 帧 %ux%u（y0=%d）", count, w, h, s_gif_y0);
}

int display_gif_count(void)
{
    gif_parse();
    return s_gif_ok ? s_gif_count : 0;
}

/**
 * @brief 绘制第 index 帧并推屏
 * @return 该帧延时 ms（资源无效返回 0）
 */
int display_gif_draw(int index)
{
    gif_parse();
    if (!s_gif_ok || index < 0 || index >= (int)s_gif_count) return 0;
    const gif_frame_entry_t *e = &s_gif_table[index];
    /* 帧偏移是相对资源文件起始的绝对偏移（含头/调色板/帧表），
     * 必须从 gif_asset_start 起算；从数据区起算会整体错位 1112 字节。 */
    const uint8_t *d = gif_asset_start + e->offset;
    const int npix = s_gif_w * s_gif_h;
    uint16_t *dst = s_fb + s_gif_y0 * TFT_W;

    if (e->flags & 1) {
        /* RLE: (u8 调色板索引, u8 行程) 对 */
        int px = 0;
        for (uint32_t left = e->size; left >= 2 && px < npix; left -= 2, d += 2) {
            const uint16_t c = s_gif_pal[d[0]];
            int run = d[1];
            if (px + run > npix) run = npix - px;
            for (int k = 0; k < run; k++) dst[px++] = c;
        }
    } else {
        for (int px = 0; px < npix; px++) dst[px] = s_gif_pal[d[px]];
    }
    /* 只重推 GIF 所在区域（上下边带静止，不必每帧重发，也缩短传输） */
    push_region(s_gif_y0, s_gif_y0 + s_gif_h - 1);
    return e->delay_ms;
}
