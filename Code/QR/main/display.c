#include "display.h"
#include "pins.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7735.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

/* ============================================================
 * TFT 显示屏实现（ST7735，128x160，RGB565）
 *
 * 绘制方式：先把整屏画进 PSRAM 帧缓冲，最后一次性 draw_bitmap，
 * 避免逐行 SPI 事务。
 *
 * ⚠️ ST7735 版型标定（若实机偏色/偏移/镜像，改下面宏）：
 *   DISPLAY_INVERT   颜色取反（多数 1.8" 需要 =1）
 *   DISPLAY_GAP_X/Y  面板偏移（常见 0/0，部分版型 2/1）
 *   DISPLAY_SWAP_XY  横竖交换（装反时用）
 *   DISPLAY_MIRROR_X/Y  镜像
 *   DISPLAY_RGB_ORDER_RGB  RGB/BGR 通道顺序
 * ============================================================ */

#define TFT_W  128
#define TFT_H  160

#define LCD_HOST  SPI2_HOST

#define DISPLAY_INVERT        1
#define DISPLAY_GAP_X         0
#define DISPLAY_GAP_Y         0
#define DISPLAY_SWAP_XY       0
#define DISPLAY_MIRROR_X      0
#define DISPLAY_MIRROR_Y      0
#define DISPLAY_RGB_ORDER_RGB 1

static const char *TAG = "display";
static esp_lcd_panel_handle_t s_panel = NULL;
static uint16_t *s_fb = NULL;   /* 128x160 RGB565 帧缓冲（PSRAM） */

/* ---- RGB565 颜色 ---- */
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_YELLOW  0xFFE0
#define C_RED     0xF800
#define C_BLUE    0x001F
#define C_GREEN   0x07E0
#define C_LBLUE   0x7DFF   /* 淡蓝 */
#define C_BROWN   0x8800   /* 深棕（描边） */

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
    spi_bus_config_t bus = {
        .sclk_io_num = TFT_PIN_SCK,
        .mosi_io_num = TFT_PIN_SDI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_W * TFT_H * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = TFT_PIN_DC,
        .cs_gpio_num = TFT_PIN_CS,
        .pclk_hz = 26 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = TFT_PIN_RST,
        /* color_space 已废弃且与 rgb_ele_order 同处一个 union，只设 rgb_ele_order */
        .rgb_ele_order = DISPLAY_RGB_ORDER_RGB ? LCD_RGB_ELEMENT_ORDER_RGB
                                               : LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(io, &panel_cfg, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, DISPLAY_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, DISPLAY_GAP_X, DISPLAY_GAP_Y));
    if (DISPLAY_SWAP_XY) {
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));
    }
    if (DISPLAY_MIRROR_X || DISPLAY_MIRROR_Y) {
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
    }
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    s_fb = heap_caps_malloc(TFT_W * TFT_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "ST7735 128x160 初始化完成");
}

static void push_fb(void)
{
    if (!s_fb) return;
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, TFT_W, TFT_H, s_fb);
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
