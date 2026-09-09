#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tft_disp.h"
#include "tft_font_data.h"

/* ============================================================
 * TFT 状态屏驱动（独立裁剪版，见 tft_disp.h）
 *
 * 移植来源：task2-camera-line/main/tft_st7735.c（实车验证，
 * 2026-09-03 提速 8MHz；ILI9163 官方初始化序列；BGR/RB 修正）。
 * 本文件只保留 初始化/清屏/文本 三个能力，供状态屏模块使用；
 * 与表情功能共用同一面板时由调用方仲裁（status_disp 在表情
 * 模式让屏），不要在两条路径里同时刷屏。
 * ============================================================ */

#define TAG "tft"

#define TFT_HOST         SPI2_HOST
#define TFT_SCLK_PIN     GPIO_NUM_13
#define TFT_MOSI_PIN     GPIO_NUM_15
#define TFT_DC_PIN       GPIO_NUM_17
#define TFT_SPI_CLOCK_HZ (8 * 1000 * 1000)
#define TFT_SPI_CHUNK_MAX 60u

static spi_device_handle_t s_spi = NULL;
static bool s_ready = false;

/* ---------------- SPI 原语（DC 前置为命令/数据） ---------------- */
static void tft_write(bool is_data, const uint8_t *bytes, size_t length)
{
    if (s_spi == NULL) {
        return;
    }
    gpio_set_level(TFT_DC_PIN, is_data ? 1 : 0);
    while (length > 0) {
        const size_t chunk = length > TFT_SPI_CHUNK_MAX ? TFT_SPI_CHUNK_MAX : length;
        spi_transaction_t t = {
            .length = chunk * 8,
            .tx_buffer = bytes,
        };
        if (spi_device_polling_transmit(s_spi, &t) != ESP_OK) {
            return;     /* 显示失败不影响整车控制 */
        }
        bytes += chunk;
        length -= chunk;
    }
}

static void tft_cmd(uint8_t cmd, const uint8_t *params, size_t len)
{
    tft_write(false, &cmd, 1);
    if (params && len) {
        tft_write(true, params, len);
    }
}

static void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    const uint8_t col[] = { (uint8_t)(x0 >> 8), (uint8_t)x0,
                            (uint8_t)(x1 >> 8), (uint8_t)x1 };
    const uint8_t row[] = { (uint8_t)(y0 >> 8), (uint8_t)y0,
                            (uint8_t)(y1 >> 8), (uint8_t)y1 };
    tft_cmd(0x2A, col, sizeof(col));    /* CASET */
    tft_cmd(0x2B, row, sizeof(row));    /* RASET */
    tft_cmd(0x2C, NULL, 0);             /* RAMWR */
}

void tft_disp_fill(uint16_t color)
{
    uint8_t pixels[512];
    for (size_t i = 0; i < sizeof(pixels); i += 2) {
        pixels[i] = (uint8_t)(color >> 8);
        pixels[i + 1] = (uint8_t)color;
    }
    tft_set_window(0, 0, TFT_DISP_WIDTH - 1, TFT_DISP_HEIGHT - 1);
    size_t remaining = TFT_DISP_WIDTH * TFT_DISP_HEIGHT * 2;
    while (remaining > 0) {
        size_t chunk = remaining > sizeof(pixels) ? sizeof(pixels) : remaining;
        tft_write(true, pixels, chunk);
        remaining -= chunk;
    }
}

void tft_disp_init(void)
{
    if (s_spi != NULL) {
        return;     /* 幂等 */
    }

    const spi_bus_config_t bus_cfg = {
        .sclk_io_num = TFT_SCLK_PIN,
        .mosi_io_num = TFT_MOSI_PIN,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TFT_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    const spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = TFT_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = -1,     /* CS 已硬件接地 */
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(TFT_HOST, &dev_cfg, &s_spi));

    /* DC 引脚在 spi_bus_add_device 之后配置（与 task2 同序）：
     * SPI 主机初始化会经 GPIO 矩阵接管 SCK/MOSI，此顺序避免 DC
     * 被外设复用覆盖。 */
    gpio_config_t dc_cfg = {
        .pin_bit_mask = 1ULL << TFT_DC_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dc_cfg));

    vTaskDelay(pdMS_TO_TICKS(120));     /* RST 与 EN 同步，等面板上电复位 */

    /* ILI9163 官方初始化序列（task2 实机验证） */
    tft_cmd(0x11, NULL, 0);             /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(120));
    const uint8_t color_mode = 0x05;    /* COLMOD 16bit/pixel */
    tft_cmd(0x3A, &color_mode, 1);
    const uint8_t gamset = 0x04;        /* GAMSET */
    tft_cmd(0x26, &gamset, 1);
    const uint8_t f2 = 0x01;            /* Driver Output Control */
    tft_cmd(0xF2, &f2, 1);
    const uint8_t gamma_pos[] = { 0x3F, 0x25, 0x1C, 0x1E, 0x20, 0x12, 0x2A, 0x90,
                                  0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t gamma_neg[] = { 0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xA7,
                                  0x3D, 0x18, 0x25, 0x2A, 0x2B, 0x2B, 0x3A };
    tft_cmd(0xE0, gamma_pos, sizeof(gamma_pos));
    tft_cmd(0xE1, gamma_neg, sizeof(gamma_neg));
    const uint8_t frmctr[] = { 0x00, 0x00 };    /* 帧率 */
    tft_cmd(0xB1, frmctr, sizeof(frmctr));
    const uint8_t invcl = 0x07;
    tft_cmd(0xB4, &invcl, 1);
    const uint8_t pwctr1[] = { 0x0A, 0x02 };
    tft_cmd(0xC0, pwctr1, sizeof(pwctr1));
    const uint8_t pwctr2 = 0x02;
    tft_cmd(0xC1, &pwctr2, 1);
    const uint8_t vmctrl[] = { 0x4F, 0x5A };
    tft_cmd(0xC5, vmctrl, sizeof(vmctrl));
    const uint8_t vcom = 0x40;
    tft_cmd(0xC7, &vcom, 1);
    const uint8_t madctl = 0xC8;        /* 竖屏正显 + BGR（实测修正） */
    tft_cmd(0x36, &madctl, 1);
    const uint8_t gctl = 0x00;
    tft_cmd(0xB7, &gctl, 1);
    tft_cmd(0x29, NULL, 0);             /* DISPON */
    vTaskDelay(pdMS_TO_TICKS(100));

    s_ready = true;
    ESP_LOGI(TAG, "TFT 状态屏就绪（ILI9163 @8MHz）");

    /* 面板自检：整屏 红→绿→蓝→黑，每色 700ms（状态页会在最后覆盖黑屏）。
     * 实机排障用——若屏幕全程白屏说明面板没收到 SPI 命令。 */
    ESP_LOGI(TAG, "面板自检 RED");
    tft_disp_fill(TFT_COLOR_RED);
    vTaskDelay(pdMS_TO_TICKS(700));
    ESP_LOGI(TAG, "面板自检 GREEN");
    tft_disp_fill(TFT_COLOR_GREEN);
    vTaskDelay(pdMS_TO_TICKS(700));
    ESP_LOGI(TAG, "面板自检 BLUE");
    tft_disp_fill(TFT_COLOR_BLUE);
    vTaskDelay(pdMS_TO_TICKS(700));
    tft_disp_fill(TFT_COLOR_BLACK);
    ESP_LOGI(TAG, "面板自检完成");
}

/* ---------------- 文本 ---------------- */
static const uint8_t *font_find(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        ch = (char)(ch - 'a' + 'A');
    }
    for (size_t i = 0; i < FONT_CHARS; i++) {
        if (s_font[i].ch == ch) {
            return s_font[i].rows;
        }
    }
    return s_font[0].rows;      /* 未知字符按空格显示（表首为空格） */
}

void tft_disp_text(uint8_t row, const char *text, uint16_t color)
{
    if (!s_ready || row >= TFT_DISP_ROWS) {
        return;
    }
    const int y = 8 + row * 16;
    enum { GLYPH_W = 5, GLYPH_H = 7 };
    uint8_t pixels[GLYPH_W * GLYPH_H * 2];
    int x = 0;

    while (*text != '\0' && x + GLYPH_W + 1 <= TFT_DISP_WIDTH) {
        const uint8_t *g = font_find(*text);
        for (int py = 0; py < GLYPH_H; py++) {
            for (int px = 0; px < GLYPH_W; px++) {
                const bool on = (g[py] & (1u << (4 - px))) != 0;
                const uint16_t c = on ? color : TFT_COLOR_BLACK;
                const size_t idx = (size_t)(py * GLYPH_W + px) * 2;
                pixels[idx] = (uint8_t)(c >> 8);
                pixels[idx + 1] = (uint8_t)c;
            }
        }
        tft_set_window((uint16_t)x, (uint16_t)y,
                       (uint16_t)(x + GLYPH_W - 1), (uint16_t)(y + GLYPH_H - 1));
        tft_write(true, pixels, sizeof(pixels));
        x += 6;
        text++;
    }
}

bool tft_disp_ready(void)
{
    return s_ready;
}
