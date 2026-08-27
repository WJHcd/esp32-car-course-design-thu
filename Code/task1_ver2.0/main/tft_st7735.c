#include "tft_st7735.h"
#include "encoder.h"
#include "pins.h"
#include "ultrasonic.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

/* LQ-TFT18SPIV33：常见 128x160 ST7735/ST7735S SPI 小屏初始参数。 */
#define TFT_HOST              SPI2_HOST
#define TFT_WIDTH             128
#define TFT_HEIGHT            160
/* 首次点亮阶段先用 1 MHz。显示数据量很小，低速更能适应杜邦线连接。 */
#define TFT_SPI_CLOCK_HZ      (1 * 1000 * 1000)
#define TFT_REFRESH_MS        250
#define TFT_COLOR_BLACK       0x0000
#define TFT_COLOR_WHITE       0xFFFF
#define TFT_COLOR_RED         0xF800
#define TFT_COLOR_GREEN       0x07E0
#define TFT_COLOR_BLUE        0x001F
#define TFT_COLOR_YELLOW      0xFFE0

typedef struct {
    char ch;
    uint8_t col[5];
} glyph_t;

/* 5x7 字库：只包含本仪表页所需字符，避免引入大字体库和帧缓冲。 */
static const glyph_t s_glyphs[] = {
    { '0', {0x3E, 0x51, 0x49, 0x45, 0x3E} },
    { '1', {0x00, 0x42, 0x7F, 0x40, 0x00} },
    { '2', {0x42, 0x61, 0x51, 0x49, 0x46} },
    { '3', {0x21, 0x41, 0x45, 0x4B, 0x31} },
    { '4', {0x18, 0x14, 0x12, 0x7F, 0x10} },
    { '5', {0x27, 0x45, 0x45, 0x45, 0x39} },
    { '6', {0x3C, 0x4A, 0x49, 0x49, 0x30} },
    { '7', {0x01, 0x71, 0x09, 0x05, 0x03} },
    { '8', {0x36, 0x49, 0x49, 0x49, 0x36} },
    { '9', {0x06, 0x49, 0x49, 0x29, 0x1E} },
    { 'A', {0x7E, 0x11, 0x11, 0x11, 0x7E} },
    { 'B', {0x7F, 0x49, 0x49, 0x49, 0x36} },
    { 'C', {0x3E, 0x41, 0x41, 0x41, 0x22} },
    { 'D', {0x7F, 0x41, 0x41, 0x22, 0x1C} },
    { 'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F} },
    { 'P', {0x7F, 0x09, 0x09, 0x09, 0x06} },
    { 'R', {0x7F, 0x09, 0x19, 0x29, 0x46} },
    { ':', {0x00, 0x36, 0x36, 0x00, 0x00} },
    { '-', {0x08, 0x08, 0x08, 0x08, 0x08} },
    { ' ', {0x00, 0x00, 0x00, 0x00, 0x00} },
};

static spi_device_handle_t s_spi;
static volatile uint16_t s_distance_cm = ULTRASONIC_INVALID;
static uint16_t s_distance_samples[3];
static uint8_t s_distance_sample_count;
static uint8_t s_distance_next_index;

static void tft_write(bool is_data, const void *data, size_t length)
{
    if (length == 0) {
        return;
    }
    ESP_ERROR_CHECK(gpio_set_level(TFT_DC_GPIO, is_data ? 1 : 0));
    spi_transaction_t transaction = {
        .length = length * 8,
        .tx_buffer = data,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_spi, &transaction));
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

static void tft_fill_screen(uint16_t color)
{
    uint8_t pixels[512];
    for (size_t i = 0; i < sizeof(pixels); i += 2) {
        pixels[i] = (uint8_t)(color >> 8);
        pixels[i + 1] = (uint8_t)color;
    }

    tft_set_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
    size_t remaining = TFT_WIDTH * TFT_HEIGHT * 2;
    while (remaining > 0) {
        const size_t chunk = remaining > sizeof(pixels) ? sizeof(pixels) : remaining;
        tft_write(true, pixels, chunk);
        remaining -= chunk;
    }
}

static const uint8_t *tft_find_glyph(char ch)
{
    for (size_t i = 0; i < sizeof(s_glyphs) / sizeof(s_glyphs[0]); i++) {
        if (s_glyphs[i].ch == ch) {
            return s_glyphs[i].col;
        }
    }
    return s_glyphs[sizeof(s_glyphs) / sizeof(s_glyphs[0]) - 1].col;
}

static void tft_draw_char(uint16_t x, uint16_t y, char ch, uint16_t fg)
{
    enum { SCALE = 2, GLYPH_W = 5 * SCALE, GLYPH_H = 7 * SCALE };
    uint8_t pixels[GLYPH_W * GLYPH_H * 2];
    const uint8_t *glyph = tft_find_glyph(ch);

    for (int py = 0; py < GLYPH_H; py++) {
        for (int px = 0; px < GLYPH_W; px++) {
            const bool on = (glyph[px / SCALE] & (1u << (py / SCALE))) != 0;
            const uint16_t color = on ? fg : TFT_COLOR_BLACK;
            const size_t index = (size_t)(py * GLYPH_W + px) * 2;
            pixels[index] = (uint8_t)(color >> 8);
            pixels[index + 1] = (uint8_t)color;
        }
    }

    tft_set_window(x, y, x + GLYPH_W - 1, y + GLYPH_H - 1);
    tft_write(true, pixels, sizeof(pixels));
}

static void tft_draw_text(uint16_t x, uint16_t y, const char *text, uint16_t color)
{
    while (*text != '\0' && x + 10 <= TFT_WIDTH) {
        tft_draw_char(x, y, *text++, color);
        x += 11;
    }
}

static int rpm_for_display(float rpm, bool valid)
{
    if (!valid || rpm < 0.5f) {
        return 0;
    }
    return rpm > 9999.0f ? 9999 : (int)(rpm + 0.5f);
}

static void tft_draw_telemetry(void)
{
    const encoder_rpm_t rpm = encoder_get_rpm();
    const uint16_t distance = s_distance_cm;
    char line[16];

    snprintf(line, sizeof(line), "A:%04dRPM", rpm_for_display(rpm.rpm[MOTOR_A], rpm.valid[MOTOR_A]));
    tft_draw_text(8, 16, line, TFT_COLOR_RED);
    snprintf(line, sizeof(line), "B:%04dRPM", rpm_for_display(rpm.rpm[MOTOR_B], rpm.valid[MOTOR_B]));
    tft_draw_text(8, 48, line, TFT_COLOR_GREEN);
    snprintf(line, sizeof(line), "C:%04dRPM", rpm_for_display(rpm.rpm[MOTOR_C], rpm.valid[MOTOR_C]));
    tft_draw_text(8, 80, line, TFT_COLOR_BLUE);

    if (distance == ULTRASONIC_INVALID) {
        strcpy(line, "D:---CM");
    } else {
        snprintf(line, sizeof(line), "D:%03uCM", (unsigned)distance);
    }
    tft_draw_text(8, 116, line, TFT_COLOR_YELLOW);
}

static void tft_task(void *unused)
{
    (void)unused;
    while (true) {
        encoder_update();
        tft_draw_telemetry();
        vTaskDelay(pdMS_TO_TICKS(TFT_REFRESH_MS));
    }
}

void tft_st7735_init(void)
{
    gpio_config_t dc_cfg = {
        .pin_bit_mask = 1ULL << TFT_DC_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dc_cfg));

    const spi_bus_config_t bus_cfg = {
        .sclk_io_num = TFT_SCLK_GPIO,
        .mosi_io_num = TFT_MOSI_GPIO,
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

    vTaskDelay(pdMS_TO_TICKS(120));  /* RST 与 EN 同步，等待屏幕上电复位完成 */
    tft_command(0x01, NULL, 0);      /* SWRESET */
    vTaskDelay(pdMS_TO_TICKS(150));
    tft_command(0x11, NULL, 0);      /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(120));
    /*
     * ST7735S 的完整上电序列。仅发送 SLPOUT/COLMOD/DISPON 对某些
     * 1.8 寸模块不足以完成电源与帧率配置，表现通常就是“背光亮、全白”。
     * 此处参数适配常见 128x160、红色 PCB 的 1.8 寸 SPI 屏。
     */
    const uint8_t frmctr1[] = { 0x01, 0x2C, 0x2D };
    const uint8_t frmctr2[] = { 0x01, 0x2C, 0x2D };
    const uint8_t frmctr3[] = { 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D };
    const uint8_t invctr = 0x07;
    const uint8_t pwctr1[] = { 0xA2, 0x02, 0x84 };
    const uint8_t pwctr2 = 0xC5;
    const uint8_t pwctr3[] = { 0x0A, 0x00 };
    const uint8_t pwctr4[] = { 0x8A, 0x2A };
    const uint8_t pwctr5[] = { 0x8A, 0xEE };
    const uint8_t vmctr1 = 0x0E;
    const uint8_t color_mode = 0x05; /* RGB565 */
    const uint8_t madctl = 0xC8;     /* 常见 1.8 寸 ST7735 竖屏 + BGR */
    const uint8_t gamma_pos[] = {
        0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
        0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
    };
    const uint8_t gamma_neg[] = {
        0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
        0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
    };
    tft_command(0xB1, frmctr1, sizeof(frmctr1));
    tft_command(0xB2, frmctr2, sizeof(frmctr2));
    tft_command(0xB3, frmctr3, sizeof(frmctr3));
    tft_command(0xB4, &invctr, 1);
    tft_command(0xC0, pwctr1, sizeof(pwctr1));
    tft_command(0xC1, &pwctr2, 1);
    tft_command(0xC2, pwctr3, sizeof(pwctr3));
    tft_command(0xC3, pwctr4, sizeof(pwctr4));
    tft_command(0xC4, pwctr5, sizeof(pwctr5));
    tft_command(0xC5, &vmctr1, 1);
    tft_command(0x3A, &color_mode, 1);
    tft_command(0x36, &madctl, 1);
    tft_command(0xE0, gamma_pos, sizeof(gamma_pos));
    tft_command(0xE1, gamma_neg, sizeof(gamma_neg));
    tft_command(0x21, NULL, 0);      /* INVON：多数 ST7735 1.8 寸屏需要 */
    tft_command(0x13, NULL, 0);      /* NORON */
    tft_command(0x29, NULL, 0);      /* DISPON */
    vTaskDelay(pdMS_TO_TICKS(100));

    /*
     * 上电自检：依次显示红、绿、蓝三个纯色。若三色都不可见而始终白屏，
     * 可确定问题在 SPI 连线/控制器型号，而非后续的文字、编码器或超声逻辑。
     */
    tft_fill_screen(TFT_COLOR_RED);
    vTaskDelay(pdMS_TO_TICKS(500));
    tft_fill_screen(TFT_COLOR_GREEN);
    vTaskDelay(pdMS_TO_TICKS(500));
    tft_fill_screen(TFT_COLOR_BLUE);
    vTaskDelay(pdMS_TO_TICKS(500));
    tft_fill_screen(TFT_COLOR_BLACK);

}

void tft_st7735_start_telemetry(void)
{
    BaseType_t created = xTaskCreate(tft_task, "tft", 4096, NULL, 1, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

void tft_st7735_panel_test(void)
{
    /* 该循环只会调用 SPI 写屏，无任何其他模块初始化或 GPIO 访问。 */
    while (true) {
        tft_fill_screen(TFT_COLOR_RED);
        vTaskDelay(pdMS_TO_TICKS(1000));
        tft_fill_screen(TFT_COLOR_GREEN);
        vTaskDelay(pdMS_TO_TICKS(1000));
        tft_fill_screen(TFT_COLOR_BLUE);
        vTaskDelay(pdMS_TO_TICKS(1000));
        tft_fill_screen(TFT_COLOR_BLACK);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tft_st7735_set_distance(uint16_t distance_cm)
{
    if (distance_cm == ULTRASONIC_INVALID) {
        return;
    }

    /*
     * 只用于显示的三点中值滤波：滤掉超声偶发飞点而不改变 walking.c 的
     * 避障输入和既有触发时机。样本数不足三点时先显示可用样本的均值。
     */
    s_distance_samples[s_distance_next_index] = distance_cm;
    s_distance_next_index = (uint8_t)((s_distance_next_index + 1u) % 3u);
    if (s_distance_sample_count < 3u) {
        s_distance_sample_count++;
    }

    uint16_t values[3] = {0};
    uint32_t sum = 0;
    for (uint8_t i = 0; i < s_distance_sample_count; i++) {
        values[i] = s_distance_samples[i];
        sum += values[i];
    }
    if (s_distance_sample_count < 3u) {
        s_distance_cm = (uint16_t)((sum + s_distance_sample_count / 2u) /
                                   s_distance_sample_count);
        return;
    }

    if (values[0] > values[1]) { uint16_t t = values[0]; values[0] = values[1]; values[1] = t; }
    if (values[1] > values[2]) { uint16_t t = values[1]; values[1] = values[2]; values[2] = t; }
    if (values[0] > values[1]) { uint16_t t = values[0]; values[0] = values[1]; values[1] = t; }
    s_distance_cm = values[1];
}
