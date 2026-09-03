#include "tft_st7735.h"
#include "encoder.h"
#include "pins.h"
#include "ultrasonic.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
/* polling 事务安全分块大小（字节）：无 DMA 缓冲时单事务上限约 64 字节，
 * 预留余量取 60；tft_write 内部按此分块，调用方无需关心。 */
#define TFT_SPI_CHUNK_MAX     60u
/*
 * 颜色宏（RGB565）。实测该屏面板为 BGR 物理排列（MADCTL BGR 位已置，
 * R/B 仍互换），故在软件侧交换 RED/BLUE 的位值，使上层调用语义正确：
 * TFT_COLOR_RED 宏在屏幕上呈现红色。
 */
#define TFT_COLOR_BLACK       0x0000
#define TFT_COLOR_WHITE       0xFFFF
#define TFT_COLOR_RED         0x001F
#define TFT_COLOR_GREEN       0x07E0
#define TFT_COLOR_BLUE        0xF800
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
    { 'E', {0x7F, 0x49, 0x49, 0x49, 0x41} },
    { 'L', {0x7F, 0x40, 0x40, 0x40, 0x40} },
    { 'O', {0x3E, 0x41, 0x41, 0x41, 0x3E} },
    { 'S', {0x46, 0x49, 0x49, 0x49, 0x31} },
    { 'T', {0x01, 0x01, 0x7F, 0x01, 0x01} },
    { '+', {0x08, 0x08, 0x3E, 0x08, 0x08} },
    { ':', {0x00, 0x36, 0x36, 0x00, 0x00} },
    { '-', {0x08, 0x08, 0x08, 0x08, 0x08} },
    { ' ', {0x00, 0x00, 0x00, 0x00, 0x00} },
};

static spi_device_handle_t s_spi;
static volatile uint16_t s_distance_cm = ULTRASONIC_INVALID;
static uint16_t s_distance_samples[3];
static uint8_t s_distance_sample_count;
static uint8_t s_distance_next_index;
/* 摄像头巡线状态（任务二）：主循环写入，TFT 任务读取显示 */
static volatile bool s_line_has;
static volatile float s_line_error;
static volatile bool s_line_stable;
static volatile bool s_camera_mode;
static volatile bool s_telemetry_clear_pending;
static SemaphoreHandle_t s_draw_mutex;
static TaskHandle_t s_tft_task_handle;

static void tft_write(bool is_data, const void *data, size_t length)
{
    if (length == 0) {
        return;
    }
    ESP_ERROR_CHECK(gpio_set_level(TFT_DC_GPIO, is_data ? 1 : 0));
    /*
     * 分块发送：polling 事务单次长度受限（无 DMA 缓冲时约 64 字节），
     * 超限会触发 ESP_ERROR_CHECK abort 重启。按 240 字节分块，
     * 兼顾效率与安全；调用方传入的缓冲必须保持有效直到全部发完。
     */
    const uint8_t *bytes = (const uint8_t *)data;
    while (length > 0) {
        const size_t chunk = length > TFT_SPI_CHUNK_MAX ? TFT_SPI_CHUNK_MAX : length;
        spi_transaction_t transaction = {
            .length = chunk * 8,
            .tx_buffer = bytes,
        };
        esp_err_t err = spi_device_polling_transmit(s_spi, &transaction);
        if (err != ESP_OK) {
            /* 写屏失败只影响显示，不允许拖垮整车控制：记录并跳过本次剩余数据 */
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
    /*
     * SCALE=1：原始 5x7 字库直接绘制（字符 5x7 像素、步进 6）。
     * 之前 SCALE=2 时每行高 14px、5 行占 148px 超出 160px 屏高，
     * 第 5 行（E 行）被裁掉。改 1 倍后 5 行只需 ~75px，全部可见。
     */
    enum { SCALE = 1, GLYPH_W = 5 * SCALE, GLYPH_H = 7 * SCALE };
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
    while (*text != '\0' && x + 6 <= TFT_WIDTH) {
        tft_draw_char(x, y, *text++, color);
        x += 6;
    }
}

void tft_st7735_show_camera_frame(const uint8_t *rgb888, int frame_w, int frame_h)
{
    if (s_spi == NULL || !s_camera_mode || rgb888 == NULL ||
        frame_w <= 0 || frame_h <= 0 || s_draw_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_draw_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return;
    }

    /* 等待锁时显示模式可能已切回遥测，重新确认后再写屏。 */
    if (!s_camera_mode) {
        xSemaphoreGive(s_draw_mutex);
        return;
    }

    /* 完整画面等比例缩放（letterbox），不再裁掉左右两侧目标。 */
    int dst_w = TFT_WIDTH;
    int dst_h = frame_h * dst_w / frame_w;
    if (dst_h > TFT_HEIGHT) {
        dst_h = TFT_HEIGHT;
        dst_w = frame_w * dst_h / frame_h;
    }
    if (dst_w <= 0 || dst_h <= 0) {
        xSemaphoreGive(s_draw_mutex);
        return;
    }
    const int dst_x0 = (TFT_WIDTH - dst_w) / 2;
    const int dst_y0 = (TFT_HEIGHT - dst_h) / 2;

    uint8_t pixels[TFT_WIDTH * 2];   /* 一行 128 像素 RGB565，高字节在前 */
    tft_set_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);

    for (int dy = 0; dy < TFT_HEIGHT; dy++) {
        for (int dx = 0; dx < TFT_WIDTH; dx++) {
            uint16_t color = TFT_COLOR_BLACK;
            if (dx >= dst_x0 && dx < dst_x0 + dst_w &&
                dy >= dst_y0 && dy < dst_y0 + dst_h) {
                const int sx = (dx - dst_x0) * frame_w / dst_w;
                const int sy = (dy - dst_y0) * frame_h / dst_h;
                const uint8_t *px = rgb888 +
                    ((size_t)sy * (size_t)frame_w + (size_t)sx) * 3;
                const uint16_t r5 = px[0] >> 3;
                const uint16_t g6 = px[1] >> 2;
                const uint16_t b5 = px[2] >> 3;
                /* 该屏 R/B 物理互换：发送时交换 R/B 位，屏幕颜色才正确 */
                color = (uint16_t)((b5 << 11) | (g6 << 5) | r5);
            }
            pixels[dx * 2]     = (uint8_t)(color >> 8);
            pixels[dx * 2 + 1] = (uint8_t)color;
        }
        tft_write(true, pixels, sizeof(pixels));
    }
    xSemaphoreGive(s_draw_mutex);
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

    /* SCALE=1 后每行高 7px、行距 8px：5 行占 y=8~152，全部在屏内 */
    snprintf(line, sizeof(line), "A:%04dRPM", rpm_for_display(rpm.rpm[MOTOR_A], rpm.valid[MOTOR_A]));
    tft_draw_text(8, 8, line, TFT_COLOR_RED);
    snprintf(line, sizeof(line), "B:%04dRPM", rpm_for_display(rpm.rpm[MOTOR_B], rpm.valid[MOTOR_B]));
    tft_draw_text(8, 24, line, TFT_COLOR_GREEN);
    snprintf(line, sizeof(line), "C:%04dRPM", rpm_for_display(rpm.rpm[MOTOR_C], rpm.valid[MOTOR_C]));
    tft_draw_text(8, 40, line, TFT_COLOR_BLUE);

    if (distance == ULTRASONIC_INVALID) {
        strcpy(line, "D:---CM");
    } else {
        snprintf(line, sizeof(line), "D:%03uCM", (unsigned)distance);
    }
    tft_draw_text(8, 56, line, TFT_COLOR_YELLOW);

    /* 第 5 行：摄像头巡线状态（任务二），含稳定标志 */
    if (s_line_has) {
        snprintf(line, sizeof(line), "E:%+.2f %s",
                 (double)s_line_error, s_line_stable ? "OK" : "!");
    } else {
        strcpy(line, "E:LOST");
    }
    tft_draw_text(8, 72, line, TFT_COLOR_WHITE);
}

static void tft_task(void *unused)
{
    (void)unused;
    while (true) {
        encoder_update();
        if (!s_camera_mode && s_draw_mutex != NULL &&
            xSemaphoreTake(s_draw_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (!s_camera_mode) {
                if (s_telemetry_clear_pending) {
                    tft_fill_screen(TFT_COLOR_BLACK);
                    s_telemetry_clear_pending = false;
                }
                tft_draw_telemetry();
            }
            xSemaphoreGive(s_draw_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(TFT_REFRESH_MS));
    }
}

void tft_st7735_init(void)
{
    /*
     * 幂等保护：综合模式与面板自检可能先后调用本函数。
     * SPI2_HOST 总线重复初始化会返回 ESP_ERR_INVALID_STATE，
     * 经 ESP_ERROR_CHECK 直接 abort 重启——实测表现为跑车途中白屏重启。
     */
    if (s_spi != NULL) {
        return;
    }

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
    s_draw_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_draw_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    /*
     * DC 引脚在 spi_bus_add_device 之后才配置为输出：SPI 主机初始化时
     * 会通过 GPIO 矩阵接管 SCK/MOSI，但不会动 DC；此处顺序保证 DC
     * 不会被任何外设复用覆盖。上电自检（红绿蓝黑）放在最后执行。
     */

    vTaskDelay(pdMS_TO_TICKS(120));  /* RST 与 EN 同步，等待屏幕上电复位完成 */
    /*
     * 初始化序列改为龙邱官方 LQ_TFT18（spi_TFT18.c）的 ILI9163 版本。
     * 该屏（LQ_TFT18SPIV33）实际为 ILI9163 控制器；此前按 ST7735 序列
     * 配置电源参数（B1/C0/C5）不匹配，表现为背光亮但全白。
     * 官方序列：RST 脉冲后 SLPOUT -> COLMOD -> GAMSET -> 0xF2 ->
     * gamma E0/E1 -> 帧率/电源/VMC -> MADCTL -> DISPON。
     */
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
    /*
     * 0x36 MADCTL：0xC0 = 竖屏正显（MX=1,MY=1，官方 type=1）。
     * 实测该屏发红显蓝、发蓝显黄、黑显白——R/B 互换且整体反色，
     * 需置 BGR 位(0x08)：0xC0 -> 0xC8。若仍有偏差再调 MY/MX。
     */
    const uint8_t madctl = 0xC8;
    tft_command(0x36, &madctl, 1);
    const uint8_t gctl = 0x00;       /* 0xB7 GCTL */
    tft_command(0xB7, &gctl, 1);
    tft_command(0x29, NULL, 0);      /* DISPON 开显示 */
    tft_command(0x2C, NULL, 0);      /* RAMWR */
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
    if (s_tft_task_handle != NULL) {
        return;
    }
    BaseType_t created = xTaskCreate(tft_task, "tft", 6144, NULL, 1,
                                     &s_tft_task_handle);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

void tft_st7735_set_camera_mode(bool enabled)
{
    const bool was_enabled = s_camera_mode;
    s_camera_mode = enabled;
    if (was_enabled && !enabled) {
        /* 控制循环不能等待一次约 0.33s 的整屏 SPI；交给 TFT 任务在
         * 获得显示锁后清除最后一帧，再恢复遥测页。 */
        s_telemetry_clear_pending = true;
    }
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

void tft_st7735_set_line(bool has_line, float error, bool stable)
{
    s_line_has = has_line;
    s_line_error = error;
    s_line_stable = stable;
}
