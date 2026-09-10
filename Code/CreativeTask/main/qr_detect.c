#include "qr_detect.h"
#include "quirc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

/* ============================================================
 * 二维码识别实现（espressif/quirc）
 *
 * quirc 流程：quirc_begin 取缓冲 → 填灰度 → quirc_end 识别
 *   → quirc_count 数量 → 逐个 quirc_extract + quirc_decode 取文本。
 * 解码失败（ECC 错误）时用 quirc_flip 对反相码重试一次。
 * ============================================================ */

static const char *TAG = "qr_detect";
static struct quirc *s_q = NULL;

void qr_detect_init(int width, int height, int max_codes)
{
    (void)max_codes;
    if (s_q) {
        quirc_destroy(s_q);
        s_q = NULL;
    }
    if (width <= 0 || height <= 0) {
        ESP_LOGE(TAG, "非法尺寸 %dx%d", width, height);
        return;
    }
    s_q = quirc_new();
    if (!s_q) {
        ESP_LOGE(TAG, "quirc_new 失败（内存不足）");
        return;
    }
    if (quirc_resize(s_q, width, height) < 0) {
        ESP_LOGE(TAG, "quirc_resize %dx%d 失败（%dKB 缓冲分配失败）",
                 width, height, width * height / 1024);
        quirc_destroy(s_q);
        s_q = NULL;
    }
}

static void copy_payload(const struct quirc_data *data, qr_result_t *out)
{
    int len = data->payload_len;
    if (len > QR_TEXT_MAX - 1) {
        len = QR_TEXT_MAX - 1;
    }
    memcpy(out->text, data->payload, (size_t)len);
    out->text[len] = '\0';
    out->detected = true;
}

static void corner_center(const struct quirc_code *code, int w, int h, qr_result_t *out)
{
    int sx = 0, sy = 0;
    for (int i = 0; i < 4; i++) {
        sx += code->corners[i].x;
        sy += code->corners[i].y;
    }
    if (w > 0) out->x_ratio = (float)(sx / 4) / (float)w;
    if (h > 0) out->y_ratio = (float)(sy / 4) / (float)h;
}

bool qr_detect_process_gray(const uint8_t *gray, qr_result_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!s_q || !gray) return false;

    static int dbg_left = 2;   /* 前两次解码打阶段耗时（诊断 PSRAM 慢） */
    int64_t t0 = 0, t1 = 0, t2 = 0, t3 = 0;

    int w = 0, h = 0;
    uint8_t *buf = quirc_begin(s_q, &w, &h);
    if (!buf) return false;
    t0 = esp_timer_get_time();
    memcpy(buf, gray, (size_t)w * (size_t)h);
    t1 = esp_timer_get_time();
    quirc_end(s_q);
    t2 = esp_timer_get_time();

    const int n = quirc_count(s_q);
    t3 = esp_timer_get_time();
    if (dbg_left > 0) {
        dbg_left--;
        ESP_LOGW("qr_detect", "阶段耗时: memcpy=%lldms quirc_end=%lldms count=%lldms n=%d",
                 (long long)((t1 - t0) / 1000), (long long)((t2 - t1) / 1000),
                 (long long)((t3 - t2) / 1000), n);
    }
    out->count = n;
    if (n <= 0) return false;

    for (int i = 0; i < n; i++) {
        struct quirc_code code;
        struct quirc_data data;
        quirc_extract(s_q, i, &code);

        if (quirc_decode(&code, &data) == QUIRC_SUCCESS) {
            if (data.payload_len > 0) {
                copy_payload(&data, out);
                corner_center(&code, w, h, out);
                return true;
            }
        } else {
            /* ECC 失败：反相码原地翻转后重新解码 */
            quirc_flip(&code);
            if (quirc_decode(&code, &data) == QUIRC_SUCCESS && data.payload_len > 0) {
                copy_payload(&data, out);
                corner_center(&code, w, h, out);
                return true;
            }
        }
    }
    return false;
}
