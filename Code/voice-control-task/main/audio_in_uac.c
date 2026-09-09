#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"

#include "usb/usb_host.h"
#include "usb/usb_types_stack.h"
#include "usb/usb_types_ch9.h"

#include "audio_in.h"

/* ============================================================
 * UAC 麦克风输入（USB Host + 等时传输）—— audio_in.h 的 USB 实现
 *
 * 硬件：JQ-CAM12-720P USB 三合一摄像头。音频与视频共用同一根 USB 线
 *       （UVC + UAC 复合设备，其他组已实现），故语音输入不再需要
 *       I2S/麦克风引脚——屏幕/编码器等接线全部原样保留。
 *
 * 数据流：
 *   USB 设备 --isoc IN--> xfer 完成回调
 *       → 声道混合 + 整数倍降采样(48k/32k→16k) → 16k mono FIFO → audio_in_read()
 *
 * 结构（IDF usb_host 异步 client 模型）：
 *   - usb_lib_task   : usb_host_install + usb_host_lib_handle_events 循环
 *   - uac_client_task: usb_host_client_handle_events 循环；client 事件回调与
 *                      isoc xfer 完成回调均在该任务上下文执行，禁止阻塞。
 *
 * ⚠️ 设备未接入时 audio_in_read() 返回 ESP_ERR_TIMEOUT（feed 任务延时重试），
 *    不伪造音频；接入后自动起流。支持 16k/32k/48kHz、1/2 声道、16bit。
 * ============================================================ */

#define TAG "audio_uac"

#define UAC_AS_SUBCLASS  0x02          /* AudioStreaming */
#define URB_COUNT        3             /* 常驻等时传输数 */
#define PKT_PER_URB      8             /* 每 transfer 等时包数（1 包 ≈ 1ms） */
#define UAC_MAX_MPS      512
#define OUT_FIFO_CAP     4096          /* 16k mono 输出 FIFO（样本数） */
/* 软件输入增益（整数倍，带饱和限幅）。UAC 麦克风电平过低导致识别
 * 不到时上调（如 4、8）；看串口 "MIC LEVEL 峰值" 日志决定：
 * 说话峰值应在 2000~30000。默认 1=不改幅度。 */
#define UAC_INPUT_GAIN   1

/* ---------------- 输出 FIFO（SPSC：client 任务写 / feed 任务读） ---------------- */
static int16_t s_out[OUT_FIFO_CAP];
static volatile uint32_t s_head = 0, s_tail = 0;   /* 游标为单调计数，差值即存量 */
static SemaphoreHandle_t s_out_sem = NULL;         /* 计数信号量：每批写入给 1 */

static void fifo_reset(void)
{
    s_head = s_tail = 0;
}

static void fifo_write(const int16_t *data, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        if (s_head - s_tail >= OUT_FIFO_CAP) {
            return;                     /* 满：丢弃本批（丢音频远好于阻塞 USB 回调） */
        }
        s_out[s_head % OUT_FIFO_CAP] = data[i];
        s_head++;
    }
}

/* ---------------- 降采样（声道混合 + 16k 整数倍盒式平均） ---------------- */
typedef struct {
    uint32_t ch;
    uint32_t decim;                     /* in_rate/16000：0=未就绪 */
    int16_t carry[4];
    uint32_t carry_n;
} resampler_t;
static resampler_t s_rs;

static void resampler_reset(void)
{
    memset(&s_rs, 0, sizeof(s_rs));
}

static void resample_push(const int16_t *pcm, uint32_t bytes)
{
    uint32_t frames = bytes / 2 / s_rs.ch;
    if (s_rs.decim == 0 || frames == 0) {
        return;
    }
    if (frames > 512) frames = 512;     /* 防御 */

    /* 1. 声道混合 → 单声道（带软件增益与饱和限幅） */
    int16_t mono[512];
    for (uint32_t i = 0; i < frames; i++) {
        int32_t v = (s_rs.ch == 2)
            ? ((int32_t)pcm[i * 2] + pcm[i * 2 + 1]) / 2
            : (int32_t)pcm[i];
        if (UAC_INPUT_GAIN > 1) {
            v *= UAC_INPUT_GAIN;
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
        }
        mono[i] = (int16_t)v;
    }

    /* 2. 拼接上批余数 */
    int16_t buf[600];
    uint32_t bn = s_rs.carry_n;
    memcpy(buf, s_rs.carry, bn * sizeof(int16_t));
    memcpy(buf + bn, mono, frames * sizeof(int16_t));
    uint32_t total = bn + frames;

    /* 3. decim 点盒式平均降采样（轻抗混叠） */
    int16_t out[512];
    uint32_t on = 0;
    uint32_t i = 0;
    for (; i + s_rs.decim <= total; i += s_rs.decim) {
        int32_t acc = 0;
        for (uint32_t j = 0; j < s_rs.decim; j++) {
            acc += buf[i + j];
        }
        if (on < 512) out[on++] = (int16_t)(acc / (int32_t)s_rs.decim);
    }
    s_rs.carry_n = total - i;
    memcpy(s_rs.carry, buf + i, s_rs.carry_n * sizeof(int16_t));

    if (on) {
        fifo_write(out, on);
        xSemaphoreGive(s_out_sem);      /* 消费方仅在 FIFO 空时等待，一批给 1 */
    }
}

/* ---------------- USB 会话状态 ---------------- */
typedef enum {
    UAC_IDLE = 0,
    UAC_OPENING,
    UAC_STREAMING,
} uac_state_t;
static uac_state_t s_state = UAC_IDLE;
static uint8_t s_pending_addr = 0;      /* 事件回调 → client 任务 */
static usb_host_client_handle_t s_client = NULL;
static usb_device_handle_t s_dev = NULL;
static usb_transfer_t *s_xfers[URB_COUNT];
static uint32_t s_xfer_n = 0;
static SemaphoreHandle_t s_install_done = NULL;   /* USB Host 库安装完成（注册前先等） */

/* ============================================================
 * isoc 完成回调（client 任务上下文；只做拷贝 + 重投）
 * ============================================================ */
static uint32_t s_raw_bytes = 0, s_raw_nonzero = 0;   /* 原始包诊断 */

static void isoc_cb(usb_transfer_t *xfer)
{
    if (s_state != UAC_STREAMING || xfer->status == USB_TRANSFER_STATUS_NO_DEVICE) {
        return;                         /* 拆除中：不重投（对象由驱动回收） */
    }
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        int offset = 0;
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            const usb_isoc_packet_desc_t *pkt = &xfer->isoc_packet_desc[i];
            if (pkt->status == USB_TRANSFER_STATUS_COMPLETED && pkt->actual_num_bytes > 0) {
                const int16_t *pc = (const int16_t *)(xfer->data_buffer + offset);
                uint32_t n = pkt->actual_num_bytes / 2;
                s_raw_bytes += pkt->actual_num_bytes;
                for (uint32_t k = 0; k < n; k++) {
                    if (pc[k] != 0) s_raw_nonzero++;
                }
                resample_push(pc, pkt->actual_num_bytes);
            }
            offset += pkt->num_bytes;
        }
    } else if (xfer->status != USB_TRANSFER_STATUS_CANCELED &&
               xfer->status != USB_TRANSFER_STATUS_SKIPPED) {
        ESP_LOGW(TAG, "isoc xfer status=%d", xfer->status);
    }
    esp_err_t err = usb_host_transfer_submit(xfer);   /* 立即重投保持流连续 */
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "isoc resubmit: %s", esp_err_to_name(err));
    }
}

/* ---------------- 配置描述符解析 ---------------- */
typedef struct {
    uint8_t iface_num, alt, ep_addr;
    uint16_t mps;
    uint32_t rate;
    uint8_t ch;
    uint8_t subslot;        /* bSubslotSize（字节/样本，仅 2=16bit 支持） */
} uac_ep_t;

/* 解析接口描述符之后的类特定格式描述符（CS_INTERFACE 0x24 / FORMAT_TYPE 0x02） */
static void parse_as_format(const uint8_t *p, const uint8_t *end, uac_ep_t *out)
{
    while (p + 2 <= end) {
        uint8_t len = p[0];
        if (len < 2 || p + len > end) break;
        if (len >= 8 && p[1] == 0x24 && p[2] == 0x02 && p[3] == 0x01) {
            out->ch = p[4];
            out->subslot = p[5];    /* bSubslotSize */
            uint8_t freq_type = p[7];
            const uint8_t *f = p + 8;
            if (freq_type == 0 && len >= 14) {
                out->rate = f[0] | (f[1] << 8) | (f[2] << 16);   /* 连续范围取 min */
            } else if (len >= (uint32_t)(8 + 3 * freq_type)) {
                out->rate = f[0] | (f[1] << 8) | (f[2] << 16);   /* 离散表取第一个 */
            }
            return;
        }
        p += len;
    }
}

/* ============================================================
 * UAC 主机侧控制：解除 Feature Unit 静音
 *
 * 不少 UAC 麦克风默认处于 MUTE=1（主机不设置就一直无声流），
 * 这是"有流但识别不到"的最常见原因。此处在起流前对配置描述符里
 * 找到的所有 Feature Unit（CS_INTERFACE subtype 0x06）发
 * SET_CUR(MUTE)=0；设备不支持会 STALL，容错忽略。
 * ============================================================ */
static SemaphoreHandle_t s_ctrl_sem = NULL;
static usb_transfer_t *s_ctrl_orphan = NULL;   /* 超时后由回调代为释放 */

static void ctrl_cb(usb_transfer_t *xfer)
{
    if (s_ctrl_orphan == xfer) {
        s_ctrl_orphan = NULL;
        usb_host_transfer_free(xfer);
        return;
    }
    xSemaphoreGive(s_ctrl_sem);
}

/* 发一个类请求；失败只告警一次（有些 FU 不支持 volume 等） */
static void uac_ctrl_set_cur(uint8_t cs, uint8_t cn, uint8_t unit_id,
                             uint8_t ac_iface, const uint8_t *data, uint16_t len)
{
    usb_transfer_t *xfer = NULL;
    if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + len, 0, &xfer) != ESP_OK) {
        return;
    }
    usb_setup_packet_t *setup = (usb_setup_packet_t *)xfer->data_buffer;
    setup->bmRequestType = 0x21;        /* 主机→设备 | 类 | 接口 */
    setup->bRequest = 0x01;             /* SET_CUR */
    setup->wValue = ((uint16_t)cs << 8) | cn;
    setup->wIndex = ((uint16_t)unit_id << 8) | ac_iface;
    setup->wLength = len;
    memcpy(xfer->data_buffer + sizeof(usb_setup_packet_t), data, len);
    xfer->num_bytes = sizeof(usb_setup_packet_t) + len;
    xfer->device_handle = s_dev;
    xfer->bEndpointAddress = 0;
    xfer->callback = ctrl_cb;
    xfer->timeout_ms = 0;
    xSemaphoreTake(s_ctrl_sem, 0);      /* 清残留计数 */
    if (usb_host_transfer_submit_control(s_client, xfer) == ESP_OK) {
        /* ⚠️ 必须边泵 handle_events 边等待：完成回调在本 client 任务
         * 分发，纯 xSemaphoreTake 会把循环停住导致永远超时。 */
        int waited = 0;
        while (xSemaphoreTake(s_ctrl_sem, pdMS_TO_TICKS(10)) != pdTRUE) {
            usb_host_client_handle_events(s_client, pdMS_TO_TICKS(5));
            if (++waited >= 80) {        /* ~800ms 超时 */
                ESP_LOGW(TAG, "UAC SET_CUR(CS=%u unit=%u) 超时（可能不支持）", cs, unit_id);
                s_ctrl_orphan = xfer;    /* 回调迟到时代为释放 */
                return;
            }
        }
        if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
            ESP_LOGW(TAG, "UAC SET_CUR(CS=%u unit=%u) status=%d（STALL=不支持，忽略）",
                     cs, unit_id, xfer->status);
        } else {
            ESP_LOGI(TAG, "UAC SET_CUR(CS=%u unit=%u) OK", cs, unit_id);
        }
    }
    usb_host_transfer_free(xfer);
}

/* 扫描配置描述符，对每个 AudioControl 下的 Feature Unit 解除静音 */
static void uac_unmute_all(const usb_config_desc_t *cfg)
{
    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    uint8_t ac_iface = 0;
    bool in_ac = false;
    while (p + 2 <= end) {
        uint8_t len = p[0];
        if (len < 2 || p + len > end) break;
        if (p[1] == USB_W_VALUE_DT_INTERFACE && len >= 5) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;
            in_ac = (intf->bInterfaceClass == 1 && intf->bInterfaceSubClass == 0x01);
            if (in_ac) ac_iface = intf->bInterfaceNumber;
        } else if (in_ac && p[1] == 0x24 && len >= 4 && p[2] == 0x06) {
            uint8_t unit_id = p[3];
            ESP_LOGI(TAG, "发现 Feature Unit id=%u（AC iface %u），解除静音", unit_id, ac_iface);
            const uint8_t zero = 0;
            uac_ctrl_set_cur(0x01, 0x00, unit_id, ac_iface, &zero, 1);   /* MUTE=0 */
        }
        p += len;
    }
}

/* 查找并启动 UAC 麦克风流；成功返回 ESP_OK 且 state=STREAMING */
static esp_err_t uac_find_and_start(usb_device_handle_t dev)
{
    const usb_config_desc_t *cfg = NULL;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(dev, &cfg));
    const uint8_t *p = (const uint8_t *)cfg + cfg->bLength;
    const uint8_t *end = (const uint8_t *)cfg + cfg->wTotalLength;
    uac_ep_t best = {0};
    int best_prio = 0;

    while (p + (int)sizeof(usb_intf_desc_t) <= end) {
        const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;
        if (intf->bDescriptorType != USB_W_VALUE_DT_INTERFACE) {
            p += intf->bLength;
            continue;
        }
        /* 诊断：打印全部接口的类/子类，便于确认模块是否带 UAC */
        ESP_LOGI(TAG, "iface %u alt %u: class=%u subclass=%u eps=%u",
                 intf->bInterfaceNumber, intf->bAlternateSetting,
                 intf->bInterfaceClass, intf->bInterfaceSubClass,
                 intf->bNumEndpoints);
        if (intf->bInterfaceClass == 1 && intf->bInterfaceSubClass == UAC_AS_SUBCLASS) {
            uac_ep_t cand = {.iface_num = intf->bInterfaceNumber,
                             .alt = intf->bAlternateSetting};
            const uint8_t *q = p + intf->bLength;
            parse_as_format(q, end, &cand);
            ESP_LOGI(TAG, "  AS iface %u alt %u: rate=%luHz ch=%u subslot=%u",
                     cand.iface_num, cand.alt, (unsigned long)cand.rate, cand.ch, cand.subslot);
            /* 端点前可能还有多个类特定(0x24/0x25)描述符：按“数够
             * bNumEndpoints 个端点”来扫，而不是按固定迭代次数 */
            uint32_t eps_seen = 0;
            while (q + 2 <= end && eps_seen < intf->bNumEndpoints) {
                uint8_t dlen = q[0];
                if (dlen < 2 || q + dlen > end) break;
                if (q[1] == USB_W_VALUE_DT_ENDPOINT) {
                    const usb_ep_desc_t *ep = (const usb_ep_desc_t *)q;
                    eps_seen++;
                    ESP_LOGI(TAG, "  ep: addr=0x%02x attr=0x%02x mps=%u",
                             ep->bEndpointAddress, ep->bmAttributes, ep->wMaxPacketSize);
                    bool is_in = (ep->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) != 0;
                    if ((ep->bmAttributes & USB_BM_ATTRIBUTES_XFER_ISOC) && is_in &&
                        ep->wMaxPacketSize > 0 && ep->wMaxPacketSize <= UAC_MAX_MPS) {
                        /* 仅接受 16bit（bSubslotSize=2）；24bit 另行适配 */
                        if (cand.subslot != 0 && cand.subslot != 2) {
                            ESP_LOGW(TAG, "  跳过 %ubit 流（仅支持 16bit）", cand.subslot * 8);
                        } else {
                        cand.ep_addr = ep->bEndpointAddress;
                        cand.mps = ep->wMaxPacketSize;
                        /* 优先级：48k > 16k > 32k > 其他（仅首个等优先级生效） */
                        uint32_t r = cand.rate;
                        int prio = (r == 48000) ? 4 : (r == 16000) ? 3 : (r == 32000) ? 2 : 1;
                        if (prio > best_prio) {
                            best = cand;
                            best_prio = prio;
                        }
                        }
                    }
                }
                q += dlen;
            }
        }
        p += intf->bLength;
    }

    if (best_prio == 0 || best.mps == 0) {
        ESP_LOGW(TAG, "未找到 UAC 麦克风 isoc IN 接口（确认是 UVC+UAC 复合摄像头）");
        usb_host_free_config_desc(cfg);
        return ESP_ERR_NOT_FOUND;
    }
    if (best.ch == 0) best.ch = 2;
    if (best.rate == 0) best.rate = 48000;
    if (best.rate % 16000 != 0 || best.rate < 16000 || best.rate > 48000) {
        ESP_LOGW(TAG, "采样率 %luHz 不支持（仅 16k/32k/48k）", (unsigned long)best.rate);
        usb_host_free_config_desc(cfg);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (best.ch != 1 && best.ch != 2) {
        ESP_LOGW(TAG, "声道数 %u 不支持（仅 1/2）", best.ch);
        usb_host_free_config_desc(cfg);
        return ESP_ERR_NOT_SUPPORTED;
    }
    ESP_LOGI(TAG, "UAC 麦克风: iface=%u alt=%u ep=0x%02x MPS=%u rate=%luHz ch=%u",
             best.iface_num, best.alt, best.ep_addr, best.mps,
             (unsigned long)best.rate, best.ch);

    esp_err_t err = usb_host_interface_claim(s_client, dev, best.iface_num, best.alt);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "interface_claim: %s", esp_err_to_name(err));
        usb_host_free_config_desc(cfg);
        return err;
    }
    uac_unmute_all(cfg);        /* 起流前解除 Feature Unit 静音（容错） */
    usb_host_free_config_desc(cfg);

    resampler_reset();
    s_rs.ch = best.ch;
    s_rs.decim = best.rate / 16000;

    /* 分配并投递等时传输 */
    size_t urb_bytes = (size_t)best.mps * PKT_PER_URB;
    s_xfer_n = 0;
    for (int i = 0; i < URB_COUNT; i++) {
        usb_transfer_t *xfer = NULL;
        err = usb_host_transfer_alloc(urb_bytes, PKT_PER_URB, &xfer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "transfer_alloc[%d]: %s", i, esp_err_to_name(err));
            break;
        }
        xfer->device_handle = dev;
        xfer->bEndpointAddress = best.ep_addr;
        xfer->callback = isoc_cb;
        xfer->num_bytes = (int)urb_bytes;
        xfer->timeout_ms = 0;
        for (int k = 0; k < PKT_PER_URB; k++) {
            xfer->isoc_packet_desc[k].num_bytes = best.mps;
        }
        err = usb_host_transfer_submit(xfer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "transfer_submit[%d]: %s", i, esp_err_to_name(err));
            usb_host_transfer_free(xfer);
            break;
        }
        s_xfers[s_xfer_n++] = xfer;
    }
    if (s_xfer_n == 0) {
        usb_host_interface_release(s_client, dev, best.iface_num);
        return ESP_FAIL;
    }

    fifo_reset();
    s_state = UAC_STREAMING;
    ESP_LOGI(TAG, "★ UAC 麦克风启动: %luHz %uch 16bit → 16kHz mono", (unsigned long)best.rate, best.ch);
    return ESP_OK;
}

/* ============================================================
 * client 事件回调（client 任务上下文；只置标志，不做阻塞调用）
 * ============================================================ */
static void client_evt_cb(const usb_host_client_event_msg_t *ev, void *arg)
{
    (void)arg;
    switch (ev->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        if (s_state == UAC_STREAMING) {
            ESP_LOGW(TAG, "已有 UAC 流，忽略新设备 addr=%u", ev->new_dev.address);
            break;
        }
        ESP_LOGI(TAG, "新 USB 设备 addr=%u，尝试作为 UAC 麦克风打开", ev->new_dev.address);
        s_pending_addr = ev->new_dev.address;
        s_state = UAC_OPENING;
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGW(TAG, "USB 设备断开，麦克风停止（等待重新接入）");
        s_state = UAC_IDLE;
        s_dev = NULL;
        s_xfer_n = 0;                   /* 等时 xfer 由驱动随设备回收，勿再触碰 */
        resampler_reset();
        fifo_reset();
        break;
    default:
        break;
    }
}

/* ============================================================
 * client 事件任务：事件循环 + 打开/启动设备
 * ============================================================ */
static void uac_client_task(void *arg)
{
    (void)arg;
    /* 等 USB Host 库安装完成，避免 register 竞态（ESP_ERR_INVALID_STATE） */
    xSemaphoreTake(s_install_done, portMAX_DELAY);

    usb_host_client_config_t cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 4,
        .async = {.client_event_callback = client_evt_cb, .callback_arg = NULL},
    };
    esp_err_t err;
    do {
        err = usb_host_client_register(&cfg, &s_client);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "client_register: %s，重试中", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } while (err != ESP_OK);
    ESP_LOGI(TAG, "UAC client 就绪：等待摄像头（USB 三合一）接入");

    /* 补扫注册前已枚举的设备（注册不会重放历史 NEW_DEV 事件） */
    uint8_t addrs[8] = {0};
    int n = 0;
    if (usb_host_device_addr_list_fill(8, addrs, &n) == ESP_OK) {
        for (int i = 0; i < n; i++) {
            if (addrs[i] && s_state != UAC_STREAMING) {
                ESP_LOGI(TAG, "发现已接入设备 addr=%u", addrs[i]);
                s_pending_addr = addrs[i];
                s_state = UAC_OPENING;
            }
        }
    }

    while (1) {
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(50));

        /* 原始包诊断（每 2s）：bytes=收到的字节数 nonzero=非零样本数。
         * bytes 在涨且 nonzero>0 → 设备有声；
         * bytes 在涨但 nonzero=0 → 设备在推静音（麦克风通路问题）。 */
        static uint32_t last = 0;
        if (s_state == UAC_STREAMING && s_raw_bytes != last &&
            (xTaskGetTickCount() & 0x3ff) < 50) {
            ESP_LOGI(TAG, "MIC RAW: bytes=%lu nonzero=%lu",
                     (unsigned long)s_raw_bytes, (unsigned long)s_raw_nonzero);
            last = s_raw_bytes;
            s_raw_bytes = 0;
            s_raw_nonzero = 0;
        }

        if (s_state == UAC_OPENING && s_pending_addr) {
            uint8_t addr = s_pending_addr;
            s_pending_addr = 0;
            err = usb_host_device_open(s_client, addr, &s_dev);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "device_open: %s", esp_err_to_name(err));
                s_state = UAC_IDLE;
                continue;
            }
            err = uac_find_and_start(s_dev);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "该设备无可用 UAC 麦克风（%s）", esp_err_to_name(err));
                usb_host_device_close(s_client, s_dev);
                s_dev = NULL;
                s_state = UAC_IDLE;
            }
        }
    }
}

/* ============================================================
 * USB Host 库事件任务
 * ============================================================ */
static void usb_lib_task(void *arg)
{
    (void)arg;
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    ESP_LOGI(TAG, "USB Host 已安装");
    xSemaphoreGive(s_install_done);             /* 通知 client 任务可注册 */
    while (1) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

/* ============================================================
 * audio_in 接口
 * ============================================================ */
esp_err_t audio_in_init(void)
{
    if (s_out_sem) {
        return ESP_OK;
    }
    s_out_sem = xSemaphoreCreateCounting(OUT_FIFO_CAP, 0);
    s_install_done = xSemaphoreCreateBinary();
    s_ctrl_sem = xSemaphoreCreateBinary();
    fifo_reset();
    resampler_reset();

    if (xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL, 0) != pdPASS ||
        xTaskCreatePinnedToCore(uac_client_task, "uac_client", 4096, NULL, 8, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "任务创建失败");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "UAC 麦克风初始化完成：摄像头接入后自动出流");
    return ESP_OK;
}

esp_err_t audio_in_read(int16_t *buf, size_t samples)
{
    if (!s_out_sem) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state != UAC_STREAMING) {
        return ESP_ERR_TIMEOUT;         /* 无流：不伪造音频，上层延时重试 */
    }
    size_t got = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(60);
    while (got < samples) {
        uint32_t avail = (uint32_t)(s_head - s_tail);
        if (avail == 0) {
            if (xTaskGetTickCount() >= deadline) {
                break;                  /* 等流超时：剩余补零返回 */
            }
            xSemaphoreTake(s_out_sem, pdMS_TO_TICKS(5));
            continue;
        }
        uint32_t n = avail;
        if (n > samples - got) n = (uint32_t)(samples - got);
        for (uint32_t i = 0; i < n; i++) {
            buf[got + i] = s_out[s_tail % OUT_FIFO_CAP];
            s_tail++;
        }
        got += n;
    }
    if (got < samples) {
        memset(buf + got, 0, (samples - got) * sizeof(int16_t));
    }
    return ESP_OK;
}

bool audio_in_is_streaming(void)
{
    return s_state == UAC_STREAMING;
}
