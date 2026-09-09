#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "motor.h"
#include "control.h"

static const char *TAG = "mycar_ble";
static control_t control;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t tx_handle;
static bool subscribed;
static uint8_t addr_type;
/* NimBLE UUID bytes are little endian. Nordic UART Service (NUS). */
#define NUS_UUID(n) BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,n,0x00,0x40,0x6e)
static const ble_uuid128_t svc_uuid = NUS_UUID(1);
static const ble_uuid128_t rx_uuid = NUS_UUID(2);
static const ble_uuid128_t tx_uuid = NUS_UUID(3);

static int access_cb(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctx, void *arg)
{
    (void)attr; (void)arg;
    char text[32];
    if (ctx->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        portENTER_CRITICAL(&lock);
        bool armed = control.armed;
        portEXIT_CRITICAL(&lock);
        const char *status = armed ? "ARMED" : "DISARMED";
        return os_mbuf_append(ctx->om, status, strlen(status)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    uint16_t len = 0;
    if (OS_MBUF_PKTLEN(ctx->om) >= sizeof(text) ||
        ble_hs_mbuf_to_flat(ctx->om, text, sizeof(text), &len) != 0) {
        portENTER_CRITICAL(&lock);
        (void)control_command(&control, "STOP", 4, esp_timer_get_time()/1000);
        portEXIT_CRITICAL(&lock);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    portENTER_CRITICAL(&lock);
    const char *reply = control_command(&control, text, len, esp_timer_get_time()/1000);
    portEXIT_CRITICAL(&lock);
    ESP_LOGI(TAG, "RX len=%u => %s", (unsigned)len, reply);
    if (subscribed) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(reply, strlen(reply));
        if (om) (void)ble_gatts_notify_custom(conn, tx_handle, om);
    }
    return 0;
}

static const struct ble_gatt_svc_def services[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &svc_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]) {
        {.uuid = &rx_uuid.u, .access_cb = access_cb,
         .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
        {.uuid = &tx_uuid.u, .access_cb = access_cb, .val_handle = &tx_handle,
         .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY}, {0}}}, {0}
};

static void advertise(void);
static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        portENTER_CRITICAL(&lock);
        control_link(&control, event->connect.status == 0);
        portEXIT_CRITICAL(&lock);
        subscribed = false;
        ESP_LOGI(TAG, "CONNECT status=%d; send ARM before motion", event->connect.status);
        if (event->connect.status) advertise();
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        portENTER_CRITICAL(&lock);
        control_link(&control, false);
        portEXIT_CRITICAL(&lock);
        subscribed = false;
        ESP_LOGW(TAG, "DISCONNECTED; disarmed");
        advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == tx_handle) subscribed = event->subscribe.cur_notify;
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;
    default: break;
    }
    return 0;
}

static void advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&svc_uuid;
    fields.num_uuids128 = 1; fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc) { ESP_LOGE(TAG, "adv fields %d", rc); return; }
    struct ble_hs_adv_fields scan = {0};
    scan.name = (uint8_t *)"MyCar-BLE"; scan.name_len = strlen("MyCar-BLE"); scan.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&scan);
    if (rc) { ESP_LOGE(TAG, "scan fields %d", rc); return; }
    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    ESP_LOGI(TAG, "Advertising MyCar-BLE: rc=%d", rc);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0); assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &addr_type); assert(rc == 0);
    advertise();
}
static void on_reset(int reason)
{
    portENTER_CRITICAL(&lock);
    control_link(&control, false);
    portEXIT_CRITICAL(&lock);
    subscribed = false;
    ESP_LOGE(TAG, "BLE reset %d; disarmed", reason);
}
static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    on_reset(-1);
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    motor_init(); /* STBY remains low throughout BLE initialization. */
    ESP_ERROR_CHECK(nvs_flash_init()); /* Never erase existing NVS silently. */
    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init(); ble_svc_gatt_init();
    int rc = ble_svc_gap_device_name_set("MyCar-BLE"); assert(rc == 0);
    rc = ble_gatts_count_cfg(services); assert(rc == 0);
    rc = ble_gatts_add_svcs(services); assert(rc == 0);
    ble_hs_cfg.sync_cb = on_sync; ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(host_task);
    TickType_t wake = xTaskGetTickCount();
    bool enabled = false;
    while (true) {
        portENTER_CRITICAL(&lock);
        bool expired = control_tick(&control, esp_timer_get_time()/1000);
        control_t current = control;
        portEXIT_CRITICAL(&lock);
        bool want_enabled = current.connected && current.armed;
        if (enabled != want_enabled) { motor_enable(want_enabled); enabled = want_enabled; }
        if (!want_enabled || !(current.f || current.t || current.s)) motor_stop_all();
        else {
            float wheels[3]; control_mix(current.f, current.t, current.s, wheels);
            for (int i = 0; i < 3; ++i) motor_set_speed(i, wheels[i]);
        }
        if (expired) ESP_LOGI(TAG, "Lease expired: STOP");
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
    }
}
