#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_hci.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "motion.h"
#include "ble_control.h"
#include "bt_remote.h"

/* ============================================================
 * 蓝牙遥控功能：NimBLE NUS + 协议状态机 + 20ms 控制环
 *
 * 移植自 task3-ble-control 的 main.c（djj）+ control.c：
 *   - 生命周期与框架对齐：bt_remote_init() 只起协议栈；
 *     bt_remote_enter() 开始广播（只有蓝牙遥控模式才可被连接）；
 *     bt_remote_exit() 主动断开 + 停广播 + 撤 ARM + 断电。
 *   - 收到命令 → ble_control 状态机（ble_control.c）解析，
 *     输出 f/t/s 分量 → motion_drive()（motion.c 含标定混控）。
 *   - 电机使能 = 模式激活 && connected && armed（STBY 总开关）。
 *   - 并发：GATT 回调在 NimBLE 主机任务，控制环/模式切换在
 *     各自任务——共享 control_t 一律走临界区。
 * ============================================================ */

#define BT_TAG "bt_remote"

static bool s_active;             /* 是否处于蓝牙遥控模式（app_menu 驱动） */
static bool s_stack_synced;       /* NimBLE on_sync 是否完成 */
static control_t s_ctrl;          /* 协议/安全状态机 */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_tx_handle;      /* TX notify 特征句柄 */
static bool s_subscribed;         /* 手机是否订阅了 TX 通知 */
static uint8_t s_addr_type;
static uint16_t s_conn_handle;    /* 当前连接（BLE_HS_CONN_HANDLE_NONE=无） */

/* NimBLE UUID bytes are little endian. Nordic UART Service (NUS). */
#define NUS_UUID(n) BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,n,0x00,0x40,0x6e)
static const ble_uuid128_t svc_uuid = NUS_UUID(1);
static const ble_uuid128_t rx_uuid = NUS_UUID(2);
static const ble_uuid128_t tx_uuid = NUS_UUID(3);

/* ============================================================
 * GATT：RX 写 = 命令（文本 / !B 按键包），TX 读 = ARMED 状态
 * ============================================================ */
static int access_cb(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctx, void *arg)
{
    (void)attr; (void)arg;
    char text[32];
    if (ctx->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        portENTER_CRITICAL(&s_lock);
        bool armed = s_ctrl.armed;
        portEXIT_CRITICAL(&s_lock);
        const char *status = armed ? "ARMED" : "DISARMED";
        return os_mbuf_append(ctx->om, status, strlen(status)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    uint16_t len = 0;
    if (OS_MBUF_PKTLEN(ctx->om) >= sizeof(text) ||
        ble_hs_mbuf_to_flat(ctx->om, text, sizeof(text), &len) != 0) {
        portENTER_CRITICAL(&s_lock);
        (void)control_command(&s_ctrl, "STOP", 4, esp_timer_get_time()/1000);
        portEXIT_CRITICAL(&s_lock);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    portENTER_CRITICAL(&s_lock);
    const char *reply = control_command(&s_ctrl, text, len, esp_timer_get_time()/1000);
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(BT_TAG, "RX len=%u => %s", (unsigned)len, reply);
    if (s_subscribed) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(reply, strlen(reply));
        if (om) (void)ble_gatts_notify_custom(conn, s_tx_handle, om);
    }
    return 0;
}

static const struct ble_gatt_svc_def s_services[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &svc_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]) {
        {.uuid = &rx_uuid.u, .access_cb = access_cb,
         .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
        {.uuid = &tx_uuid.u, .access_cb = access_cb, .val_handle = &s_tx_handle,
         .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY}, {0}}}, {0}
};

/* ============================================================
 * 广播（只有蓝牙遥控模式激活时调用）
 * ============================================================ */
static int gap_event(struct ble_gap_event *event, void *arg);   /* 互引前置声明 */

static void advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&svc_uuid;
    fields.num_uuids128 = 1; fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc) { ESP_LOGE(BT_TAG, "adv fields %d", rc); return; }
    struct ble_hs_adv_fields scan = {0};
    scan.name = (uint8_t *)"MyCar-BLE"; scan.name_len = strlen("MyCar-BLE"); scan.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&scan);
    if (rc) { ESP_LOGE(BT_TAG, "scan fields %d", rc); return; }
    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    ESP_LOGI(BT_TAG, "Advertising MyCar-BLE: rc=%d", rc);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        portENTER_CRITICAL(&s_lock);
        control_link(&s_ctrl, event->connect.status == 0);
        portEXIT_CRITICAL(&s_lock);
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_subscribed = false;
            ESP_LOGI(BT_TAG, "CONNECT; 手机按 1(ARM) 后开始遥控");
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            if (s_active) advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        portENTER_CRITICAL(&s_lock);
        control_link(&s_ctrl, false);   /* 清运动 + 撤 ARM */
        portEXIT_CRITICAL(&s_lock);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed = false;
        ESP_LOGW(BT_TAG, "DISCONNECTED; disarmed");
        if (s_active) advertise();      /* 模式内断线：继续等重连 */
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_handle) {
            s_subscribed = event->subscribe.cur_notify;
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (s_active) advertise();
        break;
    default:
        break;
    }
    return 0;
}

/* ============================================================
 * NimBLE 主机回调 / 主机任务
 * ============================================================ */
static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0); assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &s_addr_type); assert(rc == 0);
    s_stack_synced = true;
    if (s_active) advertise();
}
static void on_reset(int reason)
{
    portENTER_CRITICAL(&s_lock);
    control_link(&s_ctrl, false);
    portEXIT_CRITICAL(&s_lock);
    s_subscribed = false;
    s_stack_synced = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    motion_stop();
    motion_enable(false);
    ESP_LOGE(BT_TAG, "BLE reset %d; disarmed", reason);
}
static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    on_reset(-1);
    nimble_port_freertos_deinit();
}

/* ============================================================
 * 20ms 控制环：租约检查 + 电机输出（仅模式激活期间工作）
 * ============================================================ */
static void ctrl_loop_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    bool prev_enabled = false;
    for (;;) {
        if (s_active) {
            portENTER_CRITICAL(&s_lock);
            bool expired = control_tick(&s_ctrl, esp_timer_get_time()/1000);
            control_t cur = s_ctrl;
            portEXIT_CRITICAL(&s_lock);
            bool want = cur.connected && cur.armed;
            if (want != prev_enabled) {
                motion_enable(want);            /* STBY 总开关 */
                prev_enabled = want;
            }
            if (!want || !(cur.f || cur.t || cur.s)) {
                motion_stop();                  /* 未使能/无动作：保持刹停 */
            } else {
                motion_drive(cur.f, cur.t, cur.s);
            }
            if (expired) ESP_LOGI(BT_TAG, "Lease expired: STOP");
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
    }
}

/* ============================================================
 * 框架接口：init / enter / exit / is_active
 * ============================================================ */
esp_err_t bt_remote_init(void)
{
    s_active = false;
    s_stack_synced = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_subscribed = false;
    control_link(&s_ctrl, false);

    /* NimBLE 协议栈（NVS 已可能被其他模块初始化，不在此强制擦除） */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(BT_TAG, "NVS 需要擦除（%s）——执行 erase_flash 或忽略", esp_err_to_name(err));
    }
    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(BT_TAG, "nimble_port_init: %s", esp_err_to_name(err));
        return err;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_svc_gap_device_name_set("MyCar-BLE");
    if (rc) { ESP_LOGE(BT_TAG, "device name %d", rc); return ESP_FAIL; }
    rc = ble_gatts_count_cfg(s_services); if (rc) { ESP_LOGE(BT_TAG, "count_cfg %d", rc); return ESP_FAIL; }
    rc = ble_gatts_add_svcs(s_services); if (rc) { ESP_LOGE(BT_TAG, "add_svcs %d", rc); return ESP_FAIL; }
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(host_task);

    if (xTaskCreatePinnedToCore(ctrl_loop_task, "bt_ctrl", 3072, NULL, 4, NULL, 1) != pdPASS) {
        ESP_LOGE(BT_TAG, "ctrl 任务创建失败");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(BT_TAG, "蓝牙遥控模块就绪（未广播；进入模式后才可被手机连接）");
    return ESP_OK;
}

esp_err_t bt_remote_enter(void)
{
    motion_enable(false);       /* 进入模式先断电，等手机 ARM */
    motion_stop();
    s_active = true;
    ESP_LOGI(BT_TAG, "★ 进入蓝牙遥控：广播 MyCar-BLE，等待手机连接");
    ESP_LOGI(BT_TAG, "  连接后：按 1 使能(ARM) → 方向键遥控；按 2 或说“选择功能”退出");
    if (s_stack_synced) {
        advertise();
    } else {
        ESP_LOGW(BT_TAG, "BLE 协议栈尚未同步，on_sync 后自动开始广播");
    }
    return ESP_OK;
}

void bt_remote_exit(void)
{
    if (!s_active) {
        return;
    }
    s_active = false;
    (void)ble_gap_adv_stop();                       /* 停止广播 */
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        (void)ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }
    portENTER_CRITICAL(&s_lock);
    control_link(&s_ctrl, false);                   /* 清运动 + 撤 ARM */
    portEXIT_CRITICAL(&s_lock);
    s_subscribed = false;
    motion_stop();
    motion_enable(false);                           /* 离开模式断电保护 */
    ESP_LOGI(BT_TAG, "已退出蓝牙遥控（广播已停，连接已断，电机断电）");
}

bool bt_remote_is_active(void)
{
    return s_active;
}

void bt_remote_link_state(bool *connected, bool *armed)
{
    portENTER_CRITICAL(&s_lock);
    if (connected) *connected = s_ctrl.connected;
    if (armed) *armed = s_ctrl.armed;
    portEXIT_CRITICAL(&s_lock);
}
