// main.c

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

static const char *TAG = "BLE_FALL";

// ——— Your 128-bit UUIDs ———
static const ble_uuid128_t fall_service_uuid = BLE_UUID128_INIT(
    0xfb,0x34,0x9b,0x5f, 0x80,0x00,0x00,0x80,
    0x00,0x10,0x00,0x00, 0x00,0xff,0x00,0x00);

static const ble_uuid128_t fall_char_uuid = BLE_UUID128_INIT(
    0xfb,0x34,0x9b,0x5f, 0x80,0x00,0x00,0x80,
    0x00,0x10,0x00,0x00, 0x01,0xff,0x00,0x00);

// Notification payload
static uint8_t fall_val[] = "FALL_DETECTED";

// Value handle for the characteristic
static uint16_t fall_val_handle;

// ——— GATT access callback ———
static int
gatt_svr_chr_access(uint16_t conn_handle,
                    uint16_t attr_handle,
                    struct ble_gatt_access_ctxt *ctxt,
                    void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        // Respond with our FALL_DETECTED string
        os_mbuf_append(ctxt->om, fall_val, sizeof(fall_val)-1);
        return 0;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// ——— GATT server table ———
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        // Primary Service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &fall_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &fall_char_uuid.u,
            .access_cb = gatt_svr_chr_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &fall_val_handle,
            .descriptors = (struct ble_gatt_dsc_def[]) { {
                .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16),
                .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
                .access_cb = NULL, // default CCCD handling
            }, {
                0 // end
            } },
        }, {
            0 // end
        } },
    },
    {
        0 // end of services
    },
};

// ——— GAP event callback ———
static int
ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Client connected");
        } else {
            ESP_LOGW(TAG, "Connection failed; restarting adv");
            ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                              (struct ble_gap_adv_params *)arg,
                              ble_gap_event, arg);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Client disconnected; reason=%d", event->disconnect.reason);
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                          (struct ble_gap_adv_params *)arg,
                          ble_gap_event, arg);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertisement complete; restarting");
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                          (struct ble_gap_adv_params *)arg,
                          ble_gap_event, arg);
        break;

    default:
        break;
    }
    return 0;
}

// ——— Start advertising ———
static void
ble_app_advertise(void)
{
    struct ble_gap_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    // Flags: no BR/EDR
    fields.flags = BLE_HS_ADV_F_BREDR_UNSUP;

    // TX power auto
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // Advertised name
    fields.name = (uint8_t *)"FALL_ALERT";
    fields.name_len = strlen("FALL_ALERT");
    fields.name_is_complete = 1;

    // Advertise your custom service UUID
    fields.uuids128 = (ble_uuid128_t *)&fall_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    static struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC,
                      NULL,
                      BLE_HS_FOREVER,
                      &adv_params,
                      ble_gap_event,
                      &adv_params);
}

// ——— BLE host sync callback ———
static void
ble_app_on_sync(void)
{
    // Use public address
    ble_hs_id_infer_auto(0, NULL);

    // Init built-in services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Register our service definitions
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    // Start advertising
    ble_app_advertise();
}

// ——— app_main ———
void
app_main(void)
{
    // Initialize NimBLE
    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());
    nimble_port_init();

    // Configure host
    ble_hs_cfg.reset_cb = NULL;
    ble_hs_cfg.sync_cb  = ble_app_on_sync;

    // Run the BLE host task
    nimble_port_freertos_init(NULL);
}

// ——— helper to broadcast a fall event ———
void notify_fall_detected(void)
{
    if (ble_gap_conn_active_count() == 0) {
        ESP_LOGI(TAG, "No client connected; skipping notify");
        return;
    }
    // Notify all subscribers of the characteristic
    ble_gatts_chr_updated_all(fall_val_handle);
    ESP_LOGI(TAG, "Notified FALL_DETECTED");
}
