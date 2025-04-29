#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"        // for ESP_GATT_CLIENT_CHAR_CFG_NOTIFY
#include "esp_gatt_common_api.h"

#define TAG                 "BLE_FALL"
#define DEVICE_NAME         "ESP32_C3_BLE"

// 16-bit UUIDs matching the React Native 'ff00'/'ff01'
#define SERVICE_UUID        0xFF00
#define CHAR_UUID           0xFF01
static const uint16_t CLIENT_CHAR_CFG_UUID = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

// Attribute indices
enum {
    IDX_SVC,
    IDX_CHAR,
    IDX_CHAR_VAL,
    IDX_CHAR_CCC,
    IDX_NB
};
static uint16_t handles[IDX_NB];
static uint16_t conn_id = 0;
static esp_gatt_if_t gatts_if_global = 0;
static uint16_t ccc_value = 0;   // tracks CCC descriptor writes

// Advertising data including 16-bit service UUID
static uint16_t adv_service_uuid = SERVICE_UUID;
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .service_uuid_len    = sizeof(adv_service_uuid),
    .p_service_uuid      = (uint8_t*)&adv_service_uuid,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// Advertising parameters (100-150 ms)
static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x00A0,
    .adv_int_max         = 0x00F0,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GAP event handler: start advertising when setup completes
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&adv_params);
    }
}

// GATT server event handler
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "REGISTER_APP status=%d app_id=%d", param->reg.status, param->reg.app_id);
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);

        // Create primary service
        esp_gatt_srvc_id_t svc_id = {0};
        svc_id.is_primary = true;
        svc_id.id.inst_id = 0;
        svc_id.id.uuid.len = ESP_UUID_LEN_16;
        svc_id.id.uuid.uuid.uuid16 = SERVICE_UUID;
        esp_ble_gatts_create_service(gatts_if, &svc_id, IDX_NB);
        break;
    }
    case ESP_GATTS_CREATE_EVT: {
        ESP_LOGI(TAG, "CREATE_SERVICE status=%d handle=%d", param->create.status, param->create.service_handle);
        handles[IDX_SVC] = param->create.service_handle;

        // Add characteristic: READ + NOTIFY
        esp_bt_uuid_t char_uuid = {0};
        char_uuid.len = ESP_UUID_LEN_16;
        char_uuid.uuid.uuid16 = CHAR_UUID;
        esp_ble_gatts_add_char(
            handles[IDX_SVC],
            &char_uuid,
            ESP_GATT_PERM_READ,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            NULL, NULL);
        break;
    }
    case ESP_GATTS_ADD_CHAR_EVT: {
        ESP_LOGI(TAG, "ADD_CHAR status=%d handle=%d", param->add_char.status, param->add_char.attr_handle);
        handles[IDX_CHAR] = param->add_char.attr_handle;

        // Add CCC descriptor (0x2902)
        esp_bt_uuid_t descr_uuid = {0};
        descr_uuid.len = ESP_UUID_LEN_16;
        descr_uuid.uuid.uuid16 = CLIENT_CHAR_CFG_UUID;
        esp_ble_gatts_add_char_descr(
            handles[IDX_SVC],
            &descr_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            NULL, NULL);
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        ESP_LOGI(TAG, "ADD_DESCR status=%d handle=%d", param->add_char_descr.status, param->add_char_descr.attr_handle);
        handles[IDX_CHAR_CCC] = param->add_char_descr.attr_handle;
        esp_ble_gatts_start_service(handles[IDX_SVC]);
        break;
    }
    case ESP_GATTS_CONNECT_EVT: {
        ESP_LOGI(TAG, "CONNECT conn_id=%d if=%d", param->connect.conn_id, gatts_if);
        conn_id = param->connect.conn_id;
        gatts_if_global = gatts_if;
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        ESP_LOGI(TAG, "DISCONNECT conn_id=%d", param->disconnect.conn_id);
        conn_id = 0;
        ccc_value = 0;
        esp_ble_gap_start_advertising(&adv_params);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        // Client wrote to CCC descriptor?
        if (param->write.handle == handles[IDX_CHAR_CCC] && param->write.len == 2) {
            ccc_value = (param->write.value[1] << 8) | param->write.value[0];
            ESP_LOGI(TAG, "CCC write: 0x%04x", ccc_value);
            esp_ble_gatts_send_response(
                gatts_if,
                param->write.conn_id,
                param->write.trans_id,
                ESP_GATT_OK,
                NULL);
        }
        break;
    }
    default:
        break;
    }
}

// Notification task: only send when client enabled notifications
static void notify_task(void *arg)
{
    const char *msg = "FALL_DETECTED";
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (conn_id != 0 && (ccc_value & ESP_GATT_CLIENT_CHAR_CFG_NOTIFY)) {
            esp_err_t err = esp_ble_gatts_send_indicate(
                gatts_if_global,
                conn_id,
                handles[IDX_CHAR],
                strlen(msg),
                (uint8_t*)msg,
                false);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Indicate failed: %s", esp_err_to_name(err));
            }
        }
    }
}

void app_main(void)
{
    // 1) Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2) Initialize BLE controller
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // 3) Initialize Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // 4) Register GAP and GATT callbacks
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    // 5) Start the notification task
    xTaskCreate(notify_task, "notify_task", 4096, NULL, 5, NULL);
}
