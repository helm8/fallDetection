#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include <string.h>

#define DEVICE_NAME "ESP32_C3_BLE"
#define SERVICE_UUID 0x00FF
#define CHAR_UUID    0xFF01

static const char *TAG = "BLE_GATT";

// Initial characteristic value (can be updated when a fall is detected)
static uint8_t char_value[20] = "Hello BLE";
static esp_attr_value_t char_attr_value = {
    .attr_len = sizeof(char_value),
    .attr_max_len = sizeof(char_value),
    .attr_value = char_value,
};

// Handle table for service, characteristic, etc.
enum {
    HANDLE_SERVICE,
    HANDLE_CHAR,
    HANDLE_CHAR_VAL,
    HANDLE_CHAR_CFG,  // For notifications if needed
    HANDLE_MAX
};

static uint16_t gatt_handles[HANDLE_MAX];

// Global variables to store connection information for notifications
static uint16_t global_conn_id = 0;
static esp_gatt_if_t global_gatts_if = 0;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,          // 20ms
    .adv_int_max = 0x40,          // 40ms
    .adv_type    = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// --- GAP Callback ---
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started");
            } else {
                ESP_LOGE(TAG, "Failed to start advertising, error code: %d", param->adv_start_cmpl.status);
            }
            break;
        default:
            break;
    }
}

// --- GATT Server Event Handler ---
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            ESP_LOGI(TAG, "GATT Server Registered, app_id %d", param->reg.app_id);
            // Set device name and configure advertisement data
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);

            // Create a primary service with a 16-bit UUID (SERVICE_UUID)
            esp_gatt_srvc_id_t service_id = {
                .id = {
                    .inst_id = 0,
                    .uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid.uuid16 = SERVICE_UUID,
                    },
                },
                .is_primary = true,
            };
            esp_ble_gatts_create_service(gatts_if, &service_id, HANDLE_MAX);
            break;
        }
        case ESP_GATTS_CREATE_EVT: {
            ESP_LOGI(TAG, "Service created, handle %d", param->create.service_handle);
            gatt_handles[HANDLE_SERVICE] = param->create.service_handle;
            // Define the characteristic properties: read, write, and notify.
            esp_gatt_char_prop_t char_prop = ESP_GATT_CHAR_PROP_BIT_READ |
                                              ESP_GATT_CHAR_PROP_BIT_WRITE |
                                              ESP_GATT_CHAR_PROP_BIT_NOTIFY;

            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = { .uuid16 = CHAR_UUID },
            };

            // Add the characteristic to the service
            esp_ble_gatts_add_char(gatt_handles[HANDLE_SERVICE], &char_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   char_prop, &char_attr_value, NULL);
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT: {
            ESP_LOGI(TAG, "Characteristic added, handle %d", param->add_char.attr_handle);
            gatt_handles[HANDLE_CHAR] = param->add_char.attr_handle;
            // Start the service so that it becomes available for connections
            esp_ble_gatts_start_service(gatt_handles[HANDLE_SERVICE]);
            break;
        }
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(TAG, "Characteristic Read, handle %d", param->read.handle);
            // Prepare response with the current value of the characteristic
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = strlen((char*)char_value);
            memcpy(rsp.attr_value.value, char_value, rsp.attr_value.len);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                        param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(TAG, "Characteristic Write, handle %d, length %d",
                     param->write.handle, param->write.len);
            // Update the characteristic's value with the data received from the client
            memcpy(char_value, param->write.value, param->write.len);
            char_value[param->write.len] = '\0';  // Ensure null termination
            ESP_LOGI(TAG, "New Value: %s", char_value);
            if (param->write.need_rsp) {
                // Send a response back to the client
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->write.handle;
                rsp.attr_value.len = param->write.len;
                memcpy(rsp.attr_value.value, param->write.value, param->write.len);
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, &rsp);
            }
            break;
        }
        case ESP_GATTS_CONNECT_EVT: {
            ESP_LOGI(TAG, "Client Connected, conn_id %d", param->connect.conn_id);
            // Store the connection information for notifications
            global_conn_id = param->connect.conn_id;
            global_gatts_if = gatts_if;
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: {
            ESP_LOGI(TAG, "Client Disconnected, conn_id %d", param->disconnect.conn_id);
            // Reset connection info and restart advertising
            global_conn_id = 0;
            esp_ble_gap_start_advertising(&adv_params);
            break;
        }
        default:
            break;
    }
}

// --- Fall Detection Task ---
// This task simulates a fall event.
// In a real scenario, replace this with your actual sensor-based fall detection.
void fall_detection_task(void *pvParameter) {
    while (1) {
        // Simulate waiting time until a fall is detected (e.g., 15 seconds)
        vTaskDelay(15000 / portTICK_PERIOD_MS);

        // Simulated fall detection:
        ESP_LOGI(TAG, "Fall detected! Preparing notification...");

        // Update the characteristic value with a special signal string.
        const char *fall_signal = "FALL_DETECTED";
        memset(char_value, 0, sizeof(char_value));
        strncpy((char *)char_value, fall_signal, sizeof(char_value) - 1);
        
        // If a client is connected, send a notification.
        if (global_conn_id != 0 && global_gatts_if != 0) {
            ESP_LOGI(TAG, "Notifying client of fall event...");
            // Send notification (false indicates no confirmation required)
            esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id,
                                        gatt_handles[HANDLE_CHAR],
                                        strlen((char*)char_value),
                                        char_value, false);
        } else {
            ESP_LOGW(TAG, "No client connected. Skipping notification.");
        }
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    // Create a task that simulates fall detection events.
    xTaskCreate(fall_detection_task, "fall_detection_task", 4096, NULL, 5, NULL);
}
