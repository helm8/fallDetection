#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"       // ** NEW BLE INCLUDE **
#include "esp_gatts_api.h"       // ** NEW BLE INCLUDE **
#include "esp_gatt_common_api.h" // ** NEW BLE INCLUDE **
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "bmi270.c"  // BMI270 driver source
extern const uint8_t bmi270_config_file[];

// --- GPIO Pins ---
static gpio_num_t GPIO_FALL_DET = GPIO_NUM_0;
static gpio_num_t GPIO_BTH_STS  = GPIO_NUM_1;
static gpio_num_t GPIO_SPK_CTRL = GPIO_NUM_2;
static gpio_num_t GPIO_BTN_SIG  = GPIO_NUM_10;
static gpio_num_t GPIO_I2C_SDA  = GPIO_NUM_5;
static gpio_num_t GPIO_I2C_SCL  = GPIO_NUM_6;

// I2C / sensor buffer
static const int I2C_TIMEOUT_MS = 1000;
static const int num_samples = 50;
struct SensorData {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    double  acc_mag, gyr_mag;
    struct SensorData *next;
};
static struct SensorData *data_buffer;

// State machine
typedef enum { IDLE_STATE, ERROR_STATE, CONFIGURATION_STATE, ALARM_STATE, FALL_STATE } state_t;
static state_t current_state = CONFIGURATION_STATE;


// === NEW BLE GATT DEFINITIONS & GLOBALS ===
#define DEVICE_NAME   "ESP32_Fall_Detect"    // ** NEW BLE DEVICE NAME **
#define SERVICE_UUID  0x00FF                // ** NEW BLE SERVICE UUID **
#define CHAR_UUID     0xFF01                // ** NEW BLE CHARACTERISTIC UUID **

static const char *TAG = "BLE_FALL";        // ** NEW BLE LOG TAG **
static uint8_t char_value[20] = "READY";    // Value that will be notified
static esp_attr_value_t char_attr_value = {   
    .attr_len     = sizeof(char_value),      // ** NEW BLE ATTRIBUTE INIT **
    .attr_max_len = sizeof(char_value),
    .attr_value   = char_value,
};

enum { HANDLE_SERVICE, HANDLE_CHAR, HANDLE_CHAR_CFG, HANDLE_MAX };
static uint16_t gatt_handles[HANDLE_MAX];     // ** NEW BLE HANDLES **
static uint16_t global_conn_id = 0;           // ** NEW BLE CONNECTION ID **
static esp_gatt_if_t global_gatts_if = 0;     // ** NEW BLE GATT IF **

// Advertising parameters
static esp_ble_adv_params_t adv_params = {    // ** NEW BLE ADVERT params **
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
static esp_ble_adv_data_t adv_data = {        // ** NEW BLE ADV DATA **
    .set_scan_rsp    = false,
    .include_name    = true,
    .include_txpower = true,
    .flag             = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};


// === NEW BLE CALLBACKS ===
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&adv_params);    // Start advertising
    } else if (event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT) {
        ESP_LOGI(TAG, "Advertising %s",
            param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS ? "started" : "failed");
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        // Register and set up advertisement
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        {
            esp_gatt_srvc_id_t svc_id = {
                .is_primary = true,
                .id.inst_id = 0,
                .id.uuid.len = ESP_UUID_LEN_16,
                .id.uuid.uuid.uuid16 = SERVICE_UUID,
            };
            esp_ble_gatts_create_service(gatts_if, &svc_id, HANDLE_MAX);
        }
        break;
    case ESP_GATTS_CREATE_EVT:
        // Service created
        gatt_handles[HANDLE_SERVICE] = param->create.service_handle;
        {
            esp_bt_uuid_t char_uuid = { .len = ESP_UUID_LEN_16, .uuid.uuid16 = CHAR_UUID };
            esp_gatt_char_prop_t props = ESP_GATT_CHAR_PROP_BIT_READ |
                                         ESP_GATT_CHAR_PROP_BIT_WRITE |
                                         ESP_GATT_CHAR_PROP_BIT_NOTIFY;
            esp_ble_gatts_add_char(
                gatt_handles[HANDLE_SERVICE],
                &char_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                props,
                &char_attr_value,
                NULL
            );
        }
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        // Characteristic added
        gatt_handles[HANDLE_CHAR] = param->add_char.attr_handle;
        {
            // Add Client Characteristic Config Descriptor
            esp_bt_uuid_t descr_uuid = { .len = ESP_UUID_LEN_16,
                                         .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG };
            esp_ble_gatts_add_char_descr(
                gatt_handles[HANDLE_SERVICE],
                &descr_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL,
                NULL
            );
        }
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        // Descriptor added; start service
        gatt_handles[HANDLE_CHAR_CFG] = param->add_char_descr.attr_handle;
        esp_ble_gatts_start_service(gatt_handles[HANDLE_SERVICE]);
        break;
    case ESP_GATTS_READ_EVT: {
        // Handle reads
        esp_gatt_rsp_t rsp = {0};
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len    = strlen((char*)char_value);
        memcpy(rsp.attr_value.value, char_value, rsp.attr_value.len);
        esp_ble_gatts_send_response(
            gatts_if,
            param->read.conn_id,
            param->read.trans_id,
            ESP_GATT_OK,
            &rsp
        );
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        // Handle writes
        size_t len = param->write.len < sizeof(char_value)-1 ? param->write.len : sizeof(char_value)-1;
        memcpy(char_value, param->write.value, len);
        char_value[len] = '\0';
        if (param->write.need_rsp) {
            esp_gatt_rsp_t rsp = {0};
            rsp.attr_value.handle = param->write.handle;
            rsp.attr_value.len    = len;
            memcpy(rsp.attr_value.value, param->write.value, len);
            esp_ble_gatts_send_response(
                gatts_if,
                param->write.conn_id,
                param->write.trans_id,
                ESP_GATT_OK,
                &rsp
            );
        }
        break;
    }
    case ESP_GATTS_CONNECT_EVT:
        // Client connected
        global_conn_id = param->connect.conn_id;
        global_gatts_if = gatts_if;
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        // Client disconnected: restart advertising
        global_conn_id = 0;
        esp_ble_gap_start_advertising(&adv_params);
        break;
    default:
        break;
    }
}


// --- Circular buffer init ---
static void clear_sample(struct SensorData *s) {
    memset(s, 0, sizeof(*s));
}
static void init_data_buffer() {
    data_buffer = malloc(sizeof(*data_buffer)); clear_sample(data_buffer);
    struct SensorData *p = data_buffer;
    for (int i = 1; i < num_samples; i++) {
        p->next = malloc(sizeof(*p)); p = p->next; clear_sample(p);
    }
    p->next = data_buffer;
}

// --- I2C helper ---
static void i2c_init(i2c_master_bus_handle_t *bus, i2c_master_dev_handle_t *dev) {
    i2c_master_bus_config_t bcfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bcfg, bus));
    i2c_device_config_t dcfg = {.device_address=0x68, .dev_addr_length=I2C_ADDR_BIT_LEN_7, .scl_speed_hz=100000};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus, &dcfg, dev));
}
static esp_err_t sensor_read(i2c_master_dev_handle_t h, uint8_t r, uint8_t *d, size_t n) {
    return i2c_master_transmit_receive(h, &r, 1, d, n, I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
}
static esp_err_t sensor_write(i2c_master_dev_handle_t h, uint8_t r, uint8_t v) {
    uint8_t buf[2] = {r,v};
    return i2c_master_transmit(h, buf, 2, I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
}

// --- alarm timer callback ---
static void IRAM_ATTR alarm_cb(void *arg) {
    if (current_state == ALARM_STATE) current_state = FALL_STATE;
}
static void alarm_timer_set(bool en) {
    static esp_timer_handle_t t;
    static bool init=false;
    if (!init) {
        esp_timer_create_args_t args={.callback=alarm_cb,.name="alarm_timer"};
        esp_timer_create(&args, &t);
        init=true;
    }
    if (en) esp_timer_start_once(t, 5000*1000);
    else     esp_timer_stop(t);
}

// --- Fall detection and BLE notify ---
static void check_fall() {
    int cnt=0; struct SensorData *p=data_buffer;
    for(int i=0;i<num_samples;i++,p=p->next) {
        if (p->gyr_mag>10000) {
            if (++cnt>3) {
                // Detected a fall!
                if (current_state == IDLE_STATE) {
                    current_state = ALARM_STATE;
                    alarm_timer_set(true);
                    // ** NEW BLE NOTIFICATION **
                    const char *msg = "FALL_DETECTED";
                    size_t len = strlen(msg);
                    memcpy(char_value, msg, len);
                    char_value[len] = '\0';
                    if (global_conn_id && global_gatts_if) {
                        esp_ble_gatts_send_indicate(
                            global_gatts_if,
                            global_conn_id,
                            gatt_handles[HANDLE_CHAR],
                            len,
                            char_value,
                            false
                        );
                    }
                }
                break;
            }
        } else {
            cnt = 0;
        }
    }
}

// --- sensor poll callback ---
static void IRAM_ATTR sensor_cb(void *arg) {
    i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t*)arg;
    uint8_t buf[12]; sensor_read(dev, BMI2_ACC_X_LSB_ADDR, buf, 12);
    data_buffer->acc_x = (buf[1]<<8)|buf[0];
    data_buffer->acc_y = (buf[3]<<8)|buf[2];
    data_buffer->acc_z = (buf[5]<<8)|buf[4];
    data_buffer->gyr_x = (buf[7]<<8)|buf[6];
    data_buffer->gyr_y = (buf[9]<<8)|buf[8];
    data_buffer->gyr_z = (buf[11]<<8)|buf[10];
    double ia = pow(data_buffer->acc_x,2) + pow(data_buffer->acc_y,2) + pow(data_buffer->acc_z,2);
    double ig = pow(data_buffer->gyr_x,2) + pow(data_buffer->gyr_y,2) + pow(data_buffer->gyr_z,2);
    data_buffer->acc_mag = sqrt(ia);
    data_buffer->gyr_mag = sqrt(ig);
    data_buffer = data_buffer->next;
    static int cnt = 0;
    if (++cnt >= num_samples) { cnt = 0; check_fall(); }
}

// --- configure GPIO & timers ---
static void configure_io() {
    gpio_reset_pin(GPIO_FALL_DET);
    gpio_reset_pin(GPIO_BTH_STS);
    gpio_reset_pin(GPIO_SPK_CTRL);
    gpio_set_direction(GPIO_FALL_DET, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_BTH_STS, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_SPK_CTRL, GPIO_MODE_OUTPUT);
}
static void setup_timers(i2c_master_dev_handle_t dev) {
    esp_timer_create_args_t scb = {.callback=sensor_cb, .arg=&dev, .name="sensor"};
    esp_timer_handle_t th;
    esp_timer_create(&scb, &th);
    esp_timer_start_periodic(th, 40 * 1000);
}

void app_main(void) {
    init_data_buffer();
    configure_io();

    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    i2c_init(&bus, &dev);
    // BMI270 init omitted for brevity
    current_state = IDLE_STATE;

    // === NEW BLE STACK INITIALIZATION ===
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_bt_controller_init(& (esp_bt_controller_config_t)BT_CONTROLLER_INIT_CONFIG_DEFAULT()));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    setup_timers(dev);

    while (1) {
        switch (current_state) {
            case IDLE_STATE:
                gpio_set_level(GPIO_SPK_CTRL, 0);
                gpio_set_level(GPIO_FALL_DET, 0);
                break;
            case ALARM_STATE:
                gpio_set_level(GPIO_SPK_CTRL, 1);
                gpio_set_level(GPIO_FALL_DET, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                gpio_set_level(GPIO_FALL_DET, 1);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;
            case FALL_STATE:
                gpio_set_level(GPIO_SPK_CTRL, 0);
                gpio_set_level(GPIO_FALL_DET, 1);
                break;
            default:
                break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
