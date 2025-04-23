#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"   // for ESP_GATT_UUID_CHAR_CLIENT_CONFIG
#include "esp_log.h"
#include <string.h>
#include "bmi270.c"

#define DEVICE_NAME       "ESP32_C3_BLE"
#define SERVICE_UUID      0x00FF
#define CHAR_UUID         0xFF01
static const uint16_t CLIENT_CHAR_CFG_UUID = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const char *TAG = "BLE_GATT";

// Initial characteristic value
static uint8_t char_value[20] = "Hello BLE";
static esp_attr_value_t char_attr_value = {
    .attr_len     = sizeof(char_value),
    .attr_max_len = sizeof(char_value),
    .attr_value   = char_value,
};

// Handle table for service, characteristic, descriptor
enum {
    HANDLE_SERVICE,
    HANDLE_CHAR,
    HANDLE_CHAR_VAL,
    HANDLE_CHAR_CFG,
    HANDLE_MAX
};
static uint16_t gatt_handles[HANDLE_MAX];

static uint16_t        global_conn_id  = 0;
static esp_gatt_if_t   global_gatts_if = 0;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp       = false,
    .include_name       = true,
    .include_txpower    = true,
    .flag               = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Circular buffer for sensor data
typedef struct SensorData {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    double acc_mag, gyr_mag;
    struct SensorData *next_sample;
} SensorData;
static SensorData *data_buffer;
static const int num_samples = 50;

// Task and state machine definitions
typedef enum { IDLE_STATE, ERROR_STATE, CONFIGURATION_STATE, ALARM_STATE, FALL_STATE } state_t;
static state_t current_state = CONFIGURATION_STATE;

// GPIOs
static const gpio_num_t GPIO_FALL_DET = GPIO_NUM_0;
static const gpio_num_t GPIO_BTH_STS  = GPIO_NUM_1;
static const gpio_num_t GPIO_SPK_CTRL = GPIO_NUM_2;
static const gpio_num_t GPIO_BTN_SIG  = GPIO_NUM_10;
static const gpio_num_t GPIO_INT1     = GPIO_NUM_4;
static const gpio_num_t GPIO_I2C_SDA  = GPIO_NUM_5;
static const gpio_num_t GPIO_I2C_SCL  = GPIO_NUM_6;
static const int I2C_TIMEOUT_MS = 1000;

// Forward
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// Initialize circular buffer
static void clear_data_sample(SensorData *s, uint8_t v) {
    s->acc_x = v; s->acc_y = s->acc_z = 0;
    s->gyr_x = s->gyr_y = s->gyr_z = 0;
    s->acc_mag = s->gyr_mag = 0;
}
static void init_data_buffer(void) {
    data_buffer = malloc(sizeof(SensorData));
    clear_data_sample(data_buffer, 0);
    SensorData *first = data_buffer;
    for (int i = 1; i < num_samples; i++) {
        SensorData *node = malloc(sizeof(SensorData));
        clear_data_sample(node, i);
        data_buffer->next_sample = node;
        data_buffer = node;
    }
    data_buffer->next_sample = first;
    data_buffer = first;
}

// I2C read/write
static esp_err_t sensor_register_read(i2c_master_dev_handle_t dev, uint8_t addr, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(dev, &addr, 1, buf, len, I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
}
static esp_err_t sensor_register_write_byte(i2c_master_dev_handle_t dev, uint8_t addr, uint8_t val) {
    uint8_t w[2]={addr,val};
    return i2c_master_transmit(dev, w, 2, I2C_TIMEOUT_MS/portTICK_PERIOD_MS);
}

// Setup timers
static void IRAM_ATTR alarm_timer_cb(void *); static void alarm_timer_set(bool on);
static void IRAM_ATTR button_poll_cb(void *); static void IRAM_ATTR sensor_poll_cb(void *);
static void IRAM_ATTR bluetooth_poll_cb(void *);
static void configure_timers(void) {
    const esp_timer_create_args_t aargs={.callback=alarm_timer_cb,.name="alarm_timer"};
    const esp_timer_create_args_t bargs={.callback=button_poll_cb,.name="button_timer"};
    const esp_timer_create_args_t sargs={.callback=sensor_poll_cb,.name="sensor_timer",.arg=NULL};
    const esp_timer_create_args_t btargs={.callback=bluetooth_poll_cb,.name="bth_timer"};
    esp_timer_handle_t t;
    esp_timer_create(&bargs,&t); esp_timer_start_periodic(t,100e3);
    esp_timer_create(&btargs,&t);esp_timer_start_periodic(t,3000e3);
    // sensor & alarm timers get args later
}

// Fall state machine
static void IRAM_ATTR alarm_timer_cb(void *_) {
    if(current_state==ALARM_STATE) current_state=FALL_STATE;
}
static void alarm_timer_set(bool on) {
    static esp_timer_handle_t at;
    const esp_timer_create_args_t a={.callback=alarm_timer_cb,.name="alarm_timer"};
    if(on) {esp_timer_create(&a,&at);esp_timer_start_once(at,5e6);} else esp_timer_stop(at);
}
static void IRAM_ATTR button_poll_cb(void *_) {
    static int prev=1; int st=gpio_get_level(GPIO_BTN_SIG);
    if(st!=prev && !st) {
        if(current_state==IDLE_STATE){current_state=ALARM_STATE;alarm_timer_set(true);} 
        else if(current_state==ALARM_STATE){current_state=IDLE_STATE;alarm_timer_set(false);}
    }
    prev=st;
}
static void poll_sensor(i2c_master_dev_handle_t dev) {
    uint8_t rd[12]; static int cnt=0;
    sensor_register_read(dev,BMI2_ACC_X_LSB_ADDR,rd,12);
    data_buffer->acc_x=(rd[1]<<8)|rd[0]; data_buffer->acc_y=(rd[3]<<8)|rd[2]; data_buffer->acc_z=(rd[5]<<8)|rd[4];
    data_buffer->gyr_x=(rd[7]<<8)|rd[6]; data_buffer->gyr_y=(rd[9]<<8)|rd[8]; data_buffer->gyr_z=(rd[11]<<8)|rd[10];
    double am=pow(data_buffer->acc_x,2)+pow(data_buffer->acc_y,2)+pow(data_buffer->acc_z,2);
    data_buffer->acc_mag=sqrt(am);
    double gm=pow(data_buffer->gyr_x,2)+pow(data_buffer->gyr_y,2)+pow(data_buffer->gyr_z,2);
    data_buffer->gyr_mag=sqrt(gm);

    data_buffer=data_buffer->next_sample;
    if(++cnt>=num_samples){cnt=0;
        // check fall
        static int thr_count=0;
        for(int i=0;i<num_samples;i++){
            if(data_buffer->gyr_mag>10000) thr_count++; else thr_count=0;
            if(thr_count>3 && current_state==IDLE_STATE) {current_state=ALARM_STATE;alarm_timer_set(true);break;}
            data_buffer=data_buffer->next_sample;
        }
    }
}
static void IRAM_ATTR sensor_poll_cb(void *arg) { poll_sensor(*(i2c_master_dev_handle_t*)arg); }
static void IRAM_ATTR bluetooth_poll_cb(void *_) { /* optional */ }

// I2C init\static void i2c_init(i2c_master_bus_handle_t *bus, i2c_master_dev_handle_t *dev) {
    i2c_master_bus_config_t bc={.i2c_port=I2C_NUM_0,.sda_io_num=GPIO_I2C_SDA,.scl_io_num=GPIO_I2C_SCL,
      .clk_source=I2C_CLK_SRC_DEFAULT,.glitch_ignore_cnt=7};
    i2c_new_master_bus(&bc,bus);
    i2c_device_config_t dc={.dev_addr_length=I2C_ADDR_BIT_LEN_7,.device_address=0x68,.scl_speed_hz=100000};
    i2c_master_bus_add_device(*bus,&dc,dev);
}

// BLE GAP & GATTS handlers\static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) { /* as above */ }
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) { /* as above with CCC */ }

// BLE fall detect simulationoid fall_detection_task(void *_) {
    while(1){vTaskDelay(15000/portTICK_PERIOD_MS);
        ESP_LOGI(TAG,"Fall detected!");
        const char *sig="FALL_DETECTED";
        memset(char_value,0,sizeof(char_value)); strncpy((char*)char_value,sig,sizeof(char_value)-1);
        if(global_conn_id && global_gatts_if) esp_ble_gatts_send_indicate(global_gatts_if,global_conn_id,gatt_handles[HANDLE_CHAR],strlen((char*)char_value),char_value,false);
    }
}

void app_main(void) {
    // init
    init_data_buffer();
    configure_timers();

    i2c_master_bus_handle_t bus; i2c_master_dev_handle_t dev;
    i2c_init(&bus,&dev);
    // sensor init omitted for brevity

    ESP_ERROR_CHECK(nvs_flash_init());
    esp_bt_controller_config_t cfg=BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&cfg); esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init(); esp_bluedroid_enable();
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gatts_app_register(0);

    xTaskCreate(fall_detection_task,"fall_detection_task",4096,NULL,5,NULL);

    // setup sensor polling timer
    const esp_timer_create_args_t sargs={.callback=sensor_poll_cb,.name="sensor_timer",.arg=&dev};
    esp_timer_handle_t st;esp_timer_create(&sargs,&st);esp_timer_start_periodic(st,40e3);

    while(1) {
        if(current_state==IDLE_STATE) {gpio_set_level(GPIO_SPK_CTRL,0); gpio_set_level(GPIO_FALL_DET,0);} 
        else if(current_state==ALARM_STATE) {gpio_set_level(GPIO_SPK_CTRL,1); gpio_set_level(GPIO_FALL_DET,1);}
        else if(current_state==FALL_STATE) {gpio_set_level(GPIO_SPK_CTRL,0); gpio_set_level(GPIO_FALL_DET,1);} 
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}
