#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
//#include "bmi270_config.h"
#include "bmi270.c"
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "driver/ledc.h"

#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"  // for CLIENT CHAR CONFIG descriptor UUID
#include "esp_log.h"
#include <string.h>

#define DEVICE_NAME "ESP32_C3_BLE"
#define SERVICE_UUID 0x00FF
#define CHAR_UUID    0xFF01
static const uint16_t CLIENT_CHAR_CFG_UUID = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const char *TAG = "BLE_GATT";

// Initial characteristic value
static uint8_t char_value[20] = "Hello BLE";
static esp_attr_value_t char_attr_value = {
    .attr_len = sizeof(char_value),
    .attr_max_len = sizeof(char_value),
    .attr_value = char_value,
};

// Handle table for service, characteristic, descriptor
enum {
    HANDLE_SERVICE,
    HANDLE_CHAR,
    HANDLE_CHAR_VAL,
    HANDLE_CHAR_CFG,  // CCC descriptor handle
    HANDLE_MAX
};
static uint16_t gatt_handles[HANDLE_MAX];

// Global state for notifications
static uint16_t global_conn_id = 0;
static esp_gatt_if_t global_gatts_if = 0;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
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

extern const uint8_t bmi_config_file[];

static gpio_num_t GPIO_FALL_DET = GPIO_NUM_0;
static gpio_num_t GPIO_BTH_STS = GPIO_NUM_1;
static gpio_num_t GPIO_SPK_CTRL = GPIO_NUM_2;
static gpio_num_t GPIO_BTN_SIG = GPIO_NUM_10;
static gpio_num_t GPIO_INT1 = GPIO_NUM_4;
static gpio_num_t GPIO_I2C_SDA = GPIO_NUM_5;
static gpio_num_t GPIO_I2C_SCL = GPIO_NUM_6;
static uint32_t I2C_TIMEOUT_MS = 1000;
static int bluetoothStatus = 0;

struct SensorData {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    double acc_mag, gyr_mag;
    struct SensorData *next_sample;
};
struct SensorData *data_buffer;
static int num_samples = 50;

typedef enum {
    IDLE_STATE,
    ERROR_STATE,
    CONFIGURATION_STATE,
    ALARM_STATE,
    FALL_STATE
} state_t;
static state_t current_state = CONFIGURATION_STATE;

// GAP Event Handler
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

// GATTS Event Handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            ESP_LOGI(TAG, "GATT Server Registered, app_id %d", param->reg.app_id);
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
            esp_gatt_srvc_id_t service_id = {
                .id = {.inst_id = 0, .uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = SERVICE_UUID}},
                .is_primary = true,
            };
            esp_ble_gatts_create_service(gatts_if, &service_id, HANDLE_MAX);
            break;
        }
        case ESP_GATTS_CREATE_EVT: {
            ESP_LOGI(TAG, "Service created, handle %d", param->create.service_handle);
            gatt_handles[HANDLE_SERVICE] = param->create.service_handle;
            esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = CHAR_UUID};
            esp_gatt_char_prop_t char_prop = ESP_GATT_CHAR_PROP_BIT_READ |
                                              ESP_GATT_CHAR_PROP_BIT_WRITE |
                                              ESP_GATT_CHAR_PROP_BIT_NOTIFY;
            esp_ble_gatts_add_char(
                gatt_handles[HANDLE_SERVICE],
                &char_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                char_prop,
                &char_attr_value,
                NULL
            );
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT: {
            ESP_LOGI(TAG, "Characteristic added, handle %d", param->add_char.attr_handle);
            gatt_handles[HANDLE_CHAR] = param->add_char.attr_handle;
            // Add CCC Descriptor
            esp_bt_uuid_t descr_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = CLIENT_CHAR_CFG_UUID};
            esp_ble_gatts_add_char_descr(
                gatt_handles[HANDLE_SERVICE],
                &descr_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL,
                NULL
            );
            esp_ble_gatts_start_service(gatt_handles[HANDLE_SERVICE]);
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
            ESP_LOGI(TAG, "CCC Descriptor added, handle %d", param->add_char_descr.attr_handle);
            gatt_handles[HANDLE_CHAR_CFG] = param->add_char_descr.attr_handle;
            break;
        }
        case ESP_GATTS_READ_EVT: {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(rsp));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len    = strlen((char*)char_value);
            memcpy(rsp.attr_value.value, char_value, rsp.attr_value.len);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                        param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            memcpy(char_value, param->write.value, param->write.len);
            char_value[param->write.len] = '\0';
            if (param->write.need_rsp) {
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(rsp));
                rsp.attr_value.handle = param->write.handle;
                rsp.attr_value.len    = param->write.len;
                memcpy(rsp.attr_value.value, param->write.value, param->write.len);
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, &rsp);
            }
            break;
        }
        case ESP_GATTS_CONNECT_EVT: {
            global_conn_id  = param->connect.conn_id;
            global_gatts_if = gatts_if;
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: {
            global_conn_id = 0;
            esp_ble_gap_start_advertising(&adv_params);
            break;
        }
        default: break;
    }
}

// Fall detection simulation task
void fall_detection_task(void *pvParameter) {
    while (1) {
        vTaskDelay(15000 / portTICK_PERIOD_MS);
        const char *fall_signal = "FALL_DETECTED";
        memset(char_value, 0, sizeof(char_value));
        strncpy((char*)char_value, fall_signal, sizeof(char_value)-1);
        if (global_conn_id && global_gatts_if) {
            esp_ble_gatts_send_indicate(
                global_gatts_if,
                global_conn_id,
                gatt_handles[HANDLE_CHAR],
                strlen((char*)char_value),
                char_value,
                false
            );
        }
    }
}

static void clear_data_sample(struct SensorData *data_sample, uint8_t test_val) {
	data_sample->acc_x = test_val;
	data_sample->acc_y = 0;
	data_sample->acc_z = 0;
	data_sample->gyr_x = 0;
	data_sample->gyr_y = 0;
	data_sample->gyr_z = 0;
	data_sample->acc_mag = 0;
	data_sample->gyr_mag = 0;
}

static void init_data_buffer(void) {
	data_buffer = (struct SensorData *) malloc(sizeof(struct SensorData));
	clear_data_sample(data_buffer, 0x0);
	struct SensorData *new_sample;
	struct SensorData *first_sample = data_buffer;
	for (int i = 1; i < num_samples; i++) {
		new_sample = (struct SensorData *) malloc(sizeof(struct SensorData));
		clear_data_sample(new_sample, i);
		data_buffer->next_sample = new_sample;
		data_buffer = new_sample;
	}
	data_buffer->next_sample = first_sample;
	data_buffer = data_buffer->next_sample;
}

static void print_data_buffer(void) {
	printf("Printing Data Buffer...\n");
	for (int i = 0; i < num_samples; i++) {
		printf("	Sample %.2d: Acc: (%.5d, %.5d, %.5d), Gyr: (%.5d, %.5d, %.5d), AccMag: %.2f, GyrMag: %.2f\n", i, data_buffer->acc_x, data_buffer->acc_y, data_buffer->acc_z, data_buffer->gyr_x, data_buffer->gyr_y, data_buffer->gyr_z, data_buffer->acc_mag, data_buffer->gyr_mag);
		if ((data_buffer->gyr_mag > 10000) && (data_buffer->next_sample->gyr_mag > 10000) && (data_buffer->next_sample->next_sample->gyr_mag > 10000)) {
			printf("Fall Detected!\n");
		}
		data_buffer = data_buffer->next_sample;
	}
	printf("... Printing Done!\n");
}

static void IRAM_ATTR alarm_timer_callback(void *arg) {
	if (current_state == ALARM_STATE) {
		current_state = FALL_STATE;
	}
}

static void alarm_timer_set(unsigned char set) {
	const esp_timer_create_args_t alarm_timer_args = {
		.callback = alarm_timer_callback,
		.name = "alarm_timer"
	};
	esp_timer_handle_t alarm_timer;
	if (set) {
		ESP_ERROR_CHECK(esp_timer_create(&alarm_timer_args, &alarm_timer));
		if (esp_timer_is_active(alarm_timer)) {
			ESP_ERROR_CHECK(esp_timer_restart(alarm_timer, 5000*1000));
		} else {
			ESP_ERROR_CHECK(esp_timer_start_once(alarm_timer, 5000*1000));
		}
	} else {
		if (esp_timer_is_active(alarm_timer)) {	
			ESP_ERROR_CHECK(esp_timer_stop(alarm_timer));
		}
	}
}

static void check_fall(void) {
    uint16_t count = 0;
    const uint16_t thres = 30000;

    for (int i = 0; i < num_samples; i++) {
        if (data_buffer->gyr_mag > thres) {
            count++;
        } else {
            count = 0;
        }

        if (count > 3) {
            printf("Fall Detected!!\n");

            if (current_state == IDLE_STATE) {
                current_state = ALARM_STATE;
                alarm_timer_set(1);

                // Send BLE notification to subscribed client
                if (global_conn_id != 0 && global_gatts_if != 0) {
                    const char *fall_signal = "FALL_DETECTED";
                    esp_ble_gatts_send_indicate(
                        global_gatts_if,
                        global_conn_id,
                        gatt_handles[HANDLE_CHAR],
                        strlen(fall_signal),
                        (uint8_t*)fall_signal,
                        false
                    );
                }
            }

            break;  // exit loop once a fall is detected
        }

        data_buffer = data_buffer->next_sample;
    }
}

static void i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
	printf("Initializing I2C Bus!\n");
	i2c_master_bus_config_t bus_config = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = GPIO_I2C_SDA,
		.scl_io_num = GPIO_I2C_SCL,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
	i2c_device_config_t dev_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = 0x68,
		.scl_speed_hz = 100000
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

// The following two functions are modified from esp-idf/examples/peripherals/i2c/i2c_basic
static esp_err_t sensor_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *read_data, size_t read_len) {
	return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, read_data, read_len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t sensor_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t write_data) {
	uint8_t write_buf[2] = {reg_addr, write_data};
	return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}


static void configure_led(void) {
	printf("Configuring LED!\n");
	gpio_reset_pin(GPIO_FALL_DET);
	gpio_reset_pin(GPIO_BTH_STS);
	gpio_reset_pin(GPIO_SPK_CTRL);
	gpio_set_direction(GPIO_FALL_DET, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_BTH_STS, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_SPK_CTRL, GPIO_MODE_OUTPUT);

	ledc_timer_config_t spk_pwm_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_7_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = 5000,
		.clk_cfg = LEDC_USE_XTAL_CLK
	};
	ESP_ERROR_CHECK(ledc_timer_config(&spk_pwm_timer));
/*
	ledc_timer_config_t fall_pwm_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_7_BIT,
		.timer_num = LEDC_TIMER_1,
		.freq_hz = 400,
		.clk_cfg = LEDC_USE_XTAL_CLK
	};
	ESP_ERROR_CHECK(ledc_timer_config(&fall_pwm_timer));
*/
	ledc_channel_config_t spk_pwm_channel = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.timer_sel = LEDC_TIMER_0,
		.intr_type = LEDC_INTR_DISABLE,
		.gpio_num = GPIO_SPK_CTRL,
		.duty = 64
	};
	ESP_ERROR_CHECK(ledc_channel_config(&spk_pwm_channel));
/*
	ledc_channel_config_t fall_pwm_channel = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_1, 
		.timer_sel = LEDC_TIMER_1,
		.intr_type = LEDC_INTR_DISABLE,
		.gpio_num = GPIO_FALL_DET,
		.duty = 64
	};
	ESP_ERROR_CHECK(ledc_channel_config(&fall_pwm_channel));
*/
}

static void IRAM_ATTR button_poll_timer_callback(void *arg) {	
	static int prev_button_state = 1;
	int button_state;
	button_state = gpio_get_level(GPIO_BTN_SIG);
	if (button_state != prev_button_state && !button_state) {
		//fallDetected = !fallDetected;
		//gpio_set_level(GPIO_FALL_DET, fallDetected);
		if (current_state == IDLE_STATE) {
			current_state = ALARM_STATE;
			alarm_timer_set(1);
		} else if (current_state == ALARM_STATE) {
			current_state = IDLE_STATE;
			alarm_timer_set(0);
		}
	}
	prev_button_state = button_state;
}

static void poll_sensor_data(i2c_master_dev_handle_t dev_handle) {
	uint8_t read_data[12];
	//int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
	static int count = 0;
	uint32_t intermediate;
	
	ESP_ERROR_CHECK(sensor_register_read(dev_handle, BMI2_ACC_X_LSB_ADDR, read_data, 12));
	data_buffer->gyr_x = (read_data[1] << 8) | read_data[0];
	data_buffer->gyr_y = (read_data[3] << 8) | read_data[2];
	data_buffer->gyr_z = (read_data[5] << 8) | read_data[4];
	data_buffer->acc_x = (read_data[7] << 8) | read_data[6];
	data_buffer->acc_y = (read_data[9] << 8) | read_data[8];
	data_buffer->acc_z = (read_data[11] << 8) | read_data[10];
	intermediate = pow(data_buffer->acc_x, 2) + pow(data_buffer->acc_y, 2) + pow(data_buffer->acc_z, 2);
	data_buffer->acc_mag = sqrt(intermediate);
	intermediate = pow(data_buffer->gyr_x, 2) + pow(data_buffer->gyr_y, 2) + pow(data_buffer->gyr_z, 2);
	data_buffer->gyr_mag = sqrt(intermediate);

	// For testing
	printf("Accelerometer Data: (%d, %d, %d), Gyroscope Data: (%d, %d, %d), Acc_mag = %.2f, Gyr_mag = %.2f\n", data_buffer->acc_x, data_buffer->acc_y, data_buffer->acc_z, data_buffer->gyr_x, data_buffer->gyr_y, data_buffer->gyr_z, data_buffer->acc_mag, data_buffer->gyr_mag);
	
	data_buffer = data_buffer->next_sample;
	
	if (count < 50) {
		count++;
	} else {
		count = 0;
		//print_data_buffer();
		check_fall();
	}
	
	// This part does not work yet, never receives interrupt signal
	ESP_ERROR_CHECK(sensor_register_read(dev_handle, BMI2_INT_STATUS_0_ADDR, read_data, 1));
	if (read_data[0] & 0x1) {
		printf("Significant Motion Detected!\n");
	}

}

static void IRAM_ATTR sensor_poll_timer_callback(void *arg) {
	i2c_master_dev_handle_t dev_handle = *(i2c_master_dev_handle_t *)arg;
	poll_sensor_data(dev_handle);
}

static void check_bluetooth_connection(void) {
	// This function will be completed later to check for a bluetooth connection
	if (bluetoothStatus) {
		gpio_set_level(GPIO_BTH_STS, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_BTH_STS, 0);
	}
}

static void IRAM_ATTR bluetooth_poll_timer_callback(void *arg) {
	check_bluetooth_connection();
}

static void start_bluetooth_connection(void) {
	// This function will be completed later to establish a bluetooth connection
	bluetoothStatus = 1;
}

static void configure_inputs(void) {
	printf("Configuring Inputs!\n");
	gpio_reset_pin(GPIO_BTN_SIG);
	gpio_reset_pin(GPIO_INT1);
	gpio_set_direction(GPIO_BTN_SIG, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GPIO_BTN_SIG, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_INT1, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GPIO_INT1, GPIO_PULLUP_ONLY);
}

static void timer_init(void) {
	printf("Initializing Timers!\n");
	const esp_timer_create_args_t button_poll_timer_args = {
		.callback = button_poll_timer_callback,
		.name = "button_poll_timer"
	};
	esp_timer_handle_t button_poll_timer;
	ESP_ERROR_CHECK(esp_timer_create(&button_poll_timer_args, &button_poll_timer));
	const esp_timer_create_args_t bluetooth_poll_timer_args = {
		.callback = bluetooth_poll_timer_callback,
		.name = "bluetooth_poll_timer"
	};
	esp_timer_handle_t bluetooth_poll_timer;
	ESP_ERROR_CHECK(esp_timer_create(&bluetooth_poll_timer_args, &bluetooth_poll_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(button_poll_timer, 100*1000));
	ESP_ERROR_CHECK(esp_timer_start_periodic(bluetooth_poll_timer, 3000*1000));
}

static void sensor_init(i2c_master_dev_handle_t dev_handle) {
	uint8_t read_data[2];
	
	// Read CHIP_ID register to verify connection with device	
	ESP_ERROR_CHECK(sensor_register_read(dev_handle, 0x0, read_data, 1));	
	if (read_data[0] == 0x24) {
		printf("Communication with BMI270 successful!\n");
	} else {
		printf("Communication with BMI270 not verified, received byte: %X\n", read_data[0]);
		current_state = ERROR_STATE;
		return;
	}
	
	// Write 0 to PWR_CONF register to disable advanced power save mode
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, 0x7C, 0x0));
	ets_delay_us(450);
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_INIT_CTRL_ADDR, 0x0));
	
	// Read INT_STATUS register to verify it is ready to configure (should read 0x0)
	ESP_ERROR_CHECK(sensor_register_read(dev_handle, BMI2_INTERNAL_STATUS_ADDR, read_data, 1));
	printf("Initial status: 0x%X\n", read_data[0]);

	// Write each byte from the configuration file to the INIT_DATA register
	uint8_t *write_data = (uint8_t *) malloc(sizeof(uint8_t) * 8193);
	write_data[0] = BMI2_INIT_DATA_ADDR;
	for (int i = 0; i < sizeof(bmi270_config_file); i++) {
		write_data[i + 1] = bmi270_config_file[i];
	}
	//printf("Last byte written to init_data register: 0x%X\n", write_data[8192]);
	//printf("Number of bytes being written: %d\n", sizeof(write_data));
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_data, 8193, -1));
	free(write_data);
	

	// Read last byte in the INIT_DATA register (for debugging)
	ESP_ERROR_CHECK(sensor_register_read(dev_handle, BMI2_INIT_DATA_ADDR, read_data, 1));
	printf("Last byte in init_data regsiter: 0x%X\n", read_data[0]);

	// Writes to INIT_CTRL register
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_INIT_CTRL_ADDR, 0x1));
	

	ets_delay_us(20000);	
	// Reads INT_STATUS register to verify configuration is good (should read 0x1)
	ESP_ERROR_CHECK(sensor_register_read(dev_handle, BMI2_INTERNAL_STATUS_ADDR, read_data, 1));
	if ((read_data[0] & 0x1) == 0x1) {
		printf("BMI270 initialized successfully!\n");
		current_state = IDLE_STATE;
	} else {
		printf("Error: BMI270 not initialized! Internal status: %X\n", read_data[0]);
		current_state = ERROR_STATE;
	}
	//printf("Total size of config file: %d\n", sizeof(bmi270_config_file));
	
	//Write to PWR_CTRL register to enable accel/gyro
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_PWR_CTRL_ADDR, 0x06));
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_ACC_CONF_ADDR, 0x37));
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_GYR_CONF_ADDR, 0x36));
	ets_delay_us(20000);	
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_FEAT_PAGE_ADDR, 0x2));
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, 0x34, 0x2A));
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, 0x3E, 0x1));
}

static void sensor_timer_init(i2c_master_dev_handle_t *dev_handle) {
	const esp_timer_create_args_t sensor_poll_timer_args = {
		.callback = sensor_poll_timer_callback,
		.name = "sensor_poll_timer",
		.arg = dev_handle
	};
	esp_timer_handle_t sensor_poll_timer;
	ESP_ERROR_CHECK(esp_timer_create(&sensor_poll_timer_args, &sensor_poll_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(sensor_poll_timer, 40*1000));
}

void app_main(void) {
	i2c_master_bus_handle_t bus_handle;
	i2c_master_dev_handle_t dev_handle;
	size_t free_data_size;
	init_data_buffer();
	print_data_buffer();
	configure_led();
	gpio_set_level(GPIO_BTH_STS, 1);
	gpio_set_level(GPIO_FALL_DET, 1);
	configure_inputs();
	start_bluetooth_connection();
	i2c_init(&bus_handle, &dev_handle);
	sensor_init(dev_handle);
	
	// Bluetooth stuff commented out for now
	
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
    	//xTaskCreate(fall_detection_task, "fall_detection_task", 4096, NULL, 5, NULL);
	
	
	gpio_set_level(GPIO_BTH_STS, 0);
	gpio_set_level(GPIO_FALL_DET, 0);
	timer_init();
	sensor_timer_init(&dev_handle);
	//gpio_dump_io_configuration(stdout, 1ULL << 10); // For debugging
	free_data_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
	printf("Number of bytes of memory available for sensor data: %d\n", free_data_size);

	// For testing
	printf("Recording Sensor Data...\n");
	//vTaskDelay(3000 / portTICK_PERIOD_MS);
	//print_data_buffer();
	while (1) {
		if (current_state == IDLE_STATE) {
			ESP_ERROR_CHECK(ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0));
			gpio_set_level(GPIO_SPK_CTRL, 0);
			gpio_set_level(GPIO_FALL_DET, 0);
		} else if (current_state == ALARM_STATE) {
			ESP_ERROR_CHECK(ledc_timer_resume(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0));
			gpio_set_level(GPIO_SPK_CTRL, 1);
			gpio_set_level(GPIO_FALL_DET, 0);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			gpio_set_level(GPIO_FALL_DET, 1);
			vTaskDelay(100 / portTICK_PERIOD_MS);
		} else if (current_state == FALL_STATE) {
			ESP_ERROR_CHECK(ledc_timer_pause(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0));
			gpio_set_level(GPIO_SPK_CTRL, 0);
			gpio_set_level(GPIO_FALL_DET, 1);
		}
	}
} 
