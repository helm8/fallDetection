#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
//#include "bmi270_config.h"
#include "bmi270.c"
#include "sdkconfig.h"
extern const uint8_t bmi_config_file[];

static gpio_num_t GPIO_FALL_DET = GPIO_NUM_0;
static gpio_num_t GPIO_BTH_STS = GPIO_NUM_1;
static gpio_num_t GPIO_SPK_CTRL = GPIO_NUM_2;
static gpio_num_t GPIO_BTN_SIG = GPIO_NUM_10;
static gpio_num_t GPIO_I2C_SDA = GPIO_NUM_5;
static gpio_num_t GPIO_I2C_SCL = GPIO_NUM_6;
static uint32_t I2C_TIMEOUT_MS = 1000;
static int fallDetected = 0;
static int bluetoothStatus = 0;



static void configure_led(void) {
	printf("Configuring LED!\n");
	gpio_reset_pin(GPIO_FALL_DET);
	gpio_reset_pin(GPIO_BTH_STS);
	gpio_reset_pin(GPIO_SPK_CTRL);
	gpio_set_direction(GPIO_FALL_DET, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_BTH_STS, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_SPK_CTRL, GPIO_MODE_OUTPUT);
}

static void IRAM_ATTR button_poll_timer_callback(void *arg) {	
	static int prev_button_state = 1;
	int button_state;
	button_state = gpio_get_level(GPIO_BTN_SIG);
	if (button_state != prev_button_state && !button_state) {
		fallDetected = !fallDetected;
		gpio_set_level(GPIO_FALL_DET, fallDetected);
	}
	prev_button_state = button_state;
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

static void configure_button(void) {
	printf("Configuring Button!\n");
	gpio_reset_pin(GPIO_BTN_SIG);
	gpio_set_direction(GPIO_BTN_SIG, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GPIO_BTN_SIG, GPIO_PULLUP_ONLY);
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
	ESP_ERROR_CHECK(esp_timer_start_periodic(button_poll_timer, 100000));
	ESP_ERROR_CHECK(esp_timer_start_periodic(bluetooth_poll_timer, 3000000));
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

static void sensor_init(i2c_master_dev_handle_t dev_handle) {
	uint8_t read_data[2];
	struct bmi2_dev bmi;
	struct bmi2_sens_data sensor_data = { { 0 } };
	uint8_t rslt;
	ESP_ERROR_CHECK(sensor_register_read(dev_handle, 0x0, read_data, 1));
	rslt = bmi2_interface_init(&bmi, BMI2_I2C_INTF);
	//bmi270_init(&bmi);

	
	if (read_data[0] == 0x24) {
		printf("Communication with BMI270 successful!\n");
	} else {
		printf("Communication with BMI270 not verified, received byte: %X\n", read_data[0]);
	}
	
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, 0x7C, 0x0));
	vTaskDelay(1 / portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_INIT_CTRL_ADDR, 0x0));
	

	for (int i = 0; i < sizeof(bmi270_config_file); i++) {
		ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_INIT_DATA_ADDR, bmi270_config_file[i]));
		//printf("Configuration Byte %d: %X\n", i, bmi270_config_file[i]);
	}
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_INIT_CTRL_ADDR, 0x1));
	ESP_ERROR_CHECK(sensor_register_read(dev_handle, BMI2_INTERNAL_STATUS_ADDR, read_data, 1));

	if ((read_data[0] & 0x1) == 0x1) {
		printf("BMI270 initialized successfully!\n");
	} else {
		printf("Error: BMI270 not initialized! Internal status: %X\n", read_data[0]);
	}
	//printf("Total size of config file: %d\n", sizeof(bmi270_config_file));
}

void app_main(void) {
	i2c_master_bus_handle_t bus_handle;
	i2c_master_dev_handle_t dev_handle;
	configure_led();
	configure_button();
	start_bluetooth_connection();
	timer_init();
	i2c_init(&bus_handle, &dev_handle);
	sensor_init(dev_handle);
	gpio_dump_io_configuration(stdout, 1ULL << 10); // For debugging
	while (1) {	
		gpio_set_level(GPIO_SPK_CTRL, 0);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_SPK_CTRL, 1);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}
