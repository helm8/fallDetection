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

struct SensorData { // A Circular linked list for storing sensor data
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyr_x;
	int16_t gyr_y;
	int16_t gyr_z;
	double acc_mag;
	double gyr_mag;
	struct SensorData *next_sample;
};

struct SensorData *data_buffer;
static int num_samples = 50;

typedef enum {
	IDLE_STATE, // normal state -poll buttons and sensor
	ERROR_STATE, // display error and wait for reset
	CONFIGURATION_STATE, // state during set up of sensor
	ALARM_STATE, // state for 5 seconds after fall is detected
	FALL_STATE // after alarm state expires
} state_t;

static state_t current_state = CONFIGURATION_STATE;


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
	uint16_t count;
	uint16_t thres = 10000;
	count = 0;
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
			}
			return;
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
	data_buffer->acc_x = (read_data[1] << 8) | read_data[0];
	//acc_y = (read_data[3] << 8) | read_data[2];
	data_buffer->acc_y = (read_data[3] << 8) | read_data[2];
	data_buffer->acc_z = (read_data[5] << 8) | read_data[4];
	data_buffer->gyr_x = (read_data[7] << 8) | read_data[6];
	data_buffer->gyr_y = (read_data[9] << 8) | read_data[8];
	data_buffer->gyr_z = (read_data[11] << 8) | read_data[12];
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
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_ACC_CONF_ADDR, 0x77));
	ESP_ERROR_CHECK(sensor_register_write_byte(dev_handle, BMI2_GYR_CONF_ADDR, 0x26));
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
	gpio_set_level(GPIO_BTH_STS, 0);
	gpio_set_level(GPIO_FALL_DET, 0);
	timer_init();
	sensor_timer_init(&dev_handle);
	gpio_dump_io_configuration(stdout, 1ULL << 10); // For debugging
	free_data_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
	printf("Number of bytes of memory available for sensor data: %d\n", free_data_size);

	// For testing
	printf("Recording Sensor Data...\n");
	//vTaskDelay(3000 / portTICK_PERIOD_MS);
	//print_data_buffer();
	while (1) {
		if (current_state == IDLE_STATE) {
			gpio_set_level(GPIO_SPK_CTRL, 0);
			gpio_set_level(GPIO_FALL_DET, 0);
		} else if (current_state == ALARM_STATE) {
			gpio_set_level(GPIO_SPK_CTRL, 1);
			gpio_set_level(GPIO_FALL_DET, 0);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			gpio_set_level(GPIO_FALL_DET, 1);
			vTaskDelay(100 / portTICK_PERIOD_MS);
		} else if (current_state == FALL_STATE) {
			gpio_set_level(GPIO_SPK_CTRL, 0);
			gpio_set_level(GPIO_FALL_DET, 1);
		}
	}
}
