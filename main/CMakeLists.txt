idf_component_register(SRCS "fallDetection.c"
                    INCLUDE_DIRS "."
		    REQUIRES esp_driver_gpio
		    REQUIRES esp_driver_i2c
		    REQUIRES bt
		    REQUIRES nvs_flash)	    
