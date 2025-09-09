idf_component_register(SRCS "temp_humidity_sensor.c"
	REQUIRES esp_driver_gpio
	REQUIRES esp_driver_rmt
	REQUIRES esp_timer
                    INCLUDE_DIRS ".")
