idf_component_register(
	SRCS "ssd1306.c"
	INCLUDE_DIRS "include"
	REQUIRES
		config
		esp_lcd
		esp_driver_i2c
)

