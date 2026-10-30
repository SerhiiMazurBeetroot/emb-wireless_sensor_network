idf_component_register(
	SRCS "ssd1306.c"
	INCLUDE_DIRS "include"
	REQUIRES
		config
		esp_driver_i2c
		esp_rom
		freertos
		u8g2
)
