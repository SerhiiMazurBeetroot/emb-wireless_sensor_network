idf_component_register(
    SRCS "nrf24.c"
    INCLUDE_DIRS "include"
    REQUIRES 
    	config
    	esp_driver_spi
    	esp_driver_gpio
    	driver
    	freertos
    	esp_rom
)

