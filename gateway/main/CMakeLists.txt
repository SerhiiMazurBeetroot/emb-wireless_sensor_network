# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS main.c
    REQUIRES 
    	nrf24
    	nvs_flash
    	freertos
    	ssd1306
    	esp_timer
    	statistics
    	config
    	button
    	protocol
    	webui
    	esp_rom
)
