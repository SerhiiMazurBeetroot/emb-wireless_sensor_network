#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "nrf24.h"
#include "nvs_flash.h"

static const char *TAG = "gateway";

/* Start NVS and the radio, enter receive mode, and print every 5-byte packet. */
void app_main(void)
{
	uint8_t payload[NRF24_PAYLOAD_LEN];
	
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "WSN gateway — step 2: listen for HELLO");
    ESP_ERROR_CHECK(nrf24_init());

    if (!nrf24_spi_probe()) {
		ESP_LOGE(TAG, "SPI probe failed");
		return;
    }
	
	nrf24_radio_init();
	nrf24_set_rx_mode();
	ESP_LOGI(TAG, "RX mode, channel 76");

	while(1) {
		if(nrf24_receive(payload, NRF24_PAYLOAD_LEN)) {
			ESP_LOGI(TAG, "RX: %c%c%c%c%c",
			                     payload[0], payload[1], payload[2], payload[3], payload[4]);
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}
