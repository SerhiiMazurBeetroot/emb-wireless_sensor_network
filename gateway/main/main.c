#include <stdio.h>

#include "esp_log.h"
#include "nrf24.h"
#include "nvs_flash.h"

static const char *TAG = "gateway";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "WSN gateway — step 1: nRF24 SPI register probe");

    ESP_ERROR_CHECK(nrf24_init());

    if (nrf24_spi_probe()) {
        ESP_LOGI(TAG, "Step 1 passed on ESP32-S3");
    } else {
        ESP_LOGE(TAG, "Step 1 failed — check wiring, 3.3 V, 47 uF, CSN/CE pins");
    }
}
