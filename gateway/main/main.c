#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nrf24.h"
#include "nvs_flash.h"

static const char *TAG = "gateway";

#define PKT_TYPE_DATA 0x01
#define PKT_TYPE_ACK 0x02
#define PKT_NODE_ID 1

/* CRC-8, poly 0x07, over the first 7 bytes of the 8-byte packet. */
static uint8_t crc8(const uint8_t *data, uint8_t len) {
	uint8_t crc = 0x00;
	
	for(uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for(uint8_t bit = 0; bit < 8; bit++) {
			if(crc & 0x80) {
				crc = (uint8_t)((crc << 1) ^ 0x07);
			} else {
				crc = (uint8_t)(crc << 1);
			}
		}
	}
	
	return crc;
}

static bool packet_crc_ok(const uint8_t *pkt) {
	return crc8(pkt, NRF24_PAYLOAD_LEN - 1) == pkt[NRF24_PAYLOAD_LEN - 1];
}

static void packet_build(uint8_t *pkt, uint8_t type, uint8_t node_id, uint8_t seq, uint8_t value) {
	memset(pkt, 0, NRF24_PAYLOAD_LEN);
	pkt[0] = type;
	pkt[1] = node_id;
	pkt[2] = seq;
	pkt[3] = value;
	pkt[NRF24_PAYLOAD_LEN - 1] = crc8(pkt, NRF24_PAYLOAD_LEN - 1);
}


/* Receive DATA, check CRC and address, answer with ACK, then listen again. */
void app_main(void)
{
	uint8_t rx[NRF24_PAYLOAD_LEN];
	uint8_t ack[NRF24_PAYLOAD_LEN];
	uint8_t last_seq = 0;
	uint8_t have_seq = false;
		
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "WSN gateway — step 3: DATA + ACK");
    ESP_ERROR_CHECK(nrf24_init());

    if (!nrf24_spi_probe()) {
		ESP_LOGE(TAG, "SPI probe failed");
		return;
    }
	
	nrf24_radio_init();
	nrf24_set_rx_mode();
	ESP_LOGI(TAG, "RX mode, channel 76");

	while(1) {
		if (!nrf24_receive(rx, NRF24_PAYLOAD_LEN)) {
           vTaskDelay(pdMS_TO_TICKS(10));
           continue;
        }
		
		if(!packet_crc_ok(rx)) {
			ESP_LOGW(TAG, "bad CRC");
			continue;
		}
		
		if(rx[0] != PKT_TYPE_DATA || rx[1] != PKT_NODE_ID) {
			ESP_LOGW(TAG, "ignored type=0x%02X node=%u", rx[0], rx[1]);
			continue;
		}
		
		const bool duplicate = have_seq && rx[2] == last_seq;
		ESP_LOGI(TAG, "DATA node=%u seq=%u value=%u%s", rx[1], rx[2], rx[3], duplicate ? " duplicate" : "");
		last_seq = rx[2];
		have_seq = true;
		
		packet_build(ack, PKT_TYPE_ACK, rx[1], rx[2], 0);
		vTaskDelay(pdMS_TO_TICKS(10));
		if(nrf24_send(ack, NRF24_PAYLOAD_LEN)) {
			ESP_LOGI(TAG, "ACK seq=%u", rx[2]);
		} else {
			ESP_LOGW(TAG, "ACK send failed seq=%u", rx[2]);
		}
		nrf24_set_rx_mode();
	}
}
