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

/* CRC-8, poly 0x07, over every byte except the last one. */
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

static void packet_seal(uint8_t *pkt) {
	pkt[NRF24_PAYLOAD_LEN - 1] = crc8(pkt, NRF24_PAYLOAD_LEN - 1);
}

static int16_t get_i16(const uint8_t *p) {
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t get_u16(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* Receive a sensor packet, check CRC and address, answer with ACK, then listen again. */
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

    ESP_LOGI(TAG, "WSN gateway — step 5: sleep");
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
		const int16_t temp = get_i16(&rx[3]);
		const int16_t hum = get_u16(&rx[5]);
		const int16_t press = get_u16(&rx[7]);
		const int16_t t_abs = (temp < 0) ? (int16_t)(-temp) : temp;
		ESP_LOGI(TAG, "DATA node=%u seq=%u temp=%s%d.%02dC hum=%u.%02u%% press=%uhPa%s",
		                 rx[1], rx[2], (temp < 0) ? "-" : "", t_abs / 100, t_abs % 100,
		                 hum / 100, hum % 100, press, duplicate ? " duplicate" : "");
		
		last_seq = rx[2];
		have_seq = true;
		
		memset(ack, 0, sizeof(ack));
		ack[0] = PKT_TYPE_ACK;
		ack[1] = rx[1];
		ack[2] = rx[2];
		packet_seal(ack);
		vTaskDelay(pdMS_TO_TICKS(10));

		if(nrf24_send(ack, NRF24_PAYLOAD_LEN)) {
			ESP_LOGI(TAG, "ACK seq=%u", rx[2]);
		} else {
			ESP_LOGW(TAG, "ACK send failed seq=%u", rx[2]);
		}
		nrf24_set_rx_mode();
	}
}
