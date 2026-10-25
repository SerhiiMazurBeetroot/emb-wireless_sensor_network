#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nrf24.h"
#include "nvs_flash.h"
#include "ssd1306.h"
#include "statistics.h"

static const char *TAG = "gateway";

#define PKT_TYPE_DATA 0x01
#define PKT_TYPE_ACK 0x02
#define PKT_NODE_ID_MIN 0x01U
#define PKT_NODE_ID_MAX 0x02U
/* Min interval between full OLED redraws (I2C + snprintf blocks IDLE). */
#define OLED_MIN_INTERVAL_US 300000

static int64_t s_last_oled_us;

/* CRC-8, poly 0x07, over bytes 0..(NRF24_PAYLOAD_LEN - 2). */
static uint8_t crc8(const uint8_t *data, uint8_t len) {
	uint8_t crc = 0x00;

	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
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

void app_main(void) {
	uint8_t rx[NRF24_PAYLOAD_LEN];
	uint8_t ack[NRF24_PAYLOAD_LEN];

	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	ESP_LOGI(TAG, "WSN gateway — step 14: statistics");
	ESP_ERROR_CHECK(nrf24_init());

	if (!nrf24_spi_probe()) {
		ESP_LOGE(TAG, "SPI probe failed");
		return;
	}

	if (!ssd1306_init()) {
		ESP_LOGE(TAG, "OLED init failed");
	}
	
	statistics_init();

	nrf24_radio_init();
	QueueHandle_t rx_q = xQueueCreate(8, NRF24_PAYLOAD_LEN);
	if (rx_q == NULL) {
		ESP_LOGE(TAG, "RX queue alloc failed");
		return;
	}
	ESP_ERROR_CHECK(nrf24_rx_start(rx_q));
	nrf24_set_rx_mode();
	ESP_LOGI(TAG, "RX mode (IRQ), channel 76");

	while (1) {
		if (xQueueReceive(rx_q, rx, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		if (!packet_crc_ok(rx)) {
			statistics_on_crc();
			statistics_log();
			ESP_LOGW(TAG, "bad CRC");

			if ((esp_timer_get_time() - s_last_oled_us) >= OLED_MIN_INTERVAL_US) {
				ssd1306_show_error("CRC");
				s_last_oled_us = esp_timer_get_time();
			}
			continue;
		}
		
		if (rx[0] != PKT_TYPE_DATA || rx[1] < PKT_NODE_ID_MIN || rx[1] > PKT_NODE_ID_MAX) {
			statistics_on_unknown(rx[0], rx[1]);
			statistics_log();

			if ((esp_timer_get_time() - s_last_oled_us) >= OLED_MIN_INTERVAL_US) {
				ssd1306_show_error("TYPE");
				s_last_oled_us = esp_timer_get_time();
			}
			ESP_LOGW(TAG, "ignored type=0x%02X node=%u", rx[0], rx[1]);
			continue;
		}

		const uint8_t nid = rx[1];
		const bool duplicate = statistics_on_data(nid, rx[2]);
		memset(ack, 0, sizeof(ack));
		ack[0] = PKT_TYPE_ACK;
		ack[1] = rx[1];
		ack[2] = rx[2];
		packet_seal(ack);
		const bool ack_ok = nrf24_send(ack, NRF24_PAYLOAD_LEN);
		statistics_on_ack(nid, ack_ok);
		if (ack_ok) {
			ESP_LOGI(TAG, "ACK seq=%u node=%u", rx[2], nid);
		} else {
			statistics_log();
			ESP_LOGW(TAG, "ACK send failed seq=%u", rx[2]);
		}
		
		nrf24_set_rx_mode();
		const int16_t temp = get_i16(&rx[3]);
		const uint16_t hum = get_u16(&rx[5]);
		const uint16_t press = get_u16(&rx[7]);
		const int16_t t_abs = (temp < 0) ? (int16_t)(-temp) : temp;

		if (duplicate) {
			ESP_LOGD(TAG, "DATA node=%u seq=%u duplicate", nid, rx[2]);
		} else {
			ESP_LOGI(TAG, "DATA node=%u seq=%u temp=%s%d.%02dC hum=%u.%02u%% press=%uhPa",
			         nid, rx[2], (temp < 0) ? "-" : "", t_abs / 100, t_abs % 100,
			         hum / 100, hum % 100, press);
		}

		if (!duplicate && (esp_timer_get_time() - s_last_oled_us) >= OLED_MIN_INTERVAL_US) {
			stat_snapshot_t snap;
			statistics_snapshot(&snap);
			if ((snap.rx % 4U) == 0U) {
				statistics_log();
				ssd1306_show_link_stats(snap.rx, snap.tx, snap.lost, snap.retries,
				                        snap.crc_err, snap.unknown,
				                        snap.node[1].rx, snap.node[2].rx);
			} else {
				ssd1306_show_node(nid, temp, hum, get_u16(&rx[9]), "OK");
			}
			s_last_oled_us = esp_timer_get_time();
		}
	}
}
