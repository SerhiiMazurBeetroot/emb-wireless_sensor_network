#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "button.h"
#include "config.h"
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
/*
 * Wait this long after a DATA frame before answering with the ACK. The node
 * needs about 0.4-0.6 ms after its own transmission to flush its FIFO, switch to
 * RX and let the radio settle; an ACK sent sooner is lost and the node retries.
 * The gateway stays in RX during the wait, so it can still hear the other node.
 */
#define ACK_TURNAROUND_US 1500

static int64_t s_last_oled_us;
static bool s_show_stats;
static uint8_t s_view_nid;
static int16_t s_view_temp;
static uint16_t s_view_hum;
static uint16_t s_view_bat;
static bool s_view_have_node;

static void oled_show_current(void)
{
	if (s_show_stats) {
		stat_snapshot_t snap;

		statistics_snapshot(&snap);
		ssd1306_show_link_stats(snap.rx, snap.tx, snap.lost, snap.retries, snap.crc_err,
		                        snap.unknown, snap.node[1].rx, snap.node[2].rx);
		return;
	}

	ssd1306_show_node(s_view_have_node ? s_view_nid : 0, s_view_temp, s_view_hum, s_view_bat,
	                  "OK");
}

static void stats_button_poll(void)
{
	if (!button_pressed()) {
		return;
	}

	s_show_stats = !s_show_stats;
	ESP_LOGI(TAG, "stats view %s", s_show_stats ? "on" : "off");
	if (s_show_stats) {
		statistics_log();
	}
	oled_show_current();
	s_last_oled_us = esp_timer_get_time();
}

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

	ESP_LOGI(TAG, "WSN gateway start");
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
	button_init(GATEWAY_PIN_STATS);
	ESP_LOGI(TAG, "RX mode (IRQ), channel 76");

	while (1) {
		stats_button_poll();
		if (xQueueReceive(rx_q, rx, pdMS_TO_TICKS(20)) != pdTRUE) {
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
		esp_rom_delay_us(ACK_TURNAROUND_US);
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

		if (!duplicate) {
			s_view_nid = nid;
			s_view_temp = temp;
			s_view_hum = hum;
			s_view_bat = get_u16(&rx[9]);
			s_view_have_node = true;
		}

		if (!duplicate && (esp_timer_get_time() - s_last_oled_us) >= OLED_MIN_INTERVAL_US) {
			oled_show_current();
			s_last_oled_us = esp_timer_get_time();
		}
	}
}
