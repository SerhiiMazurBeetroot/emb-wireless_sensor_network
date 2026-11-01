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
#include "protocol.h"
#include "ssd1306.h"
#include "statistics.h"
#include "webui.h"

static const char *TAG = "gateway";

/* Min interval between full OLED redraws (I2C + snprintf blocks IDLE). */
#define OLED_MIN_INTERVAL_US 300000
/* The screen is redrawn this often even without new frames, so the ages keep counting. */
#define OLED_REFRESH_US 1000000
/* Each node gets the whole screen for this long before the next one is shown. */
#define OLED_NODE_PAGE_US 5000000
/*
 * Wait this long after a DATA frame before answering with the ACK. The node
 * needs about 0.4-0.6 ms after its own transmission to flush its FIFO, switch to
 * RX and let the radio settle; an ACK sent sooner is lost and the node retries.
 * The gateway stays in RX during the wait, so it can still hear the other node.
 */
#define ACK_TURNAROUND_US 1500
/*
 * A node is shown as offline after missing three sample intervals in a row.
 * The extra 2 s cover the node's own wake, sensor and retry time.
 */
#define NODE_OFFLINE_AFTER_S (3U * PROTO_SAMPLE_INTERVAL_S + 2U)
#define NODE_COUNT (PROTO_NODE_ID_MAX - PROTO_NODE_ID_MIN + 1U)

/* Latest new (non-repeat) frame from each node, indexed by node id. */
typedef struct {
	bool have;
	proto_frame_t frame;
} node_latest_t;

_Static_assert(NODE_COUNT <= SSD1306_MAX_NODES, "the OLED layout has room for fewer nodes");

static int64_t s_last_oled_us;
/* 0 shows the node pages. 1 and 2 are the two link screens, in button order. */
static uint8_t s_view;
static node_latest_t s_latest[PROTO_NODE_ID_MAX + 1];
/* Which node the screen shows (0 = first node id) and since when. */
static uint8_t s_page;
static int64_t s_page_since_us;

/* Copy the statistics into the table the link screen draws: total first, then each node. */
static void oled_show_link(const stat_snapshot_t *snap)
{
	ssd1306_link_view_t link = {
		.rx = {snap->rx},
		.lost = {snap->lost},
		.retries = {snap->retries},
		.dup = {snap->dup},
		.reboots = {snap->reboots},
		.crc_err = snap->crc_err,
		.sensor_err = snap->err[ERR_SENSOR],
		.low_bat = snap->err[ERR_BATTERY],
	};

	for (uint8_t i = 0; i < NODE_COUNT; i++) {
		const stat_node_t *n = &snap->node[PROTO_NODE_ID_MIN + i];
		link.rx[i + 1] = n->rx;
		link.lost[i + 1] = n->lost;
		link.retries[i + 1] = n->retries;
		link.dup[i + 1] = n->dup;
		link.reboots[i + 1] = n->reboots;
	}
	ssd1306_show_link(&link, (uint8_t)(s_view - 1U));
}

/* Draw whichever screen is selected: one node, or the link statistics. */
static void oled_show_current(void)
{
	stat_snapshot_t snap;

	statistics_snapshot(&snap);
	if (s_view != 0) {
		oled_show_link(&snap);
		return;
	}

	const int64_t now = esp_timer_get_time();
	ssd1306_node_view_t views[NODE_COUNT];

	for (uint8_t i = 0; i < NODE_COUNT; i++) {
		const uint8_t id = (uint8_t)(PROTO_NODE_ID_MIN + i);
		const node_latest_t *latest = &s_latest[id];
		const int64_t last_us = snap.node[id].last_seen_us;
		const uint32_t age_s = (last_us > 0) ? (uint32_t)((now - last_us) / 1000000) : 0U;

		views[i] = (ssd1306_node_view_t){
			.node_id = id,
			.have_data = latest->have,
			.online = latest->have && age_s < NODE_OFFLINE_AFTER_S,
			.age_s = age_s,
			.temp_c_x100 = latest->frame.temp_c_x100,
			.hum_pct_x100 = latest->frame.hum_pct_x100,
			.press_hpa = latest->frame.press_hpa,
			.bat_mv = latest->frame.bat_mv,
			.sensor_err = (latest->frame.flags & PROTO_FLAG_SENSOR_ERR) != 0,
			.low_bat = (latest->frame.flags & PROTO_FLAG_LOW_BAT) != 0,
		};
	}
	ssd1306_show_node(views, NODE_COUNT, s_page);
}

static void stats_button_poll(void)
{
	if (!button_pressed()) {
		return;
	}

	s_view = (uint8_t)((s_view + 1U) % (1U + SSD1306_LINK_PAGES));
	ESP_LOGI(TAG, "oled view %s", s_view == 0 ? "nodes" : (s_view == 1 ? "traffic" : "faults"));
	if (s_view == 0) {
		s_page_since_us = esp_timer_get_time();
	} else {
		statistics_log();
	}
	oled_show_current();
	s_last_oled_us = esp_timer_get_time();
}

/*
 * Called every loop pass. Moves to the next node after OLED_NODE_PAGE_US, and
 * redraws once a second so the age keeps counting and a silent node turns OFF
 * on time.
 */
static void oled_refresh_if_due(void)
{
	const int64_t now = esp_timer_get_time();
	bool redraw = (now - s_last_oled_us) >= OLED_REFRESH_US;

	if (s_view == 0 && (now - s_page_since_us) >= OLED_NODE_PAGE_US) {
		s_page = (uint8_t)((s_page + 1U) % NODE_COUNT);
		s_page_since_us = now;
		redraw = true;
	}
	if (redraw) {
		oled_show_current();
		s_last_oled_us = esp_timer_get_time();
	}
}

/* A node just sent new readings: show it now and give it a full page time. */
static void oled_show_node_now(uint8_t node_id)
{
	s_page = (uint8_t)(node_id - PROTO_NODE_ID_MIN);
	s_page_since_us = esp_timer_get_time();
	if ((s_page_since_us - s_last_oled_us) >= OLED_MIN_INTERVAL_US) {
		oled_show_current();
		s_last_oled_us = esp_timer_get_time();
	}
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

	ESP_LOGI(TAG, "WSN gateway start, frame %u bytes", (unsigned)PROTO_FRAME_LEN);
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
	webui_start();
	ESP_LOGI(TAG, "RX mode (IRQ), channel 76");

	while (1) {
		stats_button_poll();
		oled_refresh_if_due();
		if (xQueueReceive(rx_q, rx, pdMS_TO_TICKS(20)) != pdTRUE) {
			continue;
		}

		proto_frame_t f;
		if (!proto_decode(rx, &f)) {
			statistics_on_crc();
			statistics_log();
			ESP_LOGW(TAG, "bad CRC");
			continue;
		}

		if (f.type != PROTO_TYPE_DATA || f.node_id < PROTO_NODE_ID_MIN ||
		    f.node_id > PROTO_NODE_ID_MAX) {
			statistics_on_unknown(f.type, f.node_id);
			statistics_log();
			ESP_LOGW(TAG, "ignored type=0x%02X node=%u", f.type, f.node_id);
			continue;
		}

		/* Every valid DATA frame gets an ACK, repeats included: the node is still waiting. */
		const bool duplicate = statistics_on_data(&f, esp_timer_get_time());
		proto_encode_ack(&f, webui_mode(), ack);
		esp_rom_delay_us(ACK_TURNAROUND_US);
		const bool ack_ok = nrf24_send(ack, NRF24_PAYLOAD_LEN);
		statistics_on_ack(f.node_id, ack_ok);
		if (ack_ok) {
			ESP_LOGI(TAG, "ACK seq=%u node=%u attempt=%u", f.seq, f.node_id, f.attempt);
		} else {
			statistics_log();
			ESP_LOGW(TAG, "ACK send failed seq=%u", f.seq);
		}
		nrf24_set_rx_mode();

		if (duplicate) {
			ESP_LOGD(TAG, "DATA node=%u seq=%u duplicate", f.node_id, f.seq);
			continue;
		}

		const int16_t t_abs = (f.temp_c_x100 < 0) ? (int16_t)(-f.temp_c_x100) : f.temp_c_x100;
		ESP_LOGI(TAG,
		         "DATA node=%u seq=%u temp=%s%d.%02dC hum=%u.%02u%% press=%uhPa bat=%umV "
		         "current=%lduA vbus=%umV mode=%u flags=0x%02X attempt=%u",
		         f.node_id, f.seq, (f.temp_c_x100 < 0) ? "-" : "", t_abs / 100, t_abs % 100,
		         f.hum_pct_x100 / 100, f.hum_pct_x100 % 100, f.press_hpa, f.bat_mv,
		         (long)f.current_ua, f.vbus_mv, f.mode, f.flags, f.attempt);
		if (f.flags & PROTO_FLAG_BOOT) {
			ESP_LOGI(TAG, "node %u (re)started", f.node_id);
		}

		s_latest[f.node_id] = (node_latest_t){.have = true, .frame = f};
		webui_note(&f);
		oled_show_node_now(f.node_id);
	}
}
