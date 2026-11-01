#include "webui.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "webui";

#define WEBUI_SSID "WSN-Gateway"
#define WEBUI_PASS "wsn-gateway"
/* Channel 1 (2412 MHz) stays clear of nRF24 channel 76 (2476 MHz). */
#define WEBUI_CHANNEL 1
/* Two nodes every 10 s: 800 samples is a little over 60 minutes, so the
 * page's 60 min window still has a point at its left edge. */
#define WEBUI_CAP 800
#define WEBUI_LOG_LINES 64
#define WEBUI_LOG_LINE 192

extern const uint8_t web_page_html_start[] asm("_binary_web_page_html_start");
extern const uint8_t web_page_html_end[] asm("_binary_web_page_html_end");

typedef struct {
	uint32_t t_s;
	uint8_t node;
	uint8_t mode;
	uint8_t flags;
	int16_t temp_c_x100;
	uint16_t hum_pct_x100;
	uint16_t press_hpa;
	uint16_t bat_mv;
	int32_t current_ua;
	uint16_t vbus_mv;
} webui_sample_t;

static webui_sample_t s_samples[WEBUI_CAP];
static uint16_t s_count;
static uint16_t s_head;
static SemaphoreHandle_t s_lock;
static volatile uint8_t s_mode = PROTO_MODE_DEEP;

typedef struct {
	uint32_t seq;
	char text[WEBUI_LOG_LINE];
} webui_log_t;

static webui_log_t s_log[WEBUI_LOG_LINES];
static uint16_t s_log_count;
static uint16_t s_log_head;
static uint32_t s_log_last;
static char s_log_partial[WEBUI_LOG_LINE];
static size_t s_log_partial_len;
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_uart_log;

static void log_commit_locked(void)
{
	webui_log_t *line = &s_log[s_log_head];

	s_log_partial[s_log_partial_len] = '\0';
	s_log_last++;
	line->seq = s_log_last;
	memcpy(line->text, s_log_partial, s_log_partial_len + 1U);
	s_log_head = (uint16_t)((s_log_head + 1U) % WEBUI_LOG_LINES);
	if (s_log_count < WEBUI_LOG_LINES) {
		s_log_count++;
	}
	s_log_partial_len = 0;
}

/* ESP-IDF prints one UART line as several vprintf calls. Join them until '\n'. */
static void log_feed(const char *chunk)
{
	portENTER_CRITICAL(&s_log_mux);
	while (chunk[0] != '\0') {
		if (chunk[0] == '\033') {
			while (chunk[0] != '\0' && chunk[0] != 'm') {
				chunk++;
			}
			if (chunk[0] == 'm') {
				chunk++;
			}
			continue;
		}
		if (chunk[0] == '\n' || chunk[0] == '\r') {
			if (s_log_partial_len > 0) {
				log_commit_locked();
			}
			chunk++;
			continue;
		}
		if (s_log_partial_len >= WEBUI_LOG_LINE - 1U) {
			log_commit_locked();
		}
		s_log_partial[s_log_partial_len++] = chunk[0];
		chunk++;
	}
	portEXIT_CRITICAL(&s_log_mux);
}

static int log_hook(const char *fmt, va_list args)
{
	char chunk[WEBUI_LOG_LINE];
	va_list copy;

	va_copy(copy, args);
	(void)vsnprintf(chunk, sizeof(chunk), fmt, copy);
	va_end(copy);
	log_feed(chunk);
	return (s_uart_log != NULL) ? s_uart_log(fmt, args) : 0;
}

static void log_hook_install(void)
{
	s_uart_log = esp_log_set_vprintf(log_hook);
}

static const char *mode_name(uint8_t mode)
{
	if (mode == PROTO_MODE_ACTIVE) {
		return "active";
	}
	if (mode == PROTO_MODE_LIGHT) {
		return "light";
	}
	return "deep";
}

static bool append(char *buf, size_t cap, size_t *used, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	const int wrote = vsnprintf(buf + *used, cap - *used, fmt, ap);
	va_end(ap);
	if (wrote < 0 || (size_t)wrote >= cap - *used) {
		return false;
	}
	*used += (size_t)wrote;
	return true;
}

static const webui_sample_t *sample_at(uint16_t index)
{
	const uint16_t start = (s_count < WEBUI_CAP) ? 0 : s_head;
	return &s_samples[(start + index) % WEBUI_CAP];
}

uint8_t webui_mode(void)
{
	return s_mode;
}

void webui_note(const proto_frame_t *frame)
{
	if (s_lock == NULL || frame == NULL) {
		return;
	}
	if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
		return;
	}

	webui_sample_t *s = &s_samples[s_head];
	s->t_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
	s->node = frame->node_id;
	s->mode = frame->mode;
	s->flags = frame->flags;
	s->temp_c_x100 = frame->temp_c_x100;
	s->hum_pct_x100 = frame->hum_pct_x100;
	s->press_hpa = frame->press_hpa;
	s->bat_mv = frame->bat_mv;
	s->current_ua = frame->current_ua;
	s->vbus_mv = frame->vbus_mv;

	s_head = (uint16_t)((s_head + 1U) % WEBUI_CAP);
	if (s_count < WEBUI_CAP) {
		s_count++;
	}
	xSemaphoreGive(s_lock);
}

static esp_err_t send_page(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store");
	return httpd_resp_send(req, (const char *)web_page_html_start,
	                       (ssize_t)(web_page_html_end - web_page_html_start));
}

static esp_err_t send_mode(httpd_req_t *req)
{
	char body[24];
	const int n = snprintf(body, sizeof(body), "{\"mode\":%u}", (unsigned)s_mode);
	httpd_resp_set_type(req, "application/json");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store");
	return httpd_resp_send(req, body, n);
}

static esp_err_t set_mode(httpd_req_t *req)
{
	char body[16];
	int left = req->content_len;
	int got = 0;

	if (left <= 0 || left >= (int)sizeof(body)) {
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "mode");
		return ESP_FAIL;
	}
	while (left > 0) {
		const int n = httpd_req_recv(req, body + got, (size_t)left);
		if (n <= 0) {
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "mode");
			return ESP_FAIL;
		}
		got += n;
		left -= n;
	}
	body[got] = '\0';

	int mode = -1;
	if (sscanf(body, "mode=%d", &mode) != 1 || mode < (int)PROTO_MODE_ACTIVE ||
	    mode > (int)PROTO_MODE_DEEP) {
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "mode");
		return ESP_FAIL;
	}

	s_mode = (uint8_t)mode;
	ESP_LOGI(TAG, "mode %s", mode_name(s_mode));
	return send_mode(req);
}

static esp_err_t send_samples(httpd_req_t *req)
{
	const size_t cap = WEBUI_CAP * 96U + 128U;
	char *buf = malloc(cap);
	if (buf == NULL) {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "nomem");
		return ESP_FAIL;
	}

	size_t used = 0;
	bool ok = true;
	if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
		free(buf);
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "busy");
		return ESP_FAIL;
	}
	const uint32_t now_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
	ok = append(buf, cap, &used, "{\"mode\":%u,\"now\":%lu,\"samples\":[", (unsigned)s_mode,
	            (unsigned long)now_s);
	for (uint16_t i = 0; ok && i < s_count; i++) {
		const webui_sample_t *s = sample_at(i);
		ok = append(buf, cap, &used, "%s[%lu,%u,%u,%d,%u,%u,%u,%ld,%u,%u]", (i == 0) ? "" : ",",
		            (unsigned long)s->t_s, s->node, s->mode, s->temp_c_x100, s->hum_pct_x100,
		            s->press_hpa, s->bat_mv, (long)s->current_ua, s->vbus_mv, s->flags);
	}
	ok = ok && append(buf, cap, &used, "]}");
	xSemaphoreGive(s_lock);

	esp_err_t err = ESP_FAIL;
	if (ok) {
		httpd_resp_set_type(req, "application/json");
		httpd_resp_set_hdr(req, "Cache-Control", "no-store");
		err = httpd_resp_send(req, buf, (ssize_t)used);
	} else {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "json");
	}
	free(buf);
	return err;
}

static esp_err_t send_csv(httpd_req_t *req)
{
	const size_t cap = WEBUI_CAP * 112U + 128U;
	char *buf = malloc(cap);
	if (buf == NULL) {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "nomem");
		return ESP_FAIL;
	}

	size_t used = 0;
	bool ok = append(buf, cap, &used,
	                 "time_s,node,mode,temp_c,humidity_pct,pressure_hpa,battery_mv,current_ua,"
	                 "voltage_mv,flags\r\n");
	if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
		free(buf);
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "busy");
		return ESP_FAIL;
	}
	for (uint16_t i = 0; ok && i < s_count; i++) {
		const webui_sample_t *s = sample_at(i);
		const int temp_abs = (s->temp_c_x100 < 0) ? -s->temp_c_x100 : s->temp_c_x100;
		ok = append(buf, cap, &used, "%lu,%u,%s,%s%d.%02d,%u.%02u,%u,%u,%ld,%u,0x%02X\r\n",
		            (unsigned long)s->t_s, s->node, mode_name(s->mode),
		            (s->temp_c_x100 < 0) ? "-" : "", temp_abs / 100, temp_abs % 100,
		            s->hum_pct_x100 / 100, s->hum_pct_x100 % 100, s->press_hpa, s->bat_mv,
		            (long)s->current_ua, s->vbus_mv, s->flags);
	}
	xSemaphoreGive(s_lock);

	esp_err_t err = ESP_FAIL;
	if (ok) {
		httpd_resp_set_type(req, "text/csv");
		httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"wsn.csv\"");
		httpd_resp_set_hdr(req, "Cache-Control", "no-store");
		err = httpd_resp_send(req, buf, (ssize_t)used);
	} else {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "csv");
	}
	free(buf);
	return err;
}

static bool json_escape(char *buf, size_t cap, size_t *used, const char *text)
{
	for (const char *p = text; *p != '\0'; p++) {
		const char *esc = NULL;
		char plain[2] = {*p, '\0'};

		if (*p == '"' || *p == '\\') {
			esc = (*p == '"') ? "\\\"" : "\\\\";
		} else if ((unsigned char)*p < 0x20) {
			continue;
		}
		if (!append(buf, cap, used, "%s", (esc != NULL) ? esc : plain)) {
			return false;
		}
	}
	return true;
}

static esp_err_t send_logs(httpd_req_t *req)
{
	char query[48];
	uint32_t after = 0;
	bool have_after = false;

	if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
		char value[16];
		if (httpd_query_key_value(query, "after", value, sizeof(value)) == ESP_OK) {
			after = (uint32_t)strtoul(value, NULL, 10);
			have_after = true;
		}
	}

	webui_log_t *copy = calloc(WEBUI_LOG_LINES, sizeof(webui_log_t));
	if (copy == NULL) {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "nomem");
		return ESP_FAIL;
	}

	uint16_t n = 0;
	uint32_t next = have_after ? after : 0;
	portENTER_CRITICAL(&s_log_mux);
	const uint16_t start = (s_log_count < WEBUI_LOG_LINES) ? 0 : s_log_head;
	for (uint16_t i = 0; i < s_log_count; i++) {
		const webui_log_t *line = &s_log[(start + i) % WEBUI_LOG_LINES];
		if (have_after && line->seq <= after) {
			continue;
		}
		copy[n++] = *line;
		next = line->seq;
	}
	portEXIT_CRITICAL(&s_log_mux);

	const size_t cap = WEBUI_LOG_LINES * (WEBUI_LOG_LINE + 8U) + 64U;
	char *buf = malloc(cap);
	if (buf == NULL) {
		free(copy);
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "nomem");
		return ESP_FAIL;
	}

	size_t used = 0;
	bool ok = append(buf, cap, &used, "{\"next\":%lu,\"lines\":[", (unsigned long)next);
	for (uint16_t i = 0; ok && i < n; i++) {
		ok = append(buf, cap, &used, "%s\"", (i == 0) ? "" : ",");
		ok = ok && json_escape(buf, cap, &used, copy[i].text);
		ok = ok && append(buf, cap, &used, "\"");
	}
	ok = ok && append(buf, cap, &used, "]}");
	free(copy);

	esp_err_t err = ESP_FAIL;
	if (ok) {
		httpd_resp_set_type(req, "application/json");
		httpd_resp_set_hdr(req, "Cache-Control", "no-store");
		err = httpd_resp_send(req, buf, (ssize_t)used);
	} else {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "json");
	}
	free(buf);
	return err;
}

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
	(void)arg;
	(void)base;
	if (id == WIFI_EVENT_AP_STACONNECTED) {
		const wifi_event_ap_staconnected_t *ev = data;
		ESP_LOGI(TAG, "client connected, aid %u", ev->aid);
	} else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
		ESP_LOGI(TAG, "client disconnected");
	}
}

static bool uri(httpd_handle_t server, const char *path, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *))
{
	const httpd_uri_t route = {
		.uri = path,
		.method = method,
		.handler = handler,
	};
	return httpd_register_uri_handler(server, &route) == ESP_OK;
}

void webui_start(void)
{
	log_hook_install();
	s_lock = xSemaphoreCreateMutex();
	if (s_lock == NULL) {
		ESP_LOGE(TAG, "mutex failed");
		return;
	}

	if (esp_netif_init() != ESP_OK || esp_event_loop_create_default() != ESP_OK) {
		ESP_LOGE(TAG, "netif init failed");
		return;
	}
	if (esp_netif_create_default_wifi_ap() == NULL) {
		ESP_LOGE(TAG, "AP netif failed");
		return;
	}

	wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
	if (esp_wifi_init(&init_cfg) != ESP_OK) {
		ESP_LOGE(TAG, "wifi init failed");
		return;
	}
	if (esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL, NULL) != ESP_OK) {
		ESP_LOGE(TAG, "wifi handler failed");
		return;
	}

	wifi_config_t wifi = {0};
	strncpy((char *)wifi.ap.ssid, WEBUI_SSID, sizeof(wifi.ap.ssid) - 1);
	wifi.ap.ssid_len = (uint8_t)strlen(WEBUI_SSID);
	strncpy((char *)wifi.ap.password, WEBUI_PASS, sizeof(wifi.ap.password) - 1);
	wifi.ap.channel = WEBUI_CHANNEL;
	wifi.ap.max_connection = 4;
	wifi.ap.authmode = WIFI_AUTH_WPA2_PSK;

	if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK || esp_wifi_set_config(WIFI_IF_AP, &wifi) != ESP_OK ||
	    esp_wifi_start() != ESP_OK) {
		ESP_LOGE(TAG, "AP start failed");
		return;
	}
	esp_wifi_set_ps(WIFI_PS_NONE);

	httpd_handle_t server = NULL;
	httpd_config_t http = HTTPD_DEFAULT_CONFIG();
	http.max_uri_handlers = 8;
	http.stack_size = 8192;
	http.lru_purge_enable = true;
	if (httpd_start(&server, &http) != ESP_OK) {
		ESP_LOGE(TAG, "http start failed");
		return;
	}
	if (!uri(server, "/", HTTP_GET, send_page) || !uri(server, "/api/samples", HTTP_GET, send_samples) ||
	    !uri(server, "/api/mode", HTTP_GET, send_mode) || !uri(server, "/api/mode", HTTP_POST, set_mode) ||
	    !uri(server, "/api/export.csv", HTTP_GET, send_csv) || !uri(server, "/api/logs", HTTP_GET, send_logs)) {
		ESP_LOGE(TAG, "route register failed");
		return;
	}

	ESP_LOGI(TAG, "AP %s password %s, open http://192.168.4.1", WEBUI_SSID, WEBUI_PASS);
}
