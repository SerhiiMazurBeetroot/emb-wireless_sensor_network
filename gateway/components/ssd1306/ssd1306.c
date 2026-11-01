#include "ssd1306.h"

#include "config.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "u8g2.h"

#define OLED_W 128
#define OLED_H 64
#define OLED_I2C_HZ 400000
#define OLED_I2C_TIMEOUT_MS 50

/* Header: title on the left, one tab per node on the right, a rule below. */
#define HEADER_RULE_Y 12
#define TAB_W 18
/* Bottom line of the node screen sits under this rule. */
#define FOOTER_RULE_Y 50

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static u8g2_t s_u8g2;
static bool s_ready;

/*
 * u8g2 hands over one I2C message in pieces between START and END. They are
 * collected here and sent as a single write. 1 KB holds even a full frame.
 */
static uint8_t s_i2c_buf[1024 + 8];
static size_t s_i2c_len;

static uint8_t u8x8_byte_esp_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	(void)u8x8;

	switch (msg) {
	case U8X8_MSG_BYTE_INIT:
	case U8X8_MSG_BYTE_SET_DC:
		return 1;
	case U8X8_MSG_BYTE_START_TRANSFER:
		s_i2c_len = 0;
		return 1;
	case U8X8_MSG_BYTE_SEND:
		if (s_i2c_len + arg_int <= sizeof(s_i2c_buf)) {
			memcpy(&s_i2c_buf[s_i2c_len], arg_ptr, arg_int);
			s_i2c_len += arg_int;
		}
		return 1;
	case U8X8_MSG_BYTE_END_TRANSFER:
		if (s_i2c_len > 0) {
			(void)i2c_master_transmit(s_dev, s_i2c_buf, s_i2c_len, OLED_I2C_TIMEOUT_MS);
		}
		return 1;
	default:
		return 0;
	}
}

/* No reset or DC pins on an I2C module: only the delays u8g2 asks for during init. */
static uint8_t u8x8_gpio_delay_esp(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	(void)u8x8;
	(void)arg_ptr;

	switch (msg) {
	case U8X8_MSG_DELAY_MILLI:
		/* With a 10 ms RTOS tick, shorter waits must busy-wait or they round down to 0. */
		if (arg_int >= 10U) {
			vTaskDelay(pdMS_TO_TICKS(arg_int));
		} else {
			esp_rom_delay_us((uint32_t)arg_int * 1000U);
		}
		break;
	case U8X8_MSG_DELAY_10MICRO:
		esp_rom_delay_us((uint32_t)arg_int * 10U);
		break;
	case U8X8_MSG_DELAY_100NANO:
		esp_rom_delay_us(1);
		break;
	default:
		break;
	}
	return 1;
}

bool ssd1306_init(void)
{
	i2c_master_bus_config_t bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_NUM_0,
		.scl_io_num = SSD1306_PIN_SCL,
		.sda_io_num = SSD1306_PIN_SDA,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	if (i2c_new_master_bus(&bus_config, &s_bus) != ESP_OK) {
		return false;
	}

	/* Most modules answer on 0x3C; some are strapped to 0x3D. */
	uint16_t addr = SSD1306_I2C_ADDR;
	if (i2c_master_probe(s_bus, addr, 50) != ESP_OK &&
	    i2c_master_probe(s_bus, 0x3D, 50) == ESP_OK) {
		addr = 0x3D;
	}

	i2c_device_config_t dev_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr,
		.scl_speed_hz = OLED_I2C_HZ,
	};
	if (i2c_master_bus_add_device(s_bus, &dev_config, &s_dev) != ESP_OK) {
		return false;
	}

	/*
	 * U8G2_R0 is the orientation this module needs. u8g2's SSD1306 setup already
	 * matches how it is mounted; R2 (180 degrees) draws the text upside down.
	 */
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&s_u8g2, U8G2_R0, u8x8_byte_esp_i2c,
	                                       u8x8_gpio_delay_esp);
	u8g2_InitDisplay(&s_u8g2);
	u8g2_SetPowerSave(&s_u8g2, 0);
	u8g2_SetFontMode(&s_u8g2, 1);

	s_ready = true;
	ssd1306_show_message("Gateway", "Waiting for nodes");
	return true;
}

static void draw_right(int x_right, int y, const char *text)
{
	u8g2_DrawStr(&s_u8g2, x_right - u8g2_GetStrWidth(&s_u8g2, text), y, text);
}

static void draw_centered(int y, const char *text)
{
	u8g2_DrawUTF8(&s_u8g2, (OLED_W - u8g2_GetUTF8Width(&s_u8g2, text)) / 2, y, text);
}

/*
 * Title on the left and, when nodes is given, a tab per node on the right:
 * the node number plus a dot that is filled while the node is online and
 * hollow when it is offline. The node being shown has an inverted tab.
 */
static void draw_header(const char *title, const ssd1306_node_view_t *nodes, uint8_t count,
                        int selected)
{
	u8g2_SetFont(&s_u8g2, u8g2_font_helvB08_tr);
	u8g2_DrawStr(&s_u8g2, 0, 9, title);

	if (nodes != NULL) {
		int x = OLED_W - count * TAB_W;

		for (uint8_t i = 0; i < count; i++, x += TAB_W) {
			const bool sel = (i == selected);
			char digit[4];

			if (sel) {
				u8g2_DrawBox(&s_u8g2, x, 0, TAB_W - 2, 11);
				u8g2_SetDrawColor(&s_u8g2, 0);
			} else {
				u8g2_DrawFrame(&s_u8g2, x, 0, TAB_W - 2, 11);
			}
			snprintf(digit, sizeof(digit), "%u", nodes[i].node_id);
			u8g2_DrawStr(&s_u8g2, x + 2, 9, digit);
			if (nodes[i].online) {
				u8g2_DrawDisc(&s_u8g2, x + TAB_W - 7, 5, 2, U8G2_DRAW_ALL);
			} else {
				u8g2_DrawCircle(&s_u8g2, x + TAB_W - 7, 5, 2, U8G2_DRAW_ALL);
			}
			u8g2_SetDrawColor(&s_u8g2, 1);
		}
	}

	u8g2_DrawHLine(&s_u8g2, 0, HEADER_RULE_Y, OLED_W);
}

/* Big temperature on the left, humidity on the right, both rounded for reading at a glance. */
static void draw_readings(const ssd1306_node_view_t *v)
{
	char text[16];
	const int t_abs = v->temp_c_x100 < 0 ? -v->temp_c_x100 : v->temp_c_x100;
	const int t_tenths = (t_abs + 5) / 10;

	snprintf(text, sizeof(text), "%s%d.%d", v->temp_c_x100 < 0 ? "-" : "", t_tenths / 10,
	         t_tenths % 10);
	u8g2_SetFont(&s_u8g2, u8g2_font_logisoso22_tf);
	u8g2_DrawStr(&s_u8g2, 0, 42, text);
	const int t_w = u8g2_GetStrWidth(&s_u8g2, text);

	u8g2_SetFont(&s_u8g2, u8g2_font_helvB10_tf);
	u8g2_DrawUTF8(&s_u8g2, t_w + 2, 30, "°C");

	snprintf(text, sizeof(text), "%u%%", (unsigned)((v->hum_pct_x100 + 50U) / 100U));
	u8g2_SetFont(&s_u8g2, u8g2_font_helvB12_tr);
	draw_right(OLED_W, 42, text);
	u8g2_SetFont(&s_u8g2, u8g2_font_5x8_tr);
	draw_right(OLED_W, 26, "RH");
}

/* Pressure on the left, battery in the middle, time since the last frame on the right. */
static void draw_footer(const ssd1306_node_view_t *v)
{
	char text[16];

	u8g2_DrawHLine(&s_u8g2, 0, FOOTER_RULE_Y, OLED_W);
	u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tr);

	if (v->press_hpa != 0) {
		snprintf(text, sizeof(text), "%uhPa", v->press_hpa);
		u8g2_DrawStr(&s_u8g2, 0, 62, text);
	}

	if (v->bat_mv == 0) {
		snprintf(text, sizeof(text), "--V");
	} else {
		snprintf(text, sizeof(text), "%u.%02uV", v->bat_mv / 1000U, (v->bat_mv % 1000U) / 10U);
	}
	const int bat_x = 52;
	if (v->low_bat) {
		u8g2_DrawBox(&s_u8g2, bat_x - 1, FOOTER_RULE_Y + 2, u8g2_GetStrWidth(&s_u8g2, text) + 2,
		             12);
		u8g2_SetDrawColor(&s_u8g2, 0);
	}
	u8g2_DrawStr(&s_u8g2, bat_x, 62, text);
	u8g2_SetDrawColor(&s_u8g2, 1);

	/* "12s ago" while online; "off 2m" says how long the node has been silent. */
	char age[12];
	if (v->age_s < 60U) {
		snprintf(age, sizeof(age), "%lus", (unsigned long)v->age_s);
	} else if (v->age_s < 3600U) {
		snprintf(age, sizeof(age), "%lum", (unsigned long)(v->age_s / 60U));
	} else {
		snprintf(age, sizeof(age), "%luh", (unsigned long)(v->age_s / 3600U));
	}
	snprintf(text, sizeof(text), v->online ? "%s ago" : "off %s", age);
	draw_right(OLED_W, 62, text);
}

void ssd1306_show_node(const ssd1306_node_view_t *nodes, uint8_t count, uint8_t selected)
{
	if (!s_ready || nodes == NULL || selected >= count) {
		return;
	}

	const ssd1306_node_view_t *v = &nodes[selected];
	char title[16];

	u8g2_ClearBuffer(&s_u8g2);
	snprintf(title, sizeof(title), "Node %u", v->node_id);
	draw_header(title, nodes, count, selected);

	if (!v->have_data) {
		u8g2_SetFont(&s_u8g2, u8g2_font_helvB10_tf);
		draw_centered(36, "No data yet");
	} else {
		if (v->sensor_err) {
			u8g2_SetFont(&s_u8g2, u8g2_font_helvB10_tf);
			draw_centered(36, "Sensor error");
		} else {
			draw_readings(v);
		}
		draw_footer(v);
	}

	u8g2_SendBuffer(&s_u8g2);
}

/* Two node columns. The right edges line up with the "n1" and "n2" marks in the header. */
static void draw_two(int y, const char *label, uint32_t a, uint32_t b)
{
	char text[12];

	u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&s_u8g2, 0, y, label);
	snprintf(text, sizeof(text), "%lu", (unsigned long)a);
	draw_right(98, y, text);
	snprintf(text, sizeof(text), "%lu", (unsigned long)b);
	draw_right(OLED_W, y, text);
}

static void draw_one(int y, const char *label, const char *value)
{
	u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&s_u8g2, 0, y, label);
	draw_right(OLED_W, y, value);
}

static void draw_link_traffic(const ssd1306_link_view_t *link)
{
	draw_header("Traffic", NULL, 0, -1);
	u8g2_SetFont(&s_u8g2, u8g2_font_helvB08_tr);
	draw_right(98, 9, "n1");
	draw_right(OLED_W, 9, "n2");

	draw_two(26, "rx", link->rx[1], link->rx[2]);
	draw_two(38, "lost", link->lost[1], link->lost[2]);
	draw_two(50, "retry", link->retries[1], link->retries[2]);
	draw_two(62, "dup", link->dup[1], link->dup[2]);
}

static void draw_link_faults(const ssd1306_link_view_t *link)
{
	char text[32];

	draw_header("Faults", NULL, 0, -1);

	snprintf(text, sizeof(text), "%lu", (unsigned long)link->crc_err);
	draw_one(26, "crc", text);
	snprintf(text, sizeof(text), "%lu", (unsigned long)link->sensor_err);
	draw_one(38, "sensor", text);
	snprintf(text, sizeof(text), "%lu", (unsigned long)link->low_bat);
	draw_one(50, "battery", text);
	snprintf(text, sizeof(text), "n1 %lu   n2 %lu", (unsigned long)link->reboots[1],
	         (unsigned long)link->reboots[2]);
	draw_one(62, "reboot", text);
}

void ssd1306_show_link(const ssd1306_link_view_t *link, uint8_t page)
{
	if (!s_ready || link == NULL) {
		return;
	}

	u8g2_ClearBuffer(&s_u8g2);
	if (page == 0) {
		draw_link_traffic(link);
	} else {
		draw_link_faults(link);
	}
	u8g2_SendBuffer(&s_u8g2);
}

void ssd1306_show_message(const char *title, const char *text)
{
	if (!s_ready) {
		return;
	}

	u8g2_ClearBuffer(&s_u8g2);
	draw_header(title, NULL, 0, -1);
	u8g2_SetFont(&s_u8g2, u8g2_font_helvB10_tf);
	draw_centered(38, text);
	u8g2_SendBuffer(&s_u8g2);
}
