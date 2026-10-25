#include "ssd1306.h"

#include "config.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"

#define SSD1306_W 128
#define SSD1306_H 64

static i2c_master_bus_handle_t s_bus;
static esp_lcd_panel_handle_t s_panel;
static uint8_t s_fb[SSD1306_W * SSD1306_H / 8];
static bool s_ready;

/* 5x7 glyphs, bit 0 is the top pixel. */
static const char s_glyph_index[] = " 0123456789ABCDEFHIKLMNOPQRSTUWXY-.%";
static const uint8_t s_glyph[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E},
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63}, {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x08, 0x08, 0x08, 0x08, 0x08}, {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x23, 0x13, 0x08, 0x64, 0x62},
};

static void fb_clear(void){
	memset(s_fb, 0, sizeof(s_fb));
}

static void fb_char(int x, int page, char ch) {
	const char *found = strchr(s_glyph_index, ch);
	const uint8_t *cols = found ? s_glyph[found - s_glyph_index] : s_glyph[0];

	for(int col = 0; col < 5 && (x + col) < SSD1306_W; col++) {
		s_fb[page * SSD1306_W + x + col] = cols[col];
	}
}

static void fb_text(int x, int page, const char *text) {
	while(*text && x < SSD1306_W - 5) {
		fb_char(x, page, *text++);
        x += 6;
	}
}

static void fb_flush(void) {
	if(s_ready) {
		esp_lcd_panel_draw_bitmap(s_panel, 0, 0, SSD1306_W, SSD1306_H, s_fb);
	}
}

bool ssd1306_init(void) {
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
	
	uint8_t addr = SSD1306_I2C_ADDR;
    if (i2c_master_probe(s_bus, addr, 50) != ESP_OK &&
        i2c_master_probe(s_bus, 0x3D, 50) == ESP_OK) {
        addr = 0x3D;
    }
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = addr,
        .scl_speed_hz = 400000,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    esp_lcd_panel_io_handle_t io = NULL;
    if (esp_lcd_new_panel_io_i2c_v2(s_bus, &io_config, &io) != ESP_OK) {
        return false;
    }
    esp_lcd_panel_ssd1306_config_t vendor = {.height = 64};
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = GPIO_NUM_NC,
        .vendor_config = &vendor,
    };
    if (esp_lcd_new_panel_ssd1306(io, &panel_config, &s_panel) != ESP_OK) {
        return false;
    }
    if (esp_lcd_panel_reset(s_panel) != ESP_OK ||
        esp_lcd_panel_init(s_panel) != ESP_OK ||
        esp_lcd_panel_disp_on_off(s_panel, true) != ESP_OK) {
        return false;
    }

    esp_lcd_panel_mirror(s_panel, true, true);
    s_ready = true;
    ssd1306_show_error("WAIT");
    return true;
}

void ssd1306_show_node(uint8_t node, int16_t temp_c_x100, uint16_t hum_pct_x100,
                       uint16_t bat_mv, const char *status)
{
    char line[22];
    int temp_abs = temp_c_x100 < 0 ? -temp_c_x100 : temp_c_x100;
    fb_clear();
    snprintf(line, sizeof(line), "NODE %u", node);
    fb_text(0, 0, line);
    snprintf(line, sizeof(line), "T %s%d.%02dC", temp_c_x100 < 0 ? "-" : "",
             temp_abs / 100, temp_abs % 100);
    fb_text(0, 2, line);
    snprintf(line, sizeof(line), "H %u.%02u%%", hum_pct_x100 / 100, hum_pct_x100 % 100);
    fb_text(0, 4, line);
    if (bat_mv == 0) {
        snprintf(line, sizeof(line), "BAT -- %s", status);
    } else {
        snprintf(line, sizeof(line), "BAT %umV %s", bat_mv, status);
    }
    fb_text(0, 6, line);
    fb_flush();
}
void ssd1306_show_statistics(uint32_t rx_count, uint8_t seq, uint32_t duplicates)
{
    char line[22];
    fb_clear();
    snprintf(line, sizeof(line), "RX %lu", (unsigned long)rx_count);
    fb_text(0, 0, line);
    snprintf(line, sizeof(line), "SEQ %u", seq);
    fb_text(0, 2, line);
    snprintf(line, sizeof(line), "DUP %lu", (unsigned long)duplicates);
    fb_text(0, 4, line);
    fb_flush();
}
void ssd1306_show_error(const char *reason)
{
    char line[22];
    fb_clear();
    snprintf(line, sizeof(line), "ERR %s", reason);
    fb_text(0, 2, line);
    fb_flush();
}

void ssd1306_show_link_stats(uint32_t rx, uint32_t tx, uint32_t lost,
                             uint32_t retries, uint32_t crc_err, uint32_t unknown,
                             uint32_t n1_rx, uint32_t n2_rx) {
	char line[22];

	fb_clear();
	/* One page per row: glyph is 7px, so bit 7 leaves a 1px gap. */
	snprintf(line, sizeof(line), "PKT %lu", (unsigned long)rx);
	fb_text(0, 0, line);
	snprintf(line, sizeof(line), "ACK %lu", (unsigned long)tx);
	fb_text(0, 1, line);
	snprintf(line, sizeof(line), "LOST %lu", (unsigned long)lost);
	fb_text(0, 2, line);
	snprintf(line, sizeof(line), "RETRY %lu", (unsigned long)retries);
	fb_text(0, 3, line);
	snprintf(line, sizeof(line), "CRC %lu", (unsigned long)crc_err);
	fb_text(0, 4, line);
	snprintf(line, sizeof(line), "OTHER %lu", (unsigned long)unknown);
	fb_text(0, 5, line);
	snprintf(line, sizeof(line), "NODE1 PKT %lu", (unsigned long)n1_rx);
	fb_text(0, 6, line);
	snprintf(line, sizeof(line), "NODE2 PKT %lu", (unsigned long)n2_rx);
	fb_text(0, 7, line);
	fb_flush();
}
