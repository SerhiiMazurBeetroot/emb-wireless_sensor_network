#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Up to this many nodes get a tab in the header and a column on the link screen. */
#define SSD1306_MAX_NODES 2

/* What the node screen shows for one node. */
typedef struct {
	uint8_t node_id;
	bool have_data;        /* false until the first frame from this node */
	bool online;           /* heard recently enough to be considered alive */
	uint32_t age_s;        /* seconds since the last frame */
	int16_t temp_c_x100;
	uint16_t hum_pct_x100;
	uint16_t press_hpa;    /* 0 hides the pressure (AHT20 has no barometer) */
	uint16_t bat_mv;       /* 0 shows "--" */
	bool sensor_err;       /* replaces the readings with "Sensor error" */
	bool low_bat;          /* draws the battery voltage inverted */
} ssd1306_node_view_t;

/* Link statistics screen. Index 0 is the total, 1..SSD1306_MAX_NODES are the nodes. */
typedef struct {
	uint32_t rx[SSD1306_MAX_NODES + 1];
	uint32_t lost[SSD1306_MAX_NODES + 1];
	uint32_t retries[SSD1306_MAX_NODES + 1];
	uint32_t dup[SSD1306_MAX_NODES + 1];
	uint32_t reboots[SSD1306_MAX_NODES + 1];
	uint32_t crc_err;
	uint32_t sensor_err;
	uint32_t low_bat;
} ssd1306_link_view_t;

bool ssd1306_init(void);
/*
 * Draw one node full screen. nodes holds every node so the header can show
 * which ones are online; selected is the index of the node to draw.
 */
void ssd1306_show_node(const ssd1306_node_view_t *nodes, uint8_t count, uint8_t selected);
void ssd1306_show_link(const ssd1306_link_view_t *link);
/* A title and one centred line, e.g. while waiting for the first frame. */
void ssd1306_show_message(const char *title, const char *text);
