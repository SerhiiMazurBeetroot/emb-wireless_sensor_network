#pragma once

#include <stdbool.h>
#include <stdint.h>


bool ssd1306_init(void);
void ssd1306_show_node(uint8_t node, int16_t temp_c_x100, uint16_t hum_pct_x100, uint16_t bat_mv, const char *status);
void ssd1306_show_statistics(uint32_t rx_count, uint8_t seq, uint32_t duplicates);
void ssd1306_show_error(const char *reason);
