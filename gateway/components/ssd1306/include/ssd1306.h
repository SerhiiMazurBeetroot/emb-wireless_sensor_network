#pragma once

#include <stdbool.h>
#include <stdint.h>

bool ssd1306_init(void);
void ssd1306_show_node(uint8_t node, int16_t temp_c_x100, uint16_t hum_pct_x100,
                       uint16_t bat_mv, const char *status);
void ssd1306_show_statistics(uint32_t rx_count, uint8_t seq, uint32_t duplicates);
void ssd1306_show_error(const char *reason);
void ssd1306_show_link_stats(uint32_t rx, uint32_t tx, uint32_t lost,
                             uint32_t retries, uint32_t crc_err, uint32_t unknown,
                             uint32_t n1_rx, uint32_t n2_rx);
