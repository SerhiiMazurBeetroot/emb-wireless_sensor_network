#pragma once

#include <stdbool.h>
#include <stdint.h>

bool aht20_init(void);
bool aht20_read(int16_t *temp_c_x100, uint16_t *hum_pct_x100, uint16_t *press_hpa);
