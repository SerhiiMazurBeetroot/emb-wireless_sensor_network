#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Read the chip id and calibration. CS is PA4 and stays high when idle. */
bool bme280_init(void);

/* One forced measurement. Temperature is 0.01 C, humidity 0.01 %, pressure whole hPa. */
bool bme280_read(int16_t *temp_c_x100, uint16_t *hum_pct_x100, uint16_t *press_hpa);
