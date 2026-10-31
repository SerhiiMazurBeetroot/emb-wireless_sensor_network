#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * INA219 at 0x40 on I2C1, shunt 0.1 ohm (marked R100).
 * Current LSB is 40 µA. Bus voltage LSB is 4 mV.
 * False when the chip does not answer; both outputs are then 0.
 */
bool ina219_read(int32_t *current_ua, uint16_t *vbus_mv);
