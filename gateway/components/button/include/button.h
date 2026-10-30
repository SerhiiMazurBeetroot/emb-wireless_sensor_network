#pragma once

#include <stdbool.h>

#include "driver/gpio.h"

void button_init(gpio_num_t pin);
/* True once per press. The pin is active low with the internal pull-up. */
bool button_pressed(void);
