#include "button.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "button";

static gpio_num_t s_pin = GPIO_NUM_NC;
static int s_level = 1;
static int64_t s_change_us;

void button_init(gpio_num_t pin)
{
	gpio_config_t cfg = {
		.pin_bit_mask = 1ULL << pin,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	ESP_ERROR_CHECK(gpio_config(&cfg));
	s_pin = pin;
	s_level = gpio_get_level(pin);
	s_change_us = 0;
	ESP_LOGI(TAG, "GPIO %d, press to GND", (int)pin);
}

bool button_pressed(void)
{
	int level;
	int64_t now;

	if (s_pin == GPIO_NUM_NC) {
		return false;
	}

	level = gpio_get_level(s_pin);
	now = esp_timer_get_time();
	if (level == s_level) {
		s_change_us = 0;
		return false;
	}
	if (s_change_us == 0) {
		s_change_us = now;
		return false;
	}
	if ((now - s_change_us) < 40000) {
		return false;
	}

	s_level = level;
	s_change_us = 0;
	return level == 0;
}
