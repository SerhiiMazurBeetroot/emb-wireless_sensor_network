#include "bme280.h"

#include "main.h"

#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

#define BME_REG_CALIB 0x88
#define BME_REG_ID 0xD0
#define BME_REG_CALIB_H 0xE1
#define BME_REG_CTRL_HUM 0xF2
#define BME_REG_STATUS 0xF3
#define BME_REG_CTRL_MEAS 0xF4
#define BME_REG_DATA 0xF7
#define BME_ID 0x60

struct bme280_calib {
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	uint8_t dig_h1;
	int16_t dig_h2;
	uint8_t dig_h3;
	int16_t dig_h4;
	int16_t dig_h5;
	int8_t dig_h6;
	int32_t t_fine;
};

static struct bme280_calib s_cal;

static void log_msg(const char *text) {
	HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

static void cs_high(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

static void cs_low(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

static void spi_write(uint8_t reg, uint8_t value) {
	uint8_t tx[2] = {(uint8_t)(reg & 0x7F), value};

	cs_low();
	HAL_SPI_Transmit(&hspi1, tx, 2, 10);
	cs_high();
}

static void spi_read(uint8_t reg, uint8_t *buf, uint16_t len) {
	uint8_t addr = (uint8_t)(reg | 0x80);

	cs_low();
	HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
	HAL_SPI_Receive(&hspi1, buf, len, 10);
	cs_high();
}

static uint16_t get_u16(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t get_i16(const uint8_t *p) {
	return (int16_t)get_u16(p);
}

/* Bosch integer compensation. Temperature comes back in 0.01 C. */
static int32_t compensate_temp(int32_t adc_t) {
	int32_t var1;
	int32_t var2;

	var1 = ((((adc_t >> 3) - ((int32_t)s_cal.dig_t1 << 1))) * ((int32_t)s_cal.dig_t2)) >> 11;
	var2 = (((((adc_t >> 4) - ((int32_t)s_cal.dig_t1)) * ((adc_t >> 4) - ((int32_t)s_cal.dig_t1))) >> 12) *
			((int32_t)s_cal.dig_t3)) >> 14;

	s_cal.t_fine = var1 + var2;
	return (s_cal.t_fine * 5 + 128) >> 8;
}

/* Pressure comes back in Pa. */
static uint32_t compensate_press(int32_t adc_p) {
	int32_t var1;
	int32_t var2;
	uint32_t p;

	var1 = (s_cal.t_fine >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)s_cal.dig_p6);
	var2 = var2 + ((var1 * ((int32_t)s_cal.dig_p5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)s_cal.dig_p4) << 16);
	var1 = (((s_cal.dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
			((((int32_t)s_cal.dig_p2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)s_cal.dig_p1)) >> 15);
	if (var1 == 0) {
		return 0;
	}
	p = (uint32_t)(((int32_t)1048576 - adc_p) - (var2 >> 12)) * 3125U;
	if (p < 0x80000000U) {
		p = (p << 1) / (uint32_t)var1;
	} else {
		p = (p / (uint32_t)var1) * 2U;
	}
	var1 = (((int32_t)s_cal.dig_p9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(p >> 2)) * ((int32_t)s_cal.dig_p8)) >> 13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + s_cal.dig_p7) >> 4));
	return p;
}

/* Humidity comes back in %RH * 1024. */
static uint32_t compensate_hum(int32_t adc_h) {
	int32_t v;

	v = s_cal.t_fine - (int32_t)76800;
	v = (((((adc_h << 14) - (((int32_t)s_cal.dig_h4) << 20) - (((int32_t)s_cal.dig_h5) * v)) +
		   (int32_t)16384) >> 15) *
		 (((((((v * ((int32_t)s_cal.dig_h6)) >> 10) *
			 (((v * ((int32_t)s_cal.dig_h3)) >> 11) + (int32_t)32768)) >> 10) +
			(int32_t)2097152) *
			   ((int32_t)s_cal.dig_h2) +
		   8192) >> 14));
	v = v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)s_cal.dig_h1)) >> 4);
	if (v < 0) {
		v = 0;
	}
	if (v > 419430400) {
		v = 419430400;
	}
	return (uint32_t)(v >> 12);
}

static void read_calibration(void) {
	uint8_t calib[26];
	uint8_t hum[7];

	spi_read(BME_REG_CALIB, calib, sizeof(calib));
	spi_read(BME_REG_CALIB_H, hum, sizeof(hum));

	s_cal.dig_t1 = get_u16(&calib[0]);
	s_cal.dig_t2 = get_i16(&calib[2]);
	s_cal.dig_t3 = get_i16(&calib[4]);
	s_cal.dig_p1 = get_u16(&calib[6]);
	s_cal.dig_p2 = get_i16(&calib[8]);
	s_cal.dig_p3 = get_i16(&calib[10]);
	s_cal.dig_p4 = get_i16(&calib[12]);
	s_cal.dig_p5 = get_i16(&calib[14]);
	s_cal.dig_p6 = get_i16(&calib[16]);
	s_cal.dig_p7 = get_i16(&calib[18]);
	s_cal.dig_p8 = get_i16(&calib[20]);
	s_cal.dig_p9 = get_i16(&calib[22]);
	s_cal.dig_h1 = calib[25];
	s_cal.dig_h2 = get_i16(&hum[0]);
	s_cal.dig_h3 = hum[2];
	s_cal.dig_h4 = (int16_t)((((int16_t)(int8_t)hum[3]) << 4) | (hum[4] & 0x0F));
	s_cal.dig_h5 = (int16_t)((((int16_t)(int8_t)hum[5]) << 4) | (hum[4] >> 4));
	s_cal.dig_h6 = (int8_t)hum[6];
}

bool bme280_init(void) {
	GPIO_InitTypeDef gpio = {0};
	uint8_t id = 0;
	char buf[40];

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    cs_high();

    spi_read(BME_REG_ID, &id, 1);
    snprintf(buf, sizeof(buf), "node: BME280 id=0x%02X\r\n", id);
    log_msg(buf);
    if (id != BME_ID) {
		return false;
	}

    read_calibration();
    return true;
}

bool bme280_read(int16_t *temp_c_x100, uint16_t *hum_pct_x100, uint16_t *press_hpa) {
	uint8_t raw[8];
	int32_t adc_p;
	int32_t adc_t;
	int32_t adc_h;
	int32_t temp;
	uint32_t press;
	uint32_t hum;

    /* Humidity oversampling is applied only after the following ctrl_meas write. */
	spi_write(BME_REG_CTRL_HUM, 0x01);
	spi_write(BME_REG_CTRL_MEAS, 0x25);

	for(int i = 0; i < 20; i++) {
		uint8_t status = 0;

		spi_read(BME_REG_STATUS, &status, 1);
		if ((status & 0x08) == 0) {
			break;
		}
		HAL_Delay(1);
	}

	spi_read(BME_REG_DATA, raw, sizeof(raw));
	adc_p = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | ((uint32_t)raw[2] >> 4));
	adc_t = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | ((uint32_t)raw[5] >> 4));
	adc_h = (int32_t)(((uint32_t)raw[6] << 8) | raw[7]);
	if (adc_t == 0x80000 || adc_p == 0x80000 || adc_h == 0x8000) {
		return false;
	}

	temp = compensate_temp(adc_t);
	press = compensate_press(adc_p);
	hum = compensate_hum(adc_h);

	*temp_c_x100 = (int16_t)temp;
	*hum_pct_x100 = (uint16_t)((hum * 100U) / 1024U);
	*press_hpa = (uint16_t)(press / 100U);
	return true;
}
