#include "aht20.h"

#include "node.h"
#include "main.h"

#if NODE_USE_AHT20

#define AHT20_ADDR (0x38U << 1)

extern I2C_HandleTypeDef hi2c1;

static bool aht20_write(const uint8_t *data, uint16_t len)
{
	return HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, (uint8_t *)data, len, 100) == HAL_OK;
}

static bool aht20_read_bytes(uint8_t *data, uint16_t len)
{
	return HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR, data, len, 100) == HAL_OK;
}

static void bus_delay(void)
{
	for (volatile uint32_t i = 0; i < 50U; i++) {
		__NOP();
	}
}

/* A slave cut off mid-byte can hold SDA low and leave I2C1 stuck BUSY until reset. */
static void aht20_bus_recover(void)
{
	GPIO_InitTypeDef gpio = {0};

	HAL_I2C_DeInit(&hi2c1);

	__HAL_RCC_GPIOB_CLK_ENABLE();
	gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Mode = GPIO_MODE_OUTPUT_OD;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_Init(GPIOB, &gpio);
	bus_delay();

	for (int i = 0; i < 9; i++) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		bus_delay();
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		bus_delay();
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	bus_delay();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	bus_delay();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	bus_delay();

	HAL_I2C_Init(&hi2c1);
}

bool aht20_init(void)
{
	uint8_t cmd[] = {0xBE, 0x08, 0x00};

	aht20_bus_recover();
	HAL_Delay(40);
	if (!aht20_write(cmd, sizeof(cmd))) {
		return false;
	}
	HAL_Delay(10);
	return true;
}

bool aht20_read(int16_t *temp_c_x100, uint16_t *hum_pct_x100, uint16_t *press_hpa)
{
	uint8_t trig[] = {0xAC, 0x33, 0x00};
	uint8_t raw[7];
	int ready = 0;

	if (!aht20_write(trig, sizeof(trig))) {
		return false;
	}

	for (int i = 0; i < 15; i++) {
		HAL_Delay(10);
		if (!aht20_read_bytes(raw, sizeof(raw))) {
			continue;
		}
		if ((raw[0] & 0x80U) == 0U) {
			ready = 1;
			break;
		}
	}
	if (!ready) {
		return false;
	}

	uint32_t rh = ((uint32_t)raw[1] << 12) | ((uint32_t)raw[2] << 4) | (raw[3] >> 4);
	uint32_t t = (((uint32_t)raw[3] & 0x0FU) << 16) | ((uint32_t)raw[4] << 8) | raw[5];
	float hum = (float)rh * 100.0f / 1048576.0f;
	float tmp = (float)t * 200.0f / 1048576.0f - 50.0f;

	*temp_c_x100 = (int16_t)(tmp * 100.0f);
	*hum_pct_x100 = (uint16_t)(hum * 100.0f);
	*press_hpa = 0;
	return true;
}

#else

bool aht20_init(void)
{
	return false;
}

bool aht20_read(int16_t *temp_c_x100, uint16_t *hum_pct_x100, uint16_t *press_hpa)
{
	(void)temp_c_x100;
	(void)hum_pct_x100;
	(void)press_hpa;
	return false;
}

#endif
