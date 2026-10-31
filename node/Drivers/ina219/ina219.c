#include "ina219.h"

#include "main.h"

#define INA219_ADDR (0x40U << 1)
#define INA219_REG_CONFIG 0x00U
#define INA219_REG_BUS 0x02U
#define INA219_REG_CURRENT 0x04U
#define INA219_REG_CAL 0x05U
/* 32 V / 1 A range, 0.1 ohm shunt: one current LSB is 40 µA. */
#define INA219_CAL 10240U
#define INA219_CONFIG 0x399FU

extern I2C_HandleTypeDef hi2c1;

static bool s_ready;

static bool ina219_write(uint8_t reg, uint16_t value)
{
	uint8_t bytes[3] = {reg, (uint8_t)(value >> 8), (uint8_t)value};

	if (HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, bytes, 3, 20) != HAL_OK) {
		(void)HAL_I2C_Init(&hi2c1);
		return false;
	}
	return true;
}

static bool ina219_read_reg(uint8_t reg, uint16_t *out)
{
	uint8_t bytes[2];

	if (HAL_I2C_Mem_Read(&hi2c1, INA219_ADDR, reg, I2C_MEMADD_SIZE_8BIT, bytes, 2, 20) != HAL_OK) {
		(void)HAL_I2C_Init(&hi2c1);
		return false;
	}
	*out = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
	return true;
}

bool ina219_read(int32_t *current_ua, uint16_t *vbus_mv)
{
	uint16_t current_raw = 0;
	uint16_t bus_raw = 0;

	if (!s_ready) {
		if (!ina219_write(INA219_REG_CAL, INA219_CAL) || !ina219_write(INA219_REG_CONFIG, INA219_CONFIG)) {
			*current_ua = 0;
			*vbus_mv = 0;
			return false;
		}
		s_ready = true;
		/* First conversion is about 1 ms. */
		HAL_Delay(2);
	}

	if (!ina219_read_reg(INA219_REG_CURRENT, &current_raw) || !ina219_read_reg(INA219_REG_BUS, &bus_raw)) {
		s_ready = false;
		*current_ua = 0;
		*vbus_mv = 0;
		return false;
	}

	*current_ua = (int32_t)(int16_t)current_raw * 40;
	*vbus_mv = (uint16_t)((bus_raw >> 3) * 4U);
	return true;
}
