#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* nRF24 register map (first byte of SPI command) */
#define NRF24_REG_CONFIG 0x00
#define NRF24_REG_RF_CH 0x05
#define NRF24_REG_STATUS 0x07

#define NRF24_SPI_CMD_R_REGISTER 0x00
#define NRF24_SPI_CMD_W_REGISTER 0x20

esp_err_t nrf24_init(void);

uint8_t nrf24_read_reg(uint8_t reg);
void nrf24_write_reg(uint8_t reg, uint8_t value);

/**
 * Step 1 check: read CONFIG, write/read RF_CH, log result.
 * @return true if RF_CH readback matches NRF24_RF_CHANNEL from config.h
 */
bool nrf24_spi_probe(void);
