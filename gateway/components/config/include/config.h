#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

/* nRF24 on SPI2 — adjust here if you wire a different DevKit */

#define NRF24_SPI_HOST SPI2_HOST

#define NRF24_PIN_SCK GPIO_NUM_12
#define NRF24_PIN_MOSI GPIO_NUM_11
#define NRF24_PIN_MISO GPIO_NUM_13
#define NRF24_PIN_CSN GPIO_NUM_10
#define NRF24_PIN_CE GPIO_NUM_14

/* Start at 1 MHz for bench wiring; raise after SPI probe passes */
#define NRF24_SPI_CLOCK_HZ (1 * 1000 * 1000)

/* Shared with node later (RF_CH register value) */
#define NRF24_RF_CHANNEL 76
