#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define NRF24_REG_CONFIG 0x00
#define NRF24_REG_EN_AA 0x01
#define NRF24_REG_EN_RXADDR 0x02
#define NRF24_REG_SETUP_AW 0x03
#define NRF24_REG_SETUP_RETR 0x04
#define NRF24_REG_RF_CH 0x05
#define NRF24_REG_RF_SETUP 0x06
#define NRF24_REG_STATUS 0x07
#define NRF24_REG_RX_ADDR_P0 0x0A
#define NRF24_REG_TX_ADDR 0x10
#define NRF24_REG_RX_PW_P0 0x11
#define NRF24_REG_FIFO_STATUS 0x17

#define NRF24_SPI_CMD_R_REGISTER 0x00
#define NRF24_SPI_CMD_W_REGISTER 0x20
#define NRF24_CMD_R_RX_PAYLOAD 0x61
#define NRF24_CMD_W_TX_PAYLOAD 0xA0
#define NRF24_CMD_FLUSH_TX 0xE1
#define NRF24_CMD_FLUSH_RX 0xE2

#define NRF24_STATUS_RX_DR 0x40
#define NRF24_STATUS_TX_DS 0x20
#define NRF24_STATUS_MAX_RT 0x10
#define NRF24_FIFO_RX_EMPTY 0x01

#define NRF24_CONFIG_EN_CRC 0x08
#define NRF24_CONFIG_PWR_UP 0x02
#define NRF24_CONFIG_PRIM_RX 0x01

#define NRF24_PAYLOAD_LEN 12
#define NRF24_ADDR_WIDTH 5


/* Configure the CE/CSN pins and the SPI bus. The radio stays powered down. */
esp_err_t nrf24_init(void);

/* Read one register. Returns the register value, not the STATUS byte. */
uint8_t nrf24_read_reg(uint8_t reg);

/* Write one byte to a register. */
void nrf24_write_reg(uint8_t reg, uint8_t value);

/* Write the channel and read it back. True when the radio returns the same value. */
bool nrf24_spi_probe(void);

/* Program the shared radio registers and leave the chip in standby. */
void nrf24_radio_init(void);

/* Turn the transmitter off: CE low, PWR_UP cleared. */
void nrf24_power_down(void);

/* Enter receive mode and raise CE. */
void nrf24_set_rx_mode(void);

/* Enter transmit mode. CE stays low until a packet is sent. */
void nrf24_set_tx_mode(void);

/* Send exactly NRF24_PAYLOAD_LEN bytes. True when the radio sets TX_DS. */
bool nrf24_send(const uint8_t *data, uint8_t len);

/* Copy one packet into data when the RX FIFO has one. Otherwise false. */
bool nrf24_receive(uint8_t *data, uint8_t len);

/*
 * IRQ-driven RX: create a queue of NRF24_PAYLOAD_LEN bytes, then call after
 * nrf24_radio_init(). A background task drains the FIFO and xQueueSend()s packets.
 */
esp_err_t nrf24_rx_start(QueueHandle_t packet_queue);
