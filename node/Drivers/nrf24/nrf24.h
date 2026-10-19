#pragma once

#include <stdbool.h>
#include <stdint.h>

#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_EN_AA       0x01
#define NRF24_REG_EN_RXADDR   0x02
#define NRF24_REG_SETUP_AW    0x03
#define NRF24_REG_SETUP_RETR  0x04
#define NRF24_REG_RF_CH       0x05
#define NRF24_REG_RF_SETUP    0x06
#define NRF24_REG_STATUS      0x07
#define NRF24_REG_RX_ADDR_P0  0x0A
#define NRF24_REG_TX_ADDR     0x10
#define NRF24_REG_RX_PW_P0    0x11
#define NRF24_REG_FIFO_STATUS 0x17

#define NRF24_SPI_CMD_R_REGISTER 0x00
#define NRF24_SPI_CMD_W_REGISTER 0x20
#define NRF24_CMD_R_RX_PAYLOAD   0x61
#define NRF24_CMD_W_TX_PAYLOAD   0xA0
#define NRF24_CMD_FLUSH_TX       0xE1
#define NRF24_CMD_FLUSH_RX       0xE2

#define NRF24_STATUS_RX_DR  0x40
#define NRF24_STATUS_TX_DS  0x20
#define NRF24_STATUS_MAX_RT 0x10
#define NRF24_FIFO_RX_EMPTY 0x01

#define NRF24_CONFIG_EN_CRC  0x08
#define NRF24_CONFIG_PWR_UP  0x02
#define NRF24_CONFIG_PRIM_RX 0x01
#define NRF24_PAYLOAD_LEN 8
#define NRF24_ADDR_WIDTH  5

#define NRF24_RF_CHANNEL 76

void nrf24_init(void);

uint8_t nrf24_read_reg(uint8_t reg);
void nrf24_write_reg(uint8_t reg, uint8_t value);

bool nrf24_spi_probe(void);

void nrf24_radio_init(void);
void nrf24_power_down(void);
void nrf24_set_rx_mode(void);
void nrf24_set_tx_mode(void);
bool nrf24_send(const uint8_t *data, uint8_t len);
bool nrf24_receive(uint8_t *data, uint8_t len);
