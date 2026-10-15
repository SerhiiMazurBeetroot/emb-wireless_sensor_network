#include "nrf24.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

static void csn_high(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

static void csn_low(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

static void ce_low(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

static void spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, len, 10);
}

static void log_msg(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 100);
}

void nrf24_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &gpio);

    csn_high();
    ce_low();
}

uint8_t nrf24_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(NRF24_SPI_CMD_R_REGISTER | (reg & 0x1F)), 0x00};
    uint8_t rx[2] = {0};

    ce_low();
    csn_low();
    spi_transfer(tx, rx, sizeof(tx));
    csn_high();

    return rx[1];
}

void nrf24_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(NRF24_SPI_CMD_W_REGISTER | (reg & 0x1F)), value};
    uint8_t rx[2] = {0};

    ce_low();
    csn_low();
    spi_transfer(tx, rx, sizeof(tx));
    csn_high();
}

bool nrf24_spi_probe(void)
{
    char buf[64];
    const uint8_t config_before = nrf24_read_reg(NRF24_REG_CONFIG);

    snprintf(buf, sizeof(buf), "CONFIG = 0x%02X\r\n", config_before);
    log_msg(buf);

    nrf24_write_reg(NRF24_REG_RF_CH, NRF24_RF_CHANNEL);
    const uint8_t channel = nrf24_read_reg(NRF24_REG_RF_CH);
    const bool ok = (channel == NRF24_RF_CHANNEL);

    if (ok) {
        snprintf(buf, sizeof(buf), "SPI OK: RF_CH write/read = %u\r\n", channel);
    } else {
        snprintf(buf, sizeof(buf), "SPI FAIL: expected RF_CH %u, got 0x%02X\r\n",
                 NRF24_RF_CHANNEL, channel);
    }
    log_msg(buf);

    return ok;
}
