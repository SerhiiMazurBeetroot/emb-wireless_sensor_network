#include "nrf24.h"

#include <string.h>

#include "config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "nrf24";

static spi_device_handle_t s_spi;

static void csn_high(void) { gpio_set_level(NRF24_PIN_CSN, 1); }

static void csn_low(void) { gpio_set_level(NRF24_PIN_CSN, 0); }

static void ce_low(void) { gpio_set_level(NRF24_PIN_CE, 0); }

static esp_err_t spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(s_spi, &t);
}

esp_err_t nrf24_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << NRF24_PIN_CSN) | (1ULL << NRF24_PIN_CE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }

    csn_high();
    ce_low();

    spi_bus_config_t bus = {
        .mosi_io_num = NRF24_PIN_MOSI,
        .miso_io_num = NRF24_PIN_MISO,
        .sclk_io_num = NRF24_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    err = spi_bus_initialize(NRF24_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    spi_device_interface_config_t dev = {
        .clock_speed_hz = NRF24_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };

    return spi_bus_add_device(NRF24_SPI_HOST, &dev, &s_spi);
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
    const uint8_t config_before = nrf24_read_reg(NRF24_REG_CONFIG);
    ESP_LOGI(TAG, "CONFIG = 0x%02X", config_before);

    nrf24_write_reg(NRF24_REG_RF_CH, NRF24_RF_CHANNEL);
    const uint8_t channel = nrf24_read_reg(NRF24_REG_RF_CH);

    const bool ok = (channel == NRF24_RF_CHANNEL);
    if (ok) {
        ESP_LOGI(TAG, "SPI OK: RF_CH write/read = %u", channel);
    } else {
        ESP_LOGE(TAG, "SPI FAIL: expected RF_CH %u, got 0x%02X", NRF24_RF_CHANNEL, channel);
    }

    return ok;
}
