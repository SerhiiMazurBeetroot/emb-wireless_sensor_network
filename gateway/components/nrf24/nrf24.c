#include "nrf24.h"

#include <stdint.h>
#include <string.h>

#include "config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

static const char *TAG = "nrf24";

static spi_device_handle_t s_spi;

/* CSN = 1: the radio is deselected and the SPI transaction is closed. */
static void csn_high(void) { gpio_set_level(NRF24_PIN_CSN, 1); }

/* CSN = 0: start an SPI transaction with the nRF24. */
static void csn_low(void) { gpio_set_level(NRF24_PIN_CSN, 0); }

/* CE = 0: reception is stopped and no transmit pulse is issued. */
static void ce_low(void) { gpio_set_level(NRF24_PIN_CE, 0); }

/* CE = 1: enables reception in RX mode; a short pulse starts a transmission in TX mode. */
static void ce_high(void) { gpio_set_level(NRF24_PIN_CE, 1); }

static const uint8_t s_addr[NRF24_ADDR_WIDTH] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};

/* Exchange len bytes over SPI: tx goes to the radio, rx collects the reply. */
static esp_err_t spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
	spi_transaction_t t = {
		.length = len * 8,
		.tx_buffer = tx,
		.rx_buffer = rx,
	};
	return spi_device_polling_transmit(s_spi, &t);
}

/* Configure CE and CSN as outputs and bring up the SPI2 bus. The radio stays off. */
esp_err_t nrf24_init(void) {
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

/* Read a register. The first reply byte is STATUS; the register value is the second byte. */
uint8_t nrf24_read_reg(uint8_t reg) {
	uint8_t tx[2] = {(uint8_t)(NRF24_SPI_CMD_R_REGISTER | (reg & 0x1F)), 0x00};
	uint8_t rx[2] = {0};

	csn_low();
	spi_transfer(tx, rx, sizeof(tx));
	csn_high();

	return rx[1];
}

/* Write one byte to a register with the W_REGISTER command. */
void nrf24_write_reg(uint8_t reg, uint8_t value) {
	uint8_t tx[2] = {(uint8_t)(NRF24_SPI_CMD_W_REGISTER | (reg & 0x1F)), value};
	uint8_t rx[2] = {0};

	csn_low();
	spi_transfer(tx, rx, sizeof(tx));
	csn_high();
}

/* Check that the radio is on the bus: write RF_CH and read the same value back. */
bool nrf24_spi_probe(void) {
	const uint8_t config_before = nrf24_read_reg(NRF24_REG_CONFIG);
	ESP_LOGI(TAG, "CONFIG = 0x%02X", config_before);

	nrf24_write_reg(NRF24_REG_RF_CH, NRF24_RF_CHANNEL);
	const uint8_t channel = nrf24_read_reg(NRF24_REG_RF_CH);

	const bool ok = (channel == NRF24_RF_CHANNEL);
	if (ok) {
		ESP_LOGI(TAG, "SPI OK: RF_CH write/read = %u", channel);
	} else {
		ESP_LOGE(TAG, "SPI FAIL: expected RF_CH %u, got 0x%02X",
				 NRF24_RF_CHANNEL, channel);
	}

	return ok;
}

/* Write several bytes to a register. Used to set the RX and TX address. */
static void write_buf(uint8_t reg, const uint8_t *data, uint8_t len) {
	uint8_t tx[1 + 32] = {0};
	uint8_t rx[1 + 32] = {0};

	tx[0] = (uint8_t)(NRF24_SPI_CMD_W_REGISTER | (reg & 0x1F));
	memcpy(&tx[1], data, len);

	csn_low();
	spi_transfer(tx, rx, (size_t)len + 1);
	csn_high();
}

/* Run a command with no data payload: FLUSH_TX or FLUSH_RX. */
static void command(uint8_t cmd) {
	uint8_t tx = cmd;
	uint8_t rx = 0;
	csn_low();
	spi_transfer(&tx, &rx, 1);
	csn_high();
}

/* Load a packet into the TX FIFO with the W_TX_PAYLOAD command. */
static void write_payload(const uint8_t *data, uint8_t len) {
	uint8_t tx[1 + 32] = {0};
	uint8_t rx[1 + 32] = {0};

	tx[0] = NRF24_CMD_W_TX_PAYLOAD;
	memcpy(&tx[1], data, len);

	csn_low();
	spi_transfer(tx, rx, (size_t)len + 1);
	csn_high();
}

/* Read a packet from the RX FIFO. The first reply byte is discarded; the payload follows. */
static void read_payload(uint8_t *data, uint8_t len) {
	uint8_t tx[1 + 32] = {0};
	uint8_t rx[1 + 32] = {0};

	tx[0] = NRF24_CMD_R_RX_PAYLOAD;

	csn_low();
	spi_transfer(tx, rx, (size_t)len + 1);
	csn_high();

	memcpy(data, &rx[1], len);
}

/* Disable auto-ack and set channel 76, 1 Mbps, 0 dBm, a 5-byte address, and 1-byte CRC. */
void nrf24_radio_init(void) {
	ce_low();
	nrf24_write_reg(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC);
	nrf24_write_reg(NRF24_REG_EN_AA, 0x00);
	nrf24_write_reg(NRF24_REG_EN_RXADDR, 0x01);
	nrf24_write_reg(NRF24_REG_SETUP_AW, 0x03);
	nrf24_write_reg(NRF24_REG_SETUP_RETR, 0x00);
	nrf24_write_reg(NRF24_REG_RF_CH, NRF24_RF_CHANNEL);
	nrf24_write_reg(NRF24_REG_RF_SETUP, 0x06);
	write_buf(NRF24_REG_RX_ADDR_P0, s_addr, NRF24_ADDR_WIDTH);
	write_buf(NRF24_REG_TX_ADDR, s_addr, NRF24_ADDR_WIDTH);
	nrf24_write_reg(NRF24_REG_RX_PW_P0, NRF24_PAYLOAD_LEN);
	command(NRF24_CMD_FLUSH_TX);
	command(NRF24_CMD_FLUSH_RX);
	nrf24_write_reg(NRF24_REG_STATUS, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS |
										  NRF24_STATUS_MAX_RT);
	nrf24_write_reg(NRF24_REG_CONFIG,
					NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP);
	vTaskDelay(pdMS_TO_TICKS(10));
}

/* Drop CE and clear PWR_UP. The radio enters power-down. */
void nrf24_power_down(void) {
	ce_low();
	nrf24_write_reg(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC);
}

/* Set PRIM_RX, wait for the standby exit, then raise CE to listen on the air. */
void nrf24_set_rx_mode(void) {
	ce_low();
	nrf24_write_reg(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC |
										  NRF24_CONFIG_PWR_UP |
										  NRF24_CONFIG_PRIM_RX);
	vTaskDelay(pdMS_TO_TICKS(10));
	ce_high();
}

/* Clear PRIM_RX and wait 2 ms. Transmission starts later with a separate CE pulse. */
void nrf24_set_tx_mode(void) {
	ce_low();
	nrf24_write_reg(NRF24_REG_CONFIG,
					NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP);
	vTaskDelay(pdMS_TO_TICKS(10));
}

/* Send 5 bytes and wait for TX_DS. False when the radio sets MAX_RT or the wait times out. */
bool nrf24_send(const uint8_t *data, uint8_t len) {
	if (len != NRF24_PAYLOAD_LEN) {
		return false;
	}

	nrf24_set_tx_mode();
	write_payload(data, len);
	ce_high();
	esp_rom_delay_us(20);
	ce_low();

	for (int i = 0; i < 100; i++) {
		const uint8_t status = nrf24_read_reg(NRF24_REG_STATUS);
		if (status & NRF24_STATUS_TX_DS) {
			nrf24_write_reg(NRF24_REG_STATUS, NRF24_STATUS_TX_DS);
			return true;
		}
		if (status & NRF24_STATUS_MAX_RT) {
			command(NRF24_CMD_FLUSH_TX);
			nrf24_write_reg(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT);
			return false;
		}
		esp_rom_delay_us(100);
	}

	return false;
}

/* Read one packet when RX_DR is set or the RX FIFO is not empty. */
bool nrf24_receive(uint8_t *data, uint8_t len) {
	if (len != NRF24_PAYLOAD_LEN) {
		return false;
	}

	const uint8_t status = nrf24_read_reg(NRF24_REG_STATUS);
	const uint8_t fifo = nrf24_read_reg(NRF24_REG_FIFO_STATUS);
	if ((status & NRF24_STATUS_RX_DR) == 0 &&
		(fifo & NRF24_FIFO_RX_EMPTY) != 0) {
		return false;
	}

	read_payload(data, len);
	nrf24_write_reg(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
	return true;
}
