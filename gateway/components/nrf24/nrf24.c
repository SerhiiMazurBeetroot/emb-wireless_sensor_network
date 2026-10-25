#include "nrf24.h"

#include <stdint.h>
#include <string.h>

#include "config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "nrf24";

static spi_device_handle_t s_spi;
static SemaphoreHandle_t s_spi_mtx;
static TaskHandle_t s_rx_task;
static QueueHandle_t s_rx_queue;

static void csn_high(void) { gpio_set_level(NRF24_PIN_CSN, 1); }
static void csn_low(void) { gpio_set_level(NRF24_PIN_CSN, 0); }
static void ce_low(void) { gpio_set_level(NRF24_PIN_CE, 0); }
static void ce_high(void) { gpio_set_level(NRF24_PIN_CE, 1); }

static const uint8_t s_addr[NRF24_ADDR_WIDTH] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};

static esp_err_t spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
	spi_transaction_t t = {
		.length = len * 8,
		.tx_buffer = tx,
		.rx_buffer = rx,
	};
	return spi_device_polling_transmit(s_spi, &t);
}

/* --- SPI без mutex: лише всередині вже заблокованих публічних API --- */

static uint8_t read_reg_nolock(uint8_t reg) {
	uint8_t tx[2] = {(uint8_t)(NRF24_SPI_CMD_R_REGISTER | (reg & 0x1F)), 0x00};
	uint8_t rx[2] = {0};

	csn_low();
	spi_transfer(tx, rx, sizeof(tx));
	csn_high();

	return rx[1];
}

static void write_reg_nolock(uint8_t reg, uint8_t value) {
	uint8_t tx[2] = {(uint8_t)(NRF24_SPI_CMD_W_REGISTER | (reg & 0x1F)), value};
	uint8_t rx[2] = {0};

	csn_low();
	spi_transfer(tx, rx, sizeof(tx));
	csn_high();
}

static void write_buf_nolock(uint8_t reg, const uint8_t *data, uint8_t len) {
	uint8_t tx[1 + 32] = {0};
	uint8_t rx[1 + 32] = {0};

	tx[0] = (uint8_t)(NRF24_SPI_CMD_W_REGISTER | (reg & 0x1F));
	memcpy(&tx[1], data, len);

	csn_low();
	spi_transfer(tx, rx, (size_t)len + 1);
	csn_high();
}

static void command_nolock(uint8_t cmd) {
	uint8_t tx = cmd;
	uint8_t rx = 0;

	csn_low();
	spi_transfer(&tx, &rx, 1);
	csn_high();
}

static void write_payload_nolock(const uint8_t *data, uint8_t len) {
	uint8_t tx[1 + 32] = {0};
	uint8_t rx[1 + 32] = {0};

	tx[0] = NRF24_CMD_W_TX_PAYLOAD;
	memcpy(&tx[1], data, len);

	csn_low();
	spi_transfer(tx, rx, (size_t)len + 1);
	csn_high();
}

static void read_payload_nolock(uint8_t *data, uint8_t len) {
	uint8_t tx[1 + 32] = {0};
	uint8_t rx[1 + 32] = {0};

	tx[0] = NRF24_CMD_R_RX_PAYLOAD;

	csn_low();
	spi_transfer(tx, rx, (size_t)len + 1);
	csn_high();

	memcpy(data, &rx[1], len);
}

static void set_rx_mode_nolock(void) {
	ce_low();
	write_reg_nolock(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC |
										 NRF24_CONFIG_PWR_UP |
										 NRF24_CONFIG_PRIM_RX);
	esp_rom_delay_us(2000);
	ce_high();
}

static void set_tx_mode_nolock(void) {
	ce_low();
	write_reg_nolock(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP);
	esp_rom_delay_us(2000);
}

static bool receive_nolock(uint8_t *data, uint8_t len) {
	const uint8_t status = read_reg_nolock(NRF24_REG_STATUS);
	const uint8_t fifo = read_reg_nolock(NRF24_REG_FIFO_STATUS);

	if ((status & NRF24_STATUS_RX_DR) == 0 && (fifo & NRF24_FIFO_RX_EMPTY) != 0) {
		return false;
	}

	read_payload_nolock(data, len);
	write_reg_nolock(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
	return true;
}

static void clear_tx_flags_nolock(void) {
	const uint8_t st = read_reg_nolock(NRF24_REG_STATUS);

	if (st & NRF24_STATUS_MAX_RT) {
		command_nolock(NRF24_CMD_FLUSH_TX);
	}
	if (st & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) {
		write_reg_nolock(NRF24_REG_STATUS, st & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));
	}
}

static void IRAM_ATTR nrf24_irq_isr(void *arg) {
	BaseType_t wake = pdFALSE;

	if (s_rx_task != NULL) {
		vTaskNotifyGiveFromISR(s_rx_task, &wake);
	}
	if (wake == pdTRUE) {
		portYIELD_FROM_ISR();
	}
}

static void nrf24_rx_task(void *arg) {
	QueueHandle_t q = (QueueHandle_t)arg;
	uint8_t pkt[NRF24_PAYLOAD_LEN];

	for (;;) {
		(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		while (receive_nolock(pkt, NRF24_PAYLOAD_LEN)) {
			if (xQueueSend(q, pkt, 0) != pdTRUE) {
				ESP_LOGW(TAG, "RX queue full, packet dropped");
			}
		}
		clear_tx_flags_nolock();

		xSemaphoreGive(s_spi_mtx);
	}
}

esp_err_t nrf24_init(void) {
	if (s_spi_mtx == NULL) {
		s_spi_mtx = xSemaphoreCreateMutex();
		if (s_spi_mtx == NULL) {
			return ESP_ERR_NO_MEM;
		}
	}

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

uint8_t nrf24_read_reg(uint8_t reg) {
	uint8_t v;

	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return 0;
	}
	v = read_reg_nolock(reg);
	xSemaphoreGive(s_spi_mtx);
	return v;
}

void nrf24_write_reg(uint8_t reg, uint8_t value) {
	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return;
	}
	write_reg_nolock(reg, value);
	xSemaphoreGive(s_spi_mtx);
}

bool nrf24_spi_probe(void) {
	bool ok;

	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return false;
	}

	const uint8_t config_before = read_reg_nolock(NRF24_REG_CONFIG);
	ESP_LOGI(TAG, "CONFIG = 0x%02X", config_before);

	write_reg_nolock(NRF24_REG_RF_CH, NRF24_RF_CHANNEL);
	const uint8_t channel = read_reg_nolock(NRF24_REG_RF_CH);

	ok = (channel == NRF24_RF_CHANNEL);
	if (ok) {
		ESP_LOGI(TAG, "SPI OK: RF_CH write/read = %u", channel);
	} else {
		ESP_LOGE(TAG, "SPI FAIL: expected RF_CH %u, got 0x%02X",
				 NRF24_RF_CHANNEL, channel);
	}

	xSemaphoreGive(s_spi_mtx);
	return ok;
}

void nrf24_radio_init(void) {
	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return;
	}

	ce_low();
	write_reg_nolock(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC);
	write_reg_nolock(NRF24_REG_EN_AA, 0x00);
	write_reg_nolock(NRF24_REG_EN_RXADDR, 0x01);
	write_reg_nolock(NRF24_REG_SETUP_AW, 0x03);
	write_reg_nolock(NRF24_REG_SETUP_RETR, 0x00);
	write_reg_nolock(NRF24_REG_RF_CH, NRF24_RF_CHANNEL);
	write_reg_nolock(NRF24_REG_RF_SETUP, 0x06);
	write_buf_nolock(NRF24_REG_RX_ADDR_P0, s_addr, NRF24_ADDR_WIDTH);
	write_buf_nolock(NRF24_REG_TX_ADDR, s_addr, NRF24_ADDR_WIDTH);
	write_reg_nolock(NRF24_REG_RX_PW_P0, NRF24_PAYLOAD_LEN);
	command_nolock(NRF24_CMD_FLUSH_TX);
	command_nolock(NRF24_CMD_FLUSH_RX);
	write_reg_nolock(NRF24_REG_STATUS, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS |
										 NRF24_STATUS_MAX_RT);
	write_reg_nolock(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP);
	esp_rom_delay_us(2000);

	xSemaphoreGive(s_spi_mtx);
}

void nrf24_power_down(void) {
	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return;
	}
	ce_low();
	write_reg_nolock(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC);
	xSemaphoreGive(s_spi_mtx);
}

void nrf24_set_rx_mode(void) {
	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return;
	}
	set_rx_mode_nolock();
	xSemaphoreGive(s_spi_mtx);
}

void nrf24_set_tx_mode(void) {
	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return;
	}
	set_tx_mode_nolock();
	xSemaphoreGive(s_spi_mtx);
}

bool nrf24_send(const uint8_t *data, uint8_t len) {
	bool ok = false;

	if (len != NRF24_PAYLOAD_LEN) {
		return false;
	}
	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return false;
	}

	set_tx_mode_nolock();
	write_payload_nolock(data, len);
	ce_high();
	esp_rom_delay_us(20);
	ce_low();

	for (int i = 0; i < 100; i++) {
		const uint8_t status = read_reg_nolock(NRF24_REG_STATUS);
		if (status & NRF24_STATUS_TX_DS) {
			write_reg_nolock(NRF24_REG_STATUS, NRF24_STATUS_TX_DS);
			ok = true;
			break;
		}
		if (status & NRF24_STATUS_MAX_RT) {
			command_nolock(NRF24_CMD_FLUSH_TX);
			write_reg_nolock(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT);
			break;
		}
		esp_rom_delay_us(100);
	}

	xSemaphoreGive(s_spi_mtx);
	return ok;
}

bool nrf24_receive(uint8_t *data, uint8_t len) {
	bool ok;

	if (len != NRF24_PAYLOAD_LEN) {
		return false;
	}
	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return false;
	}
	ok = receive_nolock(data, len);
	xSemaphoreGive(s_spi_mtx);
	return ok;
}

esp_err_t nrf24_rx_start(QueueHandle_t packet_queue) {
	static bool started;

	if (packet_queue == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	if (started) {
		return ESP_OK;
	}
	if (s_spi_mtx == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	s_rx_queue = packet_queue;

	if (xTaskCreate(nrf24_rx_task, "nrf24_rx", 3072, packet_queue, 6, &s_rx_task) != pdPASS) {
		return ESP_ERR_NO_MEM;
	}

	gpio_config_t irq = {
		.pin_bit_mask = 1ULL << NRF24_PIN_IRQ,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_NEGEDGE,
	};
	esp_err_t err = gpio_config(&irq);
	if (err != ESP_OK) {
		return err;
	}

	err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		return err;
	}

	err = gpio_isr_handler_add(NRF24_PIN_IRQ, nrf24_irq_isr, NULL);
	if (err != ESP_OK) {
		return err;
	}

	started = true;
	ESP_LOGI(TAG, "IRQ RX on GPIO %d", (int)NRF24_PIN_IRQ);
	return ESP_OK;
}