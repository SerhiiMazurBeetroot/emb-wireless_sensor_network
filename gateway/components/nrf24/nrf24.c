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

/*
 * CONFIG bits that every mode keeps set: 1-byte CRC on, and the TX_DS / MAX_RT
 * interrupts masked. The IRQ pin then only goes low for RX_DR, so the gateway's
 * own ACK transmissions no longer wake the receive task.
 */
#define NRF24_CONFIG_BASE \
	(NRF24_CONFIG_EN_CRC | NRF24_CONFIG_MASK_TX_DS | NRF24_CONFIG_MASK_MAX_RT)

/* Power-down -> standby takes up to 1.5 ms (datasheet Tpd2stby). */
#define NRF24_POWER_UP_US 2000
/* Standby -> RX or TX settling is 130 us when the chip is already powered up. */
#define NRF24_SETTLE_US 150
/*
 * The receive task also wakes on its own this often and checks the FIFO. If the
 * falling edge on the IRQ pin is ever missed, reception resumes after at most
 * this delay instead of stopping for good.
 */
#define NRF24_RX_POLL_MS 100

static spi_device_handle_t s_spi;
static SemaphoreHandle_t s_spi_mtx;
static TaskHandle_t s_rx_task;
static QueueHandle_t s_rx_queue;

/* CSN high deselects the radio and ends the SPI transaction; low starts one. */
static void csn_high(void) { gpio_set_level(NRF24_PIN_CSN, 1); }
static void csn_low(void) { gpio_set_level(NRF24_PIN_CSN, 0); }
/* CE low stops listening; CE high listens in RX mode or fires a packet in TX mode. */
static void ce_low(void) { gpio_set_level(NRF24_PIN_CE, 0); }
static void ce_high(void) { gpio_set_level(NRF24_PIN_CE, 1); }

/* One address for the whole star: nodes and gateway all send and listen on it. */
static const uint8_t s_addr[NRF24_ADDR_WIDTH] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};

/* Clock len bytes out of tx while collecting the radio's reply into rx. */
static esp_err_t spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
	spi_transaction_t t = {
		.length = len * 8,
		.tx_buffer = tx,
		.rx_buffer = rx,
	};
	return spi_device_polling_transmit(s_spi, &t);
}

/*
 * The *_nolock helpers talk to the radio without taking s_spi_mtx. Only call
 * them from a function that already holds the mutex; the public API below does.
 */

/* Read a register. The first reply byte is STATUS, the register value comes second. */
static uint8_t read_reg_nolock(uint8_t reg) {
	uint8_t tx[2] = {(uint8_t)(NRF24_SPI_CMD_R_REGISTER | (reg & 0x1F)), 0x00};
	uint8_t rx[2] = {0};

	csn_low();
	spi_transfer(tx, rx, sizeof(tx));
	csn_high();

	return rx[1];
}

/* Write one byte to a register. */
static void write_reg_nolock(uint8_t reg, uint8_t value) {
	uint8_t tx[2] = {(uint8_t)(NRF24_SPI_CMD_W_REGISTER | (reg & 0x1F)), value};
	uint8_t rx[2] = {0};

	csn_low();
	spi_transfer(tx, rx, sizeof(tx));
	csn_high();
}

/* Write several bytes to a register, e.g. the 5-byte RX/TX address. */
static void write_buf_nolock(uint8_t reg, const uint8_t *data, uint8_t len) {
	uint8_t tx[1 + NRF24_MAX_PAYLOAD] = {0};
	uint8_t rx[1 + NRF24_MAX_PAYLOAD] = {0};

	if (len > NRF24_MAX_PAYLOAD) {
		return;
	}

	tx[0] = (uint8_t)(NRF24_SPI_CMD_W_REGISTER | (reg & 0x1F));
	memcpy(&tx[1], data, len);

	csn_low();
	spi_transfer(tx, rx, (size_t)len + 1);
	csn_high();
}

/* Send a command that has no data bytes, such as FLUSH_TX or FLUSH_RX. */
static void command_nolock(uint8_t cmd) {
	uint8_t tx = cmd;
	uint8_t rx = 0;

	csn_low();
	spi_transfer(&tx, &rx, 1);
	csn_high();
}

/* Load one packet into the TX FIFO. It goes on air at the next CE pulse. */
static void write_payload_nolock(const uint8_t *data, uint8_t len) {
	uint8_t tx[1 + NRF24_MAX_PAYLOAD] = {0};
	uint8_t rx[1 + NRF24_MAX_PAYLOAD] = {0};

	if (len > NRF24_MAX_PAYLOAD) {
		return;
	}

	tx[0] = NRF24_CMD_W_TX_PAYLOAD;
	memcpy(&tx[1], data, len);

	csn_low();
	spi_transfer(tx, rx, (size_t)len + 1);
	csn_high();
}

/* Pop the oldest packet from the RX FIFO. The first reply byte is STATUS and is dropped. */
static void read_payload_nolock(uint8_t *data, uint8_t len) {
	uint8_t tx[1 + NRF24_MAX_PAYLOAD] = {0};
	uint8_t rx[1 + NRF24_MAX_PAYLOAD] = {0};

	if (len > NRF24_MAX_PAYLOAD) {
		return;
	}

	tx[0] = NRF24_CMD_R_RX_PAYLOAD;

	csn_low();
	spi_transfer(tx, rx, (size_t)len + 1);
	csn_high();

	memcpy(data, &rx[1], len);
}

/*
 * Write a new CONFIG value and wait until the radio can use it. Switching
 * between RX and TX while powered up needs only the short settling time; the
 * long wait is needed only when PWR_UP was off, i.e. on the first power-up.
 */
static void write_config_nolock(uint8_t config) {
	const bool was_powered = (read_reg_nolock(NRF24_REG_CONFIG) & NRF24_CONFIG_PWR_UP) != 0;

	write_reg_nolock(NRF24_REG_CONFIG, config);
	esp_rom_delay_us(was_powered ? NRF24_SETTLE_US : NRF24_POWER_UP_US);
}

/* Switch to receive and raise CE, so the radio starts listening on the channel. */
static void set_rx_mode_nolock(void) {
	ce_low();
	write_config_nolock(NRF24_CONFIG_BASE | NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX);
	ce_high();
}

/* Switch to transmit with CE low. Nothing is sent until nrf24_send() pulses CE. */
static void set_tx_mode_nolock(void) {
	ce_low();
	write_config_nolock(NRF24_CONFIG_BASE | NRF24_CONFIG_PWR_UP);
}

/*
 * Read one packet if the radio has one. RX_DR alone is not enough: the flag is
 * cleared once per read, while the FIFO can still hold up to two more packets.
 */
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

/* Clear leftover TX flags so they never hold the IRQ line low. */
static void clear_tx_flags_nolock(void) {
	const uint8_t st = read_reg_nolock(NRF24_REG_STATUS);

	if (st & NRF24_STATUS_MAX_RT) {
		command_nolock(NRF24_CMD_FLUSH_TX);
	}
	if (st & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) {
		write_reg_nolock(NRF24_REG_STATUS, st & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));
	}
}

/* IRQ pin went low: a packet arrived. Only wake the receive task; SPI is not ISR-safe. */
static void IRAM_ATTR nrf24_irq_isr(void *arg) {
	BaseType_t wake = pdFALSE;

	if (s_rx_task != NULL) {
		vTaskNotifyGiveFromISR(s_rx_task, &wake);
	}
	if (wake == pdTRUE) {
		portYIELD_FROM_ISR();
	}
}

/*
 * Receive task: sleeps until the IRQ fires or NRF24_RX_POLL_MS passes, then
 * empties the RX FIFO into the packet queue. Waking without an IRQ costs one
 * short SPI check and finds the FIFO empty in the normal case.
 */
static void nrf24_rx_task(void *arg) {
	QueueHandle_t q = (QueueHandle_t)arg;
	uint8_t pkt[NRF24_PAYLOAD_LEN];

	for (;;) {
		(void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(NRF24_RX_POLL_MS));

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
		.max_transfer_sz = 1 + NRF24_MAX_PAYLOAD,
	};

	err = spi_bus_initialize(NRF24_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		return err;
	}

	/* CSN is driven by hand (spics_io_num = -1) so multi-step commands stay in one frame. */
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

/*
 * Same settings as the nodes: channel 76, 1 Mbps, 0 dBm, 5-byte address,
 * fixed 12-byte payload on pipe 0. Hardware auto-ACK and retransmit are off;
 * delivery is confirmed by the application-level ACK frame instead.
 */
void nrf24_radio_init(void) {
	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return;
	}

	ce_low();
	write_reg_nolock(NRF24_REG_CONFIG, NRF24_CONFIG_BASE);
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
	write_config_nolock(NRF24_CONFIG_BASE | NRF24_CONFIG_PWR_UP);

	xSemaphoreGive(s_spi_mtx);
}

void nrf24_power_down(void) {
	if (xSemaphoreTake(s_spi_mtx, portMAX_DELAY) != pdTRUE) {
		return;
	}
	ce_low();
	write_reg_nolock(NRF24_REG_CONFIG, NRF24_CONFIG_BASE);
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

/*
 * Send one frame: switch to TX, load the FIFO, pulse CE for at least 10 us,
 * then poll STATUS for up to ~10 ms. With auto-ACK off, TX_DS means "left the
 * antenna", not "was received". The caller switches back to RX afterwards.
 */
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

/*
 * Start interrupt-driven reception: create the receive task, then hook the
 * falling edge of the nRF24 IRQ pin. Safe to call twice; the second call does
 * nothing.
 */
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
