#include "node.h"

#include "main.h"
#include "nrf24.h"

#if NODE_USE_AHT20
#include "aht20.h"
#else
#include "bme280.h"
#endif

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;

#define PKT_TYPE_DATA 0x01
#define PKT_TYPE_ACK 0x02
#define PKT_RETRY_MAX 3
#define PKT_ACK_TIMEOUT_MS 200

typedef enum {
	NODE_STATE_INIT,
	NODE_STATE_READ_SENSOR,
	NODE_STATE_READ_BATTERY,
	NODE_STATE_BUILD,
	NODE_STATE_SEND,
	NODE_STATE_WAIT_ACK,
	NODE_STATE_RETRY,
	NODE_STATE_DEEP_SLEEP,
} node_state_t;

static bool s_radio_ready;
static bool s_sensor_ready;
static volatile bool s_rtc_woke;
static uint8_t s_seq;
static uint8_t s_attempt;
static uint32_t s_packets_lost;

static int16_t s_temp;
static uint16_t s_hum;
static uint16_t s_press;
static uint16_t s_bat_mv;

static uint8_t s_tx[NRF24_PAYLOAD_LEN];
static uint8_t s_rx[NRF24_PAYLOAD_LEN];
static TickType_t s_ack_start;

static node_state_t s_state = NODE_STATE_INIT;

static void node_log(const char *text)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

void SystemClock_Config(void);

static uint8_t crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0x00;

	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
				crc = (uint8_t)((crc << 1) ^ 0x07);
			} else {
				crc = (uint8_t)(crc << 1);
			}
		}
	}
	return crc;
}

static bool packet_crc_ok(const uint8_t *pkt)
{
	return crc8(pkt, NRF24_PAYLOAD_LEN - 1) == pkt[NRF24_PAYLOAD_LEN - 1];
}

static void put_u16(uint8_t *p, uint16_t value)
{
	p[0] = (uint8_t)value;
	p[1] = (uint8_t)(value >> 8);
}

static void packet_seal(uint8_t *pkt)
{
	pkt[NRF24_PAYLOAD_LEN - 1] = crc8(pkt, NRF24_PAYLOAD_LEN - 1);
}

static bool ack_matches(const uint8_t *pkt, uint8_t seq)
{
	return packet_crc_ok(pkt) && pkt[0] == PKT_TYPE_ACK && pkt[1] == PKT_NODE_ID &&
	       pkt[2] == seq;
}

static void battery_adc_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	(void)RCC->APB2ENR;

	GPIOA->MODER |= (3U << (1U * 2U));
	GPIOA->PUPDR &= ~(3U << (1U * 2U));

	ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;
	ADC1->CR1 = 0;
	ADC1->CR2 = 0;
	ADC1->SMPR2 |= ADC_SMPR2_SMP1;
	ADC1->SQR1 = 0;
	ADC1->SQR3 = 1U;
	ADC1->CR2 = ADC_CR2_ADON;

	for (volatile uint32_t i = 0; i < 500U; i++) {
	}
}

static uint16_t read_battery_mv(void)
{
	static bool ready;

	if(!ready) {
		battery_adc_init();
		ready = true;
	}

	uint32_t sum = 0;
	uint8_t got = 0;

	for(uint8_t i = 0; i < 8U; i++) {
		ADC1->SR = 0;
		ADC1->CR2 |= ADC_CR2_SWSTART;

		uint16_t raw = 0;
		for(uint32_t n = 0; n < 100000U; n++) {
			if((ADC1->SR & ADC_SR_EOC) != 0U) {
				raw = (uint16_t)ADC1->DR;
				break;
			}
		}
		if(raw == 0U) {
			continue;
		}
		sum += raw;
		got++;
	}

	if(got == 0U) {
		return 0;
	}

	return (uint16_t)((sum / got) * 6600UL / 4095UL);
}

/* Re-init SPI2 + nRF24 after STOP (clocks/GPIO may need refresh). */
static void node_radio_wake(void)
{
	HAL_SPI_Init(&hspi2);
	nrf24_init();
	s_radio_ready = nrf24_spi_probe();
	if (!s_radio_ready) {
		node_log("node: SPI probe fail after wake\r\n");
		return;
	}
	nrf24_radio_init();
	nrf24_flush_rx();
}

static void node_deep_sleep(uint32_t seconds)
{
	char buf[48];
	uint32_t wake_ticks = (seconds > 0U) ? (seconds - 1U) : 0U;

	snprintf(buf, sizeof(buf), "node: deep sleep %lu s\r\n", (unsigned long)seconds);
	node_log(buf);

	nrf24_power_down();

	vTaskSuspendAll();
	s_rtc_woke = false;
	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wake_ticks, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) !=
	    HAL_OK) {
		Error_Handler();
	}

	/* A pending SysTick or any other IRQ ends WFI early, so sleep until the RTC fires. */
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
	while (!s_rtc_woke) {
		HAL_SuspendTick();
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		HAL_ResumeTick();
		SystemClock_Config();
	}
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	(void)xTaskResumeAll();
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *rtc)
{
	(void)rtc;
	s_rtc_woke = true;
}

static bool sensor_init(void)
{
// node 2 only has AHT20
#if NODE_USE_AHT20
	return aht20_init();
#else
// node 1 has BME280
	return bme280_init();
#endif
}

static bool sensor_read(void)
{
#if NODE_USE_AHT20
	return aht20_read(&s_temp, &s_hum, &s_press);
#else
	return bme280_read(&s_temp, &s_hum, &s_press);
#endif
}

static void node_task(void *arg)
{
	(void)arg;

	for (;;) {
		switch (s_state) {
		case NODE_STATE_INIT:
			node_log("node: WSN start\r\n");
			nrf24_init();
			s_radio_ready = nrf24_spi_probe();
			if (s_radio_ready) {
				nrf24_radio_init();
				char buf[48];
				snprintf(buf, sizeof(buf), "node: channel 76, node_id %u\r\n",
				         (unsigned)PKT_NODE_ID);
				node_log(buf);
			} else {
				node_log("node: SPI probe failed\r\n");
			}

			s_sensor_ready = sensor_init();
			if (!s_sensor_ready) {
				node_log("node: ERR_SENSOR init\r\n");
			}

			s_state = (s_radio_ready && s_sensor_ready) ? NODE_STATE_READ_SENSOR
			                                            : NODE_STATE_DEEP_SLEEP;
			break;

		case NODE_STATE_READ_SENSOR:
			node_radio_wake();
			if (!s_radio_ready) {
				s_state = NODE_STATE_DEEP_SLEEP;
				break;
			}

			if (!s_sensor_ready) {
				s_sensor_ready = sensor_init();
				if (!s_sensor_ready) {
					node_log("node: ERR_SENSOR init\r\n");
					s_state = NODE_STATE_DEEP_SLEEP;
					break;
				}
			}

			if (!sensor_read()) {
				node_log("node: ERR_SENSOR read\r\n");
				s_sensor_ready = false;
				s_state = NODE_STATE_DEEP_SLEEP;
			} else {
				s_state = NODE_STATE_READ_BATTERY;
			}
			break;

		case NODE_STATE_READ_BATTERY: {
			char buf[32];
			s_bat_mv = read_battery_mv();
			snprintf(buf, sizeof(buf), "node: BAT %u mV\r\n", s_bat_mv);
			node_log(buf);
			s_state = NODE_STATE_BUILD;
			break;
		}

		case NODE_STATE_BUILD:
			memset(s_tx, 0, sizeof(s_tx));
			s_tx[0] = PKT_TYPE_DATA;
			s_tx[1] = PKT_NODE_ID;
			s_tx[2] = s_seq;

			put_u16(&s_tx[3], (uint16_t)s_temp);
			put_u16(&s_tx[5], s_hum);
			put_u16(&s_tx[7], s_press);
			put_u16(&s_tx[9], s_bat_mv);
			packet_seal(s_tx);

			s_attempt = 1;
			s_state = NODE_STATE_SEND;
			break;

		case NODE_STATE_SEND: {
			char buf[96];
			uint16_t t_abs =
			    (s_temp < 0) ? (uint16_t)(-s_temp) : (uint16_t)s_temp;
			snprintf(buf, sizeof(buf),
			         "node: DATA seq=%u temp=%s%d.%02dC hum=%u.%02u%% press=%uhPa "
			         "retry %u/%u\r\n",
			         s_seq, (s_temp < 0) ? "-" : "", t_abs / 100, t_abs % 100,
			         s_hum / 100, s_hum % 100, s_press, s_attempt, PKT_RETRY_MAX);
			node_log(buf);

			if (!nrf24_send(s_tx, NRF24_PAYLOAD_LEN)) {
				node_log("node: TX FAIL\r\n");
				s_state = NODE_STATE_RETRY;
			} else {
				nrf24_flush_rx();
				nrf24_set_rx_mode();
				s_ack_start = xTaskGetTickCount();
				s_state = NODE_STATE_WAIT_ACK;
			}
			break;
		}

		case NODE_STATE_WAIT_ACK:
			if (nrf24_receive(s_rx, NRF24_PAYLOAD_LEN) && ack_matches(s_rx, s_seq)) {
				char buf[32];
				snprintf(buf, sizeof(buf), "node: ACK seq=%u\r\n", s_seq);
				node_log(buf);
				s_seq++;
				s_state = NODE_STATE_DEEP_SLEEP;
			} else if ((TickType_t)(xTaskGetTickCount() - s_ack_start) >=
			           pdMS_TO_TICKS(PKT_ACK_TIMEOUT_MS)) {
				char buf[48];
				snprintf(buf, sizeof(buf), "node: timeout seq=%u attempt=%u\r\n", s_seq,
				         s_attempt);
				node_log(buf);
				s_state = NODE_STATE_RETRY;
			} else {
				vTaskDelay(pdMS_TO_TICKS(1));
			}
			break;

		case NODE_STATE_RETRY:
			if (s_attempt < PKT_RETRY_MAX) {
				s_attempt++;
				nrf24_flush_rx();
				nrf24_radio_init();
				s_state = NODE_STATE_SEND;
			} else {
				char buf[48];
				s_packets_lost++;
				snprintf(buf, sizeof(buf), "node: no ACK seq=%u packets_lost=%lu\r\n", s_seq,
				         (unsigned long)s_packets_lost);
				node_log(buf);
				s_seq++;
				s_state = NODE_STATE_DEEP_SLEEP;
			}
			break;

		case NODE_STATE_DEEP_SLEEP:
			node_deep_sleep(SAMPLE_INTERVAL_SEC);
			s_state = NODE_STATE_READ_SENSOR;
			break;

		default:
			s_state = NODE_STATE_INIT;
			break;
		}
	}
}

void node_start(void)
{
	if (xTaskCreate(node_task, "node", 512, NULL, 2, NULL) != pdPASS) {
		node_log("node: xTaskCreate FAILED\r\n");
	}
}
