#include "node.h"

#include "main.h"
#include "nrf24.h"
#include "protocol.h"

#if NODE_USE_AHT20
#include "aht20.h"
#else
#include "bme280.h" // node 1 has BME280
#endif
#include "ina219.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;

_Static_assert(PKT_NODE_ID >= PROTO_NODE_ID_MIN && PKT_NODE_ID <= PROTO_NODE_ID_MAX,
               "PKT_NODE_ID is outside the range the gateway accepts");

#define PKT_RETRY_MAX 3
#define PKT_ACK_TIMEOUT_MS 200
/*
 * Below this the HT7333 can no longer hold the 3.3 V rail, so the radio and
 * the sensor start to misbehave. Frames then carry PROTO_FLAG_LOW_BAT.
 */
#define NODE_LOW_BAT_MV 3400U

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
static bool s_sensor_err;
static volatile bool s_rtc_woke;
static uint8_t s_seq;
static uint8_t s_attempt;
static uint32_t s_packets_lost;

/* True until the gateway acknowledges the first frame after reset. */
static bool s_boot = true;
/*
 * Events the gateway cannot see on its own (PROTO_FLAG_PREV_LOST,
 * PROTO_FLAG_RADIO_ERR). A bit stays set until a frame carrying it is
 * acknowledged, so a lost frame cannot hide the report.
 */
static uint8_t s_pending_flags;

static int16_t s_temp;
static uint16_t s_hum;
static uint16_t s_press;
static uint16_t s_bat_mv;
/* INA219: average over the last sleep interval (matches s_mode in the same frame). */
static int32_t s_ina_frame_ua;
static uint16_t s_ina_frame_vbus_mv;
/* Stays deep until the gateway's ACK names another mode. */
static uint8_t s_mode = PROTO_MODE_DEEP;

static proto_frame_t s_frame;
static uint8_t s_tx[NRF24_PAYLOAD_LEN];
static uint8_t s_rx[NRF24_PAYLOAD_LEN];
static TickType_t s_ack_start;

static node_state_t s_state = NODE_STATE_INIT;

static void node_log(const char *text)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

void SystemClock_Config(void);

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

static const char *node_mode_name(uint8_t mode)
{
	if (mode == PROTO_MODE_ACTIVE) {
		return "active";
	}
	if (mode == PROTO_MODE_LIGHT) {
		return "light sleep";
	}
	return "deep sleep";
}

static int64_t s_ina_sum_i;
static int64_t s_ina_sum_v;
static int s_ina_samples;

static void ina_sleep_sample(void)
{
	int32_t ua;
	uint16_t vmv;

	if (ina219_read(&ua, &vmv)) {
		s_ina_sum_i += (int64_t)ua;
		s_ina_sum_v += (int64_t)vmv;
		s_ina_samples++;
	}
}

static void ina_frame_commit(void)
{
	if (s_ina_samples > 0) {
		s_ina_frame_ua = (int32_t)(s_ina_sum_i / s_ina_samples);
		s_ina_frame_vbus_mv = (uint16_t)(s_ina_sum_v / s_ina_samples);
	}
	s_ina_sum_i = 0;
	s_ina_sum_v = 0;
	s_ina_samples = 0;
}

/* One RTC tick (~1 s) in light sleep (SLEEP) or deep sleep (STOP). Radio stays off. */
static void rtc_sleep_one_second(void)
{
	vTaskSuspendAll();
	s_rtc_woke = false;
	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK) {
		Error_Handler();
	}

	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
	while (!s_rtc_woke) {
		HAL_SuspendTick();
		if (s_mode == PROTO_MODE_LIGHT) {
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			HAL_ResumeTick();
		} else {
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			HAL_ResumeTick();
			SystemClock_Config();
		}
	}
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	(void)xTaskResumeAll();
}

static void node_deep_sleep(uint32_t seconds)
{
	char buf[64];
	uint32_t t;

	snprintf(buf, sizeof(buf), "node: %s %lu s\r\n", node_mode_name(s_mode), (unsigned long)seconds);
	node_log(buf);
	nrf24_power_down();

	s_ina_sum_i = 0;
	s_ina_sum_v = 0;
	s_ina_samples = 0;

	if (s_mode == PROTO_MODE_ACTIVE) {
		for (t = 0; t < seconds; t++) {
			vTaskDelay(pdMS_TO_TICKS(1000));
			ina_sleep_sample();
		}
	} else {
		for (t = 0; t < seconds; t++) {
			rtc_sleep_one_second();
			ina_sleep_sample();
		}
	}
	ina_frame_commit();
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

/* Flags that describe this cycle, plus any earlier events not yet delivered. */
static uint8_t build_flags(void)
{
	uint8_t flags = s_pending_flags;

	if (s_boot) {
		flags |= PROTO_FLAG_BOOT;
	}
	if (s_sensor_err) {
		flags |= PROTO_FLAG_SENSOR_ERR;
	}
	if (s_bat_mv < NODE_LOW_BAT_MV) {
		flags |= PROTO_FLAG_LOW_BAT;
	}
	return flags;
}

static void node_task(void *arg)
{
	(void)arg;

	for (;;) {
		switch (s_state) {
		case NODE_STATE_INIT:
			{
				char buf[48];

				snprintf(buf, sizeof(buf), "node: WSN start, node_id %u\r\n",
				         (unsigned)PKT_NODE_ID);
				node_log(buf);
			}
			nrf24_init();
			s_radio_ready = nrf24_spi_probe();
			if (s_radio_ready) {
				nrf24_radio_init();
				node_log("node: channel 76\r\n");
			} else {
				node_log("node: SPI probe failed\r\n");
			}

			s_sensor_ready = sensor_init();
			if (!s_sensor_ready) {
				node_log("node: ERR_SENSOR init\r\n");
			}

			/* A missing sensor is reported over the air, so only a dead radio skips sending. */
			s_state = s_radio_ready ? NODE_STATE_READ_SENSOR : NODE_STATE_DEEP_SLEEP;
			break;

		/*
		 * The sensor is retried on every wake. If it is still broken the node
		 * sends a frame anyway, with PROTO_FLAG_SENSOR_ERR and zero readings, so
		 * the gateway sees the node is alive and counts the sensor error.
		 */
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
				}
			}

			if (s_sensor_ready && !sensor_read()) {
				node_log("node: ERR_SENSOR read\r\n");
				s_sensor_ready = false;
			}

			s_sensor_err = !s_sensor_ready;
			if (s_sensor_err) {
				s_temp = 0;
				s_hum = 0;
				s_press = 0;
			}
			s_state = NODE_STATE_READ_BATTERY;
			break;

		case NODE_STATE_READ_BATTERY: {
			char buf[64];
			static bool ina_up;
			static bool ina_warned;
			int32_t probe_ua;
			uint16_t probe_v;
			bool ina_present;

			s_bat_mv = read_battery_mv();
			ina_present = (s_ina_frame_ua != 0 || s_ina_frame_vbus_mv != 0) ||
			              ina219_read(&probe_ua, &probe_v);
			if (ina_present) {
				if (!ina_up) {
					node_log("node: INA219 ok\r\n");
					ina_up = true;
				}
				ina_warned = false;
			} else if (!ina_warned) {
				node_log("node: INA219 absent\r\n");
				ina_warned = true;
				ina_up = false;
			} else {
				ina_up = false;
			}
			snprintf(buf, sizeof(buf), "node: BAT %u mV INA avg %ld uA %u mV (%s)\r\n", s_bat_mv,
			         (long)s_ina_frame_ua, s_ina_frame_vbus_mv, node_mode_name(s_mode));
			node_log(buf);
			s_state = NODE_STATE_BUILD;
			break;
		}

		/* Flags are fixed here, so every attempt of one seq reports the same state. */
		case NODE_STATE_BUILD:
			s_frame = (proto_frame_t){
				.type = PROTO_TYPE_DATA,
				.node_id = PKT_NODE_ID,
				.seq = s_seq,
				.flags = build_flags(),
				.temp_c_x100 = s_temp,
				.hum_pct_x100 = s_hum,
				.press_hpa = s_press,
				.bat_mv = s_bat_mv,
				.mode = s_mode,
				.current_ua = s_ina_frame_ua,
				.vbus_mv = s_ina_frame_vbus_mv,
			};
			s_attempt = 1;
			s_state = NODE_STATE_SEND;
			break;

		/* The attempt number changes per send, so the frame and its CRC are re-encoded each time. */
		case NODE_STATE_SEND: {
			char buf[112];
			uint16_t t_abs =
			    (s_temp < 0) ? (uint16_t)(-s_temp) : (uint16_t)s_temp;

			s_frame.attempt = s_attempt;
			proto_encode(&s_frame, s_tx);

			snprintf(buf, sizeof(buf),
			         "node: DATA seq=%u temp=%s%d.%02dC hum=%u.%02u%% press=%uhPa "
			         "flags=0x%02X retry %u/%u\r\n",
			         s_seq, (s_temp < 0) ? "-" : "", t_abs / 100, t_abs % 100,
			         s_hum / 100, s_hum % 100, s_press, s_frame.flags, s_attempt,
			         PKT_RETRY_MAX);
			node_log(buf);

			if (!nrf24_send(s_tx, NRF24_PAYLOAD_LEN)) {
				node_log("node: TX FAIL\r\n");
				s_pending_flags |= PROTO_FLAG_RADIO_ERR;
				s_state = NODE_STATE_RETRY;
			} else {
				nrf24_flush_rx();
				nrf24_set_rx_mode();
				s_ack_start = xTaskGetTickCount();
				s_state = NODE_STATE_WAIT_ACK;
			}
			break;
		}

		/* An ACK confirms the flags it carried were delivered, so those are cleared. */
		case NODE_STATE_WAIT_ACK:
			if (nrf24_receive(s_rx, NRF24_PAYLOAD_LEN) &&
			    proto_is_ack_for(s_rx, PKT_NODE_ID, s_seq)) {
				proto_frame_t ack;
				char buf[48];
				if (proto_decode(s_rx, &ack) && ack.mode <= PROTO_MODE_DEEP) {
					s_mode = ack.mode;
				}
				snprintf(buf, sizeof(buf), "node: ACK seq=%u %s\r\n", s_seq, node_mode_name(s_mode));
				node_log(buf);
				s_boot = false;
				s_pending_flags &= (uint8_t)~s_frame.flags;
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
				s_pending_flags |= PROTO_FLAG_PREV_LOST;
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
	if (xTaskCreate(node_task, "node", 768, NULL, 2, NULL) != pdPASS) {
		node_log("node: xTaskCreate FAILED\r\n");
	}
}
