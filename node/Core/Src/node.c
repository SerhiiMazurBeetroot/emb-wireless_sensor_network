#include "node.h"

#include "main.h"
#include "nrf24.h"
#include "bme280.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart1;

#define PKT_TYPE_DATA 0x01
#define PKT_TYPE_ACK 0x02
#define PKT_NODE_ID 1
#define PKT_RETRY_MAX 3
#define PKT_ACK_TIMEOUT_MS 50

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
static uint8_t s_seq;
static uint8_t s_attempt;
static uint32_t s_packets_lost;

static int16_t s_temp;
static uint16_t s_hum;
static uint16_t s_press;
static uint16_t s_bat_mv;

static uint8_t s_tx[NRF24_PAYLOAD_LEN];
static uint8_t s_rx[NRF24_PAYLOAD_LEN];
static TickType_t s_ack_deadline;

static node_state_t s_state = NODE_STATE_INIT;

static void node_log(const char *text) {
    HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

void SystemClock_Config(void);

/* CRC-8, poly 0x07, over the first 7 bytes of the 8-byte packet. */
static uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;

    for(uint8_t i = 0; i < len; i++) {
        crc ^=data[i];
        for(uint8_t bit = 0; bit < 8; bit++) {
            if(crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

static bool packet_crc_ok(const uint8_t *pkt) {
	return crc8(pkt, NRF24_PAYLOAD_LEN - 1) == pkt[NRF24_PAYLOAD_LEN - 1];
}

static void put_u16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void packet_seal(uint8_t *pkt) {
	pkt[NRF24_PAYLOAD_LEN - 1] = crc8(pkt, NRF24_PAYLOAD_LEN - 1);
}

static bool ack_matches(const uint8_t *pkt, uint8_t seq) {
	  return packet_crc_ok(pkt) && pkt[0] == PKT_TYPE_ACK && pkt[1] == PKT_NODE_ID && pkt[2] == seq;
}

/* TODO: finish */
static uint16_t read_battery_mv(void) {
	return 0;
}

/* Power down the radio and wait about 10 s. LSI is not exact. */
static void node_deep_sleep(uint32_t seconds) {
	char buf[48];
	uint32_t wake_ticks = (seconds > 0U) ? (seconds - 1U) : 0U;

    snprintf(buf, sizeof(buf), "node: deep sleep %lu s\r\n", (unsigned long)seconds);
	node_log(buf);

	nrf24_power_down();

	if(HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wake_ticks, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK) {
		Error_Handler();
	}

	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	HAL_ResumeTick();

	SystemClock_Config();
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
}

static void node_task(void *arg) {
	(void)arg;

	for(;;) {
		switch(s_state) {
			case NODE_STATE_INIT:
	            node_log("node: WSN node - step 11: deep sleep\r\n");
	            nrf24_init();
	            s_radio_ready = nrf24_spi_probe();
	            if(s_radio_ready) {
	            	nrf24_radio_init();
	                node_log("node: channel 76, node_id 1\r\n");
	            } else {
	                node_log("node: SPI probe failed\r\n");
	            }

	            if(!bme280_init()) {
	          	  node_log("node: ERR_SENSOR\r\n");
	          	  s_radio_ready = false;
	            }

	            s_state = s_radio_ready ? NODE_STATE_READ_SENSOR : NODE_STATE_DEEP_SLEEP;
	            break;

			case NODE_STATE_READ_SENSOR:
				if (!bme280_read(&s_temp, &s_hum, &s_press)) {
					node_log("node: ERR_SENSOR\r\n");
					s_state = NODE_STATE_DEEP_SLEEP;
				} else {
					s_state = NODE_STATE_READ_BATTERY;
				}
				break;

			case NODE_STATE_READ_BATTERY:
				s_bat_mv = read_battery_mv();
				s_state = NODE_STATE_BUILD;
				break;

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
				uint16_t t_abs = (s_temp < 0) ? (uint16_t)(-s_temp) : (uint16_t)s_temp;
				snprintf(buf, sizeof(buf),
						 "node: DATA seq=%u temp=%s%d.%02dC hum=%u.%02u%% press=%uhPa retry %u/%u\r\n",
						 s_seq, (s_temp < 0) ? "-" : "", t_abs / 100, t_abs % 100,
						 s_hum / 100, s_hum % 100, s_press, s_attempt, PKT_RETRY_MAX);
				node_log(buf);

				if(!nrf24_send(s_tx, NRF24_PAYLOAD_LEN)) {
					node_log("node: TX FAIL\r\n");
					s_state = NODE_STATE_RETRY;
				} else {
					nrf24_set_rx_mode();
	                s_ack_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(PKT_ACK_TIMEOUT_MS);
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
	            } else if(xTaskGetTickCount() >= s_ack_deadline) {
	            	char buf[48];
	                snprintf(buf, sizeof(buf), "node: timeout seq=%u attempt=%u\r\n", s_seq, s_attempt);
	                node_log(buf);
	                s_state = NODE_STATE_RETRY;
	            } else {
	            	vTaskDelay(pdMS_TO_TICKS(1));
	            }
	            break;

			case NODE_STATE_RETRY:
				if(s_attempt < PKT_RETRY_MAX) {
					s_attempt++;
					s_state = NODE_STATE_SEND;
				} else {
					char buf[48];
					s_packets_lost++;
					snprintf(buf, sizeof(buf), "node: no ACK seq=%u packets_lost=%lu\r\n",
					                         s_seq, (unsigned long)s_packets_lost);
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

void node_start(void) {
	xTaskCreate(node_task, "node", 512, NULL, 2, NULL);
}
