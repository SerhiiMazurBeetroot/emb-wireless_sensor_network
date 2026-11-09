#pragma once

/*
 * Radio protocol shared by the nodes (STM32) and the gateway (ESP32-S3).
 *
 * Both firmwares include this one file, so the frame layout, packet types,
 * flags and CRC can only change in one place. Any change here changes what is
 * sent over the air: rebuild and reflash all three boards together.
 */

#include <stdbool.h>
#include <stdint.h>

/* Node IDs the gateway accepts. Each node build sets its own PKT_NODE_ID. */
#define PROTO_NODE_ID_MIN 1U
#define PROTO_NODE_ID_MAX 2U

/* How often a node wakes and sends. The gateway uses it to decide when a node is offline. */
#define PROTO_SAMPLE_INTERVAL_S 10U

#define PROTO_TYPE_DATA 0x01U
#define PROTO_TYPE_ACK 0x02U

/* Power mode the node used for this sample. An ACK carries the mode the gateway wants next. */
#define PROTO_MODE_ACTIVE 0U
#define PROTO_MODE_LIGHT 1U
#define PROTO_MODE_DEEP 2U

/*
 * DATA flags. BOOT, SENSOR_ERR and LOW_BAT describe the node right now.
 * PREV_LOST and RADIO_ERR report problems the gateway could not see itself;
 * the node keeps them set until one of its frames is acknowledged.
 */
/* No frame acknowledged since reset: seq restarted at 0, the gateway must resync. */
#define PROTO_FLAG_BOOT 0x01U
/* Sensor init or read failed. Temperature, humidity and pressure are 0. */
#define PROTO_FLAG_SENSOR_ERR 0x02U
/* Battery below the node's low-battery threshold, or the ADC read failed. */
#define PROTO_FLAG_LOW_BAT 0x04U
/* An earlier frame used all its attempts without getting an ACK. */
#define PROTO_FLAG_PREV_LOST 0x08U
/* The node's radio reported a failed transmission (no TX_DS). */
#define PROTO_FLAG_RADIO_ERR 0x10U

/* Byte offsets in a frame. Multi-byte fields are little-endian. */
#define PROTO_OFF_TYPE 0
#define PROTO_OFF_NODE 1    /* source node; an ACK carries the node it answers */
#define PROTO_OFF_SEQ 2     /* wraps 255 -> 0; the same value on every retry */
#define PROTO_OFF_FLAGS 3   /* PROTO_FLAG_*, 0 in ACK frames */
#define PROTO_OFF_ATTEMPT 4 /* 1 on first send, 2 and 3 on retries */
#define PROTO_OFF_TEMP 5    /* int16, 0.01 C */
#define PROTO_OFF_HUM 7     /* uint16, 0.01 %RH */
#define PROTO_OFF_PRESS 9   /* uint16, hPa; 0 when the sensor has no barometer */
#define PROTO_OFF_BAT 11    /* uint16, mV; 0 when not measured */
#define PROTO_OFF_MODE 13   /* PROTO_MODE_*; in an ACK this is the commanded mode */
#define PROTO_OFF_CURR 14   /* int32, µA: sleep-interval average (INA219); 0 if absent */
#define PROTO_OFF_VBUS 18   /* uint16, bus mV averaged with current; 0 if absent */
#define PROTO_OFF_CRC 20    /* CRC-8 over every byte before it */
#define PROTO_FRAME_LEN 21

_Static_assert(PROTO_OFF_CRC + 1 == PROTO_FRAME_LEN, "CRC must be the last byte of the frame");
_Static_assert(PROTO_FRAME_LEN == 21, "frame size changed: reflash the gateway and both nodes together");
_Static_assert(PROTO_FRAME_LEN <= 32, "an nRF24 payload holds at most 32 bytes");

/* One frame in decoded form. An ACK uses type, node_id, seq, attempt and mode. */
typedef struct {
	uint8_t type;
	uint8_t node_id;
	uint8_t seq;
	uint8_t flags;
	uint8_t attempt;
	int16_t temp_c_x100;
	uint16_t hum_pct_x100;
	uint16_t press_hpa;
	uint16_t bat_mv;
	uint8_t mode;
	int32_t current_ua;
	uint16_t vbus_mv;
} proto_frame_t;

/* CRC-8, polynomial 0x07, initial value 0, no final XOR. */
static inline uint8_t proto_crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0x00;

	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

static inline void proto_put_u16(uint8_t *p, uint16_t value)
{
	p[0] = (uint8_t)value;
	p[1] = (uint8_t)(value >> 8);
}

static inline uint16_t proto_get_u16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline void proto_put_i32(uint8_t *p, int32_t value)
{
	uint32_t bits = (uint32_t)value;

	p[0] = (uint8_t)bits;
	p[1] = (uint8_t)(bits >> 8);
	p[2] = (uint8_t)(bits >> 16);
	p[3] = (uint8_t)(bits >> 24);
}

static inline int32_t proto_get_i32(const uint8_t *p)
{
	return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	                 ((uint32_t)p[3] << 24));
}

/* Write every field of f into out and append the CRC. */
static inline void proto_encode(const proto_frame_t *f, uint8_t out[PROTO_FRAME_LEN])
{
	out[PROTO_OFF_TYPE] = f->type;
	out[PROTO_OFF_NODE] = f->node_id;
	out[PROTO_OFF_SEQ] = f->seq;
	out[PROTO_OFF_FLAGS] = f->flags;
	out[PROTO_OFF_ATTEMPT] = f->attempt;
	proto_put_u16(&out[PROTO_OFF_TEMP], (uint16_t)f->temp_c_x100);
	proto_put_u16(&out[PROTO_OFF_HUM], f->hum_pct_x100);
	proto_put_u16(&out[PROTO_OFF_PRESS], f->press_hpa);
	proto_put_u16(&out[PROTO_OFF_BAT], f->bat_mv);
	out[PROTO_OFF_MODE] = f->mode;
	proto_put_i32(&out[PROTO_OFF_CURR], f->current_ua);
	proto_put_u16(&out[PROTO_OFF_VBUS], f->vbus_mv);
	out[PROTO_OFF_CRC] = proto_crc8(out, PROTO_OFF_CRC);
}

/* Fill f from a received frame. Returns false when the CRC does not match; f is then unusable. */
static inline bool proto_decode(const uint8_t in[PROTO_FRAME_LEN], proto_frame_t *f)
{
	if (proto_crc8(in, PROTO_OFF_CRC) != in[PROTO_OFF_CRC]) {
		return false;
	}

	f->type = in[PROTO_OFF_TYPE];
	f->node_id = in[PROTO_OFF_NODE];
	f->seq = in[PROTO_OFF_SEQ];
	f->flags = in[PROTO_OFF_FLAGS];
	f->attempt = in[PROTO_OFF_ATTEMPT];
	f->temp_c_x100 = (int16_t)proto_get_u16(&in[PROTO_OFF_TEMP]);
	f->hum_pct_x100 = proto_get_u16(&in[PROTO_OFF_HUM]);
	f->press_hpa = proto_get_u16(&in[PROTO_OFF_PRESS]);
	f->bat_mv = proto_get_u16(&in[PROTO_OFF_BAT]);
	f->mode = in[PROTO_OFF_MODE];
	f->current_ua = proto_get_i32(&in[PROTO_OFF_CURR]);
	f->vbus_mv = proto_get_u16(&in[PROTO_OFF_VBUS]);
	return true;
}

/* Build the ACK for one DATA frame. mode is the power mode the node should use next. */
static inline void proto_encode_ack(const proto_frame_t *data, uint8_t mode, uint8_t out[PROTO_FRAME_LEN])
{
	const proto_frame_t ack = {
		.type = PROTO_TYPE_ACK,
		.node_id = data->node_id,
		.seq = data->seq,
		.attempt = data->attempt,
		.mode = mode,
	};

	proto_encode(&ack, out);
}

/* True when in is a valid ACK addressed to this node for this seq. */
static inline bool proto_is_ack_for(const uint8_t in[PROTO_FRAME_LEN], uint8_t node_id, uint8_t seq)
{
	proto_frame_t f;

	return proto_decode(in, &f) && f.type == PROTO_TYPE_ACK && f.node_id == node_id && f.seq == seq;
}
