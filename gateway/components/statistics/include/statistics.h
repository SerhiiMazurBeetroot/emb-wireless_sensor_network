#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "protocol.h"

#define STAT_NODE_MIN PROTO_NODE_ID_MIN
#define STAT_NODE_MAX PROTO_NODE_ID_MAX

/*
 * Error counters. The node-side ones (RADIO, SENSOR, TIMEOUT, BATTERY) come
 * from the flags byte of new DATA frames; a retransmitted copy is not counted
 * twice.
 */
typedef enum {
	ERR_RADIO = 0, /* node: a transmission did not leave its radio (PROTO_FLAG_RADIO_ERR) */
	ERR_SENSOR,    /* node: sensor init or read failed (PROTO_FLAG_SENSOR_ERR) */
	ERR_CRC,       /* gateway: frame arrived with a bad CRC */
	ERR_TIMEOUT,   /* node: gave up on a frame after all attempts (PROTO_FLAG_PREV_LOST) */
	ERR_ACK,       /* gateway: an ACK failed to transmit */
	ERR_BATTERY,   /* node: frame sent with low battery (PROTO_FLAG_LOW_BAT) */
	ERR_PROTOCOL,  /* gateway: valid CRC but unknown type or node id */
	STAT_ERR_COUNT,
} stat_err_t;

typedef struct {
	uint32_t rx;          /* valid DATA frames, including repeats */
	uint32_t tx;          /* ACKs sent */
	uint32_t lost;        /* seq numbers that never arrived */
	uint32_t retries;     /* retransmissions made by the node, from the attempt byte */
	uint32_t dup;         /* repeats received because the node missed our ACK */
	uint32_t reboots;     /* node resets detected from PROTO_FLAG_BOOT */
	int64_t last_seen_us; /* esp_timer time of the last valid frame; 0 = never heard */
} stat_node_t;

typedef struct {
	uint32_t rx;
	uint32_t tx;
	uint32_t lost;
	uint32_t crc_err;
	uint32_t retries;
	uint32_t dup;
	uint32_t reboots;
	uint32_t unknown;
	uint32_t err[STAT_ERR_COUNT];
	stat_node_t node[STAT_NODE_MAX + 1];
} stat_snapshot_t;

void statistics_init(void);
void statistics_on_crc(void);
void statistics_on_unknown(uint8_t type, uint8_t node);
/*
 * Account for one valid DATA frame received at now_us. Returns true when it is
 * a repeat of the previous frame from that node, i.e. its readings are not new.
 */
bool statistics_on_data(const proto_frame_t *frame, int64_t now_us);
void statistics_on_ack(uint8_t node, bool ok);
void statistics_snapshot(stat_snapshot_t *out);
void statistics_log(void);
