#pragma once

#include <stdbool.h>
#include <stdint.h>

#define STAT_NODE_MIN 1U
#define STAT_NODE_MAX 2U

typedef enum {
	ERR_RADIO = 0,
	ERR_SENSOR,
	ERR_CRC,
	ERR_TIMEOUT,
	ERR_ACK,
	ERR_BATTERY,
	ERR_PROTOCOL,
	STAT_ERR_COUNT,
} stat_err_t;

typedef struct {
	uint32_t rx;
	uint32_t tx;
	uint32_t lost;
	uint32_t retries;
} stat_node_t;

typedef struct {
	uint32_t rx;
	uint32_t tx;
	uint32_t lost;
	uint32_t crc_err;
	uint32_t retries;
	uint32_t unknown;
	uint32_t err[STAT_ERR_COUNT];
	stat_node_t node[STAT_NODE_MAX + 1];
} stat_snapshot_t;

void statistics_init(void);
void statistics_on_crc(void);
void statistics_on_unknown(uint8_t type, uint8_t node);
/* True when this seq was already counted for the node. */
bool statistics_on_data(uint8_t node, uint8_t seq);
void statistics_on_ack(uint8_t node, bool ok);
void statistics_snapshot(stat_snapshot_t *out);
void statistics_log(void);