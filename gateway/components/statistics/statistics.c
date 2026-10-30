#include "statistics.h"

#include <stddef.h>

#include "esp_log.h"

static const char *TAG = "stats";

/* A jump in seq larger than this is treated as a resync, not as lost frames. */
#define STAT_MAX_GAP 10U

/* What the gateway remembers about the last frame from one node. */
typedef struct {
	uint8_t last_seq;
	bool last_boot;      /* the last frame carried PROTO_FLAG_BOOT */
	uint8_t max_attempt; /* highest attempt number seen for last_seq */
	bool have_seq;
} stat_seq_t;

static stat_snapshot_t s_stat;
static stat_seq_t s_seq[STAT_NODE_MAX + 1];

void statistics_init(void) {
	s_stat = (stat_snapshot_t){0};
	for(uint8_t i = 0; i <= STAT_NODE_MAX; i++) {
		s_seq[i] = (stat_seq_t){0};
	}
}

static void count_err(stat_err_t err) {
	if((unsigned)err < STAT_ERR_COUNT) {
		s_stat.err[err]++;
	}
}

static void add_lost(stat_node_t *n, uint32_t count) {
	s_stat.lost += count;
	n->lost += count;
}

static void add_retries(stat_node_t *n, uint32_t count) {
	s_stat.retries += count;
	n->retries += count;
}

/* Turn the node-reported flags of a new frame into error counts. */
static void count_node_flags(uint8_t flags) {
	if(flags & PROTO_FLAG_RADIO_ERR) {
		count_err(ERR_RADIO);
	}
	if(flags & PROTO_FLAG_SENSOR_ERR) {
		count_err(ERR_SENSOR);
	}
	if(flags & PROTO_FLAG_PREV_LOST) {
		count_err(ERR_TIMEOUT);
	}
	if(flags & PROTO_FLAG_LOW_BAT) {
		count_err(ERR_BATTERY);
	}
}

void statistics_on_crc(void) {
	s_stat.crc_err++;
	count_err(ERR_CRC);
}

void statistics_on_unknown(uint8_t type, uint8_t node) {
	(void)type;
	(void)node;
	s_stat.unknown++;
	count_err(ERR_PROTOCOL);
}

bool statistics_on_data(const proto_frame_t *frame, int64_t now_us) {
	const uint8_t id = frame->node_id;

	if(id < STAT_NODE_MIN || id > STAT_NODE_MAX) {
		statistics_on_unknown(frame->type, id);
		return false;
	}

	stat_node_t *n = &s_stat.node[id];
	stat_seq_t *s = &s_seq[id];
	const bool boot = (frame->flags & PROTO_FLAG_BOOT) != 0;
	const uint8_t attempt = (frame->attempt > 0U) ? frame->attempt : 1U;

	s_stat.rx++;
	n->rx++;
	n->last_seen_us = now_us;

	/*
	 * Repeat: the node did not hear our ACK and sent the same frame again.
	 * The boot flag must match too, otherwise a reset node that happens to
	 * restart on the same seq would be taken for a repeat.
	 */
	if(s->have_seq && frame->seq == s->last_seq && boot == s->last_boot) {
		s_stat.dup++;
		n->dup++;
		if(attempt > s->max_attempt) {
			add_retries(n, attempt - s->max_attempt);
			s->max_attempt = attempt;
		}
		return true;
	}

	if(boot && !(s->have_seq && s->last_boot)) {
		/*
		 * First frame after a node reset. Its seq restarted at 0, so comparing
		 * with the old seq would invent a gap. Frames 0..seq-1 sent since the
		 * reset never arrived and are the only real loss.
		 */
		if(s->have_seq) {
			s_stat.reboots++;
			n->reboots++;
		}
		add_lost(n, frame->seq);
	} else if(s->have_seq) {
		const uint8_t gap = (uint8_t)(frame->seq - (uint8_t)(s->last_seq + 1U));
		if(gap >= 1U && gap <= STAT_MAX_GAP) {
			add_lost(n, gap);
		}
	}

	/* Attempts 1..attempt-1 of this frame were sent but never reached us. */
	add_retries(n, attempt - 1U);
	count_node_flags(frame->flags);

	s->last_seq = frame->seq;
	s->last_boot = boot;
	s->max_attempt = attempt;
	s->have_seq = true;
	return false;
}

void statistics_on_ack(uint8_t node, bool ok) {
	if(node < STAT_NODE_MIN || node > STAT_NODE_MAX) {
		return;
	}
	if(ok) {
		s_stat.tx++;
		s_stat.node[node].tx++;
		return;
	}
	count_err(ERR_ACK);
}

void statistics_snapshot(stat_snapshot_t *out) {
	if(out != NULL) {
		*out = s_stat;
	}
}


void statistics_log(void) {
	ESP_LOGI(TAG, "RX=%lu TX=%lu LOST=%lu CRC=%lu RETRIES=%lu DUP=%lu REBOOTS=%lu UNK=%lu",
	         (unsigned long)s_stat.rx, (unsigned long)s_stat.tx,
	         (unsigned long)s_stat.lost, (unsigned long)s_stat.crc_err,
	         (unsigned long)s_stat.retries, (unsigned long)s_stat.dup,
	         (unsigned long)s_stat.reboots, (unsigned long)s_stat.unknown);
	ESP_LOGI(TAG, "ERR radio=%lu sensor=%lu crc=%lu timeout=%lu ack=%lu bat=%lu proto=%lu",
	         (unsigned long)s_stat.err[ERR_RADIO], (unsigned long)s_stat.err[ERR_SENSOR],
	         (unsigned long)s_stat.err[ERR_CRC], (unsigned long)s_stat.err[ERR_TIMEOUT],
	         (unsigned long)s_stat.err[ERR_ACK], (unsigned long)s_stat.err[ERR_BATTERY],
	         (unsigned long)s_stat.err[ERR_PROTOCOL]);
	for (uint8_t id = STAT_NODE_MIN; id <= STAT_NODE_MAX; id++) {
		const stat_node_t *n = &s_stat.node[id];
		ESP_LOGI(TAG, "N%u RX=%lu TX=%lu LOST=%lu RETRIES=%lu DUP=%lu REBOOTS=%lu", id,
		         (unsigned long)n->rx, (unsigned long)n->tx, (unsigned long)n->lost,
		         (unsigned long)n->retries, (unsigned long)n->dup,
		         (unsigned long)n->reboots);
	}
}
