#include "statistics.h"

#include "esp_log.h"

static const char *TAG = "stats";

typedef struct {
	uint8_t last_seq;
	bool have_seq;
} stat_seq_t;

static stat_snapshot_t s_stat;
static stat_seq_t s_seq[STAT_NODE_MAX + 1];

void statistics_init(void) {
	s_stat = (stat_snapshot_t){0};
	for(uint8_t i = 0; i <= STAT_NODE_MAX; i++) {
		s_seq[i].have_seq = false;
	}
}

static void count_err(stat_err_t err) {
	if((unsigned)err < STAT_ERR_COUNT) {
		s_stat.err[err]++;
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

bool statistics_on_data(uint8_t node, uint8_t seq) {
	bool retry = false;
	
	if(node < STAT_NODE_MIN || node > STAT_NODE_MAX) {
		statistics_on_unknown(0x01, node);
		return false;
	}
	
	s_stat.rx++;
	s_stat.node[node].rx++;
	
	if(s_seq[node].have_seq) {
		if(seq == s_seq[node].last_seq) {
			retry = true;
			s_stat.retries++;
			s_stat.node[node].retries++;
		} else {
			const uint8_t gap = (uint8_t)(seq - (uint8_t)(s_seq[node].last_seq + 1U));
			if(gap >= 1U && gap <= 10U) {
				s_stat.lost += gap;
				s_stat.node[node].lost += gap;
			}
		}
	}
	
	s_seq[node].last_seq = seq;
	s_seq[node].have_seq = true;
	return retry;
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
	ESP_LOGI(TAG, "RX=%lu TX=%lu LOST=%lu CRC=%lu RETRIES=%lu UNK=%lu",
	         (unsigned long)s_stat.rx, (unsigned long)s_stat.tx,
	         (unsigned long)s_stat.lost, (unsigned long)s_stat.crc_err,
	         (unsigned long)s_stat.retries, (unsigned long)s_stat.unknown);
	ESP_LOGI(TAG, "ERR radio=%lu sensor=%lu crc=%lu timeout=%lu ack=%lu bat=%lu proto=%lu",
	         (unsigned long)s_stat.err[ERR_RADIO], (unsigned long)s_stat.err[ERR_SENSOR],
	         (unsigned long)s_stat.err[ERR_CRC], (unsigned long)s_stat.err[ERR_TIMEOUT],
	         (unsigned long)s_stat.err[ERR_ACK], (unsigned long)s_stat.err[ERR_BATTERY],
	         (unsigned long)s_stat.err[ERR_PROTOCOL]);
	for (uint8_t id = STAT_NODE_MIN; id <= STAT_NODE_MAX; id++) {
		const stat_node_t *n = &s_stat.node[id];
		ESP_LOGI(TAG, "N%u RX=%lu TX=%lu LOST=%lu RETRIES=%lu", id,
		         (unsigned long)n->rx, (unsigned long)n->tx,
		         (unsigned long)n->lost, (unsigned long)n->retries);
	}
}
