#pragma once

#ifndef PKT_NODE_ID
#define PKT_NODE_ID 0x01U
#endif

#ifndef NODE_USE_AHT20
#define NODE_USE_AHT20 0
#endif

#define SAMPLE_INTERVAL_SEC 10U

void node_start(void);
