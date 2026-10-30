#pragma once

#include "protocol.h"

#ifndef PKT_NODE_ID
#define PKT_NODE_ID 0x01U
#endif

#ifndef NODE_USE_AHT20
#define NODE_USE_AHT20 0
#endif

#define SAMPLE_INTERVAL_SEC PROTO_SAMPLE_INTERVAL_S

void node_start(void);
