#pragma once

#include <stdint.h>

#include "protocol.h"

/* SoftAP + page. A failure is logged; the radio keeps running. */
void webui_start(void);

/* Mode selected on the page: PROTO_MODE_ACTIVE, LIGHT or DEEP. */
uint8_t webui_mode(void);

/* Store one new DATA frame for the charts and the CSV export. */
void webui_note(const proto_frame_t *frame);
