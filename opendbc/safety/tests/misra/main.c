#include "opendbc/safety/safety.h"

// this file is checked by cppcheck

extern uint32_t microsecond_timer_get(void);

// Ignore misra-c2012-8.7 as these functions are only called from libsafety
SAFETY_UNUSED(heartbeat_engaged);
// madsheartbeat2pnw: written by panda's board/main_comms.h (USB 0xf3 param2) and read by
// mads_heartbeat_engaged_check(), which panda's board/main.c calls at 1 Hz -- neither of which
// is part of this standalone MISRA translation unit.
SAFETY_UNUSED(heartbeat_engaged_mads);
SAFETY_UNUSED(mads_heartbeat_engaged_check);

SAFETY_UNUSED(safety_rx_hook);
SAFETY_UNUSED(safety_tx_hook);
SAFETY_UNUSED(safety_fwd_hook);
SAFETY_UNUSED(safety_tick);
SAFETY_UNUSED(set_safety_hooks);
