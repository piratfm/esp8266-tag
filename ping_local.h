#ifndef __PING_LOCAL_H__
#define __PING_LOCAL_H__
#include "lwip/ip_addr.h"
#include "lwip/icmp.h"
/**
 * PING_USE_SOCKETS: Set to 1 to use sockets, otherwise the raw api is used
 */
#ifndef PING_USE_SOCKETS
#define PING_USE_SOCKETS    LWIP_SOCKET
#endif

/**
 * PING_DEBUG: Enable debugging for PING.
 */
#ifndef PING_DEBUG
#define PING_DEBUG     LWIP_DBG_OFF
#endif

/** ping receive timeout - in milliseconds */
#ifndef PING_RCV_TIMEO
#define PING_RCV_TIMEO  100
#endif

/** ping delay - in milliseconds */
#ifndef PING_COARSE
#define PING_COARSE     100
#endif

/** ping identifier - must fit on a u16_t */
#ifndef PING_ID
#define PING_ID        0xAFAF
#endif

/** ping additional data size to include in the packet */
#ifndef PING_DATA_SIZE
#define PING_DATA_SIZE 32
#endif

/** ping result action - no default action */
#ifndef PING_RESULT
#define PING_RESULT(ping_ok)
#endif

#define DEFAULT_PING_MAX_COUNT 4
#define PING_TIMEOUT_MS 1000

struct ping_msg{
	struct ping_option *ping_opt;
	struct raw_pcb *ping_pcb;
	uint32 ping_start;
	uint32 ping_sent;
	uint32 timeout_count;
	uint32 max_count;
	uint32 sent_count;
	uint32 coarse_time;
};

#endif /* __PING_LOCAL_H__ */
