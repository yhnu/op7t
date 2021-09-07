#ifndef _MEMORY_PLUS_H
#define _MEMORY_PLUS_H
#include <oneplus/memplus/memplus_helper.h>
#define AID_APP	10000  /* first app user */

enum {
	RECLAIM_STANDBY,
	RECLAIM_QUEUE,
	RECLAIM_DONE,
	SWAPIN_QUEUE,
};

enum {
	TYPE_NORMAL,
	TYPE_FREQUENT,
	TYPE_SYS_IGNORE,
	TYPE_WILL_NEED,
	TYPE_END
};
#define MEMPLUS_TYPE_MASK	0x7

#endif /* _MEMORY_PLUS_H */
