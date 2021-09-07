#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h>
#include <asm/unistd.h>
#include <errno.h>
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>

#define PERF_MAX 5

struct read_format {
	uint64_t nr;
	struct {
		uint64_t value;
		uint64_t id;
	} values[];
};

/* test loading */
void do_something() {
	int i;
	char* ptr;

	ptr = (char *) malloc(100*1024);
	for (i = 0; i < 100*1024; i++)
		ptr[i] = (char) (i & 0xff); // pagefault
	free(ptr);
}

static inline int monitor_cases(int idx) {
	switch (idx) {
	case 0: return PERF_COUNT_HW_CPU_CYCLES;
	case 1: return PERF_COUNT_HW_INSTRUCTIONS;
	case 2: return PERF_COUNT_HW_CACHE_MISSES;
	case 3: return PERF_COUNT_HW_BUS_CYCLES;
	case 4: return PERF_COUNT_HW_BRANCH_MISSES;
	}

	return PERF_COUNT_HW_CPU_CYCLES;
}

int main(int argc, char**argv) {
	int i, cnt = 10, opt, events = 1, fds[PERF_MAX] = {0};
	bool grouping = false;
	uint64_t ids[PERF_MAX] = {0}, vals[PERF_MAX] = {0};
	char buf[4096];
	struct read_format* rf = (struct read_format*) buf;
	struct perf_event_attr peas[PERF_MAX];

	while ((opt = getopt(argc, argv, "c:e:g")) != -1) {
		switch (opt) {
		case 'e':
			events = atoi(optarg);
			if (events < 1) events = 1;
			if (events > PERF_MAX) events = PERF_MAX;
			break;
		case 'g':
			grouping = true;
			break;
		case 'c':
			cnt = atoi(optarg);
			if (cnt < 10) cnt = 10;
			if (cnt > 1000) cnt = 1000;
			break;
		default: /* '?' */
			fprintf(stderr,
				"Usage: %s -e 3 -g 0 -c 10\n", argv[0]);
			return 0;
		}
	}

	printf ("monitor events: %d\n", events);
	printf ("using grouping: %d\n", grouping);

	/* start monitor perf events */
	memset(&peas, 0, sizeof(struct perf_event_attr) * PERF_MAX);
	peas[0].type = PERF_TYPE_HARDWARE;
	peas[0].size = sizeof(struct perf_event_attr);
	peas[0].config = monitor_cases(0);
	peas[0].disabled = 1;
	peas[0].exclude_kernel = 1;
	peas[0].exclude_hv = 1;
	if (grouping)
		peas[0].read_format = PERF_FORMAT_GROUP | PERF_FORMAT_ID;
	fds[0] = syscall(__NR_perf_event_open, &peas[0], 0, -1, -1, 0);
	ioctl(fds[0], PERF_EVENT_IOC_ID, &ids[0]);

	for (i = 1; i < events; ++i) {
		memset(&peas, 0, sizeof(struct perf_event_attr));
		peas[i].type = PERF_TYPE_HARDWARE;
		peas[i].size = sizeof(struct perf_event_attr);
		peas[i].config = monitor_cases(i);
		peas[i].disabled = 1;
		peas[i].exclude_kernel = 1;
		peas[i].exclude_hv = 1;
		if (grouping) {
			peas[i].read_format = PERF_FORMAT_GROUP | PERF_FORMAT_ID;
			fds[i] = syscall(__NR_perf_event_open, &peas[i], 0, -1, fds[0], 0);
			ioctl(fds[i], PERF_EVENT_IOC_ID, &ids[i]);
		} else
			fds[i] = syscall(__NR_perf_event_open, &peas[i], 0, -1, -1, 0);
	}

	ioctl(fds[0], PERF_EVENT_IOC_RESET, grouping? PERF_IOC_FLAG_GROUP: 0);
	ioctl(fds[0], PERF_EVENT_IOC_ENABLE, grouping? PERF_IOC_FLAG_GROUP: 0);
	for (i = 1; i < events && !grouping; ++i) {
		ioctl(fds[i], PERF_EVENT_IOC_RESET, PERF_IOC_FLAG_GROUP);
		ioctl(fds[i], PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP);
	}

	while (cnt--) {
		do_something();
		memset(buf, 0, sizeof(buf));
		if (grouping) {
			read(fds[0], buf, sizeof(buf));
			for (uint64_t i = 0; i < rf->nr; i++) {
				if (rf->values[i].id == ids[i]) {
					vals[i] = rf->values[i].value;
				}
			}
		} else {
			for (i = 0; i < events; ++i)
				read(fds[i], &vals[i], sizeof(uint64_t));
		}
		printf("cpu cycles: %ld\n", vals[0]);
		printf("instructions: %ld\n", vals[1]);
		printf("cache misses: %ld\n", vals[2]);
		printf("bus cycles: %ld\n", vals[3]);
		printf("branch misses: %ld\n", vals[4]);
		printf("----------------------\n");
	}

	/* disable */
	if (grouping)
		ioctl(fds[0], PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP);
	else {
		for (i = 0; i < events; ++i)
			if (fds[i])
				ioctl(fds[i], PERF_EVENT_IOC_DISABLE, 0);
	}
	return 0;
}
