#include <iostream>
#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/perf_event.h>
#include <asm/unistd.h>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>

using namespace std;

#define FREQ (576000)

inline long perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
		int cpu, int group_fd, unsigned long flags)
{
	return syscall(__NR_perf_event_open, hw_event, pid, cpu, group_fd, flags);
}

void test1() {
	cout << getpid() << ": test: print 1000" << endl;
	int cnt = 100000;
	while (--cnt)
		cout << cnt;
	cout << endl;
}

void test2() {
	cout << getpid() << ": test: print 5000" << endl;
	int cnt = 5000;
	while (--cnt)
		cout << cnt;
	cout << endl;
}

long long parse(string content) {
	/* take first col */
	if (!content.empty()) {
		stringstream ss(content);
		string s;
		getline(ss, s, ' ');
		return stoll(s);
	}
	return 0LL;
}

void loop(int fd, void (*f)(void)) {
	cout << "---------------------------------" << endl;
	long long count;
	long long ts1, ts2;

	ioctl(fd, PERF_EVENT_IOC_RESET, 0);
	ioctl(fd, PERF_EVENT_IOC_ENABLE, 0);

	std::string content;
	android::base::ReadFileToString("/proc/self/schedstat", &content);
	cout << content;
	ts1 = parse(content);
	content.clear();

	f();
	android::base::ReadFileToString("/proc/self/schedstat", &content);
	cout << content;
	ts2 = parse(content);
	content.clear();

	ioctl(fd, PERF_EVENT_IOC_DISABLE, 0);
	read(fd, &count, sizeof(long long));
	printf("%d: cpu time: %lld, cycle: %lld, freq: %d\n", getpid(), ts2 - ts1, count, FREQ);
}

int main()
{
	struct perf_event_attr pe;
	int fd;

	memset(&pe, 0, sizeof(struct perf_event_attr));
	pe.type = PERF_TYPE_HARDWARE;
	pe.size = sizeof(struct perf_event_attr);
	pe.config = PERF_COUNT_HW_CPU_CYCLES;
	pe.disabled = 1;
	pe.exclude_kernel = 1;
	pe.exclude_hv = 1;

	fd = perf_event_open(&pe, 0, -1, -1, 0);
	if (fd == -1) {
		fprintf(stderr, "Error opening leader %llx\n", pe.config);
		exit(EXIT_FAILURE);
	}

	loop(fd, test1);
//	loop(fd, test2);

	close(fd);
	return 0;
}
