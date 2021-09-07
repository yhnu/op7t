#include <iostream>
#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/perf_event.h>
#include <linux/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <asm/unistd.h>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>

using namespace std;

typedef unsigned int u32;
typedef unsigned long long u64;
typedef long long s64;

#define HT_IOC_MAGIC 'k'
#define HT_IOC_COLLECT _IOR(HT_IOC_MAGIC, 0, struct ai_parcel)
#define HT_IOC_SCHEDSTAT _IOWR(HT_IOC_MAGIC, 1, int)
#define HT_IOC_MAX 2

#define AI_THREAD_PARCEL_MAX (10)

struct ai_thread_parcel {
	u32 tid;
	u64 exec_time; // schedstat
	u64 inst; // pmu related
	u64 cycle;
	u64 cache_miss_l1;
	u64 cache_miss_l2;
	u64 cache_miss_l3;
};

struct ai_parcel {
	u32 pid;
	u32 fps;
	u32 efps;
	char layer[64];
	u32 cpu_cur_freq_0;
	u32 cpu_cur_freq_1;
	u32 cpu_cur_freq_2;
	u32 gpu_freq;
	u64 ddr_freq;
	u32 ddr_bw;
	u32 volt_now; // battery part
	u32 curr_now;
	u64 queued_ts;
	u64 prev_queued_ts;
	u32 thread_amount; // default maximum 10 threads
	u32 boost_cnt;
	u64 boost_ts_us;
	u64 boost_end_ts_us;
	u64 data_collect_ts_us;
	u64 data_collect_end_ts_us;
	u64 utils[8];
	struct ai_thread_parcel t[AI_THREAD_PARCEL_MAX];
};
// sysnode /dev/ht_ctl (temporary)

#define HT_AI_IOC_PATH "/dev/ht_ctl"

void dump_parcel_thread(struct ai_thread_parcel &t, int idx) {
		cout << "t[" << idx << "]: tid: " << t.tid << endl;
		cout << "t[" << idx << "]: exec_time: " << t.exec_time << endl;
		cout << "t[" << idx << "]: inst: " << t.inst << endl;
		cout << "t[" << idx << "]: cycle: " << t.cycle << endl;
		cout << "t[" << idx << "]: cache_miss_l1: " << t.cache_miss_l1 << endl;
		cout << "t[" << idx << "]: cache_miss_l2: " << t.cache_miss_l2 << endl;
		cout << "t[" << idx << "]: cache_miss_l2: " << t.cache_miss_l3 << endl;
}

void dump_parcel(struct ai_parcel& p) {
	cout << "pid: " << p.pid << endl;
	cout << "fps: " << p.fps << endl;
	cout << "efps: " << p.efps << endl;
	cout << "layer: " << p.layer << endl;
	cout << "cpu_cur_freq_0: " << p.cpu_cur_freq_0 << endl;
	cout << "cpu_cur_freq_1: " << p.cpu_cur_freq_1 << endl;
	cout << "cpu_cur_freq_2: " << p.cpu_cur_freq_2 << endl;
	cout << "gpu_freq: " << p.gpu_freq << endl;
	cout << "ddr_freq: " << p.ddr_freq << endl;
	cout << "ddr_bw: " << p.ddr_bw << endl;
	cout << "volt_now: " << p.volt_now << endl;
	cout << "curr_now: " << p.curr_now << endl;
	cout << "queued_ts: " << p.queued_ts << endl;
	cout << "thread_amount: " << p.thread_amount << endl;
	cout << "boost_cnt: " << p.boost_cnt << endl;
	for (u32 i = 0; i < p.thread_amount; ++i)
		dump_parcel_thread(p.t[i], i);
}

int main(int, char**) {
	int fd;
	struct ai_parcel parcel;

	fd = open(HT_AI_IOC_PATH, O_WRONLY);
	if (fd == -1)
		return -1;

	ioctl(fd, HT_IOC_COLLECT, &parcel);

	/* parse data */
	dump_parcel(parcel);

	close(fd);
	return 0;
}
