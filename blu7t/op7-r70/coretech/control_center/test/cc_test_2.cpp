#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <streambuf>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <asm/unistd.h>
#include <string>
#include <unistd.h>
#include <time.h>
#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>

using namespace std;

typedef unsigned int u32;
typedef unsigned long long u64;

#define CC_CTL_PARAM_SIZE 4

enum CC_CTL_GROUP {
	CC_CTL_GROUP_DEFAULT,
	CC_CTL_GROUP_AI,
	CC_CTL_GROUP_GRAPHIC,
	CC_CTL_GROUP_FRAMEWORK,
	CC_CTL_GROUP_SYSTEM,
	CC_CTL_GROUP_OTHERS,

	CC_CTL_GROUP_MAX
};

enum CC_CTL_CATEGORY {
	CC_CTL_CATEGORY_CLUS_0_FREQ,
	CC_CTL_CATEGORY_CLUS_1_FREQ,
	CC_CTL_CATEGORY_CLUS_2_FREQ,
	CC_CTL_CATEGORY_CPU_FREQ_BOOST,
	CC_CTL_CATEGORY_DDR_FREQ,
	CC_CTL_CATEGORY_SCHED_PRIME_BOOST,

	CC_CTL_CATEGORY_CLUS_0_FREQ_QUERY,
	CC_CTL_CATEGORY_CLUS_1_FREQ_QUERY,
	CC_CTL_CATEGORY_CLUS_2_FREQ_QUERY,
	CC_CTL_CATEGORY_DDR_FREQ_QUERY,

	CC_CTL_CATEGORY_MAX
};

enum CC_CTL_TYPE {
	CC_CTL_TYPE_ONESHOT,
	CC_CTL_TYPE_PERIOD,
	CC_CTL_TYPE_RESET,

	/* NONBLOCK region, the order is matter!! */
	CC_CTL_TYPE_ONESHOT_NONBLOCK,
	CC_CTL_TYPE_PERIOD_NONBLOCK,
	CC_CTL_TYPE_RESET_NONBLOCK,

	CC_CTL_TYPE_MAX
};

struct cc_command {
	pid_t pid;
	pid_t leader;
	int period_us;
	int prio;
	int group;
	int category;
	int type;
	u64 params[CC_CTL_PARAM_SIZE];
	u64 response;
	bool bind_leader;
	int status;
};

#define CC_CTL "/dev/cc_ctl"
#define CC_IOC_MAGIC 'c'
#define CC_IOC_COMMAND _IOWR(CC_IOC_MAGIC, 0, struct cc_command)
#define CC_IOC_MAX 1

int& cd_get_group()
{
	static int group = -1;
	return group;
}

void cd_set_group(enum CC_CTL_GROUP ccg)
{
	int &group = cd_get_group();
	group = ccg;
}

int cd_get_pid()
{
	static pid_t pid = 0;
	if (pid == 0)
		pid = getpid();
	return pid;
}

/* return reference for update purpose */
int& cd_get_fd()
{
	/* one time open, to avoid open too much times */
	static int cc_ctl_fd = 0;
	if (cc_ctl_fd == 0)
		cc_ctl_fd = open("/dev/cc_ctl", O_WRONLY);
	return cc_ctl_fd;
}

void cd_release()
{
	int& cc_ctl_fd = cd_get_fd();
	if (cc_ctl_fd > 0)
		close(cc_ctl_fd);
	cc_ctl_fd = 0;
}

static inline void cd_cmd_setup(enum CC_CTL_CATEGORY ccc, enum CC_CTL_TYPE cct,
	int delay_us, u64 param1, u64 param2, u64 param3, u64 param4, struct cc_command &cc)
{
	cc.group = cd_get_group();
	cc.category = ccc;
	cc.type = cct;
	cc.pid = cd_get_pid();
	cc.period_us = delay_us;
	cc.prio = rand() % 3; // 0 ~ 2
	cc.params[0] = param1; /* optional */
	cc.params[1] = param2; /* optional */
	cc.params[2] = param3; /* optional */
	cc.params[3] = param4; /* optional */
	cc.response = 0;
	cc.status = 0;
}

static inline void cd_routine(enum CC_CTL_CATEGORY ccc, enum CC_CTL_TYPE cct,
	int delay_us, u64 param1, u64 param2, u64 param3, u64 param4)
{
	struct cc_command cc;
	int cc_ctl_fd = cd_get_fd();

	if (cc_ctl_fd < 0) {
		printf("control center not ready\n");
		return;
	}

	cd_cmd_setup(ccc, cct, delay_us, param1, param2, param3, param4, cc);
	ioctl(cc_ctl_fd, CC_IOC_COMMAND, &cc);

	/* verify result */
	if (cc.status)
		printf("%s failed\n", __func__);
	else
		printf("%s successful\n", __func__);
}

void adjust(enum CC_CTL_CATEGORY ccc, enum CC_CTL_TYPE cct,
	int delay_us, u64 param1 = 0, u64 param2 = 0, u64 param3 = 0, u64 param4 = 0)
{
	cd_routine(ccc, cct, delay_us, param1, param2, param3, param4);
}

void cd_init(enum CC_CTL_GROUP ccg)
{
	printf("control daemon init\n");
	cd_set_group(ccg);
	if (cd_get_fd() < 0)
		printf("open control center interface failed\n");
}

void cd_deinit()
{
	cd_release();
	printf("control daemon closed\n");
}

int main(int, char**) {
	enum CC_CTL_CATEGORY c = CC_CTL_CATEGORY_DDR_FREQ;
	int s, p, t;
	int target0[] = {576000, 672000, 768000, 844800, 940800, 1036800, 1113600, 1209600, 1305600, 1382400, 1478400, 1555200, 1632000, 1708800, 1785600};
	int target1[] = {710400, 825600, 940800, 1056000, 1171200, 1286400, 1401600, 1497600, 1612800, 1708800, 1804800, 1920000, 2016000, 2131200, 2227200, 2323200, 2419200};
	int target2[] = {825600, 940800, 1056000, 1171200, 1286400, 1401600, 1497600, 1612800, 1708800, 1804800, 1920000, 2016000, 2131200, 2227200, 2323200, 2419200, 2534400, 2649600, 2745600, 2841600};
	int target3[] = {
		200, 300, 451, 547, 681, 768, 1017, 1353, 1555, 1804, 2092
	};
	int* target = target3;
	bool b;

	srand(time(NULL));

	/* init */
	cd_init(CC_CTL_GROUP_AI);
	while (1) {
		int cc = rand() % 4;
		switch (cc) {
		case 0: c = CC_CTL_CATEGORY_CLUS_0_FREQ; target = target0; break;
		case 1: c = CC_CTL_CATEGORY_CLUS_1_FREQ; target = target1; break;
		case 2: c = CC_CTL_CATEGORY_CLUS_2_FREQ; target = target2; break;
		case 3: c = CC_CTL_CATEGORY_DDR_FREQ; target = target3; break;
		}
		s = rand() % 15000 + 5000; // 5000 ~ 19999 us
		p = rand() % 15000 + 5000; // 5000 ~ 19999 us
		t = rand() % 11;
		b = rand() % 2; // 0 ~ 1

		cout << "s: " << s << ", p: " << p << ", target: " << target[t] << ", block: " << b << endl;

		/* adjust */
		adjust(c, (b? CC_CTL_TYPE_ONESHOT: CC_CTL_TYPE_ONESHOT_NONBLOCK), p /* delay us */, target[t] /* min freq */, target[t] /* max freq */);

		usleep (s);

		/* reset */
		adjust(c, (b? CC_CTL_TYPE_RESET: CC_CTL_TYPE_RESET_NONBLOCK), 0 /* delay us */);
	}
	/* deinit */
	cd_deinit();
	return 0;
}
