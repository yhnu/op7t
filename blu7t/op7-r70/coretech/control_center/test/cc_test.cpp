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
#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>

#include <gtest/gtest.h>

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
	int period_us;
	int prio;
	int group;
	int category;
	int type;
	u64 params[CC_CTL_PARAM_SIZE];
	u64 response;
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
	u64 param1, u64 param2, u64 param3, u64 param4, struct cc_command &cc)
{
	cc.group = cd_get_group();
	cc.category = ccc;
	cc.type = cct;
	cc.pid = cd_get_pid();
	cc.prio = 0;
	cc.period_us = 5000000;
	cc.params[0] = param1; /* optional */
	cc.params[1] = param2; /* optional */
	cc.params[2] = param3; /* optional */
	cc.params[3] = param4; /* optional */
	cc.response = 0;
	cc.status = 0;
}

static inline void cd_routine(enum CC_CTL_CATEGORY ccc, enum CC_CTL_TYPE cct,
	u64 param1, u64 param2, u64 param3, u64 param4)
{
	struct cc_command cc;
	int cc_ctl_fd = cd_get_fd();

	if (cc_ctl_fd < 0) {
		printf("control center not ready\n");
		return;
	}

	cd_cmd_setup(ccc, cct, param1, param2, param3, param4, cc);
	ioctl(cc_ctl_fd, CC_IOC_COMMAND, &cc);

	/* verify result */
	if (cc.status)
		printf("%s failed\n", __func__);
	else
		printf("%s successful\n", __func__);
}

void adjust(enum CC_CTL_CATEGORY ccc, enum CC_CTL_TYPE cct,
	u64 param1 = 0, u64 param2 = 0, u64 param3 = 0, u64 param4 = 0)
{
	cd_routine(ccc, cct, param1, param2, param3, param4);
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

/* test part */
bool valid_cpufreq(unsigned int target, int cluster) {
	string paths[] = {
		"/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq",
		"/sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq",
		"/sys/devices/system/cpu/cpu7/cpufreq/scaling_cur_freq"
	};
	ifstream ifs(paths[cluster].c_str());
	string result ((istreambuf_iterator<char>(ifs)), istreambuf_iterator<char>());
	cout << "Test cluster " << cluster << " set cpufreq to " << target << ", result " << stoi(result) << endl;
	return stoi(result) == (int)target;
}

bool valid_ddrfreq(long long target) {
	string path("/sys/kernel/debug/clk/measure_only_mccc_clk/clk_measure");
	ifstream ifs(path.c_str());
	string result ((istreambuf_iterator<char>(ifs)), istreambuf_iterator<char>());
	cout << "Test set ddrfreq to " << target << ", " << stoll(result) << endl;
	return stoll(result) == target;
}

TEST(cpufreq_test, test0) {
	int cluster_idx = 0;
	unsigned int target[] = {
		576000, 672000, 768000, 844800, 940800, 1036800, 1113600, 1209600, 1305600, 1382400, 1478400, 1555200, 1632000, 1708800, 1785600
	};
	cd_init(CC_CTL_GROUP_AI);

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_0_FREQ, CC_CTL_TYPE_ONESHOT, target[i], target[i]);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (int i = (int) sizeof(target)/sizeof(unsigned int) - 1; i >= 0; --i) {
		adjust(CC_CTL_CATEGORY_CLUS_0_FREQ, CC_CTL_TYPE_ONESHOT, target[i], target[i]);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_0_FREQ, CC_CTL_TYPE_ONESHOT, target[i], target[i]);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	adjust(CC_CTL_CATEGORY_CLUS_0_FREQ, CC_CTL_TYPE_RESET);
	cd_deinit();
}

TEST(cpufreq_test, test1) {
	int cluster_idx = 1;
	unsigned int target[] = {
		710400, 825600, 940800, 1056000, 1171200, 1286400, 1401600, 1497600, 1612800, 1708800, 1804800, 1920000, 2016000, 2131200, 2227200, 2323200, 2419200
	};
	cd_init(CC_CTL_GROUP_AI);

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_1_FREQ, CC_CTL_TYPE_ONESHOT, target[i], target[i]);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (int i = (int) sizeof(target)/sizeof(unsigned int) - 1; i >= 0; --i) {
		adjust(CC_CTL_CATEGORY_CLUS_1_FREQ, CC_CTL_TYPE_ONESHOT, target[i], target[i]);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_1_FREQ, CC_CTL_TYPE_ONESHOT, target[i], target[i]);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	adjust(CC_CTL_CATEGORY_CLUS_1_FREQ, CC_CTL_TYPE_RESET);
	cd_deinit();
}

TEST(cpufreq_test, test2) {
	int cluster_idx = 2;
	unsigned int target[] = {
		825600, 940800, 1056000, 1171200, 1286400, 1401600, 1497600, 1612800, 1708800, 1804800, 1920000, 2016000, 2131200, 2227200, 2323200, 2419200, 2534400, 2649600, 2745600, 2841600
	};
	cd_init(CC_CTL_GROUP_AI);

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_2_FREQ, CC_CTL_TYPE_ONESHOT, target[i], target[i]);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (int i = (int) sizeof(target)/sizeof(unsigned int) - 1; i >= 0; --i) {
		adjust(CC_CTL_CATEGORY_CLUS_2_FREQ, CC_CTL_TYPE_ONESHOT, target[i], target[i]);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_2_FREQ, CC_CTL_TYPE_ONESHOT, target[i], target[i]);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	adjust(CC_CTL_CATEGORY_CLUS_2_FREQ, CC_CTL_TYPE_RESET);
	cd_deinit();
}

TEST(ddrfreq_test, test) {
	unsigned int target[] = {
		200, 300, 451, 547, 681, 768, 1017, 1353, 1555, 1804, 2092
	};
	long long expected[] = {
		200000000LL, 300030003LL, 451263537LL, 547345374LL, 681663258LL, 768049155LL, 1018329938LL, 1355013550LL, 1555209953LL, 1805054151LL, 2096436058LL
	};
	cd_init(CC_CTL_GROUP_AI);

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_DDR_FREQ, CC_CTL_TYPE_ONESHOT, target[i]);
		ASSERT_TRUE(valid_ddrfreq(expected[i]));
	}

	for (int i = (int) sizeof(target)/sizeof(unsigned int) - 1; i >= 0; --i) {
		adjust(CC_CTL_CATEGORY_DDR_FREQ, CC_CTL_TYPE_ONESHOT, target[i]);
		ASSERT_TRUE(valid_ddrfreq(expected[i]));
	}

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_DDR_FREQ, CC_CTL_TYPE_ONESHOT, target[i]);
		ASSERT_TRUE(valid_ddrfreq(expected[i]));
	}

	adjust(CC_CTL_CATEGORY_DDR_FREQ, CC_CTL_TYPE_RESET);
	cd_deinit();
}

TEST(cpufreq_test_nb, test0) {
	int cluster_idx = 0;
	unsigned int target[] = {
		576000, 672000, 768000, 844800, 940800, 1036800, 1113600, 1209600, 1305600, 1382400, 1478400, 1555200, 1632000, 1708800, 1785600
	};
	cd_init(CC_CTL_GROUP_AI);

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_0_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i], target[i]);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (int i = (int) sizeof(target)/sizeof(unsigned int) - 1; i >= 0; --i) {
		adjust(CC_CTL_CATEGORY_CLUS_0_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i], target[i]);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_0_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i], target[i]);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	adjust(CC_CTL_CATEGORY_CLUS_0_FREQ, CC_CTL_TYPE_RESET_NONBLOCK, cluster_idx);
	sleep(1);
	cd_deinit();
}

TEST(cpufreq_test_nb, test1) {
	int cluster_idx = 1;
	unsigned int target[] = {
		710400, 825600, 940800, 1056000, 1171200, 1286400, 1401600, 1497600, 1612800, 1708800, 1804800, 1920000, 2016000, 2131200, 2227200, 2323200, 2419200
	};
	cd_init(CC_CTL_GROUP_AI);

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_1_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i], target[i]);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (int i = (int) sizeof(target)/sizeof(unsigned int) - 1; i >= 0; --i) {
		adjust(CC_CTL_CATEGORY_CLUS_1_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i], target[i]);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_1_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i], target[i]);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	adjust(CC_CTL_CATEGORY_CLUS_1_FREQ, CC_CTL_TYPE_RESET_NONBLOCK, cluster_idx);
	sleep(1);
	cd_deinit();
}

TEST(cpufreq_test_nb, test2) {
	int cluster_idx = 2;
	unsigned int target[] = {
		825600, 940800, 1056000, 1171200, 1286400, 1401600, 1497600, 1612800, 1708800, 1804800, 1920000, 2016000, 2131200, 2227200, 2323200, 2419200, 2534400, 2649600, 2745600, 2841600
	};
	cd_init(CC_CTL_GROUP_AI);

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_2_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i], target[i]);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (int i = (int) sizeof(target)/sizeof(unsigned int) - 1; i >= 0; --i) {
		adjust(CC_CTL_CATEGORY_CLUS_2_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i], target[i]);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_CLUS_2_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i], target[i]);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(target[i], cluster_idx));
	}

	adjust(CC_CTL_CATEGORY_CLUS_2_FREQ, CC_CTL_TYPE_RESET_NONBLOCK, cluster_idx);
	sleep(1);
	cd_deinit();
}

TEST(ddrfreq_test_nb, test) {
	unsigned int target[] = {
		200, 300, 451, 547, 681, 768, 1017, 1353, 1555, 1804, 2092
	};
	long long expected[] = {
		200000000LL, 300030003LL, 451263537LL, 547345374LL, 681663258LL, 768049155LL, 1018329938LL, 1355013550LL, 1555209953LL, 1805054151LL, 2096436058LL
	};
	cd_init(CC_CTL_GROUP_AI);

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_DDR_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i]);
		sleep(1);
		ASSERT_TRUE(valid_ddrfreq(expected[i]));
	}

	for (int i = (int) sizeof(target)/sizeof(unsigned int) - 1; i >= 0; --i) {
		adjust(CC_CTL_CATEGORY_DDR_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i]);
		sleep(1);
		ASSERT_TRUE(valid_ddrfreq(expected[i]));
	}

	for (unsigned int i = 0; i < sizeof(target)/sizeof(unsigned int); ++i) {
		adjust(CC_CTL_CATEGORY_DDR_FREQ, CC_CTL_TYPE_ONESHOT_NONBLOCK, target[i]);
		sleep(1);
		ASSERT_TRUE(valid_ddrfreq(expected[i]));
	}

	adjust(CC_CTL_CATEGORY_DDR_FREQ, CC_CTL_TYPE_RESET_NONBLOCK);
	sleep(1);
	cd_deinit();
}

TEST(sched_prime_boost_test, test) {
	cd_init(CC_CTL_GROUP_AI);
	adjust(CC_CTL_CATEGORY_SCHED_PRIME_BOOST, CC_CTL_TYPE_ONESHOT, getpid());
	adjust(CC_CTL_CATEGORY_SCHED_PRIME_BOOST, CC_CTL_TYPE_RESET);
	cd_deinit();
}

TEST(sched_prime_boost_test_nb, test) {
	cd_init(CC_CTL_GROUP_AI);
	adjust(CC_CTL_CATEGORY_SCHED_PRIME_BOOST, CC_CTL_TYPE_ONESHOT_NONBLOCK, getpid());
	adjust(CC_CTL_CATEGORY_SCHED_PRIME_BOOST, CC_CTL_TYPE_RESET_NONBLOCK);
	cd_deinit();
}

TEST(cpufreq_boost_boost_test, test) {
	cd_init(CC_CTL_GROUP_AI);
	adjust(CC_CTL_CATEGORY_CPU_FREQ_BOOST, CC_CTL_TYPE_ONESHOT, 1, 1);
	adjust(CC_CTL_CATEGORY_CPU_FREQ_BOOST, CC_CTL_TYPE_RESET);
	cd_deinit();
}

TEST(cpufreq_boost_boost_test_nb, test) {
	cd_init(CC_CTL_GROUP_AI);
	adjust(CC_CTL_CATEGORY_CPU_FREQ_BOOST, CC_CTL_TYPE_ONESHOT_NONBLOCK, 1, 1);
	adjust(CC_CTL_CATEGORY_CPU_FREQ_BOOST, CC_CTL_TYPE_RESET_NONBLOCK);
	cd_deinit();
}

TEST(query, test) {
	cd_init(CC_CTL_GROUP_AI);
	adjust(CC_CTL_CATEGORY_CLUS_0_FREQ_QUERY, CC_CTL_TYPE_ONESHOT);
	adjust(CC_CTL_CATEGORY_CLUS_1_FREQ_QUERY, CC_CTL_TYPE_ONESHOT);
	adjust(CC_CTL_CATEGORY_CLUS_2_FREQ_QUERY, CC_CTL_TYPE_ONESHOT);
	adjust(CC_CTL_CATEGORY_DDR_FREQ_QUERY, CC_CTL_TYPE_ONESHOT);
	cd_deinit();
}

TEST(cpufreq_boundry_test, test_boundry_0) {
	int cluster_idx = 0;
	unsigned int target_max = UINT_MAX;
	unsigned int target_min = 0;
	unsigned int expected_max[3] = { 1785600, 2419200, 2841600 };
	unsigned int expected_min[3] = { 576000, 710400, 825600 };

	cd_init(CC_CTL_GROUP_AI);

	for (cluster_idx = 0; cluster_idx < 3; ++cluster_idx) {
		enum CC_CTL_CATEGORY ccc = CC_CTL_CATEGORY_CLUS_0_FREQ;
		if (cluster_idx == 1)
			ccc = CC_CTL_CATEGORY_CLUS_1_FREQ;
		else if (cluster_idx == 2)
			ccc = CC_CTL_CATEGORY_CLUS_2_FREQ;

		adjust(ccc, CC_CTL_TYPE_ONESHOT_NONBLOCK, target_max, target_max);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(expected_max[cluster_idx], cluster_idx));
		adjust(ccc, CC_CTL_TYPE_RESET_NONBLOCK, cluster_idx);
	}

	for (cluster_idx = 0; cluster_idx < 3; ++cluster_idx) {
		enum CC_CTL_CATEGORY ccc = CC_CTL_CATEGORY_CLUS_0_FREQ;
		if (cluster_idx == 1)
			ccc = CC_CTL_CATEGORY_CLUS_1_FREQ;
		else if (cluster_idx == 2)
			ccc = CC_CTL_CATEGORY_CLUS_2_FREQ;

		adjust(ccc, CC_CTL_TYPE_ONESHOT_NONBLOCK, target_min, target_min);
		sleep(1);
		ASSERT_TRUE(valid_cpufreq(expected_min[cluster_idx], cluster_idx));
		adjust(ccc, CC_CTL_TYPE_RESET_NONBLOCK, cluster_idx);
	}

	cd_deinit();
}
