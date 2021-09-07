#ifndef __INCLUDE_HOUSTON__
#define __INCLUDE_HOUSTON__

#ifndef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#endif

#include <linux/sched.h>
#include <linux/thermal.h>
#include <linux/power_supply.h>

#ifdef CONFIG_CONTROL_CENTER
#include <linux/oem/control_center.h>
#endif

#define HT_CLUSTERS 3
#define HT_CPUS_PER_CLUS 4
#define CLUS_0_IDX 0
#define CLUS_1_IDX 4
#define CLUS_2_IDX 7

#define MIN_POLLING_VAL 5
#define MAX_REPORT_PERIOD 18000

#define HT_CTL_NODE "ht_ctl"
#define HT_TAG "ht_monitor: "
#define ht_logv(fmt...) \
	do { \
		if (ht_log_lv < 1) \
			pr_info(HT_TAG fmt); \
	} while (0)

#define ht_logi(fmt...) \
	do { \
		if (ht_log_lv < 2) \
			pr_info(HT_TAG fmt); \
	} while (0)

#define ht_logw(fmt...) \
	do { \
		if (ht_log_lv < 3) \
			pr_warn(HT_TAG fmt); \
	} while (0)

#define ht_loge(fmt...) pr_err(HT_TAG fmt)
#define ht_logd(fmt...) pr_debug(HT_TAG fmt)

#define FPS_COLS (9)
#define FPS_LAYER_LEN (128)
#define FPS_PROCESS_NAME_LEN (64)
#define FPS_DATA_BUF_SIZE (256)
#define FPS_CACHE_LAYER_LEN (64)
#define FPS_TARGET_NUM (2)

enum boost_config {
	FRAME_BOOST = 1,
	CPU_BOOST,

	BOOST_MAX
};

enum boost_lv {
	BOOST_LV_0 = 0,
	BOOST_LV_1,
	BOOST_LV_2,
	BOOST_LV_3,
	BOOST_LV_4,

	BOOST_LV_MAX
};

/* customize your monitor */
enum {
	HT_TS,
	HT_CLUS_FREQ_0_MIN,
	HT_CLUS_FREQ_0_CUR,
	HT_CLUS_FREQ_0_MAX,
	HT_ISO_0,
	HT_CLUS_FREQ_1_MIN,
	HT_CLUS_FREQ_1_CUR,
	HT_CLUS_FREQ_1_MAX,
	HT_ISO_1,
	HT_CLUS_FREQ_2_MIN,
	HT_CLUS_FREQ_2_CUR,
	HT_CLUS_FREQ_2_MAX,
	HT_ISO_2,
	HT_GPU_FREQ,
	HT_BAT_VOLT_NOW,
	HT_BAT_CURR_NOW,
	HT_HW_INSTRUCTION,
	HT_HW_CACHE_MISS,
	HT_HW_CYCLE,
	HT_CPU_0,
	HT_CPU_1,
	HT_CPU_2,
	HT_CPU_3,
	HT_CPU_4_0,
	HT_CPU_4_1,
	HT_CPU_5_0,
	HT_CPU_5_1,
	HT_CPU_6_0,
	HT_CPU_6_1,
	HT_CPU_7_0,
	HT_CPU_7_1,
	HT_THERM_0,
	HT_THERM_1,
	HT_UTIL_0,
	HT_UTIL_1,
	HT_UTIL_2,
	HT_UTIL_3,
	HT_UTIL_4,
	HT_UTIL_5,
	HT_UTIL_6,
	HT_UTIL_7,
	HT_FPS_PROCESS,
	HT_FPS_LAYER,
	HT_FPS_PID,
	HT_FPS_ALIGN,
	HT_FPS_1,
	HT_FPS_2,
	HT_FPS_3,
	HT_FPS_4,
	HT_FPS_5,
	HT_FPS_6,
	HT_FPS_7,
	HT_FPS_8,
	HT_FPS_MISS_LAYER,
	HT_RENDER_PID,
	HT_RENDER_UTIL,
	HT_NT_RTG,
	HT_RTG_UTIL_SUM,
	HT_MONITOR_SIZE
};

/* pick one unique magic number */
#define HT_IOC_MAGIC 'k'
#define HT_IOC_COLLECT _IOR(HT_IOC_MAGIC, 0, struct ai_parcel)
#define HT_IOC_SCHEDSTAT _IOWR(HT_IOC_MAGIC, 1, u64)
#define HT_IOC_CPU_LOAD _IOWR(HT_IOC_MAGIC, 2, struct cpuload)
#define HT_IOC_MAX 2

#define AI_THREAD_PARCEL_MAX (10)

struct ai_thread_parcel {
	u32 tid;
	u64 exec_time_ns; // schedstat
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
	long long fps_align_ns;
	u32 cpu;
	u32 clus;
	//char layer[FPS_CACHE_LAYER_LEN];
	u32 cpu_cur_freq_0;
	u32 cpu_cur_freq_1;
	u32 cpu_cur_freq_2;
	u32 cpu_orig_freq_0;
	u32 cpu_orig_freq_1;
	u32 cpu_orig_freq_2;
	u32 cpu_cc_min_freq_1;
	u32 cpu_cc_max_freq_1;
	u32 cpu_orig_min_freq_1;
	u32 cpu_orig_max_freq_1;
	u32 gpu_freq;
	u64 ddr_freq;
	u32 ddr_bw;
	u32 volt_now; // battery part
	u32 curr_now;
	u64 queued_ts_us;
	u64 prev_queued_ts_us;
	u32 thread_amount; // default maximum 10 threads
	u32 boost_cnt;
	u64 notify_start_ts_us;
	u64 notify_end_ts_us;
	u64 utils[8];
	u32 skin_temp;
#ifdef CONFIG_CONTROL_CENTER
	struct cc_boost_ts cbt[CC_BOOST_TS_SIZE];
#endif
	struct ai_thread_parcel t[AI_THREAD_PARCEL_MAX];
};

/* cpu load info */
struct cpuload {
	union {
		int min;
		int pid;
	};
	union {
		int max;
		int tid;
	};
	int avg;
	int sum;
	long long iowait_min;
	long long iowait_max;
	long long iowait_avg;
	long long iowait_sum;
};
#endif // __INCLUDE_HOUSTON__
