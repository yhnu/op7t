#ifndef __CONTROL_CENTER_INC__
#define __CONTROL_CENTER_INC__

#define CC_CTL_VERSION "2.0"

#ifndef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#endif

#define CLUSTER_NUM (3)

#define CC_CTL_NODE "cc_ctl"
#define CC_TAG "control center: "
#define cc_logv(fmt...) \
	do { \
		if (cc_log_lv < 1) \
			pr_info(CC_TAG fmt); \
	} while (0)

#define cc_logi(fmt...) \
	do { \
		if (cc_log_lv < 2) \
			pr_info(CC_TAG fmt); \
	} while (0)

#define cc_logw(fmt...) \
	do { \
		if (cc_log_lv < 3) \
			pr_warn(CC_TAG fmt); \
	} while (0)

/* special for ratelimited version */
#define cc_logw_ratelimited(fmt...) \
	do { \
		if (cc_log_lv < 3) \
			pr_warn_ratelimited(CC_TAG fmt); \
	} while (0)

#define cc_loge(fmt...) pr_err(CC_TAG fmt)
#define cc_logd(fmt...) pr_debug(CC_TAG fmt)

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
	CC_CTL_CATEGORY_DDR_VOTING_FREQ,
	CC_CTL_CATEGORY_DDR_LOCK_FREQ,
	CC_CTL_CATEGORY_SCHED_PRIME_BOOST,

	/* TODO move out from control part */
	/* query */
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

	/*
	 * NONBLOCK version.
	 * Always paired with the original one!
	 */
	CC_CTL_TYPE_ONESHOT_NONBLOCK,
	CC_CTL_TYPE_PERIOD_NONBLOCK,
	CC_CTL_TYPE_RESET_NONBLOCK,

	CC_CTL_TYPE_MAX
};

/* TODO consider display off case */
enum CC_PRIO {
	CC_PRIO_HIGH,
	CC_PRIO_MED,
	CC_PRIO_LOW,

	CC_PRIO_MAX // not priority max, TODO rename later
};

#define CC_IOC_MAGIC 'c'
#define CC_IOC_COMMAND _IOWR(CC_IOC_MAGIC, 0, struct cc_command)
#define CC_IOC_MAX 1

extern void cc_process(struct cc_command* cc);
extern void cc_tsk_process(struct cc_command* cc);
extern void cc_boost_ts_collect(struct cc_boost_ts* source);
#endif
