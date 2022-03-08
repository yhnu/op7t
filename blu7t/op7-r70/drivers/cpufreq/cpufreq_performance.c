/*
 *  linux/drivers/cpufreq/cpufreq_performance.c
 *
 *  Copyright (C) 2002 - 2003 Dominik Brodowski <linux@brodo.de>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/oneplus/boot_mode.h>

#define CPUFREQ_INDEX 5
static void cpufreq_gov_performance_limits(struct cpufreq_policy *policy)
{
	unsigned int index = 0;
	unsigned int valid_freq;
	struct cpufreq_frequency_table *table, *pos;
	static unsigned int first_cpu = 1010;
	pr_debug("setting to %u kHz\n", policy->max);
	if (get_boot_mode() ==  MSM_BOOT_MODE__WLAN
		|| (get_boot_mode() ==  MSM_BOOT_MODE__RF)
		|| (get_boot_mode() ==  MSM_BOOT_MODE__FACTORY)) {
		if (first_cpu != cpumask_first(policy->related_cpus))
			first_cpu = cpumask_first(policy->related_cpus);
			table = policy->freq_table;
			if (!table) {
				pr_err("Failed to get freqtable\n");
			} else {
				for (pos = table; pos->frequency
					!= CPUFREQ_TABLE_END; pos++)
					index++;
				if (index > CPUFREQ_INDEX)
					index = index - CPUFREQ_INDEX;
				valid_freq = table[index].frequency;
				if (valid_freq)
					__cpufreq_driver_target(policy,
						valid_freq,
						CPUFREQ_RELATION_H);
				else
					__cpufreq_driver_target(policy,
						policy->max,
						CPUFREQ_RELATION_H);
			}
	} else
		__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
}

static struct cpufreq_governor cpufreq_gov_performance = {
	.name		= "performance",
	.owner		= THIS_MODULE,
	.limits		= cpufreq_gov_performance_limits,
};

static int __init cpufreq_gov_performance_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_performance);
}

static void __exit cpufreq_gov_performance_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_performance);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_PERFORMANCE
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &cpufreq_gov_performance;
}
#endif
#ifndef CONFIG_CPU_FREQ_GOV_PERFORMANCE_MODULE
struct cpufreq_governor *cpufreq_fallback_governor(void)
{
	return &cpufreq_gov_performance;
}
#endif

MODULE_AUTHOR("Dominik Brodowski <linux@brodo.de>");
MODULE_DESCRIPTION("CPUfreq policy governor 'performance'");
MODULE_LICENSE("GPL");

fs_initcall(cpufreq_gov_performance_init);
module_exit(cpufreq_gov_performance_exit);
