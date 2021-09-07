// SPDX-License-Identifier: GPL-2.0
/*
 * KLapse - A simple dynamic RGB lapsing module
 *
 * Copyright (C) 2019 Tanish Manku <tanish2k09.dev@gmail.com>.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/klapse.h>

/* Tunables */
static unsigned short enabled = DEFAULT_ENABLE;
static unsigned short daytime_r = MAX_SCALE;
static unsigned short daytime_g = MAX_SCALE;
static unsigned short daytime_b = MAX_SCALE;
static unsigned short target_r = MAX_SCALE;
static unsigned short target_g = MAX_SCALE * 79 / 100;
static unsigned short target_b = MAX_SCALE * 59 / 100;
static unsigned int start_minute = 1050;
static unsigned int stop_minute = 450;
static unsigned short b_cache = 100;
static unsigned short dimmer = 100;
static unsigned int bl_lower = LOWER_BL_LVL;
static unsigned int bl_upper = UPPER_BL_LVL;
static unsigned int pulse_freq = 30000;
static unsigned int fadeback_min = 90;
static unsigned int target_minute = 300;
static bool dimmer_auto = 0;
static unsigned int dimmer_start_minute = 1380;
static unsigned int dimmer_stop_minute = 360;
static unsigned int flow_freq = DEFAULT_FLOW_FREQ;

/* Core */
static void pulse(unsigned long data);
static void set_rgb_flow(unsigned long data);

static unsigned short current_r = MAX_SCALE;
static unsigned short current_g = MAX_SCALE;
static unsigned short current_b = MAX_SCALE;
static unsigned short flow_r = MAX_SCALE;
static unsigned short flow_g = MAX_SCALE;
static unsigned short flow_b = MAX_SCALE;
static unsigned int active_minutes;
static unsigned int last_bl = MAX_BRIGHTNESS;
static unsigned long local_time;
static struct rtc_time tm;
static struct timeval time;
static DEFINE_TIMER(pulse_timer, pulse, 0, 0);
static DEFINE_TIMER(flow_timer, set_rgb_flow, 0, 0);

static void restart_timer(void)
{
	mod_timer(&pulse_timer, jiffies + msecs_to_jiffies(pulse_freq));
}

static void flush_timer(void)
{
	if (timer_pending(&pulse_timer))
		mod_timer_pending(&pulse_timer, jiffies);
}

static void flush_flow_timer(void)
{
	if (timer_pending(&flow_timer))
		mod_timer_pending(&flow_timer, jiffies);
}

static void calc_active_minutes(void)
{
	bool isPulse = enabled == 1 || dimmer_auto == 1;

	if (isPulse)
		flush_timer();

	if(start_minute > stop_minute)
		active_minutes = (24 * 60) + stop_minute - start_minute;
	else
		active_minutes = stop_minute - start_minute;

	if (fadeback_min > active_minutes)
		fadeback_min = active_minutes;

	if (isPulse)
		pulse(0);
}

static int get_minutes_since_start(void)
{
	int hour;
	hour = tm.tm_hour - (start_minute / 60);

	if (hour < 0)
		hour += 24;

	return hour * 60 + tm.tm_min - (start_minute % 60);
}

static int get_minutes_before_stop(void)
{
	return active_minutes - get_minutes_since_start();
}

static void set_rgb(int r, int g, int b)
{
#if KLAPSE_MDSS
	klapse_kcal_push(r,g,b);
#else
	K_RED = r;
	K_GREEN = g;
	K_BLUE = b;
#endif
}

static void set_rgb_brightness(int r,int g,int b)
{
	r = r * dimmer / 100;
	g = g * dimmer / 100;
	b = b * dimmer / 100;

	r = clamp_t(int, r, MIN_SCALE, MAX_SCALE);
	g = clamp_t(int, g, MIN_SCALE, MAX_SCALE);
	b = clamp_t(int, b, MIN_SCALE, MAX_SCALE);

	set_rgb(r,g,b);
}

static void fetch_full_rgb(void)
{
	current_r = K_RED * 100 / dimmer;
	current_g = K_GREEN * 100 / dimmer;
	current_b = K_BLUE * 100 / dimmer;
}

static bool mins_in_range(unsigned int start, unsigned int stop, unsigned int check)
{
	/* Handle 24-hour ranges */
	if (start < stop)
		return check >= start && check < stop;
	else if (start == stop)
		return false;
	else
		return check < stop || check >= start;
}

static void limit_current_rgb(void)
{
	current_r = clamp_t(typeof(current_r), current_r, MIN_SCALE, MAX_SCALE);
	current_g = clamp_t(typeof(current_g), current_g, MIN_SCALE, MAX_SCALE);
	current_b = clamp_t(typeof(current_b), current_b, MIN_SCALE, MAX_SCALE);
}

static void limit_flow_rgb(void)
{
	flow_r = clamp_t(typeof(flow_r), flow_r, MIN_SCALE, MAX_SCALE);
	flow_g = clamp_t(typeof(flow_g), flow_g, MIN_SCALE, MAX_SCALE);
	flow_b = clamp_t(typeof(flow_b), flow_b, MIN_SCALE, MAX_SCALE);
}

static unsigned short calc_time_fwd_offset(int daytime, int target)
{
	daytime -= (daytime - target) * get_minutes_since_start() / target_minute;
	return daytime;
}

static void time_scale_fwd_rgb(void)
{
	current_r = calc_time_fwd_offset(daytime_r, target_r);
	current_g = calc_time_fwd_offset(daytime_g, target_g);
	current_b = calc_time_fwd_offset(daytime_b, target_b);

	limit_current_rgb();
}

static unsigned short calc_time_rev_offset(int backtime, int daytime, int target)
{
	target += (daytime - target) * (fadeback_min - backtime) / fadeback_min;
	return target;
}

static void time_scale_rev_rgb(int backtime)
{
	current_r = calc_time_rev_offset(backtime, daytime_r, target_r);
	current_g = calc_time_rev_offset(backtime, daytime_g, target_g);
	current_b = calc_time_rev_offset(backtime, daytime_b, target_b);

	limit_current_rgb();
}

static unsigned short calc_bl_scale_offset(bl_type_t bl_lvl, int daytime, int target)
{
	daytime -= (daytime - target) * (int) (bl_upper - bl_lvl) / (int) (bl_upper - bl_lower);
	return daytime;
}

static void bl_scale_rgb(bl_type_t bl_lvl)
{
	flow_r = calc_bl_scale_offset(bl_lvl, daytime_r, target_r);
	flow_g = calc_bl_scale_offset(bl_lvl, daytime_g, target_g);
	flow_b = calc_bl_scale_offset(bl_lvl, daytime_b, target_b);

	limit_flow_rgb();
}

static void set_timed_dimmer(void)
{
	if (dimmer_auto == 1 &&
		!mins_in_range(dimmer_start_minute,
				dimmer_stop_minute,
				tm.tm_hour * 60 + tm.tm_min))
		dimmer = 100;
	else
		dimmer = b_cache;
}

static void pulse(unsigned long data)
{
	int backtime;

	// Get time
	do_gettimeofday(&time);
	local_time = (u32)(time.tv_sec - (sys_tz.tz_minuteswest * 60));
	rtc_time_to_tm(local_time, &tm);

	set_timed_dimmer();

	if (enabled == 1) {
		backtime = get_minutes_before_stop();

		if(!mins_in_range(start_minute, stop_minute, tm.tm_hour * 60 + tm.tm_min)) {
			set_rgb_brightness(daytime_r,daytime_g,daytime_b);
			if (!timer_pending(&pulse_timer))
				restart_timer();
			return;
		} else if (backtime > fadeback_min) {
			backtime = get_minutes_since_start();

			if (backtime >= target_minute) {
				current_r = target_r;
				current_g = target_g;
				current_b = target_b;
			}
			else {
				time_scale_fwd_rgb();
			}
		} else {
			time_scale_rev_rgb(backtime);
		}
	}

	set_rgb_brightness(current_r, current_g, current_b);

	if (!timer_pending(&pulse_timer))
		restart_timer();
}

static void step_current_rgb_to_flow(void)
{
	if (current_r < flow_r)
		++current_r;
	else if (current_r > flow_r)
		--current_r;

	if (current_g < flow_g)
		++current_g;
	else if (current_g > flow_g)
		--current_g;

	if (current_b < flow_b)
		++current_b;
	else if (current_b > flow_b)
		--current_b;
}

static void set_rgb_flow(unsigned long data)
{
	step_current_rgb_to_flow();

	set_rgb_brightness(current_r, current_g, current_b);

	if (current_r == flow_r &&
		current_g == flow_g &&
		current_b == flow_b)
		return;

	if (!timer_pending(&flow_timer))
		mod_timer(&flow_timer, jiffies + msecs_to_jiffies(flow_freq));
}

void set_rgb_slider(bl_type_t bl_lvl)
{
	if (bl_lvl >= MIN_BRIGHTNESS) {
		if ((enabled == 2) && (bl_lvl <= MAX_BRIGHTNESS)) {
			flush_flow_timer();
			if (bl_lvl > bl_upper) {
				flow_r = daytime_r;
				flow_g = daytime_g;
				flow_b = daytime_b;
			} else if (bl_lvl <= bl_lower) {
				flow_r = target_r;
				flow_g = target_g;
				flow_b = target_b;
			} else {
				bl_scale_rgb(bl_lvl);
			}
			set_rgb_flow(0);
		}
		last_bl = bl_lvl;
	}
}

static void set_enabled(unsigned short val)
{
	if ((val == 1) && (enabled != 1)) {
		flush_timer();
		enabled = 1;
		pulse(0);
		return;
	} else if (val == 0) {
		set_rgb_brightness(daytime_r, daytime_g, daytime_b);
		current_r = daytime_r;
		current_g = daytime_g;
		current_b = daytime_b;

		if (dimmer_auto == 0)
			flush_timer();
	} else if (val == 2) {
		set_rgb_slider(last_bl);
		if (enabled == 1 && !dimmer_auto)
			flush_timer();
	}
	enabled = val;
}

static void dimmer_pipe(unsigned short tmpval)
{
	if ((tmpval >= 10) && (tmpval <= 100) && (tmpval != b_cache)) {
		/*
		* At this point, dimmer may have mutated. The real
		* RGB values must be restored before applying new brightness.
		*/
		if (dimmer_auto == 0 && enabled != 1) {
			b_cache = tmpval;
			if (enabled == 2) {
				dimmer = tmpval;
				set_rgb_slider(last_bl);
			} else {
				fetch_full_rgb();
				dimmer = tmpval;
				set_rgb_brightness(current_r, current_g, current_b);
			}
		} else {
			flush_timer();
			b_cache = tmpval;
			pulse(0);
		}
	}
}

/* MODULE PARAMS */
static int param_enabled_set(const char *buf, const struct kernel_param *kp)
{
	unsigned int tmp = 0;

	if (kstrtouint(buf, 10, &tmp) != 0)
		return -EINVAL;

	if (tmp < 3)
		set_enabled(tmp);

	return tmp;
}

static int param_rgb_set(const char *buf, const struct kernel_param *kp)
{
	unsigned int tmp = 255;

	if (kstrtoint(buf, 10, &tmp) != 0)
		return -EINVAL;

	if (tmp < MIN_SCALE || tmp > MAX_SCALE)
		return -EINVAL;

	param_set_ushort(buf, kp);

	if (enabled == 2) {
		set_rgb_slider(last_bl);
	} else if (enabled == 1) {
		flush_timer();
		pulse(0);
	}

	return tmp;
}

static int param_transition_minutes_set(const char *buf, const struct kernel_param *kp)
{
	unsigned int tmp = 0;

	if (kstrtouint(buf, 10, &tmp) != 0)
		return -EINVAL;

	if (tmp >= 1440)
		return -EINVAL;

	flush_timer();
	param_set_uint(buf, kp);
	calc_active_minutes();

	if (dimmer_auto || enabled == 1)
		pulse(0);

	return tmp;
}

static int param_time_minutes_set(const char *buf, const struct kernel_param *kp)
{
	unsigned int tmp = 0;

	if (kstrtouint(buf, 10, &tmp) != 0)
		return -EINVAL;

	if (start_minute == tmp || stop_minute == tmp || tmp >= 1440)
		return -EINVAL;

	param_set_uint(buf, kp);
	calc_active_minutes();
	return tmp;
}

static int param_dimmer_set(const char *buf, const struct kernel_param *kp)
{
	unsigned int tmp = 0;

	if (kstrtouint(buf, 10, &tmp) != 0)
		return -EINVAL;

	if (tmp > 100 || tmp < 10 || tmp == dimmer)
		return -EINVAL;

	dimmer_pipe(tmp);

	return tmp;
}

static int param_dimmer_auto_set(const char *buf, const struct kernel_param *kp)
{
	unsigned int tmp = 0;

	if (kstrtouint(buf, 10, &tmp) != 0)
		return -EINVAL;

	if (tmp > 1 || tmp == dimmer_auto)
		return -EINVAL;

	flush_timer();

	if (dimmer != 100)
		fetch_full_rgb();

	dimmer_auto = tmp;

	if (!tmp && enabled != 1)
		set_rgb_brightness(current_r, current_g, current_b);
	else
		pulse(0);

	return tmp;
}

static int param_dimmer_minutes_set(const char *buf, const struct kernel_param *kp)
{
	unsigned int tmp = 0;

	if (kstrtouint(buf, 10, &tmp) != 0)
		return -EINVAL;

	if (tmp >= 1440 || tmp == dimmer_start_minute || tmp == dimmer_stop_minute)
		return -EINVAL;

	flush_timer();
	param_set_uint(buf, kp);

	if ((dimmer_auto == 1) || (enabled == 1)) {
		pulse(0);
	}

	return tmp;
}

static int param_bl_range_set(const char *buf, const struct kernel_param *kp)
{
	unsigned int tmp = 0;

	if (kstrtouint(buf, 0, &tmp) != 0)
		return -EINVAL;

	if (tmp > MAX_BRIGHTNESS || tmp < MIN_BRIGHTNESS)
		return -EINVAL;

	if (tmp < bl_lower)
		bl_lower = tmp;
	else if (tmp > bl_upper)
		bl_upper = tmp;
	else
		param_set_uint(buf, kp);

	return tmp;
}

static int param_freqs_set(const char *buf, const struct kernel_param *kp)
{
	unsigned int tmp = 0;

	if (kstrtouint(buf, 0, &tmp) != 0)
		return -EINVAL;

	if (tmp >= 50 && tmp <= 10*60000)
		param_set_uint(buf, kp);

	return tmp;
}

static const struct kernel_param_ops enabled_param_ops = {
	.set = param_enabled_set,
	.get = param_get_ushort,
};

static const struct kernel_param_ops dimmer_param_ops = {
	.set = param_dimmer_set,
	.get = param_get_ushort,
};

static const struct kernel_param_ops dimmer_auto_param_ops = {
	.set = param_dimmer_auto_set,
	.get = param_get_bool,
};

static const struct kernel_param_ops rgb_param_ops = {
	.set = param_rgb_set,
	.get = param_get_ushort,
};

static const struct kernel_param_ops transition_param_ops = {
	.set = param_transition_minutes_set,
	.get = param_get_uint,
};

static const struct kernel_param_ops time_minutes_param_ops = {
	.set = param_time_minutes_set,
	.get = param_get_uint,
};

static const struct kernel_param_ops dimmer_minutes_param_ops = {
	.set = param_dimmer_minutes_set,
	.get = param_get_uint,
};

static const struct kernel_param_ops bl_range_param_ops = {
	.set = param_bl_range_set,
	.get = param_get_uint,
};

static const struct kernel_param_ops freqs_param_ops = {
	.set = param_freqs_set,
	.get = param_get_uint,
};

module_param_cb(enabled_mode, &enabled_param_ops, &enabled, 0644);
module_param_cb(dimmer_factor, &dimmer_param_ops, &b_cache, 0644);
module_param_cb(dimmer_factor_auto, &dimmer_auto_param_ops, &dimmer_auto, 0644);
module_param_cb(dimmer_auto_start_minute, &dimmer_minutes_param_ops, &dimmer_start_minute, 0644);
module_param_cb(dimmer_auto_stop_minute, &dimmer_minutes_param_ops, &dimmer_stop_minute, 0644);
module_param_cb(daytime_r, &rgb_param_ops, &daytime_r, 0644);
module_param_cb(daytime_g, &rgb_param_ops, &daytime_g, 0644);
module_param_cb(daytime_b, &rgb_param_ops, &daytime_b, 0644);
module_param_cb(target_r, &rgb_param_ops, &target_r, 0644);
module_param_cb(target_g, &rgb_param_ops, &target_g, 0644);
module_param_cb(target_b, &rgb_param_ops, &target_b, 0644);
module_param_cb(start_minute, &time_minutes_param_ops, &start_minute, 0644);
module_param_cb(stop_minute, &time_minutes_param_ops, &stop_minute, 0644);
module_param_cb(target_minutes, &transition_param_ops, &target_minute, 0644);
module_param_cb(fadeback_minutes, &transition_param_ops, &fadeback_min, 0644);
module_param_cb(pulse_freq, &freqs_param_ops, &pulse_freq, 0644);
module_param_cb(bl_range_upper, &bl_range_param_ops, &bl_upper, 0644);
module_param_cb(bl_range_lower, &bl_range_param_ops, &bl_lower, 0644);
module_param_cb(flow_freq, &freqs_param_ops, &flow_freq, 0644);

static int __init klapse_init(void)
{
	calc_active_minutes();

	return 0;
}

static void __exit klapse_exit(void)
{
	del_timer_sync(&pulse_timer);
	del_timer_sync(&flow_timer);
}

module_init(klapse_init);
module_exit(klapse_exit);

MODULE_VERSION("5.0");
MODULE_AUTHOR("tanish2k09");
MODULE_LICENSE("GPLv2");

