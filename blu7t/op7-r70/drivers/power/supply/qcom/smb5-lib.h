/* Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SMB5_CHARGER_H
#define __SMB5_CHARGER_H
#include <linux/alarmtimer.h>
#include <linux/ktime.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>

/* @bsp, 2019/04/17 Battery & Charging porting */
#include <linux/power/oem_external_fg.h>
#include <linux/extcon.h>
#include <linux/usb/class-dual-role.h>
#include "storm-watch.h"

enum print_reason {
	PR_INTERRUPT	= BIT(0),
	PR_REGISTER	= BIT(1),
	PR_MISC		= BIT(2),
	PR_PARALLEL	= BIT(3),
	PR_OTG		= BIT(4),
	PR_WLS		= BIT(5),
/* @bsp, 2019/04/17 Battery & Charging porting */
	PR_OP_DEBUG	= BIT(6),
	PR_FAST_DEBUG = BIT(7),
};

/* @bsp, 2019/04/17 Battery & Charging porting */
#define BATT_TYPE_FCC_VOTER "BATT_TYPE_FCC_VOTER"
#define PSY_ICL_VOTER		"PSY_ICL_VOTER"
#define TEMP_REGION_MAX               9
#define NON_STANDARD_CHARGER_CHECK_S 100
#define TIME_1000MS 1000
#define REDET_COUTNT 5
#define APSD_CHECK_COUTNT 15
#define DASH_CHECK_COUNT 40
#define BOOST_BACK_COUNT 2
#define TIME_200MS 200
#define TIME_100MS 100
#define TIME_3S 3000
#define NORMAL_CHECK_INTERVAL_PERIOD 300 /*ms*/
#define FAST_CHECK_INTERVAL_PERIOD 100 /*ms*/
#define FAST_CHECK_THRESHOLD_TEMP 45
#define HIGH_TEMP_SHORT_CHECK_TIMEOUT 1500 /*ms*/
#define FIRST_PROTECT_CONNECTER_TEMP 60
#define SECOND_PROTECT_CONNECTER_TEMP 45
#define SECOND_PROTECT_INTERVAL_TEMP 15
#define THIRD_PROTECT_RISE_RATE 3
#define THIRD_PROTECT_LOOP_TEMP 40
#define THIRD_PROTECT_INTERVAL_TEMP 15
#define THIRD_PROTECT_BASE_TEMP 20
#define FV_OFFSET_VOLTAGE 70
#define SKIN_THERMAL_HIGH 40
#define SKIN_THERMAL_NORMAL 37
#define FULL_COUNT_SW_NUM 1
#define DEFAULT_VOTER			"DEFAULT_VOTER"
#define USER_VOTER			"USER_VOTER"
/* @bsp, 2019/04/17 Battery & Charging porting */
#define HW_DETECT_VOTER		"HW_DETECT_VOTER"
#define PD_VOTER			"PD_VOTER"
#define DCP_VOTER			"DCP_VOTER"
#define QC_VOTER			"QC_VOTER"
#define USB_PSY_VOTER			"USB_PSY_VOTER"
#define PL_TAPER_WORK_RUNNING_VOTER	"PL_TAPER_WORK_RUNNING_VOTER"
#define USBIN_V_VOTER			"USBIN_V_VOTER"
#define CHG_STATE_VOTER			"CHG_STATE_VOTER"
#define TAPER_END_VOTER			"TAPER_END_VOTER"
#define THERMAL_DAEMON_VOTER		"THERMAL_DAEMON_VOTER"
#define DIE_TEMP_VOTER			"DIE_TEMP_VOTER"
#define BOOST_BACK_VOTER		"BOOST_BACK_VOTER"
#define MICRO_USB_VOTER			"MICRO_USB_VOTER"
#define DEBUG_BOARD_VOTER		"DEBUG_BOARD_VOTER"
#define PD_SUSPEND_SUPPORTED_VOTER	"PD_SUSPEND_SUPPORTED_VOTER"
#define PL_DELAY_VOTER			"PL_DELAY_VOTER"
#define CTM_VOTER			"CTM_VOTER"
#define SW_QC3_VOTER			"SW_QC3_VOTER"
#define AICL_RERUN_VOTER		"AICL_RERUN_VOTER"
#define SW_ICL_MAX_VOTER		"SW_ICL_MAX_VOTER"
#define PL_QNOVO_VOTER			"PL_QNOVO_VOTER"
#define QNOVO_VOTER			"QNOVO_VOTER"
#define BATT_PROFILE_VOTER		"BATT_PROFILE_VOTER"
#define OTG_DELAY_VOTER			"OTG_DELAY_VOTER"
#define USBIN_I_VOTER			"USBIN_I_VOTER"
#define WEAK_CHARGER_VOTER		"WEAK_CHARGER_VOTER"
#define OTG_VOTER			"OTG_VOTER"
#define PL_FCC_LOW_VOTER		"PL_FCC_LOW_VOTER"
#define WBC_VOTER			"WBC_VOTER"
#define HW_LIMIT_VOTER			"HW_LIMIT_VOTER"
#define PL_SMB_EN_VOTER			"PL_SMB_EN_VOTER"
#define FORCE_RECHARGE_VOTER		"FORCE_RECHARGE_VOTER"
#define LPD_VOTER			"LPD_VOTER"
#define FCC_STEPPER_VOTER		"FCC_STEPPER_VOTER"
#define SW_THERM_REGULATION_VOTER	"SW_THERM_REGULATION_VOTER"
#define JEITA_ARB_VOTER			"JEITA_ARB_VOTER"
#define MOISTURE_VOTER			"MOISTURE_VOTER"
#define HVDCP2_ICL_VOTER		"HVDCP2_ICL_VOTER"
#define AICL_THRESHOLD_VOTER		"AICL_THRESHOLD_VOTER"
#define USBOV_DBC_VOTER			"USBOV_DBC_VOTER"
#define CHG_TERMINATION_VOTER		"CHG_TERMINATION_VOTER"
#define THERMAL_THROTTLE_VOTER		"THERMAL_THROTTLE_VOTER"
#define VOUT_VOTER			"VOUT_VOTER"
#define DR_SWAP_VOTER			"DR_SWAP_VOTER"
#define USB_SUSPEND_VOTER		"USB_SUSPEND_VOTER"
#define CHARGER_TYPE_VOTER		"CHARGER_TYPE_VOTER"
#define HDC_IRQ_VOTER			"HDC_IRQ_VOTER"
#define DETACH_DETECT_VOTER		"DETACH_DETECT_VOTER"
#define CC_MODE_VOTER			"CC_MODE_VOTER"
#define MAIN_FCC_VOTER			"MAIN_FCC_VOTER"

#define BOOST_BACK_STORM_COUNT	3
#define WEAK_CHG_STORM_COUNT	8

#define VBAT_TO_VRAW_ADC(v)		div_u64((u64)v * 1000000UL, 194637UL)

#define ITERM_LIMITS_PMI632_MA		5000
#define ITERM_LIMITS_PM8150B_MA		10000
#define ADC_CHG_ITERM_MASK		32767

#define SDP_100_MA			100000
#define SDP_CURRENT_UA			500000
#define CDP_CURRENT_UA			1500000
#define DCP_CURRENT_UA			1500000
#define HVDCP_CURRENT_UA		3000000
#define TYPEC_DEFAULT_CURRENT_UA	900000
#define TYPEC_MEDIUM_CURRENT_UA		1500000
#define TYPEC_HIGH_CURRENT_UA		3000000

#define ROLE_REVERSAL_DELAY_MS		2000

enum smb_mode {
	PARALLEL_MASTER = 0,
	PARALLEL_SLAVE,
	NUM_MODES,
};

enum sink_src_mode {
	SINK_MODE,
	SRC_MODE,
	AUDIO_ACCESS_MODE,
	UNATTACHED_MODE,
};

enum qc2_non_comp_voltage {
	QC2_COMPLIANT,
	QC2_NON_COMPLIANT_9V,
	QC2_NON_COMPLIANT_12V
};

enum {
	BOOST_BACK_WA			= BIT(0),
	SW_THERM_REGULATION_WA		= BIT(1),
	WEAK_ADAPTER_WA			= BIT(2),
	USBIN_OV_WA			= BIT(3),
	CHG_TERMINATION_WA		= BIT(4),
};

enum jeita_cfg_stat {
	JEITA_CFG_NONE = 0,
	JEITA_CFG_FAILURE,
	JEITA_CFG_COMPLETE,
};

enum {
	RERUN_AICL = 0,
	RESTART_AICL,
};

enum smb_irq_index {
	/* CHGR */
	CHGR_ERROR_IRQ = 0,
	CHG_STATE_CHANGE_IRQ,
	STEP_CHG_STATE_CHANGE_IRQ,
	STEP_CHG_SOC_UPDATE_FAIL_IRQ,
	STEP_CHG_SOC_UPDATE_REQ_IRQ,
	FG_FVCAL_QUALIFIED_IRQ,
	VPH_ALARM_IRQ,
	VPH_DROP_PRECHG_IRQ,
	/* DCDC */
	OTG_FAIL_IRQ,
	OTG_OC_DISABLE_SW_IRQ,
	OTG_OC_HICCUP_IRQ,
	BSM_ACTIVE_IRQ,
	HIGH_DUTY_CYCLE_IRQ,
	INPUT_CURRENT_LIMITING_IRQ,
	CONCURRENT_MODE_DISABLE_IRQ,
	SWITCHER_POWER_OK_IRQ,
	/* BATIF */
	BAT_TEMP_IRQ,
	ALL_CHNL_CONV_DONE_IRQ,
	BAT_OV_IRQ,
	BAT_LOW_IRQ,
	BAT_THERM_OR_ID_MISSING_IRQ,
	BAT_TERMINAL_MISSING_IRQ,
	BUCK_OC_IRQ,
	VPH_OV_IRQ,
	/* USB */
	USBIN_COLLAPSE_IRQ,
	USBIN_VASHDN_IRQ,
	USBIN_UV_IRQ,
	USBIN_OV_IRQ,
	USBIN_PLUGIN_IRQ,
	USBIN_REVI_CHANGE_IRQ,
	USBIN_SRC_CHANGE_IRQ,
	USBIN_ICL_CHANGE_IRQ,
	/* DC */
	DCIN_VASHDN_IRQ,
	DCIN_UV_IRQ,
	DCIN_OV_IRQ,
	DCIN_PLUGIN_IRQ,
	DCIN_REVI_IRQ,
	DCIN_PON_IRQ,
	DCIN_EN_IRQ,
	/* TYPEC */
	TYPEC_OR_RID_DETECTION_CHANGE_IRQ,
	TYPEC_VPD_DETECT_IRQ,
	TYPEC_CC_STATE_CHANGE_IRQ,
	TYPEC_VCONN_OC_IRQ,
	TYPEC_VBUS_CHANGE_IRQ,
	TYPEC_ATTACH_DETACH_IRQ,
	TYPEC_LEGACY_CABLE_DETECT_IRQ,
	TYPEC_TRY_SNK_SRC_DETECT_IRQ,
	/* MISC */
	WDOG_SNARL_IRQ,
	WDOG_BARK_IRQ,
	AICL_FAIL_IRQ,
	AICL_DONE_IRQ,
	SMB_EN_IRQ,
	IMP_TRIGGER_IRQ,
	TEMP_CHANGE_IRQ,
	TEMP_CHANGE_SMB_IRQ,
	/* FLASH */
	VREG_OK_IRQ,
	ILIM_S2_IRQ,
	ILIM_S1_IRQ,
	VOUT_DOWN_IRQ,
	VOUT_UP_IRQ,
	FLASH_STATE_CHANGE_IRQ,
	TORCH_REQ_IRQ,
	FLASH_EN_IRQ,
	SDAM_STS_IRQ,
	/* END */
	SMB_IRQ_MAX,
};

enum float_options {
	FLOAT_DCP		= 1,
	FLOAT_SDP		= 2,
	DISABLE_CHARGING	= 3,
	SUSPEND_INPUT		= 4,
};

enum chg_term_config_src {
	ITERM_SRC_UNSPECIFIED,
	ITERM_SRC_ADC,
	ITERM_SRC_ANALOG
};

enum comp_clamp_levels {
	CLAMP_LEVEL_DEFAULT = 0,
	CLAMP_LEVEL_1,
	MAX_CLAMP_LEVEL,
};

struct clamp_config {
	u16 reg[3];
	u16 val[3];
};

struct smb_irq_info {
	const char			*name;
	const irq_handler_t		handler;
	const bool			wake;
	const struct storm_watch	storm_data;
	struct smb_irq_data		*irq_data;
	int				irq;
	bool				enabled;
};

static const unsigned int smblib_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

enum lpd_reason {
	LPD_NONE,
	LPD_MOISTURE_DETECTED,
	LPD_FLOATING_CABLE,
};

/* Following states are applicable only for floating cable during LPD */
enum lpd_stage {
	/* initial stage */
	LPD_STAGE_NONE,
	/* started and ongoing */
	LPD_STAGE_FLOAT,
	/* cancel if started,  or don't start */
	LPD_STAGE_FLOAT_CANCEL,
	/* confirmed and mitigation measures taken for 60 s */
	LPD_STAGE_COMMIT,
};

enum thermal_status_levels {
	TEMP_SHUT_DOWN = 0,
	TEMP_SHUT_DOWN_SMB,
	TEMP_ALERT_LEVEL,
	TEMP_ABOVE_RANGE,
	TEMP_WITHIN_RANGE,
	TEMP_BELOW_RANGE,
};

enum icl_override_mode {
	/* APSD/Type-C/QC auto */
	HW_AUTO_MODE,
	/* 100/150/500/900mA */
	SW_OVERRIDE_USB51_MODE,
	/* ICL other than USB51 */
	SW_OVERRIDE_HC_MODE,
};

/* EXTCON_USB and EXTCON_USB_HOST are mutually exclusive */
static const u32 smblib_extcon_exclusive[] = {0x3, 0};

struct smb_regulator {
	struct regulator_dev	*rdev;
	struct regulator_desc	rdesc;
};

struct smb_irq_data {
	void			*parent_data;
	const char		*name;
	struct storm_watch	storm_data;
};

struct smb_chg_param {
	const char	*name;
	u16		reg;
	int		min_u;
	int		max_u;
	int		step_u;
	int		(*get_proc)(struct smb_chg_param *param,
				    u8 val_raw);
	int		(*set_proc)(struct smb_chg_param *param,
				    int val_u,
				    u8 *val_raw);
};

struct buck_boost_freq {
	int freq_khz;
	u8 val;
};

struct smb_chg_freq {
	unsigned int		freq_5V;
	unsigned int		freq_6V_8V;
	unsigned int		freq_9V;
	unsigned int		freq_12V;
	unsigned int		freq_removal;
	unsigned int		freq_below_otg_threshold;
	unsigned int		freq_above_otg_threshold;
};

struct smb_params {
	struct smb_chg_param	fcc;
	struct smb_chg_param	fv;
	struct smb_chg_param	usb_icl;
	struct smb_chg_param	icl_max_stat;
	struct smb_chg_param	icl_stat;
	struct smb_chg_param	otg_cl;
	struct smb_chg_param	dc_icl;
	struct smb_chg_param	jeita_cc_comp_hot;
	struct smb_chg_param	jeita_cc_comp_cold;
	struct smb_chg_param	freq_switcher;
	struct smb_chg_param	aicl_5v_threshold;
	struct smb_chg_param	aicl_cont_threshold;
};

struct parallel_params {
	struct power_supply	*psy;
};

struct smb_iio {
	struct iio_channel	*temp_chan;
	struct iio_channel	*usbin_i_chan;
	struct iio_channel	*usbin_v_chan;
	struct iio_channel	*mid_chan;
	struct iio_channel	*batt_i_chan;
	struct iio_channel	*connector_temp_chan;
/* @bsp, 2019/04/17 Battery & Charging porting */
/* @bsp 2018/07/30 add usb connector temp detect and wr*/
	struct iio_channel	*op_connector_temp_chan;
	struct iio_channel	*op_skin_therm_chan;
	struct iio_channel	*sbux_chan;
	struct iio_channel	*vph_v_chan;
	struct iio_channel	*die_temp_chan;
	struct iio_channel	*skin_temp_chan;
	struct iio_channel	*smb_temp_chan;
};

struct smb_charger {
	struct device		*dev;
	char			*name;
	struct regmap		*regmap;
	struct smb_irq_info	*irq_info;
	struct smb_params	param;
	struct smb_iio		iio;
	int			*debug_mask;
	int			*pd_disabled;
	int			*usb_connector_temp;
	int			*usb_interval_temp;
	int			*disable_connector_protect;
	int			*call_on;
	enum smb_mode		mode;
	struct smb_chg_freq	chg_freq;
	int			smb_version;
	int			otg_delay_ms;
	int			*weak_chg_icl_ua;
	bool			pd_not_supported;

	/* locks */
	struct mutex		smb_lock;
	struct mutex		ps_change_lock;
	struct mutex		dr_lock;

/* @bsp, 2019/04/17 Battery & Charging porting */
	struct mutex		write_lock;
	struct mutex		sw_dash_lock;
/* @bsp, 2018/07/26 Enable external stm6620 ship mode*/
	struct pinctrl_state *ship_mode_default;
	struct pinctrl_state *usb_temperature_default;
	struct pinctrl *pinctrl;
	struct mutex		irq_status_lock;

	/* power supplies */
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*dc_psy;
	struct power_supply		*bms_psy;
/* @bsp, 2019/04/17 Battery & Charging porting */
	struct power_supply_desc	usb_psy_desc;
	struct power_supply		*usb_main_psy;
	struct power_supply		*usb_port_psy;
	struct power_supply		*wls_psy;
	struct power_supply		*cp_psy;
	enum power_supply_type		real_charger_type;

	/* dual role class */
	struct dual_role_phy_instance	*dual_role;

	/* notifiers */
	struct notifier_block	nb;
/* @bsp, 2019/04/17 Battery & Charging porting */
#if defined(CONFIG_FB)
	struct notifier_block		fb_notif;
#elif defined(CONFIG_MSM_RDM_NOTIFY)
	struct notifier_block		msm_drm_notifier;
#endif

	/* parallel charging */
	struct parallel_params	pl;

	/* CC Mode */
	int	adapter_cc_mode;

	/* regulators */
	struct smb_regulator	*vbus_vreg;
	struct smb_regulator	*vconn_vreg;
	struct regulator	*dpdm_reg;

	/* votables */
	struct votable		*dc_suspend_votable;
/* @bsp, 2019/04/17 Battery & Charging porting */
	/*usb connector hw auto detection*/
	struct votable		*otg_toggle_votable;
	struct votable		*fcc_votable;
	struct votable		*fcc_main_votable;
	struct votable		*fv_votable;
	struct votable		*usb_icl_votable;
	struct votable		*awake_votable;
	struct votable		*pl_disable_votable;
	struct votable		*chg_disable_votable;
	struct votable		*pl_enable_votable_indirect;
	struct votable		*cp_disable_votable;
	struct votable		*smb_override_votable;
	struct votable		*icl_irq_disable_votable;
	struct votable		*limited_irq_disable_votable;
	struct votable		*hdc_irq_disable_votable;
	struct votable		*temp_change_irq_disable_votable;

	/* work */
	struct work_struct	bms_update_work;
	struct work_struct	pl_update_work;
	struct work_struct	jeita_update_work;
	struct work_struct	moisture_protection_work;
	struct work_struct	chg_termination_work;
	struct delayed_work	ps_change_timeout_work;
/* @bsp, 2019/04/17 Battery & Charging porting */
	struct delayed_work rechk_sw_dsh_work;
	struct delayed_work	re_kick_work;
	struct delayed_work	unplug_check_work;
	struct delayed_work	recovery_suspend_work;
	struct delayed_work	check_switch_dash_work;
	struct delayed_work non_standard_charger_check_work;
	struct delayed_work heartbeat_work;
	struct delayed_work re_det_work;
	struct delayed_work op_re_set_work;
	struct delayed_work	op_check_apsd_work;
	struct work_struct	get_aicl_work;
	struct delayed_work connecter_check_work;
	struct work_struct	otg_switch_work;
	struct wakeup_source	chg_wake_lock;

	struct delayed_work	clear_hdc_work;
	struct delayed_work	icl_change_work;
	struct delayed_work	pl_enable_work;
	struct delayed_work	uusb_otg_work;
	struct delayed_work	bb_removal_work;
	struct delayed_work	lpd_ra_open_work;
	struct delayed_work	lpd_detach_work;
	struct delayed_work	thermal_regulation_work;
	struct delayed_work	usbov_dbc_work;
	struct delayed_work	role_reversal_check;
	struct delayed_work	pr_swap_detach_work;

	struct alarm		lpd_recheck_timer;
	struct alarm		moisture_protection_alarm;
	struct alarm		chg_termination_alarm;

	/* secondary charger config */
	bool			sec_pl_present;
	bool			sec_cp_present;
	int			sec_chg_selected;
	int			cp_reason;

	/* pd */
	int			voltage_min_uv;
	int			voltage_max_uv;
	int			pd_active;
	bool			pd_hard_reset;
	bool			pr_swap_in_progress;
	bool			early_usb_attach;
	bool			ok_to_pd;
	bool			typec_legacy;

	/* cached status */
/* @bsp, 2019/04/17 Battery & Charging porting */
	int				BATT_TEMP_T0;
	int				BATT_TEMP_T1;
	int				BATT_TEMP_T2;
	int				BATT_TEMP_T3;
	int				BATT_TEMP_T4;
	int				BATT_TEMP_T5;
	int				BATT_TEMP_T6;
	int				batt_health;
	int				ibatmax[TEMP_REGION_MAX];
	int				vbatmax[TEMP_REGION_MAX];
	int				vbatdet[TEMP_REGION_MAX];
	int				temp_littel_cool_voltage;
	int				temp_littel_cool_current;
	int				temp_cool_voltage;
	int				temp_cool_current;
	int				fake_chgvol;
	int				fake_temp;
	int				fake_protect_sts;
	int				non_stand_chg_current;
	int				non_stand_chg_count;
	int				redet_count;
	int				reset_count;
	int				dump_count;
	int				ck_apsd_count;
	int				ck_dash_count;
	int				ck_unplug_count;
	int				recovery_boost_count;
	int				op_icl_val;
	int				plug_irq;
	int				hw_detect;
	bool				otg_switch;
	bool				use_fake_chgvol;
	bool				use_fake_temp;
	bool				use_fake_protect_sts;
	bool				vbus_present;
	bool				hvdcp_present;
	bool				dash_present;
	bool				charger_collpse;
	bool				usb_enum_status;
	bool				non_std_chg_present;
	bool				usb_type_redet_done;
	bool				time_out;
	bool				disable_normal_chg_for_dash;
	bool				ship_mode;
	bool				dash_on;
	bool				chg_ovp;
	bool				is_power_changed;
	bool				recharge_pending;
	bool				recharge_status;
	bool temp_littel_cool_set_current_0_point_25c;
	bool				oem_lcd_is_on;
	bool                            disable_ctrl_current;
	bool				chg_enabled;
	bool				op_apsd_done;
	bool				re_trigr_dash_done;
	bool				boot_usb_present;
	bool				init_irq_done;
	bool				is_aging_test;
	bool				revert_boost_trigger;
	bool               switch_on_fastchg;
	bool				probe_done;
	int				ffc_count;
	int				FFC_TEMP_T1;
	int				FFC_TEMP_T2;
	int				FFC_TEMP_T3;
	int				FFC_NOR_FCC;
	int				FFC_WARM_FCC;
	int				FFC_NORMAL_CUTOFF;
	int				FFC_WARM_CUTOFF;
	int				FFC_VBAT_FULL;
	enum ffc_step			ffc_status;
	enum temp_region_type		mBattTempRegion;
	enum batt_status_type		battery_status;
	short				mBattTempBoundT0;
	short				mBattTempBoundT1;
	short				mBattTempBoundT2;
	short				mBattTempBoundT3;
	short				mBattTempBoundT4;
	short				mBattTempBoundT5;
	short				mBattTempBoundT6;
	uint32_t			bus_client;
	bool				is_audio_adapter;
	int					fv_offset_voltage_mv;
	int					normal_check_interval_period;
	int					fast_check_interval_period;
	int					fast_check_threshold_temp;
	int					high_temp_short_check_timeout;
	int					first_protect_connecter_temp;
	int					second_protect_connecter_temp;
	int					second_protect_interval_temp;
	int					third_protect_rise_rate;
	int					third_protect_loop_temp;
	int					third_protect_interval_temp;
	int					third_protect_base_temp;
	int					skin_thermal_high_threshold;
	int					skin_thermal_normal_threshold;
	bool					enable_dash_current_adjust;
	int					full_count_sw_num;

	bool			system_suspend_supported;
	int			boost_threshold_ua;
	int			system_temp_level;
	int			thermal_levels;
	int			*thermal_mitigation;
	int			dcp_icl_ua;
	int			fake_capacity;
	int			fake_batt_status;
	bool			step_chg_enabled;
	bool			sw_jeita_enabled;
	bool			typec_legacy_use_rp_icl;
	bool			is_hdc;
	bool			chg_done;
	int			connector_type;
	bool			otg_en;
	bool			suspend_input_on_debug_batt;
	int			default_icl_ua;
	int			otg_cl_ua;
/* @bsp, 2019/04/17 Battery & Charging porting */
	int			sw_iterm_ma;
	int			little_cold_iterm_ma;
	bool			check_batt_full_by_sw;
/*@bsp, 20181023 icl set 1A if battery lower than 15%*/
	bool			OTG_ICL_CTRL;
	int			OTG_LOW_BAT;
	int			OTG_LOW_BAT_ICL;
	int			OTG_NORMAL_BAT_ICL;
/* @bsp, 2018/07/26 add the external ship-mode and connecter temp detect start*/
	int			shipmode_en;
	int			connecter_temp;
	int			count_total;
	int			count_run;
	int			filter_count;
	int			pre_temp;
	int			current_temp;
	bool			connector_short;
	int			connecter_voltage;
	int			skin_thermal_temp;
	bool			is_skin_thermal_high;
	int			disconnect_vbus;
	int			vbus_ctrl;
	bool			low_voltage_charger;
/* @bsp, 2018/07/26 add the external ship-mode and connecter temp detect end*/
	bool			vph_sel_disable;
	bool			vph_set_flag;
	bool			uusb_apsd_rerun_done;
	bool			typec_present;
	int			fake_input_current_limited;
	int			typec_mode;
	int			usb_icl_change_irq_enabled;
	u32			jeita_status;
	u8			float_cfg;
	bool			jeita_arb_flag;
	bool			use_extcon;
	bool			otg_present;
	bool			hvdcp_disable;
	int			hw_max_icl_ua;
	int			auto_recharge_soc;
	enum sink_src_mode	sink_src_mode;
	enum jeita_cfg_stat	jeita_configured;
	int			charger_temp_max;
	int			smb_temp_max;
	u8			typec_try_mode;
	enum lpd_stage		lpd_stage;
	bool			lpd_disabled;
	enum lpd_reason		lpd_reason;
	bool			fcc_stepper_enable;
	int			die_temp;
	int			smb_temp;
	int			skin_temp;
	int			connector_temp;
	int			thermal_status;
	int			main_fcc_max;
	u32			jeita_soft_thlds[2];
	u32			jeita_soft_hys_thlds[2];
	int			jeita_soft_fcc[2];
	int			jeita_soft_fv[2];
	bool			moisture_present;
	bool			uusb_moisture_protection_enabled;
	bool			hw_die_temp_mitigation;
	bool			hw_connector_mitigation;
	bool			hw_skin_temp_mitigation;
	int			connector_pull_up;
	int			aicl_5v_threshold_mv;
	int			default_aicl_5v_threshold_mv;
	int			aicl_cont_threshold_mv;
	int			default_aicl_cont_threshold_mv;
	bool			aicl_max_reached;
	int			charge_full_cc;
	int			cc_soc_ref;
	int			last_cc_soc;
	int			dr_mode;
	int			usbin_forced_max_uv;
	int			init_thermal_ua;
	u32			comp_clamp_level;

	/* workaround flag */
	u32			wa_flags;
	int			boost_current_ua;
	int                     qc2_max_pulses;
	enum qc2_non_comp_voltage qc2_unsupported_voltage;
	bool			dbc_usbov;

	/* extcon for VBUS / ID notification to USB for uUSB */
	struct extcon_dev	*extcon;

	/* battery profile */
	int			batt_profile_fcc_ua;
	int			batt_profile_fv_uv;

	int			usb_icl_delta_ua;
	int			pulse_cnt;

	int			die_health;
	int			connector_health;

	/* flash */
	u32			flash_derating_soc;
	u32			flash_disable_soc;
	u32			headroom_mode;
	bool			flash_init_done;
	bool			flash_active;
	u32			irq_status;

	/* wireless */
	int			wireless_vout;
};

/* @bsp, 2018/07/13 Battery & Charging porting */
int smblib_set_prop_charge_parameter_set(struct smb_charger *chg);
extern void set_mcu_en_gpio_value(int value);
extern void usb_sw_gpio_set(int value);
extern bool op_set_fast_chg_allow(struct smb_charger *chg, bool enable);
extern bool get_prop_fast_chg_started(struct smb_charger *chg);
extern void mcu_en_gpio_set(int value);
extern void switch_mode_to_normal(void);
extern struct smb_charger *g_chg;
void op_disconnect_vbus(struct smb_charger *chg, bool enable);

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val);
int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val);
int smblib_write(struct smb_charger *chg, u16 addr, u8 val);
int smblib_batch_write(struct smb_charger *chg, u16 addr, u8 *val, int count);
int smblib_batch_read(struct smb_charger *chg, u16 addr, u8 *val, int count);

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u);
int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend);
int smblib_get_aicl_cont_threshold(struct smb_chg_param *param, u8 val_raw);
int smblib_enable_charging(struct smb_charger *chg, bool enable);
int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u);
int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend);
int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend);

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw);
int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw);
int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw);
int smblib_set_chg_freq(struct smb_chg_param *param,
				int val_u, u8 *val_raw);
int smblib_set_prop_boost_current(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_aicl_cont_threshold(struct smb_chg_param *param,
				int val_u, u8 *val_raw);
int smblib_vbus_regulator_enable(struct regulator_dev *rdev);
int smblib_vbus_regulator_disable(struct regulator_dev *rdev);
int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev);

int smblib_vconn_regulator_enable(struct regulator_dev *rdev);
int smblib_vconn_regulator_disable(struct regulator_dev *rdev);
int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev);

irqreturn_t default_irq_handler(int irq, void *data);
irqreturn_t smb_en_irq_handler(int irq, void *data);
irqreturn_t chg_state_change_irq_handler(int irq, void *data);
irqreturn_t batt_temp_changed_irq_handler(int irq, void *data);
irqreturn_t batt_psy_changed_irq_handler(int irq, void *data);
irqreturn_t usbin_uv_irq_handler(int irq, void *data);
irqreturn_t usb_plugin_irq_handler(int irq, void *data);
irqreturn_t usb_source_change_irq_handler(int irq, void *data);
irqreturn_t icl_change_irq_handler(int irq, void *data);
irqreturn_t typec_state_change_irq_handler(int irq, void *data);
irqreturn_t typec_attach_detach_irq_handler(int irq, void *data);
irqreturn_t dc_plugin_irq_handler(int irq, void *data);
irqreturn_t high_duty_cycle_irq_handler(int irq, void *data);
irqreturn_t switcher_power_ok_irq_handler(int irq, void *data);
irqreturn_t wdog_snarl_irq_handler(int irq, void *data);
irqreturn_t wdog_bark_irq_handler(int irq, void *data);
irqreturn_t typec_or_rid_detection_change_irq_handler(int irq, void *data);
irqreturn_t temp_change_irq_handler(int irq, void *data);
irqreturn_t usbin_ov_irq_handler(int irq, void *data);
irqreturn_t sdam_sts_change_irq_handler(int irq, void *data);
int smblib_get_prop_input_suspend(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_system_temp_level_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_iterm(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_input_suspend(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_batt_status(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val);
/* @bsp, 2019/04/17 Battery & Charging porting */
void op_bus_vote(int disable);
int get_prop_fast_adapter_update(struct smb_charger *chg);
void op_handle_usb_plugin(struct smb_charger *chg);
int op_rerun_apsd(struct smb_charger *chg);
irqreturn_t smblib_handle_aicl_done(int irq, void *data);
void op_charge_info_init(struct smb_charger *chg);
int update_dash_unplug_status(void);
int get_prop_batt_status(struct smb_charger *chg);
int get_prop_chg_protect_status(struct smb_charger *chg);
int op_set_prop_otg_switch(struct smb_charger *chg,
				bool enalbe);
int check_allow_switch_dash(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_chg_voltage(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_batt_temp(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_chg_protect_status(struct smb_charger *chg,
				const union power_supply_propval *val);
bool op_get_fastchg_ing(struct smb_charger *chg);
bool get_prop_fastchg_status(struct smb_charger *chg);
int op_usb_icl_set(struct smb_charger *chg, int icl_ua);
int op_get_aicl_result(struct smb_charger *chg);
bool get_prop_fastchg_is_ok(struct smb_charger *chg);

int smblib_set_prop_input_current_limited(struct smb_charger *chg,
				const union power_supply_propval *val);

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_dc_online(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_get_prop_dc_voltage_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_dc_voltage_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_voltage_wls_output(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_voltage_wls_output(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_dc_reset(struct smb_charger *chg);
int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_online(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_suspend(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_voltage_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_voltage_max_design(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_usb_voltage_max_limit(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_low_power(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_typec_select_rp(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_input_current_settled(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_input_voltage_settled(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val);
int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val);
int smblib_get_prop_charger_temp(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_die_health(struct smb_charger *chg);
int smblib_get_prop_connector_health(struct smb_charger *chg);
int smblib_get_prop_vph_voltage_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_sdp_current_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_pd_voltage_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_pd_voltage_min(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_typec_select_rp(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_pd_active(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_ship_mode(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_rechg_soc_thresh(struct smb_charger *chg,
				const union power_supply_propval *val);
void smblib_suspend_on_debug_battery(struct smb_charger *chg);
int smblib_rerun_apsd_if_required(struct smb_charger *chg);
int smblib_get_prop_fcc_delta(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_thermal_threshold(struct smb_charger *chg, u16 addr, int *val);
int smblib_dp_dm(struct smb_charger *chg, int val);
int smblib_disable_hw_jeita(struct smb_charger *chg, bool disable);
int smblib_run_aicl(struct smb_charger *chg, int type);
int smblib_set_icl_current(struct smb_charger *chg, int icl_ua);
int smblib_get_icl_current(struct smb_charger *chg, int *icl_ua);
int smblib_get_charge_current(struct smb_charger *chg, int *total_current_ua);
int smblib_get_prop_pr_swap_in_progress(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_pr_swap_in_progress(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_force_dr_mode(struct smb_charger *chg, int mode);
int smblib_get_prop_from_bms(struct smb_charger *chg,
				enum power_supply_property psp,
				union power_supply_propval *val);
int smblib_get_iio_channel(struct smb_charger *chg, const char *propname,
					struct iio_channel **chan);
int smblib_read_iio_channel(struct smb_charger *chg, struct iio_channel *chan,
							int div, int *data);
int smblib_configure_hvdcp_apsd(struct smb_charger *chg, bool enable);
int smblib_icl_override(struct smb_charger *chg, enum icl_override_mode mode);
enum alarmtimer_restart smblib_lpd_recheck_timer(struct alarm *alarm,
				ktime_t time);
int smblib_toggle_smb_en(struct smb_charger *chg, int toggle);
void smblib_hvdcp_detect_enable(struct smb_charger *chg, bool enable);
void smblib_hvdcp_exit_config(struct smb_charger *chg);
void smblib_apsd_enable(struct smb_charger *chg, bool enable);
int smblib_force_vbus_voltage(struct smb_charger *chg, u8 val);
int smblib_get_irq_status(struct smb_charger *chg,
				union power_supply_propval *val);

int smblib_init(struct smb_charger *chg);
int smblib_deinit(struct smb_charger *chg);
#endif /* __SMB5_CHARGER_H */
