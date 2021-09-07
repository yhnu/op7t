/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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
#ifndef _MSM_DRM_NOTIFY_H_
#define _MSM_DRM_NOTIFY_H_

#include <linux/notifier.h>

/* A hardware display blank change occurred */
#define MSM_DRM_EVENT_BLANK			0x01
/* A hardware display blank early change occurred */
#define MSM_DRM_EARLY_EVENT_BLANK		0x02

enum {
	/* panel: power on */
	MSM_DRM_BLANK_UNBLANK,
	/* panel: power off */
	MSM_DRM_BLANK_POWERDOWN,
	/* panel: power on for tp*/
	MSM_DRM_BLANK_UNBLANK_CUST,
	/* panel: lcd doze mode */
	MSM_DRM_BLANK_NORMAL,
	/* panel: power off */
	MSM_DRM_BLANK_POWERDOWN_CUST,
	MSM_DRM_ONSCREENFINGERPRINT_EVENT,
	MSM_DRM_BLANK_UNBLANK_CHARGE,
	MSM_DRM_BLANK_POWERDOWN_CHARGE,
	MSM_DRM_DYNAMICFPS_60 = 60,
	MSM_DRM_DYNAMICFPS_90 = 90,
};

enum msm_drm_display_id {
	/* primary display */
	MSM_DRM_PRIMARY_DISPLAY,
	/* external display */
	MSM_DRM_EXTERNAL_DISPLAY,
	MSM_DRM_DISPLAY_MAX
};

struct msm_drm_notifier {
	enum msm_drm_display_id id;
	void *data;
};

int msm_drm_register_client(struct notifier_block *nb);
int msm_drm_unregister_client(struct notifier_block *nb);

int dsi_panel_backlight_get(void);
#endif
