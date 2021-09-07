#ifndef _BOOT_MODE_H_
#define _BOOT_MODE_H_ 1

enum oem_boot_mode {
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__FASTBOOT,
	MSM_BOOT_MODE__RECOVERY,
	MSM_BOOT_MODE__AGING,
	MSM_BOOT_MODE__FACTORY,
	MSM_BOOT_MODE__RF,
	MSM_BOOT_MODE__WLAN,
	MSM_BOOT_MODE__MOS,
	MSM_BOOT_MODE__CHARGE,
};

enum oem_boot_mode get_boot_mode(void);

enum oem_projcet {
    OEM_PROJECT_18821=18821,
    OEM_PROJECT_18827=18827,
    OEM_PROJECT_11811=11811,
};

int get_oem_project(void);
int get_second_board_absent(void);

#endif
