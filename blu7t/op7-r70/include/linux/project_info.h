#ifndef _PROJECT_INFO_H_
#define _PROJECT_INFO_H_ 1
typedef		__u32		uint32;
typedef     __u8        uint8;

/*******SECURE_BOOTn = 0x00786078+ 0x4*n, n=[1..14]******/
#define SECURE_BOOT_BASE		0x00786078
#define SECURE_BOOT1			(SECURE_BOOT_BASE + 0x4*1)
#define BUF_SIZE		64

#define SMEM_DUMP_INFO 135
#define SMEM_PROJECT_INFO 136

#include <linux/string.h>
#include <linux/kallsyms.h>
extern unsigned long totalram_pages;

struct project_info {
    char project_name[8];  //eg, 16859
    uint32  hw_version;  //PCB number, T0, EVT
    uint32  rf_v1;   //v1 for mainboard_rf_version
    uint32  rf_v2;   //v2 for aboard_rf_version
    uint32  rf_v3;
    uint32  modem;
    uint32  operator;
    uint32  ddr_manufacture_info;
    uint32  ddr_row;
    uint32  ddr_column;
    uint32  ddr_fw_version;
    uint32  ddr_reserve_info;
    uint32  platform_id;
    uint32  ftm_uart_boot_mode;
    uint32  feature_id;
    uint32  a_board_version;
    uint32  prj_version;
};

#define DUMP_REASON_SIZE 256

struct dump_info{
    char    dump_reason[DUMP_REASON_SIZE];  //dump reason
};

struct component_info {
	char *version;
	char *manufacture;
};

enum{
	HW_VERSION__UNKNOWN,
	HW_VERSION__11 = 11,//all EVB
	HW_VERSION__12,	//T0
};

enum COMPONENT_TYPE {
	DDR,
	EMMC,
	F_CAMERA,
	R_CAMERA,
	SECOND_R_CAMERA,
	THIRD_R_CAMERA,
	R_MODULE,
	F_MODULE,
	R_OIS,
	SECOND_R_OIS,
	TP,
	LCD,
	WCN,
	I_SENSOR,
	G_SENSOR,
	M_SENSOR,
	GYRO,
	BACKLIGHT,
	MAINBOARD,
	/*Add new component here*/
	FINGERPRINTS,
	TOUCH_KEY,
	UFS,
	ABOARD,
	NFC,
	FAST_CHARGE,
	CPU,
    RF_VERSION,
	COMPONENT_MAX,
};

char *parse_function_builtin_return_address(unsigned long function_address);
int push_component_info(enum COMPONENT_TYPE type,
	char *version, char *manufacture);
int reset_component_info(enum COMPONENT_TYPE type);
uint32 get_hw_version(void);
void save_dump_reason_to_smem(char *info, char *function_name);


#endif
