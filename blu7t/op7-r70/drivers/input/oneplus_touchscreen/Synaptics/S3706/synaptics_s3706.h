/***********************************************************
* Description : OnePlus touchpanel driver
* 
* File		  : synaptics_s3706.h
*
* Function	  : third party interface
* 
* Source	  : provide by synaptics
*
* Version	  : V1.0
*
***********************************************************/
#ifndef SYNAPTICS_H_S3508
#define SYNAPTICS_H_S3508

/*********PART1:Head files**********************/
#include <linux/i2c.h>
#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif
#include <linux/version.h>

#include "../synaptics_common.h"
#ifdef CONFIG_SYNAPTIC_RED
#include "../synaptics_touch_panel_remote.h"
#endif

/*********PART2:Define Area**********************/
#define ENABLE_UNICODE  0x40
#define ENABLE_VEE          0x20
#define ENABLE_CIRCLE   0x08
#define ENABLE_SWIPE        0x02
#define ENABLE_DTAP         0x01

#define UNICODE_DETECT  0x0b
#define VEE_DETECT          0x0a
#define CIRCLE_DETECT   0x08
#define SWIPE_DETECT        0x07
#define DTAP_DETECT         0x03
#define TOUCHHOLD_DOWN		0x0f
#define TOUCHHOLD_UP		0x1f
#define SINGLE_TAP			0x10

#define RESET_TO_NORMAL_TIME 80        /*Sleep time after reset*/
#define POWEWRUP_TO_RESET_TIME 10


#define SPURIOUS_FP_LIMIT 60
#define SPURIOUS_FP_RX_NUM 9
#define SPURIOUS_FP_BASE_DATA_RETRY 10

/*firmware update bootloader V7V8*/
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
#define KERNEL_ABOVE_3_6
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define SSTRTOUL(...) kstrtoul(__VA_ARGS__)
#else
#define SSTRTOUL(...) strict_strtoul(__VA_ARGS__)
#endif

#define PDT_PROPS (0X00EF)
#define PDT_START (0x00E9)
#define PDT_END (0x00D0)
#define PDT_ENTRY_SIZE (0x0006)
#define PAGES_TO_SERVICE (10)
#define PAGE_SELECT_LEN (2)
#define ADDRESS_LEN (2)
#define SYNAPTICS_RMI4_F01 (0x01)
#define SYNAPTICS_RMI4_F34 (0x34)
#define PRODUCT_INFO_SIZE 2
#define PRODUCT_ID_SIZE 10
#define BUILD_ID_SIZE 3

#define F12_GESTURE_DETECTION_LEN 5
#define MAX_INTR_REGISTERS 4
#define MAX_WRITE_SIZE 1024

#define MASK_16BIT 0xFFFF
#define MASK_8BIT 0xFF
#define MASK_7BIT 0x7F
#define MASK_6BIT 0x3F
#define MASK_5BIT 0x1F
#define MASK_4BIT 0x0F
#define MASK_3BIT 0x07
#define MASK_2BIT 0x03
#define MASK_1BIT 0x01

#define DO_LOCKDOWN false

#define MAX_IMAGE_NAME_LEN 256
#define MAX_FIRMWARE_ID_LEN 10

#define IMAGE_HEADER_VERSION_10 0x10
#define MAX_UTILITY_PARAMS 20

#define V7_CONFIG_ID_SIZE 32
#define V7_FLASH_STATUS_OFFSET 0
#define V7_PARTITION_ID_OFFSET 1
#define V7_BLOCK_NUMBER_OFFSET 2
#define V7_TRANSFER_LENGTH_OFFSET 3
#define V7_COMMAND_OFFSET 4
#define V7_PAYLOAD_OFFSET 5
#define V7_PARTITION_SUPPORT_BYTES 4

#define F35_ERROR_CODE_OFFSET 0
#define SLEEP_MODE_NORMAL (0x00)

#define ENABLE_WAIT_MS (1 * 1000)
#define WRITE_WAIT_MS (3 * 1000)
#define ERASE_WAIT_MS (5 * 1000)

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

#define INT_DISABLE_WAIT_MS 20
#define ENTER_FLASH_PROG_WAIT_MS 20
#define READ_CONFIG_WAIT_MS 20

#define ABS(a,b) ((a - b > 0) ? a - b : b - a)

enum exp_fn {
        RMI_DEV = 0,
        RMI_FW_UPDATER,
        RMI_TEST_REPORTING,
        RMI_PROXIMITY,
        RMI_ACTIVE_PEN,
        RMI_GESTURE,
        RMI_VIDEO,
        RMI_DEBUG,
        RMI_LAST,
};

enum f34_version {
        F34_V0 = 0,
        F34_V1,
        F34_V2,
};

enum bl_version {
        BL_V5 = 5,
        BL_V6 = 6,
        BL_V7 = 7,
        BL_V8 = 8,
};

enum flash_area {
        NONE = 0,
        UI_FIRMWARE,
        UI_CONFIG,
};

enum update_mode {
        NORMAL = 1,
        FORCE = 2,
        LOCKDOWN = 8,
};

enum config_area {
        UI_CONFIG_AREA = 0,
        PM_CONFIG_AREA,
        BL_CONFIG_AREA,
        DP_CONFIG_AREA,
        FLASH_CONFIG_AREA,
#ifdef SYNA_TDDI
        TDDI_FORCE_CONFIG_AREA,
        TDDI_LCM_DATA_AREA,
        TDDI_OEM_DATA_AREA,
#endif
        UPP_AREA,
};

enum v7_status {
        SUCCESS = 0x00,
        DEVICE_NOT_IN_BOOTLOADER_MODE,
        INVALID_PARTITION,
        INVALID_COMMAND,
        INVALID_BLOCK_OFFSET,
        INVALID_TRANSFER,
        NOT_ERASED,
        FLASH_PROGRAMMING_KEY_INCORRECT,
        BAD_PARTITION_TABLE,
        CHECKSUM_FAILED,
        FLASH_HARDWARE_FAILURE = 0x1f,
};

enum v7_partition_id {
        BOOTLOADER_PARTITION = 0x01,
        DEVICE_CONFIG_PARTITION,
        FLASH_CONFIG_PARTITION,
        MANUFACTURING_BLOCK_PARTITION,
        GUEST_SERIALIZATION_PARTITION,
        GLOBAL_PARAMETERS_PARTITION,
        CORE_CODE_PARTITION,
        CORE_CONFIG_PARTITION,
        GUEST_CODE_PARTITION,
        DISPLAY_CONFIG_PARTITION,
        EXTERNAL_TOUCH_AFE_CONFIG_PARTITION,
        UTILITY_PARAMETER_PARTITION,
};

enum v7_flash_command {
        CMD_V7_IDLE = 0x00,
        CMD_V7_ENTER_BL,
        CMD_V7_READ,
        CMD_V7_WRITE,
        CMD_V7_ERASE,
        CMD_V7_ERASE_AP,
        CMD_V7_SENSOR_ID,
};

enum flash_command {
        CMD_IDLE = 0,
        CMD_WRITE_FW,
        CMD_WRITE_CONFIG,
        CMD_WRITE_LOCKDOWN,
        CMD_WRITE_GUEST_CODE,
        CMD_WRITE_BOOTLOADER,
        CMD_WRITE_UTILITY_PARAM,
        CMD_READ_CONFIG,
        CMD_ERASE_ALL,
        CMD_ERASE_UI_FIRMWARE,
        CMD_ERASE_UI_CONFIG,
        CMD_ERASE_BL_CONFIG,
        CMD_ERASE_DISP_CONFIG,
        CMD_ERASE_FLASH_CONFIG,
        CMD_ERASE_GUEST_CODE,
        CMD_ERASE_BOOTLOADER,
        CMD_ERASE_UTILITY_PARAMETER,
        CMD_ENABLE_FLASH_PROG,
};

enum container_id {
        TOP_LEVEL_CONTAINER = 0,
        UI_CONTAINER,
        UI_CONFIG_CONTAINER,
        BL_CONTAINER,
        BL_IMAGE_CONTAINER,
        BL_CONFIG_CONTAINER,
        BL_LOCKDOWN_INFO_CONTAINER,
        PERMANENT_CONFIG_CONTAINER,
        GUEST_CODE_CONTAINER,
        BL_PROTOCOL_DESCRIPTOR_CONTAINER,
        UI_PROTOCOL_DESCRIPTOR_CONTAINER,
        RMI_SELF_DISCOVERY_CONTAINER,
        RMI_PAGE_CONTENT_CONTAINER,
        GENERAL_INFORMATION_CONTAINER,
        DEVICE_CONFIG_CONTAINER,
        FLASH_CONFIG_CONTAINER,
        GUEST_SERIALIZATION_CONTAINER,
        GLOBAL_PARAMETERS_CONTAINER,
        CORE_CODE_CONTAINER,
        CORE_CONFIG_CONTAINER,
        DISPLAY_CONFIG_CONTAINER,
        EXTERNAL_TOUCH_AFE_CONFIG_CONTAINER,
        UTILITY_CONTAINER,
        UTILITY_PARAMETER_CONTAINER,
};

enum utility_parameter_id {
        UNUSED = 0,
        FORCE_PARAMETER,
        ANTI_BENDING_PARAMETER,
};

/*********PART3:Struct Area**********************/
/*
 * struct synaptics_rmi4_fn_desc - function descriptor fields in PDT entry
 * @query_base_addr: base address for query registers
 * @cmd_base_addr: base address for command registers
 * @ctrl_base_addr: base address for control registers
 * @data_base_addr: base address for data registers
 * @intr_src_count: number of interrupt sources
 * @fn_version: version of function
 * @fn_number: function number
 */
struct synaptics_rmi4_fn_desc {
        union {
                struct {
                        unsigned char query_base_addr;
                        unsigned char cmd_base_addr;
                        unsigned char ctrl_base_addr;
                        unsigned char data_base_addr;
                        unsigned char intr_src_count:3;
                        unsigned char reserved_1:2;
                        unsigned char fn_version:2;
                        unsigned char reserved_2:1;
                        unsigned char fn_number;
                } __packed;
                unsigned char data[6];
        };
};

/*
 * synaptics_rmi4_fn_full_addr - full 16-bit base addresses
 * @query_base: 16-bit base address for query registers
 * @cmd_base: 16-bit base address for command registers
 * @ctrl_base: 16-bit base address for control registers
 * @data_base: 16-bit base address for data registers
 */
struct synaptics_rmi4_fn_full_addr {
        unsigned short query_base;
        unsigned short cmd_base;
        unsigned short ctrl_base;
        unsigned short data_base;
};

/*
 * struct synaptics_rmi4_input_settings - current input settings
 * @num_of_fingers: maximum number of fingers for 2D touch
 * @valid_button_count: number of valid 0D buttons
 * @max_touch_width: maximum touch width
 * @sensor_max_x: maximum x coordinate for 2D touch
 * @sensor_max_y: maximum y coordinate for 2D touch
 * @force_min: minimum force value
 * @force_max: maximum force value
 * @stylus_enable: flag to indicate reporting of stylus data
 * @eraser_enable: flag to indicate reporting of eraser data
 */
struct synaptics_rmi4_input_settings {
        unsigned char num_of_fingers;
        unsigned char valid_button_count;
        unsigned char max_touch_width;
        int sensor_max_x;
        int sensor_max_y;
        int force_min;
        int force_max;
        bool stylus_enable;
        bool eraser_enable;
};

/*
 * struct synaptics_rmi4_device_info - device information
 * @version_major: RMI protocol major version number
 * @version_minor: RMI protocol minor version number
 * @manufacturer_id: manufacturer ID
 * @product_props: product properties
 * @product_info: product information
 * @product_id_string: product ID
 * @build_id: firmware build ID
 * @support_fn_list: linked list for function handlers
 */
struct synaptics_rmi4_device_info {
        unsigned int version_major;
        unsigned int version_minor;
        unsigned char manufacturer_id;
        unsigned char product_props;
        unsigned char product_info[PRODUCT_INFO_SIZE];
        unsigned char product_id_string[PRODUCT_ID_SIZE + 1];
        unsigned char build_id[BUILD_ID_SIZE];
        struct list_head support_fn_list;
};

/*
 * struct synaptics_rmi4_data - RMI4 device instance data
 * @pdev: pointer to platform device
 * @input_dev: pointer to associated input device
 * @stylus_dev: pointer to associated stylus device
 * @hw_if: pointer to hardware interface data
 * @rmi4_mod_info: device information
 * @board_prop_dir: /sys/board_properties directory for virtual key map file
 * @pwr_reg: pointer to regulator for power control
 * @bus_reg: pointer to regulator for bus pullup control
 * @rmi4_reset_mutex: mutex for software reset
 * @rmi4_report_mutex: mutex for input event reporting
 * @rmi4_io_ctrl_mutex: mutex for communication interface I/O
 * @rmi4_exp_init_mutex: mutex for expansion function module initialization
 * @rmi4_irq_enable_mutex: mutex for enabling/disabling interrupt
 * @rb_work: work for rebuilding input device
 * @rb_workqueue: workqueue for rebuilding input device
 * @fb_notifier: framebuffer notifier client
 * @reset_work: work for issuing reset after display framebuffer ready
 * @reset_workqueue: workqueue for issuing reset after display framebuffer ready
 * @early_suspend: early suspend power management
 * @current_page: current RMI page for register access
 * @button_0d_enabled: switch for enabling 0d button support
 * @num_of_tx: number of Tx channels for 2D touch
 * @num_of_rx: number of Rx channels for 2D touch
 * @num_of_fingers: maximum number of fingers for 2D touch
 * @max_touch_width: maximum touch width
 * @valid_button_count: number of valid 0D buttons
 * @report_enable: input data to report for F$12
 * @no_sleep_setting: default setting of NoSleep in F01_RMI_CTRL00 register
 * @gesture_detection: detected gesture type and properties
 * @intr_mask: interrupt enable mask
 * @button_txrx_mapping: Tx Rx mapping of 0D buttons
 * @num_of_intr_regs: number of interrupt registers
 * @f01_query_base_addr: query base address for f$01
 * @f01_cmd_base_addr: command base address for f$01
 * @f01_ctrl_base_addr: control base address for f$01
 * @f01_data_base_addr: data base address for f$01
 * @f51_query_base_addr: query base address for f$51
 * @firmware_id: firmware build ID
 * @irq: attention interrupt
 * @sensor_max_x: maximum x coordinate for 2D touch
 * @sensor_max_y: maximum y coordinate for 2D touch
 * @force_min: minimum force value
 * @force_max: maximum force value
 * @flash_prog_mode: flag to indicate flash programming mode status
 * @irq_enabled: flag to indicate attention interrupt enable status
 * @fingers_on_2d: flag to indicate presence of fingers in 2D area
 * @suspend: flag to indicate whether in suspend state
 * @sensor_sleep: flag to indicate sleep state of sensor
 * @stay_awake: flag to indicate whether to stay awake during suspend
 * @fb_ready: flag to indicate whether display framebuffer in ready state
 * @f11_wakeup_gesture: flag to indicate support for wakeup gestures in F$11
 * @f12_wakeup_gesture: flag to indicate support for wakeup gestures in F$12
 * @enable_wakeup_gesture: flag to indicate usage of wakeup gestures
 * @wedge_sensor: flag to indicate use of wedge sensor
 * @report_pressure: flag to indicate reporting of pressure data
 * @stylus_enable: flag to indicate reporting of stylus data
 * @eraser_enable: flag to indicate reporting of eraser data
 * @external_afe_buttons: flag to indicate presence of external AFE buttons
 * @reset_device: pointer to device reset function
 * @irq_enable: pointer to interrupt enable function
 * @sleep_enable: pointer to sleep enable function
 * @report_touch: pointer to touch reporting function
 */
struct synaptics_rmi4_data {
        struct platform_device *pdev;
        struct input_dev *input_dev;
        struct input_dev *stylus_dev;
        const struct synaptics_dsx_hw_interface *hw_if;
        struct synaptics_rmi4_device_info rmi4_mod_info;
        struct synaptics_rmi4_input_settings input_settings;
        struct kobject *board_prop_dir;
        struct regulator *pwr_reg;
        struct regulator *bus_reg;
        struct mutex rmi4_reset_mutex;
        struct mutex rmi4_report_mutex;
        struct mutex rmi4_io_ctrl_mutex;
        struct mutex rmi4_exp_init_mutex;
        struct mutex rmi4_irq_enable_mutex;
        struct delayed_work rb_work;
        struct workqueue_struct *rb_workqueue;
#ifdef CONFIG_FB
        struct notifier_block fb_notifier;
        struct work_struct reset_work;
        struct workqueue_struct *reset_workqueue;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend early_suspend;
#endif
        unsigned char current_page;
        unsigned char button_0d_enabled;
        unsigned char num_of_tx;
        unsigned char num_of_rx;
        unsigned char num_of_fingers;
        unsigned char max_touch_width;
        unsigned char valid_button_count;
        unsigned char report_enable;
        unsigned char no_sleep_setting;
        unsigned char gesture_detection[F12_GESTURE_DETECTION_LEN];
        unsigned char intr_mask[MAX_INTR_REGISTERS];
        unsigned char *button_txrx_mapping;
        unsigned short num_of_intr_regs;
        unsigned short f01_query_base_addr;
        unsigned short f01_cmd_base_addr;
        unsigned short f01_ctrl_base_addr;
        unsigned short f01_data_base_addr;
#ifdef F51_DISCRETE_FORCE
        unsigned short f51_query_base_addr;
#endif
        unsigned int firmware_id;
        int irq;
        int sensor_max_x;
        int sensor_max_y;
        int force_min;
        int force_max;
        bool flash_prog_mode;
        bool irq_enabled;
        bool fingers_on_2d;
        bool suspend;
        bool sensor_sleep;
        bool stay_awake;
        bool fb_ready;
        bool f11_wakeup_gesture;
        bool f12_wakeup_gesture;
        bool enable_wakeup_gesture;
        bool wedge_sensor;
        bool report_pressure;
        bool stylus_enable;
        bool eraser_enable;
        bool external_afe_buttons;
        int (*reset_device)(struct synaptics_rmi4_data *rmi4_data,
                        bool rebuild);
        int (*irq_enable)(struct synaptics_rmi4_data *rmi4_data, bool enable,
                        bool attn_only);
        void (*sleep_enable)(struct synaptics_rmi4_data *rmi4_data,
                        bool enable);
        //void (*report_touch)(struct synaptics_rmi4_data *rmi4_data,   //Roland
                        //struct synaptics_rmi4_fn *fhandler);
};

struct synaptics_dsx_bus_access {
        unsigned char type;
        int (*read)(struct synaptics_rmi4_data *rmi4_data, unsigned short addr,
                unsigned char *data, unsigned int length);
        int (*write)(struct synaptics_rmi4_data *rmi4_data, unsigned short addr,
                unsigned char *data, unsigned int length);
};

struct synaptics_dsx_hw_interface {
        struct synaptics_dsx_board_data *board_data;
        const struct synaptics_dsx_bus_access *bus_access;
        int (*bl_hw_init)(struct synaptics_rmi4_data *rmi4_data);
        int (*ui_hw_init)(struct synaptics_rmi4_data *rmi4_data);
};

struct pdt_properties {
        union {
                struct {
                        unsigned char reserved_1:6;
                        unsigned char has_bsr:1;
                        unsigned char reserved_2:1;
                } __packed;
                unsigned char data[1];
        };
};

struct partition_table {
        unsigned char partition_id:5;
        unsigned char byte_0_reserved:3;
        unsigned char byte_1_reserved;
        unsigned char partition_length_7_0;
        unsigned char partition_length_15_8;
        unsigned char start_physical_address_7_0;
        unsigned char start_physical_address_15_8;
        unsigned char partition_properties_7_0;
        unsigned char partition_properties_15_8;
} __packed;

struct f01_device_control {
        union {
                struct {
                        unsigned char sleep_mode:2;
                        unsigned char nosleep:1;
                        unsigned char reserved:2;
                        unsigned char charger_connected:1;
                        unsigned char report_rate:1;
                        unsigned char configured:1;
                } __packed;
                unsigned char data[1];
        };
};

struct f34_v7_query_0 {
        union {
                struct {
                        unsigned char subpacket_1_size:3;
                        unsigned char has_config_id:1;
                        unsigned char f34_query0_b4:1;
                        unsigned char has_thqa:1;
                        unsigned char f34_query0_b6__7:2;
                } __packed;
                unsigned char data[1];
        };
};

struct f34_v7_query_1_7 {
        union {
                struct {
                        /* query 1 */
                        unsigned char bl_minor_revision;
                        unsigned char bl_major_revision;

                        /* query 2 */
                        unsigned char bl_fw_id_7_0;
                        unsigned char bl_fw_id_15_8;
                        unsigned char bl_fw_id_23_16;
                        unsigned char bl_fw_id_31_24;

                        /* query 3 */
                        unsigned char minimum_write_size;
                        unsigned char block_size_7_0;
                        unsigned char block_size_15_8;
                        unsigned char flash_page_size_7_0;
                        unsigned char flash_page_size_15_8;

                        /* query 4 */
                        unsigned char adjustable_partition_area_size_7_0;
                        unsigned char adjustable_partition_area_size_15_8;

                        /* query 5 */
                        unsigned char flash_config_length_7_0;
                        unsigned char flash_config_length_15_8;

                        /* query 6 */
                        unsigned char payload_length_7_0;
                        unsigned char payload_length_15_8;

                        /* query 7 */
                        unsigned char f34_query7_b0:1;
                        unsigned char has_bootloader:1;
                        unsigned char has_device_config:1;
                        unsigned char has_flash_config:1;
                        unsigned char has_manufacturing_block:1;
                        unsigned char has_guest_serialization:1;
                        unsigned char has_global_parameters:1;
                        unsigned char has_core_code:1;
                        unsigned char has_core_config:1;
                        unsigned char has_guest_code:1;
                        unsigned char has_display_config:1;
                        unsigned char f34_query7_b11__15:5;
                        unsigned char f34_query7_b16__23;
                        unsigned char f34_query7_b24__31;
                } __packed;
                unsigned char data[21];
        };
};

struct f34_v7_data0 {
        union {
                struct {
                        unsigned char operation_status:5;
                        unsigned char device_cfg_status:2;
                        unsigned char bl_mode:1;
                } __packed;
                unsigned char data[1];
        };
};

struct f34_v7_data_1_5 {
        union {
                struct {
                        unsigned char partition_id:5;
                        unsigned char f34_data1_b5__7:3;
                        unsigned char block_offset_7_0;
                        unsigned char block_offset_15_8;
                        unsigned char transfer_length_7_0;
                        unsigned char transfer_length_15_8;
                        unsigned char command;
                        unsigned char payload_0;
                        unsigned char payload_1;
                } __packed;
                unsigned char data[8];
        };
};

struct f34_v5v6_flash_properties {
        union {
                struct {
                        unsigned char reg_map:1;
                        unsigned char unlocked:1;
                        unsigned char has_config_id:1;
                        unsigned char has_pm_config:1;
                        unsigned char has_bl_config:1;
                        unsigned char has_disp_config:1;
                        unsigned char has_ctrl1:1;
                        unsigned char has_query4:1;
                } __packed;
                unsigned char data[1];
        };
};

struct register_offset {
        unsigned char properties;
        unsigned char properties_2;
        unsigned char block_size;
        unsigned char block_count;
        unsigned char gc_block_count;
        unsigned char flash_status;
        unsigned char partition_id;
        unsigned char block_number;
        unsigned char transfer_length;
        unsigned char flash_cmd;
        unsigned char payload;
};

struct block_count {
        unsigned short ui_firmware;
        unsigned short ui_config;
        unsigned short dp_config;
        unsigned short pm_config;
        unsigned short fl_config;
        unsigned short bl_image;
        unsigned short bl_config;
        unsigned short utility_param;
        unsigned short lockdown;
        unsigned short guest_code;
        unsigned short total_count;
};

struct physical_address {
        unsigned short ui_firmware;
        unsigned short ui_config;
        unsigned short dp_config;
        unsigned short pm_config;
        unsigned short fl_config;
        unsigned short bl_image;
        unsigned short bl_config;
        unsigned short utility_param;
        unsigned short lockdown;
        unsigned short guest_code;
};

struct container_descriptor {
        unsigned char content_checksum[4];
        unsigned char container_id[2];
        unsigned char minor_version;
        unsigned char major_version;
        unsigned char reserved_08;
        unsigned char reserved_09;
        unsigned char reserved_0a;
        unsigned char reserved_0b;
        unsigned char container_option_flags[4];
        unsigned char content_options_length[4];
        unsigned char content_options_address[4];
        unsigned char content_length[4];
        unsigned char content_address[4];
};

struct image_header_10 {
        unsigned char checksum[4];
        unsigned char reserved_04;
        unsigned char reserved_05;
        unsigned char minor_header_version;
        unsigned char major_header_version;
        unsigned char reserved_08;
        unsigned char reserved_09;
        unsigned char reserved_0a;
        unsigned char reserved_0b;
        unsigned char top_level_container_start_addr[4];
};

struct block_data {
        unsigned int size;
        const unsigned char *data;
};

struct image_metadata {
        bool contains_firmware_id;
        bool contains_bootloader;
        bool contains_guest_code;
        bool contains_disp_config;
        bool contains_perm_config;
        bool contains_flash_config;
        bool contains_utility_param;
        unsigned int firmware_id;
        unsigned int checksum;
        unsigned int bootloader_size;
        unsigned int disp_config_offset;
        unsigned char bl_version;
        unsigned char product_id[PRODUCT_ID_SIZE + 1];
        unsigned char cstmr_product_id[PRODUCT_ID_SIZE + 1];
        unsigned char utility_param_id[MAX_UTILITY_PARAMS];
        struct block_data bootloader;
        struct block_data utility;
        struct block_data ui_firmware;
        struct block_data ui_config;
        struct block_data dp_config;
        struct block_data pm_config;
        struct block_data fl_config;
        struct block_data bl_image;
        struct block_data bl_config;
        struct block_data utility_param[MAX_UTILITY_PARAMS];
        struct block_data lockdown;
        struct block_data guest_code;
        struct block_count blkcount;
        struct physical_address phyaddr;
};

struct synaptics_rmi4_fwu_handle {
        enum bl_version bl_version;
        bool initialized;
        bool in_bl_mode;
        bool in_ub_mode;
        bool bl_mode_device;
        bool force_update;
        bool do_lockdown;
        bool has_guest_code;
        bool has_utility_param;
        bool new_partition_table;
        bool incompatible_partition_tables;
        bool write_bootloader;
        unsigned int data_pos;
        unsigned char *ext_data_source;
        unsigned char *read_config_buf;
        unsigned char intr_mask;
        unsigned char command;
        unsigned char bootloader_id[2];
        unsigned char config_id[32];
        unsigned char flash_status;
        unsigned char partitions;
        unsigned short block_size;
        unsigned short config_size;
        unsigned short config_area;
        unsigned short config_block_count;
        unsigned short flash_config_length;
        unsigned short payload_length;
        unsigned short partition_table_bytes;
        unsigned short read_config_buf_size;
        const unsigned char *config_data;
        const unsigned char *image;
        unsigned char *image_name;
        unsigned int image_size;
        struct image_metadata img;
        struct register_offset off;
        struct block_count blkcount;
        struct physical_address phyaddr;
        struct f34_v5v6_flash_properties flash_properties;
        struct synaptics_rmi4_fn_desc f34_fd;
        struct synaptics_rmi4_fn_desc f35_fd;
        struct synaptics_rmi4_data *rmi4_data;
        struct workqueue_struct *fwu_workqueue;
        struct work_struct fwu_work;
};

struct synaptics_register
{
        uint8_t F01_RMI_QUERY_BASE;
        uint8_t F01_RMI_CMD_BASE;
        uint8_t F01_RMI_CTRL_BASE;
        uint8_t F01_RMI_DATA_BASE;

        uint8_t F01_RMI_QUERY11;
        uint8_t F01_RMI_DATA01;
        uint8_t F01_RMI_CMD00;
        uint8_t F01_RMI_CTRL00;
        uint8_t F01_RMI_CTRL01;
        uint8_t F01_RMI_CTRL02;

        uint8_t F12_2D_QUERY_BASE;
        uint8_t F12_2D_CMD_BASE;
        uint8_t F12_2D_CTRL_BASE;
        uint8_t F12_2D_DATA_BASE;

        uint8_t F12_2D_CTRL08;
        uint8_t F12_2D_CTRL11;
        uint8_t F12_2D_CTRL20;
        uint8_t F12_2D_CTRL23;
        uint8_t F12_2D_CTRL27;
        uint8_t F12_2D_CTRL32;
        uint8_t F12_2D_DATA04;
        uint8_t F12_2D_DATA15;
        uint8_t F12_2D_DATA38;
        uint8_t F12_2D_DATA39;
        uint8_t F12_2D_CMD00;

        uint8_t F34_FLASH_QUERY_BASE;
        uint8_t F34_FLASH_CMD_BASE;
        uint8_t F34_FLASH_CTRL_BASE;
        uint8_t F34_FLASH_DATA_BASE;

        uint8_t SynaF34_FlashControl;
        uint8_t SynaF34Reflash_BlockNum;
        uint8_t SynaF34Reflash_BlockData;
        uint8_t SynaF34ReflashQuery_BootID;
        uint8_t SynaF34ReflashQuery_FlashPropertyQuery;
        uint8_t SynaF34ReflashQuery_FirmwareBlockSize;
        uint8_t SynaF34ReflashQuery_FirmwareBlockCount;
        uint8_t SynaF34ReflashQuery_ConfigBlockSize;
        uint8_t SynaF34ReflashQuery_ConfigBlockCount;

        uint8_t F51_CUSTOM_QUERY_BASE;
        uint8_t F51_CUSTOM_CMD_BASE;
        uint8_t F51_CUSTOM_CTRL_BASE;
        uint8_t F51_CUSTOM_DATA_BASE;

        uint8_t F51_CUSTOM_CTRL00;
        uint8_t F51_CUSTOM_CTRL31;
        uint8_t F51_CUSTOM_CTRL50;
        uint8_t F51_CUSTOM_DATA;

        uint8_t F54_ANALOG_QUERY_BASE;
        uint8_t F54_ANALOG_COMMAND_BASE;
        uint8_t F54_ANALOG_CONTROL_BASE;
        uint8_t F54_ANALOG_DATA_BASE;

        uint8_t F55_SENSOR_CTRL01;
        uint8_t F55_SENSOR_CTRL02;
};

struct data_logger
{
        bool data_logger_control;
        int loglength_addr;
        int loginfo_addr;
};

struct chip_data_s3706 {
        uint32_t *p_tp_fw;
        tp_dev tp_type;
        struct i2c_client *client;
        struct synaptics_proc_operations *syna_ops; /*synaptics func provide to synaptics common driver*/
#ifdef CONFIG_SYNAPTIC_RED
        int        enable_remote;                                           /*Redremote connect state*/
        struct remotepanel_data *premote_data;
#endif/*CONFIG_SYNAPTIC_RED*/
        struct synaptics_register         reg_info;
        struct hw_resource                  *hw_res;
        struct data_logger                  d_log;
        struct synaptics_rmi4_fwu_handle *fwu;          /*3706 firmware update use*/
        int16_t *spuri_fp_data;
		bool is_power_down;
		int en_up_down;
		int in_gesture_mode;
};

static inline int secure_memcpy(unsigned char *dest, unsigned int dest_size,
                const unsigned char *src, unsigned int src_size,
                unsigned int count)
{
        if (dest == NULL || src == NULL) {
                return -EINVAL;
        }

        if (count > dest_size || count > src_size) {
                return -EINVAL;
        }

        memcpy((void *)dest, (const void *)src, count);

        return 0;
}

#endif /*SYNAPTICS_H_S3508*/
