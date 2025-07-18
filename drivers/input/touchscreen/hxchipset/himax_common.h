/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for common functions
 *
 *  Copyright (C) 2019 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#ifndef HIMAX_COMMON_H
#define HIMAX_COMMON_H

#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/pm_wakeup.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include "himax_platform.h"
#include <linux/kallsyms.h>
#include <linux/version.h>
//#include <linux/time64.h>
#if defined(CONFIG_OF)
	#include <linux/of_gpio.h>
#endif

/*Linden code for JLINDEN-3952 by chenxin at 20230306 start*/
#define HIMAX_DRIVER_VER "2.1.0.15_balsa_03"
/*Linden code for JLINDEN-3952 by chenxin at 20230306 end*/

#define FLASH_DUMP_FILE "/sdcard/HX_Flash_Dump.bin"

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
#define HX_TP_PROC_2T2R
/*if enable, selftest works in driver*/
/*#define HX_TP_SELF_TEST_DRIVER*/
#endif
/*===========Himax Option function=============*/
#define HX_RST_PIN_FUNC
#define HX_EXCP_RECOVERY

/*#define HX_NEW_EVENT_STACK_FORMAT*/
/*#define HX_BOOT_UPGRADE*/
#define HX_SMART_WAKEUP
/*#define HX_GESTURE_TRACK*/
#define HX_RESUME_SEND_CMD	/*Need to enable on TDDI chipset*/
/*#define HX_HIGH_SENSE*/
/*#define HX_PALM_REPORT*/
#define HX_USB_DETECT_GLOBAL
/*Linden code for JLINDEN-305 by lumz2 at 20221117 start*/
#define HX_RW_FILE
/*Linden code for JLINDEN-305 by lumz2 at 20221117 end*/


/* for MTK special platform.If turning on,
 * it will report to system by using specific format.
 */
/*#define HX_PROTOCOL_A*/
#define HX_PROTOCOL_B_3PA

#define HX_ZERO_FLASH
/* linden code for JLINDEN-11559 by dingying3 at 20230815 start */
#define Lenovo_pen_node_fun
#if defined(Lenovo_pen_node_fun)
#include <linux/string.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#endif
/* linden code for JLINDEN-11559 by dingying3 at 20230815 end */
#define Lenovo_pen_id_fun
//#define HX_FIRMWARE_HEADER
/*system suspend-chipset power off,
 *oncell chipset need to enable the definition
 */
/*#define HX_RESUME_HW_RESET*/

/*for Himax auto-motive chipset
 */
/*#define HX_PON_PIN_SUPPORT*/

/*#define HX_PARSE_FROM_DT*/

/*Enable this if testing suspend/resume
 *on nitrogen8m
 */
#define HX_CONFIG_DRM_PANEL

/*used for self test get dsram fail in stress test*/
/*Linden code for JLINDEN-1361 by lumz2 at 20230128 start*/
#define HX_STRESS_SELF_TEST
/*Linden code for JLINDEN-1361 by lumz2 at 20230128 end*/

/*=============================================*/

/* Enable it if driver go into suspend/resume twice */
/*#undef HX_CONFIG_FB*/

/* Enable it if driver go into suspend/resume twice */
/*#undef HX_CONFIG_DRM*/

#if defined(HX_CONFIG_DRM_PANEL)
#undef HX_CONFIG_FB
#include <drm/drm_panel.h>
//extern struct drm_panel gNotifier_dummy_panel;
#elif defined(HX_CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(HX_CONFIG_DRM)
#include <linux/msm_drm_notify.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
#define KERNEL_VER_ABOVE_4_19
#endif

/*Linden code for JLINDEN-305 by lumz2 at 20221117 start*/
/* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))*/
#define KERNEL_VER_5_10
/*#endif*/

#if 0
/*Linden code for JLINDEN-305 by lumz2 at 20221117 end*/
#define proc_op(name) proc_##name
#define proc_opl(name) proc_##name
#define proc_ops_name proc_ops
#define time_var timespec64
#define time_func ktime_get_real_ts64
#define owner_line

#else
#define proc_op(name) name
#define proc_opl(name) l##name
#define proc_ops_name file_operations
#define time_var timespec
#define time_func getnstimeofday
#define owner_line .owner\
		=\
		THIS_MODULE,
#endif
#if defined(HX_ZERO_FLASH)
/*zero flash case, you need to setup the fix_touch_info of module*/
/*Please set the size according to IC*/
#define DSRAM_SIZE HX_48K_SZ
#define HX_RESUME_SET_FW
/* used for 102p overlay */
/*#define HX_ALG_OVERLAY*/
/* used for 102d overlay */
/*#define HX_CODE_OVERLAY*/
/*Independent threads run the notification chain notification function resume*/
/*Linden code for JLINDEN-822 by chenxin at 20230109 start*/
#define HX_CONTAINER_SPEED_UP
/*Linden code for JLINDEN-822 by chenxin at 20230109 end*/
#else
#define HX_TP_PROC_GUEST_INFO
#endif

#if defined(HX_EXCP_RECOVERY) && defined(HX_ZERO_FLASH)
/* used for 102e/p zero flash */
/*#define HW_ED_EXCP_EVENT*/
#endif

#if defined(HX_BOOT_UPGRADE) || defined(HX_ZERO_FLASH)
/* FW Auto upgrade case, you need to setup the fix_touch_info of module
 */
extern char *g_fw_boot_upgrade_name;
/* Linden code for JLINDEN-10584 by dingying at 20230607 start */
extern char BOOT_UPGRADE_FWNAME[128];
/* Linden code for JLINDEN-10584 by dingying at 20230607 end */
#if defined(HX_ZERO_FLASH)
extern char *g_fw_mp_upgrade_name;
#define MPAP_FWNAME "Himax_mpfw.bin"
#define RESUME_FW_REQUEST_FROM_BUFFER
#endif
#endif

#if defined(HX_PARSE_FROM_DT)
extern uint32_t g_proj_id;
#endif

#if defined(HX_SMART_WAKEUP)
/*This feature need P-sensor driver notified, and FW need to support*/
/*#define HX_ULTRA_LOW_POWER*/
#endif

#if defined(HX_SMART_WAKEUP) && defined(HX_RESUME_SET_FW)
/* decide whether reload FW after Smart Wake Up */
#define HX_SWU_RESUME_SET_FW
#endif

#if defined(HX_CONTAINER_SPEED_UP)
/*Resume queue delay work time after LCM RST (unit:ms)
 */
/*Linden code for JLINDEN-1276 by chenxin at 20230113 start*/
#define DELAY_TIME 85
/*Linden code for JLINDEN-1276 by chenxin at 20230113 end*/
#endif

#if defined(HX_RST_PIN_FUNC)
/* origin is 20/50 */
/*Linden code for JLINDEN-822 by chenxin at 20230109 start*/
#define RST_LOW_PERIOD_S 2000
#define RST_LOW_PERIOD_E 2100
/*Linden code for JLINDEN-822 by chenxin at 20230109 end*/
#if defined(HX_ZERO_FLASH)
/*Linden code for JLINDEN-822 by chenxin at 20230109 start*/
#define RST_HIGH_PERIOD_S 1000
#define RST_HIGH_PERIOD_E 1100
/*Linden code for JLINDEN-822 by chenxin at 20230109 end*/
#else
#define RST_HIGH_PERIOD_S 50000
#define RST_HIGH_PERIOD_E 50100
#endif
#endif

#if defined(HX_CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data);
#elif defined(HX_CONFIG_DRM)
int drm_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data);
#endif

#define HX_85XX_J_SERIES_PWON		"HX85xxJ"
#define HX_83102D_SERIES_PWON		"HX83102D"
#define HX_83102E_SERIES_PWON		"HX83102E"
#define HX_83102J_SERIES_PWON		"HX83102J"
#define HX_83108A_SERIES_PWON		"HX83108A"
#define HX_83112A_SERIES_PWON		"HX83112A"
#define HX_83112F_SERIES_PWON		"HX83112F"
#define HX_83121A_SERIES_PWON		"HX83121A"

#define HX_TP_BIN_CHECKSUM_SW		1
#define HX_TP_BIN_CHECKSUM_HW		2
#define HX_TP_BIN_CHECKSUM_CRC		3

#define SHIFTBITS 5

#define FW_SIZE_32k		32768
#define FW_SIZE_60k		61440
#define FW_SIZE_64k		65536
#define FW_SIZE_124k	126976
#define FW_SIZE_128k	131072
#define FW_SIZE_255k    261120
/********************************************************
* PANEL_PORTRAIT_0		:	VERITCAL
* PANEL_LANDSCAPE_90	:	HORIZONTAL_NOTCH_RIGHT_SIDE
* PANEL_LANDSCAPE_180	:   RESERVER VERITCAL
* PANEL_LANDSCAPE_270	:	HORIZONTAL_NOTCH_LEFT_SIDE
*********************************************************/
#define PANEL_PORTRAIT_0 		0
#define PANEL_LANDSCAPE_90		1
#define PANEL_LANDSCAPE_180		2
#define PANEL_LANDSCAPE_270		3

#define NO_ERR 0
#define READY_TO_SERVE 1
#define WORK_OUT	2
#define HX_EMBEDDED_FW 3
#define BUS_FAIL -1
#define HX_INIT_FAIL -1
#define MEM_ALLOC_FAIL -2
#define CHECKSUM_FAIL -3
#define GESTURE_DETECT_FAIL -4
#define INPUT_REGISTER_FAIL -5
#define FW_NOT_READY -6
#define LENGTH_FAIL -7
#define OPEN_FILE_FAIL -8
#define PROBE_FAIL -9
#define ERR_WORK_OUT	-10
#define ERR_STS_WRONG	-11
#define ERR_TEST_FAIL	-12
#define HW_CRC_FAIL 1

#define HX_FINGER_ON	1
#define HX_FINGER_LEAVE	2

#if defined(HX_PALM_REPORT)
#define PALM_REPORT 1
#define NOT_REPORT -1
#endif

#define STYLUS_INFO_SZ 12
#if defined(Lenovo_pen_node_fun)

#define HIMAX_MAX_BUFFER		32
#define MAX_IO_CONTROL_REPORT	16

/* linden code for JLINDEN-11559 by dingying3 at 20230815 start */
#define DRIVER_NAME "lenovo_penraw_driver"
#define DEVICE_NAME "lenovo_penraw"
/* linden code for JLINDEN-11559 by dingying3 at 20230815 end */
static const unsigned int MINOR_NUMBER_START = 0; /* the minor number starts at */
static const unsigned int NUMBER_MINOR_NUMBER = 1; /* the number of minor numbers */
static unsigned int major_number; /* the major number of the device */
/* ioctl for direct access */
static struct cdev penraw_char_dev; /* character device */
static struct class* penraw_char_dev_class = NULL; /* class object */

enum{
    DATA_TYPE_RAW = 0
};

enum pen_status {
	TS_NONE,
	TS_RELEASE,
	TS_TOUCH,
};

struct himax_pen_coords_buffer {
	signed char status;
	signed char tool_type;
	signed char tilt_x;
	signed char tilt_y;
	unsigned long int x;
	unsigned long int y;
	unsigned long int p;
};

struct himax_pen_info {
	unsigned char frame_no;
	unsigned char data_type;
	unsigned char reserve[2];
	struct himax_pen_coords_buffer coords;
};

struct io_pen_report {
	unsigned char report_num;
	unsigned char reserve[3];
	struct himax_pen_info pen_info[MAX_IO_CONTROL_REPORT];
};

//Direct access via IOCTL
static struct himax_pen_info pen_buffer[HIMAX_MAX_BUFFER];
static unsigned char pen_report_num;
static unsigned char pen_frame_no;
static unsigned char pen_buffer_wp;
#endif

enum HX_TS_PATH {
	HX_REPORT_COORD = 1,
	HX_REPORT_SMWP_EVENT,
	HX_REPORT_COORD_RAWDATA,
};

enum HX_TS_STATUS {
	HX_TS_GET_DATA_FAIL = -4,
	HX_EXCP_EVENT,
	HX_CHKSUM_FAIL,
	HX_PATH_FAIL,
	HX_TS_NORMAL_END = 0,
	HX_EXCP_REC_OK,
	HX_READY_SERVE,
	HX_REPORT_DATA,
	HX_EXCP_WARNING,
	HX_IC_RUNNING,
	HX_ZERO_EVENT_COUNT,
	HX_RST_OK,
};

enum cell_type {
	CHIP_IS_ON_CELL,
	CHIP_IS_IN_CELL
};

#if defined(HX_SMART_WAKEUP)
#define HX_KEY_DOUBLE_CLICK KEY_POWER
#define HX_KEY_UP           KEY_UP
#define HX_KEY_DOWN         KEY_DOWN
#define HX_KEY_LEFT         KEY_LEFT
#define HX_KEY_RIGHT        KEY_RIGHT
#define HX_KEY_C            KEY_C
#define HX_KEY_Z            KEY_Z
#define HX_KEY_M            KEY_M
#define HX_KEY_O            KEY_O
#define HX_KEY_S            KEY_S
#define HX_KEY_V            KEY_V
#define HX_KEY_W            KEY_W
#define HX_KEY_E            KEY_E
#define HX_KEY_LC_M         263
#define HX_KEY_AT           264
#define HX_KEY_RESERVE      265
#define HX_KEY_FINGER_GEST  266
#define HX_KEY_V_DOWN       267
#define HX_KEY_V_LEFT       268
#define HX_KEY_V_RIGHT      269
#define HX_KEY_F_RIGHT      270
#define HX_KEY_F_LEFT       271
#define HX_KEY_DF_UP        272
#define HX_KEY_DF_DOWN      273
#define HX_KEY_DF_LEFT      274
#define HX_KEY_DF_RIGHT     275
#endif

enum fix_touch_info {
	FIX_HX_RX_NUM = 48,
	FIX_HX_TX_NUM = 32,
	FIX_HX_BT_NUM = 0,
	FIX_HX_MAX_PT = 10,
	FIX_HX_INT_IS_EDGE = true,
	FIX_HX_STYLUS_FUNC = 1,
	FIX_HX_STYLUS_ID_V2 = 0,
	FIX_HX_STYLUS_RATIO = 1,
#if defined(HX_TP_PROC_2T2R)
	FIX_HX_RX_NUM_2 = 0,
	FIX_HX_TX_NUM_2 = 0,
#endif
};

#if defined(HX_ZERO_FLASH)
	#define HX_SPI_OPERATION
	#define HX_0F_DEBUG
#endif
struct himax_ic_data {
	int vendor_fw_ver;
	int vendor_config_ver;
	int vendor_touch_cfg_ver;
	int vendor_display_cfg_ver;
	int vendor_cid_maj_ver;
	int vendor_cid_min_ver;
	int vendor_panel_ver;
	int vendor_sensor_id;
	int ic_adc_num;
	uint8_t vendor_cus_info[12];
	uint8_t vendor_proj_info[12];
	uint32_t flash_size;
	uint32_t HX_RX_NUM;
	uint32_t HX_TX_NUM;
	uint32_t HX_BT_NUM;
	uint32_t HX_X_RES;
	uint32_t HX_Y_RES;
	uint32_t HX_MAX_PT;
	uint8_t HX_INT_IS_EDGE;
	uint8_t HX_STYLUS_FUNC;
	uint8_t HX_STYLUS_ID_V2;
	uint8_t HX_STYLUS_RATIO;
#if defined(HX_TP_PROC_2T2R)
	int HX_RX_NUM_2;
	int HX_TX_NUM_2;
#endif
	/*Linden code for JLINDEN-3584 by chenxin29 at 20230302 start*/
	char str_fw_ver[8];
	/*Linden code for JLINDEN-3584 by chenxin29 at 20230302 end*/
};

struct himax_virtual_key {
	int index;
	int keycode;
	int x_range_min;
	int x_range_max;
	int y_range_min;
	int y_range_max;
};

struct himax_target_point_data {
	int x;
	int y;
	int w;
	int id;
};

struct himax_target_stylus_data {
	int32_t x;
	int32_t y;
	int32_t w;
	uint32_t hover;
	int32_t tilt_x;
	uint32_t btn;
	uint32_t btn2;
	int32_t tilt_y;
	uint32_t on;
	int pre_btn;
	int pre_btn2;
	uint8_t battery_info;
	uint64_t id;
};

struct himax_target_report_data {

	struct himax_target_point_data *p;

	int finger_on;
	int finger_num;

#if defined(HX_SMART_WAKEUP)
	int SMWP_event_chk;
#endif

	struct himax_target_stylus_data *s;

	int ig_count;
};

struct himax_report_data {
	int touch_all_size;
	int raw_cnt_max;
	int raw_cnt_rmd;
	int touch_info_size;
	uint8_t	finger_num;
	uint8_t	finger_on;
	uint8_t *hx_coord_buf;
	uint8_t hx_state_info[2];
#if defined(HX_SMART_WAKEUP)
	int event_size;
	uint8_t *hx_event_buf;
#endif

	int rawdata_size;
	uint8_t diag_cmd;
	uint8_t *hx_rawdata_buf;
	uint8_t rawdata_frame_size;
};

/*Linden code for JLINDEN-503 by chenxin at 20221214 start*/
#if defined(HX_USB_DETECT_GLOBAL)
#define USB_DETECT_IN 1
#define USB_DETECT_OUT 0

struct himax_usb_charger_detection {
	struct notifier_block charger_notif;
	uint8_t usb_connected;
	struct workqueue_struct *himax_charger_notify_wq;
	struct work_struct charger_notify_work;
};
#endif
/*Linden code for JLINDEN-503 by chenxin at 20221214 end*/

struct himax_ts_data {
	bool initialized;
	bool suspended;
	int notouch_frame;
	int ic_notouch_frame;
	atomic_t suspend_mode;
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t useScreenRes;
	uint8_t diag_cmd;
	uint8_t diag_storage_type;
	bool diag_dirly;
	char chip_name[30];
	uint8_t chip_cell_type;
#if defined(Lenovo_pen_id_fun)
	uint8_t pen_id[7];
#endif
	uint8_t direction;
	uint8_t game_mode;
	/*Linden code for JLINDEN-3952 by chenxin at 20230306 start*/
	uint8_t support_pen;
	/*Linden code for JLINDEN-3952 by chenxin at 20230306 end*/
	int32_t cmd_param[4];
	uint32_t chip_max_dsram_size;

	uint8_t protocol_type;
	uint8_t first_pressed;
	uint8_t coord_data_size;
	uint8_t area_data_size;
	uint8_t coordInfoSize;
	uint8_t raw_data_frame_size;
	uint8_t raw_data_nframes;
	uint8_t nFinger_support;
	uint8_t irq_enabled;
	uint8_t diag_self[50];

	uint16_t finger_pressed;
	uint16_t last_slot;
	uint16_t pre_finger_mask;
	uint16_t old_finger;
	int hx_point_num;
	uint8_t hx_stylus_num;

	uint32_t debug_log_level;
	uint32_t widthFactor;
	uint32_t heightFactor;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;

	int rst_gpio;
	int use_irq;
	int (*power)(int on);
	int pre_finger_data[10][2];

	struct device *dev;
	struct workqueue_struct *himax_wq;
	struct work_struct work;
	struct input_dev *input_dev;

	struct input_dev *stylus_dev;

	struct hrtimer timer;
	struct i2c_client *client;
	struct himax_platform_data *pdata;
	struct mutex reg_lock;
	struct mutex rw_lock;
	atomic_t irq_state;
	spinlock_t irq_lock;

/******* SPI-start *******/
	struct spi_device	*spi;
	int hx_irq;
	uint8_t *xfer_buff;
/******* SPI-end *******/

	int in_self_test;
	int suspend_resume_done;
	int bus_speed;

#if defined(HX_CONFIG_FB) || defined(HX_CONFIG_DRM)
	struct notifier_block fb_notif;
	struct workqueue_struct *himax_att_wq;
	struct delayed_work work_att;
#endif

	struct workqueue_struct *dump_wq;
	struct work_struct dump_work;
	struct workqueue_struct *himax_boot_upgrade_wq;
	struct delayed_work work_boot_upgrade;
#if defined(RESUME_FW_REQUEST_FROM_BUFFER)
	size_t g_fw_len;
	uint8_t *g_fw_buf;
	bool g_fw_sta;
#endif
#if defined(HX_CONTAINER_SPEED_UP)
	struct workqueue_struct *ts_int_workqueue;
	struct delayed_work ts_int_work;
#endif

	struct workqueue_struct *himax_diag_wq;
	struct delayed_work himax_diag_delay_work;

	uint8_t *cfg_buf;
	uint8_t SMWP_enable;
	uint8_t gesture_cust_en[26];
	struct wakeup_source *ts_SMWP_wake_lock;
#if defined(HX_ULTRA_LOW_POWER)
	bool psensor_flag;
#endif


#if defined(HX_HIGH_SENSE)
	uint8_t HSEN_enable;
#endif

#if defined(HX_USB_DETECT_GLOBAL)
	uint8_t usb_connected;
	uint8_t *cable_config;
	/*Linden code for JLINDEN-503 by chenxin at 20221214 start*/
	uint32_t charger_detection_enable;
	struct himax_usb_charger_detection *charger_detection;
	/*Linden code for JLINDEN-503 by chenxin at 20221214 end*/
#endif

#if defined(HX_TP_PROC_GUEST_INFO)
	struct workqueue_struct *guest_info_wq;
	struct work_struct guest_info_work;
#endif
	uint8_t slave_write_reg;
	uint8_t slave_read_reg;
	bool acc_slave_reg;
	bool select_slave_reg;
	/* Linden code for JLINDEN-11049 by dingying at 20231020 start */
	bool major_state;
	/* Linden code for JLINDEN-11049 by dingying at 20231020 end */

};

struct himax_debug {
	bool flash_dump_going;
	bool is_checking_irq;
	bool is_call_help;
	void (*fp_ts_dbg_func)(struct himax_ts_data *ts, int start);
	int (*fp_set_diag_cmd)(struct himax_ic_data *ic_data,
				struct himax_report_data *hx_touch_data);
};

enum input_protocol_type {
	PROTOCOL_TYPE_A	= 0x00,
	PROTOCOL_TYPE_B	= 0x01,
};

#if defined(HX_HIGH_SENSE)
	void himax_set_HSEN_func(uint8_t HSEN_enable);
#endif

#if defined(HX_SMART_WAKEUP)
void himax_set_SMWP_func(uint8_t SMWP_enable);

#define GEST_PTLG_ID_LEN	(4)
#define GEST_PTLG_HDR_LEN	(4)
#define GEST_PTLG_HDR_ID1	(0xCC)
#define GEST_PTLG_HDR_ID2	(0x44)
#define GEST_PT_MAX_NUM		(128)

extern uint8_t *wake_event_buffer;
#endif

extern int g_mmi_refcnt;
extern int *g_inspt_crtra_flag;
extern uint32_t g_hx_chip_inited;
extern bool g_has_alg_overlay;
/*void himax_HW_reset(uint8_t loadconfig,uint8_t int_off);*/
#if defined(HX_FIRMWARE_HEADER)
#include "himax_firmware.h"
extern int32_t g_hx_panel_id;
int32_t get_fw_index(int32_t fw_type);
void mapping_panel_id_from_dt(struct device_node *dt);
extern struct firmware g_embedded_fw;
#endif

int himax_chip_common_suspend(struct himax_ts_data *ts);
int himax_chip_common_resume(struct himax_ts_data *ts);

#if defined(HX_RW_FILE)
/*Linden code for JLINDEN-305 by lumz2 at 20221117 start*/
#if !defined(KERNEL_VER_5_10)
/*Linden code for JLINDEN-305 by lumz2 at 20221117 end*/
extern struct filename* (*kp_getname_kernel)(const char *filename);
extern void (*kp_putname_kernel)(struct filename *name);
extern struct file * (*kp_file_open_name)(struct filename *name,
			int flags, umode_t mode);
/*Linden code for JLINDEN-305 by lumz2 at 20221117 start*/
#endif
/*Linden code for JLINDEN-305 by lumz2 at 20221117 end*/
#endif

struct himax_core_fp;
extern struct himax_core_fp g_core_fp;
extern struct himax_ts_data *private_ts;
extern struct himax_ic_data *ic_data;
extern struct device *g_device;

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
	int himax_debug_init(void);
	int himax_debug_remove(void);
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_INSPECT)
	extern char *g_rslt_data;
	extern uint32_t g_rslt_data_len;
	extern void (*fp_himax_self_test_init)(void);
#endif

#if defined(HX_CONFIG_DRM)
#if defined(HX_CONFIG_DRM_PANEL)
	extern struct drm_panel *active_panel;
#endif
#endif
extern int HX_TOUCH_INFO_POINT_CNT;

extern bool ic_boot_done;

int himax_parse_dt(struct himax_ts_data *ts, struct himax_platform_data *pdata);

extern void himax_parse_dt_ic_info(struct himax_ts_data *ts,
	struct himax_platform_data *pdata);

int himax_report_data(struct himax_ts_data *ts, int ts_path, int ts_status);

int himax_report_data_init(void);

int himax_dev_set(struct himax_ts_data *ts);
int himax_input_register_device(struct input_dev *input_dev);

#if defined(HX_RW_FILE)
extern int hx_open_file(char *file_name);
extern int hx_write_file(char *write_data, uint32_t write_size, loff_t pos);
extern int hx_close_file(void);
#endif
#endif
