/*
 * Copyright (C) 2010 - 2021 Novatek, Inc.
 *
 * $Revision: 103375 $
 * $Date: 2022-07-29 10:34:16 +0800 (週五, 29 七月 2022) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "nt36xxx_mem_map.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
#define HAVE_PROC_OPS
#endif

#ifdef CONFIG_MTK_SPI
/* Please copy mt_spi.h file under mtk spi driver folder */
#include "mt_spi.h"
#endif

#ifdef CONFIG_SPI_MT65XX
#include <linux/platform_data/spi-mt65xx.h>
#endif

#define NVT_DEBUG 1

//---GPIO number---
#define NVTTOUCH_RST_PIN 980
#define NVTTOUCH_INT_PIN 943


//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
//#define IRQ_TYPE_EDGE_FALLING 2
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING


//---SPI driver info.---
#define NVT_SPI_NAME "NVT-ts"

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"
/* Linden code for JLINDEN-11612 by dingying at 20230818 start */
/* Linden code for JLINDEN-11127 by dingying at 20230707 start */
#define NVT_PEN_NAME "lenovo_ts,pen"
/* Linden code for JLINDEN-11127 by dingying at 20230707 end */
/* Linden code for JLINDEN-11612 by dingying at 20230818 end */

//---Touch info.---
#define TOUCH_DEFAULT_MAX_WIDTH 1200
#define TOUCH_DEFAULT_MAX_HEIGHT 2000
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#define TOUCH_FORCE_NUM 1000
//---for Pen---
#define PEN_PRESSURE_MAX (4095)
#define PEN_DISTANCE_MAX (1)
#define PEN_TILT_MIN (-60)
#define PEN_TILT_MAX (60)

/* Enable only when module have tp reset pin and connected to host */
#define NVT_TOUCH_SUPPORT_HW_RST 0

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define NVT_SAVE_TEST_DATA_IN_FILE 1
#define MT_PROTOCOL_B 1
#define WAKEUP_GESTURE 1
/*Linden code for JLINDEN-3546 by lumz2 at 20230302 start*/
#define NVT_CUST_PROC_CMD 1
#define NVT_EDGE_REJECT 1
#define NVT_EDGE_GRID_ZONE 1
#define NVT_PALM_MODE 1
#define NVT_SUPPORT_PEN 1
/*Linden code for JLINDEN-3546 by lumz2 at 20230302 end*/
#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif
#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"
#define MP_UPDATE_FIRMWARE_NAME   "novatek_ts_mp.bin"

/* linden code for JLINDEN-7558 by dingying3 at 2023/4/14 start */
#define NVT_SUPER_RESOLUTION_N 10
#if NVT_SUPER_RESOLUTION_N
#define POINT_DATA_LEN 108
#else
#define POINT_DATA_LEN 65
#endif
/* linden code for JLINDEN-7558 by dingying3 at 2023/4/14 end */

#define POINT_DATA_CHECKSUM 1
#define POINT_DATA_CHECKSUM_LEN 65

//---ESD Protect.---
#define NVT_TOUCH_ESD_PROTECT 0
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500	/* ms */
#define NVT_TOUCH_WDT_RECOVERY 1
/*Linden code for JLINDEN-3912 by dingying3 at 20230315 start*/
#define NVT_TOUCH_ESD_DISP_RECOVERY 1
/*Linden code for JLINDEN-3912 by dingying3 at 20230315 end*/

#define CHECK_PEN_DATA_CHECKSUM 0

/*Linden code for JLINDEN-3546 by lumz2 at 20230302 start*/
#if NVT_CUST_PROC_CMD
struct edge_grid_zone_info {
    uint8_t degree;
    uint8_t direction;
    uint16_t y1;
    uint16_t y2;
};
#endif
/*Linden code for JLINDEN-3546 by lumz2 at 20230302 end*/

/*Linden code for JLINDEN-1423 by lumz2 at 20230209 start*/
#define USB_DETECT_IN 1
#define USB_DETECT_OUT 0
#define CMD_CHARGER_ON	(0x53)
#define CMD_CHARGER_OFF (0x51)

struct nvt_usb_charger_detection {
	struct notifier_block charger_notif;
	uint8_t usb_connected;
	struct workqueue_struct *nvt_charger_notify_wq;
	struct work_struct charger_notify_work;
};
/*Linden code for JLINDEN-1423 by lumz2 at 20230209 end*/

struct nvt_ts_data {
	struct spi_device *client;
	struct input_dev *input_dev;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
	/*Linden code for JLINDEN-283 by chenxin at 20221125 start*/
	uint8_t nvt_gesture_en;
	/*Linden code for JLINDEN-283 by chenxin at 20221125 end*/
#if defined(CONFIG_DRM_PANEL)
	struct notifier_block drm_panel_notif;
#elif defined(_MSM_DRM_NOTIFY_H_)
	struct notifier_block drm_notif;
#elif defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t hw_crc;
	uint16_t nvt_pid;
	uint8_t *rbuf;
	uint8_t *xbuf;
	struct mutex xbuf_lock;
	bool irq_enabled;
	bool pen_support;
	bool stylus_resol_double;
	uint8_t x_gang_num;
	uint8_t y_gang_num;
	struct input_dev *pen_input_dev;
	int8_t pen_phys[32];
#ifdef CONFIG_MTK_SPI
	struct mt_chip_conf spi_ctrl;
#endif
#ifdef CONFIG_SPI_MT65XX
    struct mtk_chip_config spi_ctrl;
#endif
     	/*Linden code for JLINDEN-1365 by chenxin at 20230118 start*/
	#ifdef CONFIG_PM
	bool dev_pm_suspend;
	struct completion dev_pm_resume_completion;
	#endif
/*Linden code for JLINDEN-3546 by lumz2 at 20230302 start*/
#if NVT_CUST_PROC_CMD
	int32_t edge_reject_state;
	struct edge_grid_zone_info edge_grid_zone_info;
	uint8_t game_mode_state;
	uint8_t pen_state;
/*Linden code for JLINDEN-11126 by dingying at 20230717 start*/
	uint8_t probe_state;
/*Linden code for JLINDEN-11126 by dingying at 20230717 end*/
#endif
/*Linden code for JLINDEN-3546 by lumz2 at 20230302 end*/
	/*Linden code for JLINDEN-1365 by chenxin at 20230118 end*/
	/*Linden code for JLINDEN-1423 by lumz2 at 20230209 start*/
	uint32_t charger_detection_enable;
	struct nvt_usb_charger_detection *charger_detection;
	/*Linden code for JLINDEN-1423 by lumz2 at 20230209 end*/
	/*Linden code for JLINDEN-3584 by chenxin29 at 20230302 start*/
	char str_fw_ver[8];
	/*Linden code for JLINDEN-3584 by chenxin29 at 20230302 end*/
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)

#define DUMMY_BYTES (1)
#define NVT_TRANSFER_LEN	(63*1024)
#define NVT_READ_LEN		(2*1024)
#define NVT_XBUF_LEN		(NVT_TRANSFER_LEN+1+DUMMY_BYTES)

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;


/*Linden code for JLINDEN-3912 by chenxin at 20230307 start*/
#if NVT_TOUCH_ESD_DISP_RECOVERY
#define ILM_CRC_FLAG        0x01
#define DLM_CRC_FLAG        0x02
#define CRC_DONE            0x04
#define F2C_RW_READ         0x00
#define F2C_RW_WRITE        0x01
#define BIT_F2C_EN          0
#define BIT_F2C_RW          1
#define BIT_CPU_IF_ADDR_INC 2
#define BIT_CPU_POLLING_EN  5
#define FFM2CPU_CTL         0x3F280
#define F2C_LENGTH          0x3F283
#define CPU_IF_ADDR         0x3F284
#define FFM_ADDR            0x3F286
#define CP_TP_CPU_REQ       0x3F291
#define TOUCH_DATA_ADDR     0x25198
#define DISP_OFF_ADDR       0x2800
#endif /* NVT_TOUCH_ESD_DISP_RECOVERY */
/*Linden code for JLINDEN-3912 by chenxin at 20230307 end*/

//---extern structures---
extern struct nvt_ts_data *ts;

#define LENOVO_MAX_BUFFER   32
#define MAX_IO_CONTROL_REPORT   16

enum{
	DATA_TYPE_RAW = 0
};

struct lenovo_pen_coords_buffer {
	signed char status;
	signed char tool_type;
	signed char tilt_x;
	signed char tilt_y;
	unsigned long int x;
	unsigned long int y;
	unsigned long int p;
};

struct lenovo_pen_info {
	unsigned char frame_no;
	unsigned char data_type;
	u16 frame_t;
	struct lenovo_pen_coords_buffer coords;
};

struct io_pen_report {
	unsigned char report_num;
	unsigned char reserve[3];
	struct lenovo_pen_info pen_info[MAX_IO_CONTROL_REPORT];
};


//---extern functions---
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len);
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len);
void nvt_bootloader_reset(void);
void nvt_eng_reset(void);
void nvt_sw_reset(void);
void nvt_sw_reset_idle(void);
void nvt_boot_ready(void);
void nvt_bld_crc_enable(void);
void nvt_fw_crc_enable(void);
void nvt_tx_auto_copy_mode(void);
void nvt_read_fw_history(uint32_t fw_history_addr);
int32_t nvt_update_firmware(char *firmware_name);
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
int32_t nvt_get_fw_info(void);
int32_t nvt_clear_fw_status(void);
int32_t nvt_check_fw_status(void);
int32_t nvt_check_spi_dma_tx_info(void);
int32_t nvt_set_page(uint32_t addr);
int32_t nvt_write_addr(uint32_t addr, uint8_t data);

#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

/*Linden code for JLINDEN-11126 by dingying at 20230717 start*/
#if NVT_CUST_PROC_CMD
#if NVT_EDGE_REJECT
extern int32_t nvt_edge_reject_set(int32_t status);
#endif
#if NVT_EDGE_GRID_ZONE
extern int32_t nvt_edge_grid_zone_set(uint8_t deg, uint8_t dir, uint16_t y1, uint16_t y2);
#endif
#if NVT_PALM_MODE
extern int32_t nvt_game_mode_set(uint8_t status);
#endif
#if NVT_SUPPORT_PEN
extern int32_t nvt_support_pen_set(uint8_t status);
#endif
#endif
/*Linden code for JLINDEN-11126 by dingying at 20230717 end*/

#endif /* _LINUX_NVT_TOUCH_H */
