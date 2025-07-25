/*
 * Huaqin  Inc. (C) 2011. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __HARDWARE_INFO_H__
#define __HARDWARE_INFO_H__

#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#define emmc_file "/sys/devices/platform/soc/4804000.ufshc/geometry_descriptor/raw_device_capacity"
#define emmc_len  18
#define EMMC_VENDOR_CMP_SIZE  2

typedef struct
{
    char  *id;
    char  *name;
} EMMC_VENDOR_TABLE;

typedef struct
{
    int  stage_value;
    char  *pcba_stage_name;
} BOARD_STAGE_TABLE;

typedef struct
{
    int  type_value;
    int  adc_value;
    char  *pcba_type_name;
} BOARD_TYPE_TABLE;

typedef struct
{
    unsigned int  adc_vol;
    char  *revision;
} BOARD_VERSION_TABLE;

typedef struct {
    int stage_value;
    int adc_value;
} SMEM_BOARD_INFO_DATA;

enum hardware_id {
    HWID_NONE = 0x00,
    HWID_DDR = 0x10,
    HWID_EMMC,
    HWID_NAND,
    HWID_FLASH,
    HWID_FLASH_SLOT,

    HWID_LCM = 0x20,
    HWID_LCD_BIAS,
    HWID_BACKLIGHT,
    HWID_CTP_DRIVER,
    HWID_CTP_MODULE,
    HWID_CTP_FW_VER,
    HWID_CTP_FW_INFO,
	HWID_CTP_COLOR_INFO,
	HWID_CTP_LOCKDOWN_INFO,

    HWID_MAIN_CAM = 0x30,
    HWID_MAIN_CAM_2,
    HWID_MAIN_CAM_3,
    HWID_SUB_CAM,
    HWID_SUB_CAM_2,
    HWID_MAIN_LENS,
    HWID_MAIN_LENS_2,
    HWID_SUB_LENS,
    HWID_SUB_LENS_2,
    HWID_MAIN_OTP,
    HWID_MAIN_OTP_2,
    HWID_MAIN_OTP_3,
    HWID_SUB_OTP,
    HWID_SUB_OTP_2,
    HWID_FLASHLIGHT,
    HWID_FLaSHLIGHT_2,

    HWId_HIFI_NAME,

    HWID_GSENSOR = 0x70,
    HWID_ALSPS,
    HWID_GYROSCOPE,
    HWID_MSENSOR,
    HWID_FINGERPRINT,
    HWID_SAR_SENSOR_1,
    HWID_SAR_SENSOR_2,
    HWID_IRDA,
    HWID_BAROMETER,
    HWID_PEDOMETER,
    HWID_HUMIDITY,
    HWID_NFC,
    HWID_TEE,

    HWID_BATERY_ID = 0xA0,
    HWID_CHARGER,
// add for batt id adc voltage hardware info add begin
    HWID_BATERY_ID_ADC,
// add for batt id adc voltage hardware info add end

    HWID_USB_TYPE_C = 0xE0,

    HWID_SUMMARY = 0xF0,
    HWID_VER,
    HWID_MAIN_CAM_SN,
    HWID_MAIN_CAM_2_SN,
    HWID_MAIN_CAM_3_SN,
    HWID_SUB_CAM_SN,
//fanjiafeng5  add for pdx213 fingerprint sn add begin
    HWID_FINGERPRINT_SN,
//fanjiafeng5 add end
    HWID_SMARTPA,
    HWID_END
};

//Add for camera otp information
struct global_otp_struct {
/*simon modified to show source camera for factory camera mixture in zal1806 start*/
	char *sensor_name;
/*simon modified to show source camera for factory camera mixture in zal1806 end*/
	int otp_valid;
	int vendor_id;
	int module_code;
	int module_ver;
	int sw_ver;
	int year;
	int month;
	int day;
	int vcm_vendorid;
	int vcm_moduleid;
};

typedef struct {
    const char *version;
    const char *lcm;
    const char *ctp_driver;
    const char *ctp_module;
    unsigned char ctp_fw_version[20];
    const char *ctp_fw_info;
	const char *ctp_color_info;
	const char *ctp_lockdown_info;
    const char *main_camera;
    const char *main_camera2;
    const char *main_camera3;
    const char *sub_camera;
    const char * hifi_name;
    const char *alsps;
    const char *barometer;
    const char *gsensor;
    const char *gyroscope;
    const char *msensor;
    const char *fingerprint;
    unsigned char *fingerprint_sn;
    const char *sar_sensor_1;
    const char *sar_sensor_2;
    const char *bat_id;
    int *bat_id_adc;
    const unsigned int *flash;
    const char *nfc;
    unsigned char main_cam_sn[20];
    unsigned char main_cam_2_sn[20];
    unsigned char main_cam_3_sn[20];
    unsigned char sub_cam_sn[20];
    unsigned char flash_slot[20];
	const char *smartpa;
    //const struct hwinfo_cam_otp *main_otp;
    //const struct hwinfo_cam_otp *sub_otp;
} HARDWARE_INFO;

void get_hardware_info_data(enum hardware_id id, const void *data);
void write_cam_otp_info(enum hardware_id id,struct global_otp_struct *cam_otp);
char* get_type_name(void);

#endif

