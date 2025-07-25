/* Huaqin  Inc. (C) 2011. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("HUAQIN SOFTWARE")
 * RECEIVED FROM HUAQIN AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. HUAQIN EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES HUAQIN PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE HUAQIN SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN HUAQIN SOFTWARE. HUAQIN SHALL ALSO NOT BE RESPONSIBLE FOR ANY HUAQIN
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND HUAQIN'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE HUAQIN SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT HUAQIN'S OPTION, TO REVISE OR REPLACE THE HUAQIN SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * HUAQIN FOR SUCH HUAQIN SOFTWARE AT ISSUE.
 *
 */

/*******************************************************************************
* Dependency
*******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/hardware_info.h>
#include <linux/regulator/consumer.h>

#include <linux/mm.h>
#include <linux/genhd.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/export.h>

#define HARDWARE_INFO_VERSION   "SM4350"
#define HARDWARE_INFO_WCN       "WCN3998"
#define hwinfo_debug  printk
/******************************************************************************
 * EMMC Configuration
*******************************************************************************/
#define DDR_TYPE "LPDDR4"
static EMMC_VENDOR_TABLE vendor_table[] =
{
    { .id = "11", .name = "Toshiba", },
    { .id = "13", .name = "Micron", },
    { .id = "15", .name = "Samsung", },
    { .id = "45", .name = "SanDisk", },
    { .id = "90", .name = "Hynix", },
    { .id = "00", .name = "NULL", },

};

static int current_type_value = 0;

static BOARD_TYPE_TABLE pcba_type_table[] =
{
    { .type_value = 0,  .pcba_type_name = "Docomo", },
    { .type_value = 1,  .pcba_type_name = "KDDI/SBM", },
    { .type_value = 2,  .pcba_type_name = "RMN/MVNO", },
    { .type_value = 3,  .pcba_type_name = "UNKNOWN", },
    { .type_value = 4,  .pcba_type_name = "UNKNOWN", },
    { .type_value = 5,  .pcba_type_name = "UNKNOWN", },
    { .type_value = 6,  .pcba_type_name = "UNKNOWN", },
    { .type_value = -1, .pcba_type_name = "UNKNOWN", },
};

static int adc_val_tolerance = 100;
static BOARD_STAGE_TABLE pcba_stage_tabe[]=
{
    {  .stage_value = 0,        .pcba_stage_name="PRE_EVT", },     //Board_ID-->PM6150L:GPIO_01
    {  .stage_value = 163,      .pcba_stage_name="EVT", },
    {  .stage_value = 550,      .pcba_stage_name="DVT1", },
    {  .stage_value = 900,      .pcba_stage_name="DVT2", },
    {  .stage_value = 1200,     .pcba_stage_name="PVT", },
    {  .stage_value = 1800,     .pcba_stage_name="MP", },
};

//static struct delayed_work psSetSensorConf_work;

/******************************************************************************
 * Hardware Info Driver
*************************`*****************************************************/
struct global_otp_struct hw_info_main_otp;
struct global_otp_struct hw_info_main2_otp;
struct global_otp_struct hw_info_main3_otp;
struct global_otp_struct hw_info_sub_otp;
static HARDWARE_INFO hwinfo_data;
static char * pcb_name = NULL;

/*
void do_psSetSensorConf_work(struct work_struct *work)
{
    hwinfo_debug("%s: current_stage_value = %d\n",__func__,current_stage_value);
    sensor_set_cmd_to_hub(ID_PROXIMITY, CUST_ACTION_SET_SENSOR_CONF, &current_stage_value);
}
*/

void write_cam_otp_info(enum hardware_id id,struct global_otp_struct *cam_otp)
{
    if(NULL == cam_otp) {
        hwinfo_debug("[HWINFO] %s the data of hwid %d is NULL\n", __func__, id);
    } else {
        switch(id) {
        case  HWID_MAIN_OTP:
            hw_info_main_otp = *cam_otp;
            break;
        case HWID_SUB_OTP:
            hw_info_sub_otp = *cam_otp;
            break;
        case HWID_MAIN_OTP_2:
            hw_info_main2_otp = *cam_otp;
            break;
        case HWID_MAIN_OTP_3:
            hw_info_main3_otp = *cam_otp;
            break;
        default:
            hwinfo_debug("[HWINFO] %s Invalid HWID\n", __func__);
            break;
        }
    }
}


void get_hardware_info_data(enum hardware_id id, const void *data)
{
    if (NULL == data) {
        hwinfo_debug("[HWINFO] %s the data of hwid %d is NULL\n", __func__, id);
    } else {
        switch (id) {
        case HWID_LCM:
            hwinfo_data.lcm = data;
            break;
        case HWID_CTP_DRIVER:
            hwinfo_data.ctp_driver = data;
            break;
        case HWID_CTP_MODULE:
            hwinfo_data.ctp_module = data;
            break;
        case HWID_CTP_FW_VER:
            strcpy(hwinfo_data.ctp_fw_version,data);
            break;
        case HWID_CTP_COLOR_INFO:
            hwinfo_data.ctp_color_info = data;
            break;
        case HWID_CTP_LOCKDOWN_INFO:
            hwinfo_data.ctp_lockdown_info = data;
            break;
        case HWID_CTP_FW_INFO:
            hwinfo_data.ctp_fw_info = data;
            break;
        case HWID_MAIN_CAM:
            hwinfo_data.main_camera = data;
            break;
        case HWID_MAIN_CAM_2:
            hwinfo_data.main_camera2 = data;
            break;
        case HWID_MAIN_CAM_3:
            hwinfo_data.main_camera3 = data;
            break;
        case HWID_SUB_CAM:
            hwinfo_data.sub_camera = data;
            break;
        case HWId_HIFI_NAME:
            hwinfo_data.hifi_name= data;
            break;
        case HWID_FLASH:
            hwinfo_data.flash = data;
            break;
        case HWID_FLASH_SLOT:
            strcpy(hwinfo_data.flash_slot,data);
            break;
        case HWID_ALSPS:
            hwinfo_data.alsps = data;
            break;
        case HWID_GSENSOR:
            hwinfo_data.gsensor = data;
            break;
        case HWID_BAROMETER:
            hwinfo_data.barometer = data;
            break;
        case HWID_GYROSCOPE:
            hwinfo_data.gyroscope = data;
            break;
        case HWID_MSENSOR:
            hwinfo_data.msensor = data;
            break;
        case HWID_SAR_SENSOR_1:
            hwinfo_data.sar_sensor_1 = data;
            break;
        case HWID_SAR_SENSOR_2:
            hwinfo_data.sar_sensor_2 = data;
            break;
        case HWID_BATERY_ID:
            hwinfo_data.bat_id = data;
            break;
        case HWID_NFC:
            hwinfo_data.nfc = data;
            break;
        case HWID_FINGERPRINT:
            hwinfo_data.fingerprint = data;
            break;
        case HWID_FINGERPRINT_SN:
            hwinfo_data.fingerprint_sn = (unsigned char *)data;
            break;
        case HWID_MAIN_CAM_SN:
            strcpy(hwinfo_data.main_cam_sn, data);
            break;
        case HWID_MAIN_CAM_2_SN:
            strcpy(hwinfo_data.main_cam_2_sn, data);
            break;
        case HWID_MAIN_CAM_3_SN:
            strcpy(hwinfo_data.main_cam_3_sn, data);
            break;
        case HWID_SUB_CAM_SN:
            strcpy(hwinfo_data.sub_cam_sn, data);
            break;
        case HWID_SMARTPA:
            hwinfo_data.smartpa = data;
            break;
        case HWID_BATERY_ID_ADC:
            hwinfo_data.bat_id_adc = (int *)data;
            break;
        default:
            hwinfo_debug("[HWINFO] %s Invalid HWID\n", __func__);
            break;
        }
    }
}
EXPORT_SYMBOL(get_hardware_info_data);

static ssize_t show_lcm(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.lcm) {
        return sprintf(buf, "lcd name :%s\n", hwinfo_data.lcm);
    } else {
        return sprintf(buf, "lcd name :Not Found\n");
    }
}

static ssize_t show_hifi(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.hifi_name) {
        return sprintf(buf, "hifi_name: %s\n", hwinfo_data.hifi_name);
    } else {
        return sprintf(buf, "hifi :Not Found\n");
    }
}


static ssize_t show_ctp(struct device *dev, struct device_attribute *attr, char *buf)
{
    if ((NULL != hwinfo_data.ctp_driver) || (NULL != hwinfo_data.ctp_module) || (NULL != hwinfo_data.ctp_fw_version)) {
        return sprintf(buf, "ctp name :%s  fw_ver:%s\n", hwinfo_data.ctp_driver, hwinfo_data.ctp_fw_version);
    } else {
        return sprintf(buf, "ctp name :Not Found\n");
    }
}

static ssize_t show_fingerprint(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.fingerprint) {
        return sprintf(buf, "fingerprint name :%s\n", hwinfo_data.fingerprint);
    } else {
        return sprintf(buf, "fingerprint name :Not Found\n");
    }
}

static ssize_t show_fingerprint_sn(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.fingerprint_sn) {
        return sprintf(buf, "fingerprint_sn :%02x%02x%02x%02x%02x%02x\n", hwinfo_data.fingerprint_sn[0],hwinfo_data.fingerprint_sn[1],hwinfo_data.fingerprint_sn[2],hwinfo_data.fingerprint_sn[3],hwinfo_data.fingerprint_sn[4],hwinfo_data.fingerprint_sn[5]);
    } else {
        return sprintf(buf, "fingerprint_sn :Not Found\n");
    }
}

static ssize_t show_fw_info(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.ctp_fw_info) {
        return sprintf(buf, "%*hp\n", 10, hwinfo_data.ctp_fw_info);
    } else {
        return sprintf(buf, "Invalid\n");
    }
}

static ssize_t show_color_info(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.ctp_color_info) {
        return sprintf(buf, "%s\n", hwinfo_data.ctp_color_info);
    } else {
        return sprintf(buf, "Invalid\n");
    }
}

static ssize_t show_lockdown_info(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.ctp_lockdown_info) {
        return sprintf(buf, "%s\n", hwinfo_data.ctp_lockdown_info);
    } else {
        return sprintf(buf, "Invalid\n");
    }
}


static ssize_t show_main_camera(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hw_info_main_otp.sensor_name)
        hwinfo_data.main_camera = hw_info_main_otp.sensor_name;
    if (NULL != hwinfo_data.main_camera) {
        return sprintf(buf, "main camera :%s\n", hwinfo_data.main_camera);
    } else {
        return sprintf(buf, "main camera :Not Found\n");
    }
}

static ssize_t show_main_camera2(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hw_info_main2_otp.sensor_name)
        hwinfo_data.main_camera2 = hw_info_main2_otp.sensor_name;
    if (NULL != hwinfo_data.main_camera2) {
        return sprintf(buf, "main camera 2 :%s\n", hwinfo_data.main_camera2);
    } else {
        return sprintf(buf, "main camera :Not Found\n");
    }
}

static ssize_t show_main_camera3(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.main_camera3) {
        return sprintf(buf, "main camera 3 :%s\n", hwinfo_data.main_camera3);
    } else {
        return sprintf(buf, "main camera :Not Found\n");
    }
}

static ssize_t show_sub_camera(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hw_info_sub_otp.sensor_name)
        hwinfo_data.sub_camera = hw_info_sub_otp.sensor_name;
    if (NULL != hwinfo_data.sub_camera) {
        return sprintf(buf, "sub camera :%s\n", hwinfo_data.sub_camera);
    } else {
        return sprintf(buf, "sub camera :Not Found\n");
    }
}

static ssize_t show_main_otp(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (hw_info_main_otp.otp_valid) {
        return sprintf(buf, "main otp :Vendor 0x%0x, ModuleCode 0x%x, ModuleVer 0x%x, SW_ver 0x%x, Date %d-%d-%d, VcmVendor 0x%0x, VcmModule 0x%x \n",
                       hw_info_main_otp.vendor_id, hw_info_main_otp.module_code, hw_info_main_otp.module_ver, hw_info_main_otp.sw_ver, hw_info_main_otp.year,
                       hw_info_main_otp.month, hw_info_main_otp.day, hw_info_main_otp.vcm_vendorid, hw_info_main_otp.vcm_moduleid);
    } else {
        return sprintf(buf, "main otp :No Valid OTP\n");
    }
}

static ssize_t show_main2_otp(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (hw_info_main2_otp.otp_valid) {
        return sprintf(buf, "main otp :Vendor 0x%0x, ModuleCode 0x%x, ModuleVer 0x%x, SW_ver 0x%x, Date %d-%d-%d, VcmVendor 0x%0x, VcmModule 0x%x \n",
                       hw_info_main2_otp.vendor_id, hw_info_main2_otp.module_code, hw_info_main2_otp.module_ver, hw_info_main2_otp.sw_ver, hw_info_main2_otp.year,
                       hw_info_main2_otp.month, hw_info_main2_otp.day, hw_info_main2_otp.vcm_vendorid, hw_info_main2_otp.vcm_moduleid);
    } else {
        return sprintf(buf, "main otp :No Valid OTP\n");
    }
}

static ssize_t show_main3_otp(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (hw_info_main3_otp.otp_valid) {
        return sprintf(buf, "main otp :Vendor 0x%0x, ModuleCode 0x%x, ModuleVer 0x%x, SW_ver 0x%x, Date %d-%d-%d, VcmVendor 0x%0x, VcmModule 0x%x \n",
                       hw_info_main3_otp.vendor_id, hw_info_main3_otp.module_code, hw_info_main3_otp.module_ver, hw_info_main3_otp.sw_ver, hw_info_main3_otp.year,
                       hw_info_main3_otp.month, hw_info_main3_otp.day, hw_info_main3_otp.vcm_vendorid, hw_info_main3_otp.vcm_moduleid);
    } else {
        return sprintf(buf, "main otp :No Valid OTP\n");
    }
}

static ssize_t show_sub_otp(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (hw_info_sub_otp.otp_valid) {
        return sprintf(buf, "sub otp :Vendor 0x%0x, ModuleCode 0x%x, ModuleVer 0x%x, SW_ver 0x%x, Date %d-%d-%d, VcmVendor 0x%0x, VcmModule 0x%0x \n",
                       hw_info_sub_otp.vendor_id, hw_info_sub_otp.module_code, hw_info_sub_otp.module_ver, hw_info_sub_otp.sw_ver, hw_info_sub_otp.year,
                       hw_info_sub_otp.month, hw_info_sub_otp.day, hw_info_sub_otp.vcm_vendorid, hw_info_sub_otp.vcm_moduleid);
    } else {
        return sprintf(buf, "sub otp :No Valid OTP\n");
    }
}

#define UFS_VENDOR_NAME "/sys/devices/platform/soc/4804000.ufshc/string_descriptors/manufacturer_name"

static int get_ufs_vendor_name(char* buff_name)
{
    struct file *pfile = NULL;
    mm_segment_t old_fs;
    loff_t pos;

    ssize_t ret = 0;
    char vendor_name[emmc_len];
    memset(vendor_name, 0, sizeof(vendor_name));

    if(buff_name == NULL) {
        return -1;
    }

    pfile = filp_open(UFS_VENDOR_NAME, O_RDONLY, 0);
    if (IS_ERR(pfile)) {
        hwinfo_debug("[HWINFO]: open UFS_VENDOR_NAME file failed!\n");
        goto ERR_0;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;

    ret = vfs_read(pfile, vendor_name, emmc_len, &pos);
    if(ret <= 0) {
        hwinfo_debug("[HWINFO]: read UFS_VENDOR_NAME  file failed!\n");
        goto ERR_1;
    }

    sprintf(buff_name,"%s",vendor_name);

ERR_1:

    filp_close(pfile, NULL);

    set_fs(old_fs);

    return 0;

ERR_0:
    return -1;
}


static unsigned int get_emmc_size(void)
{
    unsigned int emmc_size = 32;
    struct file *pfile = NULL;
    mm_segment_t old_fs;
    loff_t pos;
    ssize_t ret = 0;

    unsigned long long Size_buf=0;
    char buf_size[emmc_len];
    memset(buf_size, 0, sizeof(buf_size));

    pfile = filp_open(emmc_file, O_RDONLY, 0);
    if (IS_ERR(pfile)) {
        hwinfo_debug("[HWINFO]: open emmc size file failed!\n");
        goto ERR_0;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;

    ret = vfs_read(pfile, buf_size, emmc_len, &pos);
    if(ret <= 0) {
        hwinfo_debug("[HWINFO]: read emmc size file failed!\n");
        goto ERR_1;
    }

    Size_buf = simple_strtoull(buf_size, NULL, 0);
    Size_buf >>= 1; //Switch to KB
    emmc_size = (((unsigned int)Size_buf) / 1024) / 1024;

    if (emmc_size > 64) {
        emmc_size = 128;
    } else if (emmc_size > 32) {
        emmc_size = 64;
    } else if (emmc_size > 16) {
        emmc_size = 32;
    } else if (emmc_size > 8) {
        emmc_size = 16;
    } else if (emmc_size > 6) {
        emmc_size = 8;
    } else if (emmc_size > 4) {
        emmc_size = 6;
    } else if (emmc_size > 3) {
        emmc_size = 4;
    } else {
        emmc_size = 0;
    }

ERR_1:

    filp_close(pfile, NULL);

    set_fs(old_fs);

    return emmc_size;

ERR_0:
    return emmc_size;
}

#define K(x) ((x) << (PAGE_SHIFT - 10))
static unsigned int get_ram_size(void)
{
    unsigned int ram_size, temp_size;
    struct sysinfo info;

    si_meminfo(&info);

    temp_size = K(info.totalram) / 1024;
    if (temp_size > 7168) {
        ram_size = 8;
    }  else if (temp_size > 5120) {
        ram_size = 6;
    } else if (temp_size > 3072) {
        ram_size = 4;
    } else if (temp_size > 2048) {
        ram_size = 3;
    } else if (temp_size > 1024) {
        ram_size = 2;
    } else if(temp_size > 512) {
        ram_size = 1;
    } else {
        ram_size = 0;
    }

    return ram_size;
}

static ssize_t show_emmc_size(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%dGB\n", get_emmc_size());
}

static ssize_t show_ram_size(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%dGB\n", get_ram_size());
}

EXPORT_SYMBOL(saved_command_line);
static ssize_t show_modem_id(struct device *dev, struct device_attribute *attr, char *buf)
{
    char *buf_model = saved_command_line;
    char chmodel_id[32] = {0};
    char *chmodelid  = strstr(buf_model, "pcb_board_id=");
    if(chmodelid == NULL) {
        return sprintf(buf, "%s\n", "pcb_board_id=");
    }

    memcpy( chmodel_id, chmodelid, 14);
    return sprintf(buf, "%s\n", chmodel_id);
}

static int sim_tray_det_1_pin = -1;
static int sim_tray_det_2_pin = -1;

static ssize_t show_sim_tray_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    int sim_tray_det_1_pin_val = 0;
    int sim_tray_det_2_pin_val = 0;

    sim_tray_det_1_pin_val =  gpio_get_value(sim_tray_det_1_pin);
    sim_tray_det_2_pin_val =  gpio_get_value(sim_tray_det_2_pin);

    if (sim_tray_det_1_pin_val == 1 && sim_tray_det_2_pin_val == 1)
        return sprintf(buf, "SIM_SD_CARD_TRAY:Yes");
    else
        return sprintf(buf, "SIM_SD_CARD_TRAY:No");
}

#define STAGE_ADC_NAME "/sys/bus/iio/devices/iio:device1/in_voltage_get_stageid_adc_input"    /* Board_ID-->PM6150L:GPIO_01 */
char* get_stage_adc(void)
{
    char tmpbuf[emmc_len];
    ssize_t ret = 0;
    unsigned int i=0;
    unsigned long long tmp_adc_val=0;
    int adc_val=0;
    int adc_val_high=0;
    int adc_val_low=0;
    struct file *pfile = NULL;
    mm_segment_t old_fs;
    loff_t pos;

    pfile = filp_open(STAGE_ADC_NAME, O_RDONLY, 0);
    if (IS_ERR(pfile)) {
        hwinfo_debug("[HWINFO]: open read_stage_adc file failed!\n");
        goto ERR_0;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;

    ret = vfs_read(pfile, tmpbuf, emmc_len, &pos);
    if(ret <= 0) {
        hwinfo_debug("[HWINFO]: read read_stage_adc  file failed!\n");
        goto ERR_1;
    }

    tmp_adc_val = simple_strtoull(tmpbuf, NULL, 0);
    adc_val = (int)(tmp_adc_val / 1000);
    hwinfo_debug("[HWINFO] %s adc_val %d.\n", __func__, adc_val);

    for (i = 0; i < ARRAY_SIZE(pcba_stage_tabe); i++) {
        adc_val_high = pcba_stage_tabe[i].stage_value + adc_val_tolerance;
        adc_val_low = pcba_stage_tabe[i].stage_value - adc_val_tolerance;
        hwinfo_debug("[HWINFO] adc_val_high=%d, adc_val_low=%d, pcb_stage_id=%s\n", adc_val_high, adc_val_low, pcba_stage_tabe[i].pcba_stage_name);
        if ((adc_val >= adc_val_low) && (adc_val <= adc_val_high)) {
            hwinfo_debug("[HWINFO] adc=%d, pcb_stage_id=%s  \r\n", adc_val, pcba_stage_tabe[i].pcba_stage_name);
            filp_close(pfile, NULL);
            set_fs(old_fs);
            return pcba_stage_tabe[i].pcba_stage_name;
        }
    }
    hwinfo_debug("[HWINFO]error pcb_stage_id can not match any stage!!! \r\n");

ERR_1:
    filp_close(pfile, NULL);
    set_fs(old_fs);
ERR_0:
    return "unknown";

}
static ssize_t show_stage_id(struct device *dev, struct device_attribute *attr, char *buf)
{
    hwinfo_debug("[HWINFO] %s success.\n", __func__);
    return sprintf(buf, "%s\n", get_stage_adc());
}

#if 0
static ssize_t show_stage_id(struct device *dev, struct device_attribute *attr, char *buf)
{
    char *buf_stage = saved_command_line;
    char chstage_id[32] = {0};
    char *chstageid  = strstr(buf_stage, "pcb_stage_id=");
    if(chstageid == NULL) {
        return sprintf(buf, "%s\n", "pcb_board_id=");
    }

    memcpy( chstage_id, chstageid, 16);
    return sprintf(buf, "%s\n", chstage_id);
}
#endif

static ssize_t show_flash(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int i = 0;
    unsigned int raw_cid[4] = {0};
    char emmc_cid[32] = {0};
    char ufs_vendor_name[32] = {0};


    if (NULL == hwinfo_data.flash) {
        get_ufs_vendor_name(ufs_vendor_name);
        return sprintf(buf, "flash name :%s %dG +%dG UFS",ufs_vendor_name,get_ram_size(),get_emmc_size());
    }

    memcpy(raw_cid, hwinfo_data.flash, sizeof(raw_cid));
    sprintf(emmc_cid, "%08x%08x%08x", raw_cid[0], raw_cid[1], raw_cid[2]);

    for (i = 0; i < ARRAY_SIZE(vendor_table); i++) {
        if (memcmp(emmc_cid, vendor_table[i].id, EMMC_VENDOR_CMP_SIZE) == 0) {
            return sprintf(buf, "flash name :%s %dGB+%dGB %s\n", vendor_table[i].name, get_ram_size(), get_emmc_size(), DDR_TYPE);
        }
    }

    return sprintf(buf, "flash name :Not Found\n");
}

static ssize_t show_wifi(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "wifi name :%s\n", HARDWARE_INFO_WCN);
}

static ssize_t show_bt(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "bt name :%s\n", HARDWARE_INFO_WCN);
}

static ssize_t show_gps(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "gps name :%s\n", HARDWARE_INFO_WCN);
}

static ssize_t show_fm(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "fm name :%s\n", HARDWARE_INFO_WCN);
}

#define SENSOR_INFO_STORE(var, pbuf)   do {\
                                        static char name[200]; \
                                        snprintf(name, sizeof(name), "%s", pbuf);\
                                        var = name;\
                                        hwinfo_debug("[HWINFO] %s success. %s\n", __func__, pbuf);\
                                    }while(0)

/*Linden code for JLINDEN-287 by xiongle at 20221125 start*/
#define SENSOR_INFO_SHOW(var, pbuf)   do {\
                                        if (NULL != var) {\
                                            return sprintf(pbuf, "%s\n", var);\
                                        } else {\
                                            return sprintf(pbuf, "fail\n");\
                                        }\
                                    }while(0)
/*Linden code for JLINDEN-287 by xiongle at 20221125 end*/

static ssize_t show_alsps(struct device *dev, struct device_attribute *attr, char *buf)
{
    SENSOR_INFO_SHOW(hwinfo_data.alsps, buf);
}

static ssize_t store_alsps(struct device *dev, struct device_attribute *attr,\
                           const char *buf, size_t count)
{
    SENSOR_INFO_STORE(hwinfo_data.alsps, buf);
    return count;
}

static ssize_t show_barometer(struct device *dev, struct device_attribute *attr, char *buf)
{
    SENSOR_INFO_SHOW(hwinfo_data.barometer, buf);
}

static ssize_t store_barometer(struct device *dev, struct device_attribute *attr,\
                           const char *buf, size_t count)
{
    SENSOR_INFO_STORE(hwinfo_data.barometer, buf);
    return count;
}

static ssize_t show_gsensor(struct device *dev, struct device_attribute *attr, char *buf)
{
    SENSOR_INFO_SHOW(hwinfo_data.gsensor, buf);
}
static ssize_t store_gsensor(struct device *dev, struct device_attribute *attr,\
                             const char *buf, size_t count)
{
    SENSOR_INFO_STORE(hwinfo_data.gsensor, buf);
    return count;
}

static ssize_t show_msensor(struct device *dev, struct device_attribute *attr, char *buf)
{
    SENSOR_INFO_SHOW(hwinfo_data.msensor, buf);
}
static ssize_t store_msensor(struct device *dev, struct device_attribute *attr,\
                             const char *buf, size_t count)
{
    SENSOR_INFO_STORE(hwinfo_data.msensor, buf);
    return count;
}

static ssize_t show_gyro(struct device *dev, struct device_attribute *attr, char *buf)
{
    SENSOR_INFO_SHOW(hwinfo_data.gyroscope, buf);
}

static ssize_t store_gyro(struct device *dev, struct device_attribute *attr,\
                          const char *buf, size_t count)
{
    SENSOR_INFO_STORE(hwinfo_data.gyroscope, buf);
    return count;
}

static ssize_t show_sar_sensor_1(struct device *dev, struct device_attribute *attr, char *buf)
{
    SENSOR_INFO_SHOW(hwinfo_data.sar_sensor_1, buf);
}
static ssize_t show_sar_sensor_2(struct device *dev, struct device_attribute *attr, char *buf)
{
    SENSOR_INFO_SHOW(hwinfo_data.sar_sensor_2, buf);
}
static ssize_t store_sar_sensor_1(struct device *dev, struct device_attribute *attr,\
                                  const char *buf, size_t count)
{
    SENSOR_INFO_STORE(hwinfo_data.sar_sensor_1, buf);
    return count;
}
static ssize_t store_sar_sensor_2(struct device *dev, struct device_attribute *attr,\
                                  const char *buf, size_t count)
{
    SENSOR_INFO_STORE(hwinfo_data.sar_sensor_2, buf);
    return count;
}

static ssize_t show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "version:%s\n", HARDWARE_INFO_VERSION);
}

static ssize_t show_bat_id(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.bat_id) {
        return sprintf(buf, "bat_id name :%s\n", hwinfo_data.bat_id);
    } else {
        return sprintf(buf, "bat_id name :Not found\n");
    }
}

static ssize_t show_bat_id_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (NULL != hwinfo_data.bat_id_adc) {
        return sprintf(buf, "bat_id_adc :%d\n", *hwinfo_data.bat_id_adc);
    } else {
        return sprintf(buf, "bat_id_adc voltage :Not found\n");
    }
}

static ssize_t show_nfc(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_NFC_NQ
    if (NULL != hwinfo_data.nfc) {
        return sprintf(buf, "nfc chip_hwid :%s\n", hwinfo_data.nfc);
    } else {
        return sprintf(buf, "nfc name :Not found\n");
    }
#else
    return sprintf(buf, "nfc name :Not support nfc\n");
#endif
}


char* get_type_name(void)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(pcba_type_table); i++) {
        if (current_type_value == pcba_type_table[i].type_value) {
            return pcba_type_table[i].pcba_type_name;
        }
    }

    return pcba_type_table[ARRAY_SIZE(pcba_type_table) - 1].pcba_type_name;
}
EXPORT_SYMBOL(get_type_name);

static ssize_t show_hw_id(struct device *dev, struct device_attribute *attr, char *buf)
{
    hwinfo_debug("[HWINFO] %s success.\n", __func__);
    return sprintf(buf, "PCBA_%s_%s\n", get_type_name(), get_stage_adc());
}

static ssize_t show_main_cam_sn(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (hwinfo_data.main_cam_sn != NULL)
        return sprintf(buf, "MAIN:%s\n", hwinfo_data.main_cam_sn);
    else
        return sprintf(buf, "MAIN:Not_found\n");
}

static ssize_t show_main_cam_2_sn(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (hwinfo_data.main_cam_2_sn != NULL)
        return sprintf(buf, "MAIN2:%s\n", hwinfo_data.main_cam_2_sn);
    else
        return sprintf(buf, "MAIN2:Not_found\n");
}

static ssize_t show_main_cam_3_sn(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (hwinfo_data.main_cam_3_sn != NULL)
        return sprintf(buf, "MAIN3:%s\n", hwinfo_data.main_cam_3_sn);
    else
        return sprintf(buf, "MAIN3:Not_found\n");
}

static ssize_t show_sub_cam_sn(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (hwinfo_data.sub_cam_sn != NULL)
        return sprintf(buf, "SUB:%s\n", hwinfo_data.sub_cam_sn);
    else
        return sprintf(buf, "SUB:Not_found\n");
}

static ssize_t show_smartpa_type(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (hwinfo_data.smartpa != NULL)
        return sprintf(buf, "smartpa:%s\n", hwinfo_data.smartpa);
    else
        return sprintf(buf, "smartpa:Not_found\n");
}

static DEVICE_ATTR(version, 0444, show_version, NULL);
static DEVICE_ATTR(lcm, 0444, show_lcm, NULL);
static DEVICE_ATTR(ctp, 0444, show_ctp, NULL);
static DEVICE_ATTR(ctp_fw, 0444, show_fw_info, NULL);
static DEVICE_ATTR(ctp_color, 0444, show_color_info, NULL);
static DEVICE_ATTR(ctp_lockdown, 0444, show_lockdown_info, NULL);
static DEVICE_ATTR(main_camera, 0444, show_main_camera, NULL);
static DEVICE_ATTR(main_camera2, 0444, show_main_camera2, NULL);
static DEVICE_ATTR(main_camera3, 0444, show_main_camera3, NULL);
static DEVICE_ATTR(sub_camera, 0444, show_sub_camera, NULL);
static DEVICE_ATTR(flash, 0444, show_flash, NULL);
static DEVICE_ATTR(sim_tray_status, 0444, show_sim_tray_status, NULL);
static DEVICE_ATTR(hifi_name, 0444, show_hifi, NULL);
static DEVICE_ATTR(emmc_size, 0444, show_emmc_size, NULL);
static DEVICE_ATTR(ram_size, 0444, show_ram_size, NULL);
static DEVICE_ATTR(modem_id, 0444, show_modem_id, NULL);
static DEVICE_ATTR(stage_id, 0444, show_stage_id, NULL);
static DEVICE_ATTR(gsensor, 0644, show_gsensor, store_gsensor);
static DEVICE_ATTR(msensor, 0644, show_msensor, store_msensor);
static DEVICE_ATTR(alsps, 0644, show_alsps, store_alsps);
static DEVICE_ATTR(barometer, 0644, show_barometer, store_barometer);
static DEVICE_ATTR(gyro, 0644, show_gyro, store_gyro);
static DEVICE_ATTR(wifi, 0444, show_wifi, NULL);
static DEVICE_ATTR(bt, 0444, show_bt, NULL);
static DEVICE_ATTR(gps, 0444, show_gps, NULL);
static DEVICE_ATTR(fm, 0444, show_fm, NULL);
static DEVICE_ATTR(main_otp, 0444, show_main_otp, NULL);
static DEVICE_ATTR(main2_otp, 0444, show_main2_otp, NULL);
static DEVICE_ATTR(main3_otp, 0444, show_main3_otp, NULL);
static DEVICE_ATTR(sub_otp, 0444, show_sub_otp, NULL);
static DEVICE_ATTR(sar, 0644, show_sar_sensor_1, store_sar_sensor_1);
static DEVICE_ATTR(sar_sensor_2, 0644, show_sar_sensor_2, store_sar_sensor_2);
static DEVICE_ATTR(bat_id, 0444, show_bat_id,NULL );
static DEVICE_ATTR(nfc, 0444, show_nfc,NULL );
static DEVICE_ATTR(hw_id, 0444, show_hw_id, NULL);
static DEVICE_ATTR(fingerprint, 0444, show_fingerprint, NULL);
static DEVICE_ATTR(fingerprint_sn, 0444, show_fingerprint_sn, NULL);
static DEVICE_ATTR(main_cam_sn, 0444, show_main_cam_sn, NULL);
static DEVICE_ATTR(main_cam_2_sn, 0444, show_main_cam_2_sn, NULL);
static DEVICE_ATTR(main_cam_3_sn, 0444, show_main_cam_3_sn, NULL);
static DEVICE_ATTR(sub_cam_sn, 0444, show_sub_cam_sn, NULL);
static DEVICE_ATTR(smartpa, 0444, show_smartpa_type, NULL);
static DEVICE_ATTR(bat_id_adc, 0444, show_bat_id_adc,NULL );

static struct attribute *hdinfo_attributes[] = {
    &dev_attr_version.attr,
    &dev_attr_lcm.attr,
    &dev_attr_ctp.attr,
    &dev_attr_ctp_fw.attr,
    &dev_attr_ctp_color.attr,
    &dev_attr_ctp_lockdown.attr,
    &dev_attr_main_camera.attr,
    &dev_attr_main_camera2.attr,
    &dev_attr_main_camera3.attr,
    &dev_attr_sub_camera.attr,
    &dev_attr_flash.attr,
    &dev_attr_sim_tray_status.attr,
    &dev_attr_hifi_name.attr,
    &dev_attr_emmc_size.attr,
    &dev_attr_ram_size.attr,
    &dev_attr_modem_id.attr,
    &dev_attr_stage_id.attr,
    &dev_attr_gsensor.attr,
    &dev_attr_msensor.attr,
    &dev_attr_alsps.attr,
    &dev_attr_barometer.attr,
    &dev_attr_gyro.attr,
    &dev_attr_wifi.attr,
    &dev_attr_bt.attr,
    &dev_attr_gps.attr,
    &dev_attr_fm.attr,
    &dev_attr_main_otp.attr,
    &dev_attr_main2_otp.attr,
    &dev_attr_main3_otp.attr,
    &dev_attr_sub_otp.attr,
    &dev_attr_sar.attr,
    &dev_attr_sar_sensor_2.attr,
    &dev_attr_bat_id.attr,
    &dev_attr_nfc.attr,
    &dev_attr_hw_id.attr,
    &dev_attr_fingerprint.attr,
    &dev_attr_fingerprint_sn.attr,
    &dev_attr_main_cam_sn.attr,
    &dev_attr_main_cam_2_sn.attr,
    &dev_attr_main_cam_3_sn.attr,
    &dev_attr_sub_cam_sn.attr,
    &dev_attr_smartpa.attr,
    &dev_attr_bat_id_adc.attr,
    NULL
};

static struct attribute_group hdinfo_attribute_group = {
    .attrs = hdinfo_attributes
};

static int hw_info_parse_dt(struct device_node *np)
{
    int ret = -1;
    int type_gpio_pin0 = -1;
    int type_gpio_pin1 = -1;
    int type_gpio_pin2 = -1;
    if (np) {
        type_gpio_pin0 = of_get_named_gpio(np, "pcb_type_gpios0", 0);
        type_gpio_pin1 = of_get_named_gpio(np, "pcb_type_gpios1", 0);
        type_gpio_pin2 = of_get_named_gpio(np, "pcb_type_gpios2", 0);

        current_type_value = (gpio_get_value(type_gpio_pin0) << 0) | (gpio_get_value(type_gpio_pin1) << 1) | (gpio_get_value(type_gpio_pin2) << 2) ;
        hwinfo_debug("[HWINFO] current_type_value=%d\n", current_type_value);

        sim_tray_det_1_pin = of_get_named_gpio(np, "sim_tray_det_1", 0);
        hwinfo_debug("[HWINFO] sim_tray_det_1_pin=%d\n", gpio_get_value(sim_tray_det_1_pin));

        sim_tray_det_2_pin = of_get_named_gpio(np, "sim_tray_det_2", 0);
        hwinfo_debug("[HWINFO] sim_tray_det_2_pin=%d\n", gpio_get_value(sim_tray_det_2_pin));

        return 0;
    }
    return ret;
}

static int HardwareInfo_driver_probe(struct platform_device *pdev)
{
    int ret = -1;
    memset(&hwinfo_data, 0, sizeof(hwinfo_data));
    memset(&hw_info_main_otp, 0, sizeof(hw_info_main_otp));
    memset(&hw_info_main2_otp, 0, sizeof(hw_info_main2_otp));
    memset(&hw_info_main3_otp, 0, sizeof(hw_info_main3_otp));
    ret = sysfs_create_group(&(pdev->dev.kobj), &hdinfo_attribute_group);
    if (ret < 0) {
        hwinfo_debug("[HWINFO] sysfs_create_group failed! (ret=%d)\n", ret);
        goto err;
    }

    ret = hw_info_parse_dt(pdev->dev.of_node);
    if (ret < 0) {
        hwinfo_debug("[HWINFO] hw_info_parse_dt failed! (ret=%d)\n", ret);
        goto err;
    }
    pcb_name = get_type_name();
    if(pcb_name && strcmp(pcb_name, "UNKNOWN") == 0) {
        hwinfo_debug("byron:  UNKNOWN pcb name \n");
    }

err:
    return ret;
}

static int HardwareInfo_driver_remove(struct platform_device *pdev)
{
    sysfs_remove_group(&(pdev->dev.kobj), &hdinfo_attribute_group);

    return 0;
}

static const struct of_device_id hwinfo_dt_match[] = {
    { .compatible = "huaqin,hardware_info", },
    { },
};

static struct platform_driver HardwareInfo_driver = {
    .probe = HardwareInfo_driver_probe,
    .remove = HardwareInfo_driver_remove,
    .driver = {
        .name = "HardwareInfo",
        .of_match_table = hwinfo_dt_match,
    },
};

static int __init HardwareInfo_mod_init(void)
{
    int ret = -1;
    hwinfo_debug("shuai %s %d",__func__,__LINE__);
    ret = platform_driver_register(&HardwareInfo_driver);
    if (ret) {
        hwinfo_debug("[HWINFO] HardwareInfo_driver registed failed! (ret=%d)\n", ret);
    }

    return ret;
}

static void __exit HardwareInfo_mod_exit(void)
{
    platform_driver_unregister(&HardwareInfo_driver);
}


module_init(HardwareInfo_mod_init);
module_exit(HardwareInfo_mod_exit);

MODULE_AUTHOR("Wang zhuquan ");
MODULE_DESCRIPTION("Huaqin Hareware Info driver");
MODULE_LICENSE("GPL");


