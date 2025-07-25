/*
 * Copyright (C) 2015 MediaTek Inc.
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

#include "hqsys_misc.h"

#if 1
#include <asm-generic/gpio.h>



#define SDCARD_CD_PIN_GPIO 449 /* 400000.pinctrl: GPIOs 355 to 511 : 355+94*/
#define ACTIVE_LEVEL_TF 1 /*set sdcard hotplug irq high or low is active*/
#endif

//int card_tray_plug = 0;

/*MISC_INFO(MISC_EMMC_SIZE,emmc_size);*/
MISC_INFO(MISC_RAM_SIZE,ram_size);
MISC_INFO(MISC_BOOT_MODE,boot_mode);
MISC_INFO(MISC_OTP_SN,otp_sn);
MISC_INFO(MISC_CARD_TRAY,card_tray);

/*static struct delayed_work get_emmc_info_work;
extern unsigned int msdc_get_capacity(int get_emmc_total);
extern char *get_emmc_name(void);*/

/*extern int msdc_emmc_get_prod_name(char *emmc_name);
static int emmc_get_emmc_name(char *emmc_name)
{
	char emmc_prod_name[8] = {0};

	if (0 == msdc_emmc_get_prod_name(emmc_prod_name)) {
		if(strncmp(emmc_prod_name,"4X6KMB",6) == 0) {
			strcpy(emmc_name,"SAMSUNG KM4X6001KM_B321");
		}
		else if(strncmp(emmc_prod_name,"GNBLQA",6) == 0) {
			strcpy(emmc_name,"KINGSTON 32EMCP16_ML4ATB29_A91");
		} else {
			pr_err("%s:emmc_prod_name is valid and is %s\n",__func__,emmc_prod_name);
			return -1;
		}
	} else {
		return -1;
	}

	return 0;
}

static char emmc_name[80] = {0};
void get_emmc_info_work_func(struct work_struct *work) {

	if (0 == emmc_get_emmc_name(emmc_name)) {
		hq_register_hw_info(HWID_EMMC,emmc_name);
	}
}*/

unsigned int round_kbytes_to_mbytes(unsigned int k){
	unsigned int r_size_m = 0;
	unsigned int in_mega = k/1024;

	if(in_mega > 64*1024){ //if memory is larger than 64G
		r_size_m = 128*1024; // It should be 128G
	}else if(in_mega > 32*1024){  //larger than 32G
		r_size_m = 64*1024; //should be 64G
	}else if(in_mega > 16*1024){
		r_size_m = 32*1024;
	}else if(in_mega > 8*1024){
		r_size_m = 16*1024;
	}else if(in_mega > 6*1024){
		r_size_m = 8*1024;
	}else if(in_mega > 4*1024){
		r_size_m = 6*1024;  //RAM may be 6G
	}else if(in_mega > 3*1024){
		r_size_m = 4*1024;
	}else if(in_mega > 2*1024){
		r_size_m = 3*1024; //RAM may be 3G
	}else if(in_mega > 1024){
		r_size_m = 2*1024;
	}else if(in_mega > 512){
		r_size_m = 1024;
	}else if(in_mega > 256){
		r_size_m = 512;
	}else if(in_mega > 128){
		r_size_m = 256;
	}else{
		k = 0;
	}
	return r_size_m;
}
/*
ssize_t hq_emmcinfo(char * buf)
{
	ssize_t count = -1;
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	loff_t pos;
        ssize_t ret = 0;

	unsigned long long Size_buf=0;
	char buf_size[qcom_emmc_len];
	memset(buf_size,0,sizeof(buf_size));

	pfile = filp_open(qcom_emmc,O_RDONLY,0);
	if (IS_ERR(pfile)) {
	    goto ERR_0;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	ret=vfs_read(pfile, buf_size, qcom_emmc_len, &pos);
	if(ret <=0 )
	{
		goto ERR_1;
	}

	Size_buf=simple_strtoull(buf_size,NULL,0);

	Size_buf>>=1; //Switch to KB


	count=sprintf(buf,"%dGB",round_kbytes_to_mbytes((unsigned int)Size_buf)/1024);

ERR_1:

	filp_close(pfile, NULL);

	set_fs(old_fs);

	return count;

ERR_0:
	return count;

}*/

static struct attribute *hq_misc_attrs[] = {
      /*&misc_info_emmc_size.attr, */
	&misc_info_ram_size.attr,
	&misc_info_boot_mode.attr,
	&misc_info_otp_sn.attr,
	&misc_info_card_tray.attr,
	NULL
};

#if 0
extern int hq_read_sn_from_otp(char *sn);
extern int hq_write_sn_to_otp(char *sn,unsigned int len);
#endif
#define SN_LEN (12) //for B6H Nikeh

static ssize_t hq_misc_show(struct kobject *kobj, struct attribute *a, char *buf){
	ssize_t count = 0;

	struct misc_info *mi = container_of(a, struct misc_info , attr);

	switch(mi->m_id){
		case MISC_RAM_SIZE:
			{
				#define K(x) ((x) << (PAGE_SHIFT - 10))
				struct sysinfo i;
				si_meminfo(&i);
				//count=sprintf(buf,"%u",(unsigned int)K(i.totalram));

				if(round_kbytes_to_mbytes(K(i.totalram)) >= 1024){
					count = sprintf(buf,"%dGB",round_kbytes_to_mbytes(K(i.totalram))/1024);
				}else{
					count = sprintf(buf,"%dMB",round_kbytes_to_mbytes(K(i.totalram)));
				}

			}
			break;
		/*case MISC_EMMC_SIZE:
			//count = sprintf(buf,"%dGB",round_kbytes_to_readable_mbytes(msdc_get_capacity(1)/2)/1024);
			        count=hq_emmcinfo(buf);
			break;*/
		case MISC_OTP_SN:
#if 0    //#ifdef CONFIG_MTK_EMMC_SUPPORT_OTP
            {
                char temp[SN_LEN+1] = {0};
                int result = 0;

                int i=0;

                result = hq_read_sn_from_otp(temp);

                if(0 == result){
                    //#if 0
                    //check if alpha and num
                    for(i=0; i<SN_LEN; i++){
                        if(!isalnum(temp[i])){
                            count = sprintf(buf,"Not Valid SN\n");
                            goto r_error;
                        }
                    }
                    //#endif
                    count = sprintf(buf,"%s",temp);
                }else{
                    count = sprintf(buf,"Read SN in OTP error %d\n",result);
                }
            }

#else
            count = sprintf(buf,"SN in OTP not enabled\n");
#endif
		    break;
		case MISC_CARD_TRAY:
			if (ACTIVE_LEVEL_TF == __gpio_get_value(SDCARD_CD_PIN_GPIO)) {
				count = sprintf(buf,"1");
			} else {
				count = sprintf(buf,"0");
			}
			break;
		default:
			count = sprintf(buf,"Not support");
			break;
	}

//r_error:
	return count;
}

static ssize_t hq_misc_store(struct kobject *kobj, struct attribute *a, const char *buf, size_t count){

	struct misc_info *mi = container_of(a, struct misc_info , attr);

	switch(mi->m_id){
#if 0     //#ifdef CONFIG_MTK_EMMC_SUPPORT_OTP
		case MISC_OTP_SN:
		    {
		        char temp[SN_LEN+1] = {0};
		        int result = 0;
		        int i = 0;

		        if(0 != strncmp(buf,"SN:=",4)){
		            printk("[%s] invalid write sn command\n",__func__);
		            break;
		        }
		        for(i=0;i<SN_LEN;i++){
		            temp[i] = buf[i+4];
		            if(('\n' == buf[i+4]) || ('\r' == buf[i+4])){
                        temp[i] = 0;
                        break;
		            }
		        }


		        result = hq_write_sn_to_otp(temp,strlen(temp));
		            if(0 != result)
		                printk("[%s] called write error %d\n",__func__,result);

		    }
		    break;
#endif
		default:
			break;
	}
	return count;
}

/* hq_misc object */
static struct kobject hq_misc_kobj;
static const struct sysfs_ops hq_misc_sysfs_ops = {
	.show = hq_misc_show,
	.store = hq_misc_store,
};

/* hq_misc type */
static struct kobj_type hq_misc_ktype = {
	.sysfs_ops = &hq_misc_sysfs_ops,
	.default_attrs = hq_misc_attrs
};


static int __init create_misc(void){
	int ret;

	/* add kobject */
	ret = register_kboj_under_hqsysfs(&hq_misc_kobj, &hq_misc_ktype, HUAQIN_MISC_NAME);
	if (ret < 0) {
		pr_err("%s fail to add hq_misc_kobj\n",__func__);
		return ret;
	}
	return 0;
}



static int __init hq_misc_sys_init(void)
{
	/* create sysfs entry at /sys/class/hq_misc/interface/misc */
	create_misc();
	/*INIT_DELAYED_WORK(&get_emmc_info_work,get_emmc_info_work_func);
	schedule_delayed_work(&get_emmc_info_work, msecs_to_jiffies(4000));*/
	return 0;
}


late_initcall(hq_misc_sys_init);
MODULE_AUTHOR("KaKa Ni <nigang@hq_misc.com>");
MODULE_DESCRIPTION("Huaqin Hardware Info Driver");
MODULE_LICENSE("GPL");
