#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include "wl2868c.h"
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/unaligned.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>


#define WL2868C_ID  0x82
#define WL2868C_ID1  0x33
#define WL2868C_ID_ADRR 0X00
#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2

struct wl2866d_chip *camera_chip;
struct mutex wl2866d_mutex;
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
static unsigned ldo5_count = 0;
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/

static const struct  wl2866d_map  wl2866d_on_config[] = {
	{0x03, 0x5A},    //LDO1   MAIN-DVDD  1.2 V
	{0x04, 0x62},    //LDO2   FRONT-DVDD 1.2 V add 0.04v
	{0x05, 0xA2},    //LDO3   AF-VDD     2.8 V
	{0x06, 0xA2},    //LDO4   MAIN-AVDD  2.8 v
	{0x07, 0x2C},    //LDO5   DOVDD      1.8 v add 0.056v
	{0x08, 0xe1},    //LDO6   FP         3.3 v
	{0x09, 0xA2},    //LDO7   FRONT-AVDD 2.8 v
	{0x03, 0x4D},    //LDO1   MAIN-DVDD  1.1V
	{0x0E, 0xFF},    //enable
	{0x0E, 0x80},    //disable
};


static int wl2866d_i2c_write(struct wl2866d_chip *chip,
			    unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret =
		    i2c_smbus_write_byte_data(chip->client, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int wl2866d_i2c_read(struct wl2866d_chip *chip,
			   unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(chip->client, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
				ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

int wl2866d_camera_power_down_all(void)
{
    int ret = -1;
	ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, 0);//bit1
	if (ret < 0) {
		pr_err("wl2866d set enable failed\n");
		return ret;
	}
    return ret;
}

//0x0e bit0:LOD1, bit1:LDO2, bit2:LDO3, bit3:LDO4
//     bit4:LOD5, bit5:LDO6, bit6:LDO7, bit7:REV
//{0x03, 0x64}, OUT_LDO1
//{0x04, 0x4B}, OUT_LDO2
//{0x05, 0x80}, OUT_LDO3
//{0x06, 0x78}, OUT_LDO4
//{0x07, 0x0F}, OUT_LDO5
//{0x08, 0x00}, OUT_LDO6
//{0x09, 0x00}, OUT_LDO7
//{0x0E, 0x0F}, ENABLE
//{0x0E, 0x00}, DISABLE
int wl2866d_camera_power_up(int out_iotype)
{
	int ret = -1;
	unsigned char reg_val = 0;

	if (camera_chip == NULL) {
		return ret;
	}
	switch (out_iotype) {
	case OUT_LDO1:
		mutex_lock(&wl2866d_mutex);
		pr_err("walf wl2866d OUT_LDO1: wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[OUT_LDO1].reg, wl2866d_on_config[OUT_LDO1].value);//bit0
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set ldo1 failed\n");
			break;
		}

		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d read ldo1 enable failed\n");
			break;
		}

		//pr_err("wl2866d ldo1 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0001;//bit 0
		//pr_err("wl2866d ldo1 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit0
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set ldo1 enable failed\n");
			break;
		}
		pr_err("wl2866d set LDO1 success!");
		mutex_unlock(&wl2866d_mutex);
		break;

case OUT_LDO1P1:
		mutex_lock(&wl2866d_mutex);
		pr_err("walf wl2866d OUT_LDO1P1: wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[OUT_LDO1P1].reg, wl2866d_on_config[OUT_LDO1P1].value);//bit0
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set ldo1 failed\n");
			break;
		}

		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d read ldo1 enable failed\n");
			break;
		}

		//pr_err("wl2866d ldo1 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0001;//bit 0
		//pr_err("wl2866d ldo1 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit0
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set ldo1 enable failed\n");
			break;
		}
		pr_err("wl2866d set LDO1 success!");
		mutex_unlock(&wl2866d_mutex);
		break;

	case OUT_LDO2:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d ldo2: wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[OUT_LDO2].reg, wl2866d_on_config[OUT_LDO2].value);//bit1
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set ldo2 failed\n");
			break;
		}

		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo2 read enable failed\n");
			break;
		}
		//pr_err("wl2866d ldo2 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0010;//bit1
		//pr_err("wl2866d ldo2 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo2 set enable failed\n");
			break;
		}
		pr_err("wl2866d set LDO2 success!");
		mutex_unlock(&wl2866d_mutex);
		break;

	case OUT_LDO3:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d LDO3: wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[OUT_LDO3].reg, wl2866d_on_config[OUT_LDO3].value);//bit2
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set ldo3 failed\n");
			break;
		}

		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d read enable failed\n");
			break;
		}

		//pr_err("wl2866d LDO3 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0100;//bit2
		//pr_err("wl2866d LDO3 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit2
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo3 set enable failed\n");
			break;
		}
		pr_err("wl2866d set LDO3 success!");
		mutex_unlock(&wl2866d_mutex);
		break;

	case OUT_LDO4:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d LDO4 : wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[OUT_LDO4].reg, wl2866d_on_config[OUT_LDO4].value);//bit3
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set ldo4 failed\n");
			break;
		}

		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d read enable failed\n");
			break;
		}

		//pr_err("wl2866d LDO4 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b1000;//bit3
		//pr_err("wl2866d LDO4 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit3
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set enable failed\n");
			break;
		}
		pr_err("wl2866d set LDO4 success!");
		mutex_unlock(&wl2866d_mutex);
		break;
	case OUT_LDO5:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d LDO5 : wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[OUT_LDO5].reg, wl2866d_on_config[OUT_LDO5].value);//bit3
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set LDO5 failed\n");
			break;
		}

		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO5 read enable failed\n");
			break;
		}

		//pr_err("wl2866d LDO5 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b10000;//bit4
		//pr_err("wl2866d LDO5 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit4
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO5 set enable failed\n");
			break;
		}
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
		ldo5_count++;
		pr_err("wl2866d ldo 5 power up count = %d", ldo5_count);
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/
		pr_err("wl2866d set LDO5 success!");
		mutex_unlock(&wl2866d_mutex);
		break;
	case OUT_LDO6:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d LDO6 : wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[OUT_LDO6].reg, wl2866d_on_config[OUT_LDO6].value);//bit3
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set LDO6 failed\n");
			break;
		}

		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO6 read enable failed\n");
			break;
		}

		//pr_err("wl2866d LDO6 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b100000;//bit5
		//pr_err("wl2866d LDO6 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit5
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO6 set enable failed\n");
			break;
		}
		pr_err("wl2866d set LDO6 success!");
		mutex_unlock(&wl2866d_mutex);
		break;
	case OUT_LDO7:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d LDO7 : wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[OUT_LDO7].reg, wl2866d_on_config[OUT_LDO7].value);//bit3
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO7 set avdd1 failed\n");
			break;
		}

		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO7 read enable failed\n");
			break;
		}

		//pr_err("wl2866d ldo7 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b1000000;//bit6
		//pr_err("wl2866d ldo7 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit6
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo7 set enable failed\n");
			break;
		}
		pr_err("wl2866d set LDO7 success!");
		mutex_unlock(&wl2866d_mutex);
		break;
	default:
		pr_err("wl2866d unknown port!!!\n");
		break;
	}
	pr_err("wl2866d   result = %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(wl2866d_camera_power_up);

int wl2866d_camera_power_down(int out_iotype)
{
	int ret = -1;
	unsigned char reg_val = 0;

	if (camera_chip == NULL) {
		return ret;
	}
	switch (out_iotype) {
	case OUT_LDO1:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d ldo1: wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo1 read enable failed\n");
			break;
		}
		//pr_err("wl2866d ldo1 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11111110;//bit0
		//pr_err("wl2866d ldo1 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit0
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo1 set enable failed\n");
			break;
		}
		pr_err("wl2866d disable ldo1 success!");
		mutex_unlock(&wl2866d_mutex);
		break;

	case OUT_LDO1P1:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d ldo1: wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo1 read enable failed\n");
			break;
		}
		//pr_err("wl2866d ldo1 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11111110;//bit0
		//pr_err("wl2866d ldo1 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit0
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo1 set enable failed\n");
			break;
		}
		pr_err("wl2866d disable ldo1 success!");
		mutex_unlock(&wl2866d_mutex);
		break;

	case OUT_LDO2:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d ldo2: wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo2 read enable failed\n");
			break;
		}

		//pr_err("wl2866d ldo2 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11111101;//bit1
		//pr_err("wl2866d ldo2 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo2 set enable failed\n");
			break;
		}
		pr_err("wl2866d disable ldo2 success!");
		mutex_unlock(&wl2866d_mutex);
		break;

	case OUT_LDO3:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d LDO3: wl2866 out put type is [%d]", out_iotype);
		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO3 read enable failed\n");
			break;
		}

		//pr_err("wl2866d LDO3 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11111011;//bit2
		//pr_err("wl2866d LDO3 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit2
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo3 set enable failed\n");
			break;
		}
		pr_err("wl2866d disable LDO3 success!");
		mutex_unlock(&wl2866d_mutex);
		break;

	case OUT_LDO4:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d LDO4: wl2866 output iotype is [%d]", out_iotype);
		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO4 read enable failed\n");
			break;
		}

		//pr_err("wl2866d LDO4 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11110111;//bit3
		//pr_err("wl2866d LDO4 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d ldo4 set enable failed\n");
			break;
		}
		pr_err("wl2866d disable LDO4 success!");
		mutex_unlock(&wl2866d_mutex);
		break;
	case OUT_LDO5:
		mutex_lock(&wl2866d_mutex);
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
		if (ldo5_count > 1) {
			ldo5_count--;
			pr_err("wl2866d ldo 5 power down count = %d", ldo5_count);
			mutex_unlock(&wl2866d_mutex);
			ret = 0;
			break;
		}
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/
		//pr_err("wl2866d LDO5: wl2866 output iotype is [%d]", out_iotype);
		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO5 read enable failed\n");
			break;
		}

		//pr_err("wl2866d LDO5 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11101111;//bit4
		//pr_err("wl2866d LDO5 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit4
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d set enable failed\n");
			break;
		}
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
		ldo5_count--;
		pr_err("wl2866d ldo 5 power down count = %d", ldo5_count);
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/
		pr_err("wl2866d disable LDO5 success!");
		mutex_unlock(&wl2866d_mutex);
		break;
	case OUT_LDO6:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d LDO6: wl2866 output iotype is [%d]", out_iotype);
		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO6 read enable failed\n");
			break;
		}

		//pr_err("wl2866d LDO6 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11011111;//bit5
		//pr_err("wl2866d LDO6 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit5
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO6 set enable failed\n");
			break;
		}
		pr_err("wl2866d disable LDO6 success!");
		mutex_unlock(&wl2866d_mutex);
		break;
	case OUT_LDO7:
		mutex_lock(&wl2866d_mutex);
		//pr_err("wl2866d LDO7: wl2866 output iotype is [%d]", out_iotype);
		ret = wl2866d_i2c_read(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO7 read enable failed\n");
			break;
		}

		//pr_err("wl2866d LDO7 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b10111111;//bit6
		//pr_err("wl2866d LDO7 after  set enable value = 0x%x\n", reg_val);

		ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);//bit6
		if (ret < 0) {
			mutex_unlock(&wl2866d_mutex);
			pr_err("wl2866d LDO7 set enable failed\n");
			break;
		}
		pr_err("wl2866d disable LDO7 success!");
		mutex_unlock(&wl2866d_mutex);
		break;
	default:
		pr_err("wl2866d unknown camera!!!\n");
		break;
	}
	pr_err("wl2866d result = %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(wl2866d_camera_power_down);


static ssize_t wl2866d_vol_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wl2866d_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	wl2866d_i2c_read(chip, wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t wl2866d_vol_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct wl2866d_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = wl2866d_i2c_write(chip, wl2866d_on_config[VOL_ENABLE].reg, reg_val);
	if (ret < 0) {
		pr_err("wl2866d set enable failed\n");
		return ret;
	}
	return count;
}


static ssize_t wl2866d_out_ldo1_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wl2866d_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	wl2866d_i2c_read(chip, wl2866d_on_config[OUT_LDO1].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t wl2866d_out_ldo1_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct wl2866d_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = wl2866d_i2c_write(chip, wl2866d_on_config[OUT_LDO1].reg, reg_val);
	if (ret < 0) {
		pr_err("wl2866d open ldo1 failed\n");
		return ret;
	}
	return count;
}

static ssize_t wl2866d_out_ldo1p1_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wl2866d_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	wl2866d_i2c_read(chip, wl2866d_on_config[OUT_LDO1P1].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t wl2866d_out_ldo1p1_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct wl2866d_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = wl2866d_i2c_write(chip, wl2866d_on_config[OUT_LDO1P1].reg, reg_val);
	if (ret < 0) {
		pr_err("wl2866d open ldo1 failed\n");
		return ret;
	}
	return count;
}


static ssize_t wl2866d_out_ldo2_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wl2866d_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	wl2866d_i2c_read(chip, wl2866d_on_config[OUT_LDO2].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t wl2866d_out_ldo2_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct wl2866d_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = wl2866d_i2c_write(chip, wl2866d_on_config[OUT_LDO2].reg, reg_val);
	if (ret < 0)	{
		pr_err("wl2866d open ldo2 failed\n");
		return ret;
	}
	return count;
}


static ssize_t wl2866d_out_ldo3_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wl2866d_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	wl2866d_i2c_read(chip, wl2866d_on_config[OUT_LDO3].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t wl2866d_out_ldo3_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct wl2866d_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = wl2866d_i2c_write(chip, wl2866d_on_config[OUT_LDO3].reg, reg_val);
	if (ret < 0)	{
		pr_err("wl2866d open ldo3 failed\n");
		return ret;
	}
	return count;
}


static ssize_t wl2866d_out_ldo4_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wl2866d_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	wl2866d_i2c_read(chip, wl2866d_on_config[OUT_LDO4].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t wl2866d_out_ldo4_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct wl2866d_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;
	ret = wl2866d_i2c_write(chip, wl2866d_on_config[OUT_LDO4].reg, reg_val);
	if (ret < 0)	{
		pr_err("wl2866d open ldo4 failed\n");
		return ret;
	}
	return count;
}

static ssize_t wl2866d_out_ldo5_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wl2866d_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	wl2866d_i2c_read(chip, wl2866d_on_config[OUT_LDO5].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t wl2866d_out_ldo5_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct wl2866d_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;
	ret = wl2866d_i2c_write(chip, wl2866d_on_config[OUT_LDO5].reg, reg_val);
	if (ret < 0)    {
		pr_err("wl2866d open ldo5 failed\n");
		return ret;
	}
	return count;
}

static ssize_t wl2866d_out_ldo6_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wl2866d_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	wl2866d_i2c_read(chip, wl2866d_on_config[OUT_LDO6].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t wl2866d_out_ldo6_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct wl2866d_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;
	ret = wl2866d_i2c_write(chip, wl2866d_on_config[OUT_LDO6].reg, reg_val);
	if (ret < 0)    {
		pr_err("wl2866d open ldo6 failed\n");
		return ret;
	}
	return count;
}

static ssize_t wl2866d_out_ldo7_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct wl2866d_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	wl2866d_i2c_read(chip, wl2866d_on_config[OUT_LDO7].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t wl2866d_out_ldo7_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct wl2866d_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;
	ret = wl2866d_i2c_write(chip, wl2866d_on_config[OUT_LDO7].reg, reg_val);
	if (ret < 0)    {
		pr_err("wl2866d open ldo7 failed\n");
		return ret;
	}
	return count;
}
static DEVICE_ATTR(vol_enable, 0664, wl2866d_vol_enable_show, wl2866d_vol_enable_store);
static DEVICE_ATTR(out_ldo1, 0664, wl2866d_out_ldo1_show, wl2866d_out_ldo1_store);
static DEVICE_ATTR(out_ldo1p1, 0664, wl2866d_out_ldo1p1_show, wl2866d_out_ldo1p1_store);
static DEVICE_ATTR(out_ldo2, 0664, wl2866d_out_ldo2_show, wl2866d_out_ldo2_store);
static DEVICE_ATTR(out_ldo3, 0664, wl2866d_out_ldo3_show, wl2866d_out_ldo3_store);
static DEVICE_ATTR(out_ldo4, 0664, wl2866d_out_ldo4_show, wl2866d_out_ldo4_store);
static DEVICE_ATTR(out_ldo5, 0664, wl2866d_out_ldo5_show, wl2866d_out_ldo5_store);
static DEVICE_ATTR(out_ldo6, 0664, wl2866d_out_ldo6_show, wl2866d_out_ldo6_store);
static DEVICE_ATTR(out_ldo7, 0664, wl2866d_out_ldo7_show, wl2866d_out_ldo7_store);

static struct attribute *wl2866d_attributes[] = {
	&dev_attr_out_ldo1.attr,
	&dev_attr_out_ldo1p1.attr,
	&dev_attr_out_ldo2.attr,
	&dev_attr_out_ldo3.attr,
	&dev_attr_out_ldo4.attr,
	&dev_attr_out_ldo5.attr,
	&dev_attr_out_ldo6.attr,
	&dev_attr_out_ldo7.attr,
	&dev_attr_vol_enable.attr,
	NULL
};

static struct attribute_group wl2866d_attribute_group = {
	.attrs = wl2866d_attributes
};

static int wl2866d_is_ldo_probed(struct  wl2866d_chip *chip)
{
        int ret = 0;

        chip->en_gpio = of_get_named_gpio(chip->dev->of_node,
                        "en-gpio", 0);
        if (!gpio_is_valid(chip->en_gpio)) {
                pr_err("%s:%d, en gpio not specified\n",
                                                __func__, __LINE__);
                return -EINVAL;
        }

        ret = gpio_request(chip->en_gpio, "wl2866d_en");
        if (ret < 0) {
                pr_err("wl2866d enable gpio request failed\n");
                return ret;
        }
        gpio_free(chip->en_gpio);
        return 0;

}

static int wl2866d_get_id(struct  wl2866d_chip *chip)
{
	unsigned char reg_val = 0;
	int ret = 0;

	wl2866d_i2c_read(chip, WL2868C_ID_ADRR, &reg_val);
	pr_err("%s:wl2866d id is 0x%x\n", __func__, reg_val);

	if ((reg_val != WL2868C_ID) && (reg_val != WL2868C_ID1)) {
		ret = -1;
		return ret;
	}
	return 0;
}

static int set_init_voltage(struct wl2866d_chip *chip)
{
	int ret = 0;
	int i;

	for (i = 0 ; i < (ARRAY_SIZE(wl2866d_on_config) - 2); i++)	{
		ret = wl2866d_i2c_write(chip, wl2866d_on_config[i].reg, wl2866d_on_config[i].value);
		if (ret < 0) {
			pr_err("wl2866d init voltage failed\n");
			return ret;
		}
	}
	//enable dischager function
	/*ret = wl2866d_i2c_write(chip, wl2866d_on_config[DISCHARGE_ENABLE].reg, wl2866d_on_config[DISCHARGE_ENABLE].value);
	if (ret < 0) {
		pr_err("wl2866d  dischager function enable failed\n");
		return ret;
	}*/
	return 0;
}

int wl2866d_camera_power_up_eeprom(void)
{
    int ret = -1;
    if (camera_chip == NULL) {
	return ret;
    }
    ret = set_init_voltage(camera_chip);
    return ret;
}
EXPORT_SYMBOL(wl2866d_camera_power_up_eeprom);

int wl2866d_camera_power_up_all(void)
{
    int ret = -1;
    if (camera_chip == NULL) {
	return ret;
    }
    ret = set_init_voltage(camera_chip);
    return ret;
}
EXPORT_SYMBOL(wl2866d_camera_power_up_all);

int wl2866d_camera_power_down_eeprom(void)
{
    int ret = -1;
    ret = wl2866d_i2c_write(camera_chip, wl2866d_on_config[VOL_ENABLE].reg, 0);//bit1
    return ret;
}
EXPORT_SYMBOL(wl2866d_camera_power_down_eeprom);

void wl2866d_print_reg(struct  wl2866d_chip *chip)
{
	int i;
	unsigned char reg_val = 0;

	for (i = 0 ; i < ARRAY_SIZE(wl2866d_on_config); i++) {
		wl2866d_i2c_read(chip, wl2866d_on_config[i].reg, &reg_val);
		pr_err("%s:wl2866d info is reg 0x%x, value 0x%x\n", __func__, wl2866d_on_config[i].reg, reg_val);
	}

}

static int wl2866d_init(struct  wl2866d_chip *chip)
{
	int ret = 0;

	mutex_init(&wl2866d_mutex);
	chip->en_gpio = of_get_named_gpio(chip->dev->of_node,
			 "en-gpio", 0);
	if (!gpio_is_valid(chip->en_gpio)) {
		pr_err("%s:%d, en gpio not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}

	pr_err("%s: en_gpio is %d\n", __func__, chip->en_gpio);
	ret = gpio_request(chip->en_gpio, "wl2866d_en");
	//ret = devm_gpio_request_one(chip->dev, chip->en_gpio,
		//				  GPIOF_OUT_INIT_LOW,
		//				  "wl2866d_en");
	if (ret < 0) {
			pr_err("wl2866d enable gpio request failed\n");
			return ret;
	}

	gpio_direction_output(chip->en_gpio, 1);
	//if (chip && gpio_is_valid(chip->en_gpio)) {
	//		gpio_set_value_cansleep(chip->en_gpio, 1);
	//}

	msleep(10);

	ret = wl2866d_get_id(chip);
	if (ret < 0) {
		pr_err("wl2866d read id failed\n");
		return ret;
	}

	ret = set_init_voltage(chip);
	if (ret < 0)
		pr_err("wl2866d init failed\n");

	if (WL2866D_DEBUG) {
		msleep(10);
		wl2866d_print_reg(chip);
	}

	return 0;

}


/* static int wl2866d_disable_power(struct  wl2866d_chip *chip)
{
	int ret = 0;

	ret = regulator_disable(chip->vin1);
	if (ret)
		dev_err(chip->dev, "Unable to disable vin1:%d\n", ret);

	if (!regulator_is_enabled(chip->vin1)) {
		ret = regulator_set_voltage(chip->vin1, 0,
					VIN1_1P35_VOL_MAX);
		if (ret)
			dev_err(chip->dev,
				"Unable to set (0) voltage for vin1:%d\n", ret);
	}

	ret = regulator_disable(chip->vin2);
	if (ret)
		dev_err(chip->dev, "Unable to disable vin2:%d\n", ret);

	if (!regulator_is_enabled(chip->vin2)) {
		ret = regulator_set_voltage(chip->vin2, 0,
					VIN2_3P3_VOL_MAX);
		if (ret)
			dev_err(chip->dev,
				"Unable to set (0) voltage for vin2:%d\n", ret);
	}
	return 0;

} */


static int wl2866d_enable_power(struct  wl2866d_chip *chip)
{
	int ret = 0;

	ret = regulator_set_voltage(chip->vin1, VIN1_1P35_VOL_MIN,
						VIN1_1P35_VOL_MAX);
	if (ret) {
		dev_err(chip->dev,
				"Unable to set voltage for vin1:%d\n", ret);
		goto put_vin1;
	}

	ret = regulator_enable(chip->vin1);
	if (ret) {
		dev_err(chip->dev, "Unable to enable vin1:%d\n", ret);
		goto unset_vin1;
	}

	ret = regulator_set_voltage(chip->vin2, VIN2_3P3_VOL_MIN,
						VIN2_3P3_VOL_MAX);
	if (ret) {
		dev_err(chip->dev,
				"Unable to set voltage for vin2:%d\n", ret);
		goto disable_vin1;
	}


	ret = regulator_enable(chip->vin2);
	if (ret) {
		dev_err(chip->dev, "Unable to enable vin2:%d\n", ret);
		goto unset_vin2;
	}
	return 0;

unset_vin2:
	ret = regulator_set_voltage(chip->vin2, 0, VIN2_3P3_VOL_MAX);
	if (ret)
		dev_err(chip->dev,
			"Unable to set (0) voltage for vin2:%d\n", ret);

disable_vin1:
	ret = regulator_disable(chip->vin1);
	if (ret)
		dev_err(chip->dev, "Unable to disable vin1:%d\n", ret);

unset_vin1:
	ret = regulator_set_voltage(chip->vin1, 0, VIN1_1P35_VOL_MAX);
	if (ret)
		dev_err(chip->dev,
			"Unable to set (0) voltage for vin1:%d\n", ret);

put_vin1:
	return ret;
}

static int wl2866d_vreg_init(struct  wl2866d_chip *chip)
{
	int ret = 0;

	chip->vin1 = devm_regulator_get(chip->dev, "vin1");

	if (IS_ERR(chip->vin1)) {
		ret = PTR_ERR(chip->vin1);
		dev_err(chip->dev, "%s: can't get VIN1,%d\n", __func__, ret);
		goto err_vin1;
	}

	chip->vin2 = devm_regulator_get(chip->dev, "vin2");

	if (IS_ERR(chip->vin2)) {
		ret = PTR_ERR(chip->vin2);
		dev_err(chip->dev, "%s: can't get VIN2,%d\n", __func__, ret);
		goto err_vin2;
	}

	return 0;

err_vin2:
	devm_regulator_put(chip->vin1);
err_vin1:
	return ret;
}

static int wl2866d_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret = 0;
	struct wl2866d_chip *chip;

	pr_err("%s,enrty\n", __func__);
	chip = devm_kzalloc(&client->dev, sizeof(struct wl2866d_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto err_mem;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check_functionality failed\n");
		ret = -EIO;
		goto  init_err;
	}

	chip->client = client;

	chip->dev = &client->dev;
	dev_set_drvdata(chip->dev, chip);
	i2c_set_clientdata(chip->client, chip);

	ret = wl2866d_is_ldo_probed(chip);
	if (ret < 0) {
		pr_err("%s,another 7-ldo maybe already probe OK\n", __func__);
		goto vreg_init_err;
	}
	ret = wl2866d_vreg_init(chip);
	if (ret < 0)	{
		dev_err(&client->dev, "get vreg failed\n");
		goto vreg_init_err;
	}

	ret = wl2866d_enable_power(chip);
	if (ret) {
		dev_err(&client->dev, "enable power failed\n");
		ret = -1;
		goto vreg_enable_err;
	}

	ret = wl2866d_init(chip);

	if (ret < 0) {
		dev_err(&client->dev, "wl2866d init fail!\n");
		ret = -ENODEV;
		goto init_err;
	}

	ret = sysfs_create_group(&client->dev.kobj,
					   &wl2866d_attribute_group);
	if (ret < 0) {
		dev_info(&client->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

	camera_chip = chip;
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
	ldo5_count = 0;
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/

	pr_err("%s,successfully\n", __func__);
	return 0;
err_sysfs:
init_err:
vreg_enable_err:
	gpio_free(chip->en_gpio);
	devm_regulator_put(chip->vin1);
	devm_regulator_put(chip->vin2);
vreg_init_err:
	devm_kfree(chip->dev, chip);
	chip = NULL;
err_mem:
	return ret;
}

static int wl2866d_remove(struct i2c_client *client)
{
	struct wl2866d_chip *chip = i2c_get_clientdata(client);

	devm_kfree(chip->dev, chip);
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
	ldo5_count = 0;
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/
	chip = NULL;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wl2866d_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wl2866d_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	pr_err("%s\n", __func__);
	ret = wl2866d_i2c_write(chip, wl2866d_on_config[VOL_DISABLE].reg, 0b10100000);//disable power except ldo6
	if (ret < 0)
	   pr_err("wl2866d close voltage failed\n");

	//wl2866d_disable_power(chip);
	return 0;
}

static int wl2866d_resume(struct device *dev)
{
	/* struct i2c_client *client = to_i2c_client(dev);
	struct wl2866d_chip *chip = i2c_get_clientdata(client); */
	//int ret = 0;

	pr_err("%s\n", __func__);
	//wl2866d_enable_power(chip);
	//gpio_direction_output(chip->en_gpio, 1);
	/*
	ret = wl2866d_i2c_write(chip, wl2866d_on_config[VOL_ENABLE].reg, wl2866d_on_config[VOL_ENABLE].value);
	if (ret < 0) {
		pr_err("wl2866d set enable failed\n");
		return ret;
	}
	*/
	return 0;
}
#endif

static const struct dev_pm_ops wl2866d_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(wl2866d_suspend, wl2866d_resume)
};

static const struct i2c_device_id wl2866d_id_table[] = {
	{"vendor,wl2868c", 0},
	{} /* NULL terminated */
};

MODULE_DEVICE_TABLE(i2c, wl2866d_id_table);


#ifdef CONFIG_OF
static const struct of_device_id wl2866d_i2c_of_match_table[] = {
		{ .compatible = "vendor,wl2868c" },
		{},
};
MODULE_DEVICE_TABLE(of, wl2866d_i2c_of_match_table);
#endif

static struct i2c_driver wl2866d_driver = {
	.driver = {
		.name = "vendor,wl2868c",
		.pm = &wl2866d_pm_ops,
		.of_match_table = of_match_ptr(wl2866d_i2c_of_match_table),
		},
	.probe = wl2866d_probe,
	.remove = wl2866d_remove,
	.id_table = wl2866d_id_table,
};

int cam_wl2868c_driver_init(void)
{
	return i2c_add_driver(&wl2866d_driver);
}

void cam_wl2868c_driver_exit(void)
{
	i2c_del_driver(&wl2866d_driver);
}

MODULE_DESCRIPTION("wl2868c driver for camera");
MODULE_AUTHOR("camera,lnc.");
MODULE_LICENSE("GPL");
