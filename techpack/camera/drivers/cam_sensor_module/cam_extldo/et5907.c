#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include  "et5907.h"
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/unaligned.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>


#define ET5907_ID  0x01
#define ET5907_ID_ADRR 0X00
#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2

struct et5907_chip *g_camera_chip;
struct mutex et5907_mutex;
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
static unsigned ldo5_count = 0;
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/

static const struct  et5907_map  et5907_on_config[] = {
	{0x04, 0x66},    //LDO1   MAIN-DVDD  1.2 V
	{0x05, 0x72},    //LDO2   FRONT-DVDD 1.2 V add 0.036v
	{0x06, 0xA0},    //LDO3   AF-VDD     2.8 V
	{0x07, 0xA0},    //LDO4   MAIN-AVDD  2.8 v
	{0x08, 0x41},    //LDO5   DOVDD      1.8 v add 0.05v
	{0x09, 0xD2},    //LDO6   FP         3.3 v
	{0x0a, 0xA0},    //LDO7   FRONT-AVDD 2.8 v
	{0x04, 0x55},    //LDO1   MAIN-DVDD  1.1 V
	{0x03, 0xFF},    //enable
	{0x03, 0x00},    //disable
};


static int et5907_i2c_write(struct et5907_chip *chip,
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

static int et5907_i2c_read(struct et5907_chip *chip,
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

int et5907_camera_power_down_all(void)
{
    int ret = -1;
	ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, 0);//bit1
	if (ret < 0) {
		pr_err("et5907 set enable failed\n");
		return ret;
	}
    return ret;
}

//0x03 bit0:LOD1, bit1:LDO2, bit2:LDO3, bit3:LDO4
//     bit4:LOD5, bit5:LDO6, bit6:LDO7, bit7:REV
//{0x04, 0x95}, OUT_LDO1
//{0x05, 0x95}, OUT_LDO2
//{0x06, 0xb3}, OUT_LDO3
//{0x07, 0xb3}, OUT_LDO4
//{0x08, 0x36}, OUT_LDO5
//{0x09, 0xf1}, OUT_LDO6
//{0x0a, 0xb3}, OUT_LDO7
//{0x03, 0xff}, ENABLE
//{0x03, 0x00}, DISABLE
int et5907_camera_power_up(int out_iotype)
{
	int ret = -1;
	unsigned char reg_val = 0;

	if (g_camera_chip == NULL) {
		pr_err("et5907 probe fail the camera_chip is NULL\n");
		return ret;
	}
	switch (out_iotype) {
	case OUT_LDO1:
		mutex_lock(&et5907_mutex);
		pr_err("walf et5907 OUT_LDO1:  out put type is [%d]", out_iotype);
		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[OUT_LDO1].reg, et5907_on_config[OUT_LDO1].value);//bit0
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set ldo1 failed\n");
			break;
		}

		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 read ldo1 enable failed\n");
			break;
		}

		//pr_err("et5907 ldo1 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0001;//bit 0
		//pr_err("et5907 ldo1 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit0
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set ldo1 enable failed\n");
			break;
		}
		pr_err("et5907 set LDO1 success!");
		mutex_unlock(&et5907_mutex);
		break;
case OUT_LDO1P1:
		mutex_lock(&et5907_mutex);
		pr_err("walf et5907 OUT_LDO1P1:  out put type is [%d]", out_iotype);
		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[OUT_LDO1P1].reg, et5907_on_config[OUT_LDO1P1].value);//bit0
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set ldo1 failed\n");
			break;
		}

		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 read ldo1 enable failed\n");
			break;
		}

		//pr_err("et5907 ldo1 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0001;//bit 0
		//pr_err("et5907 ldo1 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit0
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set ldo1 enable failed\n");
			break;
		}
		pr_err("et5907 set LDO1 success!");
		mutex_unlock(&et5907_mutex);
		break;

	case OUT_LDO2:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 ldo2:out put type is [%d]", out_iotype);
		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[OUT_LDO2].reg, et5907_on_config[OUT_LDO2].value);//bit1
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set ldo2 failed\n");
			break;
		}

		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo2 read enable failed\n");
			break;
		}
		//pr_err("et5907 ldo2 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0010;//bit1
		//pr_err("et5907 ldo2 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo2 set enable failed\n");
			break;
		}
		pr_err("et5907 set LDO2 success!");
		mutex_unlock(&et5907_mutex);
		break;

	case OUT_LDO3:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 LDO3: out put type is [%d]", out_iotype);
		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[OUT_LDO3].reg, et5907_on_config[OUT_LDO3].value);//bit2
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set ldo3 failed\n");
			break;
		}

		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 read enable failed\n");
			break;
		}

		//pr_err("et5907 LDO3 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b0100;//bit2
		//pr_err("et5907 LDO3 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit2
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo3 set enable failed\n");
			break;
		}
		pr_err("et5907 set LDO3 success!");
		mutex_unlock(&et5907_mutex);
		break;

	case OUT_LDO4:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 LDO4 : out put type is [%d]", out_iotype);
		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[OUT_LDO4].reg, et5907_on_config[OUT_LDO4].value);//bit3
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set ldo4 failed\n");
			break;
		}

		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 read enable failed\n");
			break;
		}

		//pr_err("et5907 LDO4 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b1000;//bit3
		//pr_err("et5907 LDO4 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit3
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set enable failed\n");
			break;
		}
		pr_err("et5907 set LDO4 success!");
		mutex_unlock(&et5907_mutex);
		break;
	case OUT_LDO5:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 LDO5 : out put type is [%d]", out_iotype);
		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[OUT_LDO5].reg, et5907_on_config[OUT_LDO5].value);//bit3
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set LDO5 failed\n");
			break;
		}

		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO5 read enable failed\n");
			break;
		}

		//pr_err("et5907 LDO5 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b10000;//bit4
		//pr_err("et5907 LDO5 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit4
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO5 set enable failed\n");
			break;
		}
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
		ldo5_count++;
		pr_err("et5907 ldo 5 power up count = %d", ldo5_count);
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/
		pr_err("et5907 set LDO5 success!");
		mutex_unlock(&et5907_mutex);
		break;
	case OUT_LDO6:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 LDO6 :out put type is [%d]", out_iotype);
		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[OUT_LDO6].reg, et5907_on_config[OUT_LDO6].value);//bit3
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set LDO6 failed\n");
			break;
		}

		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO6 read enable failed\n");
			break;
		}

		//pr_err("et5907 LDO6 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b100000;//bit5
		//pr_err("et5907 LDO6 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit5
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO6 set enable failed\n");
			break;
		}
		pr_err("et5907 set LDO6 success!");
		mutex_unlock(&et5907_mutex);
		break;
	case OUT_LDO7:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 LDO7:out put type is [%d]", out_iotype);
		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[OUT_LDO7].reg, et5907_on_config[OUT_LDO7].value);//bit3
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO7 set avdd1 failed\n");
			break;
		}

		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO7 read enable failed\n");
			break;
		}

		//pr_err("et5907 ldo7 before set enable value = 0x%x\n", reg_val);
		reg_val |= 0b1000000;//bit6
		//pr_err("et5907 ldo7 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit6
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo7 set enable failed\n");
			break;
		}
		pr_err("et5907 set LDO7 success!");
		mutex_unlock(&et5907_mutex);
		break;
	default:
		pr_err("et5907 unknown port!!!\n");
		break;
	}
	pr_err("et5907   result = %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(et5907_camera_power_up);

int et5907_camera_power_down(int out_iotype)
{
	int ret = -1;
	unsigned char reg_val = 0;

        if (g_camera_chip == NULL) {
                pr_err("et5907 probe fail the camera_chip is NULL\n");
                return ret;
        }
	switch (out_iotype) {
	case OUT_LDO1:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 ldo1:out put type is [%d]", out_iotype);
		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo1 read enable failed\n");
			break;
		}
		//pr_err("et5907 ldo1 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11111110;//bit0
		//pr_err("et5907 ldo1 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit0
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo1 set enable failed\n");
			break;
		}
		pr_err("et5907 disable ldo1 success!");
		mutex_unlock(&et5907_mutex);
		break;
case OUT_LDO1P1:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 ldo1:out put type is [%d]", out_iotype);
		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo1 read enable failed\n");
			break;
		}
		//pr_err("et5907 ldo1 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11111110;//bit0
		//pr_err("et5907 ldo1 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit0
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo1 set enable failed\n");
			break;
		}
		pr_err("et5907 disable ldo1 success!");
		mutex_unlock(&et5907_mutex);
		break;

	case OUT_LDO2:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 ldo2: et5907 out put type is [%d]", out_iotype);
		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo2 read enable failed\n");
			break;
		}

		//pr_err("et5907 ldo2 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11111101;//bit1
		//pr_err("et5907 ldo2 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo2 set enable failed\n");
			break;
		}
		pr_err("et5907 disable ldo2 success!");
		mutex_unlock(&et5907_mutex);
		break;

	case OUT_LDO3:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 LDO3: out put type is [%d]", out_iotype);
		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO3 read enable failed\n");
			break;
		}

		//pr_err("et5907 LDO3 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11111011;//bit2
		//pr_err("et5907 LDO3 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit2
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo3 set enable failed\n");
			break;
		}
		pr_err("et5907 disable LDO3 success!");
		mutex_unlock(&et5907_mutex);
		break;

	case OUT_LDO4:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 LDO4: output iotype is [%d]", out_iotype);
		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO4 read enable failed\n");
			break;
		}

		//pr_err("et5907 LDO4 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11110111;//bit3
		//pr_err("et5907 LDO4 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit1
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 ldo4 set enable failed\n");
			break;
		}
		pr_err("et5907 disable LDO4 success!");
		mutex_unlock(&et5907_mutex);
		break;
	case OUT_LDO5:
		mutex_lock(&et5907_mutex);
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
		if (ldo5_count > 1) {
			ldo5_count--;
			pr_err("et5907 ldo 5 power down count = %d", ldo5_count);
			mutex_unlock(&et5907_mutex);
			ret = 0;
			break;
		}
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/
		//pr_err("et5907 LDO5:output iotype is [%d]", out_iotype);
		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO5 read enable failed\n");
			break;
		}

		//pr_err("et5907 LDO5 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11101111;//bit4
		//pr_err("et5907 LDO5 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit4
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 set enable failed\n");
			break;
		}
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
		ldo5_count--;
		pr_err("et5907 ldo 5 power down count = %d", ldo5_count);
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/
		pr_err("et5907 disable LDO5 success!");
		mutex_unlock(&et5907_mutex);
		break;
	case OUT_LDO6:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 LDO6:output iotype is [%d]", out_iotype);
		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO6 read enable failed\n");
			break;
		}

		//pr_err("et5907 LDO6 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b11011111;//bit5
		//pr_err("et5907 LDO6 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit5
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO6 set enable failed\n");
			break;
		}
		pr_err("et5907 disable LDO6 success!");
		mutex_unlock(&et5907_mutex);
		break;
	case OUT_LDO7:
		mutex_lock(&et5907_mutex);
		//pr_err("et5907 LDO7:output iotype is [%d]", out_iotype);
		ret = et5907_i2c_read(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO7 read enable failed\n");
			break;
		}

		//pr_err("et5907 LDO7 before set enable value = 0x%x\n", reg_val);
		reg_val &= 0b10111111;//bit6
		//pr_err("et5907 LDO7 after  set enable value = 0x%x\n", reg_val);

		ret = et5907_i2c_write(g_camera_chip, et5907_on_config[VOL_ENABLE].reg, reg_val);//bit6
		if (ret < 0) {
			mutex_unlock(&et5907_mutex);
			pr_err("et5907 LDO7 set enable failed\n");
			break;
		}
		pr_err("et5907 disable LDO7 success!");
		mutex_unlock(&et5907_mutex);
		break;
	default:
		pr_err("et5907 unknown camera!!!\n");
		break;
	}
	pr_err("et5907 result = %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(et5907_camera_power_down);


static ssize_t et5907_vol_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct et5907_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	et5907_i2c_read(chip, et5907_on_config[VOL_ENABLE].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t et5907_vol_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct et5907_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = et5907_i2c_write(chip, et5907_on_config[VOL_ENABLE].reg, reg_val);
	if (ret < 0) {
		pr_err("et5907 set enable failed\n");
		return ret;
	}
	return count;
}


static ssize_t et5907_out_ldo1_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct et5907_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	et5907_i2c_read(chip, et5907_on_config[OUT_LDO1].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t et5907_out_ldo1_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct et5907_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = et5907_i2c_write(chip, et5907_on_config[OUT_LDO1].reg, reg_val);
	if (ret < 0) {
		pr_err("et5907 open ldo1 failed\n");
		return ret;
	}
	return count;
}

static ssize_t et5907_out_ldo1p1_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct et5907_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	et5907_i2c_read(chip, et5907_on_config[OUT_LDO1P1].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t et5907_out_ldo1p1_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct et5907_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = et5907_i2c_write(chip, et5907_on_config[OUT_LDO1P1].reg, reg_val);
	if (ret < 0) {
		pr_err("et5907 open ldo1 failed\n");
		return ret;
	}
	return count;
}


static ssize_t et5907_out_ldo2_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct et5907_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	et5907_i2c_read(chip, et5907_on_config[OUT_LDO2].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t et5907_out_ldo2_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct et5907_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = et5907_i2c_write(chip, et5907_on_config[OUT_LDO2].reg, reg_val);
	if (ret < 0)	{
		pr_err("et5907 open ldo2 failed\n");
		return ret;
	}
	return count;
}


static ssize_t et5907_out_ldo3_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct et5907_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	et5907_i2c_read(chip, et5907_on_config[OUT_LDO3].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t et5907_out_ldo3_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct et5907_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;

	ret = et5907_i2c_write(chip, et5907_on_config[OUT_LDO3].reg, reg_val);
	if (ret < 0)	{
		pr_err("et5907 open ldo3 failed\n");
		return ret;
	}
	return count;
}


static ssize_t et5907_out_ldo4_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct et5907_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	et5907_i2c_read(chip, et5907_on_config[OUT_LDO4].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t et5907_out_ldo4_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct et5907_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;
	ret = et5907_i2c_write(chip, et5907_on_config[OUT_LDO4].reg, reg_val);
	if (ret < 0)	{
		pr_err("et5907 open ldo4 failed\n");
		return ret;
	}
	return count;
}

static ssize_t et5907_out_ldo5_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct et5907_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	et5907_i2c_read(chip, et5907_on_config[OUT_LDO5].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t et5907_out_ldo5_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct et5907_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;
	ret = et5907_i2c_write(chip, et5907_on_config[OUT_LDO5].reg, reg_val);
	if (ret < 0)    {
		pr_err("et5907 open ldo5 failed\n");
		return ret;
	}
	return count;
}

static ssize_t et5907_out_ldo6_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct et5907_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	et5907_i2c_read(chip, et5907_on_config[OUT_LDO6].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t et5907_out_ldo6_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct et5907_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;
	ret = et5907_i2c_write(chip, et5907_on_config[OUT_LDO6].reg, reg_val);
	if (ret < 0)    {
		pr_err("et5907 open ldo6 failed\n");
		return ret;
	}
	return count;
}

static ssize_t et5907_out_ldo7_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct et5907_chip *chip = dev_get_drvdata(dev);
	unsigned char reg_val = 0;

	et5907_i2c_read(chip, et5907_on_config[OUT_LDO7].reg, &reg_val);
	return snprintf(buf, 10, "%d\n", reg_val);

}

static ssize_t et5907_out_ldo7_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned char reg_val = 0;
	struct et5907_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &reg_val);
	if (ret < 0)
		return ret;
	ret = et5907_i2c_write(chip, et5907_on_config[OUT_LDO7].reg, reg_val);
	if (ret < 0)    {
		pr_err("et5907 open ldo7 failed\n");
		return ret;
	}
	return count;
}
static DEVICE_ATTR(vol_enable, 0664, et5907_vol_enable_show, et5907_vol_enable_store);
static DEVICE_ATTR(out_ldo1, 0664, et5907_out_ldo1_show, et5907_out_ldo1_store);
static DEVICE_ATTR(out_ldo1p1, 0664, et5907_out_ldo1p1_show, et5907_out_ldo1p1_store);
static DEVICE_ATTR(out_ldo2, 0664, et5907_out_ldo2_show, et5907_out_ldo2_store);
static DEVICE_ATTR(out_ldo3, 0664, et5907_out_ldo3_show, et5907_out_ldo3_store);
static DEVICE_ATTR(out_ldo4, 0664, et5907_out_ldo4_show, et5907_out_ldo4_store);
static DEVICE_ATTR(out_ldo5, 0664, et5907_out_ldo5_show, et5907_out_ldo5_store);
static DEVICE_ATTR(out_ldo6, 0664, et5907_out_ldo6_show, et5907_out_ldo6_store);
static DEVICE_ATTR(out_ldo7, 0664, et5907_out_ldo7_show, et5907_out_ldo7_store);

static struct attribute *et5907_attributes[] = {
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

static struct attribute_group et5907_attribute_group = {
	.attrs = et5907_attributes
};

static int et5907_is_ldo_probed(struct  et5907_chip *chip)
{
	int ret = 0;

	chip->en_gpio = of_get_named_gpio(chip->dev->of_node,
			"en-gpio", 0);
	if (!gpio_is_valid(chip->en_gpio)) {
		pr_err("%s:%d, en gpio not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}

	ret = gpio_request(chip->en_gpio, "et5907_en");
	if (ret < 0) {
		pr_err("et5907 enable gpio request failed\n");
		return ret;
	}
	gpio_free(chip->en_gpio);
	return 0;

}

static int et5907_get_id(struct  et5907_chip *chip)
{
	unsigned char reg_val = 0;
	int ret = 0;

	et5907_i2c_read(chip, ET5907_ID_ADRR, &reg_val);
	pr_err("%s:et5907 id is 0x%x\n", __func__, reg_val);

	if ((reg_val != ET5907_ID)) {
		ret = -1;
		return ret;
	}
	return 0;
}

static int set_init_voltage(struct et5907_chip *chip)
{
	int ret = 0;
	int i;

	for (i = 0 ; i < (ARRAY_SIZE(et5907_on_config) - 2); i++)	{
		ret = et5907_i2c_write(chip, et5907_on_config[i].reg, et5907_on_config[i].value);
		if (ret < 0) {
			pr_err("et5907 init voltage failed\n");
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

int et5907_camera_power_up_all(void)
{
    int ret = -1;
    if (g_camera_chip == NULL) {
	pr_err("et5907 probe fail the camera_chip is NULL\n");
	return ret;
    }
    ret = set_init_voltage(g_camera_chip);
    return ret;
}
EXPORT_SYMBOL(et5907_camera_power_up_all);


void et5907_print_reg(struct  et5907_chip *chip)
{
	int i;
	unsigned char reg_val = 0;

	for (i = 0 ; i < ARRAY_SIZE(et5907_on_config); i++) {
		et5907_i2c_read(chip, et5907_on_config[i].reg, &reg_val);
		pr_err("%s:et5907 info is reg 0x%x, value 0x%x\n", __func__, et5907_on_config[i].reg, reg_val);
	}

}

static int et5907_init(struct  et5907_chip *chip)
{
	int ret = 0;

	mutex_init(&et5907_mutex);
	chip->en_gpio = of_get_named_gpio(chip->dev->of_node,
			 "en-gpio", 0);
	if (!gpio_is_valid(chip->en_gpio)) {
		pr_err("%s:%d, en gpio not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}

	pr_err("%s: en_gpio is %d\n", __func__, chip->en_gpio);
	ret = gpio_request(chip->en_gpio, "et5907_en");
	//ret = devm_gpio_request_one(chip->dev, chip->en_gpio,
		//				  GPIOF_OUT_INIT_LOW,
		//				  "et5907_en");
	if (ret < 0) {
			pr_err("et5907 enable gpio request failed\n");
			return ret;
	}

	gpio_direction_output(chip->en_gpio, 1);
	//if (chip && gpio_is_valid(chip->en_gpio)) {
	//		gpio_set_value_cansleep(chip->en_gpio, 1);
	//}

	msleep(10);

	ret = et5907_get_id(chip);
	if (ret < 0) {
		pr_err("et5907 read id failed\n");
		return ret;
	}

	ret = set_init_voltage(chip);
	if (ret < 0)
		pr_err("et5907 init failed\n");

	if (ET5907_DEBUG) {
		msleep(10);
		et5907_print_reg(chip);
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


static int et5907_enable_power(struct  et5907_chip *chip)
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

	/*ret = regulator_set_voltage(chip->vin2, VIN2_3P3_VOL_MIN,
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
	}*/
	return 0;

/*unset_vin2:
	ret = regulator_set_voltage(chip->vin2, 0, VIN2_3P3_VOL_MAX);
	if (ret)
		dev_err(chip->dev,
			"Unable to set (0) voltage for vin2:%d\n", ret);

disable_vin1:
	ret = regulator_disable(chip->vin1);
	if (ret)
		dev_err(chip->dev, "Unable to disable vin1:%d\n", ret);*/

unset_vin1:
	ret = regulator_set_voltage(chip->vin1, 0, VIN1_1P35_VOL_MAX);
	if (ret)
		dev_err(chip->dev,
			"Unable to set (0) voltage for vin1:%d\n", ret);

put_vin1:
	return ret;
}

static int et5907_vreg_init(struct  et5907_chip *chip)
{
	int ret = 0;

	chip->vin1 = devm_regulator_get(chip->dev, "vin1");

	if (IS_ERR(chip->vin1)) {
		ret = PTR_ERR(chip->vin1);
		dev_err(chip->dev, "%s: can't get VIN1,%d\n", __func__, ret);
		goto err_vin1;
	}

	/*chip->vin2 = devm_regulator_get(chip->dev, "vin2");

	if (IS_ERR(chip->vin2)) {
		ret = PTR_ERR(chip->vin2);
		dev_err(chip->dev, "%s: can't get VIN2,%d\n", __func__, ret);
		goto err_vin2;
	}*/

	return 0;

/*err_vin2:
	devm_regulator_put(chip->vin1);*/
err_vin1:
	return ret;
}

static int et5907_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret = 0;
	struct et5907_chip *chip;

	pr_err("%s,enrty\n", __func__);
	chip = devm_kzalloc(&client->dev, sizeof(struct et5907_chip), GFP_KERNEL);
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

	ret = et5907_is_ldo_probed(chip);
	if (ret < 0) {
		pr_err("%s, another 7-ldo maybe already probe OK\n", __func__);
		goto vreg_init_err;
	}

	ret = et5907_vreg_init(chip);
	if (ret < 0)	{
		dev_err(&client->dev, "get vreg failed\n");
		goto vreg_init_err;
	}

	ret = et5907_enable_power(chip);
	if (ret) {
		dev_err(&client->dev, "enable power failed\n");
		ret = -1;
		goto vreg_enable_err;
	}

	ret = et5907_init(chip);

	if (ret < 0) {
		dev_err(&client->dev, "et5907 init fail!\n");
		ret = -ENODEV;
		goto init_err;
	}

	ret = sysfs_create_group(&client->dev.kobj,
					   &et5907_attribute_group);
	if (ret < 0) {
		dev_info(&client->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

	g_camera_chip = chip;
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
	//devm_regulator_put(chip->vin2);
vreg_init_err:
	devm_kfree(chip->dev, chip);

	chip = NULL;
err_mem:
	return ret;
}

static int et5907_remove(struct i2c_client *client)
{
	struct et5907_chip *chip = i2c_get_clientdata(client);

	devm_kfree(chip->dev, chip);
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 start*/
	ldo5_count = 0;
/*Linden code for JLINDEN-8309 by huabinchen at 2023/06/30 end*/
	chip = NULL;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int et5907_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct et5907_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	pr_err("%s\n", __func__);
	ret = et5907_i2c_write(chip, et5907_on_config[VOL_DISABLE].reg, 0b10100000);//disable power except ldo6
	if (ret < 0)
	   pr_err("et5907 close voltage failed\n");

	//wl2866d_disable_power(chip);
	return 0;
}

static int et5907_resume(struct device *dev)
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

static const struct dev_pm_ops et5907_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(et5907_suspend, et5907_resume)
};

static const struct i2c_device_id et5907_id_table[] = {
	{"vendor,et5907", 0},
	{} /* NULL terminated */
};

MODULE_DEVICE_TABLE(i2c, et5907_id_table);


#ifdef CONFIG_OF
static const struct of_device_id et5907_i2c_of_match_table[] = {
		{ .compatible = "vendor,et5907" },
		{},
};
MODULE_DEVICE_TABLE(of, et5907_i2c_of_match_table);
#endif

static struct i2c_driver et5907_driver = {
	.driver = {
		.name = "vendor,et5907",
		.pm = &et5907_pm_ops,
		.of_match_table = of_match_ptr(et5907_i2c_of_match_table),
		},
	.probe = et5907_probe,
	.remove = et5907_remove,
	.id_table = et5907_id_table,
};


int cam_et5907_driver_init(void)
{
	return i2c_add_driver(&et5907_driver);
}

void cam_et5907_driver_exit(void)
{
	i2c_del_driver(&et5907_driver);
}

MODULE_DESCRIPTION("et5907 driver for camera");
MODULE_AUTHOR("camera,lnc.");
MODULE_LICENSE("GPL");
