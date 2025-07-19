/*  Date: 2020/11/95:w 10:00:00
 *  Revision: 1.4
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file sgm37604ayg.c
   brief This file contains all function implementations for the sgm37604ayg in linux

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */
#define SGM37604AYG_NAME "sgm37604ayg"
#define SGM37604AYG_INPUT_DEV_NAME	"SGM37604AYG"

static bool driver_prob_status = false;

struct sgm37604ayg_data {
	struct i2c_client *sgm37604ayg_client;
	unsigned char mode;
	signed char sensor_type;
	struct input_dev *input;
	/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
	int hwen_en;
	/*linden code for JLINDEN-396 by yangjinmin at 20221206 end*/
};

struct i2c_client g_sgm37604ayg_client;
/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
struct sgm37604ayg_data *al_sgm37604_client;
/*linden code for JLINDEN-396 by yangjinmin at 20221206 end*/

static int sgm37604ayg_smbus_read_byte(unsigned char reg_addr)
{

	int ret = 0;
	 if (driver_prob_status) {
		ret = i2c_smbus_read_byte_data(&g_sgm37604ayg_client, reg_addr);
        	printk(KERN_ERR "sgm37604ayg_smbus_read_byte reg_addr:0x%0x data:0x%0x,ok\n",reg_addr,ret);
	}
	return ret;
}

static int sgm37604ayg_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0) {
		printk(KERN_ERR "sgm37604ayg_smbus_write_byte reg_addr:0x%0x data:0x%0x error\n",reg_addr,*data);
		return -1;
        }
        printk(KERN_ERR "sgm37604ayg_smbus_write_byte reg_addr:0x%0x data:0x%0x,ok\n",reg_addr,*data);
	return 0;
}


static ssize_t sgm37604ayg_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count){
	return count;
}

static ssize_t sgm37604ayg_i2c_show(struct device *dev,struct device_attribute *attr,char *buf){
	//struct sgm37604ayg_data *sgm_data =  i2c_get_clientdata(dev);
	ssize_t len =0;
	int ret = 0;
	//unsigned char i = 0;
	//unsigned char reg_val = 0;
	ret = sgm37604ayg_smbus_read_byte(0x11);
	len += snprintf(buf+len,PAGE_SIZE-len,"0x11:0x%0x\n",ret);
	ret = sgm37604ayg_smbus_read_byte(0x1A);
	len += snprintf(buf+len,PAGE_SIZE-len,"0x1A:0x%0x\n",ret);
	ret = sgm37604ayg_smbus_read_byte(0x19);
	len += snprintf(buf+len,PAGE_SIZE-len,"0x19:0x%0x\n",ret);
	ret = sgm37604ayg_smbus_read_byte(0x1F);
	len += snprintf(buf+len,PAGE_SIZE-len,"0x1F:0x%0x\n",ret);
	return len;
}


static DEVICE_ATTR(sgm37604reg,0664,sgm37604ayg_i2c_show,sgm37604ayg_store);

static struct attribute *sgm37604ayg_attributes[] = {
	&dev_attr_sgm37604reg.attr,
	NULL
};

static struct attribute_group sgm37604ayg_attribute_group = {
	.attrs = sgm37604ayg_attributes
};


int sgm37604ayg_write_reg(unsigned char reg_addr, unsigned char *data)
{
    if (driver_prob_status) {
        sgm37604ayg_smbus_write_byte(&g_sgm37604ayg_client,reg_addr, data);
    }

    return 0;
}
EXPORT_SYMBOL(sgm37604ayg_write_reg);

int sgm37604ayg_set_bl(unsigned int bl_lv)
{
	unsigned char data1 = bl_lv & 0x0f;
	unsigned char data2 = ((bl_lv >> 4) & 0xff);
/* Linden code for JLINDEN-5157 by zhengjie6 at 2023/03/23 start */
	unsigned char data3 = 0x2C;
	sgm37604ayg_write_reg(0x1A, &data1);
	sgm37604ayg_write_reg(0x19, &data2);
	sgm37604ayg_write_reg(0x53, &data3);
/* Linden code for JLINDEN-5157 by zhengjie6 at 2023/03/23 end */

    return 0;
}
EXPORT_SYMBOL(sgm37604ayg_set_bl);

/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
static int sgm37604_get_dt_data(struct device *dev,
		struct sgm37604ayg_data *drvdata)
{
	int rc = 0;
	struct device_node *np = dev->of_node;
	pr_err("[%s] start\n", __func__);
	drvdata->hwen_en = of_get_named_gpio(np, "sgm37604ayg,hwen-gpio", 0);
	if (!gpio_is_valid(drvdata->hwen_en)) {
		rc = drvdata->hwen_en;
		pr_err("[%s] failed get hwen_en gpio, rc=%d\n", __func__, rc);
		goto get_tp_rst_gpio_err;
	}
	pr_err("%s hwen_en --<%d>\n", __func__, drvdata->hwen_en);

get_tp_rst_gpio_err:
	if (gpio_is_valid(drvdata->hwen_en))
		gpio_free(drvdata->hwen_en);
	return rc;
}

static int sgm37604_gpio_request(struct sgm37604ayg_data *drvdata)
{
	int rc = 0;
	pr_err("[%s] enter\n", __func__);
	if (gpio_is_valid(drvdata->hwen_en)) {
		rc = gpio_request(drvdata->hwen_en, "sgm37604_hwen_en");
		if (rc) {
			pr_err("request for sgm37604_hwen_en failed, rc=%d\n", rc);
			goto error_release_hwen_en_en;
		}
	}
error_release_hwen_en_en:
	if (gpio_is_valid(drvdata->hwen_en))
		gpio_free(drvdata->hwen_en);
	return rc;
}
int sgm37604ayg_bl_enable(int en)
{

	int rc = -1;
	unsigned char data = 0x00;
	if(al_sgm37604_client != NULL){
		if (gpio_is_valid(al_sgm37604_client->hwen_en)){
			if (en){
				rc = gpio_get_value(al_sgm37604_client->hwen_en);
				if(rc != 0){
					printk(KERN_ERR "sgm37604ayg hwen_bl is enabled\n");
				}else{
					gpio_set_value(al_sgm37604_client->hwen_en, true);
					usleep_range(3500, 4000);
					sgm37604ayg_write_reg(0x11, &data);
				}
				printk(KERN_ERR "sgm37604ayg hwen_bl gpio_request ok\n");
				return 0;
			}
		}else{
			printk(KERN_ERR "sgm37604ayg hwen_bl gpio:24 is not valid\n");
			rc = -1;
		}
	}
	return rc;
}
EXPORT_SYMBOL_GPL(sgm37604ayg_bl_enable);
/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/

static struct of_device_id sgm37604ayg_match_table[];

static int sgm37604ayg_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	struct sgm37604ayg_data *data;
	struct input_dev *dev;
	printk("%s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct sgm37604ayg_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	if (client->dev.of_node) {
		memset(data, 0 , sizeof(struct sgm37604ayg_data));
	}else{
		err = -ENODEV;
		goto kfree_exit;

	}
	/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
	err = sgm37604_get_dt_data(&client->dev, data);
	if(err){
		printk("sgm37604_get_dt_data is error");
		goto sgm37604_get_dt_data_err;
	}
	err = sgm37604_gpio_request(data);
	if(err){
		printk("sgm37604_gpio_request is error");
		goto sgm37604_gpio_request_err;
	}
	al_sgm37604_client = data;
	/*linden code for JLINDEN-396 by yangjinmin at 20221206 end*/
	i2c_set_clientdata(client, data);
	data->sgm37604ayg_client = client;
    g_sgm37604ayg_client = *client;
	dev = input_allocate_device();
	if (!dev){
  	    printk(KERN_INFO "input allocate device error\n");
	}
	dev->name = SGM37604AYG_INPUT_DEV_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_drvdata(dev, data);
	printk(KERN_ERR"sgm37604ayg probe OK \n");
        driver_prob_status = true;
	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
	}

	data->input = dev;

	err = sysfs_create_group(&data->input->dev.kobj,
	    &sgm37604ayg_attribute_group);
	if (err < 0)
		goto error_sysfs;

	return 0;

error_sysfs:
	input_unregister_device(data->input);
/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
sgm37604_gpio_request_err:
sgm37604_get_dt_data_err:
/*linden code for JLINDEN-396 by yangjinmin at 20221206 end*/
kfree_exit:
	kfree(data);
exit:
	return err;
}
static int  sgm37604ayg_remove(struct i2c_client *client)
{
	struct sgm37604ayg_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&data->input->dev.kobj, &sgm37604ayg_attribute_group);
	input_unregister_device(data->input);


	kfree(data);

	return 0;
}

static const struct i2c_device_id sgm37604ayg_id[] = {
	{ SGM37604AYG_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sgm37604ayg_id);

static struct of_device_id sgm37604ayg_match_table[] = {
	{ .compatible = "sgm,37604ayg", },
	{ },
};

static struct i2c_driver sgm37604ayg_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SGM37604AYG_NAME,
		.of_match_table = sgm37604ayg_match_table,
	},
	.id_table	= sgm37604ayg_id,
	.probe		= sgm37604ayg_probe,
	.remove		= sgm37604ayg_remove,

};
static int __init sgm37604ayg_init(void)
{
        printk("init sgm37604ayg device\n");
	return i2c_add_driver(&sgm37604ayg_driver);
}

static void __exit sgm37604ayg_exit(void)
{
	i2c_del_driver(&sgm37604ayg_driver);
}

MODULE_AUTHOR("HongTao guo <guohongtao5@huaqin.com>");
MODULE_DESCRIPTION("SGM37604AYG ic driver");
MODULE_LICENSE("GPL");

module_init(sgm37604ayg_init);
module_exit(sgm37604ayg_exit);
