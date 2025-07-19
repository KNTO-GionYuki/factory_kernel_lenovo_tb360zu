/*  Date: 2020/11/95:w 10:00:00
 *  Revision: 1.4
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file SM5109.c
   brief This file contains all function implementations for the SM5109 in linux

*/

#include <linux/init.h>
#include <linux/module.h>
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
#define SM5109_NAME "sm5109"
#define SM5109_INPUT_DEV_NAME	"SM5109"

int g_tp_rst_gpio = 0;
EXPORT_SYMBOL(g_tp_rst_gpio);

static bool driver_prob_status = false;
struct sm5109_data {
	struct i2c_client *sm5109_client;
	unsigned char mode;
	signed char sensor_type;
	struct input_dev *input;
	/*linden code for JLINDEN-222 by yangjinmin at 20221130 start*/
	int lcd_enn_en_gpio;
	int lcd_enp_en_gpio;
	int tp_rst_gpio;
	/*linden code for JLINDEN-222 by yangjinmin at 20221130 end*/
};

struct i2c_client g_sm5109_client;
/*linden code for JLINDEN-222 by yangjinmin at 20221130 start*/
struct sm5109_data *al_sm5109_client;
/*linden code for JLINDEN-222 by yangjinmin at 20221130 end*/

static int sm5109_smbus_write_byte(struct i2c_client *client,
    unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0) {
		printk(KERN_ERR "sm5109_smbus_write_byte error\n");
		return -1;
        }
        printk(KERN_ERR "sm5109_smbus_write_byte ok\n");
	return 0;
}

int sm5109_write_reg(unsigned char reg_addr, unsigned char *data)
{
    if (driver_prob_status) {
        sm5109_smbus_write_byte(&g_sm5109_client,reg_addr, data);
    }

    return 0;
}
EXPORT_SYMBOL_GPL(sm5109_write_reg);

static struct of_device_id sm5109_match_table[];

/*linden code for JLINDEN-222 by yangjinmin at 20221130 start*/
static int sm5109_get_dt_data(struct device *dev,
	struct sm5109_data *drvdata)
{
	int rc = 0;
	struct device_node *np = dev->of_node;
	pr_err("[%s] start\n", __func__);
	drvdata->lcd_enn_en_gpio = of_get_named_gpio(np, "qcom,platform-enn-en-gpio", 0);
	if (!gpio_is_valid(drvdata->lcd_enn_en_gpio)) {
		rc = drvdata->lcd_enn_en_gpio;
		pr_err("[%s] failed get lcd_enn_en_gpio gpio, rc=%d\n", __func__, rc);
		goto get_lcd_enn_en_gpio_err;
	}
	pr_err("%s lcd_enn_en_gpio --<%d>\n", __func__, drvdata->lcd_enn_en_gpio);

	drvdata->lcd_enp_en_gpio = of_get_named_gpio(np, "qcom,platform-enp-en-gpio", 0);
	if (!gpio_is_valid(drvdata->lcd_enp_en_gpio)) {
		rc = drvdata->lcd_enp_en_gpio;
		pr_err("[%s] failed get lcd_enn_enp_gpio gpio, rc=%d\n", __func__, rc);
		goto get_lcd_enp_en_gpio_err;
	}
	pr_err("%s lcd_enp_en_gpio --<%d>\n", __func__, drvdata->lcd_enp_en_gpio);

	/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
	drvdata->tp_rst_gpio = of_get_named_gpio(np, "qcom,platform-tp-rst-gpio", 0);
	if (!gpio_is_valid(drvdata->tp_rst_gpio)) {
		rc = drvdata->tp_rst_gpio;
		pr_err("[%s] failed get tp_rst_gpio gpio, rc=%d\n", __func__, rc);
		goto get_tp_rst_gpio_err;
	}
	pr_err("%s lcd_enp_en_gpio --<%d>\n", __func__, drvdata->lcd_enp_en_gpio);

get_tp_rst_gpio_err:
	if (gpio_is_valid(drvdata->tp_rst_gpio))
		gpio_free(drvdata->tp_rst_gpio);
	/*linden code for JLINDEN-396 by yangjinmin at 20221206 end*/
get_lcd_enp_en_gpio_err:
	if (gpio_is_valid(drvdata->lcd_enp_en_gpio))
		gpio_free(drvdata->lcd_enp_en_gpio);
get_lcd_enn_en_gpio_err:
	if (gpio_is_valid(drvdata->lcd_enn_en_gpio))
		gpio_free(drvdata->lcd_enn_en_gpio);
	return rc;
}

static int sm5109_gpio_request(struct sm5109_data *drvdata)
{
	int rc = 0;
	pr_err("[%s] enter\n", __func__);
	if (gpio_is_valid(drvdata->lcd_enn_en_gpio)) {
		rc = gpio_request(drvdata->lcd_enn_en_gpio, "lcd_enn_en");
		if (rc) {
			pr_err("request for lcd_enn_en_gpio failed, rc=%d\n", rc);
			goto error_release_enn_en;
		}
	}

	if (gpio_is_valid(drvdata->lcd_enp_en_gpio)) {
		rc = gpio_request(drvdata->lcd_enp_en_gpio, "lcd_enp_en");
		if (rc) {
			pr_err("request for lcd_enp_en_gpio failed, rc=%d\n", rc);
			goto error_release_enp_en;
		}
	}
	/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
	if (gpio_is_valid(drvdata->tp_rst_gpio)) {
		rc = gpio_request(drvdata->tp_rst_gpio, "tp_rst");
		if (rc) {
			pr_err("request for lcd_enp_en_gpio failed, rc=%d\n", rc);
			goto error_release_tp;
		}
		g_tp_rst_gpio = drvdata->tp_rst_gpio;
	}
error_release_tp:
	if (gpio_is_valid(drvdata->tp_rst_gpio))
		gpio_free(drvdata->tp_rst_gpio);
	/*linden code for JLINDEN-396 by yangjinmin at 20221206 end*/
error_release_enp_en:
	if (gpio_is_valid(drvdata->lcd_enp_en_gpio))
		gpio_free(drvdata->lcd_enp_en_gpio);
error_release_enn_en:
	if (gpio_is_valid(drvdata->lcd_enn_en_gpio))
		gpio_free(drvdata->lcd_enn_en_gpio);
	return rc;
}

/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
void sm5109_bias_disable(int en)
{
	pr_err("[%s] enter\n", __func__);
	if (al_sm5109_client != NULL){
		if (gpio_is_valid(al_sm5109_client->lcd_enn_en_gpio)){
			gpio_set_value(al_sm5109_client->lcd_enn_en_gpio, en);
		}
		mdelay(3);
		if (gpio_is_valid(al_sm5109_client->lcd_enp_en_gpio)){
			gpio_set_value(al_sm5109_client->lcd_enp_en_gpio, en);
		}
	}
}
EXPORT_SYMBOL_GPL(sm5109_bias_disable);

void sm5109_bias_enable(int en)
{
	pr_err("[%s] enter\n", __func__);
	if (al_sm5109_client != NULL){
		if (gpio_is_valid(al_sm5109_client->lcd_enp_en_gpio)){
			gpio_set_value(al_sm5109_client->lcd_enp_en_gpio, en);
		}
		mdelay(3);
		if (gpio_is_valid(al_sm5109_client->lcd_enn_en_gpio)){
			gpio_set_value(al_sm5109_client->lcd_enn_en_gpio, en);
		}
	}
}
/*linden code for JLINDEN-396 by yangjinmin at 20221206 end*/
EXPORT_SYMBOL_GPL(sm5109_bias_enable);

void sm5109_bias_set(void)
{
	/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
	unsigned char bias_data = 0x12;
	/*linden code for JLINDEN-497 by yangjinmin at 20221214 start*/
	unsigned char bias_cur = 0x43;
        sm5109_write_reg(0x03,&bias_cur);
        mdelay(1);
	/*linden code for JLINDEN-497 by yangjinmin at 20221214 end*/
	sm5109_write_reg(0x00,&bias_data);
	mdelay(1);
	sm5109_write_reg(0x01,&bias_data);
	mdelay(1);
	/*linden code for JLINDEN-396 by yangjinmin at 20221206 end*/
}
EXPORT_SYMBOL_GPL(sm5109_bias_set);
/*linden code for JLINDEN-222 by yangjinmin at 20221130 end*/
/*linden code for JLINDEN-396 by yangjinmin at 20221206 start*/
void tp_rst_gpio_enable(int en)
{
	if(al_sm5109_client != NULL){
		pr_err("[%s] enter\n", __func__);
		if (gpio_is_valid(al_sm5109_client->tp_rst_gpio)){
			gpio_set_value(al_sm5109_client->tp_rst_gpio, en);
		}
	}
}
EXPORT_SYMBOL_GPL(tp_rst_gpio_enable);
/*linden code for JLINDEN-396 by yangjinmin at 20221206 end*/

static int sm5109_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	struct sm5109_data *data;
	struct input_dev *dev;
	printk("%s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct sm5109_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	if (client->dev.of_node) {
		memset(data, 0 , sizeof(struct sm5109_data));
	}else{
		err = -ENODEV;
		goto kfree_exit;

	}

/*linden code for JLINDEN-222 by yangjinmin at 20221130 start*/
	err = sm5109_get_dt_data(&client->dev, data);
	if(err){
		printk("sm5109_get_dt_data is error");
		goto sm5109_get_dt_data_err;
	}
	err = sm5109_gpio_request(data);
	if(err){
		printk("sm5109_gpio_request is error");
		goto sm5109_gpio_request_err;
	}
	al_sm5109_client = data;
/*linden code for JLINDEN-222 by yangjinmin at 20221130 end*/
	i2c_set_clientdata(client, data);
	data->sm5109_client = client;
        g_sm5109_client = *client;
	dev = input_allocate_device();
	if (!dev){
  	    printk(KERN_INFO "input allocate device error\n");
	}
	dev->name = SM5109_INPUT_DEV_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_drvdata(dev, data);
	printk(KERN_ERR"sm5109 probe OK \n");
        driver_prob_status = true;
	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
	}

	data->input = dev;


	return 0;
/*linden code for JLINDEN-222 by yangjinmin at 20221130 start*/
sm5109_gpio_request_err:
sm5109_get_dt_data_err:
/*linden code for JLINDEN-222 by yangjinmin at 20221130 end*/
kfree_exit:
	kfree(data);
exit:
	return err;
}
static int  sm5109_remove(struct i2c_client *client)
{
	struct sm5109_data *data = i2c_get_clientdata(client);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static const struct i2c_device_id sm5109_id[] = {
	{ SM5109_NAME, 0 },
	{ }
};


static struct of_device_id sm5109_match_table[] = {
	{ .compatible = "sm,sm5109", },
	{ },
};

/*linden code for JLINDEN-222 by yangjinmin at 20221130 start*/
static struct i2c_driver sm5109_driver = {
/*linden code for JLINDEN-222 by yangjinmin at 20221130 end*/
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SM5109_NAME,
		.of_match_table = sm5109_match_table,
	},
	.id_table	= sm5109_id,
	.probe		= sm5109_probe,
	.remove		= sm5109_remove,

};

//EXPORT_SYMBOL(sm5109_driver);

static int __init sm5109_init(void)
{
    printk("init sm5109 device\n");
	return i2c_add_driver(&sm5109_driver);
}

static void __exit sm5109_exit(void)
{
	i2c_del_driver(&sm5109_driver);
}

MODULE_AUTHOR("xiepeng <xiepeng6@huaqin.com>");
MODULE_DESCRIPTION("sm5109 ic driver");
MODULE_LICENSE("GPL");

module_init(sm5109_init);
module_exit(sm5109_exit);
