#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
 
struct sar_ant_fnc_data {
	int ant1_status;
	int ant2_status;
	int ant1_irq;
	int ant2_irq;
	int ant1_irq_disabled;
	int ant2_irq_disabled;
	struct input_dev *input_device;
	struct pinctrl *customer_pinctrl;
	spinlock_t ant1_irq_lock;
	spinlock_t ant2_irq_lock;
};
 
struct sar_ant_fnc_data *sar_ant_data;
static int ant1_old_status = -1;
static int ant2_old_status = -1;
 
static struct timer_list ant2_input_timer;
static void ant2_input_timer_function(struct timer_list *data);
static DEFINE_TIMER(ant2_input_timer,ant2_input_timer_function);
 
static struct timer_list ant1_input_timer;
static void ant1_input_timer_function(struct timer_list *data);
static DEFINE_TIMER(ant1_input_timer,ant1_input_timer_function);
 
//关闭中断
static void input_irq_disable(struct sar_ant_fnc_data *data, int dx)
{
	
	unsigned long irqflags;
	spinlock_t *lock;
	int irq;
	int *irq_disabled;
 
	if (dx == data->ant1_status){
		lock = &data->ant1_irq_lock;
		irq = data->ant1_irq;
		irq_disabled = &data->ant1_irq_disabled;
	}
	else{
		lock = &data->ant2_irq_lock;
		irq = data->ant2_irq;
		irq_disabled = &data->ant2_irq_disabled;
	}
	spin_lock_irqsave(lock, irqflags);
	if (!(*irq_disabled)){
		disable_irq_nosync(irq);
		*irq_disabled = 1;
	}
	spin_unlock_irqrestore(lock, irqflags);
}
 
//开启中断
static void input_irq_enable(struct sar_ant_fnc_data *data, int dx)
{
 
	unsigned long irqflags;
	spinlock_t *lock;
	int irq;
	int *irq_disabled;
 
	if (dx == data->ant1_status){
		lock = &data->ant1_irq_lock;
		irq = data->ant1_irq;
		irq_disabled = &data->ant1_irq_disabled;
	}
	else{
		lock = &data->ant2_irq_lock;
		irq = data->ant2_irq;
		irq_disabled = &data->ant2_irq_disabled;
	}
	spin_lock_irqsave(lock, irqflags);
	if (*irq_disabled){
		enable_irq(irq);
		*irq_disabled = 0;
	}
	spin_unlock_irqrestore(lock, irqflags);
}
 
//上报按键键值
static void report_event(unsigned int event_value)
{
	printk("---------event_value =%d------------\n",event_value);
	input_report_key(sar_ant_data->input_device,event_value, 1);
	input_sync(sar_ant_data->input_device);
	input_report_key(sar_ant_data->input_device,event_value, 0);
	input_sync(sar_ant_data->input_device);
}
 
//消抖后上报键值
static void ant1_input_timer_function(struct timer_list *data)
{
	int value = gpio_get_value(sar_ant_data->ant1_status);
	
//消抖
	if(value != ant1_old_status){
		ant1_old_status = -1;
		input_irq_enable(sar_ant_data, sar_ant_data->ant1_status);
		return;
	}
	
    if(0 == value){
    	report_event(KEY_SAR85_DOWN);   //测试 return;
	}else{		
        report_event(KEY_SAR85_UP);  // 高电平上报KEY_F5
	}

	printk("ant1 input_sar_key ,status :%d\n", value);
 
	ant1_old_status = -1;
	input_irq_enable(sar_ant_data, sar_ant_data->ant1_status);
}
 
//ant2 消抖处理
static void ant2_input_timer_function(struct timer_list *data)
{
	int value = gpio_get_value(sar_ant_data->ant2_status);
//消抖
	if(value != ant2_old_status){
		ant2_old_status = -1;
		input_irq_enable(sar_ant_data, sar_ant_data->ant2_status);
		return;
	}
 
	if(0 == value){
    	report_event(KEY_SAR87_DOWN);   //测试 return;
	}else{
		report_event(KEY_SAR87_UP);   //ant2开启
	}

	printk("ant2 input_sar_key ,status :%d\n", value);
 
	ant2_old_status = -1;
	input_irq_enable(sar_ant_data, sar_ant_data->ant2_status);
}
 
// ant1状态改变触发的中断处理函数
static irqreturn_t input_irq_ant1_handler(int irq, void *dev_id)
{
	input_irq_disable(sar_ant_data, sar_ant_data->ant1_status);
	if(irq==sar_ant_data->ant1_irq){
       		ant1_old_status = gpio_get_value(sar_ant_data->ant1_status);
        	mod_timer(&ant1_input_timer,jiffies+HZ/50); //开启20ms防抖定时器
    	}
 
	return IRQ_HANDLED;
}
 
// ant2状态改变触发的中断处理函数
static irqreturn_t input_irq_ant2_status_handler(int irq, void *dev_id)
{
 
	input_irq_disable(sar_ant_data, sar_ant_data->ant2_status);
	if(irq==sar_ant_data->ant2_irq){
       		ant2_old_status = gpio_get_value(sar_ant_data->ant2_status);
        	mod_timer(&ant2_input_timer,jiffies+HZ/50); //开启20ms的防抖处理
    	}
 
	return IRQ_HANDLED;
}
 
//获取设备树中关于IO口并设置管脚的初始功能，这里主要关注上下拉状态
static int customer_gpio_configure(struct sar_ant_fnc_data *ddata, bool active)
{
	struct pinctrl_state *set_state;
	int retval;
 
	if (active) {
		set_state = pinctrl_lookup_state(ddata->customer_pinctrl, "ant_active");
		if (IS_ERR(set_state)) {
			printk(KERN_ERR " %s: cannot get ts pinctrl active state\n", __func__);
			return PTR_ERR(set_state);
		}
	} else {
		set_state = pinctrl_lookup_state(ddata->customer_pinctrl, "ant_suspend");
		if (IS_ERR(set_state)) {
			printk(KERN_ERR "%s: cannot get gpiokey pinctrl sleep state\n", __func__);
			return PTR_ERR(set_state);
		}
	}
 
    retval = pinctrl_select_state(ddata->customer_pinctrl, set_state);
	if (retval) {
		printk(KERN_ERR "%s: cannot set ts pinctrl active state\n", __func__);
		return retval;
	}
	return 0;
}
 
static int gpio_parse_dt(struct device *dev, struct sar_ant_fnc_data *pdata)
{
	enum of_gpio_flags flags;
	struct device_node *np = dev->of_node;
	int error;
 
	//获取ant1管脚，并设置成输入状态
	pdata->ant1_status = of_get_named_gpio_flags(np, "ant1", 0, &flags);
	if (pdata->ant1_status < 0)
		return pdata->ant1_status;
	if (gpio_is_valid(pdata->ant1_status)) {
		error = gpio_request(pdata->ant1_status, "ant1");
		if (error) {
			printk(KERN_ERR "%s %d: gpio request failed", __func__,__LINE__);
			return error;
		}
		error = gpio_direction_input(pdata->ant1_status);
		if (error) {
			printk(KERN_ERR "%s %d: gpio request failed", __func__,__LINE__);
			return error;
		}
	}
 
	//设置ant2状态管脚并设置成输入状态
	pdata->ant2_status = of_get_named_gpio_flags(np, "ant2", 0, &flags);
	if (pdata->ant2_status < 0)
		return pdata->ant2_status;
	if (gpio_is_valid(pdata->ant2_status)) {
		error = gpio_request(pdata->ant2_status, "ant2");
		if (error) {
			printk(KERN_ERR "%s %d: gpio request failed", __func__,__LINE__);
			return error;
		}
		error = gpio_direction_input(pdata->ant2_status);
		if (error) {
			printk(KERN_ERR "%s %d: gpio request failed", __func__,__LINE__);
			return error;
		}
	}
	return 0;
}
 
//初始化输入系统
static int init_input_dev(struct platform_device *pdev)
{
	struct input_dev *_input_device = NULL;
 
	_input_device = input_allocate_device();
	if (_input_device == NULL){
		printk("*%s %d* _input_device allocate device err!\n", __FUNCTION__, __LINE__);
		return -1;
	}
//设置需要上报的键值，主要有F4 F5
	set_bit(EV_KEY, _input_device->evbit);
	set_bit(KEY_SAR85_DOWN, _input_device->keybit);
	set_bit(KEY_SAR85_UP, _input_device->keybit);
	set_bit(KEY_SAR87_DOWN, _input_device->keybit);
	set_bit(KEY_SAR87_UP, _input_device->keybit);
 
    input_set_drvdata(_input_device, sar_ant_data);
    if (input_register_device(_input_device)){
		printk("*%s %d* Unable to register %s _input_device device!\n", __FUNCTION__, __LINE__, _input_device->name);
        input_free_device(_input_device);
        return -1;
    }
    _input_device->name = "sar_ant_key";
	sar_ant_data->input_device = _input_device;
    return 0;
}
 
// 驱动主入口 probe函数，调用上面的各种方法
static int sar_ant_fnc_probe(struct platform_device *pdev)
{
	int error;
	printk("*%s sar_ant  probe \n", __FUNCTION__);
	sar_ant_data = kzalloc(sizeof(struct sar_ant_fnc_data), GFP_KERNEL);
	if (sar_ant_data == NULL){
		return -ENOMEM;
	}
 
	sar_ant_data->ant2_irq_disabled = 0;
	sar_ant_data->ant1_irq_disabled = 0;
 
	/* get pinctrl*/
	sar_ant_data->customer_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(sar_ant_data->customer_pinctrl)) {
		if (PTR_ERR(sar_ant_data->customer_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		printk(KERN_ERR "%s: Target does not use pinctrl\n", __func__);
		sar_ant_data->customer_pinctrl = NULL;
	}
	/*set gpio default state*/
	if (sar_ant_data->customer_pinctrl) {
		error = customer_gpio_configure(sar_ant_data, true);
		if (error) {
			printk(KERN_ERR "%s: cannot set ts pinctrl active state\n", __func__);
			return error;
		}
	}
	/* get dts config*/
	error = gpio_parse_dt(&pdev->dev, sar_ant_data);
	if (error) {
		printk(KERN_ERR "%s: Failed to parse dt\n", __func__);
		error = -ENOMEM;
		return error;
	}
	/* set irq*/
	sar_ant_data->ant1_irq = gpio_to_irq(sar_ant_data->ant1_status);
	if (sar_ant_data->ant1_irq<=0) 
    		printk(KERN_ERR "%s: ant1_status  request failed", __func__);
 
	sar_ant_data->ant2_irq = gpio_to_irq(sar_ant_data->ant2_status);
	if (sar_ant_data->ant2_irq<=0) 
    		printk(KERN_ERR "%s: ant2_irq  request failed", __func__);
 
	spin_lock_init(&sar_ant_data->ant1_irq_lock);
	spin_lock_init(&sar_ant_data->ant2_irq_lock);
 
	/* request irq*/
	if (request_irq(sar_ant_data->ant1_irq, input_irq_ant1_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING , "ant1_irq", sar_ant_data)) {
		printk("*%s %d* request ant1_irq  err\n", __FUNCTION__, __LINE__);
		kfree(sar_ant_data);
		return -EINVAL;
    }
    if (request_irq(sar_ant_data->ant2_irq, input_irq_ant2_status_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "ant2_irq", sar_ant_data)){
		printk("*%s %d* request ant2_irq err\n", __FUNCTION__, __LINE__);
		free_irq(sar_ant_data->ant1_irq, sar_ant_data);
		kfree(sar_ant_data);
		return -EINVAL;
    }
	/* init input device */
	if (init_input_dev(pdev))
    {
		printk("*%s %d* init  input device err\n", __FUNCTION__, __LINE__);
		free_irq(sar_ant_data->ant1_irq, sar_ant_data);
		free_irq(sar_ant_data->ant2_irq, sar_ant_data);
		kfree(sar_ant_data);
		return -EINVAL;
    }
 
	printk("%s success\n", __FUNCTION__);
	
	return 0;
}
 
//驱动退出函数
static int  sar_ant_fnc_remove(struct platform_device *pdev)
{
	free_irq(sar_ant_data->ant1_irq, sar_ant_data);
	free_irq(sar_ant_data->ant2_irq, sar_ant_data);
	kfree(sar_ant_data);
	del_timer_sync(&ant2_input_timer);
	del_timer_sync(&ant1_input_timer);
	return 0;
}
 
//驱动和设备树匹配ID，只要匹配到dts的设备才会调用probe函数
static struct of_device_id sar_ant_fnc_of_match[] = {
        { .compatible = "sar_ant_key" },
        { }
};
MODULE_DEVICE_TABLE(of, sar_ant_fnc_of_match);
 
//驱动结构体
static struct platform_driver sar_ant_fnc_driver = {
        .driver         = {
                .name           = "sar_ant_fnc",
                .owner          = THIS_MODULE,
                .of_match_table = of_match_ptr(sar_ant_fnc_of_match),
        },
        .probe          = sar_ant_fnc_probe,
        .remove         = sar_ant_fnc_remove,
};
 
//注册平台设备驱动
static int __init sar_ant_fnc_init(void)
{
	return platform_driver_register(&sar_ant_fnc_driver);
}
 
static void __exit sar_ant_fnc_exit(void)
{
	platform_driver_unregister(&sar_ant_fnc_driver);
}
module_init(sar_ant_fnc_init);
module_exit(sar_ant_fnc_exit);
MODULE_LICENSE("GPL");
