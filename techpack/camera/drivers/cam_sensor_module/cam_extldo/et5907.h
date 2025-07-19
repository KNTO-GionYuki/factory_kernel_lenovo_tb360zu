/*
 * Marvell 88PM80x Interface
 *
 * Copyright (C) 2012 Marvell International Ltd.
 * Qiao Zhou <zhouqiao@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ET5907_H
#define __ET5907_H

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/atomic.h>
#include <linux/regulator/consumer.h>
#include "ldo_common.h"

#define ET5907_DEBUG 1


int et5907_camera_power_up(int out_iotype);
int et5907_camera_power_down(int out_iotype);
int et5907_camera_power_down_all(void);
int et5907_camera_power_up_all(void);

int cam_et5907_driver_init(void);
void cam_et5907_driver_exit(void);

struct et5907_chip {
	struct device *dev;
	struct i2c_client *client;
	int    en_gpio;
	struct regulator *vin1;
	struct regulator *vin2;
};

struct et5907_map {
	u8 reg;
	int value;
};



#endif /* __WL2866D_H */
