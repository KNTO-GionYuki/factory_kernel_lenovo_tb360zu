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

#ifndef __WL2868C_H
#define __WL2868C_H

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/atomic.h>
#include <linux/regulator/consumer.h>
#include "ldo_common.h"


#define WL2866D_DEBUG 1



int wl2866d_camera_power_up(int out_iotype);
int wl2866d_camera_power_down(int out_iotype);
int wl2866d_camera_power_up_eeprom(void);
int wl2866d_camera_power_down_eeprom(void);
int wl2866d_camera_power_down_all(void);
int wl2866d_camera_power_up_all(void);

int cam_wl2868c_driver_init(void);
void cam_wl2868c_driver_exit(void);

struct wl2866d_chip {
	struct device *dev;
	struct i2c_client *client;
	int    en_gpio;
	struct regulator *vin1;
	struct regulator *vin2;
};

struct wl2866d_map {
	u8 reg;
	int value;
};


#endif /* __WL2866D_H */
