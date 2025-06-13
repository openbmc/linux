/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * sbtsi-common.h - hwmon driver for a SBI Temperature Sensor Interface (SB-TSI)
 *		    compliant AMD SoC temperature device.
 *		    Also register to misc driver with an IOCTL.
 *
 * Copyright (c) 2020, Google Inc.
 * Copyright (c) 2020, Kun Yi <kunyi@google.com>
 * Copyright (C) 2025 Advanced Micro Devices, Inc.
 */

struct apml_sbtsi_device {
	struct miscdevice sbtsi_misc_dev;
	struct i2c_client *client;
	struct i3c_device *i3cdev;
	struct regmap *regmap;
	struct mutex lock;	//lock for tsi devices
	u8 dev_static_addr;
} __packed;

int sbtsi_match_i2c(struct device *dev, const void *data);
int sbtsi_match_i3c(struct device *dev, const void *data);
