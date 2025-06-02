// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for RAA229126
 *
 * Copyright (c) 2022 Renesas Corporation.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "pmbus.h"

#define RAA229126_NUM_PAGES 2

enum chip {
	raa229126,
	raa229639,
	raa229641
};

static struct pmbus_driver_info raa229126_info = {
	.pages = RAA229126_NUM_PAGES,
	.format[PSC_VOLTAGE_IN] = direct,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_CURRENT_IN] = direct,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_TEMPERATURE] = direct,
	.format[PSC_POWER] = direct,
	.m[PSC_TEMPERATURE] = 1,
	.m[PSC_VOLTAGE_OUT] = 1, .R[PSC_VOLTAGE_OUT] = 3,
	.m[PSC_VOLTAGE_IN] = 1, .R[PSC_VOLTAGE_IN] = 2,
	.m[PSC_CURRENT_OUT] = 1, .R[PSC_CURRENT_OUT] = 1, .m[PSC_CURRENT_IN] = 1, .R[PSC_CURRENT_IN] = 2,
	.m[PSC_POWER] = 1,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_IIN |
		   PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN | PMBUS_HAVE_POUT |
		   PMBUS_HAVE_TEMP,


	.func[1] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_IIN |
		   PMBUS_HAVE_IOUT | PMBUS_HAVE_PIN | PMBUS_HAVE_POUT |
		   PMBUS_HAVE_TEMP,


};

static int raa229126_probe(struct i2c_client *client)
{
	struct pmbus_driver_info *info;
	u8 buf[I2C_SMBUS_BLOCK_MAX];
	int ret;

	dev_err(&client->dev, "raa229126_probe: start\n");
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_WORD_DATA |
				     I2C_FUNC_SMBUS_READ_BLOCK_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK))
	{
		dev_err(&client->dev, "raa229126_probe: error in i2c_check_functionality\n");
		return -ENODEV;
	}

	/* Read Manufacturer id */
	ret = i2c_smbus_read_i2c_block_data(client, PMBUS_IC_DEVICE_ID, I2C_SMBUS_BLOCK_DATA, buf);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read PMBUS_IC_DEVICE_ID\n");
		return ret;
	}
	//if (ret != 4 || strncmp(buf, "\x00\x82\xd2\x49", 4)) {
	//	dev_err(&client->dev, "DEVICE_ID unrecognized\n");
	//	return -ENODEV;
	//}

	info = devm_kmemdup(&client->dev, &raa229126_info, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	return pmbus_do_probe(client, info);
}

static const struct i2c_device_id raa229126_id[] = {
	{"raa229126", raa229126},
	{"raa229639", raa229639},
	{"raa229641", raa229641},
    {}
};

MODULE_DEVICE_TABLE(i2c, raa229126_id);

static const struct of_device_id __maybe_unused raa229126_of_match[] = {
	{.compatible = "renesas,raa229126",.data = (void *)raa229126,},
	{.compatible = "renesas,raa229639",.data = (void *)raa229639,},
    {.compatible = "renesas,raa229641",.data = (void *)raa229641,},
    {}
};
MODULE_DEVICE_TABLE(of, raa229126_of_match);

static struct i2c_driver raa229126_driver = {
	.driver = {
		   .name = "raa229126",
		   .of_match_table = of_match_ptr(raa229126_of_match),
	},
	.probe = raa229126_probe,
	.id_table = raa229126_id,
};

module_i2c_driver(raa229126_driver);

MODULE_AUTHOR("Zhangsj29");
MODULE_DESCRIPTION("PMBus driver for Renesas RAA229126 family");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
