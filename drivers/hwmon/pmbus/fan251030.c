// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for fan251030
 *
 * Copyright (c) 2022 Onsemi Corporation.
 */
#include <linux/hwmon.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include "pmbus.h"

#define FAN251030_NUM_PAGES 1
enum chips { fan251015,fan251030};

static struct pmbus_driver_info fan251030_info = {
    .pages = FAN251030_NUM_PAGES,    
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
        
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_TEMP| PMBUS_HAVE_TEMP2|
		PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_INPUT,

};

static int fan251030_probe(struct i2c_client *client)
{
	struct pmbus_driver_info *info;
	u8 buf[I2C_SMBUS_BLOCK_MAX];
	int ret;

	dev_err(&client->dev, "fan251030_probe: start\n");
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_WORD_DATA |
				     I2C_FUNC_SMBUS_READ_BLOCK_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK))
	{
		dev_err(&client->dev, "fan251030_probe: Error i2c_check_functionality\n");
		return -ENODEV;
	}
	/* Read Manufacturer id */
	ret = i2c_smbus_read_i2c_block_data(client, PMBUS_IC_DEVICE_ID, I2C_SMBUS_BLOCK_DATA, buf);
	if (ret < 0) {
		dev_err(&client->dev, "fan251030_probe: Failed to read PMBUS_IC_DEVICE_ID\n");
		return ret;
	}

	info = devm_kmemdup(&client->dev, &fan251030_info, sizeof(*info), GFP_KERNEL);
	if (!info)
	{
		dev_err(&client->dev, "fan251030_probe: Failed in devm_kmemdup\n");
		return -ENOMEM;
	}

	dev_err(&client->dev, "fan251030_probe: end\n");
	return pmbus_do_probe(client, info);
}


static const struct i2c_device_id fan251030_id[] = {
    {"fan251015", fan251015},
	{"fan251030", fan251030},
	{}
};

MODULE_DEVICE_TABLE(i2c,fan251030_id);

/*
static const struct of_device_id __maybe_unused fan251030_of_match[] = {
    {.compatible = "onsemi,fan251015", .data = (void *)fan251015, },
    {.compatible = "onsemi,fan251030", .data = (void *)fan251030, },
	{}
};
*/
static const struct of_device_id fan251030_of_match[] = {
    {.compatible = "onsemi,fan251030", .data = (void *)fan251030, },
        {}
};


MODULE_DEVICE_TABLE(of, fan251030_of_match);

static struct i2c_driver fan251030_driver = {
    .driver = {
        .name = "fan251030",
		.of_match_table = of_match_ptr(fan251030_of_match),
    },
    .probe = fan251030_probe,
    .id_table = fan251030_id,
};

module_i2c_driver(fan251030_driver);

MODULE_AUTHOR("Shaojie Zhang");
MODULE_DESCRIPTION("PMBus driver for Onsemi fan251030");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
