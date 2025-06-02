// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * apml_sbtsi.c - hwmon driver for a SBI Temperature Sensor Interface (SB-TSI)
 *                compliant AMD SoC temperature device.
 * 		   Also register to misc driver with an IOCTL.
 *
 * Copyright (c) 2020, Google Inc.
 * Copyright (c) 2020, Kun Yi <kunyi@google.com>
 * Copyright (C) 2022 Advanced Micro Devices, Inc.
 */

#include <linux/err.h>
#include <linux/fs.h>
#include <linux/hwmon.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include <linux/amd-apml.h>

/*
 * SB-TSI registers only support SMBus byte data access. "_INT" registers are
 * the integer part of a temperature value or limit, and "_DEC" registers are
 * corresponding decimal parts.
 */
#define SBTSI_REG_TEMP_INT		0x01 /* RO */
#define SBTSI_REG_STATUS		0x02 /* RO */
#define SBTSI_REG_CONFIG		0x03 /* RO */
#define SBTSI_REG_TEMP_HIGH_INT		0x07 /* RW */
#define SBTSI_REG_TEMP_LOW_INT		0x08 /* RW */
#define SBTSI_REG_TEMP_DEC		0x10 /* RW */
#define SBTSI_REG_TEMP_HIGH_DEC		0x13 /* RW */
#define SBTSI_REG_TEMP_LOW_DEC		0x14 /* RW */

#define TBAI_WR_LEN			0x4  /* Write length */
#define TBAI_FLUSH_RD_LEN		0x4  /* flush buffer read length */
#define MAX_PROTO_RD_SZ			32   /* Maximum bytes read in one transaction */
#define DWORD_TO_BYTES			0x4  /* Number of bytes in dword */
/* Maximum dwords possible to read in one transaction */
#define MAX_DWORDS_READ			0x8

#define SBTSI_CONFIG_READ_ORDER_SHIFT	5

#define SBTSI_TEMP_MIN	0
#define SBTSI_TEMP_MAX	255875
#define TB_ACQUIRE	0x31
#define TB_FLUSH	0x32

/*
 * SBTSI_STEP_INC Fractional portion of temperature
 * One increment of these bits is equivalent to a step of 0.125 °C
 *
 * SBTSI_INT_OFFSET Integer offset for temperature value
 *
 * SBTSI_DEC_OFFSET offset for decimal bits in register[7:5]
 *
 * SBTSI_DEC_MASK Mask for decimal value
 */
#define SBTSI_STEP_INC		125
#define SBTSI_INT_OFFSET	3
#define SBTSI_DEC_OFFSET	5
#define SBTSI_DEC_MASK		0x7

struct apml_sbtsi_device {
	struct miscdevice sbtsi_misc_dev;
	struct i2c_client *client;
	struct i3c_device *i3cdev;
	struct regmap *regmap;
	struct mutex lock;
	u8 dev_static_addr;
} __packed;

/*
 * From SB-TSI spec: CPU temperature readings and limit registers encode the
 * temperature in increments of 0.125 from 0 to 255.875. The "high byte"
 * register encodes the base-2 of the integer portion, and the upper 3 bits of
 * the "low byte" encode in base-2 the decimal portion.
 *
 * e.g. INT=0x19, DEC=0x20 represents 25.125 degrees Celsius
 *
 * Therefore temperature in millidegree Celsius =
 *   (INT + DEC / 256) * 1000 = (INT * 8 + DEC / 32) * 125
 */
static inline int sbtsi_reg_to_mc(s32 integer, s32 decimal)
{
	return ((integer << SBTSI_INT_OFFSET) +
	       (decimal >> SBTSI_DEC_OFFSET)) * SBTSI_STEP_INC;
}

/*
 * Inversely, given temperature in millidegree Celsius
 *   INT = (TEMP / 125) / 8
 *   DEC = ((TEMP / 125) % 8) * 32
 * Caller have to make sure temp doesn't exceed 255875, the max valid value.
 */
static inline void sbtsi_mc_to_reg(s32 temp, u8 *integer, u8 *decimal)
{
	temp /= SBTSI_STEP_INC;
	*integer = temp >> SBTSI_INT_OFFSET;
	*decimal = (temp & SBTSI_DEC_MASK) << SBTSI_DEC_OFFSET;
}

static int sbtsi_read(struct device *dev, enum hwmon_sensor_types type,
		      u32 attr, int channel, long *val)
{
	struct apml_sbtsi_device *tsi_dev = dev_get_drvdata(dev);
	unsigned int temp_int, temp_dec, cfg;
	int ret;

	switch (attr) {
	case hwmon_temp_input:
		/*
		 * ReadOrder bit specifies the reading order of integer and
		 * decimal part of CPU temp for atomic reads. If bit == 0,
		 * reading integer part triggers latching of the decimal part,
		 * so integer part should be read first. If bit == 1, read
		 * order should be reversed.
		 */
		ret = regmap_read(tsi_dev->regmap, SBTSI_REG_CONFIG, &cfg);
		if (ret < 0)
			return ret;

		mutex_lock(&tsi_dev->lock);
		if (cfg & BIT(SBTSI_CONFIG_READ_ORDER_SHIFT)) {
			ret = regmap_read(tsi_dev->regmap, SBTSI_REG_TEMP_DEC, &temp_dec);
			ret = regmap_read(tsi_dev->regmap, SBTSI_REG_TEMP_INT, &temp_int);
		} else {
			ret = regmap_read(tsi_dev->regmap, SBTSI_REG_TEMP_INT, &temp_int);
			ret = regmap_read(tsi_dev->regmap, SBTSI_REG_TEMP_DEC, &temp_dec);
		}
		mutex_unlock(&tsi_dev->lock);
		break;
	case hwmon_temp_max:
		mutex_lock(&tsi_dev->lock);
		ret = regmap_read(tsi_dev->regmap, SBTSI_REG_TEMP_HIGH_INT, &temp_int);
		ret = regmap_read(tsi_dev->regmap, SBTSI_REG_TEMP_HIGH_DEC, &temp_dec);
		mutex_unlock(&tsi_dev->lock);
		break;
	case hwmon_temp_min:
		mutex_lock(&tsi_dev->lock);
		ret = regmap_read(tsi_dev->regmap, SBTSI_REG_TEMP_LOW_INT, &temp_int);
		ret = regmap_read(tsi_dev->regmap, SBTSI_REG_TEMP_LOW_DEC, &temp_dec);
		mutex_unlock(&tsi_dev->lock);
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	//*val = sbtsi_reg_to_mc(temp_int, temp_dec);
	// Report the Temp in C (rather than mC)
	*val = temp_int;

	return 0;
}

static int sbtsi_write(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long val)
{
	struct apml_sbtsi_device *tsi_dev = dev_get_drvdata(dev);
	unsigned int temp_int, temp_dec;
	int reg_int, reg_dec, err;

	switch (attr) {
	case hwmon_temp_max:
		reg_int = SBTSI_REG_TEMP_HIGH_INT;
		reg_dec = SBTSI_REG_TEMP_HIGH_DEC;
		break;
	case hwmon_temp_min:
		reg_int = SBTSI_REG_TEMP_LOW_INT;
		reg_dec = SBTSI_REG_TEMP_LOW_DEC;
		break;
	default:
		return -EINVAL;
	}

	val = clamp_val(val, SBTSI_TEMP_MIN, SBTSI_TEMP_MAX);
	sbtsi_mc_to_reg(val, (u8 *)&temp_int, (u8 *)&temp_dec);

	mutex_lock(&tsi_dev->lock);
	err = regmap_write(tsi_dev->regmap, reg_int, temp_int);
	if (err)
		goto exit;

	err = regmap_write(tsi_dev->regmap, reg_dec, temp_dec);
exit:
	mutex_unlock(&tsi_dev->lock);
	return err;
}

static umode_t sbtsi_is_visible(const void *data,
				enum hwmon_sensor_types type,
				u32 attr, int channel)
{
	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		case hwmon_temp_min:
			return 0644;
		case hwmon_temp_max:
			return 0644;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct hwmon_channel_info *sbtsi_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX),
	NULL
};

static const struct hwmon_ops sbtsi_hwmon_ops = {
	.is_visible = sbtsi_is_visible,
	.read = sbtsi_read,
	.write = sbtsi_write,
};

static const struct hwmon_chip_info sbtsi_chip_info = {
	.ops = &sbtsi_hwmon_ops,
	.info = sbtsi_info,
};

static int tbai_protocol(struct apml_sbtsi_device *tsi_dev, u8 cmd, u8 *input,
			 u8 count, u8 *output)
{
	struct i3c_priv_xfer xfers[] = {
		{
			.rnw = 1,
			.len = count,
			.data.out = output,
		},
	};
	int ret;

	if (!tsi_dev->i3cdev)
		return -EOPNOTSUPP;

	ret = regmap_bulk_write(tsi_dev->regmap, cmd, input, TBAI_WR_LEN);
	if (ret < 0)
		return ret;

	return i3c_device_do_priv_xfers(tsi_dev->i3cdev, xfers, 1);
}

static int flush_trace_buffer(struct apml_sbtsi_device *tsi_dev, struct apml_tbai_msg *tbai_msg)
{
	u8 input[4] = {0};
	u8 output[4] = {0};
	int ret, i;

	ret = tbai_protocol(tsi_dev, tbai_msg->reg_in[TBAI_CMD_INDEX], input, 4, output);
	if (ret)
		return ret;
	for (i = 0; i < TBAI_FLUSH_RD_LEN; i++)
		tbai_msg->data_out.bytes_out[i] = output[i];
	return ret;
}

static int acquire_trace_buffer(struct apml_sbtsi_device *tsi_dev, struct apml_tbai_msg *tbai_msg)
{
	int dword_read, dword_remain, i, j, ret;
	u16 offset, offset_new;
	u8 input[TBAI_WR_LEN] = {0};
	/* TODO: static memory as max supported is 8 Dwords */
	u8 *output;

	/* Dwords to read from user*/
	dword_remain = tbai_msg->reg_in[TBAI_DWORD_RD_INDEX];
	/* Extract the offset to update, if more than 8 Dwords require to read */
	offset = tbai_msg->reg_in[TBAI_OFFSET_HI] << 8 |
		 tbai_msg->reg_in[TBAI_OFFSET_LO];

	/* If Dwords to read is 0 or more than 32, return */
	if (tbai_msg->reg_in[TBAI_DWORD_RD_INDEX] == 0 ||
	    tbai_msg->reg_in[TBAI_DWORD_RD_INDEX] > MAX_TBAI_DWORDS)
		return -EINVAL;

	/*
	 * Set required variables to read dwords
	 * Maximum dwords supported from i3c protocol is 8
	 */
	for (i = 0; i <= tbai_msg->reg_in[TBAI_DWORD_RD_INDEX] / 8 &&
	     dword_remain > 0; i++) {
		if (dword_remain > MAX_DWORDS_READ) {
			dword_remain -= MAX_DWORDS_READ;
			dword_read = MAX_DWORDS_READ;
		} else {
			dword_read = dword_remain;
			dword_remain = 0;
		}
		/* update offset if more than 8 Dwords require to read */
		offset_new = i * MAX_PROTO_RD_SZ + offset;
		input[0] = tbai_msg->reg_in[TBAI_LUT_INDEX];
		input[1] = offset_new & 0xFF;
		input[2] = (offset_new >> 8) & 0xFF;
		input[3] = dword_read - 1;

		/*
		 * TODO: Optimize to allocate memory at once as per user request
		 * Currently in A0, only one Dword can be read, due to bug.
		 * Optimize in B0.
		 */
		output = kcalloc(dword_read * DWORD_TO_BYTES, sizeof(u8), GFP_KERNEL);
		if (!output)
			return -ENOMEM;

		ret = tbai_protocol(tsi_dev, tbai_msg->reg_in[TBAI_CMD_INDEX],
				    input, dword_read * DWORD_TO_BYTES, output);
		if (ret) {
			kfree(output);
			return ret;
		}
		for (j = 0; j < dword_read * DWORD_TO_BYTES; j++) {
			/*
			 * TODO: In A0, only one Dword is supported
			 * APML module is optimized to read max of 32 Dwords at a time.
			 * dwords exceeding 8 need to be tested in B0 platform.
			 */
			tbai_msg->data_out.bytes_out[j + (i * MAX_PROTO_RD_SZ)] = output[j];
		}
		kfree(output);
	}
	return 0;
}

static long sbtsi_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int __user *arguser = (int  __user *)arg;
	struct apml_message msg = { 0 };
	struct apml_tbai_msg tbai_msg = {0};
	struct apml_sbtsi_device *tsi_dev;
	int ret;

	switch (cmd) {
	case SBRMI_IOCTL_CMD:
		if (copy_struct_from_user(&msg, sizeof(msg), arguser, sizeof(struct apml_message)))
			return -EFAULT;

		if (msg.cmd != APML_REG)
			return -EINVAL;

		tsi_dev = container_of(fp->private_data, struct apml_sbtsi_device, sbtsi_misc_dev);
		if (!tsi_dev)
			return -EFAULT;

		mutex_lock(&tsi_dev->lock);

		if (!msg.data_in.reg_in[RD_FLAG_INDEX]) {
			ret = regmap_write(tsi_dev->regmap,
					   msg.data_in.reg_in[REG_OFF_INDEX],
					   msg.data_in.reg_in[REG_VAL_INDEX]);
		} else {
			ret = regmap_read(tsi_dev->regmap,
					  msg.data_in.reg_in[REG_OFF_INDEX],
					  (int *)&msg.data_out.reg_out[RD_WR_DATA_INDEX]);
			if (ret)
				goto out;

			if (copy_to_user(arguser, &msg, sizeof(struct apml_message)))
				ret = -EFAULT;
		}
out:
		mutex_unlock(&tsi_dev->lock);
		return ret;
	case SBTBAI_IOCTL_CMD:
		if (copy_struct_from_user(&tbai_msg, sizeof(tbai_msg), arguser,
					  sizeof(struct apml_tbai_msg)))
			return -EFAULT;

		tsi_dev = container_of(fp->private_data, struct apml_sbtsi_device, sbtsi_misc_dev);
		if (!tsi_dev)
			return -EFAULT;

		mutex_lock(&tsi_dev->lock);
		if (tbai_msg.reg_in[TBAI_CMD_INDEX] == TB_ACQUIRE)
			ret = acquire_trace_buffer(tsi_dev, &tbai_msg);
		else if (tbai_msg.reg_in[TBAI_CMD_INDEX] == TB_FLUSH)
			ret = flush_trace_buffer(tsi_dev, &tbai_msg);
		else
			ret = -EINVAL;

		if (ret)
			goto tbai_exit;
		if (copy_to_user(arguser, &tbai_msg, sizeof(struct apml_tbai_msg)))
			ret = -EFAULT;
tbai_exit:
		mutex_unlock(&tsi_dev->lock);
		return ret;
	default:
		break;
	}
	return 0;
}

static const struct file_operations sbtsi_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= sbtsi_ioctl,
	.compat_ioctl	= sbtsi_ioctl,
};

static int create_misc_tsi_device(struct apml_sbtsi_device *tsi_dev,
				  struct device *dev)
{
	int ret;

	tsi_dev->sbtsi_misc_dev.name		= devm_kasprintf(dev, GFP_KERNEL,
						  "sbtsi-%x", tsi_dev->dev_static_addr);
	tsi_dev->sbtsi_misc_dev.minor		= MISC_DYNAMIC_MINOR;
	tsi_dev->sbtsi_misc_dev.fops		= &sbtsi_fops;
	tsi_dev->sbtsi_misc_dev.parent		= dev;
	tsi_dev->sbtsi_misc_dev.nodename	= devm_kasprintf(dev, GFP_KERNEL,
						  "sbtsi-%x", tsi_dev->dev_static_addr);
	tsi_dev->sbtsi_misc_dev.mode		= 0600;

	ret = misc_register(&tsi_dev->sbtsi_misc_dev);
	if (ret)
		return ret;

	dev_info(dev, "register %s device\n", tsi_dev->sbtsi_misc_dev.name);
	return ret;
}

static int sbtsi_i3c_probe(struct i3c_device *i3cdev)
{
	struct device *dev = &i3cdev->dev;
	struct device *hwmon_dev;
	struct apml_sbtsi_device *tsi_dev;
	struct regmap_config sbtsi_i3c_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	struct regmap *regmap;

	dev_err(dev, "SBTSI: PID: %llx\n", i3cdev->desc->info.pid);
	if (!(I3C_PID_INSTANCE_ID(i3cdev->desc->info.pid) == 0 ||
	      i3cdev->desc->info.pid == 0x22400000001)) {
		dev_err(dev, "SBTSI: Error PID: %llx\n", i3cdev->desc->info.pid);
		return -ENXIO;
	}

	regmap = devm_regmap_init_i3c(i3cdev, &sbtsi_i3c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&i3cdev->dev, "Failed to register i3c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	tsi_dev = devm_kzalloc(dev, sizeof(struct apml_sbtsi_device), GFP_KERNEL);
	if (!tsi_dev)
	{
		dev_err(dev, "SBTSI: Error Mem All0c\n");
		return -ENOMEM;
	}

	tsi_dev->i3cdev = i3cdev;
	tsi_dev->regmap = regmap;
	mutex_init(&tsi_dev->lock);

	dev_set_drvdata(dev, (void *)tsi_dev);
	hwmon_dev = devm_hwmon_device_register_with_info(dev, "sbtsi_i3c", tsi_dev,
							 &sbtsi_chip_info, NULL);

	if (!hwmon_dev)
	{
		dev_err(dev, "SBTSI: Error hwmon_device_register \n" );
		return PTR_ERR_OR_ZERO(hwmon_dev);
	}

	/* Need to verify for the static address for i3cdev */
	tsi_dev->dev_static_addr = i3cdev->desc->info.static_addr;

	return create_misc_tsi_device(tsi_dev, dev);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int sbtsi_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *tsi_id)
#else
static int sbtsi_i2c_probe(struct i2c_client *client)
#endif
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct apml_sbtsi_device *tsi_dev;
	struct regmap_config sbtsi_i2c_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};

	tsi_dev = devm_kzalloc(dev, sizeof(struct apml_sbtsi_device), GFP_KERNEL);
	if (!tsi_dev)
		return -ENOMEM;

	mutex_init(&tsi_dev->lock);
	tsi_dev->regmap = devm_regmap_init_i2c(client, &sbtsi_i2c_regmap_config);
	tsi_dev->client = client;
	if (IS_ERR(tsi_dev->regmap))
		return PTR_ERR(tsi_dev->regmap);

	dev_set_drvdata(dev, (void *)tsi_dev);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 tsi_dev,
							 &sbtsi_chip_info,
							 NULL);

	if (!hwmon_dev)
		return PTR_ERR_OR_ZERO(hwmon_dev);

	tsi_dev->dev_static_addr = client->addr;

	return create_misc_tsi_device(tsi_dev, dev);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
static int sbtsi_i3c_remove(struct i3c_device *i3cdev)
#else
static void sbtsi_i3c_remove(struct i3c_device *i3cdev)
#endif
{
	struct apml_sbtsi_device *tsi_dev = dev_get_drvdata(&i3cdev->dev);

	if (tsi_dev)
		misc_deregister(&tsi_dev->sbtsi_misc_dev);

	dev_info(&i3cdev->dev, "Removed sbtsi-i3c driver\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
	return 0;
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int sbtsi_i2c_remove(struct i2c_client *client)
#else
static void sbtsi_i2c_remove(struct i2c_client *client)
#endif
{
	struct apml_sbtsi_device *tsi_dev = dev_get_drvdata(&client->dev);

	if (tsi_dev)
		misc_deregister(&tsi_dev->sbtsi_misc_dev);

	dev_info(&client->dev, "Removed sbtsi driver\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static const struct i3c_device_id sbtsi_i3c_id[] = {
                    /* (MID, PARTID, EXTIN, DRVDATA) */
	I3C_DEVICE_EXTRA_INFO(0, 0x0000, 0x118, NULL), /* P0 - IOD0 - SBTSI */
	I3C_DEVICE_EXTRA_INFO(0, 0x0001, 0x118, NULL), /* P0 - IOD1 - SBTST */
	I3C_DEVICE_EXTRA_INFO(0, 0x0100, 0x118, NULL), /* P1 - IOD0 - SBTSI */
	I3C_DEVICE_EXTRA_INFO(0, 0x0101, 0x118, NULL), /* P1 - IOD1 - SBTSI */
	I3C_DEVICE_EXTRA_INFO(0x112, 0, 0x1, NULL),
	I3C_DEVICE_EXTRA_INFO(0, 0x0, 0x0, NULL),
	{}
};
MODULE_DEVICE_TABLE(i3c, sbtsi_i3c_id);

static struct i3c_driver sbtsi_i3c_driver = {
	.driver = {
		.name = "sbtsi_i3c",
	},
	.probe = sbtsi_i3c_probe,
	.remove = sbtsi_i3c_remove,
	.id_table = sbtsi_i3c_id,
};

static const struct i2c_device_id sbtsi_id[] = {
	{"sbtsi", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sbtsi_id);

static const struct of_device_id __maybe_unused sbtsi_of_match[] = {
	{
		.compatible = "amd,sbtsi",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sbtsi_of_match);

static struct i2c_driver sbtsi_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "sbtsi",
		.of_match_table = of_match_ptr(sbtsi_of_match),
	},
	.probe = sbtsi_i2c_probe,
	.remove = sbtsi_i2c_remove,
	.id_table = sbtsi_id,
};

module_i3c_i2c_driver(sbtsi_i3c_driver, &sbtsi_driver)

MODULE_AUTHOR("Kun Yi <kunyi@google.com>");
MODULE_DESCRIPTION("Hwmon driver for AMD SB-TSI emulated sensor");
MODULE_LICENSE("GPL");
