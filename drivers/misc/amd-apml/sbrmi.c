// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * sbrmi.c - hwmon driver for a SB-RMI mailbox
 *           compliant AMD SoC device.
 *
 * Copyright (C) 2021-2022 Advanced Micro Devices, Inc.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "sbrmi-common.h"

/* Do not allow setting negative power limit */
#define SBRMI_PWR_MIN	0

/* Try 2 byte address size before switching to 1 byte */
#define MAX_RETRY	5


/* SBRMI REVISION REG */
#define SBRMI_REV	0x0

#define MAX_WAIT_TIME_SEC	(3)

/* SBRMI registers data out is 1 byte */
#define SBRMI_REG_DATA_SIZE		0x1
/* Default SBRMI register address is 1 byte */
#define SBRMI_REG_ADDR_SIZE_DEF		0x1
/* TURIN SBRMI register address is 2 byte */
#define SBRMI_REG_ADDR_SIZE_TWO_BYTE	0x2

/* Two xfers, one write and one read require to read the data */
#define I3C_I2C_MSG_XFER_SIZE		0x2

/* DIMM temp */
#define DIMM_BASE_ID          (0x80)
#define DIMM_TEMP_OFFSET      (21)

/* to hold the data.in, and data.out fields of for
 * i3c_transfer to work when the MIPI driver is in
 * DMA mode.
 */
struct i3c_sbrmi_data {
	u32 reg;
	u32 val;
};
static struct kmem_cache *i3c_sbrmi_cache;

static int configure_regmap(struct apml_sbrmi_device *rmi_dev);

enum sbrmi_msg_id {
	SBRMI_READ_PKG_PWR_CONSUMPTION = 0x1,
	SBRMI_WRITE_PKG_PWR_LIMIT,
	SBRMI_READ_PKG_PWR_LIMIT,
	SBRMI_READ_PKG_MAX_PWR_LIMIT,
	SBRMI_READ_DIMM_THERMAL_SENSOR = 0x48,
};

static int sbrmi_get_max_pwr_limit(struct apml_sbrmi_device *rmi_dev)
{
	struct apml_message msg = { 0 };
	int ret = 0;

	msg.cmd = SBRMI_READ_PKG_MAX_PWR_LIMIT;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;
	ret = rmi_mailbox_xfer(rmi_dev, &msg);
	if (ret < 0)
		return ret;
	rmi_dev->pwr_limit_max = msg.data_out.mb_out[RD_WR_DATA_INDEX];

	return ret;
}

static int sbrmi_read(struct device *dev, enum hwmon_sensor_types type,
		      u32 attr, int channel, long *val)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(dev);
	struct apml_message msg = { 0 };
	int ret = 0;

	if ((type != hwmon_power) && (type != hwmon_temp))
		return -EINVAL;
	/* Configure regmap if not configured yet */
	if (!rmi_dev->regmap) {
		ret = configure_regmap(rmi_dev);
		if (ret < 0) {
			pr_err("regmap configuration failed with return value:%d in hwmon read ops\n", ret);
			return ret;
		}
	}

	mutex_lock(&rmi_dev->lock);
	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;

	switch (attr) {
	case hwmon_power_input:
		msg.cmd = SBRMI_READ_PKG_PWR_CONSUMPTION;
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case hwmon_power_cap:
		msg.cmd = SBRMI_READ_PKG_PWR_LIMIT;
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case hwmon_power_cap_max:
		if (!rmi_dev->pwr_limit_max) {
			/* Cache maximum power limit */
			ret = sbrmi_get_max_pwr_limit(rmi_dev);
		}
		msg.data_out.mb_out[RD_WR_DATA_INDEX] = rmi_dev->pwr_limit_max;
		break;
	case hwmon_temp_input:
		msg.cmd = SBRMI_READ_DIMM_THERMAL_SENSOR;
		msg.data_in.mb_in[RD_WR_DATA_INDEX] = (DIMM_BASE_ID + channel);
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		if (ret < 0)
		{
			//TBD: Temporary work-around for phosphor PID control operation
			//     Return 0 instead of the error
			*val = -1;
			ret = 0;
		}
		break;

	default:
		ret = -EINVAL;
	}
	if (!ret)
	{
		if (type == hwmon_power)
		{
			// hwmon power attributes are in microWatt
			*val = (long)msg.data_out.mb_out[RD_WR_DATA_INDEX] * 1000;
		}
		else if (type == hwmon_temp)
		{
			if(*val == -1)
			{
				// change DIMM Temp value to 0
				*val = 0;
			}
			else
			{
				// sbrmi temp is floating point, convert to deg C rational num
				*val = msg.data_out.mb_out[RD_WR_DATA_INDEX] >> DIMM_TEMP_OFFSET;
				*val = (*val / 4);
			}
			// TBD: Report in C rather than mC for phosphor PID fan control
			//*val= (tmp * 1000);
		}
	}

	mutex_unlock(&rmi_dev->lock);
	return ret;
}

static int sbrmi_write(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long val)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(dev);
	struct apml_message msg = { 0 };
	int ret;

	if (type != hwmon_power && attr != hwmon_power_cap)
		return -EINVAL;
	/* Configure regmap if not configured yet */
	if (!rmi_dev->regmap) {
		ret = configure_regmap(rmi_dev);
		if (ret < 0) {
			pr_err("regmap configuration failed with return value:%d in hwmon write ops\n", ret);
			return ret;
		}
	}
	/*
	 * hwmon power attributes are in microWatt
	 * mailbox read/write is in mWatt
	 */
	val /= 1000;

	val = clamp_val(val, SBRMI_PWR_MIN, rmi_dev->pwr_limit_max);

	msg.cmd = SBRMI_WRITE_PKG_PWR_LIMIT;
	msg.data_in.mb_in[RD_WR_DATA_INDEX] = val;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 0;

	mutex_lock(&rmi_dev->lock);
	ret = rmi_mailbox_xfer(rmi_dev, &msg);
	mutex_unlock(&rmi_dev->lock);
	return ret;
}

static umode_t sbrmi_is_visible(const void *data,
				enum hwmon_sensor_types type,
				u32 attr, int channel)
{
	switch (type) {
	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
		case hwmon_power_cap_max:
			return 0444;
		case hwmon_power_cap:
			return 0644;
		}
		break;
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct hwmon_channel_info *sbrmi_info[] = {
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_INPUT | HWMON_P_CAP | HWMON_P_CAP_MAX),
	HWMON_CHANNEL_INFO(temp,
		HWMON_T_INPUT, HWMON_T_INPUT, HWMON_T_INPUT, HWMON_T_INPUT,
		HWMON_T_INPUT, HWMON_T_INPUT, HWMON_T_INPUT, HWMON_T_INPUT,
		HWMON_T_INPUT, HWMON_T_INPUT, HWMON_T_INPUT, HWMON_T_INPUT,
		HWMON_T_INPUT, HWMON_T_INPUT, HWMON_T_INPUT, HWMON_T_INPUT),
	NULL
};

static const struct hwmon_ops sbrmi_hwmon_ops = {
	.is_visible = sbrmi_is_visible,
	.read = sbrmi_read,
	.write = sbrmi_write,
};

static const struct hwmon_chip_info sbrmi_chip_info = {
	.ops = &sbrmi_hwmon_ops,
	.info = sbrmi_info,
};

static long sbrmi_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int __user *arguser = (int  __user *)arg;
	struct apml_message msg = { 0 };
	bool read = false;
	int ret = -EFAULT;
	struct apml_sbrmi_device *rmi_dev;

	rmi_dev = fp->private_data;
	if (!rmi_dev)
		return -ENODEV;

	/*
	 * If device remove/unbind is called do not allow new transaction
	 */
	if (atomic_read(&rmi_dev->no_new_trans))
		return -EBUSY;
	/* Copy the structure from user */
	if (copy_struct_from_user(&msg, sizeof(msg), arguser,
				  sizeof(struct apml_message)))
		return ret;
	/*
	 * Only one I2C/I3C transaction can happen at
	 * one time. Take lock across so no two protocol is
	 * invoked at same time, modifying the register value.
	 */
	mutex_lock(&rmi_dev->lock);
	/* Verify device unbind/remove is not invoked */
	if (atomic_read(&rmi_dev->no_new_trans)) {
		mutex_unlock(&rmi_dev->lock);
		return -EBUSY;
	}

	/* Is this a read/monitor/get request */
	if (msg.data_in.reg_in[RD_FLAG_INDEX])
		read = true;

	/*
	 * Set the in_progress variable to true, to wait for
	 * completion during unbind/remove of driver
	 */
	atomic_set(&rmi_dev->in_progress, 1);

	switch (msg.cmd) {
	case 0 ... 0x999:
		/* Mailbox protocol */
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case APML_CPUID:
		ret = rmi_cpuid_read(rmi_dev, &msg);
		break;
	case APML_MCA_MSR:
		/* MCAMSR protocol */
		ret = rmi_mca_msr_read(rmi_dev, &msg);
		break;
	case APML_REG:
		/* REG R/W */
		if (read) {
			ret = regmap_read(rmi_dev->regmap,
					  msg.data_in.mb_in[REG_OFF_INDEX],
					  &msg.data_out.mb_out[RD_WR_DATA_INDEX]);
		} else {
			ret = regmap_write(rmi_dev->regmap,
					    msg.data_in.mb_in[REG_OFF_INDEX],
					    msg.data_in.reg_in[REG_VAL_INDEX]);
		}
		break;
	default:
		break;
	}

	/* Send complete only if device is unbinded/remove */
	if (atomic_read(&rmi_dev->no_new_trans))
		complete(&rmi_dev->misc_fops_done);

	atomic_set(&rmi_dev->in_progress, 0);
	mutex_unlock(&rmi_dev->lock);

	/* Copy results back to user only for get/monitor commands and firmware failures */
	if ((read && !ret) || ret == -EPROTOTYPE) {
		if (copy_to_user(arguser, &msg, sizeof(struct apml_message)))
			ret = -EFAULT;
	}
	return ret;
}

static int sbrmi_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *mdev = filp->private_data;
	struct apml_sbrmi_device *rmi_dev = container_of(mdev, struct apml_sbrmi_device,
							 sbrmi_misc_dev);
	int ret = 0;

	if (!rmi_dev)
		return -ENODEV;

	if (!rmi_dev->regmap) {
		ret = configure_regmap(rmi_dev);
		if (ret < 0) {
			pr_err("regmap configuration failed with return value:%d in misc dev open\n", ret);
			return ret;
		}
	}
	filp->private_data = rmi_dev;
	return 0;
}

static int sbrmi_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static const struct file_operations sbrmi_fops = {
	.owner		= THIS_MODULE,
	.open		= sbrmi_open,
	.release	= sbrmi_release,
	.unlocked_ioctl	= sbrmi_ioctl,
	.compat_ioctl	= sbrmi_ioctl,
};

static int create_misc_rmi_device(struct apml_sbrmi_device *rmi_dev,
				  struct device *dev)
{
	int ret;

	rmi_dev->sbrmi_misc_dev.name		= devm_kasprintf(dev, GFP_KERNEL,
						  "sbrmi-%x", rmi_dev->dev_static_addr);
	rmi_dev->sbrmi_misc_dev.minor		= MISC_DYNAMIC_MINOR;
	rmi_dev->sbrmi_misc_dev.fops		= &sbrmi_fops;
	rmi_dev->sbrmi_misc_dev.parent		= dev;
	rmi_dev->sbrmi_misc_dev.nodename	= devm_kasprintf(dev, GFP_KERNEL,
						  "sbrmi-%x", rmi_dev->dev_static_addr);
	rmi_dev->sbrmi_misc_dev.mode		= 0600;

	ret = misc_register(&rmi_dev->sbrmi_misc_dev);
	if (ret)
		return ret;

	dev_info(dev, "register %s device\n", rmi_dev->sbrmi_misc_dev.name);
	return ret;
}

static int sbrmi_i2c_reg_read(struct i2c_client *i2cdev, int reg_size, u32 *val)
{
	struct i2c_msg xfer[I3C_I2C_MSG_XFER_SIZE];
	int reg = SBRMI_REV;
	int val_size = SBRMI_REG_DATA_SIZE;

	xfer[0].addr = i2cdev->addr;
	xfer[0].flags = 0;
	xfer[0].len = reg_size;
	xfer[0].buf = (void *)&reg;

	xfer[1].addr = i2cdev->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = val_size;
	xfer[1].buf = (void *)val;

	return i2c_transfer(i2cdev->adapter, xfer, I3C_I2C_MSG_XFER_SIZE);
}

static int sbrmi_i2c_identify_reg_addr_size(struct i2c_client *i2c, u32 *size, u32 *rev)
{
	u32 reg_size;
	int ret, i;

	reg_size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;

	/*
	 * Sending 1 byte address size in Turin cause unrecoverable error
	 * Before trying to switch to 1 bytes, retry. 
	 */
	for (i = 0; i < MAX_RETRY; i++) {
		ret = sbrmi_i2c_reg_read(i2c, reg_size, rev);
		if (ret != I3C_I2C_MSG_XFER_SIZE) {
			usleep_range(10000, 20000);
			continue;
		} else {
			break;
		}
	}
	if (ret != I3C_I2C_MSG_XFER_SIZE) {
		reg_size = SBRMI_REG_ADDR_SIZE_DEF;
		ret = sbrmi_i2c_reg_read(i2c, reg_size, rev);
		if (ret != I3C_I2C_MSG_XFER_SIZE) {
			pr_err("I2C reg read failed with return value:%d\n", ret);
			return ret;
		}
	}

	if (*rev == 0x21)
		*size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
	else
		*size = SBRMI_REG_ADDR_SIZE_DEF;
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int sbrmi_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *rmi_id)
#else
static int sbrmi_i2c_probe(struct i2c_client *client)
#endif
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct apml_sbrmi_device *rmi_dev;

	rmi_dev = devm_kzalloc(dev, sizeof(struct apml_sbrmi_device), GFP_KERNEL);
	if (!rmi_dev)
		return -ENOMEM;

	atomic_set(&rmi_dev->in_progress, 0);
	atomic_set(&rmi_dev->no_new_trans, 0);
	rmi_dev->client = client;
	mutex_init(&rmi_dev->lock);

	dev_set_drvdata(dev, (void *)rmi_dev);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 rmi_dev,
							 &sbrmi_chip_info,
							 NULL);

	if (!hwmon_dev)
		return PTR_ERR_OR_ZERO(hwmon_dev);

	rmi_dev->dev_static_addr = client->addr;

	init_completion(&rmi_dev->misc_fops_done);
	return create_misc_rmi_device(rmi_dev, dev);
}

static int sbrmi_i3c_reg_read(struct i3c_device *i3cdev, int reg_size, u32 *val)
{
	struct i3c_priv_xfer xfers[I3C_I2C_MSG_XFER_SIZE];
	struct i3c_sbrmi_data *i3c_data;
	int reg = SBRMI_REV;
	int val_size = SBRMI_REG_DATA_SIZE;
	int rc = -1;

	i3c_data = kmem_cache_alloc(i3c_sbrmi_cache, GFP_KERNEL);
	if (!i3c_data) {
		printk(KERN_ERR "sbrmi_i3c_reg_read():kmem_cache_alloc() failed.\n");
		return -ENOMEM;
	}
	i3c_data->val = 0;
	i3c_data->reg = 0;

	xfers[0].rnw = false;
	xfers[0].len = reg_size;
	i3c_data->reg = reg;
	xfers[0].data.out = &i3c_data->reg;

	xfers[1].rnw = true;
	xfers[1].len = val_size;
	xfers[1].data.in = &i3c_data->val;

	rc = i3c_device_do_priv_xfers(i3cdev, xfers, I3C_I2C_MSG_XFER_SIZE);
	if (rc < 0) {
		/* free */
		kmem_cache_free(i3c_sbrmi_cache, i3c_data);
		return rc;
	}
	*val = i3c_data->val;

	/* free */
	kmem_cache_free(i3c_sbrmi_cache, i3c_data);
	return rc;
}

static int sbrmi_i3c_identify_reg_addr_size(struct i3c_device *i3cdev, u32 *size, u32 *rev)
{
	u32 reg_size;
	int ret, i;

	reg_size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
	for (i = 0; i < MAX_RETRY; i++) {
		ret = sbrmi_i3c_reg_read(i3cdev, reg_size, rev);
		if (ret < 0) {
			usleep_range(10000, 20000);
			continue;
		} else {
			break;
		}
	}
	if (ret < 0) {
		reg_size = SBRMI_REG_ADDR_SIZE_DEF;
		ret = sbrmi_i3c_reg_read(i3cdev, reg_size, rev);
		if (ret < 0) {
			pr_err("I3C reg read failed with return value:%d\n", ret);
			return ret;
		}
	}

	pr_err("sbrmi_i3c_identify_reg_addr_size(): *rev = 0x%x, *size = 0x%x\n",
		*rev, *size);
	if ((*rev & 0xff) == 0x21)
		*size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
	else
		*size = SBRMI_REG_ADDR_SIZE_DEF;
	return ret;
}

static int init_rmi_regmap(struct apml_sbrmi_device *rmi_dev, u32 size, u32 rev)
{
	struct regmap_config sbrmi_regmap_config = {
		.reg_bits = 8 * size,
		.val_bits = 8,
		.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	};
	struct regmap *regmap;

	if (rmi_dev->i3cdev) {
		regmap = devm_regmap_init_i3c(rmi_dev->i3cdev,
					      &sbrmi_regmap_config);
		if (IS_ERR(regmap)) {
			dev_err(&rmi_dev->i3cdev->dev,
				"Failed to register i3c regmap %d\n",
				(int)PTR_ERR(regmap));
			return PTR_ERR(regmap);
		}
	} else if (rmi_dev->client) {
		regmap = devm_regmap_init_i2c(rmi_dev->client,
					      &sbrmi_regmap_config);
		if (IS_ERR(regmap))
			return PTR_ERR(regmap);
	} else {
		return -ENODEV;
	}

	rmi_dev->regmap = regmap;
	rmi_dev->rev = rev;
	return 0;
}

/*
 * configure_regmap call should happen in probe, however if the server is power off,
 * regmap configuration may fail and hence driver probe will fail.
 * configure the regmap in hwmon/ioctl
 */
static int configure_regmap(struct apml_sbrmi_device *rmi_dev)
{
	u32 size;
	u32 rev = 0;
	int ret = 0;

	if (rmi_dev->i3cdev) {
		ret = sbrmi_i3c_identify_reg_addr_size(rmi_dev->i3cdev, &size, &rev);
		if (ret < 0) {
			pr_err("Reg size identification failed with return value:%d\n", ret);
			return ret;
		}
	} else if (rmi_dev->client) {
		ret = sbrmi_i2c_identify_reg_addr_size(rmi_dev->client, &size, &rev);
		if (ret < 0) {
			pr_err("Reg size identification failed with return value:%d\n", ret);
			return ret;
		}
	} else {
		return ret;
	}
	ret = init_rmi_regmap(rmi_dev, size, rev);
	return ret;
}

static int sbrmi_i3c_probe(struct i3c_device *i3cdev)
{
	struct device *dev = &i3cdev->dev;
	struct device *hwmon_dev;
	struct apml_sbrmi_device *rmi_dev;
	int ret;

	dev_info(dev, "SBRMI: PID: %llx\n", i3cdev->desc->info.pid);

	if (!(I3C_PID_INSTANCE_ID(i3cdev->desc->info.pid) == 1 ||
	      i3cdev->desc->info.pid == 0x22400000002)) {
		dev_info(dev, "SBRMI: PID Error: %llx\n", i3cdev->desc->info.pid);
		return -ENXIO;
	}

	rmi_dev = devm_kzalloc(dev, sizeof(struct apml_sbrmi_device), GFP_KERNEL);
	if (!rmi_dev)
	{
		dev_info(dev, "SBRMI: Error Mem Alloc\n");
		return -ENOMEM;
	}

	atomic_set(&rmi_dev->in_progress, 0);
	atomic_set(&rmi_dev->no_new_trans, 0);
	rmi_dev->i3cdev = i3cdev;
	mutex_init(&rmi_dev->lock);

	dev_set_drvdata(dev, (void *)rmi_dev);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "sbrmi_i3c", rmi_dev,
							 &sbrmi_chip_info, NULL);

	if (!hwmon_dev)
	{
		dev_info(dev, "SBRMI: Error HWMON Device Register\n");
		return PTR_ERR_OR_ZERO(hwmon_dev);
	}

	/* Need to verify for the static address for i3cdev */
	rmi_dev->dev_static_addr = i3cdev->desc->info.static_addr;

	i3c_sbrmi_cache = kmem_cache_create_usercopy("i3c-data-cache",
					sizeof(struct i3c_sbrmi_data), 0, 0, 0,
					sizeof(struct i3c_sbrmi_data), NULL);
	if (IS_ERR(i3c_sbrmi_cache)) {
		ret = PTR_ERR(i3c_sbrmi_cache);
		return -1;
	}

	init_completion(&rmi_dev->misc_fops_done);
	return create_misc_rmi_device(rmi_dev, dev);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int sbrmi_i2c_remove(struct i2c_client *client)
#else
static void sbrmi_i2c_remove(struct i2c_client *client)
#endif
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(&client->dev);

	if (!rmi_dev)
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
		return 0;
#else
		return;
#endif

	/*
	 * Set the no_new_trans so no new transaction can
	 * occur in sbrmi_ioctl
	 */
	atomic_set(&rmi_dev->no_new_trans, 1);
	/*
	 * If any transaction is in progress wait for the
	 * transaction to get complete
	 * Max wait for 3 sec for any pending transaction to
	 * complete
	 */
	if (atomic_read(&rmi_dev->in_progress))
		wait_for_completion_timeout(&rmi_dev->misc_fops_done,
					    MAX_WAIT_TIME_SEC * HZ);
	misc_deregister(&rmi_dev->sbrmi_misc_dev);
	/* Assign fops and parent of misc dev to NULL */
	rmi_dev->sbrmi_misc_dev.fops = NULL;
	rmi_dev->sbrmi_misc_dev.parent = NULL;

	dev_info(&client->dev, "Removed sbrmi driver\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
static int sbrmi_i3c_remove(struct i3c_device *i3cdev)
#else
static void sbrmi_i3c_remove(struct i3c_device *i3cdev)
#endif
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(&i3cdev->dev);

	if (!rmi_dev)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
		return 0;
#else
		return;
#endif

	/*
	 * Set the no_new_trans so no new transaction can
	 * occur in sbrmi_ioctl
	 */
	atomic_set(&rmi_dev->no_new_trans, 1);
	/*
	 * If any transaction is in progress wait for the
	 * transaction to get complete
	 * Max wait for 3 sec for any pending transaction to
	 * complete
	 */
	if (atomic_read(&rmi_dev->in_progress))
		wait_for_completion_timeout(&rmi_dev->misc_fops_done,
					    MAX_WAIT_TIME_SEC * HZ);
	misc_deregister(&rmi_dev->sbrmi_misc_dev);
	/* Assign fops and parent of misc dev to NULL */
	rmi_dev->sbrmi_misc_dev.fops = NULL;
	rmi_dev->sbrmi_misc_dev.parent = NULL;

	kmem_cache_destroy(i3c_sbrmi_cache);

	dev_info(&i3cdev->dev, "Removed sbrmi_i3c driver\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
	return 0;
#endif
}

static const struct i2c_device_id sbrmi_id[] = {
	{"sbrmi", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sbrmi_id);

static const struct of_device_id __maybe_unused sbrmi_of_match[] = {
	{
		.compatible = "amd,sbrmi",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sbrmi_of_match);

static const struct i3c_device_id sbrmi_i3c_id[] = {
	I3C_DEVICE_EXTRA_INFO(0, 0x000, 0x118, NULL), /* P0 - IOD0 - SBRMI */
	I3C_DEVICE_EXTRA_INFO(0, 0x100, 0x118, NULL), /* P1 - IOD0 - SBRMI */
	I3C_DEVICE_EXTRA_INFO(0x112, 0x0, 0x2, NULL),
	I3C_DEVICE_EXTRA_INFO(0x112, 0x0, 0x118, NULL), /* Socket:0, IOD:0 */
	I3C_DEVICE_EXTRA_INFO(0x112, 0x100, 0x118, NULL), /* Socket:1 IOD:0 */
	I3C_DEVICE_EXTRA_INFO(0, 0x0, 0x0, NULL),
	{}
};
MODULE_DEVICE_TABLE(i3c, sbrmi_i3c_id);

static struct i2c_driver sbrmi_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "sbrmi",
		.of_match_table = of_match_ptr(sbrmi_of_match),
	},
	.probe = sbrmi_i2c_probe,
	.remove = sbrmi_i2c_remove,
	.id_table = sbrmi_id,
};

static struct i3c_driver sbrmi_i3c_driver = {
	.driver = {
		.name = "sbrmi_i3c",
	},
	.probe = sbrmi_i3c_probe,
	.remove = sbrmi_i3c_remove,
	.id_table = sbrmi_i3c_id,
};

module_i3c_i2c_driver(sbrmi_i3c_driver, &sbrmi_driver)

int sbrmi_match_i3c(struct device *dev, const void *data)
{
	const struct device_node *node = (const struct device_node *)data;

	if (dev->of_node == node && dev->driver == &sbrmi_i3c_driver.driver)
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(sbrmi_match_i3c);

int sbrmi_match_i2c(struct device *dev, const void *data)
{
	const struct device_node *node = (const struct device_node *)data;

	if (dev->of_node == node && dev->driver == &sbrmi_driver.driver)
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(sbrmi_match_i2c);

MODULE_AUTHOR("Akshay Gupta <akshay.gupta@amd.com>");
MODULE_AUTHOR("Naveenkrishna Chatradhi <naveenkrishna.chatradhi@amd.com>");
MODULE_DESCRIPTION("Hwmon driver for AMD SB-RMI emulated sensor");
MODULE_LICENSE("GPL");
