/*
    nct7363.c - Linux kernel driver for hardware monitoring
    Copyright (C) 2008 Nuvoton Technology Corp.
                       Wei Song
                 2016 Nuvoton Technology Corp.
                       Sheng-Yuan Huang
                 2020 Nuvoton Technology Corp.
                       Kuan-Wei Ho

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation - version 2.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.


    Supports following chips:
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-vid.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>

/* Addresses to scan */
static unsigned short normal_i2c[] = { 0x20, 0x21, 0x22, 0x23, I2C_CLIENT_END };

/* Insmod parameters */
static unsigned short force_subclients[4];
module_param_array(force_subclients, short, NULL, 0);
MODULE_PARM_DESC(force_subclients, "List of subclient addresses: "
		       "{bus, clientaddr, subclientaddr1, subclientaddr2}");

static bool reset;
module_param(reset, bool, 0);
MODULE_PARM_DESC(reset, "Set to 1 to reset chip, not recommended");


#define DEBUG 1
#define DRVNAME "nct736x"
#define NCT7362_REG_DID   0xFD
#define NCT7362_ID_MASK	0xFFFFFF
#define NCT7362_ID		(0x491988 & NCT7362_ID_MASK)	/* Chip ID */

#define NCT7363_REG_OUTPUT_PORT 0x1
#define NCT7363_REG_OUTIN_CONFIG 0x3

#define NCT7362_REG_I2C_ADDR    0x46
#define NCT7362_REG_GLOBAL_CONTROL  0x00

#define NCT7362_REG_FAN(index)    (0x48 + (index)*2 )
#define NCT7362_REG_FAN_LSB(index)    (0x49 + (index)*2 )
#define NCT7362_FAN_LSB_MASK	0x1F
#define NCT7362_REG_PWM_CTRL1 0x38
#define NCT7362_REG_PWM_CTRL2 0x39
#define NCT7362_REG_FANIN_CTRL1 0x41
#define NCT7362_REG_FANIN_CTRL2 0x42
#define NCT7362_REG_WDT_CONFIG 0x2A
#define NCT7362_REG_GPIO_00_03_CONFIG 0x20
#define NCT7362_REG_GPIO_04_07_CONFIG 0x21
#define NCT7362_REG_GPIO_10_13_CONFIG 0x22
#define NCT7362_REG_GPIO_14_17_CONFIG 0x23

#define NCT7362_REG_PWM(index)    (0x90 + (index)*2 )
#define NCT7362_REG_PWM_DEFAULT_VALUE  0x10

static inline unsigned long FAN_FROM_REG(u16 val)
{
	if ((val >= 0x1fff) || (val == 0))
		return	0;
	return (1350000UL / val);
}

static inline u16 FAN_TO_REG(long rpm)
{
	if (rpm <= 0)
		return 0x1fff;
	return clamp_val((1350000 + (rpm >> 1)) / rpm, 1, 0x1fff);
}

static inline unsigned long TIME_FROM_REG(u8 reg)
{
	return (reg * 100);
}

static inline u8 TIME_TO_REG(unsigned long val)
{
	return clamp_val((val + 50) / 100, 0, 0xff);
}

enum chip_types {nct7362d,nct7363};

struct nct7362_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	unsigned long last_updated;	/* In jiffies */
	enum chip_types chip_type; /* For recording what the chip is */ 
	const struct attribute_group *groups[3];

	struct i2c_client *client;

	u16 has_fan;	/* Enable fan 0-16 */
	u16 fan[16];		/* Register value combine */

	u16 has_pwm;	/* Enable pwm 0-16 */
	u16 pwm[16];		/* Register value combine */

	char valid;
	int bmc_set_pwm;	//enable bmc to set fan speed
	u32 fan_sel_gpio;   //get fan gpio from board dts
};

static u8 nct7362_read_value(struct i2c_client *client, u16 reg);
static int nct7362_write_value(struct i2c_client *client, u16 reg, u8 value);
static int nct7362_probe(struct i2c_client *client);
static int nct7362_detect(struct i2c_client *client,
			 struct i2c_board_info *info);
//static int nct7362_remove(struct i2c_client *client);

static void nct7362_init_client(struct i2c_client *client,u32 gpio);
static struct nct7362_data *nct7362_update_device(struct device *dev);


#define ALARM_STATUS      0

static ssize_t
show_fan(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int index = sensor_attr->index;
	struct nct7362_data *data = nct7362_update_device(dev);
	u16 val;

	val = data->fan[index] & 0x1fff;

	return sprintf(buf, "%lu\n", FAN_FROM_REG(val));
}

static ssize_t
show_pwm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int index = sensor_attr->index;
	struct nct7362_data *data = nct7362_update_device(dev);
	u16 val;

	val = data->pwm[index] & 0xff;

	return sprintf(buf, "%u\n", ((unsigned int)val));
}

static ssize_t
store_pwm(struct device *dev, struct device_attribute *attr,
	 const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int index = sensor_attr->index;
	struct nct7362_data *data = nct7362_update_device(dev);
	struct i2c_client *client = data->client;
	unsigned long tmpVal;
	int err;

	err = kstrtoul(buf, 10, &tmpVal);
	if (err){
		return 0;
	}	
	mutex_lock(&data->update_lock);
	data->pwm[index] = tmpVal;
	nct7362_write_value(client, NCT7362_REG_PWM(index), tmpVal & 0xFF);

	//add for setting GPIOx H->L when set pwm in the first time
	if(data->bmc_set_pwm == 0)
	{
		u8 reg_value=0;
		u8 gpio = (u8)data->fan_sel_gpio;

		if(DEBUG) dev_err(dev,"store_pwm: set GPIO%d H->L when setting pwm for the first time (%d)\n",
				gpio,data->bmc_set_pwm);

		reg_value = nct7362_read_value(client, NCT7363_REG_OUTIN_CONFIG);
		if(DEBUG) dev_err(dev,"store_pwm: Read NCT7363_REG_OUTIN_CONFIG==0x%x\n",reg_value);

		reg_value &= ~(1<<gpio);
		nct7362_write_value(client, NCT7363_REG_OUTIN_CONFIG, reg_value);
		if(DEBUG) dev_err(dev,"store_pwm: Write NCT7363_REG_OUTIN_CONFIG==0x%x\n",reg_value);

		reg_value = nct7362_read_value(client, NCT7363_REG_OUTPUT_PORT);
		if(DEBUG) dev_err(dev,"store_pwm: Read NCT7363_REG_OUTPUT_PORT==0x%x\n",reg_value);

		reg_value &= ~(1<<gpio);
		nct7362_write_value(client, NCT7363_REG_OUTPUT_PORT, reg_value);
		if(DEBUG)dev_err(dev,"store_pwm: Write NCT7363_REG_OUTPUT_PORT==0x%x\n",reg_value);

		data->bmc_set_pwm=1;
	}

	mutex_unlock(&data->update_lock);

	return count;
}

static umode_t nct7362_fan_is_visible(struct kobject *kobj,
				     struct attribute *attr, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct nct7362_data *data = dev_get_drvdata(dev);
	int fan = index;	/* fan index */

	if (!(data->has_fan & (1 << fan)))
		return 0;

	return attr->mode;
}

static umode_t nct7362_pwm_is_visible(struct kobject *kobj,
				     struct attribute *attr, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct nct7362_data *data = dev_get_drvdata(dev);
	int pwm = index;	/* pwm index */

	if (!(data->has_pwm & (1 << pwm)))
		return 0;
	return attr->mode;
}

#define NOT_USED			-1

#define FAN_INPUT     0
#define SENSOR_DEV_ATTR_FAN(index)						\
		static SENSOR_DEVICE_ATTR_2(fan##index##_input, S_IRUGO, show_fan,		\
			NULL, FAN_INPUT, index); \

#define DEV_ATTR_ATTR_LIST_FAN(index)	\
	&sensor_dev_attr_fan##index##_input.dev_attr.attr

SENSOR_DEV_ATTR_FAN(0);
SENSOR_DEV_ATTR_FAN(1);
SENSOR_DEV_ATTR_FAN(2);
SENSOR_DEV_ATTR_FAN(3);
SENSOR_DEV_ATTR_FAN(4);
SENSOR_DEV_ATTR_FAN(5);
SENSOR_DEV_ATTR_FAN(6);
SENSOR_DEV_ATTR_FAN(7);
SENSOR_DEV_ATTR_FAN(8);
SENSOR_DEV_ATTR_FAN(9);
SENSOR_DEV_ATTR_FAN(10);
SENSOR_DEV_ATTR_FAN(11);
SENSOR_DEV_ATTR_FAN(12);
SENSOR_DEV_ATTR_FAN(13);
SENSOR_DEV_ATTR_FAN(14);
SENSOR_DEV_ATTR_FAN(15);

static struct attribute *nct7362_attributes_fan[] = {
	DEV_ATTR_ATTR_LIST_FAN(0),
	DEV_ATTR_ATTR_LIST_FAN(1),
	DEV_ATTR_ATTR_LIST_FAN(2),
	DEV_ATTR_ATTR_LIST_FAN(3),
	DEV_ATTR_ATTR_LIST_FAN(4),
	DEV_ATTR_ATTR_LIST_FAN(5),
	DEV_ATTR_ATTR_LIST_FAN(6),
	DEV_ATTR_ATTR_LIST_FAN(7),
	DEV_ATTR_ATTR_LIST_FAN(8),
	DEV_ATTR_ATTR_LIST_FAN(9),
	DEV_ATTR_ATTR_LIST_FAN(10),
	DEV_ATTR_ATTR_LIST_FAN(11),
	DEV_ATTR_ATTR_LIST_FAN(12),
	DEV_ATTR_ATTR_LIST_FAN(13),
	DEV_ATTR_ATTR_LIST_FAN(14),
	DEV_ATTR_ATTR_LIST_FAN(15),
	NULL
};

static const struct attribute_group nct7362_group_fan = {
	.attrs = nct7362_attributes_fan,
	.is_visible = nct7362_fan_is_visible,
};

#define PWM_OUTPUT     0
#define SENSOR_DEV_ATTR_PWM(index) \
	static SENSOR_DEVICE_ATTR_2(pwm##index, S_IRUGO | S_IWUSR, show_pwm, \
			store_pwm, PWM_OUTPUT, index);

#define DEV_ATTR_ATTR_LIST_PWM(index)	\
	&sensor_dev_attr_pwm##index.dev_attr.attr

SENSOR_DEV_ATTR_PWM(0);
SENSOR_DEV_ATTR_PWM(1);
SENSOR_DEV_ATTR_PWM(2);
SENSOR_DEV_ATTR_PWM(3);
SENSOR_DEV_ATTR_PWM(4);
SENSOR_DEV_ATTR_PWM(5);
SENSOR_DEV_ATTR_PWM(6);
SENSOR_DEV_ATTR_PWM(7);
SENSOR_DEV_ATTR_PWM(8);
SENSOR_DEV_ATTR_PWM(9);
SENSOR_DEV_ATTR_PWM(10);
SENSOR_DEV_ATTR_PWM(11);
SENSOR_DEV_ATTR_PWM(12);
SENSOR_DEV_ATTR_PWM(13);
SENSOR_DEV_ATTR_PWM(14);
SENSOR_DEV_ATTR_PWM(15);

static struct attribute *nct7362_attributes_pwm[] = {
	DEV_ATTR_ATTR_LIST_PWM(0),
	DEV_ATTR_ATTR_LIST_PWM(1),
	DEV_ATTR_ATTR_LIST_PWM(2),
	DEV_ATTR_ATTR_LIST_PWM(3),
	DEV_ATTR_ATTR_LIST_PWM(4),
	DEV_ATTR_ATTR_LIST_PWM(5),
	DEV_ATTR_ATTR_LIST_PWM(6),
	DEV_ATTR_ATTR_LIST_PWM(7),
	DEV_ATTR_ATTR_LIST_PWM(8),
	DEV_ATTR_ATTR_LIST_PWM(9),
	DEV_ATTR_ATTR_LIST_PWM(10),
	DEV_ATTR_ATTR_LIST_PWM(11),
	DEV_ATTR_ATTR_LIST_PWM(12),
	DEV_ATTR_ATTR_LIST_PWM(13),
	DEV_ATTR_ATTR_LIST_PWM(14),
	DEV_ATTR_ATTR_LIST_PWM(15),
	NULL
};

static const struct attribute_group nct7362_group_pwm = {
	.attrs = nct7362_attributes_pwm,
	.is_visible = nct7362_pwm_is_visible,
};

static void nct7362_init_client(struct i2c_client *client,u32 gpio)
{
	//Nigeria
	if(gpio == 5) {
		// init /- pwm0 fanin9 10 11 -/- fanin 12 GPIO5 6 7 -/- GPIO 10 11 12 13 -/- GPIO 14 15 16 17 -/
		nct7362_write_value(client, NCT7362_REG_WDT_CONFIG, 0x00);
		nct7362_write_value(client, NCT7362_REG_PWM_CTRL1, 0x01);
		nct7362_write_value(client, NCT7362_REG_PWM_CTRL2, 0x0);
		nct7362_write_value(client, NCT7362_REG_FANIN_CTRL1, 0x0);
		nct7362_write_value(client, NCT7362_REG_FANIN_CTRL2, 0x1E);
		nct7362_write_value(client, NCT7362_REG_GPIO_00_03_CONFIG, 0xA9);
		nct7362_write_value(client, NCT7362_REG_GPIO_04_07_CONFIG, 0x2);
		nct7362_write_value(client, NCT7362_REG_GPIO_10_13_CONFIG, 0x0);
		nct7362_write_value(client, NCT7362_REG_GPIO_14_17_CONFIG, 0x0);
	}
	else if(gpio == 3)  //Kenya
	{
		// init /- pwm0, pwm8, pwm15, fanin9 10 11 -/- fanin 12 GPIO5 6 7 -/- GPIO 10 11 12 13 -/- GPIO 14 15 16 17 -/
		nct7362_write_value(client, NCT7362_REG_WDT_CONFIG, 0x00);
		nct7362_write_value(client, NCT7362_REG_PWM_CTRL1, 0x01);
		nct7362_write_value(client, NCT7362_REG_PWM_CTRL2, 0x81);
		nct7362_write_value(client, NCT7362_REG_FANIN_CTRL1, 0x7E);
		nct7362_write_value(client, NCT7362_REG_FANIN_CTRL2, 0xF6);
		nct7362_write_value(client, NCT7362_REG_GPIO_00_03_CONFIG, 0x29);
		nct7362_write_value(client, NCT7362_REG_GPIO_04_07_CONFIG, 0xAA);
		nct7362_write_value(client, NCT7362_REG_GPIO_10_13_CONFIG, 0xA9);
		nct7362_write_value(client, NCT7362_REG_GPIO_14_17_CONFIG, 0x6A);
	}
}

static int __init nct7362d_find(int addr, struct i2c_client *client, struct i2c_board_info *info)
{
	int err;
	u32 devid = 0;

	err = -ENODEV;

    devid = (nct7362_read_value(client, addr) & 0xFF);
    devid = (devid<<8) | (nct7362_read_value(client, addr+1) & 0xFF);
    devid = (devid<<8) | (nct7362_read_value(client, addr+2) & 0xFF);
    devid = devid & NCT7362_ID_MASK;

	switch (devid) {
	case NCT7362_ID:
        /* Fill in the remaining client fields and put into the global list */
    	strlcpy(info->type, "nct7362", I2C_NAME_SIZE);
        dev_info(&client->dev, "Detected Nuvoton %s chip at 0x%02x\n", "nct7362", client->addr);
		break;
	default:
		dev_err(&client->dev, "nct7363: Unsupported device 0x%08x\n", devid);
		goto err;
	}
	err = 0;

err:
	return err;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int nct7362_detect(struct i2c_client *client,
			 struct i2c_board_info *info)
{
	int ret;

	dev_info(&client->dev, "Diver for nct7362_detect: nct7362_detect...\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		return -ENODEV;
	}
    
	ret = nct7362d_find(NCT7362_REG_DID, client, info);

	if ( ret != 0){
		dev_info(&client->dev,"Driver use nct7363 ...\n");
		strlcpy(info->type, "nct7363", I2C_NAME_SIZE);
	}

	return 0;
}

#define TMP_MASK 0x3

static int nct736x_init(struct i2c_client *client,u32 gpio)
{
	nct7362_init_client(client, gpio);
	// Set Default PWM0 to 50%
	nct7362_write_value(client, NCT7362_REG_PWM(0), NCT7362_REG_PWM_DEFAULT_VALUE);
	if (gpio==3)
	{
		// Set Default PWM8, PWM15 to 50%
		nct7362_write_value(client, NCT7362_REG_PWM(8), NCT7362_REG_PWM_DEFAULT_VALUE);
		nct7362_write_value(client, NCT7362_REG_PWM(15), NCT7362_REG_PWM_DEFAULT_VALUE);
	}
	return 0;
}

static int nct7362_probe(struct i2c_client *client)
{
	int i,ret;
	struct device *dev = &client->dev;
	struct nct7362_data *data;
	struct device *hwmon_dev;

	dev_err(dev,"nct7362_probe Start\n");

	if (!(data = devm_kzalloc(dev,sizeof(struct nct7362_data), GFP_KERNEL))) {
		dev_err(dev,"nct7362_probe: Error allocating memory\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);

	data->client = client; 
	data->bmc_set_pwm = 0;

	//add board_id compatibale
	struct device_node *np = client->dev.of_node;

	if(of_property_read_u32(np,"fan_sel_gpio",&data->fan_sel_gpio))
	{
		dev_err(&client->dev,"nct7362_probe: Error: no fan_sel_gpio in DTS, default to Nigeria \n");
	}

	if(DEBUG) dev_err(dev,"nct7362_probe: fan_sel_gpio =%d\n", data->fan_sel_gpio);

	/* Initialize the chip */
	ret = nct736x_init(client, data->fan_sel_gpio);
	if(ret != 0)
	{
		dev_err(&client->dev, "nct7362_probe: init error\n");
	}

	/* Check chip type*/
	data->chip_type = nct7363;
	
	data->has_fan = nct7362_read_value(client, NCT7362_REG_FANIN_CTRL1);
	data->has_fan |= nct7362_read_value(client, NCT7362_REG_FANIN_CTRL2) << 8;
	data->has_pwm = nct7362_read_value(client, NCT7362_REG_PWM_CTRL1);
	data->has_pwm |= nct7362_read_value(client, NCT7362_REG_PWM_CTRL2) << 8;

	if(DEBUG) dev_err(&client->dev, "nct7362_probe: data->has_fan =0x%x,data->has_pwm=0x%x\n",
			data->has_fan,data->has_pwm);

	/* Multi-Function detecting for Volt and TR/TD.
	   Just deal with the DISABLE in has_xxxx because
	   if it is not monitored, multi-function selection is useless.*/

	/* First update fan */
	for (i = 0; i < ARRAY_SIZE(data->fan); i++) {
		if (!(data->has_fan & (1 << i)))
			continue;
		data->fan[i] =
			((u16)nct7362_read_value(client, NCT7362_REG_FAN(i))) << 5;
		data->fan[i] |=
		  nct7362_read_value(client, NCT7362_REG_FAN_LSB(i)) & NCT7362_FAN_LSB_MASK;
	}
	/* First update pwm */
	for (i = 0; i < ARRAY_SIZE(data->pwm); i++) {
		if (!(data->has_pwm & (1 << i)))
			continue;
		data->pwm[i] =
			((u16)nct7362_read_value(client, NCT7362_REG_PWM(i)));
	}

	/* Prepare for sysfs hooks */
	data->groups[0] = &nct7362_group_fan;
	data->groups[1] = &nct7362_group_pwm;
	data->groups[2] = NULL;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
					client->name,
					data, data->groups);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static struct nct7362_data *nct7362_update_device(struct device *dev)
{
	struct nct7362_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int i;

	mutex_lock(&data->update_lock);

	if(DEBUG) dev_err(dev,"nct7362_probe: fan_sel_gpio =%d\n", data->fan_sel_gpio);

	nct7362_init_client(client, data->fan_sel_gpio);
	if (!(time_after(jiffies, data->last_updated + HZ * 2)
	      || !data->valid))
		goto END;

	/* Update fan */
	for (i = 0; i < ARRAY_SIZE(data->fan); i++) {
		if (!(data->has_fan & (1 << i))) {
			continue;
		}
		data->fan[i] =
			((u16)nct7362_read_value(client, NCT7362_REG_FAN(i))) << 5;
		data->fan[i] |=
		  nct7362_read_value(client, NCT7362_REG_FAN_LSB(i)) & NCT7362_FAN_LSB_MASK;
	}
	/* Update pwm */
	for (i = 0; i < ARRAY_SIZE(data->pwm); i++) {
		if (!(data->has_pwm & (1 << i))) {
			continue;
		}
		data->pwm[i] =
			((u16)nct7362_read_value(client, NCT7362_REG_PWM(i)));
	}
	data->last_updated = jiffies;
	data->valid = 1;

END:
	mutex_unlock(&data->update_lock);
	return data;
}

/* Ignore the possibility that somebody change bank outside the driver
   Must be called with data->update_lock held, except during initialization */
static u8 nct7362_read_value(struct i2c_client *client, u16 reg)
{
	u8 res = 0xff;

	res = i2c_smbus_read_byte_data(client, reg & 0xff);
	return res;
}

/* Must be called with data->update_lock held, except during initialization */
static int nct7362_write_value(struct i2c_client *client, u16 reg, u8 value)
{
	int res;

	res = i2c_smbus_write_byte_data(client, reg & 0xff, value);
	return res;
}

static const struct i2c_device_id nct7362_id[] = {
	{ "nct7362", nct7362d },
	{ "nct7363", nct7363  },
	{},
};

//MODULE_DEVICE_TABLE(i2c, nct7362_id);

static const struct of_device_id nct7362_of_match[] = {
	{ .compatible = "nct,nct7362",.data = (void *)nct7362d, },
	{ .compatible = "nct,nct7363",.data = (void *)nct7363, },
	{}
};
MODULE_DEVICE_TABLE(of, nct7362_of_match);

static struct i2c_driver nct7362_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = DRVNAME,
		   .of_match_table = of_match_ptr(nct7362_of_match),
	},
	.class		= I2C_CLASS_HWMON,
	.probe		= nct7362_probe,
	//.remove		= nct7362_remove,
	.id_table	= nct7362_id,
	.detect		= nct7362_detect,
	.address_list	= normal_i2c,
};

#ifdef DBG_INIT_FUNCTION

static int __init sensors_nct7362_init(void)
{
	return i2c_add_driver(&nct7362_driver);
}

static void __exit sensors_nct7362_exit(void)
{
	i2c_del_driver(&nct7362_driver);
}

module_init(sensors_nct7362_init);
module_exit(sensors_nct7362_exit);

#else

module_i2c_driver(nct7362_driver);

#endif

MODULE_AUTHOR("Sheng-Yuan Huang");
MODULE_DESCRIPTION("NCT7362 driver");
MODULE_LICENSE("GPL");
