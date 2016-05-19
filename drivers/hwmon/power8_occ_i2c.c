/*
 * OCC HWMON driver - read IBM Power8 On Chip Controller sensor data via
 * i2c.
 *
 * Copyright 2015 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/device.h>

#define OCC_I2C_ADDR 0x50
#define OCC_I2C_NAME "occ-i2c"

#define OCC_DATA_MAX	4096 /* 4KB at most */
/* i2c read and write occ sensors */
#define I2C_READ_ERROR	1
#define I2C_WRITE_ERROR	2

/* Defined in POWER8 Processor Registers Specification */
/* To generate attn to OCC */
#define ATTN_DATA	0x0006B035
/* For BMC to read/write SRAM */
#define OCB_ADDRESS		0x0006B070
#define OCB_DATA		0x0006B075
#define OCB_STATUS_CONTROL_AND	0x0006B072
#define OCB_STATUS_CONTROL_OR	0x0006B073
/* See definition in:
 * https://github.com/open-power/docs/blob/master/occ/OCC_OpenPwr_FW_Interfaces.pdf
 */
#define OCC_COMMAND_ADDR	0xFFFF6000
#define OCC_RESPONSE_ADDR	0xFFFF7000

#define MAX_SENSOR_ATTR_LEN	32

enum sensor_t {
	freq,
	temp,
	power,
	caps,
	MAX_OCC_SENSOR_TYPE
};

/* OCC sensor data format */
struct occ_sensor {
	uint16_t sensor_id;
	uint16_t value;
};

struct power_sensor {
	uint16_t sensor_id;
	uint32_t update_tag;
	uint32_t accumulator;
	uint16_t value;
};

struct caps_sensor {
	uint16_t curr_powercap;
	uint16_t curr_powerreading;
	uint16_t norm_powercap;
	uint16_t max_powercap;
	uint16_t min_powercap;
	uint16_t user_powerlimit;
};

struct sensor_data_block {
	uint8_t sensor_type[4];
	uint8_t reserved0;
	uint8_t sensor_format;
	uint8_t sensor_length;
	uint8_t sensor_num;
	struct occ_sensor *sensor;
	struct power_sensor *power;
	struct caps_sensor *caps;
};

struct occ_poll_header {
	uint8_t status;
	uint8_t ext_status;
	uint8_t occs_present;
	uint8_t config;
	uint8_t occ_state;
	uint8_t reserved0;
	uint8_t reserved1;
	uint8_t error_log_id;
	uint32_t error_log_addr_start;
	uint16_t error_log_length;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t occ_code_level[16];
	uint8_t sensor_eye_catcher[6];
	uint8_t sensor_block_num;
	uint8_t sensor_data_version;
};

struct occ_response {
	uint8_t sequence_num;
	uint8_t cmd_type;
	uint8_t rtn_status;
	uint16_t data_length;
	struct occ_poll_header header;
	struct sensor_data_block *blocks;
	uint16_t chk_sum;
	int sensor_block_id[MAX_OCC_SENSOR_TYPE];
};

struct sensor_attr_data {
	enum sensor_t type;
	uint32_t hwmon_index;
	uint32_t attr_id;
	char name[MAX_SENSOR_ATTR_LEN];
	struct device_attribute dev_attr;
};

struct sensor_group {
	char *name;
	struct sensor_attr_data *sattr;
	struct attribute_group group;
};

/* data private to each client */
struct occ_drv_data {
	struct i2c_client	*client;
	struct device		*hwmon_dev;
	struct mutex		update_lock;
	bool			valid;
	unsigned long		last_updated;
	/* Minimum timer interval for sampling In jiffies */
	unsigned long		update_interval;
	unsigned long		occ_online;
	uint16_t		user_powercap;
	struct occ_response	occ_resp;
	struct sensor_group	sensor_groups[MAX_OCC_SENSOR_TYPE];
};

static void deinit_occ_resp_buf(struct occ_response *p)
{
	int i;

	if (!p)
		return;

	if (!p->blocks)
		return;

	for (i = 0; i < p->header.sensor_block_num; i++) {
		kfree(p->blocks[i].sensor);
		kfree(p->blocks[i].power);
		kfree(p->blocks[i].caps);
	}

	kfree(p->blocks);

	memset(p, 0, sizeof(*p));

	for (i = 0; i < ARRAY_SIZE(p->sensor_block_id); i++)
		p->sensor_block_id[i] = -1;
}

static ssize_t occ_i2c_read(struct i2c_client *client, void *buf, size_t count)
{
	WARN_ON(count > OCC_DATA_MAX);

	dev_dbg(&client->dev, "i2c_read: reading %zu bytes @0x%x.\n",
		count, client->addr);
	return i2c_master_recv(client, buf, count);
}

static ssize_t occ_i2c_write(struct i2c_client *client, const void *buf,
				size_t count)
{
	WARN_ON(count > OCC_DATA_MAX);

	dev_dbg(&client->dev, "i2c_write: writing %zu bytes @0x%x.\n",
		count, client->addr);
	return i2c_master_send(client, buf, count);
}

/* read 8-byte value and put into data[offset] */
static int occ_getscomb(struct i2c_client *client, uint32_t address,
		uint8_t *data, int offset)
{
	uint32_t ret;
	char buf[8];
	int i;

	/* P8 i2c slave requires address to be shifted by 1 */
	address = address << 1;

	ret = occ_i2c_write(client, &address,
		sizeof(address));

	if (ret != sizeof(address))
		return -I2C_WRITE_ERROR;

	ret = occ_i2c_read(client, buf, sizeof(buf));
	if (ret != sizeof(buf))
		return -I2C_READ_ERROR;

	for (i = 0; i < 8; i++)
		data[offset + i] = buf[7 - i];

	return 0;
}

static int occ_putscom(struct i2c_client *client, uint32_t address,
		uint32_t data0, uint32_t data1)
{
	uint32_t buf[3];
	uint32_t ret;

	/* P8 i2c slave requires address to be shifted by 1 */
	address = address << 1;

	buf[0] = address;
	buf[1] = data1;
	buf[2] = data0;

	ret = occ_i2c_write(client, buf, sizeof(buf));
	if (ret != sizeof(buf))
		return I2C_WRITE_ERROR;

	return 0;
}

static void *occ_get_sensor_by_type(struct occ_response *resp, enum sensor_t t)
{
	void *sensor;

	if (!resp->blocks)
		return NULL;

	if (resp->sensor_block_id[t] == -1)
		return NULL;

	switch (t) {
	case temp:
	case freq:
		sensor = resp->blocks[resp->sensor_block_id[t]].sensor;
		break;
	case power:
		sensor = resp->blocks[resp->sensor_block_id[t]].power;
		break;
	case caps:
		sensor = resp->blocks[resp->sensor_block_id[t]].caps;
		break;
	default:
		sensor = NULL;
	}

	return sensor;
}

static int occ_renew_sensor(struct occ_response *resp, uint8_t sensor_length,
	uint8_t sensor_num, enum sensor_t t, int block)
{
	void *sensor;
	int ret;

	sensor = occ_get_sensor_by_type(resp, t);

	/* empty sensor block, release older sensor data */
	if (sensor_num == 0 || sensor_length == 0) {
		kfree(sensor);
		return -1;
	}

	if (!sensor || sensor_num !=
			resp->blocks[resp->sensor_block_id[t]].sensor_num) {
		kfree(sensor);
		switch (t) {
		case temp:
		case freq:
			resp->blocks[block].sensor =
				kcalloc(sensor_num,
					sizeof(struct occ_sensor), GFP_KERNEL);
			if (!resp->blocks[block].sensor) {
				ret = -ENOMEM;
				goto err;
			}
			break;
		case power:
			resp->blocks[block].power =
				kcalloc(sensor_num,
					sizeof(struct power_sensor),
					GFP_KERNEL);
			if (!resp->blocks[block].power) {
				ret = -ENOMEM;
				goto err;
			}
			break;
		case caps:
			resp->blocks[block].caps =
				kcalloc(sensor_num,
					sizeof(struct caps_sensor), GFP_KERNEL);
			if (!resp->blocks[block].caps) {
				ret = -ENOMEM;
				goto err;
			}
			break;
		default:
			ret = -ENOMEM;
			goto err;
		}
	}

	return 0;
err:
	deinit_occ_resp_buf(resp);
	return ret;
}

#define RESP_DATA_LENGTH	3
#define RESP_HEADER_OFFSET	5
#define SENSOR_STR_OFFSET	37
#define SENSOR_BLOCK_NUM_OFFSET	43
#define SENSOR_BLOCK_OFFSET	45

static inline uint16_t get_occdata_length(uint8_t *data)
{
	return be16_to_cpup((const __be16 *)&data[RESP_DATA_LENGTH]);
}

static int parse_occ_response(struct i2c_client *client,
		uint8_t *data, struct occ_response *resp)
{
	int b;
	int s;
	int ret;
	int dnum = SENSOR_BLOCK_OFFSET;
	struct occ_sensor *f_sensor;
	struct occ_sensor *t_sensor;
	struct power_sensor *p_sensor;
	struct caps_sensor *c_sensor;
	uint8_t sensor_block_num;
	uint8_t sensor_type[4];
	uint8_t sensor_format;
	uint8_t sensor_length;
	uint8_t sensor_num;

	/* check if the data is valid */
	if (strncmp(&data[SENSOR_STR_OFFSET], "SENSOR", 6) != 0) {
		dev_dbg(&client->dev,
			"ERROR: no SENSOR String in response\n");
		ret = -1;
		goto err;
	}

	sensor_block_num = data[SENSOR_BLOCK_NUM_OFFSET];
	if (sensor_block_num == 0) {
		dev_dbg(&client->dev, "ERROR: SENSOR block num is 0\n");
		ret = -1;
		goto err;
	}

	/* if sensor block has changed, re-malloc */
	if (sensor_block_num != resp->header.sensor_block_num) {
		deinit_occ_resp_buf(resp);
		resp->blocks = kcalloc(sensor_block_num,
			sizeof(struct sensor_data_block), GFP_KERNEL);
		if (!resp->blocks)
			return -ENOMEM;
	}

	memcpy(&resp->header, &data[RESP_HEADER_OFFSET], sizeof(resp->header));
	resp->header.error_log_addr_start =
		be32_to_cpu(resp->header.error_log_addr_start);
	resp->header.error_log_length =
		be16_to_cpu(resp->header.error_log_length);

	dev_dbg(&client->dev, "Reading %d sensor blocks\n",
		resp->header.sensor_block_num);
	for (b = 0; b < sensor_block_num; b++) {
		/* 8-byte sensor block head */
		strncpy(sensor_type, &data[dnum], 4);
		sensor_format = data[dnum+5];
		sensor_length = data[dnum+6];
		sensor_num = data[dnum+7];
		dnum = dnum + 8;

		dev_dbg(&client->dev,
			"sensor block[%d]: type: %s, sensor_num: %d\n",
			b, sensor_type, sensor_num);

		if (strncmp(sensor_type, "FREQ", 4) == 0) {
			ret = occ_renew_sensor(resp, sensor_length,
				sensor_num, freq, b);
			if (ret)
				continue;

			resp->sensor_block_id[freq] = b;
			for (s = 0; s < sensor_num; s++) {
				f_sensor = &resp->blocks[b].sensor[s];
				f_sensor->sensor_id =
					be16_to_cpup((const __be16 *)
							&data[dnum]);
				f_sensor->value = be16_to_cpup((const __be16 *)
							&data[dnum+2]);
				dev_dbg(&client->dev,
					"sensor[%d]-[%d]: id: %u, value: %u\n",
					b, s, f_sensor->sensor_id,
					f_sensor->value);
				dnum = dnum + sensor_length;
			}
		} else if (strncmp(sensor_type, "TEMP", 4) == 0) {
			ret = occ_renew_sensor(resp, sensor_length,
				sensor_num, temp, b);
			if (ret)
				continue;

			resp->sensor_block_id[temp] = b;
			for (s = 0; s < sensor_num; s++) {
				t_sensor = &resp->blocks[b].sensor[s];
				t_sensor->sensor_id =
					be16_to_cpup((const __be16 *)
							&data[dnum]);
				t_sensor->value = be16_to_cpup((const __be16 *)
							&data[dnum+2]);
				dev_dbg(&client->dev,
					"sensor[%d]-[%d]: id: %u, value: %u\n",
					b, s, t_sensor->sensor_id,
					t_sensor->value);
				dnum = dnum + sensor_length;
			}
		} else if (strncmp(sensor_type, "POWR", 4) == 0) {
			ret = occ_renew_sensor(resp, sensor_length,
				sensor_num, power, b);
			if (ret)
				continue;

			resp->sensor_block_id[power] = b;
			for (s = 0; s < sensor_num; s++) {
				p_sensor = &resp->blocks[b].power[s];
				p_sensor->sensor_id =
					be16_to_cpup((const __be16 *)
							&data[dnum]);
				p_sensor->update_tag =
					be32_to_cpup((const __be32 *)
							&data[dnum+2]);
				p_sensor->accumulator =
					be32_to_cpup((const __be32 *)
							&data[dnum+6]);
				p_sensor->value = be16_to_cpup((const __be16 *)
							&data[dnum+10]);

				dev_dbg(&client->dev,
					"sensor[%d]-[%d]: id: %u, value: %u\n",
					b, s, p_sensor->sensor_id,
					p_sensor->value);

				dnum = dnum + sensor_length;
			}
		} else if (strncmp(sensor_type, "CAPS", 4) == 0) {
			ret = occ_renew_sensor(resp, sensor_length,
				sensor_num, caps, b);
			if (ret)
				continue;

			resp->sensor_block_id[caps] = b;
			for (s = 0; s < sensor_num; s++) {
				c_sensor = &resp->blocks[b].caps[s];
				c_sensor->curr_powercap =
					be16_to_cpup((const __be16 *)
							&data[dnum]);
				c_sensor->curr_powerreading =
					be16_to_cpup((const __be16 *)
							&data[dnum+2]);
				c_sensor->norm_powercap =
					be16_to_cpup((const __be16 *)
							&data[dnum+4]);
				c_sensor->max_powercap =
					be16_to_cpup((const __be16 *)
							&data[dnum+6]);
				c_sensor->min_powercap =
					be16_to_cpup((const __be16 *)
							&data[dnum+8]);
				c_sensor->user_powerlimit =
					be16_to_cpup((const __be16 *)
							&data[dnum+10]);

				dnum = dnum + sensor_length;
				dev_dbg(&client->dev, "CAPS sensor #%d:\n", s);
				dev_dbg(&client->dev, "curr_powercap is %x\n",
					c_sensor->curr_powercap);
				dev_dbg(&client->dev,
					"curr_powerreading is %x\n",
					c_sensor->curr_powerreading);
				dev_dbg(&client->dev, "norm_powercap is %x\n",
					c_sensor->norm_powercap);
				dev_dbg(&client->dev, "max_powercap is %x\n",
					c_sensor->max_powercap);
				dev_dbg(&client->dev, "min_powercap is %x\n",
					c_sensor->min_powercap);
				dev_dbg(&client->dev, "user_powerlimit is %x\n",
					c_sensor->user_powerlimit);
			}

		} else {
			dev_dbg(&client->dev,
				"ERROR: sensor type %s not supported\n",
				resp->blocks[b].sensor_type);
			ret = -1;
			goto err;
		}

		strncpy(resp->blocks[b].sensor_type, sensor_type, 4);
		resp->blocks[b].sensor_format = sensor_format;
		resp->blocks[b].sensor_length = sensor_length;
		resp->blocks[b].sensor_num = sensor_num;
	}

	return 0;
err:
	deinit_occ_resp_buf(resp);
	return ret;
}


/* Refer to OCC interface document for OCC command format
 * https://github.com/open-power/docs/blob/master/occ/OCC_OpenPwr_FW_Interfaces.pdf
 */
static uint8_t occ_send_cmd(struct i2c_client *client, uint8_t seq,
		uint8_t type, uint16_t length, uint8_t *data, uint8_t *resp)
{
	uint32_t cmd1, cmd2;
	uint16_t checksum;
	int i;

	length = cpu_to_le16(length);
	cmd1 = (seq << 24) | (type << 16) | length;
	memcpy(&cmd2, data, length);
	cmd2 <<= ((4 - length) * 8);

	/* checksum: sum of every bytes of cmd1, cmd2 */
	checksum = 0;
	for (i = 0; i < 4; i++)
		checksum += (cmd1 >> (i * 8)) & 0xFF;
	for (i = 0; i < 4; i++)
		checksum += (cmd2 >> (i * 8)) & 0xFF;
	cmd2 |= checksum << ((2 - length) * 8);

	/* Init OCB */
	occ_putscom(client, OCB_STATUS_CONTROL_OR,  0x08000000, 0x00000000);
	occ_putscom(client, OCB_STATUS_CONTROL_AND, 0xFBFFFFFF, 0xFFFFFFFF);

	/* Send command */
	occ_putscom(client, OCB_ADDRESS, OCC_COMMAND_ADDR, 0x00000000);
	occ_putscom(client, OCB_ADDRESS, OCC_COMMAND_ADDR, 0x00000000);
	occ_putscom(client, OCB_DATA, cmd1, cmd2);

	/* Trigger attention */
	occ_putscom(client, ATTN_DATA, 0x01010000, 0x00000000);

	/* Get response data */
	occ_putscom(client, OCB_ADDRESS, OCC_RESPONSE_ADDR, 0x00000000);
	occ_getscomb(client, OCB_DATA, resp, 0);

	/* return status */
	return resp[2];
}

static int occ_get_all(struct i2c_client *client, struct occ_response *occ_resp)
{
	uint8_t *occ_data;
	uint16_t num_bytes;
	int i;
	int ret;
	uint8_t poll_cmd_data;

	poll_cmd_data = 0x10;

	/*
	 * TODO: fetch header, and then allocate the rest of the buffer based
	 * on the header size. Assuming the OCC has a fixed sized header
	 */
	occ_data = devm_kzalloc(&client->dev, OCC_DATA_MAX, GFP_KERNEL);

	ret = occ_send_cmd(client, 0, 0, 1, &poll_cmd_data, occ_data);
	if (ret) {
		dev_err(&client->dev, "ERROR: OCC Poll: 0x%x\n", ret);
		ret = -EINVAL;
		goto out;
	}

	num_bytes = get_occdata_length(occ_data);

	dev_dbg(&client->dev, "OCC data length: %d\n", num_bytes);

	if (num_bytes > OCC_DATA_MAX) {
		dev_dbg(&client->dev, "ERROR: OCC data length must be < 4KB\n");
		ret = -EINVAL;
		goto out;
	}

	if (num_bytes <= 0) {
		dev_dbg(&client->dev, "ERROR: OCC data length is zero\n");
		ret = -EINVAL;
		goto out;
	}

	/* read remaining data */
	for (i = 8; i < num_bytes + 8; i = i + 8)
		occ_getscomb(client, OCB_DATA, occ_data, i);

	ret = parse_occ_response(client, occ_data, occ_resp);

out:
	devm_kfree(&client->dev, occ_data);
	return ret;
}


static int occ_update_device(struct device *dev)
{
	struct occ_drv_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret = 0;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + data->update_interval)
	    || !data->valid) {
		data->valid = 1;
		ret = occ_get_all(client, &data->occ_resp);
		if (ret)
			data->valid = 0;
		data->last_updated = jiffies;
	}
	mutex_unlock(&data->update_lock);

	return ret;
}


static void *occ_get_sensor(struct device *hwmon_dev, enum sensor_t t)
{
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret;

	ret = occ_update_device(dev);
	if (ret != 0) {
		dev_dbg(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return NULL;
	}

	return occ_get_sensor_by_type(&data->occ_resp, t);
}

static int occ_get_sensor_value(struct device *hwmon_dev, enum sensor_t t,
					int index)
{
	void *sensor;

	if (t == caps)
		return -1;

	sensor = occ_get_sensor(hwmon_dev, t);

	if (!sensor)
		return -1;

	if (t == power)
		return ((struct power_sensor *)sensor)[index].value;

	return ((struct occ_sensor *)sensor)[index].value;
}

static int occ_get_sensor_id(struct device *hwmon_dev, enum sensor_t t,
					int index)
{
	void *sensor;

	if (t == caps)
		return -1;

	sensor = occ_get_sensor(hwmon_dev, t);

	if (!sensor)
		return -1;

	if (t == power)
		return ((struct power_sensor *)sensor)[index].sensor_id;

	return ((struct occ_sensor *)sensor)[index].sensor_id;
}

/* sysfs attributes for occ hwmon device */

static ssize_t show_input(struct device *hwmon_dev,
				struct device_attribute *da, char *buf)
{
	struct sensor_attr_data *sdata = container_of(da,
					struct sensor_attr_data, dev_attr);
	int val;

	val = occ_get_sensor_value(hwmon_dev, sdata->type,
					sdata->hwmon_index - 1);
	if (sdata->type == temp)
		/* in millidegree Celsius */
		val *= 1000;

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", val);
}

static ssize_t show_label(struct device *hwmon_dev,
			struct device_attribute *da, char *buf)
{
	struct sensor_attr_data *sdata = container_of(da,
					struct sensor_attr_data, dev_attr);
	int val;

	val = occ_get_sensor_id(hwmon_dev, sdata->type,
					sdata->hwmon_index - 1);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", val);
}

static ssize_t show_caps(struct device *hwmon_dev,
		struct device_attribute *da, char *buf)
{
	struct sensor_attr_data *sdata = container_of(da,
					struct sensor_attr_data, dev_attr);
	int nr = sdata->attr_id;
	int n = sdata->hwmon_index - 1;
	struct caps_sensor *sensor;
	int val;

	sensor = occ_get_sensor(hwmon_dev, caps);
	if (!sensor) {
		val = -1;
		return snprintf(buf, PAGE_SIZE - 1, "%d\n", val);
	}

	switch (nr) {
	case 0:
		val = sensor[n].curr_powercap;
		break;
	case 1:
		val = sensor[n].curr_powerreading;
		break;
	case 2:
		val = sensor[n].norm_powercap;
		break;
	case 3:
		val = sensor[n].max_powercap;
		break;
	case 4:
		val = sensor[n].min_powercap;
		break;
	case 5:
		val = sensor[n].user_powerlimit;
		break;
	default:
		val = -1;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", val);
}

static ssize_t show_update_interval(struct device *hwmon_dev,
				struct device_attribute *attr, char *buf)
{
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%u\n",
		jiffies_to_msecs(data->update_interval));
}

static ssize_t set_update_interval(struct device *hwmon_dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	data->update_interval = msecs_to_jiffies(val);
	return count;
}
static DEVICE_ATTR(update_interval, S_IWUSR | S_IRUGO,
		show_update_interval, set_update_interval);

static ssize_t show_name(struct device *hwmon_dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE - 1, "%s\n", OCC_I2C_NAME);
}
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static ssize_t show_user_powercap(struct device *hwmon_dev,
				struct device_attribute *attr, char *buf)
{
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", data->user_powercap);
}


static ssize_t set_user_powercap(struct device *hwmon_dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	uint16_t val;
	uint8_t resp[8];
	int err;

	err = kstrtou16(buf, 10, &val);
	if (err)
		return err;

	dev_dbg(dev, "set user powercap to: %u\n", val);
	val = cpu_to_le16(val);
	err = occ_send_cmd(client, 0, 0x22, 2, (uint8_t *)&val, resp);
	if (err != 0) {
		dev_dbg(dev,
			"ERROR: Set User Powercap: wrong return status: %x\n",
			err);
		if (err == 0x13)
			dev_info(dev,
				"ERROR: set invalid powercap value: %x\n", val);
		return -EINVAL;
	}
	data->user_powercap = val;
	return count;
}
static DEVICE_ATTR(user_powercap, S_IWUSR | S_IRUGO,
		show_user_powercap, set_user_powercap);

static void deinit_sensor_groups(struct device *hwmon_dev,
					struct sensor_group *sensor_groups)
{
	int cnt;

	for (cnt = 0; cnt < MAX_OCC_SENSOR_TYPE; cnt++) {
		if (sensor_groups[cnt].group.attrs)
			devm_kfree(hwmon_dev, sensor_groups[cnt].group.attrs);
		if (sensor_groups[cnt].sattr)
			devm_kfree(hwmon_dev, sensor_groups[cnt].sattr);
		sensor_groups[cnt].group.attrs = NULL;
		sensor_groups[cnt].sattr = NULL;
	}
}

static void occ_remove_hwmon_attrs(struct device *hwmon_dev)
{
	struct occ_drv_data *data = dev_get_drvdata(hwmon_dev->parent);
	struct sensor_group *sensor_groups = data->sensor_groups;
	int i;

	if (!hwmon_dev)
		return;

	device_remove_file(hwmon_dev, &dev_attr_update_interval);
	device_remove_file(hwmon_dev, &dev_attr_name);
	device_remove_file(hwmon_dev, &dev_attr_user_powercap);

	for (i = 0; i < MAX_OCC_SENSOR_TYPE; i++)
		sysfs_remove_group(&hwmon_dev->kobj, &sensor_groups[i].group);

	deinit_sensor_groups(hwmon_dev, sensor_groups);
}

static void sensor_attr_init(struct sensor_attr_data *sdata,
				char *sensor_group_name,
				char *attr_name,
				ssize_t (*show)(struct device *dev,
						struct device_attribute *attr,
						char *buf))
{
	sysfs_attr_init(&sdata->dev_attr.attr);

	snprintf(sdata->name, MAX_SENSOR_ATTR_LEN, "%s%d_%s",
		sensor_group_name, sdata->hwmon_index, attr_name);
	sdata->dev_attr.attr.name = sdata->name;
	sdata->dev_attr.attr.mode = S_IRUGO;
	sdata->dev_attr.show = show;
}

/* create hwmon sensor sysfs attributes */
static int create_sensor_group(struct device *hwmon_dev, enum sensor_t type,
				int sensor_num)
{
	struct occ_drv_data *data = dev_get_drvdata(hwmon_dev->parent);
	struct sensor_group *sensor_groups = data->sensor_groups;
	struct sensor_attr_data *sdata;
	int ret;
	int cnt;

	/* each sensor has 'label' and 'input' attributes */
	sensor_groups[type].group.attrs = devm_kzalloc(hwmon_dev,
						sizeof(struct attribute *) *
						sensor_num * 2 + 1, GFP_KERNEL);
	if (!sensor_groups[type].group.attrs) {
		ret = -ENOMEM;
		goto err;
	}

	sensor_groups[type].sattr = devm_kzalloc(hwmon_dev,
					sizeof(struct sensor_attr_data) *
					sensor_num * 2, GFP_KERNEL);
	if (!sensor_groups[type].sattr) {
		ret = -ENOMEM;
		goto err;
	}

	for (cnt = 0; cnt < sensor_num; cnt++) {
		sdata = &sensor_groups[type].sattr[cnt];
		/* hwomon attributes index starts from 1 */
		sdata->hwmon_index = cnt + 1;
		sdata->type = type;
		sensor_attr_init(sdata, sensor_groups[type].name, "input",
					show_input);
		sensor_groups[type].group.attrs[cnt] = &sdata->dev_attr.attr;

		sdata = &sensor_groups[type].sattr[cnt + sensor_num];
		sdata->hwmon_index = cnt + 1;
		sdata->type = type;
		sensor_attr_init(sdata, sensor_groups[type].name, "label",
					show_label);
		sensor_groups[type].group.attrs[cnt + sensor_num] =
			&sdata->dev_attr.attr;
	}

	ret = sysfs_create_group(&hwmon_dev->kobj, &sensor_groups[type].group);
	if (ret)
		goto err;

	return ret;
err:
	deinit_sensor_groups(hwmon_dev, sensor_groups);
	return ret;
}

static void caps_sensor_attr_init(struct sensor_attr_data *sdata,
					char *attr_name, uint32_t hwmon_index,
					uint32_t attr_id)
{
	sdata->type = caps;
	sdata->hwmon_index = hwmon_index;
	sdata->attr_id = attr_id;

	/* FIXME, to be compatible with user space app, we do not
	 * generate caps1_* attributes.
	 */
	if (sdata->hwmon_index == 1)
		snprintf(sdata->name, MAX_SENSOR_ATTR_LEN, "%s_%s",
			"caps", attr_name);
	else
		snprintf(sdata->name, MAX_SENSOR_ATTR_LEN, "%s%d_%s",
			"caps", sdata->hwmon_index, attr_name);

	sysfs_attr_init(&sdata->dev_attr.attr);
	sdata->dev_attr.attr.name = sdata->name;
	sdata->dev_attr.attr.mode = S_IRUGO;
	sdata->dev_attr.show = show_caps;
}

static char *caps_sensor_name[] = {
	"curr_powercap",
	"curr_powerreading",
	"norm_powercap",
	"max_powercap",
	"min_powercap",
	"user_powerlimit",
};

static int create_caps_sensor_group(struct device *hwmon_dev, int sensor_num)
{
	struct occ_drv_data *data = dev_get_drvdata(hwmon_dev->parent);
	struct sensor_group *sensor_groups = data->sensor_groups;
	int field_num = ARRAY_SIZE(caps_sensor_name);
	struct sensor_attr_data *sdata;
	int ret;
	int cnt;
	int i;

	sensor_groups[caps].group.attrs = devm_kzalloc(hwmon_dev,
						sizeof(struct attribute *) *
						sensor_num * field_num + 1,
						GFP_KERNEL);
	if (!sensor_groups[caps].group.attrs) {
		ret = -ENOMEM;
		goto err;
	}

	sensor_groups[caps].sattr = devm_kzalloc(hwmon_dev,
					sizeof(struct sensor_attr_data) *
					sensor_num * field_num,
					GFP_KERNEL);
	if (!sensor_groups[caps].sattr) {
		ret = -ENOMEM;
		goto err;
	}

	for (cnt = 0; cnt < sensor_num; cnt++) {
		for (i = 0; i < field_num; i++) {
			sdata = &sensor_groups[caps].sattr[cnt * field_num + i];
			caps_sensor_attr_init(sdata, caps_sensor_name[i],
						cnt + 1, i);
			sensor_groups[caps].group.attrs[cnt * field_num + i] =
						&sdata->dev_attr.attr;
		}
	}

	ret = sysfs_create_group(&hwmon_dev->kobj, &sensor_groups[caps].group);
	if (ret)
		goto err;

	return ret;
err:
	deinit_sensor_groups(hwmon_dev, sensor_groups);
	return ret;
}

static int occ_create_hwmon_attrs(struct device *dev)
{
	struct occ_drv_data *drv_data = dev_get_drvdata(dev);
	struct device *hwmon_dev = drv_data->hwmon_dev;
	struct sensor_group *sensor_groups = drv_data->sensor_groups;
	int i;
	int sensor_num;
	int ret;
	struct occ_response *rsp;
	enum sensor_t t;

	rsp = &drv_data->occ_resp;

	for (i = 0; i < ARRAY_SIZE(rsp->sensor_block_id); i++)
		rsp->sensor_block_id[i] = -1;

	/* read sensor data from occ. */
	ret = occ_update_device(dev);
	if (ret != 0) {
		dev_dbg(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}
	if (!rsp->blocks)
		return -1;

	ret = device_create_file(hwmon_dev, &dev_attr_name);
	if (ret)
		goto error;

	ret = device_create_file(hwmon_dev, &dev_attr_update_interval);
	if (ret)
		goto error;

	if (rsp->sensor_block_id[caps] >= 0) {
		/* user powercap: only for master OCC */
		ret = device_create_file(hwmon_dev, &dev_attr_user_powercap);
		if (ret)
			goto error;
	}

	sensor_groups[freq].name = "freq";
	sensor_groups[temp].name = "temp";
	sensor_groups[power].name = "power";
	sensor_groups[caps].name =  "caps";

	for (t = 0; t < MAX_OCC_SENSOR_TYPE; t++) {
		if (rsp->sensor_block_id[t] < 0)
			continue;

		sensor_num =
			rsp->blocks[rsp->sensor_block_id[t]].sensor_num;
		if (t == caps)
			ret = create_caps_sensor_group(hwmon_dev, sensor_num);
		else
			ret = create_sensor_group(hwmon_dev, t, sensor_num);
		if (ret)
			goto error;
	}

	return 0;
error:
	dev_err(dev, "ERROR: cannot create hwmon attributes\n");
	occ_remove_hwmon_attrs(drv_data->hwmon_dev);
	return ret;
}

static ssize_t show_occ_online(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct occ_drv_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%lu\n", data->occ_online);
}

static ssize_t set_occ_online(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct occ_drv_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	if (val == 1) {
		if (data->occ_online == 1)
			return count;

		/* populate hwmon sysfs attr using sensor data */
		dev_dbg(dev, "occ register hwmon @0x%x\n", data->client->addr);

		data->hwmon_dev = hwmon_device_register(dev);
		if (IS_ERR(data->hwmon_dev))
			return PTR_ERR(data->hwmon_dev);

		err = occ_create_hwmon_attrs(dev);
		if (err) {
			hwmon_device_unregister(data->hwmon_dev);
			return err;
		}
		data->hwmon_dev->parent = dev;
	} else if (val == 0) {
		if (data->occ_online == 0)
			return count;

		occ_remove_hwmon_attrs(data->hwmon_dev);
		hwmon_device_unregister(data->hwmon_dev);
		data->hwmon_dev = NULL;
	} else
		return -EINVAL;

	data->occ_online = val;
	return count;
}

static DEVICE_ATTR(online, S_IWUSR | S_IRUGO,
		show_occ_online, set_occ_online);

static int occ_create_i2c_sysfs_attr(struct device *dev)
{
	/* create an i2c sysfs attribute, to indicate whether OCC is active */
	return device_create_file(dev, &dev_attr_online);
}


/* device probe and removal */

enum occ_type {
	occ_id,
};

static int occ_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct occ_drv_data *data;

	data = devm_kzalloc(dev, sizeof(struct occ_drv_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);
	data->update_interval = HZ;

	occ_create_i2c_sysfs_attr(dev);

	dev_info(dev, "occ i2c driver ready: i2c addr@0x%x\n", client->addr);

	return 0;
}

static int occ_remove(struct i2c_client *client)
{
	struct occ_drv_data *data = i2c_get_clientdata(client);

	/* free allocated sensor memory */
	deinit_occ_resp_buf(&data->occ_resp);

	device_remove_file(&client->dev, &dev_attr_online);

	if (!data->hwmon_dev)
		return 0;

	occ_remove_hwmon_attrs(data->hwmon_dev);
	hwmon_device_unregister(data->hwmon_dev);
	return 0;
}

/* used by old-style board info. */
static const struct i2c_device_id occ_ids[] = {
	{ OCC_I2C_NAME, occ_id, },
	{ /* LIST END */ }
};
MODULE_DEVICE_TABLE(i2c, occ_ids);

/* use by device table */
static const struct of_device_id i2c_occ_of_match[] = {
	{.compatible = "ibm,occ-i2c"},
	{},
};
MODULE_DEVICE_TABLE(of, i2c_occ_of_match);

/* i2c-core uses i2c-detect() to detect device in bellow address list.
 *  If exists, address will be assigned to client.
 * It is also possible to read address from device table.
 */
static const unsigned short normal_i2c[] = {0x50, 0x51, I2C_CLIENT_END };

static struct i2c_driver occ_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= OCC_I2C_NAME,
		.pm	= NULL,
		.of_match_table = i2c_occ_of_match,
	},
	.probe		= occ_probe,
	.remove		= occ_remove,
	.id_table       = occ_ids,
	.address_list	= normal_i2c,
};

module_i2c_driver(occ_driver);

MODULE_AUTHOR("Li Yi <shliyi@cn.ibm.com>");
MODULE_DESCRIPTION("BMC OCC hwmon driver");
MODULE_LICENSE("GPL");
