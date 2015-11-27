/*
 * BMC OCC HWMON driver - read IBM Power8 OCC (On Chip Controller)
 * sensor data via i2c.
 *
 * Copyright (c) 2015 IBM (Alvin Wang, Li Yi)
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


/* OCC sensor data format */
struct occ_sensor {
	uint16_t sensor_id;
	uint16_t value;
};

struct powr_sensor {
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
	char sensor_type[5];
	uint8_t reserved0;
	uint8_t sensor_format;
	uint8_t sensor_length;
	uint8_t num_of_sensors;
	struct occ_sensor *sensor;
	struct powr_sensor *powr;
	struct caps_sensor *caps;
};

struct occ_poll_data {
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
	char occ_version_string[17];
	char sensor_eye_catcher[7];
	uint8_t num_of_sensor_blocks;
	uint8_t sensor_data_version;
	struct sensor_data_block *blocks;
};

struct occ_response {
	uint8_t sequence_num;
	uint8_t cmd_type;
	uint8_t rtn_status;
	uint16_t data_length;
	struct occ_poll_data data;
	uint16_t chk_sum;
	int temp_block_id;
	int freq_block_id;
	int power_block_id;
	int caps_block_id;
};

/* data private to each client */
struct occ_drv_data {
	struct i2c_client	*client;
	struct device		*hwmon_dev;
	struct mutex		update_lock;
	bool			valid;
	unsigned long		last_updated;
	/* Minimum timer interval for sampling In jiffies */
	unsigned long		sample_time;
	struct occ_response	occ_resp;
};

/* i2c read and write occ sensors */

#define OCC_DATA_MAX 4096 /* 4KB at most */
#define I2C_STATUS_REG 0x000d0001
#define I2C_ERROR_REG  0x000d0002
#define I2C_READ_ERROR 1
#define I2C_WRITE_ERROR 2
#define I2C_DATABUFFER_SIZE_ERROR 3

/* To generate attn to OCC */
#define ATTN_DATA                0x0006B035

/* For BMC to read/write SRAM */
#define OCB_ADDRESS              0x0006B070
#define OCB_DATA                 0x0006B075
#define OCB_STATUS_CONTROL_AND   0x0006B072
#define OCB_STATUS_CONTROL_OR    0x0006B073

#define OCC_COMMAND_ADDR 0xFFFF6000
#define OCC_RESPONSE_ADDR 0xFFFF7000

static void deinit_occ_resp_buf(struct occ_response *p)
{
	int b;

	if (!p)
		return;

	if (!p->data.blocks)
		return;

	for (b = 0; b < p->data.num_of_sensor_blocks; b++) {
		if (!p->data.blocks[b].sensor)
			kfree(p->data.blocks[b].sensor);
		if (!p->data.blocks[b].powr)
			kfree(p->data.blocks[b].powr);
		if (!p->data.blocks[b].caps)
			kfree(p->data.blocks[b].caps);
	}

	kfree(p->data.blocks);

	memset(p, 0, sizeof(*p));
}

static ssize_t occ_i2c_read(struct i2c_client *client, char *buf, size_t count)
{
	int ret = 0;

	if (count > OCC_DATA_MAX)
		count = OCC_DATA_MAX;

	dev_dbg(&client->dev, "i2c_read: reading %zu bytes @0x%x.\n",
		count, client->addr);
	return i2c_master_recv(client, buf, count);
}

static ssize_t occ_i2c_write(struct i2c_client *client, const char *buf,
				size_t count)
{
	int ret = 0;

	if (count > OCC_DATA_MAX)
		count = OCC_DATA_MAX;

	dev_dbg(&client->dev, "i2c_write: writing %zu bytes @0x%x.\n",
		count, client->addr);
	return i2c_master_send(client, buf, count);
}

/* read two 4-byte value */
static int occ_getscom(struct i2c_client *client, uint32_t address,
			uint32_t *value0, uint32_t *value1)
{
	uint32_t ret = 0;
	char buf[8];

	/* P8 i2c slave requires address to be shifted by 1 */
	address = address << 1;

	ret = occ_i2c_write(client, (const char *)&address,
		sizeof(address));
	/* FIXME: ast i2c driver does not read corret value
	 * if (ret != sizeof(address))
	 *	return -I2C_WRITE_ERROR;
	 */
	ret = occ_i2c_read(client, buf, sizeof(buf));
	/* FIXME: ast i2c driver does not read corret value
	 * if (ret != sizeof(buf))
	 *	return -I2C_READ_ERROR;
	 */
	memcpy(value1, &buf[0], sizeof(*value1));
	memcpy(value0, &buf[4], sizeof(*value0));

	return 0;
}

/* read 8-byte value and put into data[offset] */
static int occ_getscomb(struct i2c_client *client, uint32_t address,
		u8 *data, int offset)
{
	uint32_t ret = 0;
	char buf[8];
	int i = 0;

	/* P8 i2c slave requires address to be shifted by 1 */
	address = address << 1;

	ret = occ_i2c_write(client, (const char *)&address,
		sizeof(address));
	/* FIXME: ast i2c driver does not read corret value
	 * if (ret != sizeof(address))
	 *	return -I2C_WRITE_ERROR;
	 */
	ret = occ_i2c_read(client, buf, sizeof(buf));
	/* FIXME: ast i2c driver does not read corret value
	 * if (ret != sizeof(buf))
	 *	return -I2C_READ_ERROR;
	 */
	for (i = 0; i < 8; i++)
		data[offset + i] = buf[7 - i];

	return 0;
}

static int occ_putscom(struct i2c_client *client, uint32_t address,
		uint32_t data0, uint32_t data1)
{
	char buf[12];
	uint32_t ret = 0;

	/* P8 i2c slave requires address to be shifted by 1 */
	address = address << 1;

	memcpy(&buf[0], &address, sizeof(address));
	memcpy(&buf[4], &data1, sizeof(data1));
	memcpy(&buf[8],	&data0, sizeof(data0));

	ret = occ_i2c_write(client, buf, sizeof(buf));
	/* FIXME: ast i2c driver does not read corret value
	 * if (ret != sizeof(buf))
	 *	return I2C_WRITE_ERROR;
	 */
	return 0;
}

static inline uint16_t get_occdata_length(u8 *d)
{
	uint16_t data_length = 0;

	data_length = d[3] << 8;
	data_length = data_length | d[4];
	return data_length;
}


static int parse_occ_response(struct i2c_client *client,
		char *d, struct occ_response *o)
{
	int b = 0;
	int s = 0;
	int ret = 0;
	int dnum = 45;
	struct occ_sensor *f_sensor;
	struct occ_sensor *t_sensor;
	struct powr_sensor *p_sensor;
	struct caps_sensor *c_sensor;

	o->sequence_num = d[0];
	o->cmd_type = d[1];
	o->rtn_status = d[2];
	o->data_length = d[3] << 8;
	o->data_length = o->data_length | d[4];
	o->data.status = d[5];
	o->data.ext_status = d[6];
	o->data.occs_present = d[7];
	o->data.config = d[8];
	o->data.occ_state = d[9];
	o->data.reserved0 = d[10];
	o->data.reserved1 = d[11];
	o->data.error_log_id = d[12];
	o->data.error_log_addr_start = d[13] << 24;
	o->data.error_log_addr_start =
		o->data.error_log_addr_start | d[14] << 16;
	o->data.error_log_addr_start =
		o->data.error_log_addr_start | d[15] << 8;
	o->data.error_log_addr_start =
		o->data.error_log_addr_start | d[16];
	o->data.error_log_length = d[17] << 8;
	o->data.error_log_length = o->data.error_log_length | d[18];
	o->data.reserved2 = d[19];
	o->data.reserved3 = d[20];
	strncpy(o->data.occ_version_string, &d[21], 16);
	strncpy(&o->data.sensor_eye_catcher[0], &d[37], 6);
	o->data.sensor_eye_catcher[6] = '\0';
	o->data.num_of_sensor_blocks = d[43];
	o->data.sensor_data_version = d[44];

	if (strcmp(o->data.sensor_eye_catcher, "SENSOR") != 0) {
		dev_err(&client->dev,
			"ERROR: SENSOR not found at byte 37 (%s)\n",
			o->data.sensor_eye_catcher);
		return -1;
	}

	if (o->data.num_of_sensor_blocks == 0) {
		dev_err(&client->dev, "ERROR: SENSOR block num is 0\n");
		return -1;
	}

	o->data.blocks = kzalloc(sizeof(struct sensor_data_block) *
				o->data.num_of_sensor_blocks, GFP_KERNEL);
	if (!o->data.blocks)
		return -ENOMEM;

	dev_dbg(&client->dev, "Reading %d sensor blocks\n",
		o->data.num_of_sensor_blocks);
	o->temp_block_id = -1;
	o->freq_block_id = -1;
	o->power_block_id = -1;
	o->caps_block_id = -1;
	for (b = 0; b < o->data.num_of_sensor_blocks; b++) {
		/* 8-byte sensor block head */
		strncpy(&o->data.blocks[b].sensor_type[0],
			(const char *)&d[dnum], 4);
		o->data.blocks[b].reserved0 = d[dnum+4];
		o->data.blocks[b].sensor_format = d[dnum+5];
		o->data.blocks[b].sensor_length = d[dnum+6];
		o->data.blocks[b].num_of_sensors = d[dnum+7];
		dnum = dnum + 8;

		dev_dbg(&client->dev,
			"sensor block[%d]: type: %s, num_of_sensors: %d\n",
			b, o->data.blocks[b].sensor_type,
			o->data.blocks[b].num_of_sensors);

		/* empty sensor block */
		if (o->data.blocks[b].num_of_sensors <= 0)
			continue;
		if (o->data.blocks[b].sensor_length == 0)
			continue;

		if (strcmp(o->data.blocks[b].sensor_type, "FREQ") == 0) {
			o->data.blocks[b].sensor =
				kzalloc(sizeof(struct occ_sensor) *
					o->data.blocks[b].num_of_sensors,
					GFP_KERNEL);

			if (!o->data.blocks[b].sensor) {
				ret = -ENOMEM;
				goto abort;
			}
			o->freq_block_id = b;
			for (s = 0; s < o->data.blocks[b].num_of_sensors; s++) {
				f_sensor = &o->data.blocks[b].sensor[s];
				f_sensor->sensor_id = (d[dnum]<<8) | d[dnum+1];
				f_sensor->value = (d[dnum+2]<<8) | d[dnum+3];
				dev_dbg(&client->dev,
					"sensor[%d]-[%d]: id: %u, value: %u\n",
					b, s, f_sensor->sensor_id,
					f_sensor->value);
				dnum = dnum + o->data.blocks[b].sensor_length;
			}
		} else if (strcmp(o->data.blocks[b].sensor_type, "TEMP") == 0) {

			o->data.blocks[b].sensor =
				kzalloc(sizeof(struct occ_sensor) *
					o->data.blocks[b].num_of_sensors,
					GFP_KERNEL);

			if (!o->data.blocks[b].sensor) {
				ret = -ENOMEM;
				goto abort;
			}

			o->temp_block_id = b;
			for (s = 0; s < o->data.blocks[b].num_of_sensors; s++) {
				t_sensor = &o->data.blocks[b].sensor[s];
				t_sensor->sensor_id = (d[dnum]<<8) | d[dnum+1];
				t_sensor->value = (d[dnum+2] << 8) | d[dnum+3];
				dev_dbg(&client->dev,
					"sensor[%d]-[%d]: id: %u, value: %u\n",
					b, s, t_sensor->sensor_id,
					t_sensor->value);
				dnum = dnum + o->data.blocks[b].sensor_length;
			}
		} else if (strcmp(o->data.blocks[b].sensor_type, "POWR") == 0) {

			o->data.blocks[b].powr =
				kzalloc(sizeof(struct powr_sensor) *
					o->data.blocks[b].num_of_sensors,
					GFP_KERNEL);

			if (!o->data.blocks[b].powr) {
				ret = -ENOMEM;
				goto abort;
			}
			o->power_block_id = b;
			for (s = 0; s < o->data.blocks[b].num_of_sensors; s++) {
				p_sensor = &o->data.blocks[b].powr[s];
				p_sensor->sensor_id = (d[dnum]<<8) | d[dnum+1];
				p_sensor->update_tag =
					(d[dnum+2] << 24) | (d[dnum+3] << 16) |
					(d[dnum+4] << 8) | d[dnum+5];
				p_sensor->accumulator =
					(d[dnum+6] << 24) | (d[dnum+7] << 16) |
					(d[dnum+8] << 8) | d[dnum+9];
				p_sensor->value =
					(d[dnum+10] << 8) | d[dnum+11];

				dev_dbg(&client->dev,
					"sensor[%d]-[%d]: id: %u, value: %u\n",
					b, s, p_sensor->sensor_id,
					p_sensor->value);

				dnum = dnum + o->data.blocks[b].sensor_length;
			}
		} else if (strcmp(o->data.blocks[b].sensor_type, "CAPS") == 0) {

			o->data.blocks[b].caps =
				kzalloc(sizeof(struct caps_sensor) *
					o->data.blocks[b].num_of_sensors,
					GFP_KERNEL);

			if (!o->data.blocks[b].caps) {
				ret = -ENOMEM;
				goto abort;
			}
			o->caps_block_id = b;
			for (s = 0; s < o->data.blocks[b].num_of_sensors; s++) {
				c_sensor = &o->data.blocks[b].caps[s];
				c_sensor->curr_powercap =
					(d[dnum] << 8) | d[dnum+1];
				c_sensor->curr_powerreading =
					(d[dnum+2] << 8) | d[dnum+3];
				c_sensor->norm_powercap =
					(d[dnum+4] << 8) | d[dnum+5];
				c_sensor->max_powercap =
					(d[dnum+6] << 8) | d[dnum+7];
				c_sensor->min_powercap =
					(d[dnum+8] << 8) | d[dnum+9];
				c_sensor->user_powerlimit =
					(d[dnum+10] << 8) | d[dnum+11];

				dnum = dnum + o->data.blocks[b].sensor_length;
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
			dev_err(&client->dev,
				"ERROR: sensor type %s not supported\n",
				o->data.blocks[b].sensor_type);
			ret = -1;
			goto abort;
		}
	}

	return 0;
abort:
	deinit_occ_resp_buf(o);
	return ret;
}

static int occ_get_all(struct i2c_client *client, struct occ_response *occ_resp)
{
	char occ_data[OCC_DATA_MAX];
	uint16_t num_bytes = 0;
	int b = 0;
	int ret = 0;

	/* Init OCB */
	occ_putscom(client, OCB_STATUS_CONTROL_OR,  0x08000000, 0x00000000);
	occ_putscom(client, OCB_STATUS_CONTROL_AND, 0xFBFFFFFF, 0xFFFFFFFF);

	/* Send poll command to OCC */
	occ_putscom(client, OCB_ADDRESS, OCC_COMMAND_ADDR, 0x00000000);
	occ_putscom(client, OCB_ADDRESS, OCC_COMMAND_ADDR, 0x00000000);
	occ_putscom(client, OCB_DATA, 0x00000001, 0x10001100);

	/* Trigger ATTN */
	occ_putscom(client, ATTN_DATA, 0x01010000, 0x00000000);

	/* Get response data */
	occ_putscom(client, OCB_ADDRESS, OCC_RESPONSE_ADDR, 0x00000000);
	occ_getscomb(client, OCB_DATA, occ_data, 0);

	num_bytes = get_occdata_length(occ_data);

	dev_dbg(&client->dev, "OCC data length: %d\n", num_bytes);

	if (num_bytes > OCC_DATA_MAX) {
		dev_err(&client->dev, "ERROR: OCC data length must be < 4KB\n");
		return -1;
	}

	if (num_bytes <= 0) {
		dev_err(&client->dev, "ERROR: OCC data length is zero\n");
		return -1;
	}

	for (b = 8; b < num_bytes + 8; b = b + 8)
		occ_getscomb(client, OCB_DATA, occ_data, b);

	ret = parse_occ_response(client, occ_data, occ_resp);

	return ret;
}


static int occ_update_device(struct device *dev)
{
	struct occ_drv_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret = 0;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + data->sample_time)
	    || !data->valid) {
		deinit_occ_resp_buf(&data->occ_resp);

		ret = occ_get_all(client, &data->occ_resp);

		data->last_updated = jiffies;
		data->valid = 1;
	}
	mutex_unlock(&data->update_lock);

	return ret;
}

/* sysfs attributes for hwmon */

static ssize_t show_occ_temp_input(struct device *hwmon_dev,
		struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index;
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	struct occ_sensor *sensor;
	int val = 0;

	ret = occ_update_device(dev);

	if (ret != 0) {
		dev_err(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}

	if (!data->occ_resp.data.blocks)
		return -1;

	sensor =
		data->occ_resp.data.blocks[data->occ_resp.temp_block_id].sensor;

	if (!sensor)
		return -1;

	/* in millidegree Celsius */
	val = sensor[n - 1].value * 1000;

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_occ_temp_label(struct device *hwmon_dev,
		struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index;
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	struct occ_sensor *sensor;
	int val = 0;

	ret = occ_update_device(dev);

	if (ret != 0) {
		dev_err(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}

	if (!data->occ_resp.data.blocks)
		return -1;

	sensor =
		data->occ_resp.data.blocks[data->occ_resp.temp_block_id].sensor;

	if (!sensor)
		return -1;

	dev_dbg(dev, "temp_block_id: %d, sensor: %d\n",
		data->occ_resp.temp_block_id, n - 1);
	val = sensor[n - 1].sensor_id;
	dev_dbg(dev, "temp%d sensor id: %d\n", n, val);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_occ_power_label(struct device *hwmon_dev,
		struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index;
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	struct powr_sensor *sensor;
	int val = 0;

	ret = occ_update_device(dev);

	if (ret != 0) {
		dev_err(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "power_block_id: %d, sensor: %d\n",
		data->occ_resp.power_block_id, n - 1);

	if (!data->occ_resp.data.blocks)
		return -1;

	sensor =
		data->occ_resp.data.blocks[data->occ_resp.power_block_id].powr;
	if (!sensor)
		return -1;

	val = sensor[n - 1].sensor_id;
	dev_dbg(dev, "power%d sensor id: %d\n", n, val);

	return sprintf(buf, "%d\n", val);
}


static ssize_t show_occ_power_input(struct device *hwmon_dev,
		struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index;
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	struct powr_sensor *sensor;
	int val = 0;

	ret = occ_update_device(dev);

	if (ret != 0) {
		dev_err(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "power block_id: %d, sensor: %d\n",
		data->occ_resp.power_block_id, n - 1);

	if (!data->occ_resp.data.blocks)
		return -1;

	sensor =
		data->occ_resp.data.blocks[data->occ_resp.power_block_id].powr;
	if (!sensor)
		return -1;

	val = sensor[n - 1].value;
	dev_dbg(dev, "power%d sensor value: %d\n", n, val);

	return sprintf(buf, "%d\n", val);
}


static ssize_t show_occ_freq_label(struct device *hwmon_dev,
		struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index;
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	struct occ_sensor *sensor;
	int val = 0;

	ret = occ_update_device(dev);

	if (ret != 0) {
		dev_err(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}

	if (!data->occ_resp.data.blocks)
		return -1;

	sensor =
		data->occ_resp.data.blocks[data->occ_resp.freq_block_id].sensor;
	if (!sensor)
		return -1;

	dev_dbg(dev, "freq_block_id: %d, sensor: %d\n",
		data->occ_resp.freq_block_id, n - 1);
	val = sensor[n - 1].sensor_id;
	dev_dbg(dev, "freq%d sensor id: %d\n", n, val);

	return sprintf(buf, "%d\n", val);
}


static ssize_t show_occ_freq_input(struct device *hwmon_dev,
		struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int n = attr->index;
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	struct occ_sensor *sensor;
	int val = 0;

	ret = occ_update_device(dev);

	if (ret != 0) {
		dev_err(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}

	if (!data->occ_resp.data.blocks)
		return -1;

	sensor =
		data->occ_resp.data.blocks[data->occ_resp.freq_block_id].sensor;
	if (!sensor)
		return -1;

	dev_dbg(dev, "block_id: %d, sensor: %d\n",
		data->occ_resp.freq_block_id, n - 1);
	val = sensor[n - 1].value;
	dev_dbg(dev, "freq%d sensor value: %d\n", n, val);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_occ_caps(struct device *hwmon_dev,
		struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute_2 *attr = to_sensor_dev_attr_2(da);
	int nr = attr->nr;
	int n = attr->index;
	struct device *dev = hwmon_dev->parent;
	struct occ_drv_data *data = dev_get_drvdata(dev);
	int ret = 0;
	struct caps_sensor *sensor;
	int val = 0;

	ret = occ_update_device(dev);
	if (ret != 0) {
		dev_err(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "block_id: %d, sensor: %d, nr: %d\n",
		data->occ_resp.caps_block_id, n - 1, nr);
	if (!data->occ_resp.data.blocks)
		return -1;

	sensor = data->occ_resp.data.blocks[data->occ_resp.caps_block_id].caps;
	if (!sensor)
		return -1;

	switch (nr) {
	case 0:
		val = sensor[n - 1].curr_powercap;
		break;
	case 1:
		val = sensor[n - 1].curr_powerreading;
		break;
	case 2:
		val = sensor[n - 1].norm_powercap;
		break;
	case 3:
		val = sensor[n - 1].max_powercap;
		break;
	case 4:
		val = sensor[n - 1].min_powercap;
		break;
	case 5:
		val = sensor[n - 1].user_powerlimit;
		break;
	default:
		val = 0;
	}

	dev_dbg(dev, "caps%d sensor value: %d, nr: %d\n", n, val, nr);

	return sprintf(buf, "%d\n", val);
}

static struct sensor_device_attribute temp_input[] = {
	SENSOR_ATTR(temp1_input, S_IRUGO, show_occ_temp_input, NULL, 1),
	SENSOR_ATTR(temp2_input, S_IRUGO, show_occ_temp_input, NULL, 2),
	SENSOR_ATTR(temp3_input, S_IRUGO, show_occ_temp_input, NULL, 3),
	SENSOR_ATTR(temp4_input, S_IRUGO, show_occ_temp_input, NULL, 4),
	SENSOR_ATTR(temp5_input, S_IRUGO, show_occ_temp_input, NULL, 5),
	SENSOR_ATTR(temp6_input, S_IRUGO, show_occ_temp_input, NULL, 6),
	SENSOR_ATTR(temp7_input, S_IRUGO, show_occ_temp_input, NULL, 7),
	SENSOR_ATTR(temp8_input, S_IRUGO, show_occ_temp_input, NULL, 8),
	SENSOR_ATTR(temp9_input, S_IRUGO, show_occ_temp_input, NULL, 9),
	SENSOR_ATTR(temp10_input, S_IRUGO, show_occ_temp_input, NULL, 10),
	SENSOR_ATTR(temp11_input, S_IRUGO, show_occ_temp_input, NULL, 11),
	SENSOR_ATTR(temp12_input, S_IRUGO, show_occ_temp_input, NULL, 12),
	SENSOR_ATTR(temp13_input, S_IRUGO, show_occ_temp_input, NULL, 13),
	SENSOR_ATTR(temp14_input, S_IRUGO, show_occ_temp_input, NULL, 14),
	SENSOR_ATTR(temp15_input, S_IRUGO, show_occ_temp_input, NULL, 15),
	SENSOR_ATTR(temp16_input, S_IRUGO, show_occ_temp_input, NULL, 16),
	SENSOR_ATTR(temp17_input, S_IRUGO, show_occ_temp_input, NULL, 17),
	SENSOR_ATTR(temp18_input, S_IRUGO, show_occ_temp_input, NULL, 18),
	SENSOR_ATTR(temp19_input, S_IRUGO, show_occ_temp_input, NULL, 19),
	SENSOR_ATTR(temp20_input, S_IRUGO, show_occ_temp_input, NULL, 20),
	SENSOR_ATTR(temp21_input, S_IRUGO, show_occ_temp_input, NULL, 21),
	SENSOR_ATTR(temp22_input, S_IRUGO, show_occ_temp_input, NULL, 22),
};

static struct sensor_device_attribute temp_label[] = {
	SENSOR_ATTR(temp1_label, S_IRUGO, show_occ_temp_label, NULL, 1),
	SENSOR_ATTR(temp2_label, S_IRUGO, show_occ_temp_label, NULL, 2),
	SENSOR_ATTR(temp3_label, S_IRUGO, show_occ_temp_label, NULL, 3),
	SENSOR_ATTR(temp4_label, S_IRUGO, show_occ_temp_label, NULL, 4),
	SENSOR_ATTR(temp5_label, S_IRUGO, show_occ_temp_label, NULL, 5),
	SENSOR_ATTR(temp6_label, S_IRUGO, show_occ_temp_label, NULL, 6),
	SENSOR_ATTR(temp7_label, S_IRUGO, show_occ_temp_label, NULL, 7),
	SENSOR_ATTR(temp8_label, S_IRUGO, show_occ_temp_label, NULL, 8),
	SENSOR_ATTR(temp9_label, S_IRUGO, show_occ_temp_label, NULL, 9),
	SENSOR_ATTR(temp10_label, S_IRUGO, show_occ_temp_label, NULL, 10),
	SENSOR_ATTR(temp11_label, S_IRUGO, show_occ_temp_label, NULL, 11),
	SENSOR_ATTR(temp12_label, S_IRUGO, show_occ_temp_label, NULL, 12),
	SENSOR_ATTR(temp13_label, S_IRUGO, show_occ_temp_label, NULL, 13),
	SENSOR_ATTR(temp14_label, S_IRUGO, show_occ_temp_label, NULL, 14),
	SENSOR_ATTR(temp15_label, S_IRUGO, show_occ_temp_label, NULL, 15),
	SENSOR_ATTR(temp16_label, S_IRUGO, show_occ_temp_label, NULL, 16),
	SENSOR_ATTR(temp17_label, S_IRUGO, show_occ_temp_label, NULL, 17),
	SENSOR_ATTR(temp18_label, S_IRUGO, show_occ_temp_label, NULL, 18),
	SENSOR_ATTR(temp19_label, S_IRUGO, show_occ_temp_label, NULL, 19),
	SENSOR_ATTR(temp20_label, S_IRUGO, show_occ_temp_label, NULL, 20),
	SENSOR_ATTR(temp21_label, S_IRUGO, show_occ_temp_label, NULL, 21),
	SENSOR_ATTR(temp22_label, S_IRUGO, show_occ_temp_label, NULL, 22),

};

#define TEMP_UNIT_ATTRS(X)                      \
{	&temp_input[X].dev_attr.attr,           \
	&temp_label[X].dev_attr.attr,          \
	NULL                                    \
}

/* 10-core CPU, occ has 22 temp sensors, more socket, more sensors */
static struct attribute *occ_temp_attr[][3] = {
	TEMP_UNIT_ATTRS(0),
	TEMP_UNIT_ATTRS(1),
	TEMP_UNIT_ATTRS(2),
	TEMP_UNIT_ATTRS(3),
	TEMP_UNIT_ATTRS(4),
	TEMP_UNIT_ATTRS(5),
	TEMP_UNIT_ATTRS(6),
	TEMP_UNIT_ATTRS(7),
	TEMP_UNIT_ATTRS(8),
	TEMP_UNIT_ATTRS(9),
	TEMP_UNIT_ATTRS(10),
	TEMP_UNIT_ATTRS(11),
	TEMP_UNIT_ATTRS(12),
	TEMP_UNIT_ATTRS(13),
	TEMP_UNIT_ATTRS(14),
	TEMP_UNIT_ATTRS(15),
	TEMP_UNIT_ATTRS(16),
	TEMP_UNIT_ATTRS(17),
	TEMP_UNIT_ATTRS(18),
	TEMP_UNIT_ATTRS(19),
	TEMP_UNIT_ATTRS(20),
	TEMP_UNIT_ATTRS(21),
};

static const struct attribute_group occ_temp_attr_group[] = {
	{ .attrs = occ_temp_attr[0] },
	{ .attrs = occ_temp_attr[1] },
	{ .attrs = occ_temp_attr[2] },
	{ .attrs = occ_temp_attr[3] },
	{ .attrs = occ_temp_attr[4] },
	{ .attrs = occ_temp_attr[5] },
	{ .attrs = occ_temp_attr[6] },
	{ .attrs = occ_temp_attr[7] },
	{ .attrs = occ_temp_attr[8] },
	{ .attrs = occ_temp_attr[9] },
	{ .attrs = occ_temp_attr[10] },
	{ .attrs = occ_temp_attr[11] },
	{ .attrs = occ_temp_attr[12] },
	{ .attrs = occ_temp_attr[13] },
	{ .attrs = occ_temp_attr[14] },
	{ .attrs = occ_temp_attr[15] },
	{ .attrs = occ_temp_attr[16] },
	{ .attrs = occ_temp_attr[17] },
	{ .attrs = occ_temp_attr[18] },
	{ .attrs = occ_temp_attr[19] },
	{ .attrs = occ_temp_attr[20] },
	{ .attrs = occ_temp_attr[21] },
};


static struct sensor_device_attribute freq_input[] = {
	SENSOR_ATTR(freq1_input, S_IRUGO, show_occ_freq_input, NULL, 1),
	SENSOR_ATTR(freq2_input, S_IRUGO, show_occ_freq_input, NULL, 2),
	SENSOR_ATTR(freq3_input, S_IRUGO, show_occ_freq_input, NULL, 3),
	SENSOR_ATTR(freq4_input, S_IRUGO, show_occ_freq_input, NULL, 4),
	SENSOR_ATTR(freq5_input, S_IRUGO, show_occ_freq_input, NULL, 5),
	SENSOR_ATTR(freq6_input, S_IRUGO, show_occ_freq_input, NULL, 6),
	SENSOR_ATTR(freq7_input, S_IRUGO, show_occ_freq_input, NULL, 7),
	SENSOR_ATTR(freq8_input, S_IRUGO, show_occ_freq_input, NULL, 8),
	SENSOR_ATTR(freq9_input, S_IRUGO, show_occ_freq_input, NULL, 9),
	SENSOR_ATTR(freq10_input, S_IRUGO, show_occ_freq_input, NULL, 10),
};

static struct sensor_device_attribute freq_label[] = {
	SENSOR_ATTR(freq1_label, S_IRUGO, show_occ_freq_label, NULL, 1),
	SENSOR_ATTR(freq2_label, S_IRUGO, show_occ_freq_label, NULL, 2),
	SENSOR_ATTR(freq3_label, S_IRUGO, show_occ_freq_label, NULL, 3),
	SENSOR_ATTR(freq4_label, S_IRUGO, show_occ_freq_label, NULL, 4),
	SENSOR_ATTR(freq5_label, S_IRUGO, show_occ_freq_label, NULL, 5),
	SENSOR_ATTR(freq6_label, S_IRUGO, show_occ_freq_label, NULL, 6),
	SENSOR_ATTR(freq7_label, S_IRUGO, show_occ_freq_label, NULL, 7),
	SENSOR_ATTR(freq8_label, S_IRUGO, show_occ_freq_label, NULL, 8),
	SENSOR_ATTR(freq9_label, S_IRUGO, show_occ_freq_label, NULL, 9),
	SENSOR_ATTR(freq10_label, S_IRUGO, show_occ_freq_label, NULL, 10),

};

#define FREQ_UNIT_ATTRS(X)                      \
{	&freq_input[X].dev_attr.attr,           \
	&freq_label[X].dev_attr.attr,          \
	NULL                                    \
}

/* 10-core CPU, occ has 22 freq sensors, more socket, more sensors */
static struct attribute *occ_freq_attr[][3] = {
	FREQ_UNIT_ATTRS(0),
	FREQ_UNIT_ATTRS(1),
	FREQ_UNIT_ATTRS(2),
	FREQ_UNIT_ATTRS(3),
	FREQ_UNIT_ATTRS(4),
	FREQ_UNIT_ATTRS(5),
	FREQ_UNIT_ATTRS(6),
	FREQ_UNIT_ATTRS(7),
	FREQ_UNIT_ATTRS(8),
	FREQ_UNIT_ATTRS(9),
};

static const struct attribute_group occ_freq_attr_group[] = {
	{ .attrs = occ_freq_attr[0] },
	{ .attrs = occ_freq_attr[1] },
	{ .attrs = occ_freq_attr[2] },
	{ .attrs = occ_freq_attr[3] },
	{ .attrs = occ_freq_attr[4] },
	{ .attrs = occ_freq_attr[5] },
	{ .attrs = occ_freq_attr[6] },
	{ .attrs = occ_freq_attr[7] },
	{ .attrs = occ_freq_attr[8] },
	{ .attrs = occ_freq_attr[9] },
};

static struct sensor_device_attribute_2 caps_curr_powercap[] = {
	SENSOR_ATTR_2(caps_curr_powercap, S_IRUGO, show_occ_caps, NULL, 0, 1),
};
static struct sensor_device_attribute_2 caps_curr_powerreading[] = {
	SENSOR_ATTR_2(caps_curr_powerreading, S_IRUGO,
		show_occ_caps, NULL, 1, 1),
};
static struct sensor_device_attribute_2 caps_norm_powercap[] = {
	SENSOR_ATTR_2(caps_norm_powercap, S_IRUGO, show_occ_caps,
		NULL, 2, 1),
};
static struct sensor_device_attribute_2 caps_max_powercap[] = {
	SENSOR_ATTR_2(caps_max_powercap, S_IRUGO, show_occ_caps, NULL, 3, 1),
};
static struct sensor_device_attribute_2 caps_min_powercap[] = {
	SENSOR_ATTR_2(caps_min_powercap, S_IRUGO, show_occ_caps, NULL, 4, 1),
};
static struct sensor_device_attribute_2 caps_user_powerlimit[] = {
	SENSOR_ATTR_2(caps_user_powerlimit, S_IRUGO, show_occ_caps, NULL, 5, 1),
};
#define CAPS_UNIT_ATTRS(X)                      \
{	&caps_curr_powercap[X].dev_attr.attr,           \
	&caps_curr_powerreading[X].dev_attr.attr,           \
	&caps_norm_powercap[X].dev_attr.attr,           \
	&caps_max_powercap[X].dev_attr.attr,           \
	&caps_min_powercap[X].dev_attr.attr,           \
	&caps_user_powerlimit[X].dev_attr.attr,           \
	NULL                                    \
}

/* 10-core CPU, occ has 1 caps sensors */
static struct attribute *occ_caps_attr[][7] = {
	CAPS_UNIT_ATTRS(0),
};
static const struct attribute_group occ_caps_attr_group[] = {
	{ .attrs = occ_caps_attr[0] },
};

static struct sensor_device_attribute power_input[] = {
	SENSOR_ATTR(power1_input, S_IRUGO, show_occ_power_input, NULL, 1),
	SENSOR_ATTR(power2_input, S_IRUGO, show_occ_power_input, NULL, 2),
	SENSOR_ATTR(power3_input, S_IRUGO, show_occ_power_input, NULL, 3),
	SENSOR_ATTR(power4_input, S_IRUGO, show_occ_power_input, NULL, 4),
	SENSOR_ATTR(power5_input, S_IRUGO, show_occ_power_input, NULL, 5),
	SENSOR_ATTR(power6_input, S_IRUGO, show_occ_power_input, NULL, 6),
	SENSOR_ATTR(power7_input, S_IRUGO, show_occ_power_input, NULL, 7),
	SENSOR_ATTR(power8_input, S_IRUGO, show_occ_power_input, NULL, 8),
	SENSOR_ATTR(power9_input, S_IRUGO, show_occ_power_input, NULL, 9),
	SENSOR_ATTR(power10_input, S_IRUGO, show_occ_power_input, NULL, 10),
	SENSOR_ATTR(power11_input, S_IRUGO, show_occ_power_input, NULL, 11),
};

static struct sensor_device_attribute power_label[] = {
	SENSOR_ATTR(power1_label, S_IRUGO, show_occ_power_label, NULL, 1),
	SENSOR_ATTR(power2_label, S_IRUGO, show_occ_power_label, NULL, 2),
	SENSOR_ATTR(power3_label, S_IRUGO, show_occ_power_label, NULL, 3),
	SENSOR_ATTR(power4_label, S_IRUGO, show_occ_power_label, NULL, 4),
	SENSOR_ATTR(power5_label, S_IRUGO, show_occ_power_label, NULL, 5),
	SENSOR_ATTR(power6_label, S_IRUGO, show_occ_power_label, NULL, 6),
	SENSOR_ATTR(power7_label, S_IRUGO, show_occ_power_label, NULL, 7),
	SENSOR_ATTR(power8_label, S_IRUGO, show_occ_power_label, NULL, 8),
	SENSOR_ATTR(power9_label, S_IRUGO, show_occ_power_label, NULL, 9),
	SENSOR_ATTR(power10_label, S_IRUGO, show_occ_power_label, NULL, 10),
	SENSOR_ATTR(power11_label, S_IRUGO, show_occ_power_label, NULL, 11),
};

#define POWER_UNIT_ATTRS(X)                      \
{	&power_input[X].dev_attr.attr,           \
	&power_label[X].dev_attr.attr,          \
	NULL                                    \
}

/* 10-core CPU, occ has 11 power sensors, more socket, more sensors */
static struct attribute *occ_power_attr[][3] = {
	POWER_UNIT_ATTRS(0),
	POWER_UNIT_ATTRS(1),
	POWER_UNIT_ATTRS(2),
	POWER_UNIT_ATTRS(3),
	POWER_UNIT_ATTRS(4),
	POWER_UNIT_ATTRS(5),
	POWER_UNIT_ATTRS(6),
	POWER_UNIT_ATTRS(7),
	POWER_UNIT_ATTRS(8),
	POWER_UNIT_ATTRS(9),
	POWER_UNIT_ATTRS(10),
};

static const struct attribute_group occ_power_attr_group[] = {
	{ .attrs = occ_power_attr[0] },
	{ .attrs = occ_power_attr[1] },
	{ .attrs = occ_power_attr[2] },
	{ .attrs = occ_power_attr[3] },
	{ .attrs = occ_power_attr[4] },
	{ .attrs = occ_power_attr[5] },
	{ .attrs = occ_power_attr[6] },
	{ .attrs = occ_power_attr[7] },
	{ .attrs = occ_power_attr[8] },
	{ .attrs = occ_power_attr[9] },
	{ .attrs = occ_power_attr[10] },
};

static void occ_remove_sysfs_files(struct device *dev)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(occ_temp_attr_group); i++)
		sysfs_remove_group(&dev->kobj, &occ_temp_attr_group[i]);

	for (i = 0; i < ARRAY_SIZE(occ_freq_attr_group); i++)
		sysfs_remove_group(&dev->kobj, &occ_freq_attr_group[i]);

	for (i = 0; i < ARRAY_SIZE(occ_power_attr_group); i++)
		sysfs_remove_group(&dev->kobj, &occ_power_attr_group[i]);

	for (i = 0; i < ARRAY_SIZE(occ_caps_attr_group); i++)
		sysfs_remove_group(&dev->kobj, &occ_caps_attr_group[i]);
}


static int occ_create_sysfs_attribute(struct device *dev)
{
	/* The sensor number varies for different
	 * platform depending on core number. We'd better
	 * create them dynamically
	 */
	struct occ_drv_data *drv_data = dev_get_drvdata(dev);
	int i = 0;
	int num_of_sensors = 0;
	int ret = 0;
	struct occ_response *rsp = NULL;

	/* get sensor number from occ. */
	ret = occ_update_device(dev);
	if (ret != 0) {
		dev_err(dev, "ERROR: cannot get occ sensor data: %d\n", ret);
		return ret;
	}

	rsp = &drv_data->occ_resp;
	if (!rsp->data.blocks)
		return -1;

	/* temp sensors */
	if (rsp->temp_block_id >= 0) {
		num_of_sensors =
			rsp->data.blocks[rsp->temp_block_id].num_of_sensors;
		for (i = 0; i < num_of_sensors; i++) {
			ret = sysfs_create_group(&drv_data->hwmon_dev->kobj,
				&occ_temp_attr_group[i]);
			if (ret) {
				dev_err(dev, "error create temp sysfs entry\n");
				goto error;
			}
		}
	}

	/* freq sensors */
	if (rsp->freq_block_id >= 0) {
		num_of_sensors =
			rsp->data.blocks[rsp->freq_block_id].num_of_sensors;
		for (i = 0; i < num_of_sensors; i++) {
			ret = sysfs_create_group(&drv_data->hwmon_dev->kobj,
				&occ_freq_attr_group[i]);
			if (ret) {
				dev_err(dev, "error create freq sysfs entry\n");
				goto error;
			}
		}
	}

	/* power sensors */
	dev_dbg(dev, "power_block_id: %d\n", rsp->power_block_id);
	if (rsp->power_block_id >= 0) {
		num_of_sensors =
			rsp->data.blocks[rsp->power_block_id].num_of_sensors;
		for (i = 0; i < num_of_sensors; i++) {
			ret = sysfs_create_group(&drv_data->hwmon_dev->kobj,
				&occ_power_attr_group[i]);
			if (ret) {
				dev_err(dev, "error create power sysfs entry\n");
				goto error;
			}
		}
	}

	/* caps sensors */
	dev_dbg(dev, "caps_block_id: %d\n", rsp->caps_block_id);
	if (rsp->caps_block_id >= 0) {
		num_of_sensors =
			rsp->data.blocks[rsp->caps_block_id].num_of_sensors;
		for (i = 0; i < num_of_sensors; i++) {
			ret = sysfs_create_group(&drv_data->hwmon_dev->kobj,
				&occ_caps_attr_group[i]);
			if (ret) {
				dev_err(dev, "error create caps sysfs entry\n");
				goto error;
			}
		}
	}

	return 0;
error:
	occ_remove_sysfs_files(drv_data->hwmon_dev);
	return ret;
}

/* device probe and removal */

#define OCC_I2C_ADDR 0x50
#define OCC_I2C_NAME "occ-i2c"

enum occ_type {
	occ_id,
};

static int occ_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct occ_drv_data *data;
	unsigned long funcs;
	struct device_node *np = dev->of_node;
	int ret = 0;

	data = devm_kzalloc(dev, sizeof(struct occ_drv_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);
	data->sample_time = HZ;

	/* We have several ways to get client address:
	 * 1. i2c_board_info
	 * 2. device table
	 * 3. user space sysfs new_device interface.
	 * Here we use the 3rd method
	 */

	/* Read address from device table. The address is supposed to be
	 * assigned by i2c-core using the address_list, but that does not work.
	 * Have to read from DT explicilty
	 */

	/*
	 * if (of_property_read_u32(np, "reg", &pval)) {
	 *	dev_err(&client->dev, "invalid reg\n");
	 * }
	 * client->addr = pval;
	 */

	/* configure the driver */
	dev_dbg(dev, "occ register hwmon @0x%x\n", client->addr);

	/* create sysfs attributes based on sensor number read from OCC */
	data->hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(data->hwmon_dev))
		return PTR_ERR(data->hwmon_dev);

	ret = occ_create_sysfs_attribute(dev);
	if (ret) {
		hwmon_device_unregister(data->hwmon_dev);
		return ret;
	}

	data->hwmon_dev->parent = dev;

	dev_dbg(dev, "%s: sensor '%s'\n",
		 dev_name(data->hwmon_dev), client->name);

	funcs = i2c_get_functionality(client->adapter);
	dev_dbg(dev, "i2c adaptor supports function: 0x%lx\n", funcs);

	dev_info(dev, "occ i2c driver ready: i2c addr@0x%x\n", client->addr);

	return 0;
}

static int occ_remove(struct i2c_client *client)
{
	struct occ_drv_data *data = i2c_get_clientdata(client);

	/* free allocated sensor memory */
	deinit_occ_resp_buf(&data->occ_resp);

	occ_remove_sysfs_files(data->hwmon_dev);
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

#ifdef CONFIG_PM
static int occ_suspend(struct device *dev)
{
	/* TODO */
	return 0;
}

static int occ_resume(struct device *dev)
{
	/* TODO */
	return 0;
}

static const struct dev_pm_ops occ_dev_pm_ops = {
	.suspend	= occ_suspend,
	.resume		= occ_resume,
};
#define OCC_DEV_PM_OPS (&occ_dev_pm_ops)
#else
#define OCC_DEV_PM_OPS NULL
#endif /* CONFIG_PM */

/* i2c-core uses i2c-detect() to detect device in bellow address list.
 *  If exists, address will be assigned to client.
 * It is also possible to read address from device table.
 */
static const unsigned short normal_i2c[] = {0x50, 0x51, I2C_CLIENT_END };

/* Return 0 if detection is successful, -ENODEV otherwise */
static int occ_detect(struct i2c_client *new_client,
		       struct i2c_board_info *info)
{
	/* i2c-core needs this function to instantiate new device */
	strncpy(info->type, OCC_I2C_NAME, sizeof(OCC_I2C_NAME));
	return 0;
}

static struct i2c_driver occ_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= OCC_I2C_NAME,
		.pm	= OCC_DEV_PM_OPS,
		.of_match_table = i2c_occ_of_match,
	},
	.probe		= occ_probe,
	.remove		= occ_remove,
	.id_table	= occ_ids,
	.address_list	= normal_i2c,
	.detect		= occ_detect,
};

module_i2c_driver(occ_driver);

MODULE_AUTHOR("Li Yi <shliyi@cn.ibm.com>");
MODULE_DESCRIPTION("BMC OCC hwmon driver");
MODULE_LICENSE("GPL");
