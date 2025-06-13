// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * apml_alertl.c - Alert_L driver for AMD APML devices
 *
 * Copyright (C) 2025 Advanced Micro Devices, Inc.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include <linux/i3c/device.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of.h>

#include "apml_alertl.h"

#define DRIVER_NAME "apml_alertl"

#define RAS_STATUS_REG		0x4C
#define RMI_STATUS_REG		0x2
#define TSI_STATUS_REG		0x2
#define RAS_ALERT_STATUS	BIT(1)
#define RAS_ALERT_ASYNC		BIT(3)

#define MAX_SOC_LEN		11
#define MAX_ERR_LEN		18

/* SBRMI and SBTSI static address for socket 0 and 1 */
#define RMI_SOCK_0_DIE_0	0x3c
#define TSI_SOCK_0_DIE_0	0x4c
#define RMI_SOCK_1_DIE_0	0x38
#define TSI_SOCK_1_DIE_0	0x48

MODULE_ALIAS("apml_alertl:" DRIVER_NAME);

/* Map static address to socket and die index */
static u8 static_addr_to_socket(u8 static_addr)
{
	/*
	 * [3:0] = Socket Index
	 * [7:4] = die Index
	 * Mapping:
	 * 0x3c, 0x4c	-> Socket 0, die 0,
	 * 0x38, 0x48	-> Socket 1, die 0,
	 */
	switch (static_addr) {
	case RMI_SOCK_0_DIE_0:
	case TSI_SOCK_0_DIE_0:
		return 0;
	case RMI_SOCK_1_DIE_0:
	case TSI_SOCK_1_DIE_0:
		return 1;
	default:
		return 0xFF;
	}
}

/* Send a uevent to userspace for an APML alert */
static int send_uevent(u8 static_address, u32 alert_src, struct device *dev)
{
	u8 soc_die_num;
	char sock[MAX_SOC_LEN];
	char src[MAX_ERR_LEN];
	char *alert_source[] = { sock, src, NULL };

	soc_die_num = static_addr_to_socket(static_address);
	if (soc_die_num == 0xFF)
		return -ENODEV;

	snprintf(sock, sizeof(sock), "Socket=0x%x", soc_die_num);
	snprintf(src, sizeof(src), "Source=0x%x", alert_src);

	dev_dbg(dev, "Sending uevent: Sock:0x%x Src:0x%x\n",
		soc_die_num, alert_src);
	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, alert_source);
	return 0;
}

/* Process and handle TSI alerts for all TSI devices */
static void handle_tsi_alerts(struct apml_alertl_data *oob_adata)
{
	struct device *dev = oob_adata->dev;
	struct apml_message msg = { 0 };
	int temp_status, ret, i;

	for (i = 0; i < oob_adata->num_of_tsi_devs; i++) {
		temp_status = 0;
		if (!oob_adata->tsi_dev[i] || !oob_adata->tsi_dev[i]->regmap) {
			dev_dbg(dev,
				"TSI device at index %d is NULL or regmap missing\n",
				i);
			continue;
		}

		/* Read TSI Status register to identify the RAS error */
		msg.data_in.reg_in[REG_OFF_INDEX] = TSI_STATUS_REG;

		mutex_lock(&oob_adata->tsi_dev[i]->lock);
		ret = regmap_read(oob_adata->tsi_dev[i]->regmap,
				  msg.data_in.reg_in[REG_OFF_INDEX],
				  &temp_status);
		mutex_unlock(&oob_adata->tsi_dev[i]->lock);

		if (ret < 0) {
			dev_dbg(dev,
				"Failed to read temperature status of TSI device index %d\n",
				i);
			continue;
		}

		if (!temp_status)
			continue;

		ret = send_uevent(oob_adata->tsi_dev[i]->dev_static_addr,
				  temp_status << 24, dev);
		if (ret)
			dev_dbg(dev,
				"Failed to send uevent for temperature alert TSI device index %d Err: %d\n",
				i, ret);
	}
}

/* Process and handle RMI alerts for all RMI devices */
static void handle_rmi_alerts(struct apml_alertl_data *oob_adata)
{
	struct device *dev = oob_adata->dev;
	struct apml_message msg = { 0 };
	int ras_status, ret, i;

	for (i = 0; i < oob_adata->num_of_rmi_devs; i++) {
		ras_status = 0;
		if (!oob_adata->rmi_dev[i] || !oob_adata->rmi_dev[i]->regmap) {
			dev_dbg(dev,
				"RMI device at index %d is NULL or regmap missing\n",
				i);
			continue;
	}

	/* Read RAS Status register to identify the RAS error */
	msg.data_in.reg_in[REG_OFF_INDEX] = RAS_STATUS_REG;

	mutex_lock(&oob_adata->rmi_dev[i]->lock);
	ret = regmap_read(oob_adata->rmi_dev[i]->regmap,
			  msg.data_in.reg_in[REG_OFF_INDEX],
			  &ras_status);
	mutex_unlock(&oob_adata->rmi_dev[i]->lock);

	if (ret < 0) {
		dev_dbg(dev, "Failed to read RAS status of RMI device index %d\n", i);
		continue;
	}

	if (!ras_status)
		continue;

	ret = send_uevent(oob_adata->rmi_dev[i]->dev_static_addr, ras_status, dev);
	if (ret)
		dev_dbg(dev,
			"Failed to send uevent for RAS alert for device %d Err: %d\n",
			i, ret);

	/* Clear the RMI Status and RAS Status register 0x4C */
	mutex_lock(&oob_adata->rmi_dev[i]->lock);
	msg.data_in.reg_in[REG_OFF_INDEX] = RAS_STATUS_REG;
	ret = regmap_write(oob_adata->rmi_dev[i]->regmap,
			   msg.data_in.reg_in[REG_OFF_INDEX],
			   ras_status);
	if (ret < 0)
		dev_dbg(dev,
			"Could not clear RAS status register for device %d\n",
			i);

	msg.data_in.reg_in[REG_OFF_INDEX] = RMI_STATUS_REG;
	ret = regmap_write(oob_adata->rmi_dev[i]->regmap,
			   msg.data_in.reg_in[REG_OFF_INDEX],
			   RAS_ALERT_ASYNC);
	mutex_unlock(&oob_adata->rmi_dev[i]->lock);
	if (ret < 0)
		dev_dbg(dev,
			"Could not clear RMI status register at device %d\n",
			i);
	}
}

/* Handles Alert_L interrupts by delegating to TSI and RMI alert handlers */
static irqreturn_t alert_l_irq_thread_handler(int irq, void *dev_id)
{
	struct apml_alertl_data *oob_adata = (struct apml_alertl_data *)dev_id;
	struct device *dev;

	dev = oob_adata->dev;

	handle_tsi_alerts(oob_adata);
	handle_rmi_alerts(oob_adata);

	return IRQ_HANDLED;
}

/* Retrieve APML device from device tree */
static void *get_apml_dev_byphandle(struct device_node *dnode,
				    const char *phandle_name,
				    int index)
{
	struct device_node *d_node;
	struct device *dev;
	void *apml_dev;

	if (!phandle_name || !dnode)
		return NULL;

	d_node = of_parse_phandle(dnode, phandle_name, index);
	if (IS_ERR_OR_NULL(d_node)) {
		pr_err("Failed to parse phandle '%s' at index %d\n",
		       phandle_name, index);
		return NULL;
	}

	if (strcmp(phandle_name, "sbrmi") == 0) {
		dev = bus_find_device(&i3c_bus_type, NULL, d_node, sbrmi_match_i3c);
		if (!dev) {
			dev = bus_find_device(&i2c_bus_type, NULL, d_node, sbrmi_match_i2c);
			if (IS_ERR_OR_NULL(dev)) {
				of_node_put(d_node);
				return NULL;
			}
		}
	}  else if (strcmp(phandle_name, "sbtsi") == 0) {
		dev = bus_find_device(&i3c_bus_type, NULL, d_node, sbtsi_match_i3c);
		if (!dev) {
			dev = bus_find_device(&i2c_bus_type, NULL, d_node, sbtsi_match_i2c);
			if (IS_ERR_OR_NULL(dev)) {
				of_node_put(d_node);
				return NULL;
			}
		}
	}

	of_node_put(d_node);
	apml_dev = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(apml_dev))
		return NULL;

	return apml_dev;
}

static int apml_alertl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dnode = dev->of_node;
	struct apml_sbrmi_device **rmi_dev;
	struct apml_sbtsi_device **tsi_dev;
	struct apml_alertl_data *oob_alert;
	struct gpio_desc *alertl_gpiod;
	char *irq_name;
	u32 irq_num;
	u8 socket_num;
	int ret, i;

	/* Allocate memory to oob_alert_data structure */
	oob_alert = devm_kzalloc(dev, sizeof(struct apml_alertl_data),
				 GFP_KERNEL);
	if (!oob_alert)
		return -ENOMEM;

	/* identify the number of devices associated with each RMI alert */
	oob_alert->num_of_rmi_devs = of_property_count_elems_of_size(dnode, "sbrmi",
								     sizeof(phandle));

	/* identify the number of devices associated with each TSI alert */
	oob_alert->num_of_tsi_devs = of_property_count_elems_of_size(dnode, "sbtsi",
								     sizeof(phandle));

	/* Allocate memory as per the number of RMI devices */
	rmi_dev = devm_kzalloc(dev, oob_alert->num_of_rmi_devs * sizeof(struct apml_sbrmi_device),
			       GFP_KERNEL);
	if (!rmi_dev)
		return -ENOMEM;
	oob_alert->rmi_dev = rmi_dev;

	/* Allocate memory as per the number of TSI devices */
	tsi_dev = devm_kzalloc(dev, oob_alert->num_of_tsi_devs * sizeof(struct apml_sbtsi_device),
			       GFP_KERNEL);
	if (!tsi_dev)
		return -ENOMEM;

	oob_alert->tsi_dev = tsi_dev;
	oob_alert->dev = dev;

	/*
	 * For each of the Alerts get the device associated
	 * Currently the ALert_L driver identification is only supported
	 * over I3C. We can add property in dts to identify the bus type
	 */
	for (i = 0; i < oob_alert->num_of_rmi_devs; i++) {
		rmi_dev[i] = get_apml_dev_byphandle(pdev->dev.of_node, "sbrmi", i);
		if (!rmi_dev[i]) {
			dev_err(dev, "RMI device %d not found\n", i);
			return -ENODEV;
		}
	}

	for (i = 0; i < oob_alert->num_of_tsi_devs; i++) {
		tsi_dev[i] = get_apml_dev_byphandle(pdev->dev.of_node, "sbtsi", i);
		if (!tsi_dev[i]) {
			dev_err(dev, "TSI device %d not found\n", i);
			return -ENODEV;
		}
	}

	/* Get the alert_l gpios, irq_number for the GPIO and register ISR*/
	alertl_gpiod = devm_gpiod_get(dev, NULL, GPIOD_IN);
	if (IS_ERR(alertl_gpiod)) {
		dev_err(&pdev->dev, "Unable to retrieve gpio\n");
		return PTR_ERR(alertl_gpiod);
	}

	irq_num = gpiod_to_irq(alertl_gpiod);
	if (irq_num < 0) {
		dev_err(dev, "No corresponding IRQ for GPIO, error: %d\n", irq_num);
		return irq_num;
	}

	if (oob_alert->num_of_rmi_devs > 0 && oob_alert->rmi_dev[0])
		socket_num = static_addr_to_socket(oob_alert->rmi_dev[0]->dev_static_addr);
	else if (oob_alert->num_of_tsi_devs > 0 && oob_alert->tsi_dev[0])
		socket_num = static_addr_to_socket(oob_alert->tsi_dev[0]->dev_static_addr);

	irq_name = devm_kasprintf(dev, GFP_KERNEL, "apml_irq%u", socket_num);
	if (!irq_name) {
		dev_dbg(dev, "Failed to allocate IRQ name\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "Register IRQ:%u\n", irq_num);
	ret = devm_request_threaded_irq(dev, irq_num,
					NULL,
					(void *)alert_l_irq_thread_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					irq_name, oob_alert);
	if (ret) {
		dev_dbg(dev, "Cannot register IRQ:%u\n", irq_num);
		return ret;
	}

	/* Set the platform data to pdev */
	platform_set_drvdata(pdev, oob_alert);

	return 0;
}

static int apml_alertl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id apml_alertl_dt_ids[] = {
	{.compatible = "apml-alertl", },
	{},
};
MODULE_DEVICE_TABLE(of, apml_alertl_dt_ids);

static struct platform_driver apml_alertl_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(apml_alertl_dt_ids),
	},
	.probe		= apml_alertl_probe,
	.remove		= apml_alertl_remove,
};

module_platform_driver(apml_alertl_driver);

MODULE_AUTHOR("Akshay Gupta <akshay.gupta@amd.com>");
MODULE_AUTHOR("Naveenkrishna Chatradhi <naveenkrishna.chatradhi@amd.com>");
MODULE_DESCRIPTION("AMD APML ALERT_L Driver");
MODULE_LICENSE("GPL");
