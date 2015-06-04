/*
 * RTC driver for the Aspeed 24xx SoCs
 *
 * Copyright 2015 IBM Corp.
 *
 * Joel Stanley <joel@jms.id.au >
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/io.h>

struct aspeed_rtc {
	struct rtc_device	*rtc_dev;
	void __iomem		*base;
	spinlock_t		lock;
};

#define RTC_TIME	0x00
#define RTC_YEAR	0x04
#define RTC_CTRL	0x10

#define RTC_UNLOCK	0x02
#define RTC_ENABLE	0x01

static int aspeed_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct aspeed_rtc *rtc = dev_get_drvdata(dev);
	unsigned int cent, year, mon, day, hour, min, sec;
	u32 reg1, reg2;

	do {
		reg2 = readl(rtc->base + RTC_YEAR);
		reg1 = readl(rtc->base + RTC_TIME);
	} while (reg2 != readl(rtc->base + RTC_YEAR));

	day  = (reg1 >> 24) & 0x1f;
	hour = (reg1 >> 16) & 0x1f;
	min  = (reg1 >>  8) & 0x3f;
	sec  = (reg1 >>  0) & 0x3f;
	cent = (reg2 >> 16) & 0x1f;
	year = (reg2 >>  8) & 0x7f;
	mon  = (reg2 >>  0) & 0x3f;

	rtc_time64_to_tm(mktime64(cent*100 + year, mon, day, hour, min, sec),
			 tm);

	return 0;
}

static int aspeed_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct aspeed_rtc *rtc = dev_get_drvdata(dev);
	unsigned long flags;
	u32 reg1, reg2, ctrl;
	int year, cent;

	/* tm_year counts from 1900 */
	cent = (tm->tm_year + 1900) / 100;
	year = tm->tm_year % 100;

	reg1 = (tm->tm_mday << 24) | (tm->tm_hour << 16) | (tm->tm_min << 8) |
		tm->tm_sec;

	reg2 = ((cent & 0x1f) << 16) | ((year & 0x7f) << 8) |
		(tm->tm_mon & 0xf);

	/* TODO: Do we need to lock? */
	spin_lock_irqsave(&rtc->lock, flags);

	ctrl = readl(rtc->base + RTC_CTRL);
	writel(ctrl | RTC_UNLOCK, rtc->base + RTC_CTRL);

	writel(reg1, rtc->base + RTC_TIME);
	writel(reg2, rtc->base + RTC_YEAR);

	writel(ctrl, rtc->base + RTC_CTRL);

	spin_unlock_irqrestore(&rtc->lock, flags);

	return 0;
}

static struct rtc_class_ops aspeed_rtc_ops = {
	.read_time = aspeed_rtc_read_time,
	.set_time = aspeed_rtc_set_time,
};

static int aspeed_rtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_rtc *rtc;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rtc->base))
		return PTR_ERR(rtc->base);

	platform_set_drvdata(pdev, rtc);

	rtc->rtc_dev = devm_rtc_device_register(&pdev->dev, pdev->name,
						&aspeed_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc->rtc_dev)) {
		dev_err(&pdev->dev, "failed to register\n");
		return PTR_ERR(rtc->rtc_dev);
	}

	spin_lock_init(&rtc->lock);

	/* Enable RTC and clear the unlock bit */
	writel(RTC_ENABLE, rtc->base + RTC_CTRL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aspeed_rtc_of_match_table[] = {
	{ .compatible = "aspeed,rtc", },
	{}
};
#endif

static struct platform_driver aspeed_rtc_driver = {
	.driver = {
		.name = "aspeed-rtc",
		.of_match_table = of_match_ptr(aspeed_rtc_of_match_table),
	},
};

module_platform_driver_probe(aspeed_rtc_driver, aspeed_rtc_probe);

MODULE_DESCRIPTION("Aspeed AST24xx RTC driver");
MODULE_AUTHOR("Joel Stanley <joel@jms.id.au>");
MODULE_LICENSE("GPL");
