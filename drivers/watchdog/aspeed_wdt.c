/*
 * Copyright 2015 IBM Corp.
 *
 * Joel Stanley <joel@jms.id.au>
 *
 * Based on the qcom-watchdog driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>

struct aspeed_wdt {
	struct watchdog_device	wdd;
	struct clk		*clk;
	unsigned long		rate;
	struct notifier_block	restart_nb;
	void __iomem		*base;
};

static const struct of_device_id aspeed_wdt_of_table[] = {
	{ .compatible = "aspeed,wdt", },
	{ },
};
MODULE_DEVICE_TABLE(of, aspeed_wdt_of_table);

#define WDT_STATUS		0x00
#define WDT_RELOAD_VALUE	0x04
#define WDT_RESTART		0x08
#define WDT_CTRL		0x0C
#define   WDT_CTRL_RESET_MODE_FULL_CHIP	(0x01 << 5)
#define   WDT_CTRL_RESET_SYSTEM		(0x1  << 1)
#define   WDT_CTRL_ENABLE		(0x1  << 0)

#define	WDT_RESTART_MAGIC	0x4755

static void aspeed_wdt_enable(struct aspeed_wdt *wdt, int count)
{
	u32 ctrl = WDT_CTRL_RESET_MODE_FULL_CHIP | WDT_CTRL_RESET_SYSTEM |
		WDT_CTRL_ENABLE;

	writel(0, wdt->base + WDT_CTRL);
	writel(count, wdt->base + WDT_RELOAD_VALUE);
	writel(WDT_RESTART_MAGIC, wdt->base + WDT_RESTART);
	writel(ctrl, wdt->base + WDT_CTRL);
}

static int aspeed_wdt_start(struct watchdog_device *wdd)
{
	struct aspeed_wdt *wdt = container_of(wdd, struct aspeed_wdt, wdd);
	dev_dbg(wdd->dev, "starting with timeout of %d (rate %lu)\n",
			  wdd->timeout, wdt->rate);
	aspeed_wdt_enable(wdt, wdd->timeout * wdt->rate);
	return 0;
}

static int aspeed_wdt_stop(struct watchdog_device *wdd)
{
	struct aspeed_wdt *wdt = container_of(wdd, struct aspeed_wdt, wdd);

	writel(0, wdt->base + WDT_CTRL);
	return 0;
}

static int aspeed_wdt_ping(struct watchdog_device *wdd)
{
	struct aspeed_wdt *wdt = container_of(wdd, struct aspeed_wdt, wdd);

	dev_dbg(wdd->dev, "ping\n");
	writel(WDT_RESTART_MAGIC, wdt->base + WDT_RESTART);
	return 0;
}

static int aspeed_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	dev_dbg(wdd->dev, "timeout set to %u\n", timeout);
	wdd->timeout = timeout;
	return aspeed_wdt_start(wdd);
}

static int aspeed_wdt_restart(struct notifier_block *nb, unsigned long action,
			      void *data)
{
	struct aspeed_wdt *wdt = container_of(nb,
			struct aspeed_wdt, restart_nb);

	/*
	 * Trigger watchdog bite:
	 *    Setup reload count to be 128ms, and enable WDT.
	 */
	aspeed_wdt_enable(wdt, 128 * wdt->rate / 1000);

	return NOTIFY_DONE;
}

static const struct watchdog_ops aspeed_wdt_ops = {
	.start		= aspeed_wdt_start,
	.stop		= aspeed_wdt_stop,
	.ping		= aspeed_wdt_ping,
	.set_timeout	= aspeed_wdt_set_timeout,
	.owner		= THIS_MODULE,
};

static const struct watchdog_info aspeed_wdt_info = {
	.options	= WDIOF_KEEPALIVEPING
			| WDIOF_MAGICCLOSE
			| WDIOF_SETTIMEOUT,
	.identity	= KBUILD_MODNAME,
};

static int aspeed_wdt_remove(struct platform_device *pdev)
{
	struct aspeed_wdt *wdt = platform_get_drvdata(pdev);

	unregister_restart_handler(&wdt->restart_nb);
	watchdog_unregister_device(&wdt->wdd);
	clk_disable_unprepare(wdt->clk);
	return 0;
}

static int aspeed_wdt_probe(struct platform_device *pdev)
{
	struct aspeed_wdt *wdt;
	struct resource *res;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdt->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	wdt->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(wdt->clk)) {
		dev_err(&pdev->dev, "failed to get input clock\n");
		return PTR_ERR(wdt->clk);
	}

	ret = clk_prepare_enable(wdt->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to setup clock\n");
		goto err;
	}

	/*
	 * We use the clock rate to calculate the max timeout, so ensure it's
	 * not zero to avoid a divide-by-zero exception.
	 *
	 * WATCHDOG_CORE assumes units of seconds, if the WDT is clocked such
	 * that it would bite before a second elapses it's usefulness is
	 * limited.  Bail if this is the case.
	 */
	wdt->rate = clk_get_rate(wdt->clk);
	if (wdt->rate == 0 ||
	    wdt->rate > 0x10000000U) {
		dev_err(&pdev->dev, "invalid clock rate\n");
		ret = -EINVAL;
		goto err;
	}

	wdt->wdd.dev = &pdev->dev;
	wdt->wdd.info = &aspeed_wdt_info;
	wdt->wdd.ops = &aspeed_wdt_ops;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = 0x10000000U / wdt->rate;

	/*
	 * If 'timeout-sec' unspecified in devicetree, assume a 30 second
	 * default, unless the max timeout is less than 30 seconds, then use
	 * the max instead.
	 */
	wdt->wdd.timeout = min(wdt->wdd.max_timeout, 30U);
	watchdog_init_timeout(&wdt->wdd, 0, &pdev->dev);

	ret = watchdog_register_device(&wdt->wdd);
	if (ret) {
		dev_err(&pdev->dev, "failed to register\n");
		goto err;
	}

	/*
	 * WDT restart notifier has priority 0 (use as a last resort)
	 */
	wdt->restart_nb.notifier_call = aspeed_wdt_restart;
	ret = register_restart_handler(&wdt->restart_nb);
	if (ret)
		dev_err(&pdev->dev, "failed to setup restart handler\n");

	dev_info(&pdev->dev, "rate %lu, max timeout %u, timeout %d\n",
		 wdt->rate, wdt->wdd.max_timeout, wdt->wdd.timeout);

	platform_set_drvdata(pdev, wdt);
	return 0;

err:
	clk_disable_unprepare(wdt->clk);
	return ret;
}

static struct platform_driver aspeed_watchdog_driver = {
	.probe = aspeed_wdt_probe,
	.remove = aspeed_wdt_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_wdt_of_table,
	},
};
module_platform_driver(aspeed_watchdog_driver);

MODULE_DESCRIPTION("Aspeed AST23/4xx Watchdog Driver");
