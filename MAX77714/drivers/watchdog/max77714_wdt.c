/*
 * Maxim MAX77714 Watchdog Driver
 *
 * Copyright (C) 2018 Maxim Integrated. All rights reserved.
 *
 * based on MAX77620
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/max77714.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/watchdog.h>

static bool nowayout = WATCHDOG_NOWAYOUT;

struct max77714_wdt {
	struct device			*dev;
	struct regmap			*rmap;
	struct watchdog_device		wdt_dev;
};

static int max77714_wdt_start(struct watchdog_device *wdt_dev)
{
	struct max77714_wdt *wdt = watchdog_get_drvdata(wdt_dev);

	return regmap_update_bits(wdt->rmap, MAX77714_REG_CNFG_GLBL2,
				  MAX77714_MASK_WDTEN, MAX77714_MASK_WDTEN);
}

static int max77714_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct max77714_wdt *wdt = watchdog_get_drvdata(wdt_dev);

	return regmap_update_bits(wdt->rmap, MAX77714_REG_CNFG_GLBL2,
				  MAX77714_MASK_WDTEN, 0);
}

static int max77714_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct max77714_wdt *wdt = watchdog_get_drvdata(wdt_dev);

	return regmap_update_bits(wdt->rmap, MAX77714_REG_CNFG_GLBL3,
				  MAX77714_MASK_WDTC, 0x1);
}

static int max77714_wdt_set_timeout(struct watchdog_device *wdt_dev,
				    unsigned int timeout)
{
	struct max77714_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	unsigned int wdt_timeout;
	u8 regval;
	int ret;

	switch (timeout) {
	case 0 ... 2:
		regval = MAX77714_SHIFT_TWD_2S;
		wdt_timeout = 2;
		break;

	case 3 ... 16:
		regval = MAX77714_SHIFT_TWD_16S;
		wdt_timeout = 16;
		break;

	case 17 ... 64:
		regval = MAX77714_SHIFT_TWD_64S;
		wdt_timeout = 64;
		break;

	default:
		regval = MAX77714_SHIFT_TWD_128S;
		wdt_timeout = 128;
		break;
	}

	ret = regmap_update_bits(wdt->rmap, MAX77714_REG_CNFG_GLBL3,
				 MAX77714_MASK_WDTC, 0x1);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(wdt->rmap, MAX77714_REG_CNFG_GLBL2,
				 MAX77714_MASK_TWD, regval);
	if (ret < 0)
		return ret;

	wdt_dev->timeout = wdt_timeout;

	return 0;
}

static const struct watchdog_info max77714_wdt_info = {
	.identity = "max77714-watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops max77714_wdt_ops = {
	.start		= max77714_wdt_start,
	.stop		= max77714_wdt_stop,
	.ping		= max77714_wdt_ping,
	.set_timeout	= max77714_wdt_set_timeout,
};

static int max77714_wdt_probe(struct platform_device *pdev)
{
	struct max77714_wdt *wdt;
	struct watchdog_device *wdt_dev;
	unsigned int regval;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->dev = &pdev->dev;

	dev_err(wdt->dev, "%s \n", __func__);
	wdt->rmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!wdt->rmap) {
		dev_err(wdt->dev, "Failed to get parent regmap\n");
		return -ENODEV;
	}

	wdt_dev = &wdt->wdt_dev;
	wdt_dev->info = &max77714_wdt_info;
	wdt_dev->ops = &max77714_wdt_ops;
	wdt_dev->min_timeout = 2;
	wdt_dev->max_timeout = 128;
	wdt_dev->max_hw_heartbeat_ms = 128 * 1000;

	platform_set_drvdata(pdev, wdt);

	/* Enable WD_RST_WK - WDT expire results in a restart */
	ret = regmap_update_bits(wdt->rmap, MAX77714_REG_CNFG2_ONOFF,
				 MAX77714_MASK_WD_RST_WK,
				 MAX77714_MASK_WD_RST_WK);
	if (ret < 0) {
		dev_err(wdt->dev, "Failed to set WD_RST_WK: %d\n", ret);
		return ret;
	}

	/* Set WDT clear in sleep mode */
	ret = regmap_update_bits(wdt->rmap, MAX77714_REG_CNFG_GLBL2,
			 MAX77714_MASK_WDTSLPC, MAX77714_MASK_WDTSLPC);
	if (ret < 0) {
		dev_err(wdt->dev, "Failed to set WDT OFF mode: %d\n", ret);
		return ret;
	}

	/* Check if WDT running and if yes then set flags properly */
	ret = regmap_read(wdt->rmap, MAX77714_REG_CNFG_GLBL2, &regval);
	if (ret < 0) {
		dev_err(wdt->dev, "Failed to read WDT CFG register: %d\n", ret);
		return ret;
	}

	switch (regval & MAX77714_MASK_TWD) {
	case MAX77714_SHIFT_TWD_2S:
		wdt_dev->timeout = 2;
		break;
	case MAX77714_SHIFT_TWD_16S:
		wdt_dev->timeout = 16;
		break;
	case MAX77714_SHIFT_TWD_64S:
		wdt_dev->timeout = 64;
		break;
	default:
		wdt_dev->timeout = 128;
		break;
	}

	if (regval & MAX77714_MASK_WDTEN)
		set_bit(WDOG_HW_RUNNING, &wdt_dev->status);

	watchdog_set_nowayout(wdt_dev, nowayout);
	watchdog_set_drvdata(wdt_dev, wdt);

	ret = watchdog_register_device(wdt_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "watchdog registration failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int max77714_wdt_remove(struct platform_device *pdev)
{
	struct max77714_wdt *wdt = platform_get_drvdata(pdev);

	max77714_wdt_stop(&wdt->wdt_dev);
	watchdog_unregister_device(&wdt->wdt_dev);

	return 0;
}

static struct platform_device_id max77714_wdt_devtype[] = {
	{ .name = "max77714-watchdog", },
	{ },
};

static struct platform_driver max77714_wdt_driver = {
	.driver	= {
		.name	= "max77714-watchdog",
	},
	.probe	= max77714_wdt_probe,
	.remove	= max77714_wdt_remove,
	.id_table = max77714_wdt_devtype,
};

module_platform_driver(max77714_wdt_driver);

MODULE_DESCRIPTION("Max77714 watchdog timer driver");

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
	"(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

MODULE_AUTHOR("Daniel.Jeong <daniel.jeong@maximintegrated.com>");
MODULE_AUTHOR("Maxim LDD <opensource@maximintegrated.com>");
MODULE_LICENSE("GPL v2");
