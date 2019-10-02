/*
 * Copyright (c) 2019 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77818.h>
#include <linux/power/max77818-battery.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>


#define REG_STATUS			0x00
#define BIT_SMX				BIT(14)
#define BIT_TMX				BIT(13)
#define BIT_VMX				BIT(12)
#define BIT_SMN				BIT(10)
#define BIT_TMN				BIT(9)
#define BIT_VMN				BIT(8)
#define BIT_dSOCi			BIT(7)
#define REG_VALRT_TH		0x01
#define REG_TALRT_TH		0x02
#define REG_SALRT_TH		0x03
#define REG_TEMP			0x08
#define REG_VCELL			0x09
#define REG_AVGVCELL		0x19
#define REG_CONFIG			0x1D
#define	BIT_Aen				BIT(2)
#define REG_VERSION			0x21
#define REG_LEARNCFG		0x28
#define REG_FILTERCFG		0x29
#define REG_MISCCFG			0x2B
#define REG_CGAIN			0x2E
#define REG_RCOMP0			0x38
#define REG_CONFIG2			0xBB
#define BIT_dSOCen			BIT(7)
#define REG_VFOCV			0xFB
#define REG_VFSOC			0xFF

#define MAX77818_FG_DELAY		1000
#define MAX77818_BATTERY_FULL	100
#define MAX77818_BATTERY_LOW	15

#define MAX77818_VERSION_NO	0x20B0
static char *batt_supplied_to[] = {
	"max77818-fuelgauge",
};

struct max77818_chip {
	struct device           *dev;
	struct max77818_dev     *max77818;
	struct regmap			*regmap;

	int						fg_irq;
	struct delayed_work		work;
	struct power_supply		*battery;
	struct power_supply_desc		psy_batt_d;

	/* mutex */
	struct mutex			lock;

	/* alert */
	int alert_threshold;

	/* State Of Connect */
	int ac_online;
	int usb_online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
	/* battery health */
	int health;
	/* battery capacity */
	int capacity_level;

	int lasttime_vcell;
	int lasttime_soc;
	int lasttime_status;

	struct max77818_fg_platform_data	*pdata;
};

static int max77818_fg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max77818_chip *chip =
		power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = chip->capacity_level;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void max77818_fg_get_vcell(struct max77818_chip *max77818_fg)
{
	uint16_t vcell;
	int rc;

	rc = max77818_fg_read(max77818_fg->regmap, REG_VCELL, &vcell);
	if (rc < 0)
		dev_err(max77818_fg->dev, "%s: err %d\n", __func__, rc);
	else {
		pr_info("%s: vcell raw value(0x%4x)\n", __func__, vcell);
		max77818_fg->vcell = (vcell>>3)*625;
	}

}

static void max77818_fg_get_soc(struct max77818_chip *max77818_fg)
{
	uint16_t soc;
	int rc;

	rc = max77818_fg_read(max77818_fg->regmap, REG_VFSOC, &soc);
	pr_info("%s fg read REG_VFSOC %d, rc=%d", __func__, soc, rc);

	if (rc < 0)
		dev_err(max77818_fg->dev, "%s: err %d\n", __func__, rc);
	else
		max77818_fg->soc = (uint16_t)soc >> 8;

	if (max77818_fg->soc > MAX77818_BATTERY_FULL) {
		max77818_fg->soc = MAX77818_BATTERY_FULL;
		max77818_fg->status = POWER_SUPPLY_STATUS_FULL;
		max77818_fg->capacity_level =
			POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		max77818_fg->health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (max77818_fg->soc < MAX77818_BATTERY_LOW) {
		max77818_fg->health = POWER_SUPPLY_HEALTH_DEAD;
		max77818_fg->capacity_level =
			POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else {
		max77818_fg->health = POWER_SUPPLY_HEALTH_GOOD;
		max77818_fg->capacity_level =
			POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
}

static uint16_t max77818_fg_get_version(struct max77818_chip *max77818_fg)
{
	uint16_t version;
	int rc;

	rc = max77818_fg_read(max77818_fg->regmap, REG_VERSION, &version);
	if (rc < 0)
		dev_err(max77818_fg->dev, "%s: err %d\n", __func__, rc);

	return version;
}

static bool max77818_fg_check_status(struct max77818_chip *max77818_fg)
{
	uint16_t data;
	bool ret = false;
	int fg_write_rt;

	/* check if Smn was generated */
	if (max77818_fg_read(max77818_fg->regmap, REG_STATUS, &data) < 0)
		return ret;

	pr_info("%s: status_reg(0x%4x)\n", __func__, data);

	/* minimum SOC threshold exceeded. */
	if (data & BIT_SMN)
		ret = true;

	/* check 1% SOC change happened */
	if (data & BIT_dSOCi) {
		max77818_fg_get_vcell(max77818_fg);
		max77818_fg_get_soc(max77818_fg);
		power_supply_changed(max77818_fg->battery);

		pr_info("max77818_fg_check_status soc changed, SOC=%d, VCELL=%d\n",
				max77818_fg->soc, max77818_fg->vcell);
	}

	/* clear status reg */
	if (!ret) {
		data = data & 0x007F;
		fg_write_rt = max77818_fg_write(max77818_fg->regmap,
			REG_STATUS, data);
		if (fg_write_rt < 0)
			return ret;
	}

	return ret;
}

static void max77818_fg_work(struct work_struct *work)
{
	struct max77818_chip *chip;

	chip = container_of(work, struct max77818_chip, work.work);

	max77818_fg_get_vcell(chip);
	max77818_fg_get_soc(chip);

	if (chip->vcell != chip->lasttime_vcell ||
			chip->soc != chip->lasttime_soc ||
			chip->status !=	chip->lasttime_status) {

		chip->lasttime_vcell = chip->vcell;
		chip->lasttime_soc = chip->soc;

		power_supply_changed(chip->battery);
	}
	schedule_delayed_work(&chip->work, MAX77818_FG_DELAY);
}

static irqreturn_t max77818_fg_irq_thread(int irq, void *irq_data)
{
	struct max77818_chip *fuelgauge = irq_data;
	bool fuel_alerted;

	if (fuelgauge->pdata->soc_alert_threshold >= 0) {
		fuel_alerted = max77818_fg_check_status(fuelgauge);
		pr_info("%s: Fuel-alert %salerted!\n",
				__func__, fuel_alerted ? "" : "NOT ");

		/* schedule_delayed_work(&fuelgauge->work, 0); */
	}

	return IRQ_HANDLED;
}

static enum power_supply_property max77818_fg_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static int max77818_fg_initialize(struct max77818_chip *chip)
{
	uint16_t config, val;
	uint8_t data[2];
	int ret;

	pr_info("%s\n", __func__);

	max77818_fg_get_vcell(chip);
	pr_info("<%s> vcell %d\n", __func__, chip->vcell);

	max77818_fg_get_soc(chip);
	pr_info("<%s> soc %d\n", __func__, chip->soc);

	/* 1. set fuel gauge alert configuration */
	/* SALRT Threshold setting */
	data[0] = chip->pdata->soc_alert_threshold;
	data[1] = 0xff;
	val = (data[1]<<8) | data[0];
	max77818_fg_write(chip->regmap, REG_SALRT_TH, val);

	/* VALRT Threshold setting */
	data[0] = 0x00;
	data[1] = 0xff;
	val = (data[1]<<8) | data[0];
	max77818_fg_write(chip->regmap, REG_VALRT_TH, val);

	/* TALRT Threshold setting */
	data[0] = 0x80;
	data[1] = 0x7f;
	val = (data[1]<<8) | data[0];
	max77818_fg_write(chip->regmap, REG_TALRT_TH, val);

	ret = max77818_fg_read(chip->regmap, REG_CONFIG, &val);
	if (ret < 0)
		return ret;

	/*Enable Alert (Aen = 1) */
	config = val | (0x01<<2);
	ret = max77818_fg_write(chip->regmap, REG_CONFIG, config);
	if (ret < 0)
		return ret;

	/* 2. set SOC 1% change alert */
	ret = max77818_fg_read(chip->regmap, REG_CONFIG2, &val);
	if (ret < 0)
		return ret;
	config = val | BIT_dSOCen;
	ret = max77818_fg_write(chip->regmap, REG_CONFIG2, config);
	if (ret < 0)
		return ret;

	pr_info("%s done\n", __func__);

	return 0;
}

#ifdef CONFIG_OF
static int max77818_fg_parse_dt(struct max77818_chip *fuelgauge)
{
	struct device *dev = fuelgauge->dev;
	struct device_node *np = of_find_node_by_name(NULL, "fuelgauge");
	struct max77818_fg_platform_data *pdata;
	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(pdata == NULL))
		return -ENOMEM;

	/* reset, irq gpio info */
	if (np == NULL) {
		pr_err("%s np NULL\n", __func__);
		return -EINVAL;
	}
	ret |= of_property_read_u32(np, "fuel_alert_soc",
			&pdata->soc_alert_threshold);
	if (ret < 0)
		pr_err("%s error reading pdata->fuel_alert_soc %d\n",
				__func__, ret);

	if (ret < 0)
		return ret;
	fuelgauge->pdata = pdata;
	return 0;

}
#endif

static int max77818_fg_probe(struct platform_device *pdev)
{
	struct max77818_dev *max77818 = dev_get_drvdata(pdev->dev.parent);
	struct max77818_fg_platform_data *pdata =
		dev_get_platdata(max77818->dev);
	struct max77818_chip *chip;
	uint16_t version;
	int ret = 0;
	u8 val;
	struct power_supply_config fg_cfg;

	pr_info("%s: MAX77818 Fuelgauge Driver Loading\n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		pr_err("%s: out of memory\n", __func__);
		pdata = ERR_PTR(-ENOMEM);
		return -ENOMEM;
	}

	mutex_init(&chip->lock);

	chip->dev = &pdev->dev;
	chip->pdata = pdata;
	chip->regmap = max77818->regmap_fuel;

#if defined(CONFIG_OF)
	ret = max77818_fg_parse_dt(chip);
	if (ret < 0) {
		pr_err("%s not found fuelgauge dt! ret[%d]\n",
				__func__, ret);
	}
#else
	pdata = dev_get_platdata(&pdev->dev);
#endif

	version = max77818_fg_get_version(chip);
	dev_info(&pdev->dev, "MAX77818 Fuelgauge Ver 0x%x\n", version);
	if (version != MAX77818_VERSION_NO) {
		ret = -ENODEV;
		goto error;
	}


	chip->psy_batt_d.name     = "battery";
	chip->psy_batt_d.type     = POWER_SUPPLY_TYPE_BATTERY;
	chip->psy_batt_d.get_property = max77818_fg_get_property;
	chip->psy_batt_d.properties   = max77818_fg_battery_props;
	chip->psy_batt_d.num_properties =
		ARRAY_SIZE(max77818_fg_battery_props);
	fg_cfg.drv_data = chip;
	fg_cfg.supplied_to = batt_supplied_to;
	fg_cfg.of_node = max77818->dev->of_node;
	fg_cfg.num_supplicants = ARRAY_SIZE(batt_supplied_to);

	chip->battery =
		devm_power_supply_register(max77818->dev,
				&chip->psy_batt_d,
				&fg_cfg);
	if (IS_ERR(chip->battery)) {
		pr_err("Couldn't register battery rc=%ld\n",
				PTR_ERR(chip->battery));
		goto error;
	}

	chip->fg_irq = regmap_irq_get_virq(max77818->irqc_intsrc,
	  MAX77818_FG_INT);
	dev_info(&pdev->dev, "MAX77818 Fuel-Gauge irq %d\n", chip->fg_irq);

	if (chip->fg_irq > 0) {
		INIT_DELAYED_WORK(&chip->work, max77818_fg_work);
		ret = request_threaded_irq(chip->fg_irq, NULL,
			max77818_fg_irq_thread,
			IRQF_TRIGGER_FALLING, "fuelgauge-irq", chip);
		if (ret) {
			pr_err("%s: Failed to Reqeust IRQ\n", __func__);
			goto error1;
		}
	}

	ret = max77818_fg_initialize(chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error: Initializing fuel-gauge\n");
		goto error2;
	}

	max77818_read(max77818->regmap_pmic, REG_INTSRCMASK, &val);
	pr_info("<%s> intsrc_mask %Xh\n", pdev->name, val);

	return 0;

error2:
	if (chip->fg_irq)
		free_irq(chip->fg_irq, chip);
error1:
	power_supply_unregister(chip->battery);
error:
	kfree(chip);
	return ret;
}

static int max77818_fg_remove(struct platform_device *pdev)
{
	struct max77818_chip *chip = platform_get_drvdata(pdev);

	power_supply_unregister(chip->battery);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM
static int max77818_fg_suspend(struct device *dev)
{
	struct max77818_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int max77818_fg_resume(struct device *dev)
{
	struct max77818_chip *chip = dev_get_drvdata(dev);

	schedule_delayed_work(&chip->work, MAX77818_FG_DELAY);
	return 0;
}
#else
#define max77818_fg_suspend NULL
#define max77818_fg_resume NULL
#endif /* CONFIG_PM */

#if defined(CONFIG_OF)
static const struct of_device_id max77818_fg_dt_ids[] = {
	{ .compatible = "maxim,max77818-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77818_fg_dt_ids);
#endif /* CONFIG_OF */

static SIMPLE_DEV_PM_OPS(max77818_fg_pm_ops, max77818_fg_suspend,
		max77818_fg_resume);

static struct platform_driver max77818_fg_driver = {
	.driver = {
		.name = "max77818-fuelgauge",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &max77818_fg_pm_ops,
#endif
#if defined(CONFIG_OF)
		.of_match_table	= max77818_fg_dt_ids,
#endif /* CONFIG_OF */
	},
	.probe	= max77818_fg_probe,
	.remove	= max77818_fg_remove,
};

static int __init max77818_fg_init(void)
{
	return platform_driver_register(&max77818_fg_driver);
}
module_init(max77818_fg_init);

static void __exit max77818_fg_exit(void)
{
	platform_driver_unregister(&max77818_fg_driver);
}
module_exit(max77818_fg_exit);

MODULE_AUTHOR("opensource@maximintegrated.com ");
MODULE_DESCRIPTION("MAX77818 Fuel Gauge");
MODULE_LICENSE("GPL");

