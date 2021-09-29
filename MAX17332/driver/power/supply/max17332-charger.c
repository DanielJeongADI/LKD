// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2021 Maxim Integrated Products, Inc.
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
#include <linux/mfd/max17332.h>
#include <linux/power/max17332-charger.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

static char *chg_supplied_to[] = {
	"max17332-charger",
};

#define __lock(_me)    mutex_lock(&(_me)->lock)
#define __unlock(_me)  mutex_unlock(&(_me)->lock)

static enum power_supply_property max17332_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

static struct device_attribute max17332_chg_attrs[] = {
	MAX17332_CHG_ATTR(max17332_chg_current),
	MAX17332_CHG_ATTR(max17332_chg_en),
	MAX17332_CHG_ATTR(max17332_chg_voltage),
};

static inline int max17332_raw_capacity_to_uamph(struct max17332_charger_data *charger,
												 int cap)
{
	return cap * 5000 / (int)charger->rsense;
}

static int max17332_get_charging_status(struct max17332_charger_data *charger,
										int *status)
{
	*status = POWER_SUPPLY_STATUS_UNKNOWN;
	return 0;
}

static int max17332_set_chg_current(struct max17332_charger_data *charger, unsigned int chg_current)
{
	u16 val, designCap_val;
	u32 current_val;
	int ret = 0, current_lsb, current_max, current_min;

	pr_info("%s : curr(%d)mA\n", __func__, chg_current);

	ret = max17332_read(charger->regmap_nvm, REG_NDESIGNCAP_NVM, &designCap_val);
	if (ret < 0) {
		pr_err("%s : fail to read REG_NDESIGNCAP_NVM\n", __func__);
		return ret;
	}

	current_lsb = (designCap_val & MAX17332_DESIGNCAP_QSCALE) >> MAX17332_DESIGNCAP_QSCALE_POS;
	pr_info("%s : current_lsb (%d)mA\n", __func__, current_lsb);
	switch (current_lsb) {
	case 0:
	case 1:
	case 2:
	case 3:
		current_lsb = 25 << current_lsb;
		break;
	case 4:
		current_lsb = 250;
		break;
	case 5:
		current_lsb = 400;
		break;
	case 6:
	case 7:
		current_lsb = 500 << (current_lsb - 6);
		break;
	default:
		return -EINVAL;
	}
	current_lsb = current_lsb / 10;
	current_max = current_lsb *
		((MAX17332_NICHGCFG1_ROOMCHARGINGI >> MAX17332_NICHGCFG1_ROOMCHARGINGI_POS) + 1);
	current_min = current_lsb;

	pr_info("%s : chg_current (%d)mA\n", __func__, chg_current);
	if (chg_current > current_max || chg_current < current_min) {
		pr_info("%s : wrong current setting (%d)mA\n", __func__, chg_current);
		return -EINVAL;
	}
	current_val = ((chg_current / current_lsb) - 1);

	pr_info("%s : current_val (%d)mA\n", __func__, current_val);
	ret = max17332_read(charger->regmap_nvm, REG_NICHGCFG1_NVM, &val);
	if (ret < 0) {
		pr_err("%s : fail to read REG_NICHGCFG1_NVM\n", __func__);
		return ret;
	}

	val &= ~MAX17332_NICHGCFG1_ROOMCHARGINGI;
	val |= current_val << MAX17332_NICHGCFG1_ROOMCHARGINGI_POS;
	ret = max17332_write(charger->regmap_nvm, REG_NICHGCFG1_NVM, val);
	if (ret < 0) {
		pr_err("%s : fail to write REG_NICHGCFG1_NVM\n", __func__);
		return ret;
	}

	return 0;
}

static ssize_t max17332_get_chg_current(struct max17332_charger_data *charger, char *buf)
{
	u16 val, designCap_val;
	int ret, rc = 0;
	int current_lsb;

	ret = max17332_read(charger->regmap_nvm, REG_NICHGCFG1_NVM, &val);
	if (ret < 0) {
		pr_err("%s : fail to read REG_NICHGCFG1_NVM\n", __func__);
		return ret;
	}

	ret = max17332_read(charger->regmap_nvm, REG_NDESIGNCAP_NVM, &designCap_val);
	if (ret < 0) {
		pr_err("%s : fail to read REG_NDESIGNCAP_NVM\n", __func__);
		return ret;
	}

	current_lsb = (designCap_val & MAX17332_DESIGNCAP_QSCALE) >> MAX17332_DESIGNCAP_QSCALE_POS;

	pr_info("%s : current_lsb (%d)mA\n", __func__, current_lsb);
	switch (current_lsb) {
	case 0:
	case 1:
	case 2:
	case 3:
		current_lsb = 25 << current_lsb;
		break;
	case 4:
		current_lsb = 250;
		break;
	case 5:
		current_lsb = 400;
		break;
	case 6:
	case 7:
		current_lsb = 500 << (current_lsb - 6);
		break;
	default:
		break;
	}
	val = (val & MAX17332_NICHGCFG1_ROOMCHARGINGI) >> MAX17332_NICHGCFG1_ROOMCHARGINGI_POS;
	val = (val + 1) * current_lsb;
	if ((val % 10) != 0)
		rc += snprintf(buf + rc, PAGE_SIZE - rc, "CURRENT : %d.%d mA\n", val/10, val%10);
	else
		rc += snprintf(buf + rc, PAGE_SIZE - rc, "CURRENT : %d mA\n", val/10);

	return rc;
}

static int max17332_set_chg_voltage(struct max17332_charger_data *charger, unsigned int voltage)
{
	u16 val = 0, designCap_val = 0;
	int ret, voltage_val, step_size, center_voltage;
	int voltage_max, voltage_min;

	pr_info("%s : voltage(%d)mV\n", __func__, voltage);

	ret = max17332_read(charger->regmap_nvm, REG_NDESIGNCAP_NVM, &designCap_val);
	if (ret < 0) {
		pr_err("%s : fail to read REG_NDESIGNCAP_NVM\n", __func__);
		return ret;
	}

	if (designCap_val & MAX17332_DESIGNCAP_VSCALE) {
		center_voltage = 3700; // mV
		step_size = 10; // mV
	} else {
		center_voltage = 4200; // mV
		step_size = 5; // mV
	}

	voltage_max = (MAX17332_NVCHGCFG1_ROOMCHARGINGV >> (MAX17332_NVCHGCFG1_ROOMCHARGINGV_POS + 1));
	voltage_max = center_voltage + (voltage_max * step_size);
	voltage_min = (((MAX17332_NVCHGCFG1_ROOMCHARGINGV >> MAX17332_NVCHGCFG1_ROOMCHARGINGV_POS) + 1) >> 1);
	voltage_min = center_voltage - (voltage_min * step_size);

	if (voltage > voltage_max || voltage < voltage_min) {
		pr_info("%s : wrong voltage setting (%d)mV\n", __func__, voltage);
		return -EINVAL;
	}

	// calculate register value
	voltage_val = (voltage - center_voltage) / step_size;

	ret = max17332_read(charger->regmap_nvm, REG_NVCHGCFG1_NVM, &val);
	if (ret < 0) {
		pr_err("%s : fail to read REG_NVCHGCFG1_NVM\n", __func__);
		return ret;
	}

	val &= ~MAX17332_NVCHGCFG1_ROOMCHARGINGV;
	val |= voltage_val << MAX17332_NVCHGCFG1_ROOMCHARGINGV_POS;
	ret = max17332_write(charger->regmap_nvm, REG_NVCHGCFG1_NVM, val);
	if (ret < 0) {
		pr_err("%s : fail to write REG_NVCHGCFG1_NVM\n", __func__);
		return ret;
	}

	return 0;
}

static ssize_t max17332_get_chg_off(struct max17332_charger_data *charger, char *buf)
{
	int rc = 0, ret;
	u16 val = 0;

	// Read CHG FET
	ret = max17332_read(charger->regmap, REG_COMMSTAT, &val);
	if (ret < 0) {
		dev_err(charger->dev, "CommStat Register reading failed (%d)\n", ret);
		return ret;
	}

	if (val & MAX17332_COMMSTAT_CHGOFF)
		rc += snprintf(buf + rc, PAGE_SIZE - rc, "COMM STATUS : CHGOff 1!\n");
	else
		rc += snprintf(buf + rc, PAGE_SIZE - rc, "COMM STATUS : CHGOff 0!\n");

	return rc;
}

static void max17332_set_chg_off(struct max17332_charger_data *charger, bool off)
{
	u16 read_val = 0;
	int ret;

	// Update DCHG FET
	ret = max17332_read(charger->regmap, REG_COMMSTAT, &read_val);
	if (ret < 0) {
		pr_err("%s : fail to read REG_COMMSTAT\n", __func__);
		return;
	}

	read_val &= ~MAX17332_COMMSTAT_CHGOFF;
	read_val |= (off << MAX17332_COMMSTAT_CHGOFF_POS);
	ret = max17332_write(charger->regmap, REG_COMMSTAT, read_val);
	if (ret < 0) {
		pr_err("%s : fail to write REG_COMMSTAT\n", __func__);
		return;
	}

	ret = max17332_read(charger->regmap, REG_COMMSTAT, &read_val);
	if (ret < 0) {
		pr_err("%s : fail to read REG_COMMSTAT\n", __func__);
		return;
	}
	pr_info("%s : reg(0x%04x) = 0x%04x, off(%d)\n", __func__, REG_COMMSTAT, read_val, off);
}

static int max17332_chg_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < (int)ARRAY_SIZE(max17332_chg_attrs); i++) {
		rc = device_create_file(dev, &max17332_chg_attrs[i]);
		if (rc)
			goto create_attrs_failed;
	}
	return rc;

create_attrs_failed:
	dev_err(dev, "%s: failed (%d)\n", __func__, rc);
	while (i--)
		device_remove_file(dev, &max17332_chg_attrs[i]);
	return rc;
}

ssize_t max17332_chg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct max17332_charger_data *charger = power_supply_get_drvdata(psy);
	const ptrdiff_t offset = attr - max17332_chg_attrs;
	int i = 0;

	dev_info(charger->dev, "%s\n", __func__);

	switch (offset) {
	case MAX17332_CHG_CURRENT:
		i = max17332_get_chg_current(charger, buf);
		break;
	case MAX17332_CHG_EN:
		i = max17332_get_chg_off(charger, buf);
		break;
	default:
		return -EINVAL;
	}
	return i;
}

ssize_t max17332_chg_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct max17332_charger_data *charger = power_supply_get_drvdata(psy);
	const ptrdiff_t offset = attr - max17332_chg_attrs;
	int x, ret;

	dev_info(charger->dev, "%s\n", __func__);

	switch (offset) {
	case MAX17332_CHG_CURRENT:
		if (sscanf(buf, "%5d\n", &x) == 1) {
			dev_info(charger->dev, "%s current = %d\n", __func__, x);
			max17332_set_chg_current(charger, x);
		}
		ret = count;
		break;
	case MAX17332_CHG_EN:
		if (sscanf(buf, "%5d\n", &x) == 1) {
			dev_info(charger->dev, "%s en = %d\n", __func__, x);
			max17332_set_chg_off(charger, x);
		}
		ret = count;
		break;
	case MAX17332_CHG_VOLTAGE:
		if (sscanf(buf, "%5d\n", &x) == 1) {
			dev_info(charger->dev, "%s en = %d\n", __func__, x);
			max17332_set_chg_voltage(charger, x);
		}
		ret = count;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int max17332_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	u16 reg;
	struct max17332_charger_data *charger =
		power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = max17332_get_charging_status(charger, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = max17332_read(charger->regmap, REG_DESIGNCAP, &reg);
		if (ret < 0)
			return ret;
		val->intval = max17332_raw_capacity_to_uamph(charger, reg);
		/* Convert to milliamphs */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = max17332_read(charger->regmap, REG_FULLCAPREP, &reg);
		if (ret < 0)
			return ret;
		val->intval = max17332_raw_capacity_to_uamph(charger, reg);
		/* Convert to milliamphs */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = max17332_read(charger->regmap, REG_QH, &reg);
		if (ret < 0)
			return ret;
		/* This register is signed as oppose to other capacity type
		 * registers.
		 */
		val->intval = max17332_raw_capacity_to_uamph(charger,
													 sign_extend32(reg, 15));
		/* Convert to milliamphs */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = max17332_read(charger->regmap, REG_REPCAP, &reg);
		if (ret < 0)
			return ret;
		val->intval = max17332_raw_capacity_to_uamph(charger, reg);
		/* Convert to milliamphs */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = max17332_read(charger->regmap, REG_CYCLES, &reg);
		if (ret < 0)
			return ret;
		val->intval = reg * charger->cycles_reg_lsb_percent / 100;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max17332_charger_set_property(struct power_supply *psy,
								 enum power_supply_property psp,
								 const union power_supply_propval *val)
{
	struct max17332_charger_data *charger =
		power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		pr_info("%s: is it full? %d\n", __func__, val->intval);
		max17332_set_chg_off(charger, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int max17332_chg_property_is_writeable(struct power_supply *psy,
										  enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static int max17332_charger_initialize(struct max17332_charger_data *charger)
{
	return 0;
}

static int max17332_charger_probe(struct platform_device *pdev)
{
	struct max17332_dev *max17332 = dev_get_drvdata(pdev->dev.parent);
	struct max17332_charger_data *charger;
	int ret = 0;
	struct power_supply_config charger_cfg = {};

	pr_info("%s: Max17332 Charger Driver Loading\n", __func__);

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	mutex_init(&charger->lock);

	charger->dev = &pdev->dev;
	charger->max17332 = max17332;
	charger->regmap = max17332->regmap_pmic;
	charger->regmap_nvm = max17332->regmap_nvm;
	charger->rsense = max17332->pdata->rsense;
	charger->cycles_reg_lsb_percent = 25;

	platform_set_drvdata(pdev, charger);
	charger->psy_chg_d.name		= "max17332-charger";
	charger->psy_chg_d.type		= POWER_SUPPLY_TYPE_UNKNOWN;
	charger->psy_chg_d.get_property	= max17332_charger_get_property;
	charger->psy_chg_d.set_property	= max17332_charger_set_property;
	charger->psy_chg_d.property_is_writeable   = max17332_chg_property_is_writeable;
	charger->psy_chg_d.properties	= max17332_charger_props;
	charger->psy_chg_d.num_properties	=
		ARRAY_SIZE(max17332_charger_props);
	charger_cfg.drv_data = charger;
	charger_cfg.supplied_to = chg_supplied_to;
	charger_cfg.of_node = max17332->dev->of_node;
	charger_cfg.num_supplicants = ARRAY_SIZE(chg_supplied_to);

	charger->psy_chg =
		devm_power_supply_register(max17332->dev,
				&charger->psy_chg_d,
				&charger_cfg);
	if (IS_ERR(charger->psy_chg)) {
		pr_err("Couldn't register psy_chg rc=%ld\n",
				PTR_ERR(charger->psy_chg));
		goto error2;
	}

	pr_info("%s: Charger Initialize\n", __func__);
	ret = max17332_charger_initialize(charger);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error: Initializing fuel-gauge\n");
		goto error2;
	}

	ret = max17332_chg_create_attrs(&charger->psy_chg->dev);
	if (ret) {
		dev_err(charger->dev,
			"%s : Failed to create_attrs\n", __func__);
	}

	pr_info("%s: Max17332 Charger Driver Loaded\n", __func__);
	return 0;

error2:
	kfree(charger);
	return ret;
}

static int max17332_charger_remove(struct platform_device *pdev)
{
	struct max17332_charger_data *charger =
		platform_get_drvdata(pdev);

	power_supply_unregister(charger->psy_chg);

	kfree(charger);

	return 0;
}

static const struct platform_device_id max17332_charger_id[] = {
	{ "max17332-charger", 0, },
	{ }
};
MODULE_DEVICE_TABLE(platform, max17332_charger_id);

static struct platform_driver max17332_charger_driver = {
	.driver = {
		.name = "max17332-charger",
	},
	.probe = max17332_charger_probe,
	.remove = max17332_charger_remove,
	.id_table = max17332_charger_id,
};
module_platform_driver(max17332_charger_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("opensource@maximintegrated.com ");
MODULE_DESCRIPTION("MAX17332 Charger");
MODULE_VERSION("1.2");
