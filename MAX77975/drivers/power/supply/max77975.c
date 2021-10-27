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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/power_supply.h>
#include <linux/power/max77975.h>

#define M2SH	__CONST_FFS

#define __lock(_charger)    mutex_lock(&(_charger)->lock)
#define __unlock(_charger)  mutex_unlock(&(_charger)->lock)

static const struct regmap_config max77975_charger_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const u8 *charger_regs[] = {
	[ID_MAX77975] = max77975_regs,
	[ID_MAX77985] = max77975_regs,
};

/*******************************************************************************
 * Charger IO
 ******************************************************************************/
int max77975_read(struct regmap *regmap, u8 addr, u8 *val)
{
	unsigned int buf = 0;
	int rc = regmap_read(regmap, (unsigned int)addr, &buf);

	if (!IS_ERR_VALUE(rc))
		*val = (u8)buf;
	return rc;
}

int max77975_write(struct regmap *regmap, u8 addr, u8 val)
{
	unsigned int buf = (unsigned int)val;

	return regmap_write(regmap, (unsigned int)addr, buf);
}

int max77975_bulk_read(struct regmap *regmap, u8 addr, u8 *dst, u16 len)
{
	return regmap_bulk_read(regmap, (unsigned int)addr, dst, (size_t)len);
}

int max77975_bulk_write(struct regmap *regmap, u8 addr, const u8 *src, u16 len)
{
	return regmap_bulk_write(regmap, (unsigned int)addr, src, (size_t)len);
}

/* charger API function */
static int max77975_charger_unlock(struct max77975_charger *charger)
{
	int rc;

	rc = regmap_update_bits(charger->regmap, charger->regs[REG_CHG_CNFG_06], BIT_CHGPROT,
			BIT_CHGPROT);

	if (IS_ERR_VALUE(rc)) {
		pr_err("%s: failed to unlock [%d]\n", __func__, rc);
		goto out;
	}

out:
	return rc;
}

static int max77975_charger_set_input_current(struct max77975_charger
	*charger,
	int input_current)
{
	int curr_step = 50;
	u8 reg_data = 0;

	/* unit mA */
	if (input_current > 100) {
		reg_data = (input_current / curr_step);
		reg_data -= 1;
	}
	pr_info("%s: reg_data(0x%02x), charging current(%d)\n",
			__func__, reg_data, input_current);

	if (charger->driver_data == ID_MAX77975) {
		return regmap_update_bits(charger->regmap, charger->regs[REG_CHG_CNFG_09],
				BIT_CHGIN_ILIM_MAX77975, reg_data);
	} else {
		return regmap_update_bits(charger->regmap, charger->regs[REG_CHG_CNFG_09],
				BIT_CHGIN_ILIM_MAX77985, reg_data);
	}
}

static int max77975_charger_get_input_current(struct max77975_charger
	*charger)
{
	u8 reg_data = 0;
	int curr_step = 50;
	int get_current;

	pr_info("max77975 charger get input current");

	max77975_read(charger->regmap, charger->regs[REG_CHG_CNFG_09], &reg_data);
	if (charger->driver_data == ID_MAX77975)
		reg_data = reg_data & BIT_CHGIN_ILIM_MAX77975;
	else
		reg_data = reg_data & BIT_CHGIN_ILIM_MAX77985;

	if (reg_data < 2)
		get_current = 100;	/* 100mA */
	else
		get_current = (reg_data + 1)*curr_step;

	return get_current;
}

static int max77975_charger_set_charge_current(struct max77975_charger
	*charger,
	int fast_charging_current)
{
	int curr_step = 50;
	u8 reg_data = 0;
	int rc;

	/* unit mA */
	if (fast_charging_current)
		reg_data = (fast_charging_current / curr_step);

	rc = regmap_update_bits(charger->regmap, REG_CHG_CNFG_02,
			BIT_CHG_CC, reg_data);
	pr_info("%s: reg_data(0x%02x), charging current(%d)\n",
			__func__, reg_data, fast_charging_current);

	return rc;
}

static int max77975_charger_get_charge_current(struct max77975_charger
	*charger)
{
	struct regmap *regmap = charger->regmap;

	u8 reg_data = 0;
	int get_current;

	max77975_read(regmap, charger->regs[REG_CHG_CNFG_02], &reg_data);

	if ((reg_data & BIT_CHG_CC) < 2)
		get_current = 100;	/* 100mA */
	else
		get_current = (reg_data & BIT_CHG_CC)*50;

	pr_info("%s: reg_data(0x%02x), get_current(%d)\n",
			__func__, reg_data, get_current);

	return get_current;
}

static int max77975_charger_set_topoff_current(struct max77975_charger
	*charger,
	int termination_current,
	int termination_time)
{
	u8 reg_data;

	/* termination_current (mA) */
	reg_data = SET_TO_ITH(termination_current);

	/* termination_time (min) */
	reg_data |= ((termination_time / 10) << M2SH(BIT_TO_TIME));
	pr_info("%s: reg_data(0x%02x), topoff(%d), time(%d)\n",
			__func__, reg_data, termination_current,
			termination_time);
	return regmap_update_bits(charger->regmap, charger->regs[REG_CHG_CNFG_03],
			BIT_TO_ITH | BIT_TO_TIME, reg_data);
}

static int max77975_charger_set_enable(struct max77975_charger *charger,
	int en)
{
	return regmap_update_bits(charger->regmap,
		charger->regs[REG_CHG_CNFG_00], BIT_MODE_CHARGER, !!en);
}

static bool max77975_charger_present_input(struct max77975_charger
	*charger)
{
	u8 chg_int_ok = 0;
	int rc;

	rc = max77975_read(charger->regmap, charger->regs[REG_CHG_INT_OK], &chg_int_ok);
	if (IS_ERR_VALUE(rc))
		return false;

	if ((chg_int_ok & BIT_CHGIN) == BIT_CHGIN)
		return true;

	/* check whether charging or not in the UVLO condition */
	if (((charger->details_0 & BIT_CHGIN_DTLS) == 0) &&
			(((charger->details_1 & BIT_CHG_DTLS) ==
					CHG_DTLS_FASTCHARGE_CC) ||
			 ((charger->details_1 & BIT_CHG_DTLS) ==
					CHG_DTLS_FASTCHARGE_CV))) {
		return true;
	} else {
		return false;
	}
}

static int max77975_charger_exit_dev(struct max77975_charger *charger)
{
	struct max77975_charger_platform_data *pdata = charger->pdata;
	int rc;

	rc = max77975_charger_set_enable(charger, false);
	if (IS_ERR_VALUE(rc)) {
		pr_err("CHG_CNFG_00 write error [%d]\n", rc);
		return rc;
	}

	rc = max77975_charger_set_charge_current(charger,
		pdata->fast_charge_current);

	return rc;
}

static int max77975_charger_init_dev(struct max77975_charger *charger)
{
	int rc;

	/* charger enable */
	rc = max77975_charger_set_enable(charger, true);

	return rc;
}


static void max77975_charger_initialize(struct max77975_charger	*charger)
{
	struct max77975_charger_platform_data *pdata = charger->pdata;
	int rc;
	u8 val, temp_val;

	pr_info("%s\n", __func__);

	/* 	0b001: PASS1
	 *	0b010: PASS2
	 *	0b011: PASS3
	 *	0b100: PASS4
	 */
	charger->chip_revision = temp_val & BIT_REVISION;
	pr_info("%s: Charger revision is 0x%x\n", __func__, charger->chip_revision);
	if (charger->driver_data == ID_MAX77985) {
		/*	0b1010: A Variant
		 *	0b1011: B Variant
		 */
		charger->chip_version = (temp_val & BIT_VERSION) >> M2SH(BIT_VERSION);
		pr_info("%s: Charger version is 0x%x\n", __func__, charger->chip_version);
	}

	/* interrupt mask - if you want to enable some bits,
	 *	you should clear them
	 */
	val  = 0;
	val |= BIT_AICL;
	val |= BIT_CHGIN;
	val |= BIT_INLIM;
	val |= BIT_CHG;
	val |= BIT_BAT;
	val |= BIT_DISQBAT;
	val |= BIT_BYP;

	rc = max77975_write(charger->regmap, charger->regs[REG_CHG_INT_MASK], val);
	if (IS_ERR_VALUE(rc)) {
		pr_err("CHG_INT_MASK write error [%d]\n", rc);
		goto out;
	}

	pr_info("%s: CHG_INT_MASK is set to 0x%x\n", __func__, val);

	rc = max77975_read(charger->regmap, charger->regs[REG_CHG_INT_MASK], &temp_val);
	if (IS_ERR_VALUE(rc)) {
		pr_err("CHG_INT_MASK read error [%d]\n", rc);
		goto out;
	}
	pr_info("%s: CHG_INT_MASK is now 0x%x\n", __func__, temp_val);

	/* unlock charger register */
	rc = max77975_charger_unlock(charger);
	if (IS_ERR_VALUE(rc))
		goto out;

	/* charge current (mA) */
	rc = max77975_charger_set_charge_current(charger,
		pdata->fast_charge_current);
	if (IS_ERR_VALUE(rc))
		goto out;

	/* input current limit (mA) */
	rc = max77975_charger_set_input_current(charger,
		pdata->input_current_limit);
	if (IS_ERR_VALUE(rc))
		goto out;

	/* topoff current(mA) and topoff timer(min) */
	rc = max77975_charger_set_topoff_current(charger,
		pdata->topoff_current, pdata->topoff_timer);
	if (IS_ERR_VALUE(rc))
		goto out;

	/* charge restart threshold(mV)  */
	val = pdata->restart_threshold < 100 ? 0x00 :
			pdata->restart_threshold <= 200 ?
			(int)DIV_ROUND_UP(pdata->restart_threshold - 100, 50) :	0x03;

	/* Fast-charge timer(hr) */
	if (charger->driver_data == ID_MAX77975) {
		temp_val = pdata->fast_charge_timer == 0 ? 0x00 :
			pdata->fast_charge_timer < 3 ? 0x01 :
			pdata->fast_charge_timer <= 8 ? (pdata->fast_charge_timer - 2) :
			pdata->fast_charge_timer <= 10 ? 0x0A : 0x00;
	} else {
		temp_val = pdata->fast_charge_timer == 0 ? 0x00 :
			pdata->fast_charge_timer < 3 ? 0x01 :
			pdata->fast_charge_timer <= 8 ?
			(pdata->fast_charge_timer - 2) : 0x00;
	}

	val = val<<M2SH(BIT_CHG_RSTRT) | temp_val<<M2SH(BIT_FCHGTIME);

	rc = regmap_update_bits(charger->regmap, charger->regs[REG_CHG_CNFG_01],
			(BIT_CHG_RSTRT | BIT_FCHGTIME), val);
	if (IS_ERR_VALUE(rc))
		goto out;

	/* charge termination voltage (mV) */
	if (charger->driver_data == ID_MAX77975) {
		val = pdata->termination_voltage < 4150 ? 0x00 :
			(int)DIV_ROUND_UP(
				pdata->termination_voltage - 4150, 10);
	} else {
		if (charger->chip_version == VERSION_A) {
			val = pdata->termination_voltage < 4150 ? 0x00 :
				(int)DIV_ROUND_UP(
					(pdata->termination_voltage - 4150) * 10, 125);
		} else {
			val = pdata->termination_voltage < 3500 ? 0x00 :
				(int)DIV_ROUND_UP(
					(pdata->termination_voltage - 3500) * 10, 250);
		}
	}

	rc = regmap_update_bits(charger->regmap,
		charger->regs[REG_CHG_CNFG_04], BIT_CHG_CV_PRM,
		val << M2SH(BIT_CHG_CV_PRM));
	if (IS_ERR_VALUE(rc))
		goto out;

	rc = max77975_read(charger->regmap, charger->regs[REG_CHIP_ID], &temp_val);
	if (IS_ERR_VALUE(rc))
		goto out;
	charger->chip_id = temp_val;

	rc = max77975_read(charger->regmap, charger->regs[REG_CHIP_REVISION], &temp_val);
	if (IS_ERR_VALUE(rc))
		goto out;

out:
	return;
}

static enum power_supply_property max77975_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

struct max77975_charger_status_map {
	int health, status, charge_type;
};

static struct max77975_charger_status_map max77975_charger_status_map[] = {
#define STATUS_MAP(_chg_dtls, _health, _status, _charge_type) \
	[CHG_DTLS_##_chg_dtls] = {\
		.health = POWER_SUPPLY_HEALTH_##_health,\
		.status = POWER_SUPPLY_STATUS_##_status,\
		.charge_type = POWER_SUPPLY_CHARGE_TYPE_##_charge_type,\
	}
	/* chg_details_xx, health, status, charge_type */
	STATUS_MAP(PREQUAL, GOOD, CHARGING, TRICKLE),
	STATUS_MAP(FASTCHARGE_CC, GOOD, CHARGING, FAST),
	STATUS_MAP(FASTCHARGE_CV, GOOD, CHARGING, FAST),
	STATUS_MAP(TOPOFF, GOOD, CHARGING, FAST),
	STATUS_MAP(DONE, GOOD, FULL, NONE),
	STATUS_MAP(OFF_TIMER_FAULT, SAFETY_TIMER_EXPIRE,
		NOT_CHARGING, NONE),
	STATUS_MAP(OFF_SUSPEND, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(OFF_INPUT_INVALID, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(OFF_JUCTION_TEMP, UNKNOWN, NOT_CHARGING, UNKNOWN),
	STATUS_MAP(OFF_WDT_EXPIRED, WATCHDOG_TIMER_EXPIRE,
		NOT_CHARGING, UNKNOWN),
	STATUS_MAP(CHG_CC_CV_REDUCED, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(BATT_REMOVED, UNKNOWN, NOT_CHARGING, NONE),
	STATUS_MAP(SUSPEND_PIN, UNKNOWN, NOT_CHARGING, NONE),
};

static irqreturn_t max77975_charger_irq(int irq, void *data)
{
	struct max77975_charger *charger = data;
	u8 reg_details[3];

	max77975_bulk_read(charger->regmap, charger->regs[REG_CHG_DETAILS_00], reg_details, 3);
	pr_info("%s: chg_dtls[0]=0x%x, [1]=0x%x, [2]=0x%x\n",
		__func__, reg_details[0], reg_details[1], reg_details[2]);

	charger->details_0 = reg_details[0];
	charger->details_1 = reg_details[1];
	charger->details_2 = reg_details[2];

	schedule_delayed_work(&charger->irq_work, IRQ_WORK_DELAY);
	return IRQ_HANDLED;
}

static void max77975_charger_irq_work(struct work_struct *work)
{
	struct max77975_charger *charger =
		container_of(work, struct max77975_charger, irq_work.work);

	u8 val, chg_int, chg_int_ok;
	u8 chg_details[3];
	bool aicl_mode, disqbat;
	bool chg_inlim, chg_input;

	chg_details[0] = charger->details_0;
	chg_details[1] = charger->details_1;
	chg_details[2] = charger->details_2;

	__lock(charger);

	max77975_read(charger->regmap, charger->regs[REG_CHG_INT], &chg_int);
	max77975_read(charger->regmap, charger->regs[REG_CHG_INT_OK], &chg_int_ok);

	if (chg_int & BIT_AICL) {
		aicl_mode = (chg_int_ok & BIT_AICL) ? false : true;
		pr_debug("CHG_IRQ_AICL_I: AICL_DTLS %s\n",
			aicl_mode ? "Not in AICL mode" : "In AICL mode");
	}

	if (chg_int & BIT_CHGIN) {
		chg_input = max77975_charger_present_input(charger);

		if (chg_input) {
			/* charger insert */
			max77975_charger_init_dev(charger);
		} else {
			/* charger remove */
			max77975_charger_exit_dev(charger);
		}
		pr_debug("CHG_IRQ_CHGIN_I: Charger input %s\n",
			chg_input ? "inserted" : "removed");
	}

	if (chg_int & BIT_INLIM) {
		chg_inlim = (chg_int_ok & BIT_INLIM) ? false : true;
		pr_debug("CHG_IRQ_INLIM_I: The CHGIN input current %s\n",
			chg_inlim ?
			"has not reached the current limit." :
			"has been reaching the current limit for at least 30ms");
	}

	if (chg_int & BIT_CHG) {
		/* do insert code here */
		val = (chg_details[1] & BIT_CHG_DTLS)>>FFS(BIT_CHG_DTLS);
		pr_debug("CHG_IRQ_CHG_I: chg_dtls = %02Xh\n", val);
	}

	if (chg_int & BIT_BAT) {
		/* do insert code here */
		val = (chg_details[1] & BIT_BAT_DTLS)>>FFS(BIT_BAT_DTLS);
		pr_debug("CHG_IRQ_BAT_I: bat_dtls = %02Xh\n", val);
	}

	if (chg_int & BIT_DISQBAT) {
		disqbat = (chg_int_ok & BIT_DISQBAT) ? false : true;
		pr_debug("CHG_IRQ_DISQBAT_I: %s\n",
			disqbat ?
			"DISQBAT is low and QBATT is not disabled" :
			"DISQBAT is high and QBATT is disabled");
	}

	if (chg_int & BIT_BYP) {
		/* do insert code here */
		val = (chg_details[2] & BIT_BYP_DTLS)>>FFS(BIT_BYP_DTLS);
		pr_debug("CHG_IRQ_BYP_I: byp_dtls = %02Xh\n", val);
	}

	__unlock(charger);

	/* notify psy changed */
	power_supply_changed(charger->psy_chg);
	return;

}

static int max77975_charger_update(struct max77975_charger *charger)
{
	int rc;
	u8 chg_details[3];
	u8 chg_dtls;

	charger->health      = POWER_SUPPLY_HEALTH_UNKNOWN;
	charger->status      = POWER_SUPPLY_STATUS_UNKNOWN;
	charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	rc = max77975_bulk_read(charger->regmap,
		charger->regs[REG_CHG_DETAILS_00], chg_details, 3);
	if (IS_ERR_VALUE(rc)) {
		pr_err("CHG_DETAILS read error [%d]\n", rc);
		goto out;
	}

	pr_info("%s: chg_details 00=0x%x, 01=0x%x, 02=0x%x\n",
		__func__, chg_details[0], chg_details[1], chg_details[2]);

	charger->present = max77975_charger_present_input(charger);
	if (unlikely(!charger->present)) {
		/* no charger present */
		charger->health      = POWER_SUPPLY_HEALTH_UNKNOWN;
		charger->status      = POWER_SUPPLY_STATUS_DISCHARGING;
		charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		goto out;
	}

	chg_dtls = chg_details[1] & BIT_CHG_DTLS;

	charger->health =
		max77975_charger_status_map[chg_dtls].health;
	charger->status =
		max77975_charger_status_map[chg_dtls].status;
	charger->charge_type =
		max77975_charger_status_map[chg_dtls].charge_type;

	if (likely(charger->health != POWER_SUPPLY_HEALTH_UNKNOWN))
		goto out;

	/* override health by TREG */
	if ((chg_details[1] & BIT_TREG) != 0)
		charger->health = POWER_SUPPLY_HEALTH_OVERHEAT;

out:
	pr_info("%s: PRESENT %d HEALTH %d STATUS %d CHARGE_TYPE %d\n",
		__func__,
		charger->present, charger->health,
		charger->status, charger->charge_type);
	return rc;
}

static int max77975_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77975_charger *charger = power_supply_get_drvdata(psy);
	int rc = 0;

	rc = max77975_charger_update(charger);
	if (IS_ERR_VALUE(rc))
		goto out;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->present;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = charger->health;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = charger->status;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = charger->charge_type;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = max77975_charger_get_charge_current(charger);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = max77975_charger_get_input_current(charger);
		break;

	default:
		rc = -EINVAL;
		goto out;
	}

out:
	pr_info("%s: <get_property> psp %d val %d [%d]\n",
		__func__, psp, val->intval, rc);
	return rc;
}

static int max77975_charger_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct max77975_charger *charger = power_supply_get_drvdata(psy);
	int rc = 0;

	__lock(charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		rc = max77975_charger_set_enable(charger, val->intval);
		if (IS_ERR_VALUE(rc))
			goto out;

		/* apply charge current */
		rc = max77975_charger_set_charge_current(charger,
			charger->pdata->fast_charge_current);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* val->intval - uA */
		rc = max77975_charger_set_charge_current(charger,
			val->intval/1000); /* mA */
		if (IS_ERR_VALUE(rc))
			goto out;
		charger->pdata->fast_charge_current =
			val->intval/1000;	/* mA */
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77975_charger_set_input_current(charger,
			val->intval/1000);	/* mA */
		if (IS_ERR_VALUE(rc))
			goto out;
		charger->pdata->input_current_limit =
			val->intval/1000;	/* mA */
		break;

	default:
		rc = -EINVAL;
		goto out;
	}

out:
	pr_info("%s: <set_property> psp %d val %d [%d]\n",
		__func__, psp, val->intval, rc);
	__unlock(charger);
	return rc;
}

#ifdef CONFIG_OF
static int max77975_charger_parse_dt(struct max77975_charger *charger)
{
	struct device *dev = charger->dev;
	struct device_node *np = dev->of_node;
	struct max77975_charger_platform_data *pdata;
	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(pdata == NULL))
		return -ENOMEM;

	pdata->fast_charge_timer = 5;	/* disable */
	ret |= of_property_read_u32(np, "fast_charge_timer",
			&pdata->fast_charge_timer);
	pr_debug("property:FCHGTIME %uhour\n", pdata->fast_charge_timer);

	pdata->fast_charge_current = 450;	/* 450mA */
	ret |= of_property_read_u32(np, "fast_charge_current",
			&pdata->fast_charge_current);
	pr_debug("property:CHG_CC %umA\n", pdata->fast_charge_current);

	pdata->termination_voltage = 4200;	/* 4200mV */
	ret |= of_property_read_u32(np, "charge_termination_voltage",
			&pdata->termination_voltage);
	pr_debug("property:CHG_CV_PRM %umV\n",
			pdata->termination_voltage);

	pdata->topoff_timer = 30;	/* 30 min */
	ret |= of_property_read_u32(np, "topoff_timer",
			&pdata->topoff_timer);
	pr_debug("property:TOPOFF_TIME %umin\n", pdata->topoff_timer);

	pdata->topoff_current = 200;	/* 200mA */
	ret |= of_property_read_u32(np, "topoff_current",
			&pdata->topoff_current);
	pr_debug("property:TOPOFF_ITH %umA\n", pdata->topoff_current);

	pdata->restart_threshold = 150;	/* 150mV */
	ret |= of_property_read_u32(np, "restart_threshold",
			&pdata->restart_threshold);
	pr_debug("property:CHG_RSTRT %umV\n", pdata->restart_threshold);

	pdata->input_current_limit = 500; /* 500mA */
	ret |= of_property_read_u32(np, "input_current_limit",
			&pdata->input_current_limit);
	pr_debug("property:INPUT_CURRENT_LIMIT %umA\n",
		pdata->input_current_limit);

	if (ret < 0)
		return ret;
	charger->pdata = pdata;
	return 0;
}
#endif

static int max77975_charger_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct max77975_charger *charger;
	struct max77975_charger_platform_data *pdata;
	struct power_supply_config charger_cfg = {};
	int ret = 0;

	pr_info("%s: MAX77975 Charger Driver Loading\n", __func__);

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		pr_err("%s: out of memory\n", __func__);
		pdata = ERR_PTR(-ENOMEM);
		return -ENOMEM;
	}

	mutex_init(&charger->lock);

	charger->pdata = pdata;
	charger->regs = charger_regs[id->driver_data];
	charger->driver_data = id->driver_data;
	charger->dev = &client->dev;
	charger->irq = client->irq;
	charger->irq_gpio = -1;

#if defined(CONFIG_OF)
	ret = max77975_charger_parse_dt(charger);
	if (ret < 0) {
		pr_err("%s not found charger dt! ret[%d]\n",
				__func__, ret);
	}
#else
	pdata = dev_get_platdata(&pdev->dev);
#endif

	charger->regmap = devm_regmap_init_i2c(client,
						 &max77975_charger_regmap_config);
	if (IS_ERR(charger->regmap)) {
		ret = PTR_ERR(charger->regmap);
		charger->regmap = NULL;
		pr_err("<%s> failed to initialize i2c\n", client->name);
		pr_err("<%s> regmap [%d]\n", client->name,	ret);
		goto abort;
	}

	i2c_set_clientdata(client, charger);
	charger->psy_chg_d.name		= "max77975";
	charger->psy_chg_d.type		= POWER_SUPPLY_TYPE_UNKNOWN;
	charger->psy_chg_d.get_property	= max77975_charger_get_property;
	charger->psy_chg_d.set_property	= max77975_charger_set_property;
	charger->psy_chg_d.properties	= max77975_charger_props;
	charger->psy_chg_d.num_properties	=
		ARRAY_SIZE(max77975_charger_props);
	charger_cfg.drv_data = charger;

	max77975_charger_initialize(charger);

	charger->psy_chg = power_supply_register(charger->dev,
				&charger->psy_chg_d, &charger_cfg);
	if (IS_ERR(charger->psy_chg)) {
		pr_err("Couldn't register psy_chg rc=%ld\n",
				PTR_ERR(charger->psy_chg));
		goto err_power_supply_register;
	}

	INIT_DELAYED_WORK(&charger->irq_work, max77975_charger_irq_work);

	/* charger interrupt */
	if (charger->irq) {
		ret = request_threaded_irq(charger->irq, NULL,
					max77975_charger_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					charger->psy_chg->desc->name, charger);
		if (ret != 0) {
			dev_err(&client->dev, "failed to add chg irq: %d\n", ret);
			goto err_irqc_chg;
		}
	}

	pr_info("%s: Max77975 Charger Driver Loaded\n", __func__);
	return 0;

err_irqc_chg:
	free_irq(charger->irq, charger);
err_power_supply_register:
	power_supply_unregister(charger->psy_chg);
abort:
	pr_err("%s: Failed to probe max77975 Charger Driver\n", __func__);
	return ret;
}

static int max77975_charger_remove(struct i2c_client  *client)
{
	struct max77975_charger *charger = i2c_get_clientdata(client);

	free_irq(charger->irq, charger);
	power_supply_unregister(charger->psy_chg);

	kfree(charger);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77975_charger_suspend(struct device *dev)
{
	return 0;
}

static int max77975_charger_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(max77975_charger_pm_ops, max77975_charger_suspend,
		max77975_charger_resume);
#define MAX77975_PM_OPS (&max77975_charger_pm_ops)
#else
#define MAX77975_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */


static const struct of_device_id max77975_charger_dt_ids[] = {
	{ .compatible = "maxim,max77975", },
	{ .compatible = "maxim,max77985", },
	{ },
};
MODULE_DEVICE_TABLE(of, max77975_charger_dt_ids);

static const struct i2c_device_id max77975_charger_id[] = {
	{ "max77975", ID_MAX77975 },
	{ "max77985", ID_MAX77985 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max77975_charger_id);

static struct i2c_driver max77975_charger_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = MAX77975_PM_OPS,
		.of_match_table = of_match_ptr(max77975_charger_dt_ids),
	},
	.probe = max77975_charger_probe,
	.remove = max77975_charger_remove,
	.id_table = max77975_charger_id,
};
module_i2c_driver(max77975_charger_driver);


MODULE_DESCRIPTION("MAX77975 Charger Driver");
MODULE_AUTHOR("opensource@maximintegrated.com ");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
