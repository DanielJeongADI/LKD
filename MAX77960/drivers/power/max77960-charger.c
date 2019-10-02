/*
 * Copyright (c) 2019 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#if defined(CONFIG_OF)
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#include <linux/power/max77960-charger.h>

#define DEV_NAME "max77960"


#define POWER_SUPPLY_TYPE_BATTERY 1

#define DECLARE_IRQ(idx, _group, _mask)		\
	[(idx)] = { .group = (_group), .mask = (_mask) }

static const struct max77960_irq_data max77960_irqs[] = {
	DECLARE_IRQ(MAX77960_AICL_INT,  CHG_INT, 1 << 7),
	DECLARE_IRQ(MAX77960_CHGIN_INT, CHG_INT, 1 << 6),
	DECLARE_IRQ(MAX77960_B2SOVRC_INT, CHG_INT, 1 << 5),
	DECLARE_IRQ(MAX77960_CHGER_INT, CHG_INT, 1 << 4),
	DECLARE_IRQ(MAX77960_BAT_INT, CHG_INT, 1 << 3),
	DECLARE_IRQ(MAX77960_CHGINLIM_INT, CHG_INT, 1 << 2),
	DECLARE_IRQ(MAX77960_DISQBAT_INT, CHG_INT, 1 << 1),
	DECLARE_IRQ(MAX77960_OTG_INT, CHG_INT, 1 << 0),
};

static enum power_supply_property max77960_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static void max77960_charger_initialize(struct max77960_charger_data *chg_data);
static int max77960_charger_get_health_state(struct max77960_charger_data
	*chg_data);

int max77960_read_reg(struct regmap *regmap, u8 addr, u8 *val)
{
    unsigned int buf = 0;
    int rc = regmap_read(regmap, (unsigned int)addr, &buf);

	if (likely(!IS_ERR_VALUE(rc)))
        *val = (u8)buf;
    return rc;
}

int max77960_write_reg(struct regmap *regmap, u8 addr, u8 val)
{
    unsigned int buf = (unsigned int)val;

    return regmap_write(regmap, (unsigned int)addr, buf);
}

int max77960_bulk_read(struct regmap *regmap, u8 addr, u8 *dst, u16 len)
{
    return regmap_bulk_read(regmap, (unsigned int)addr, dst, (size_t)len);
}

int max77960_bulk_write(struct regmap *regmap, u8 addr, const u8 *src, u16 len)
{
    return regmap_bulk_write(regmap, (unsigned int)addr, src, (size_t)len);
}

int max77960_update_reg(struct regmap *regmap, u8 reg, u8 val, u8 mask)
{
	u8 old_val, new_val, rval;
	int ret = 0;

	ret = max77960_read_reg(regmap, reg, &rval);
	if (ret < 0)
		return ret;

	old_val = rval & 0xff;
	new_val = (val & mask) | (old_val & (~mask));

	ret = max77960_write_reg(regmap, reg, new_val);

	return ret;
}

static bool max77960_charger_unlock(struct max77960_charger_data *chg_data)
{
	struct regmap *regmap = chg_data->regmap;
	u8 reg_data;
	u8 chgprot;
	int retry_cnt = 0;
	bool need_init = false;

	do {
		max77960_read_reg(regmap, MAX77960_CHG_CNFG_06, &reg_data);
		chgprot = ((reg_data & 0x0C) >> 2);
		if (chgprot != 0x03) {
			pr_info("%s: Unlock err, chgprot(0x%x), retry(%d)\n",
				__func__,	chgprot, retry_cnt);
			max77960_write_reg(regmap, MAX77960_CHG_CNFG_06,
				(0x03 << 2));
			need_init = true;
			msleep(20);
		} else {
			pr_info("%s: Unlock success, chgprot(0x%x)\n",
				__func__, chgprot);
			break;
		}
	} while ((chgprot != 0x03) && (++retry_cnt < 10));

	return need_init;
}

static void max77960_charger_check_unlock_state(struct max77960_charger_data
	*chg_data)
{
	bool need_reg_init;

	need_reg_init = max77960_charger_unlock(chg_data);
	if (need_reg_init) {
		pr_info("%s: charger locked state, reg init\n", __func__);
		max77960_charger_initialize(chg_data);
	}
}

static void max77960_charger_set_state(struct max77960_charger_data
	*chg_data, int en)
{
	struct regmap *regmap = chg_data->regmap;

	u8 reg_data;

	max77960_read_reg(regmap, MAX77960_CHG_CNFG_00, &reg_data);

	if (en)
		reg_data |= MAX77960_MODE_CHGR;
	else
		reg_data &= ~MAX77960_MODE_CHGR;

	pr_info("%s: CHG_CNFG_00(0x%02x)\n", __func__, reg_data);

	max77960_write_reg(regmap, MAX77960_CHG_CNFG_00, reg_data);
}


static bool max77818_charger_present_input(struct max77960_charger_data
	*chg_data)
{
    u8 chg_int_ok = 0;
    int rc;
	struct regmap *regmap = chg_data->regmap;

	rc = max77960_read_reg(regmap, MAX77960_CHG_INT_OK, &chg_int_ok);
	if (unlikely(IS_ERR_VALUE(rc)))
        return false;

	if ((chg_int_ok & BIT_CHGIN_OK) == BIT_CHGIN_OK)
		return true;

		/* check whether charging or not in the UVLO condition */
		if (((chg_data->details_0 & BIT_CHGIN_DTLS) == 0) &&
			(((chg_data->details_1 & BIT_CHG_DTLS)
					== CHG_DTLS_FASTCHARGE_CC) ||
			 ((chg_data->details_1 & BIT_CHG_DTLS)
					== CHG_DTLS_FASTCHARGE_CV))) {
			return true;
	}

			return false;
}

static void max77960_charger_set_input_current(struct max77960_charger_data
	*chg_data, int input_current)
{
	struct regmap *regmap = chg_data->regmap;
	u8 reg_data = 0;
	int current_limit_step;

	current_limit_step = chg_data->input_curr_limit_step;
	/* unit mA */
	if (!input_current) {
		reg_data = 0;
	} else {
		if (input_current % current_limit_step)
	    goto err_value;
		if (input_current <= 100)
      reg_data = 0;
	  else if (input_current <= 6300) /* 6300mA */
			reg_data |= ((input_current / current_limit_step)+1);
    else
      reg_data |= 0x7F;
  }

	pr_info("max77960 set input current: reg_data(0x%02x), current(%dmA)\n",
		reg_data, input_current);

	max77960_write_reg(regmap, MAX77960_CHG_CNFG_08, reg_data);
err_value:
	pr_info("max77960 set input current: %dmA\n", input_current);
	pr_info("max77960 not match the register.\n");
}

static int max77960_charger_get_input_current(struct max77960_charger_data
	*chg_data)
{
	struct regmap *regmap = chg_data->regmap;
	u8 reg_data = 0;
	int get_current = 0;

	max77960_read_reg(regmap, MAX77960_CHG_CNFG_08, &reg_data);
	if ((reg_data & BIT_CHGIN_ILIM) < 4)
		get_current = 100; 	/* 100mA */
	else if ((reg_data & BIT_CHGIN_ILIM) > 0x7F)
		get_current = 6300;	/* 6300mA */
	else
		get_current = (reg_data & BIT_CHGIN_ILIM)*50 - 50;

	pr_info("max77960 get input current: reg_data(0x%02x), current(%dmA)\n",
	  reg_data, get_current);

	return get_current;
}

static void max77960_charger_set_charge_current(struct max77960_charger_data
	*chg_data, int cur)
{
	struct regmap *regmap = chg_data->regmap;

	u8 reg_data = 0;

	max77960_read_reg(regmap, MAX77960_CHG_CNFG_02, &reg_data);
	reg_data &= ~MAX77960_CHG_CC;

	if (!cur) {
		/* No charger */
		max77960_write_reg(regmap, MAX77960_CHG_CNFG_02, reg_data);
	} else {
		if (cur % 50)
	    goto err_value;
		if (cur > 450)
      reg_data |= ((cur / chg_data->charging_curr_step) + 3);
		else
      reg_data |= ((cur / 50) - 2);

		max77960_write_reg(regmap, MAX77960_CHG_CNFG_02, reg_data);
	}

	pr_info("max77960 set charge current: reg_data(0x%02x), charge(%d)\n",
			reg_data, cur);

err_value:
	pr_info("max77960 set charge current: %dmA\n", cur);
	pr_info("max77960 not match the register.\n");
}

static int max77960_charger_get_charge_current(struct max77960_charger_data
	*chg_data)
{
	struct regmap *regmap = chg_data->regmap;

	u8 reg_data;
	int get_current = 0;

	max77960_read_reg(regmap, MAX77960_CHG_CNFG_02, &reg_data);
	pr_info("max77960 CHG_CNFG_02(0x%02x)\n", reg_data);

	reg_data &= MAX77960_CHG_CC;
	if (reg_data > 0x07)
    get_current = reg_data * chg_data->charging_curr_step - 300;
	else
    get_current = reg_data * 50 + 100;

	pr_info("max77960 get charge current: reg(0x%02x),", reg_data);
	pr_info("max77960 get charge current: %dmA\n", get_current);
	return get_current;
}

static void max77960_charger_set_topoff_current(struct max77960_charger_data
	*chg_data, int cur, int time)
{
	struct regmap *regmap = chg_data->regmap;

	u8 reg_data;

	if (cur >= 600)
		reg_data = 0x07;
	else if (cur >= 500)
		reg_data = 0x04;
	else if (cur >= 400)
		reg_data = 0x03;
	else if (cur >= 300)
		reg_data = 0x02;
	else if (cur >= 200)
		reg_data = 0x01;
	else
		reg_data = 0x00;

  /* topoff timer setting*/
  time = time / 10;
  reg_data |= (time << 3);
	pr_info("max77960 set topoff cur: reg_data(0x%02x), topoff current(%d)\n",
		reg_data, cur);

	max77960_write_reg(regmap, MAX77960_CHG_CNFG_03, reg_data);
}

static int max77960_charger_get_charger_state(struct max77960_charger_data
	*chg_data)
{
	struct regmap *regmap = chg_data->regmap;

	int state;
	u8 reg_data;

	max77960_read_reg(regmap, MAX77960_CHG_DTLS_01, &reg_data);
	reg_data = ((reg_data & BIT_CHG_DTLS) >> FFS(BIT_CHG_DTLS));
	pr_info("max77960 get charger state CHG_DTLS(0x%02x)\n", reg_data);

	switch (reg_data) {
		case 0x0:
		case 0x1:
		case 0x2:
			state = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case 0x3:
		case 0x4:
			state = POWER_SUPPLY_STATUS_FULL;
			break;
		case 0x5:
		case 0x6:
		case 0x7:
			state = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case 0x8:
		case 0xA:
		case 0xB:
			state = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		default:
			state = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
	}

	return state;
}

static int max77960_charger_get_health_state(struct max77960_charger_data
	*chg_data)
{
	struct regmap *regmap = chg_data->regmap;

	int state;
	u8  reg_data;

	max77960_read_reg(regmap, MAX77960_CHG_DTLS_01, &reg_data);
	reg_data = ((reg_data & BIT_BAT_DTLS) >> FFS(BIT_BAT_DTLS));

	pr_info("max77960 CHG_DTLS_01(0x%02x)\n",  reg_data);
	switch (reg_data) {
		case 0x00:
			pr_info("No battery and the charger is suspended\n");
			state = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		case 0x01:
			pr_info("battery is okay, but its voltage is low(<VPRECHG)\n");
			state = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case 0x02:
			pr_info("battery dead\n");
			state = POWER_SUPPLY_HEALTH_DEAD;
			break;
		case 0x03:
			pr_info("battery good\n");
			state = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case 0x04:
		pr_info("battery is okay but its voltage is low\n");
			state = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case 0x05:
		pr_info("battery ovp\n");
			state = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			break;
		case 0x06:
		pr_info("battery ocp\n");
			state = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			break;
		case 0x07:
		pr_info("battery not available\n");
			state = POWER_SUPPLY_HEALTH_UNKNOWN;
			break;
		default:
		pr_info("battery unknown\n");
			state = POWER_SUPPLY_HEALTH_UNKNOWN;
			break;
	}

#ifdef CONFIG_BATTERY_ENABLED
	if (state == POWER_SUPPLY_HEALTH_GOOD) {
		union power_supply_propval value;
		{
			struct power_supply *psy;
			int ret;
			int state_helth;

				psy = power_supply_get_by_name("battery");
			if (!psy)
				value.intval = 0;
			else {
				if (psy->desc->get_property != NULL) {
					state_helth = POWER_SUPPLY_PROP_HEALTH;
					ret = psy->desc->get_property(psy,
						state_helth, &value);
					if (ret < 0)
						value.intval = 0;
				}
				power_supply_put(psy);
			}
		}
	}
#endif
	return state;
}
static int max77960_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max77960_charger_data *chg_data = power_supply_get_drvdata(psy);
	u8 reg_data;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = POWER_SUPPLY_TYPE_BATTERY;
			if (max77960_read_reg(chg_data->regmap,
						MAX77960_CHG_INT_OK, &reg_data) == 0) {
			if (reg_data & BIT_CHGIN_OK)
					val->intval = POWER_SUPPLY_TYPE_MAINS;
				}
			break;

		case POWER_SUPPLY_PROP_STATUS:
			val->intval = max77960_charger_get_charger_state(chg_data);
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = max77960_charger_get_health_state(chg_data);
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = max77960_charger_get_charge_current(chg_data);
			break;

        case POWER_SUPPLY_PROP_CURRENT_MAX:
            val->intval = max77960_charger_get_input_current(chg_data);
		break;

		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			if (!chg_data->is_charging)
				val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			else
				val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int max77960_charger_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct max77960_charger_data *chg_data = power_supply_get_drvdata(psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			chg_data->status = val->intval;
			break;
			/* val->intval : type */
		case POWER_SUPPLY_PROP_ONLINE:
			/* check and unlock */
			max77960_charger_check_unlock_state(chg_data);
			max77960_charger_set_state(chg_data, val->intval);
           /* apply charge current */
            max77960_charger_set_input_current(chg_data,
                    chg_data->fast_charge_current);  /* mA */
		break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			max77960_charger_set_charge_current(chg_data,
				val->intval);	/* mA */
            chg_data->fast_charge_current = val->intval;	/* mA */
			break;
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			max77960_charger_set_input_current(chg_data,
				val->intval);	/* mA */
		chg_data->input_current_limit = val->intval;	/* mA */
		break;
		default:
			return -EINVAL;
	}

	return 0;
}

static void max77960_charger_initialize(struct max77960_charger_data *chg_data)
{
	struct regmap *regmap = chg_data->regmap;
	u8 reg_data;

	/*
	 * interrupt mask - if you want to enable some bits,
	 * you should clear them
	 */
	reg_data = BIT_AICL_M | BIT_CHGIN_M | BIT_B2SOVRC_M
		| BIT_CHG_M | BIT_BAT_M | BIT_CHGINILIM_M
		| BIT_DISQBAT_M | BIT_OTG_PLIM_M;
    max77960_write_reg(regmap, MAX77960_CHG_INT_MASK, reg_data);

	/* unlock charger protect */
	reg_data = (0x03 << 2);
	max77960_write_reg(regmap, MAX77960_CHG_CNFG_06, reg_data);

    /* charge current (mA) */
	max77960_charger_set_charge_current(chg_data,
		chg_data->fast_charge_current);

  	/* input current limit (mA) */
	max77960_charger_set_input_current(chg_data,
		chg_data->input_current_limit);

    /* topoff current(mA) and topoff timer(min) */
	max77960_charger_set_topoff_current(chg_data,
		chg_data->topoff_current, chg_data->topoff_timer);

	/*
	 * Smart Power Selector
	 * CHG on, OTG off, DCDC on
	 */
    reg_data = (0x05 << 0);
    max77960_write_reg(regmap, MAX77960_CHG_CNFG_00, reg_data);

	/*
     * fast charge timer, enable STAT output
     * restart threshold, prequal charge enable
	 */
    reg_data = BIT_STAT_EN | BIT_PQEN | (0x01 << 4) | (0x01 << 0);
    max77960_write_reg(regmap, MAX77960_CHG_CNFG_01, reg_data);

	/*
     * charge termination voltage (8V)
	 */
    reg_data = (0x00 << 0);
    max77960_write_reg(regmap, MAX77960_CHG_CNFG_04, reg_data);
}

static void max77960_chgin_isr_work(struct work_struct *work)
{
	struct max77960_charger_data *chg_data =
		container_of(work, struct max77960_charger_data, chgin_work);
	struct regmap *regmap = chg_data->regmap;
	u8 reg_data;
    bool chg_input;

	max77960_read_reg(regmap, MAX77960_CHG_INT_MASK, &reg_data);
	reg_data |= BIT_CHGIN_M;
	max77960_write_reg(regmap, MAX77960_CHG_INT_MASK, reg_data);

	max77960_read_reg(regmap, MAX77960_CHG_DTLS_00, &reg_data);
    chg_data->details_0 = reg_data;
	max77960_read_reg(regmap, MAX77960_CHG_DTLS_01, &reg_data);
	chg_data->details_1 = reg_data;
	max77960_read_reg(regmap, MAX77960_CHG_DTLS_02, &reg_data);
    chg_data->details_2 = reg_data;

    chg_input = max77818_charger_present_input(chg_data);
	if (chg_input) {
        max77960_charger_set_state(chg_data, true);
        /* apply charge current */
        max77960_charger_set_input_current(chg_data,
				chg_data->fast_charge_current);  /* mA */
	} else {
        max77960_charger_set_state(chg_data, false);
    }
	pr_info("max77960 CHG_INT_CHGIN: charge input %s\n",
		chg_input ? "inserted" : "removed");
	max77960_read_reg(regmap, MAX77960_CHG_INT_MASK, &reg_data);
	reg_data &= ~BIT_CHGIN_M;
	max77960_write_reg(regmap, MAX77960_CHG_INT_MASK, reg_data);
}

static irqreturn_t max77960_charger_irq(int irq, void *data)
{
	struct max77960_charger_data *chg_data = data;
	struct regmap *regmap = chg_data->regmap;
	u8 reg_data;

	max77960_read_reg(regmap, MAX77960_CHG_INT, &reg_data);

	if (reg_data & BIT_CHGIN_I)
        queue_work(chg_data->wqueue, &chg_data->chgin_work);
	/*
	 * add other interrupts
	 * else {
	 *
	 *	}
	 */
	return IRQ_HANDLED;
}

static const struct power_supply_desc max77960_charger_power_supply_desc = {
	.name = "max77960-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = max77960_charger_props,
	.num_properties = ARRAY_SIZE(max77960_charger_props),
	.get_property = max77960_charger_get_property,
	.set_property = max77960_charger_set_property,
};

static char *chg_supplied_to[] = {
	"max77960-charger",
};



static const struct regmap_config max77960_charger_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct regmap_irq max77960_chg_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_OTG_PLIM_I,	},
	{ .reg_offset = 0, .mask = BIT_CHGINILIM_I,	},
	{ .reg_offset = 0, .mask = BIT_BAT_I,	},
	{ .reg_offset = 0, .mask = BIT_CHG_I,	},
	{ .reg_offset = 0, .mask = BIT_B2SOVRC_I,	},
	{ .reg_offset = 0, .mask = BIT_CHGIN_I,	},
	{ .reg_offset = 0, .mask = BIT_AICL_I,	},
};

static const struct regmap_irq_chip max77960_chg_irq_chip = {
	.name = "max77960 chg",
	.status_base = MAX77960_CHG_INT,
	.mask_base = MAX77960_CHG_INT_MASK,
	.num_regs = 1,
	.irqs = max77960_chg_irqs,
	.num_irqs = ARRAY_SIZE(max77960_chg_irqs),
};


#if defined(CONFIG_OF)
static int of_max77960_dt(struct device *dev, struct max77960_charger_data
	*chg_data)
{
	struct device_node *np_max77960 = dev->of_node;
	unsigned int gpio;
    int rc;

	if (!np_max77960)
		return -EINVAL;

	chg_data->irq_gpio = of_get_named_gpio(np_max77960,
		"max77960,irq-gpio", 0);

    gpio = (unsigned)chg_data->irq_gpio;

    rc = gpio_request(gpio, "max77960-irq");
    if (unlikely(IS_ERR_VALUE(rc))) {
		pr_err("max77960 failed to request gpio %u [%d]\n", gpio, rc);
        chg_data->irq_gpio = -1;
        goto out;
    }

    gpio_direction_input(gpio);
	pr_info("max77960 INTGPIO %u assigned\n", gpio);

    /* override pdata irq */
    chg_data->irq = gpio_to_irq(gpio);

	pr_info("max77960 property:INTGPIO %d\n", chg_data->irq_gpio);
	pr_info("max77960 property:IRQ     %d\n", chg_data->irq);
out:
	return 0;
}
#endif /* CONFIG_OF */



static int max77960_charger_probe(struct i2c_client *client,
		       const struct i2c_device_id *i2c_id)
{
	struct max77960_charger_data *chg_data;
	const struct regmap_config *rmap_cfg;
	struct power_supply_config charger_cfg;
	int ret = 0;

	pr_info("%s: MAX77960 Charger Driver Loading\n", __func__);

	chg_data = devm_kzalloc(&client->dev, sizeof(*chg_data), GFP_KERNEL);
	if (!chg_data)
		return -ENOMEM;

	rmap_cfg = &max77960_charger_regmap_config;
	chg_data->regmap = devm_regmap_init_i2c(client, rmap_cfg);
	if (IS_ERR(chg_data->regmap))
		return PTR_ERR(chg_data->regmap);
	if (client->dev.of_node)
    	of_max77960_dt(&client->dev, chg_data);
	chg_data->dev = &client->dev;
	chg_data->aicl_on = false;

  chg_data->topoff_current = 0; /* 100mA */
  chg_data->topoff_timer = 30; /* 30min */

	chg_data->input_curr_limit_step = 50;
  chg_data->input_current_limit = 500; /* 500mA */

	chg_data->charging_curr_step = 100;
	chg_data->fast_charge_current = 450; /* 450mA */

	max77960_charger_initialize(chg_data);

	chg_data->wqueue =
		create_singlethread_workqueue(dev_name(&client->dev));
	if (!chg_data->wqueue) {
		pr_info("%s: Fail to Create Workqueue\n", __func__);
		goto err_free;
	}

	INIT_WORK(&chg_data->chgin_work, max77960_chgin_isr_work);

	charger_cfg.drv_data = chg_data;
	charger_cfg.supplied_to = chg_supplied_to;
	charger_cfg.num_supplicants = ARRAY_SIZE(chg_supplied_to);

	chg_data->psy_chg = devm_power_supply_register(&client->dev,
			&max77960_charger_power_supply_desc, &charger_cfg);
	if (!chg_data->psy_chg) {
		pr_err("max77960 couldn't register psy_chg (%ld)\n",
			PTR_ERR(chg_data->psy_chg));
		goto err_power_supply_register;
	}

	ret = request_threaded_irq(chg_data->irq, NULL,
			max77960_charger_irq, IRQF_TRIGGER_FALLING,
			"charger-chgin", chg_data);
	if (ret) {
		dev_err(&client->dev, "max77960 failed to add chg irq chip: %d\n",
			ret);
		goto err_irq;
    }

	pr_info("%s: MAX77960 charger Driver Loaded\n", __func__);
	return 0;

err_irq:
	power_supply_unregister(chg_data->psy_chg);

err_power_supply_register:
	destroy_workqueue(chg_data->wqueue);

err_free:
	kfree(chg_data);

	return ret;

}

static int max77960_charger_remove(struct i2c_client *client)
{
	struct max77960_charger_data *chg_data = i2c_get_clientdata(client);

	destroy_workqueue(chg_data->wqueue);
	free_irq(chg_data->irq_chg, NULL);
	power_supply_unregister(chg_data->psy_chg);
	kfree(chg_data);

	return 0;
}

#if defined CONFIG_PM
static int max77960_charger_suspend(struct device *dev)
{
	return 0;
}

static int max77960_charger_resume(struct device *dev)
{
	return 0;
}
#else
#define max77960_charger_suspend NULL
#define max77960_charger_resume NULL
#endif

static const struct dev_pm_ops max77960_charger_pm_ops = {
	.suspend = max77960_charger_suspend,
	.resume = max77960_charger_resume,
};

static const struct i2c_device_id max77960_charger_id[] = {
	{ DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77960_charger_id);

#ifdef CONFIG_OF
static const struct of_device_id max77960_charger_of_match[] = {
	{ .compatible =  DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(of, max77960_charger_of_match);
#endif


static struct i2c_driver max77960_i2c_driver = {
	.driver = {
		.name = DEV_NAME,
		.pm = &max77960_charger_pm_ops,
		.of_match_table = of_match_ptr(max77960_charger_of_match),
	},
	.probe = max77960_charger_probe,
	.remove = max77960_charger_remove,
	.id_table = max77960_charger_id,
};

module_i2c_driver(max77960_i2c_driver);

MODULE_DESCRIPTION("max77960 charger driver");
MODULE_AUTHOR("opensource@maximintegrated.com");
MODULE_LICENSE("GPL");
