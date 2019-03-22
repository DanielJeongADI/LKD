// SPDX-License-Identifier: GPL-2.0+
/*
 * max77816.c - REGULATOR device driver for MAX77816
 *
 * Copyright (C) 2019 Maxim Integrated. All rights reserved.
 *
 * Author:
 *  Daniel Jeong <daniel.jeong@maximintegrated.com>
 *	Maxim Opensource <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max77816.h>
#include <linux/regulator/of_regulator.h>

#define DRIVER_NAME		"max77816"

struct max77816_chip {
	struct regmap *regmap;
	struct device *dev;
	struct regulator_desc regulator_desc;
	struct regulator_dev *regulator;
};

static int max77816_get_status(struct regulator_dev *rdev)
{
	struct max77816_chip *pchip = rdev_get_drvdata(rdev);
	unsigned int data;
	int ret;

	ret = regmap_read(pchip->regmap, MAX77816_REG_STATUS, &data);
	if (ret < 0) {
		dev_err(pchip->dev,
			"failed to read i2c: %d @ function %s\n",
			ret, __func__);
		return ret;
	}
	return (data & MAX77816_MASK_ST);
}

static int max77816_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct max77816_chip *pchip = rdev_get_drvdata(rdev);
	int ret;

	if (mode != REGULATOR_MODE_NORMAL)
		ret = regmap_update_bits(pchip->regmap,
				MAX77816_REG_CONFIG1,
				MAX77816_MASK_FPWM, 0);
	else
		ret = regmap_update_bits(pchip->regmap,
				MAX77816_REG_CONFIG1,
				MAX77816_MASK_FPWM, MAX77816_MASK_FPWM);

	if (ret < 0) {
		dev_err(pchip->dev,
			"failed to update i2c: %d @ function %s\n",
			ret, __func__);
	}
	return ret;
}

static unsigned int max77816_get_mode(struct regulator_dev *rdev)
{
	struct max77816_chip *pchip = rdev_get_drvdata(rdev);
	unsigned int rval;
	int ret;

	ret = regmap_read(pchip->regmap, MAX77816_REG_CONFIG1, &rval);
	if (ret < 0) {
		dev_err(pchip->dev,
			"failed to read i2c: %d @ function %s\n",
			ret, __func__);
		return ret;
	}

	return (rval & MAX77816_MASK_FPWM)
			? REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL;
}

static struct regulator_ops max77816_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.get_status = max77816_get_status,
	.set_mode = max77816_set_mode,
	.get_mode = max77816_get_mode,
};

static const struct regmap_config max77816_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77816_REG_INT,
	.use_single_rw = true,
};

static struct regulator_desc max77816_reg_vout = {
	.name = "vout",
	.id = MAX77816_ID_VOUT,
	.ops = &max77816_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_mask = MAX77816_MASK_BB_EN,
	.enable_reg = MAX77816_REG_CONFIG2,
	.min_uV = MAX77816_VOUT_MIN_UV,
	.uV_step = MAX77816_VOUT_STEP_UV,
	.n_voltages = MAX77816_MASK_VOUT + 1,
	.vsel_reg = MAX77816_REG_VOUT,
	.vsel_mask = MAX77816_MASK_VOUT,
	.active_discharge_off = MAX77816_AD_DISABLE,
	.active_discharge_on = MAX77816_MASK_AD,
	.active_discharge_mask = MAX77816_MASK_AD,
	.active_discharge_reg = MAX77816_REG_CONFIG1,
};

static struct regulator_desc max77816_reg_vout_h = {
	.name = "vout_h",
	.id = MAX77816_ID_VOUT_H,
	.ops = &max77816_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_mask = MAX77816_MASK_BB_EN,
	.enable_reg = MAX77816_REG_CONFIG2,
	.min_uV = MAX77816_VOUT_H_MIN_UV,
	.uV_step = MAX77816_VOUT_H_STEP_UV,
	.n_voltages = MAX77816_MASK_VOUT_H + 1,
	.vsel_reg = MAX77816_REG_VOUT,
	.vsel_mask = MAX77816_MASK_VOUT_H,
	.active_discharge_off = MAX77816_AD_DISABLE,
	.active_discharge_on = MAX77816_MASK_AD,
	.active_discharge_mask = MAX77816_MASK_AD,
	.active_discharge_reg = MAX77816_REG_CONFIG1,
};

static int max77816_init_regulator(struct max77816_chip *pchip,
				   struct device_node *node)
{
	int ret;
	unsigned int rval;
	struct regulator_config config = { };

	config.regmap = pchip->regmap;
	config.driver_data = pchip;
	config.dev = pchip->dev;

	ret = regmap_read(pchip->regmap, MAX77816_REG_CONFIG2, &rval);
	if (ret < 0) {
		dev_err(pchip->dev,
			"failed to read i2c: %d @ function %s\n", ret,
			__func__);
		return ret;
	}

	switch (rval & MAX77816_MASK_GPIO_CFG) {
	case MAX77816_SUB_A_F:
	case MAX77816_SUB_B:
	case MAX77816_SUB_D:
	case MAX77816_SUB_E:
		pchip->regulator_desc = max77816_reg_vout;
		pchip->regulator
			= devm_regulator_register(pchip->dev,
					&max77816_reg_vout, &config);
		break;
	case MAX77816_SUB_C:
		pchip->regulator_desc = max77816_reg_vout_h;
		pchip->regulator
			= devm_regulator_register(pchip->dev,
			&max77816_reg_vout_h, &config);
		break;

	default:
		pchip->regulator_desc = max77816_reg_vout;
		pchip->regulator = devm_regulator_register(pchip->dev,
				&max77816_reg_vout, &config);
		dev_warn(pchip->dev, "Unknown Versioin: %d\n",
		rval & MAX77816_MASK_GPIO_CFG);
		break;
	}

	if (IS_ERR(pchip->regulator)) {
		ret = PTR_ERR(pchip->regulator);
		return ret;
	}
	return 0;
}

static int max77816_regulator_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device_node *node = client->dev.of_node;
	struct device *dev = &client->dev;
	struct max77816_chip *pchip;
	int ret;

	pchip = devm_kzalloc(dev, sizeof(struct max77816_chip), GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;

	i2c_set_clientdata(client, pchip);
	pchip->dev = dev;

	pchip->regmap = devm_regmap_init_i2c(client, &max77816_regmap_config);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(dev, "failed to initialize regmap: %d\n", ret);
		return ret;
	}

	ret = max77816_init_regulator(pchip, node);
	if (ret < 0) {
		dev_err(dev, "failed to register regulator: %d\n", ret);
		return ret;
	}

	dev_info(pchip->dev, "max77816 init done\n");
	return 0;
}

static const struct i2c_device_id max77816_i2c_id[] = {
	{DRIVER_NAME},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77816_i2c_id);

static const struct of_device_id max77816_of_match[] = {
	{.compatible = "maxim,max77816"},
	{},
};
MODULE_DEVICE_TABLE(of, max77816_of_match);

static struct i2c_driver max77816_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(max77816_of_match),
	},
	.probe	= max77816_regulator_probe,
	.id_table = max77816_i2c_id,
};
module_i2c_driver(max77816_driver);

MODULE_AUTHOR("Daniel Jeong <daniel.jeong@maximintegrated.com>");
MODULE_AUTHOR("Maxim Opensource <opensource@maximintegrated.com>");
MODULE_DESCRIPTION("Regulator Device Driver for Maxim MAX77816");
MODULE_LICENSE("GPL");
