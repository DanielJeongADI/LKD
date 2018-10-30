// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxim MAX77932 Dual Phase Switched Capacitor Converter
 *
 * Copyright (C) 2018 Maxim Integrated. All rights reserved.
 *
 * Author:
 *	Maxim LDD <opensource@maximintegrated.com>
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
#include <linux/regulator/max77932.h>
#include <linux/regulator/of_regulator.h>

#ifdef __TEST_DEVICE_NODE__
#include <linux/string.h>
#include <linux/sysfs.h>
#endif

#define DRIVER_NAME		"max77932"
#define DRIVER_IRQ_NAME		"max77932-irq"

static const struct regmap_range max77932_readable_ranges[] = {
	regmap_reg_range(MAX77932_REG_INT_SRC, MAX77932_REG_EN_CFG2),
	regmap_reg_range(MAX77932_REG_I2C_CFG, MAX77932_REG_DEVICE_ID),
};

static const struct regmap_access_table max77932_readable_table = {
	.yes_ranges = max77932_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77932_readable_ranges),
};

static const struct regmap_range max77932_writable_ranges[] = {
	regmap_reg_range(MAX77932_REG_INT_SRC_M, MAX77932_REG_INT_SRC_M),
	regmap_reg_range(MAX77932_REG_SCC_EN, MAX77932_REG_EN_CFG2),
};

static const struct regmap_access_table max77932_writable_table = {
	.yes_ranges = max77932_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77932_writable_ranges),
};

static int max77932_get_status(struct regulator_dev *rdev)
{
	struct max77932_chip *pchip = rdev_get_drvdata(rdev);
	unsigned int data;
	int ret;

	ret = regmap_read(pchip->regmap, MAX77932_REG_STATUS, &data);
	if (ret < 0) {
		dev_err(pchip->dev,
			"failed to read i2c: %d @ function %s\n",
			ret, __func__);
		return ret;
	}
	return (data & 0xff);
}

static int max77932_write_ocp1(struct max77932_chip *pchip, int max_uA)
{
	int ret;
	unsigned int rval;

	if (max_uA > 9600000) {
		max_uA = 9600000;
		dev_warn(pchip->dev,
			 "input value(%d) > max value(9600000 mA)  %s\n",
			 max_uA, __func__);
	}
	rval = (max_uA - 4200000) / 200000;
	ret = regmap_update_bits(pchip->regmap, MAX77932_REG_OCP1, 0x1f, rval);
	if (ret < 0) {
		dev_err(pchip->dev,
			"failed to write i2c: %d @ function %s\n", ret,
			__func__);
		return ret;
	}
	return 0;
}

static int max77932_write_ocp2(struct max77932_chip *pchip, int max_uV)
{
	int ret;
	unsigned int rval;

	if (max_uV > 240) {
		max_uV = 240;
		dev_warn(pchip->dev,
			 "input value(%d) > max value(240mV)  %s\n",
			 max_uV, __func__);
	}
	rval = (max_uV - 90) / 10;
	ret = regmap_update_bits(pchip->regmap, MAX77932_REG_OCP2, 0x0f, rval);
	if (ret < 0) {
		dev_err(pchip->dev,
			"failed to write i2c: %d @ function %s\n", ret,
			__func__);
		return ret;
	}
	return 0;
}

static int max77932_set_current_limit(struct regulator_dev *rdev, int min_uA,
				      int max_uA)
{
	struct max77932_chip *pchip = rdev_get_drvdata(rdev);
	return max77932_write_ocp1(pchip, max_uA);
}

static int max77932_get_current_limit(struct regulator_dev *rdev)
{
	struct max77932_chip *pchip = rdev_get_drvdata(rdev);
	unsigned int rval, ocp;
	int ret;

	ret = regmap_read(pchip->regmap, MAX77932_REG_OCP1, &rval);
	if (ret < 0) {
		dev_err(pchip->dev,
			"failed to read i2c: %d @ function %s\n", ret,
			__func__);
		return ret;
	}

	ocp = rval & 0x1f;
	if (ocp > 0x1b)
		return 9600000;
	return (4200000 + 200000 * ocp);
}

static struct regulator_ops max77932_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.get_status = max77932_get_status,
	.set_current_limit = max77932_set_current_limit,
	.get_current_limit = max77932_get_current_limit,
};

static struct regulator_desc max77932_reg = {
	.name = "scc",
	.id = 0,
	.ops = &max77932_ops,
	.type = REGULATOR_CURRENT,
	.owner = THIS_MODULE,
	.enable_mask = 0x01,
	.enable_reg = MAX77932_REG_SCC_EN,
	.active_discharge_off = 0x00,
	.active_discharge_on = 0x10,
	.active_discharge_mask = 0x10,
	.active_discharge_reg = MAX77932_REG_SCC_CFG1,
};

static const struct regmap_config max77932_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77932_REG_DEVICE_ID,
	.rd_table = &max77932_readable_table,
	.wr_table = &max77932_writable_table,
};

static const struct regmap_irq max77932_irqs[] = {
	REGMAP_IRQ_REG(MAX77932_IRQ_SS_FLT, 0, 0x01),
	REGMAP_IRQ_REG(MAX77932_IRQ_T_SHDN, 0, 0x02),
	REGMAP_IRQ_REG(MAX77932_IRQ_T_ALM2, 0, 0x04),
	REGMAP_IRQ_REG(MAX77932_IRQ_T_ALM1, 0, 0x08),
	REGMAP_IRQ_REG(MAX77932_IRQ_OCP, 0, 0x10),
	REGMAP_IRQ_REG(MAX77932_IRQ_OC_ALM, 0, 0x20),
	REGMAP_IRQ_REG(MAX77932_IRQ_OOVP, 0, 0x40),
	REGMAP_IRQ_REG(MAX77932_IRQ_IOVP, 0, 0x80),
};

static struct regmap_irq_chip max77932_irq_chip = {
	.name = DRIVER_IRQ_NAME,
	.irqs = max77932_irqs,
	.num_irqs = ARRAY_SIZE(max77932_irqs),
	.num_regs = 1,
	.status_base = MAX77932_REG_INT_SRC,
	.mask_base = MAX77932_REG_INT_SRC_M,
};

static int max77932_init_interrupt(struct max77932_chip *pchip,
				   struct device_node *node)
{
	int ret = 0, nirq;

	nirq = of_get_named_gpio(node, "irq-gpio", 0);
	if (nirq > 0) {
		gpio_request_one(nirq, GPIOF_DIR_IN, DRIVER_IRQ_NAME);
		pchip->irq = gpio_to_irq(nirq);
		if (pchip->irq < 0) {
			dev_warn(pchip->dev, "irq can't be configurated\n");
			return -EINVAL;
		}

		max77932_irq_chip.irq_drv_data = pchip;
		ret = devm_regmap_add_irq_chip(pchip->dev, pchip->regmap,
					       pchip->irq,
					       IRQF_TRIGGER_FALLING |
					       IRQF_ONESHOT | IRQF_SHARED, 0,
					       &max77932_irq_chip,
					       &pchip->irq_data);
	}
	return ret;
}

static int max77932_init_regulator(struct max77932_chip *pchip,
				   struct device_node *node)
{
	int ret;
	int ocp1 = 0, ocp2 = 0;
	struct device *dev = pchip->dev;
	struct regulator_config config = { };
	static struct regulator_init_data max77932_init_data;

	ret = of_property_read_u32(node, "maxim,ocp1-threshold_uA", &ocp1);
	if (ocp1 != 0) {
		dev_info(dev, "%s ocp1: %d uA\n", __func__, ocp1);
		max77932_init_data.constraints.max_uA = ocp1;
		config.init_data = &max77932_init_data;
	} else {
		max77932_init_data.constraints.max_uA = 9600000;
	}
	ret = of_property_read_u32(node, "maxim,ocp2-threshold_mV", &ocp2);
	if (ocp2 != 0) {
		dev_info(dev, "%s ocp2: %d uA\n", __func__, ocp2);
		ret = max77932_write_ocp2(pchip, ocp2);
		if (ret < 0)
			return ret;
	}

	config.regmap = pchip->regmap;
	config.driver_data = pchip;
	config.dev = dev;

	pchip->regulator_desc = max77932_reg;
	pchip->regulator = devm_regulator_register(dev, &max77932_reg, &config);
	if (IS_ERR(pchip->regulator)) {
		ret = PTR_ERR(pchip->regulator);
		return ret;
	}
	return 0;
}

#ifdef __TEST_DEVICE_NODE__
static struct max77932_chip *test_if;
static ssize_t max77932_test_store(struct device *dev,
				   struct device_attribute *devAttr,
				   const char *buf, size_t size)
{
	int ret;
	int rval, reg;
	char *args[3];
	char *sep = " ,\t";
	char *buff = (char *)buf;
	struct regmap *if_regmap = test_if->regmap;

	args[0] = strsep(&buff, sep);
	if (args[0] == NULL)
		return -2;

	//register read
	args[1] = strsep(&buff, sep);
	if (strncmp("read", args[0], 4) == 0) {
		reg = (int)simple_strtoul(args[1], NULL, 0);
		ret = regmap_read(if_regmap, reg, &rval);
		if (ret < 0)
			dev_err(test_if->dev,
				"failed to read i2c: %d @ function %s\n", ret,
				__func__);
		else
			dev_err(test_if->dev, "read [0x%x] = 0x%x\n", reg,
				rval);
		return size;
	}
	//register write
	if (strncmp("write", args[0], 5) == 0) {
		reg = (int)simple_strtoul(args[1], NULL, 0);
		args[2] = strsep(&buff, sep);
		rval = (int)simple_strtoul(args[2], NULL, 0);
		ret = regmap_update_bits(if_regmap, reg, 0xff, rval);
		if (ret < 0)
			dev_err(test_if->dev,
				"failed to write i2c: %d @ function %s\n", ret,
				__func__);
		else
			dev_err(test_if->dev, "write [0x%x] = 0x%x\n", reg,
				rval);
		return size;
	}
	//scc enable/disable
	if (strncmp("scc", args[0], 4) == 0) {
		if (strncmp("enable", args[1], 5) == 0) {
			ret = regmap_update_bits(if_regmap,
						 MAX77932_REG_SCC_EN, 0x1, 0x1);
			if (ret < 0)
				dev_err(test_if->dev,
					"failed to write i2c: %d @ function %s\n",
					ret, __func__);
			else
				dev_err(test_if->dev, "SCC ENABLE\n");
		} else {
			ret = regmap_update_bits(if_regmap,
						 MAX77932_REG_SCC_EN, 0x1, 0x0);
			if (ret < 0)
				dev_err(test_if->dev,
					"failed to write i2c: %d @ function %s\n",
					ret, __func__);
			else
				dev_err(test_if->dev, "SCC DIABLE\n");
		}
	}
	//ocp1 configuration
	if (strncmp("ocp1", args[0], 4) == 0) {
		rval = (int)simple_strtoul(args[1], NULL, 0);
		dev_err(test_if->dev, "Set OCP1 %d\n", rval);
		max77932_write_ocp1(test_if, rval);
	}
	//ocp2 configuration
	if (strncmp("ocp2", args[0], 4) == 0) {
		rval = (int)simple_strtoul(args[1], NULL, 0);
		dev_err(test_if->dev, "Set OCP2 %d\n", rval);
		max77932_write_ocp2(test_if, rval);
	}
	return size;
}

static struct device_attribute max77932_attribute = {
	.attr = {
		 .name = "max77932_test_if",
		 .mode = 0x0666,
		 },
	.show = NULL,
	.store = max77932_test_store,
};

static int max77932_test_node(struct max77932_chip *pchip)
{
	int ret;
	struct regulator_dev *rdev = pchip->regulator;

	ret = sysfs_create_file(&rdev->dev.kobj, &max77932_attribute.attr);
	test_if = pchip;
	return ret;
}
#else
static int max77932_test_node(struct max77932_chip *pchip)
{
	return 0;
}
#endif

static int max77932_regulator_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct device_node *node = client->dev.of_node;
	struct device *dev = &client->dev;
	struct max77932_chip *pchip;
	int ret;

	pchip = devm_kzalloc(dev, sizeof(struct max77932_chip), GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;

	i2c_set_clientdata(client, pchip);
	pchip->dev = dev;

	pchip->regmap = devm_regmap_init_i2c(client, &max77932_regmap_config);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(dev, "failed to initialize regmap: %d\n", ret);
		return ret;
	}

	ret = max77932_init_regulator(pchip, node);
	if (ret < 0) {
		dev_err(dev, "failed to register regulator: %d\n", ret);
		return ret;
	}

	ret = max77932_init_interrupt(pchip, node);
	if (ret < 0)
		dev_err(pchip->dev, "failed to add regmap irq: %d\n", ret);

	dev_info(pchip->dev, "max77932 init done\n");
	max77932_test_node(pchip);
	return 0;
}

static const struct i2c_device_id max77932_i2c_id[] = {
	{DRIVER_NAME},
	{}
};

MODULE_DEVICE_TABLE(i2c, max77932_i2c_id);

static const struct of_device_id max77932_of_match[] = {
	{.compatible = "maxim,max77932"},
	{},
};

MODULE_DEVICE_TABLE(of, ltc3676_of_match);

static struct i2c_driver max77932_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = of_match_ptr(max77932_of_match),
		   },
	.probe = max77932_regulator_probe,
	.id_table = max77932_i2c_id,
};

module_i2c_driver(max77932_driver);

MODULE_AUTHOR("Maxim LDD <opensource@maximintegrated.com>");
MODULE_DESCRIPTION("MAX77932 Dual Phase Switched Capacitor Converter");
MODULE_LICENSE("GPL v2");
