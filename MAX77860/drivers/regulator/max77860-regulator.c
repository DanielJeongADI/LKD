// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2021 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/version.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regmap.h>
#include <linux/mfd/max77860.h>
#include <linux/regulator/max77860-regulator.h>
#include <linux/module.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>

#ifdef __TEST_DEVICE_NODE__
#include <linux/string.h>
#include <linux/sysfs.h>
#endif

#define M2SH	__CONST_FFS

/* Register */
/* Safeout */
#define REG_SAFEOUTCTRL		0xC6
#define BIT_SAFEOUT1		BITS(1, 0)
#define BIT_ACTDISSAFEO1	BIT(4)
#define BIT_ENSAFEOUT1		BIT(6)

struct max77860_data {
	struct device *dev;
	struct max77860_dev *iodev;
	struct regulator_dev *rdev;
};

static unsigned int max77860_safeout_volt_table[] = {
	4850000, 4900000, 4950000, 3300000,
};

static struct regulator_ops max77860_safeout_ops = {
	.list_voltage		= regulator_list_voltage_table,
	.map_voltage		= regulator_map_voltage_ascend,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
};

static struct regulator_desc max77860_safeout_desc = {	
	.name		= "SAFEOUT1",	
	.id		= MAX77860_SAFEOUT1,
	.ops		= &max77860_safeout_ops,
	.type		= REGULATOR_VOLTAGE,	
	.owner		= THIS_MODULE,			
	.n_voltages	= ARRAY_SIZE(max77860_safeout_volt_table),
	.volt_table	= max77860_safeout_volt_table,
	.vsel_reg	= REG_SAFEOUTCTRL,	
	.vsel_mask	= BIT_SAFEOUT1,				
	.enable_reg	= REG_SAFEOUTCTRL,	
	.enable_mask	= BIT_ENSAFEOUT1,		
};

#ifdef CONFIG_OF
	static struct max77860_regulator_platform_data
*max77860_regulator_parse_dt(struct device *dev)
{
	struct device_node *np = of_find_node_by_name(NULL, "regulator");
	struct max77860_regulator_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(pdata == NULL))
		return ERR_PTR(-ENOMEM);

	if (np == NULL) {
		pr_err("%s np NULL\n", __func__);
		pdata = ERR_PTR(-EINVAL);
		goto out;
	} 
	
	if (!of_node_cmp(np->name,
		max77860_safeout_desc.name))

	pdata->initdata = of_get_regulator_init_data(dev, np,
		&max77860_safeout_desc);
	pdata->of_node = np;
	of_node_put(np);

out:
	return pdata;
}
#endif

#ifdef __TEST_DEVICE_NODE__
static struct max77860_data *test_if;
static ssize_t max77860_test_store(struct device *dev,
			struct device_attribute *devAttr,
			const char *buf, size_t size)
{
	int ret, reg;
	u8 rval;
	char *args[3];
	char *sep = ",\t";
	char *buff = (char *)buf;
	struct regmap *if_regmap = test_if->iodev->regmap_pmic;
	
	args[0] = strsep(&buff, sep);
	if (args[0] == NULL)
		return -2;
		
	// register read
	args[1] = strsep(&buff, sep);
	if(strncmp("read", args[0], 4) == 0){
		reg = (int)simple_strtoul(args[1], NULL, 0);
		ret = max77860_read(if_regmap, reg, &rval);
		if (ret < 0)
			dev_err(test_if->dev, "failed to read i2c: %d @ function %s\n", ret, __func__);
		else
			dev_err(test_if->dev, "read [0x%x] = 0x%x\n", reg, rval);
		return size;
	}
	// register write
	else if(strncmp("write", args[0], 5) == 0){
		reg = (int)simple_strtoul(args[1], NULL, 0);
		args[2] = strsep(&buff, sep);
		rval = (int)simple_strtoul(args[2], NULL, 0);
		ret = max77860_write(if_regmap, reg, rval);
		if (ret < 0)
			dev_err(test_if->dev, "failed to write i2c: %d @ function %s\n", ret, __func__);
		else
			dev_err(test_if->dev, "write [0x%x] = 0x%x\n", reg, rval);
		return size;
	}
	else
		dev_err(test_if->dev, "Command not supported.\n");
	
	return 0;
}

static struct device_attribute max77860_attribute = {
		.attr = {
			.name = "max77860_test_if",
			.mode = 0x0666,
			},
		.show = NULL,
		.store = max77860_test_store,
};

static int max77860_test_node(struct max77860_data *max77860)
{	
	int ret;
	
	ret = sysfs_create_file(&max77860->dev->kobj, &max77860_attribute.attr);
	test_if = max77860;
	return ret;
}
#else
static int max77860_test_node(struct max77860_data *pmic)
{	
	return 0;
}
#endif

static int max77860_regulator_probe(struct platform_device *pdev)
{
	struct max77860_dev *iodev;
	struct max77860_regulator_platform_data *pdata;
	struct regulator_dev *rdev;
	struct max77860_data *max77860;
	struct regmap *regmap;
	struct regulator_config config;
	int ret, size;

	dev_info(&pdev->dev, "%s\n", __func__);
	
	iodev = dev_get_drvdata(pdev->dev.parent);
	pdata =	dev_get_platdata(&pdev->dev);

#ifdef CONFIG_OF
	pdata = max77860_regulator_parse_dt(&pdev->dev);
#endif
	if (unlikely(IS_ERR(pdata))) {
		pr_info("[%s:%d] !pdata\n", __FILE__, __LINE__);
		dev_err(pdev->dev.parent, "No platform init data supplied.\n");
		return PTR_ERR(pdata);
	}

	max77860 = kzalloc(sizeof(struct max77860_data), GFP_KERNEL);
	if (!max77860)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *);
	max77860->rdev = kzalloc(size, GFP_KERNEL);
	if (!max77860->rdev) {
		pr_info("[%s:%d] if (!max77860->rdev)\n", __FILE__, __LINE__);
		kfree(max77860);
		return -ENOMEM;
	}

	rdev = max77860->rdev;
	max77860->dev = &pdev->dev;
	max77860->iodev = iodev;
	platform_set_drvdata(pdev, max77860);
	regmap = max77860->iodev->regmap_pmic;

	config.dev = &pdev->dev;
	config.driver_data = max77860;
	config.init_data = pdata->initdata;
	config.of_node = pdata->of_node;
	config.regmap = regmap;
	rdev = regulator_register(&max77860_safeout_desc,
		&config);

	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(max77860->dev, "regulator init failed!\n");
		rdev = NULL;
		goto err;
	}

	max77860_test_node(max77860);
	return 0;
err:
	kfree(max77860->rdev);
	kfree(max77860);

	return ret;
}

static int max77860_regulator_remove(struct platform_device *pdev)
{
	struct max77860_data *max77860 = platform_get_drvdata(pdev);
	struct regulator_dev *rdev = max77860->rdev;

	dev_info(&pdev->dev, "%s\n", __func__);
	regulator_unregister(rdev);
	kfree(max77860->rdev);
	kfree(max77860);

	return 0;
}

static const struct platform_device_id max77860_regulator_id[] = {
	{"max77860-regulator", 0},
	{},
};

MODULE_DEVICE_TABLE(platform, max77860_regulator_id);

static struct platform_driver max77860_regulator_driver = {
	.driver = {
		.name = MAX77860_REGULATOR_NAME,
		.owner = THIS_MODULE,
	},
	.probe = max77860_regulator_probe,
	.remove = max77860_regulator_remove,
	.id_table = max77860_regulator_id,
};

static int __init max77860_regulator_init(void)
{
	pr_err("%s\n", __func__);
	return platform_driver_register(&max77860_regulator_driver);
}

static void __exit max77860_regulator_exit(void)
{
	platform_driver_unregister(&max77860_regulator_driver);
}

subsys_initcall(max77860_regulator_init);
module_exit(max77860_regulator_exit);

MODULE_DESCRIPTION("MAXIM 77860 Regulator Driver");
MODULE_AUTHOR("opensource@maximintegrated.com ");
MODULE_LICENSE("GPL");
