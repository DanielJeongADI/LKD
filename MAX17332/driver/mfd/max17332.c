// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2021 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/string.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max17332.h>

#define DRIVER_DESC    "MAX17332 MFD Driver"
#define DRIVER_NAME    MAX17332_NAME
#define DRIVER_VERSION "1.2"
#define DRIVER_AUTHOR  "opensource@maximintegrated.com"

#define I2C_ADDR_PMIC		(0x6C >> 1) /* Model Gauge */
#define I2C_ADDR_NVM		(0x16 >> 1) /* NVM */

#define __lock(_me)    mutex_lock(&(_me)->lock)
#define __unlock(_me)  mutex_unlock(&(_me)->lock)

static const struct regmap_config max17332_regmap_config = {
	.reg_bits   = 8,
	.val_bits   = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config max17332_regmap_config_nvm = {
	.reg_bits   = 8,
	.val_bits   = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.cache_type = REGCACHE_NONE,
};

/*******************************************************************************
 * Chip IO
 ******************************************************************************/
int max17332_read(struct regmap *regmap, u8 addr, u16 *val)
{
	unsigned int buf = 0;
	int rc;

	rc = regmap_read(regmap, (unsigned int)addr, &buf);

	if (!IS_ERR_VALUE(rc))
		*val = (u16)buf;
	else
		pr_err("%s: regmap_read returns error no: %d\n", __func__, rc);

	return rc;
}
EXPORT_SYMBOL(max17332_read);

int max17332_write(struct regmap *regmap, u8 addr, u16 val)
{
	int ret;

	ret = regmap_write(regmap, (unsigned int)addr, (unsigned int)val);
	if (ret < 0)
		pr_err("%s: regmap_write returns error no: %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL(max17332_write);

int max17332_update_bits(struct regmap *regmap, u8 addr, u16 mask, u16 val)
{
	int ret;

	ret = regmap_update_bits(regmap, (unsigned int)addr,
							(unsigned int)mask, (unsigned int)val);
	if (ret < 0)
		pr_err("%s: regmap_update_bits returns error no: %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL(max17332_update_bits);

/* Declare Interrupt */
static const struct regmap_irq max17332_intsrc_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_STATUS_PA,},
	{ .reg_offset = 0, .mask = BIT_STATUS_SMX,},
	{ .reg_offset = 0, .mask = BIT_STATUS_TMX,},
	{ .reg_offset = 0, .mask = BIT_STATUS_VMX,},
	{ .reg_offset = 0, .mask = BIT_STATUS_CA,},
	{ .reg_offset = 0, .mask = BIT_STATUS_SMN,},
	{ .reg_offset = 0, .mask = BIT_STATUS_TMN,},
	{ .reg_offset = 0, .mask = BIT_STATUS_VMN,},
	{ .reg_offset = 0, .mask = BIT_STATUS_DSOCI,},
	{ .reg_offset = 0, .mask = BIT_STATUS_IMX,},
	{ .reg_offset = 0, .mask = BIT_STATUS_ALLOWCHGB,},
	{ .reg_offset = 0, .mask = BIT_STATUS_BST,},
	{ .reg_offset = 0, .mask = BIT_STATUS_IMN,},
	{ .reg_offset = 0, .mask = BIT_STATUS_POR,},
};

static const struct regmap_irq_chip max17332_intsrc_irq_chip = {
	.name = "max17332 intsrc",
	.status_base = REG_STATUS,
	.mask_base = REG_STATUS_MASK,
	.num_regs = 1,
	.irqs = max17332_intsrc_irqs,
	.num_irqs = ARRAY_SIZE(max17332_intsrc_irqs),
};

static void max17332_lock_write_protection(struct max17332_dev *dev, bool lock_en)
{
	int ret;
	u16 val;

	ret = max17332_read(dev->regmap_pmic, REG_COMMSTAT, &val);
	if (ret < 0)
		goto err;

	if (lock_en) {
		/* lock write protection */
		ret = max17332_write(dev->regmap_pmic, REG_COMMSTAT, val | 0x00F9);
		ret |= max17332_write(dev->regmap_pmic, REG_COMMSTAT, val | 0x00F9);
	} else {
		/* unlock write protection */
		ret = max17332_write(dev->regmap_pmic, REG_COMMSTAT, val & 0xFF06);
		ret |= max17332_write(dev->regmap_pmic, REG_COMMSTAT, val & 0xFF06);
	}
	if (ret < 0)
		goto err;

	ret = max17332_read(dev->regmap_pmic, REG_COMMSTAT, &val);
	if (ret < 0)
		goto err;

	pr_info("%s: CommStat : 0x%04X\n", __func__, val);
	return;
err:
	pr_err("<%s> failed\n", __func__);
}

int max17332_map_irq(struct max17332_dev *max17332, int irq)
{
	return regmap_irq_get_virq(max17332->irqc_intsrc, irq);
}
EXPORT_SYMBOL_GPL(max17332_map_irq);

static int max17332_pmic_irq_int(struct max17332_dev *chip)
{
	struct device *dev = chip->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;

	/* disable all interrupt source */
	max17332_write(chip->regmap_pmic, REG_PROGALRTS, 0xFFFF);

	/* interrupt source */
	rc = regmap_add_irq_chip(chip->regmap_pmic, chip->irq,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
			IRQF_SHARED, -1, &max17332_intsrc_irq_chip,
			&chip->irqc_intsrc);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add insrc irq chip: %d\n",
			rc);
		goto out;
	}

	pr_info("<%s> IRQ initialize done\n", client->name);
	return 0;

out:
	return rc;
}


/*******************************************************************************
 *  device
 ******************************************************************************/
static int max17332_add_devices(struct max17332_dev *me,
		struct mfd_cell *cells, int n_devs)
{
	struct device *dev = me->dev;
	int rc;

	pr_info("%s: size %d\n", __func__, n_devs);
	rc = mfd_add_devices(dev, -1, cells, n_devs, NULL, 0, NULL);

	return rc;
}

static struct mfd_cell max17332_devices[] = {
	{ .name = MAX17332_BATTERY_NAME,	},
	{ .name = MAX17332_CHARGER_NAME,	},
};

static int max17332_pre_init_data(struct max17332_dev *pmic)
{
#ifdef CONFIG_OF
	int size = 0, cnt = 0;
	struct device *dev = pmic->dev;
	struct device_node *np = dev->of_node;

	u16 *init_data;

	pr_info("%s: Pmic initialize\n", __func__);
	size = device_property_read_u16_array(dev, "max17332,pmic-init", NULL, 0);
	if (size > 0) {
		init_data = kmalloc(size, GFP_KERNEL);
		of_property_read_u16_array(np, "max17332,pmic-init", init_data, size);
		for (cnt = 0; cnt < size; cnt += 2) {
			max17332_write(pmic->regmap_pmic,
				(u8)(init_data[cnt]), init_data[cnt+1]);
		}
		kfree(init_data);
	}

	pr_info("%s: nvm initialize\n", __func__);
	size = device_property_read_u16_array(dev, "max17332,nvm-init", NULL, 0);
	if (size > 0) {
		init_data = kmalloc(size, GFP_KERNEL);
		of_property_read_u16_array(np, "max17332,nvm-init", init_data, size);
		for (cnt = 0; cnt < size; cnt += 2) {
			if (((u8)(init_data[cnt]) >= REG_NROMID0_NVM) && ((u8)(init_data[cnt]) <= REG_NROMID3_NVM))
				continue;
			max17332_write(pmic->regmap_nvm,
				(u8)(init_data[cnt]), init_data[cnt+1]);
		}
		kfree(init_data);

		max17332_update_bits(pmic->regmap_pmic, REG_CONFIG2,
			MAX17332_CONFIG2_POR_CMD,
			MAX17332_CONFIG2_POR_CMD);

		// Wait 500 ms for POR_CMD to clear;
		mdelay(500);
	}

#endif
	return 0;
}

static void *max17332_pmic_get_platdata(struct max17332_dev *pmic)
{
#ifdef CONFIG_OF
	struct device *dev = pmic->dev;
	struct device_node *np = dev->of_node;
	struct i2c_client *client = to_i2c_client(dev);
	struct max17332_pmic_platform_data *pdata;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		pr_err("<%s> out of memory (%uB requested)\n", client->name,
				(unsigned int) sizeof(*pdata));
		pdata = ERR_PTR(-ENOMEM);
		goto out;
	}

	pr_info("%s rsense\n", __func__);
	ret = of_property_read_u32(np, "rsense", &pdata->rsense);
	if (ret < 0)
		pdata->rsense = 10;

out:
	return pdata;
#else /* CONFIG_OF */
	return dev_get_platdata(pmic->dev) ?
		dev_get_platdata(pmic->dev) : ERR_PTR(-EINVAL);
#endif /* CONFIG_OF */
}

static int max17332_pmic_setup(struct max17332_dev *me)
{
	struct device *dev = me->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;

	pr_info("%s: max17332_pmic_get_platdata\n", __func__);
	me->pdata = max17332_pmic_get_platdata(me);
	if (IS_ERR(me->pdata)) {
		rc = PTR_ERR(me->pdata);
		me->pdata = NULL;
		pr_err("<%s> platform data is missing [%d]\n",
			client->name, rc);
		goto out;
	}

	pr_info("%s: max17332_pmic_irq_int\n", __func__);
	/* IRQ init */
	rc = max17332_pmic_irq_int(me);
	if (rc != 0) {
		dev_err(&client->dev, "failed to initialize irq: %d\n", rc);
		goto err_irq_init;
	}

	pr_info("%s: max17332_add_devices\n", __func__);
	rc = max17332_add_devices(me, max17332_devices,
			ARRAY_SIZE(max17332_devices));
	if (IS_ERR_VALUE(rc)) {
		pr_err("<%s> failed to add sub-devices [%d]\n",
			client->name, rc);
		goto err_add_devices;
	}

	/* set device able to wake up system */
	device_init_wakeup(dev, true);
	enable_irq_wake((unsigned int)me->irq);

	pr_info("%s: Done\n", __func__);
	return 0;

err_add_devices:
	regmap_del_irq_chip(me->irq, me->irqc_intsrc);
err_irq_init:
out:
	return rc;
}

/*******************************************************************************
 *** MAX17332 MFD Core
 ******************************************************************************/

static __always_inline void max17332_destroy(struct max17332_dev *me)
{
	struct device *dev = me->dev;

	mfd_remove_devices(me->dev);

	if (likely(me->irq > 0))
		regmap_del_irq_chip(me->irq, me->irqc_intsrc);

	if (likely(me->irq_gpio >= 0))
		gpio_free((unsigned int)me->irq_gpio);

	if (likely(me->regmap_pmic))
		regmap_exit(me->regmap_pmic);

	if (likely(me->regmap_nvm))
		regmap_exit(me->regmap_nvm);

#ifdef CONFIG_OF
	if (likely(me->pdata))
		devm_kfree(dev, me->pdata);
#endif /* CONFIG_OF */

	mutex_destroy(&me->lock);
	devm_kfree(dev, me);
}

static int max17332_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct max17332_dev *me;
	int rc;

	pr_info("%s: Max17332 I2C Driver Loading\n", __func__);

	me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
	if (unlikely(!me)) {
		pr_err("<%s> out of memory (%uB requested)\n", client->name,
				(unsigned int) sizeof(*me));
		return -ENOMEM;
	}

	i2c_set_clientdata(client, me);

	mutex_init(&me->lock);
	me->dev      = &client->dev;
	me->irq      = client->irq;
	me->irq_gpio = -1;

	me->pmic = client;

	me->regmap_pmic = devm_regmap_init_i2c(client, &max17332_regmap_config);
	if (IS_ERR(me->regmap_pmic)) {
		rc = PTR_ERR(me->regmap_pmic);
		me->regmap_pmic = NULL;
		pr_err("<%s> failed to initialize i2c\n",
			client->name);
		pr_err("<%s> regmap pmic [%d]\n",
			client->name,	rc);
		goto abort;
	}

	me->nvm = i2c_new_dummy(client->adapter, I2C_ADDR_NVM);
	if (!me->nvm) {
		rc = -ENOMEM;
		goto abort;
	}

	i2c_set_clientdata(me->nvm, me);
	me->regmap_nvm = devm_regmap_init_i2c(me->nvm, &max17332_regmap_config_nvm);
	if (IS_ERR(me->regmap_nvm)) {
		rc = PTR_ERR(me->regmap_nvm);
		me->regmap_nvm = NULL;
		pr_err("<%s> failed to initialize i2c\n",
			client->name);
		pr_err("<%s> regmap nvm [%d]\n",
			client->name,	rc);
		goto abort;
	}

	/* unlock Write Protection */
	max17332_lock_write_protection(me, false);

	max17332_pre_init_data(me);
	rc = max17332_pmic_setup(me);
	if (rc != 0) {
		pr_err("<%s> failed to set up interrupt\n",
			client->name);
		pr_err("<%s> and add sub-device [%d]\n",
			client->name,	rc);
		goto abort;
	}

	pr_info("%s: Done\n", __func__);
	return 0;
abort:
	pr_err("%s: Failed to probe\n", __func__);
	i2c_set_clientdata(client, NULL);
	max17332_destroy(me);
	return rc;
}

static int max17332_i2c_remove(struct i2c_client *client)
{
	struct max17332_dev *me = i2c_get_clientdata(client);

	i2c_set_clientdata(client, NULL);
	max17332_destroy(me);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max17332_suspend(struct device *dev)
{
	struct max17332_dev *me = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);

	__lock(me);

	pr_info("<%s> suspending\n", client->name);

	__unlock(me);
	return 0;
}

static int max17332_resume(struct device *dev)
{
	struct max17332_dev *me = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);

	__lock(me);

	pr_info("<%s> resuming\n", client->name);

	__unlock(me);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max17332_pm, max17332_suspend, max17332_resume);

#ifdef CONFIG_OF
static const struct of_device_id max17332_of_id[] = {
	{ .compatible = "maxim,max17332"},
	{ },
};
MODULE_DEVICE_TABLE(of, max17332_of_id);
#endif /* CONFIG_OF */

static const struct i2c_device_id max17332_i2c_id[] = {
	{ MAX17332_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max17332_i2c_id);

static struct i2c_driver max17332_i2c_driver = {
	.driver.name            = DRIVER_NAME,
	.driver.owner           = THIS_MODULE,
	.driver.pm              = &max17332_pm,
#ifdef CONFIG_OF
	.driver.of_match_table  = max17332_of_id,
#endif /* CONFIG_OF */
	.id_table               = max17332_i2c_id,
	.probe                  = max17332_i2c_probe,
	.remove                 = max17332_i2c_remove,
};

static __init int max17332_init(void)
{
	int rc = -ENODEV;

	rc = i2c_add_driver(&max17332_i2c_driver);
	if (rc != 0)
		pr_err("Failed to register I2C driver: %d\n", rc);
	pr_info("%s: Added I2C Driver\n", __func__);
	return rc;
}
module_init(max17332_init);

static __exit void max17332_exit(void)
{
	i2c_del_driver(&max17332_i2c_driver);
}
module_exit(max17332_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
