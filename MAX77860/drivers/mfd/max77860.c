// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2021 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <ugur.usug@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
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
#include <linux/mfd/max77860.h>

#define DRIVER_VERSION	"1.0"

#define I2C_ADDR_PMIC		(0xCC >> 1) /* PMIC (CLOGIC/SAFELDOs) */
#define I2C_ADDR_CHARGER	(0xD2 >> 1) /* Charger, ADC */
#define I2C_ADDR_USBC		(0x4A >> 1) /* USBC */

#define __lock(_chip)    mutex_lock(&(_chip)->lock)
#define __unlock(_chip)  mutex_unlock(&(_chip)->lock)

static const struct regmap_config max77860_regmap_config = {
	.reg_bits   = 8,
	.val_bits   = 8,
	.cache_type = REGCACHE_NONE,
};

/*******************************************************************************
 * Chip IO
 ******************************************************************************/
int max77860_read(struct regmap *regmap, u8 addr, u8 *val)
{
	unsigned int buf = 0;
	int rc = regmap_read(regmap, (unsigned int)addr, &buf);

	if (!IS_ERR_VALUE(rc))
		*val = (u8)buf;
	return rc;
}
EXPORT_SYMBOL(max77860_read);

int max77860_write(struct regmap *regmap, u8 addr, u8 val)
{
	unsigned int buf = (unsigned int)val;

	return regmap_write(regmap, (unsigned int)addr, buf);
}
EXPORT_SYMBOL(max77860_write);

int max77860_bulk_read(struct regmap *regmap, u8 addr, u8 *dst, u16 len)
{
	return regmap_bulk_read(regmap, (unsigned int)addr, dst, (size_t)len);
}
EXPORT_SYMBOL(max77860_bulk_read);

int max77860_bulk_write(struct regmap *regmap, u8 addr, const u8 *src, u16 len)
{
	return regmap_bulk_write(regmap, (unsigned int)addr, src, (size_t)len);
}
EXPORT_SYMBOL(max77860_bulk_write);

/*******************************************************************************
 *  device
 ******************************************************************************/
static int max77860_add_devices(struct max77860_dev *chip,
				struct mfd_cell *cells, int n_devs)
{
	struct device *dev = chip->dev;
	int rc;

	rc = mfd_add_devices(dev, -1, cells, n_devs, NULL, 0, NULL);

	return rc;
}

/*******************************************************************************
 *** max77860 PMIC
 ******************************************************************************/

/* Declare Interrupt */
static const struct regmap_irq max77860_intsrc_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_CHGR_INT,},	/* CHGR_INT */
	{ .reg_offset = 0, .mask = BIT_SYS_INT,},	/* SYS_INT */
	{ .reg_offset = 0, .mask = BIT_USBC_INT,},	/* USBC_INT */
	{ .reg_offset = 0, .mask = BIT_B2SOVRC_INT,},	/* B2SOVRC_INT */
	{ .reg_offset = 0, .mask = BIT_SLAVE_INT,},	/* SLAVE_INT */
};

static const struct regmap_irq_chip max77860_intsrc_irq_chip = {
	.name = "max77860 intsrc",
	.status_base = REG_INTSRC,
	.mask_base = REG_INTSRCMASK,
	.num_regs = 1,
	.irqs = max77860_intsrc_irqs,
	.num_irqs = ARRAY_SIZE(max77860_intsrc_irqs),
};

static const struct regmap_irq max77860_sys_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_T120C_INT,},	/* T120C_INT */
	{ .reg_offset = 0, .mask = BIT_T140C_INT,},	/* T140C_INT */
	{ .reg_offset = 0, .mask = BIT_LOWSYS_INT,},	/* LOWSYS_INT */
	{ .reg_offset = 0, .mask = BIT_SYSUVLO_INT,},	/* SYSUVLO_INT */
	{ .reg_offset = 0, .mask = BIT_SYSOVLO_INT,},	/* SYSOVLO_INT */
	{ .reg_offset = 0, .mask = BIT_TSHDN_INT,},	/* TSHDN_INT */
};

static const struct regmap_irq_chip max77860_sys_irq_chip = {
	.name = "max77860 system",
	.status_base = REG_SYSINTSRC,
	.mask_base = REG_SYSINTMASK,
	.num_regs = 1,
	.irqs = max77860_sys_irqs,
	.num_irqs = ARRAY_SIZE(max77860_sys_irqs),
};

static const struct regmap_irq max77860_chg_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_CHG_BYP_I,},	/* BYP_I */
	{ .reg_offset = 0, .mask = BIT_CHG_BAT2SOC_I,},	/* BAT2SOC_I */
	{ .reg_offset = 0, .mask = BIT_CHG_BATP_I,},	/* BATP_I */
	{ .reg_offset = 0, .mask = BIT_CHG_BAT_I,},	/* BAT_I */
	{ .reg_offset = 0, .mask = BIT_CHG_CHG_I,},	/* CHG_I */
	{ .reg_offset = 0, .mask = BIT_CHG_TOPOFF_I,},	/* TOPOFF_I */
	{ .reg_offset = 0, .mask = BIT_CHG_CHGIN_I,},	/* CHGIN_I */
	{ .reg_offset = 0, .mask = BIT_CHG_AICL_I,},	/* AICL_I */
};

static const struct regmap_irq_chip max77860_chg_irq_chip = {
	.name = "max77860 chg",
	.status_base = REG_CHARGER_INT,
	.mask_base = REG_CHARGER_INT_MASK,
	.num_regs = 1,
	.irqs = max77860_chg_irqs,
	.num_irqs = ARRAY_SIZE(max77860_chg_irqs),
};

int max77860_map_irq(struct max77860_dev *max77860, int irq)
{
	return regmap_irq_get_virq(max77860->irqc_intsrc, irq);
}
EXPORT_SYMBOL_GPL(max77860_map_irq);

int max77860_map_sys_irq(struct max77860_dev *max77860, int irq)
{
	return regmap_irq_get_virq(max77860->irqc_sys, irq);
}
EXPORT_SYMBOL_GPL(max77860_map_sys_irq);

int max77860_map_chg_irq(struct max77860_dev *max77860, int irq)
{
	return regmap_irq_get_virq(max77860->irqc_chg, irq);
}
EXPORT_SYMBOL_GPL(max77860_map_chg_irq);

int max77860_map_usbc_irq(struct max77860_dev *max77860, int irq)
{
	return regmap_irq_get_virq(max77860->irqc_usbc, irq);
}
EXPORT_SYMBOL_GPL(max77860_map_usbc_irq);

static int max77860_pmic_irq_int(struct max77860_dev *chip)
{
	struct device *dev = chip->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;

	/* disable all interrupt source */
	max77860_write(chip->regmap_pmic, REG_INTSRCMASK, 0xFF);

	/* interrupt source */
	rc = regmap_add_irq_chip(chip->regmap_pmic, chip->irq,
				 IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
				 IRQF_SHARED, -1, &max77860_intsrc_irq_chip,
				 &chip->irqc_intsrc);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add insrc irq chip: %d\n", rc);
		goto out;
	}

	/* system interrupt */
	rc = regmap_add_irq_chip(chip->regmap_pmic, chip->irq,
				 IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
				 IRQF_SHARED, -1, &max77860_sys_irq_chip,
				 &chip->irqc_sys);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add system irq chip: %d\n",
			rc);
		goto err_irqc_sys;
	}

	/* charger interrupt */
	rc = regmap_add_irq_chip(chip->regmap_chg, chip->irq,
				 IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
				 IRQF_SHARED, -1, &max77860_chg_irq_chip,
				 &chip->irqc_chg);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add chg irq chip: %d\n", rc);
		goto err_irqc_chg;
	}

	pr_err("<%s> IRQ initialize done\n", client->name);
	return 0;

err_irqc_chg:
	regmap_del_irq_chip(chip->irq, chip->irqc_sys);
err_irqc_sys:
	regmap_del_irq_chip(chip->irq, chip->irqc_intsrc);
out:
	return rc;
}

static int max77860_pre_init_data(struct max77860_dev *pmic)
{
#ifdef CONFIG_OF
	int size, cnt;
	const int *list;
	struct device *dev = pmic->dev;
	struct device_node *np = dev->of_node;

	u8 *init_data;

	list = of_get_property(np, "max77860,pmic-init", &size);
	if (list) {
		init_data = kmalloc(size, GFP_KERNEL);
		of_property_read_u8_array(np, "max77860,pmic-init",
					  init_data, size);
		for (cnt = 0; cnt < size; cnt += 2) {
			max77860_write(pmic->regmap_pmic,
				       init_data[cnt], init_data[cnt + 1]);
		}
		kfree(init_data);
	}

	list = of_get_property(np, "max77860,chg-init", &size);
	if (list) {
		init_data = kmalloc(size, GFP_KERNEL);
		of_property_read_u8_array(np, "max77860,chg-init",
					  init_data, size);
		for (cnt = 0; cnt < size; cnt += 2) {
			max77860_write(pmic->regmap_chg,
				       init_data[cnt], init_data[cnt + 1]);
		}
		kfree(init_data);
	}

	list = of_get_property(np, "max77860,usbc-init", &size);
	if (list) {
		init_data = kmalloc(size, GFP_KERNEL);
		of_property_read_u8_array(np, "max77860,usbc-init",
					  init_data, size);
		for (cnt = 0; cnt < size; cnt += 2) {
			max77860_write(pmic->regmap_usbc,
				       init_data[cnt], init_data[cnt + 1]);
		}
		kfree(init_data);
	}
#endif
	return 0;
}

static struct mfd_cell max77860_devices[] = {
	{ .name = MAX77860_REGULATOR_NAME,	},
	{ .name = MAX77860_CHARGER_NAME,	},
	{ .name = MAX77860_USBC_NAME,	        },
	{ .name = MAX77860_ADC_NAME,	        },
};

static int max77860_pmic_setup(struct max77860_dev *chip)
{
	struct device *dev = chip->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;
	u8 chip_id;
	u8 val = 0;

	/* IRQ init */
	pr_info("%s: max77860_pmic_irq_int\n", __func__);
	pr_info("<%s> property:IRQ %d\n", client->name, chip->irq);

	rc = max77860_pmic_irq_int(chip);
	if (rc != 0) {
		dev_err(&client->dev, "failed to initialize irq: %d\n", rc);
		goto err_irq_init;
	}

	pr_info("%s: max77860_add_devices\n", __func__);
	rc = max77860_add_devices(chip, max77860_devices,
				  ARRAY_SIZE(max77860_devices));
	if (IS_ERR_VALUE(rc)) {
		pr_err("<%s> failed to add sub-devices [%d]\n",
		       client->name, rc);
		goto err_add_devices;
	}

	pr_info("<%s> driver core " DRIVER_VERSION " installed\n",
		client->name);

	chip_id = 0;

	max77860_read(chip->regmap_pmic, REG_PMICID,  &chip_id);
	pr_info("<%s> pmic id %Xh\n", client->name, chip_id);

	/* clear IRQ */
	max77860_read(chip->regmap_pmic, REG_INTSRC, &val);
	pr_info("<%s> intsrc %Xh\n", client->name, val);

	max77860_read(chip->regmap_pmic, REG_INTSRCMASK, &val);
	pr_info("<%s> intsrc_mask %Xh\n", client->name, val);

	val  &= ~(BIT_CHGR_INT | BIT_USBC_INT);
	rc = max77860_write(chip->regmap_pmic, REG_INTSRCMASK, val);
	if (IS_ERR_VALUE(rc)) {
		pr_err("REG_INTSRCMASK write error [%d]\n", rc);
		goto err_irq_init;
	}

	val = 0;
	max77860_read(chip->regmap_pmic, REG_INTSRCMASK, &val);
	pr_info("<%s> intsrc_mask %Xh\n", client->name, val);

	/* set device able to wake up system */
	device_init_wakeup(dev, true);	//TODO Ugur check
	enable_irq_wake((unsigned int)chip->irq);

	pr_info("%s: Done\n", __func__);

	return 0;

err_add_devices:
	regmap_del_irq_chip(chip->irq, chip->irqc_intsrc);
err_irq_init:
	return rc;
}

/*******************************************************************************
 *** max77860 MFD Core
 ******************************************************************************/

static __always_inline void max77860_destroy(struct max77860_dev *chip)
{
	struct device *dev = chip->dev;

	mfd_remove_devices(chip->dev);

	if (likely(chip->irq > 0))
		regmap_del_irq_chip(chip->irq, chip->irqc_intsrc);

	if (likely(chip->irq_gpio >= 0))
		gpio_free((unsigned int)chip->irq_gpio);

	if (likely(chip->regmap_pmic))
		regmap_exit(chip->regmap_pmic);

	if (likely(chip->regmap_usbc))
		regmap_exit(chip->regmap_usbc);

	if (likely(chip->regmap_chg))
		regmap_exit(chip->regmap_chg);

	mutex_destroy(&chip->lock);
	devm_kfree(dev, chip);
}

static int max77860_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77860_dev *chip;
	int rc;

	pr_info("%s: max77860 Driver Loading\n", __func__);

	if (client->irq <= 0) {
		dev_err(dev, "No interrupt support, no core IRQ\n");
		return -EINVAL;
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (unlikely(!chip)) {
		pr_err("<%s> out of memory (%uB requested)\n", client->name,
		       (unsigned int)sizeof(*chip));
		return -ENOMEM;
	}

	i2c_set_clientdata(client, chip);

	chip->dev      = dev;
	chip->irq      = client->irq;
	chip->irq_gpio = -1;
	chip->pmic = client;
	mutex_init(&chip->lock);

	chip->regmap_pmic = devm_regmap_init_i2c(client,
						 &max77860_regmap_config);
	if (IS_ERR(chip->regmap_pmic)) {
		rc = PTR_ERR(chip->regmap_pmic);
		chip->regmap_pmic = NULL;
		pr_err("<%s> failed to initialize i2c\n", client->name);
		pr_err("<%s> regmap pmic [%d]\n", client->name,	rc);
		goto abort;
	}

	chip->chg = i2c_new_dummy(client->adapter, I2C_ADDR_CHARGER);
	if (!chip->chg) {
		rc = -ENOMEM;
		goto abort;
	}
	i2c_set_clientdata(chip->chg, chip);
	chip->regmap_chg = devm_regmap_init_i2c(chip->chg,
						&max77860_regmap_config);
	if (IS_ERR(chip->regmap_chg)) {
		rc = PTR_ERR(chip->regmap_chg);
		chip->regmap_chg = NULL;
		pr_err("<%s> failed to initialize i2c\n", client->name);
		pr_err("<%s> regmap chg [%d]\n", client->name, rc);
		goto abort;
	}

	chip->usbc = i2c_new_dummy(client->adapter, I2C_ADDR_USBC);
	if (!chip->usbc) {
		rc = -ENOMEM;
		goto abort;
	}
	i2c_set_clientdata(chip->usbc, chip);
	chip->regmap_usbc = devm_regmap_init_i2c(chip->usbc,
						 &max77860_regmap_config);
	if (IS_ERR(chip->regmap_usbc)) {
		rc = PTR_ERR(chip->regmap_usbc);
		chip->regmap_usbc = NULL;
		pr_err("<%s> failed to initialize i2c\n", client->name);
		pr_err("<%s> regmap chgdet [%d]\n", client->name, rc);
		goto abort;
	}

	max77860_pre_init_data(chip);
	rc = max77860_pmic_setup(chip);
	if (rc != 0) {
		pr_err("<%s> failed to set up interrupt\n", client->name);
		pr_err("<%s> and add sub-device [%d]\n", client->name, rc);
		goto abort;
	}

	return 0;

abort:
	pr_err("%s: Failed to probe max77860\n", __func__);
	i2c_set_clientdata(client, NULL);
	max77860_destroy(chip);
	return rc;
}

static int max77860_remove(struct i2c_client *client)
{
	struct max77860_dev *chip = i2c_get_clientdata(client);

	i2c_set_clientdata(client, NULL);
	max77860_destroy(chip);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77860_suspend(struct device *dev)
{
	struct max77860_dev *chip = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);

	__lock(chip);

	pr_info("<%s> suspending\n", client->name);

	__unlock(chip);
	return 0;
}

static int max77860_resume(struct device *dev)
{
	struct max77860_dev *chip = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);

	__lock(chip);

	pr_info("<%s> resuming\n", client->name);

	__unlock(chip);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77860_pm, max77860_suspend, max77860_resume);

static const struct of_device_id max77860_of_match_table[] = {
	{ .compatible = "maxim,max77860", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max77860_of_match_table);

static const struct i2c_device_id max77860_id_table[] = {
	{ "max77860", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, max77860_id_table);

static struct i2c_driver max77860_driver = {
	.driver		= {
		.name	= "max77860",
		.of_match_table = max77860_of_match_table,
	},
	.probe		= max77860_probe,
	.remove		= max77860_remove,
	.id_table       = max77860_id_table,
};

static __init int max77860_init(void)
{
	int rc = -ENODEV;

	pr_info("%s\n", __func__);
	rc = i2c_add_driver(&max77860_driver);
	if (rc != 0)
		pr_err("Failed to register I2C driver: %d\n", rc);
	pr_info("%s: Added I2C Driver\n", __func__);
	return rc;
}

static __exit void max77860_exit(void)
{
	i2c_del_driver(&max77860_driver);
}

module_init(max77860_init);
module_exit(max77860_exit);

MODULE_DESCRIPTION("max77860 MFD PMIC");
MODULE_AUTHOR("ugur.usug@maximintegrated.com ");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
