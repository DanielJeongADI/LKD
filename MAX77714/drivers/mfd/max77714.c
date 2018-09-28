/*
 * Maxim MAX77714 MFD Driver
 *
 * Copyright (C) 2018 Maxim Integrated. All rights reserved.
 *
 * Author:
 *	Daniel Jeong <daniel.jeong@maximintegrated.com>
 *	Maxim LDD <opensource@maximintegrated.com>
 *
 * based on MAX77620 Driver
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/****************** Teminology used in driver ********************
 * Here are some terminology used from datasheet for quick reference:
 * Flexible Power Sequence (FPS):
 * The Flexible Power Sequencer (FPS) allows each regulator to power up under
 * hardware or software control. Additionally, each regulator can power on
 * independently or among a group of other regulators with an adjustable
 * power-up and power-down delays (sequencing). GPIO1, GPIO2, and GPIO3 can
 * be programmed to be part of a sequence allowing external regulators to be
 * sequenced along with internal regulators. 32KHz clock can be programmed to
 * be part of a sequence.
 * There is 3 FPS confguration registers and all resources are configured to
 * any of these FPS or no FPS.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77714.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>

static const struct resource gpio_resources[] = {
	DEFINE_RES_IRQ(MAX77714_IRQ_TOP_GPIO),
};

static const struct resource power_resources[] = {
	DEFINE_RES_IRQ(MAX77714_IRQ_LBT_MBATRESET),
};

static const struct resource rtc_resources[] = {
	DEFINE_RES_IRQ(MAX77714_IRQ_TOP_RTC),
};

static const struct resource thermal_resources[] = {
	DEFINE_RES_IRQ(MAX77714_IRQ_LBT_TJALRM1),
	DEFINE_RES_IRQ(MAX77714_IRQ_LBT_TJALRM2),
};

static const struct regmap_irq max77714_top_irqs[] = {
	REGMAP_IRQ_REG(MAX77714_IRQ_TOP_GLBL, 0, MAX77714_MASK_IRQ_GLBL),
	REGMAP_IRQ_REG(MAX77714_IRQ_TOP_SD, 0, MAX77714_MASK_IRQ_SD),
	REGMAP_IRQ_REG(MAX77714_IRQ_TOP_LDO, 0, MAX77714_MASK_IRQ_LDO),
	REGMAP_IRQ_REG(MAX77714_IRQ_TOP_GPIO, 0, MAX77714_MASK_IRQ_GPIO),
	REGMAP_IRQ_REG(MAX77714_IRQ_TOP_RTC, 0, MAX77714_MASK_IRQ_RTC),
	REGMAP_IRQ_REG(MAX77714_IRQ_TOP_ONOFF, 0, MAX77714_MASK_IRQ_ONOFF),
	REGMAP_IRQ_REG(MAX77714_IRQ_TOP_IRQ, 0, MAX77714_MASK_IRQ),
	REGMAP_IRQ_REG(MAX77714_IRQ_LBT_MBATRESET, 1, MAX77714_MASK_MBATTRESET_R),
	REGMAP_IRQ_REG(MAX77714_IRQ_LBT_TJALRM1, 1, MAX77714_MASK_TJALRM1_R),
	REGMAP_IRQ_REG(MAX77714_IRQ_LBT_TJALRM2, 1, MAX77714_MASK_TJALRM2_R),
};

static const struct mfd_cell max77714_children[] = {
	{ .name = "max77714-pinctrl", },
	{ .name = "max77714-clock", },
	{ .name = "max77714-pmic", },
	{ .name = "max77714-watchdog", },
	{
		.name = "max77714-gpio",
		.resources = gpio_resources,
		.num_resources = ARRAY_SIZE(gpio_resources),
	}, {
		.name = "max77714-rtc",
		.resources = rtc_resources,
		.num_resources = ARRAY_SIZE(rtc_resources),
	}, {
		.name = "max77714-power",
		.resources = power_resources,
		.num_resources = ARRAY_SIZE(power_resources),
	}, {
		.name = "max77714-thermal",
		.resources = thermal_resources,
		.num_resources = ARRAY_SIZE(thermal_resources),
	},
};

static const struct regmap_range max77714_readable_ranges[] = {
	/* PMIC-GPIO */
	regmap_reg_range(MAX77714_REG_INT_TOP, MAX77714_REG_32K_CONFIG),	
	regmap_reg_range(MAX77714_REG_CNFG_GLBL1, MAX77714_REG_FPS_RSTIO),
	/* BUCK */
	regmap_reg_range(MAX77714_REG_CNFG1_SD0, MAX77714_REG_CNFG3_SD3),
	/* LDO */
	regmap_reg_range(MAX77714_REG_LDO_CNFG1_L0, MAX77714_REG_LDO_CNFG3),
	/* GPIO */
	regmap_reg_range(MAX77714_REG_CNFG_GPIO0, MAX77714_REG_AME_GPIO),
	/* SBIAS */
	regmap_reg_range(MAX77714_REG_CID0, MAX77714_REG_CID4),
	/* BBC */
	regmap_reg_range(MAX77714_REG_CNFG_BBC, MAX77714_REG_CNFG_BBC),
	/* I2C */
	regmap_reg_range(MAX77714_REG_I2C_CTRL1, MAX77714_REG_I2C_CTRL2),	
};

static const struct regmap_access_table max77714_readable_table = {
	.yes_ranges = max77714_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77714_readable_ranges),
};

static const struct regmap_range max77714_writable_ranges[] = {	
	/* PMIC-GPIO */
	regmap_reg_range(MAX77714_REG_INT_TOP, MAX77714_REG_32K_CONFIG),	
	regmap_reg_range(MAX77714_REG_CNFG_GLBL1, MAX77714_REG_FPS_RSTIO),
	/* BUCK */
	regmap_reg_range(MAX77714_REG_CNFG1_SD0, MAX77714_REG_CNFG3_SD3),
	/* LDO */
	regmap_reg_range(MAX77714_REG_LDO_CNFG1_L0, MAX77714_REG_LDO_CNFG3),
	/* GPIO */
	regmap_reg_range(MAX77714_REG_CNFG_GPIO0, MAX77714_REG_AME_GPIO),
	/* BBC */
	regmap_reg_range(MAX77714_REG_CNFG_BBC, MAX77714_REG_CNFG_BBC),
	/* I2C */
	regmap_reg_range(MAX77714_REG_I2C_CTRL1, MAX77714_REG_I2C_CTRL2),
};

static const struct regmap_access_table max77714_writable_table = {
	.yes_ranges = max77714_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77714_writable_ranges),
};

static const struct regmap_range max77714_cacheable_ranges[] = {	
	regmap_reg_range(MAX77714_REG_CNFG_GLBL1, MAX77714_REG_FPS_RSTIO),
	regmap_reg_range(MAX77714_REG_CNFG1_SD0, MAX77714_REG_CNFG3_SD3),
	regmap_reg_range(MAX77714_REG_LDO_CNFG1_L0, MAX77714_REG_LDO_CNFG3),
};

static const struct regmap_access_table max77714_volatile_table = {
	.no_ranges = max77714_cacheable_ranges,
	.n_no_ranges = ARRAY_SIZE(max77714_cacheable_ranges),
};

static const struct regmap_config max77714_regmap_config = {
	.name = "power-slave",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77714_REG_MAX,
	.cache_type = REGCACHE_RBTREE,
	.rd_table = &max77714_readable_table,
	.wr_table = &max77714_writable_table,
	.volatile_table = &max77714_volatile_table,
};

/*
 * MAX77714 has the following steps of the interrupt handling
 * for TOP interrupts:
 * 1. When interrupt occurs from PMIC, mask the PMIC interrupt by setting GLBLM.
 * 2. Read IRQTOP and service the interrupt.
 * 3. Once all interrupts has been checked and serviced, the interrupt service
 *    routine un-masks the hardware interrupt line by clearing GLBLM.
 */
static int max77714_irq_global_mask(void *irq_drv_data)
{
	struct max77714_chip *chip = irq_drv_data;
	int ret;

	ret = regmap_update_bits(chip->rmap, MAX77714_REG_INT_TOPM,
				 MAX77714_MASK_GLBLM, MAX77714_MASK_GLBLM);
	if (ret < 0)
		dev_err(chip->dev, "Failed to set GLBLM: %d\n", ret);

	return ret;
}

static int max77714_irq_global_unmask(void *irq_drv_data)
{
	struct max77714_chip *chip = irq_drv_data;
	int ret;

	ret = regmap_update_bits(chip->rmap, MAX77714_REG_INT_TOPM,
				 MAX77714_MASK_GLBLM, 0);
	if (ret < 0)
		dev_err(chip->dev, "Failed to reset GLBLM: %d\n", ret);

	return ret;
}

static struct regmap_irq_chip max77714_top_irq_chip = {
	.name = "max77714-top",
	.irqs = max77714_top_irqs,
	.num_irqs = ARRAY_SIZE(max77714_top_irqs),
	.num_regs = 2,
	.status_base = MAX77714_REG_INT_TOP,
	.mask_base = MAX77714_REG_INT_TOPM,
	.handle_pre_irq = max77714_irq_global_mask,
	.handle_post_irq = max77714_irq_global_unmask,
};


static int max77714_get_fps_period_reg_value(struct max77714_chip *chip,
					     int tperiod)
{	
	switch(tperiod)	{
	case 0 ... 30:
		dev_warn(chip->dev, "Input is smaller than Minimum.\n");
		return MAX77714_FPS_TFST_31_US;
	case 31 ... 62:
		return MAX77714_FPS_TFST_31_US;
	case 63 ... 126:
		return MAX77714_FPS_TFST_63_US;
	case 127 ... 255:
		return MAX77714_FPS_TFST_127_US;
	case 256 ... 507:
		return MAX77714_FPS_TFST_256_US;
	case 508 ... 983:
		return MAX77714_FPS_TFST_508_US;
	case 985 ... 1935:
		return MAX77714_FPS_TFST_984_US;
	case 1936 ... 3903:
		return MAX77714_FPS_TFST_1936_US;
	case 3904:
		return MAX77714_FPS_TFST_3904_US;
	default:
		dev_warn(chip->dev, "Input is greater than Maximum.\n");
		return MAX77714_FPS_TFST_3904_US;
	}
}

/* max77714_config_fps: Configure FPS configuration registers
 *			based on platform specific information.
 */
static int max77714_config_fps(struct max77714_chip *chip,
			       struct device_node *fps_np)
{
	struct device *dev = chip->dev;
	unsigned int mask = 0, config = 0;
	u32 fps_max_period;
	u32 param_val;
	int tperiod, fps_id;
	int ret;
	char fps_name[10];

	for (fps_id = 0; fps_id < MAX77714_FPS_COUNT; fps_id++) {
		sprintf(fps_name, "fps%d", fps_id);
		dev_err(chip->dev, "%s --- %s \n", __func__,fps_name);
		if (!strcmp(fps_np->name, fps_name))
			break;
	}

	ret = of_property_read_u32(fps_np, "maxim,fps-source", &param_val);
	if (!ret) {
		switch(fps_id == 0){
			case 0:
				mask = MAX77714_MASK_SRCFPS0;
				config = param_val & MAX77714_MASK_SRCFPS0;
				break;

			case 1:
				mask = MAX77714_MASK_SRCFPS1;
				config = param_val & MAX77714_MASK_SRCFPS1;
				break;
		}
	}

	ret = of_property_read_u32(fps_np, "maxim,fps-enable", &param_val);
	if (!ret) {
		switch(fps_id == 0){
			case 0:
				mask = MAX77714_MASK_ENFPS0;
				config = param_val & MAX77714_MASK_ENFPS0;
				break;

			case 1:
				mask = MAX77714_MASK_ENFPS1;
				config = param_val & MAX77714_MASK_ENFPS1;
				break;
		}
	}

	ret = regmap_update_bits(chip->rmap, MAX77714_REG_CNFG_GLBL3,
				 mask, config);
	if (ret < 0) {
		dev_err(dev, "Failed to update MAX77714_REG_CNFG_GLBL3: %d\n", ret);
		return ret;
	}

	return 0;
}

static int max77714_initialise_fps(struct max77714_chip *chip)
{
	struct device *dev = chip->dev;
	struct device_node *fps_np, *fps_child;
	int rval = 0, mask = 0;
	int fps_id;
	int ret;
	int param_val;

	fps_np = of_get_child_by_name(dev->of_node, "fps");
	if (!fps_np)
		goto skip_fps;

	ret = of_property_read_u32(fps_np, "maxim,power-up-period",
				   &param_val);
	if (!ret) {
		dev_err(dev, "%s type %d\n", __func__, param_val);
		mask = MAX77714_MASK_MSTR_PU;
		rval = param_val & MAX77714_MASK_MSTR_PU;
	}

	ret = of_property_read_u32(fps_np, "maxim,power-down-period",
				   &param_val);
	if (!ret) {
		dev_err(dev, "%s type %d\n", __func__, param_val);
		mask |= MAX77714_MASK_MSTR_PD;
		rval |= (param_val & MAX77714_MASK_MSTR_PD);
	}

	ret = regmap_update_bits(chip->rmap, MAX77714_REG_MSTR_PU_PD,
							mask, rval);
	if (ret < 0){
		dev_err(dev, "Failed to set MAX77714_REG_MSTR_PU_PD: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(fps_np, "maxim,sleep-entry-period",
				   &param_val);
	if (!ret) {		
		dev_err(dev, "%s type %d\n", __func__, param_val);
		mask = MAX77714_MASK_MSTR_SLPENTY;
		rval = param_val & MAX77714_MASK_MSTR_SLPENTY;
	}

	ret = of_property_read_u32(fps_np, "maxim,sleep-exit-period",
				   &param_val);
	if (!ret) {		
		dev_err(dev, "%s type %d\n", __func__, param_val);
		mask |= MAX77714_MASK_MSTR_SLPEXT;
		rval |= (param_val & MAX77714_MASK_MSTR_SLPEXT);
	}

	ret = regmap_update_bits(chip->rmap, MAX77714_REG_MSTR_SLPENTRY_EXIT, 
							mask, rval);
	if (ret < 0){
		dev_err(dev, "Failed to set MAX77714_REG_MSTR_PU_PD: %d\n", ret);
		return ret;
	}

	for_each_child_of_node(fps_np, fps_child) {
		ret = max77714_config_fps(chip, fps_child);
		if (ret < 0)
			return ret;
	}

skip_fps:
	/* Enable wake on EN0 pin */
	ret = regmap_update_bits(chip->rmap, 
				MAX77714_REG_CNFG2_ONOFF,
				 MAX77714_MASK_WK_EN0,
				 MAX77714_MASK_WK_EN0);
	if (ret < 0) {
		dev_err(dev, "Failed to update WK_EN0: %d\n", ret);
		return ret;
	}

	return 0;
}



static int max77714_initialise_onoff(struct max77714_chip *chip, 
									struct device_node *node)
{
	int param_val;
	int ret = of_property_read_u32(node, 
					"maxim,sleep-enable", &param_val);

	chip->sleep_enable = 0;
	if(!ret){
		chip->sleep_enable = param_val;
	}

	chip->enable_global_lpm = 0;
	ret = of_property_read_u32(node,
					"maxim,low-power-mode-enable", &param_val);

	if(!ret){
		chip->enable_global_lpm = param_val;
		ret = regmap_update_bits(chip->rmap, MAX77714_REG_CNFG_GLBL2,
				 MAX77714_MASK_GLBL_LPM, param_val); 
		if (ret < 0) {
			dev_err(chip->dev, "Failed to update CNFG_GLBL2: %d\n", ret);
			return ret;
		}
	}
	return ret;
}

static int max77714_read_cid(struct max77714_chip *chip)
{
	unsigned int val;
	int i;
	int ret;

	for (i = MAX77714_REG_CID0; i <= MAX77714_REG_CID4; i++) {
		ret = regmap_read(chip->rmap, i, &val);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to read CID: %d\n", ret);
			return ret;
		}
		switch(i)
		{
			case MAX77714_REG_CID0:
			case MAX77714_REG_CID1:
			case MAX77714_REG_CID2:
				dev_info(chip->dev, "CID%d: 0x%02x\n",
							i - MAX77714_REG_CID0, val);
			break;
			case MAX77714_REG_CID3:
				dev_info(chip->dev, "DDIM 0x%02x, DDIO 0x%02x\n",
					MAX77714_CID3_DIDM(val), MAX77714_CID3_DIDO(val));
			break;
			case MAX77714_REG_CID4:
				dev_info(chip->dev, "OTP:0x%02X\n",val);
			break;
		}			
	}
	return ret;
}

static int max77714_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device_node *node = client->dev.of_node;
	const struct regmap_config *rmap_config;
	struct max77714_chip *chip;
	const struct mfd_cell *mfd_cells;
	int n_mfd_cells;
	int ret, irq;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;

	mfd_cells = max77714_children;
	n_mfd_cells = ARRAY_SIZE(max77714_children);
	rmap_config = &max77714_regmap_config;

	chip->rmap = devm_regmap_init_i2c(client, rmap_config);
	if (IS_ERR(chip->rmap)) {
		ret = PTR_ERR(chip->rmap);
		dev_err(chip->dev, "Failed to intialise regmap: %d\n", ret);
		return ret;
	}

	ret = max77714_read_cid(chip);
	if (ret < 0)
		return ret;

	irq = of_get_named_gpio(node,"irq-gpio", 0);
	gpio_request_one(irq, GPIOF_DIR_IN,"max77714-top");
	chip->irq = gpio_to_irq(irq);
	if(chip->irq < 0){
		dev_warn(chip->dev, "irq can't be configurated\n");
		return -EINVAL;
	}

	max77714_top_irq_chip.irq_drv_data = chip;
	ret = devm_regmap_add_irq_chip(chip->dev, chip->rmap, 
					   chip->irq,
				       IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED,
				       0, &max77714_top_irq_chip,
				       &chip->top_irq_data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add regmap irq: %d\n", ret);
		return ret;
	}

	ret = max77714_initialise_onoff(chip, node);
	if (ret < 0)
		return ret;

	ret = max77714_initialise_fps(chip);
	if (ret < 0)
		return ret;

	ret =  devm_mfd_add_devices(chip->dev, PLATFORM_DEVID_NONE,
				    mfd_cells, n_mfd_cells, NULL, 0,
				    regmap_irq_get_domain(chip->top_irq_data));
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add MFD children: %d\n", ret);
		return ret;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int max77714_i2c_suspend(struct device *dev)
{
	struct max77714_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int config;
	int fps;
	int ret;

	config = (chip->sleep_enable) ? MAX77714_MASK_SLPEN : 0;
	ret = regmap_update_bits(chip->rmap, MAX77714_REG_CNFG1_ONOFF,
				 MAX77714_MASK_SLPEN, config);
	if (ret < 0) {
		dev_err(dev, "Failed to configure sleep in suspend: %d\n", ret);
		return ret;
	}

	/* Disable WK_EN0 */
	ret = regmap_update_bits(chip->rmap, MAX77714_REG_CNFG2_ONOFF,
				 MAX77714_MASK_WK_EN0, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to configure WK_EN in suspend: %d\n", ret);
		return ret;
	}

	disable_irq(client->irq);

	return 0;
}

static int max77714_i2c_resume(struct device *dev)
{
	struct max77714_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	int fps;

	/* Enable WK_EN0 */
	ret = regmap_update_bits(chip->rmap, MAX77714_REG_CNFG2_ONOFF,
				 MAX77714_MASK_WK_EN0,
				 MAX77714_MASK_WK_EN0);
	if (ret < 0) {
		dev_err(dev, "Failed to configure WK_EN0 n resume: %d\n", ret);
		return ret;
	}

	enable_irq(client->irq);

	return 0;
}
#endif

static const struct i2c_device_id max77714_id[] = {
	{"max77714", 0},
	{},
};

static const struct dev_pm_ops max77714_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max77714_i2c_suspend, max77714_i2c_resume)
};

static struct i2c_driver max77714_driver = {
	.driver = {
		.name = "max77714",
		.pm = &max77714_pm_ops,
	},
	.probe = max77714_probe,
	.id_table = max77714_id,
};
builtin_i2c_driver(max77714_driver);