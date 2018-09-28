/*
 * MAXIM MAX77714 GPIO driver
 *
 * Copyright (c) 2018, MAXIM Integrated.  All rights reserved.
 *
 * based on MAX77620
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77714.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define GPIO_REG_ADDR(offset) (MAX77714_REG_CNFG_GPIO0 + offset)

struct max77714_gpio {
	struct gpio_chip	gpio_chip;
	struct regmap		*rmap;
	struct device		*dev;
	int			gpio_irq;
	int			irq_base;
	int			gpio_base;
};

static const struct regmap_irq max77714_gpio_irqs[] = {
	[0] = {
		.mask = MAX77714_IRQ_LVL2_GPIO_EDGE0,
		.type_rising_mask = MAX77714_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77714_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 0,
	},
	[1] = {
		.mask = MAX77714_IRQ_LVL2_GPIO_EDGE1,
		.type_rising_mask = MAX77714_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77714_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 1,
	},
	[2] = {
		.mask = MAX77714_IRQ_LVL2_GPIO_EDGE2,
		.type_rising_mask = MAX77714_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77714_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 2,
	},
	[3] = {
		.mask = MAX77714_IRQ_LVL2_GPIO_EDGE3,
		.type_rising_mask = MAX77714_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77714_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 3,
	},
	[4] = {
		.mask = MAX77714_IRQ_LVL2_GPIO_EDGE4,
		.type_rising_mask = MAX77714_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77714_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 4,
	},
	[5] = {
		.mask = MAX77714_IRQ_LVL2_GPIO_EDGE5,
		.type_rising_mask = MAX77714_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77714_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 5,
	},
	[6] = {
		.mask = MAX77714_IRQ_LVL2_GPIO_EDGE6,
		.type_rising_mask = MAX77714_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77714_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 6,
	},
	[7] = {
		.mask = MAX77714_IRQ_LVL2_GPIO_EDGE7,
		.type_rising_mask = MAX77714_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77714_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 7,
	},
};

static struct regmap_irq_chip max77714_gpio_irq_chip = {
	.name = "max77714-gpio",
	.irqs = max77714_gpio_irqs,
	.num_irqs = ARRAY_SIZE(max77714_gpio_irqs),
	.num_regs = 1,
	.num_type_reg = 8,
	.irq_reg_stride = 1,
	.type_reg_stride = 1,
	.status_base = MAX77714_REG_INT_LVL2_GPIO,
	.type_base = MAX77714_REG_CNFG_GPIO0,
};

static int max77714_gpio_dir_input(struct gpio_chip *gc, unsigned int offset)
{
	struct max77714_gpio *mgpio = gpiochip_get_data(gc);
	int ret;
	struct max77714_chip *chip = dev_get_drvdata(mgpio->dev->parent);

	ret = regmap_update_bits(mgpio->rmap, GPIO_REG_ADDR(offset),
				 MAX77714_CNFG_GPIO_DIR_MASK,
				 MAX77714_CNFG_GPIO_DIR_INPUT);
	if (ret < 0)
		dev_err(mgpio->dev, "CNFG_GPIOx dir update failed: %d\n", ret);

	return ret;
}

static int max77714_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct max77714_gpio *mgpio = gpiochip_get_data(gc);
	unsigned int val;
	int ret;
	struct max77714_chip *chip = dev_get_drvdata(mgpio->dev->parent);

	ret = regmap_read(mgpio->rmap, GPIO_REG_ADDR(offset), &val);
	if (ret < 0) {
		dev_err(mgpio->dev, "CNFG_GPIOx read failed: %d\n", ret);
		return ret;
	}

	if  (val & MAX77714_CNFG_GPIO_DIR_MASK)
		return !!(val & MAX77714_CNFG_GPIO_INPUT_VAL_MASK);
	else
		return !!(val & MAX77714_CNFG_GPIO_OUTPUT_VAL_MASK);
}

static int max77714_gpio_dir_output(struct gpio_chip *gc, unsigned int offset,
				    int value)
{
	struct max77714_gpio *mgpio = gpiochip_get_data(gc);
	u8 val;
	int ret;
	struct max77714_chip *chip = dev_get_drvdata(mgpio->dev->parent);

	val = (value) ? MAX77714_CNFG_GPIO_OUTPUT_VAL_HIGH :
				MAX77714_CNFG_GPIO_OUTPUT_VAL_LOW;

	ret = regmap_update_bits(mgpio->rmap, GPIO_REG_ADDR(offset),
				 MAX77714_CNFG_GPIO_OUTPUT_VAL_MASK, val);
	if (ret < 0) {
		dev_err(mgpio->dev, "CNFG_GPIOx val update failed: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(mgpio->rmap, GPIO_REG_ADDR(offset),
				 MAX77714_CNFG_GPIO_DIR_MASK,
				 MAX77714_CNFG_GPIO_DIR_OUTPUT);
	if (ret < 0)
		dev_err(mgpio->dev, "CNFG_GPIOx dir update failed: %d\n", ret);

	return ret;
}

static int max77714_gpio_set_debounce(struct gpio_chip *gc,
				      unsigned int offset,
				      unsigned int debounce)
{
	struct max77714_gpio *mgpio = gpiochip_get_data(gc);
	u8 val;
	int ret;
	struct max77714_chip *chip = dev_get_drvdata(mgpio->dev->parent);

	switch (debounce) {
	case 0:
		val = MAX77714_CNFG_GPIO_DBNC_None;
		break;
	case 1 ... 8:
		val = MAX77714_CNFG_GPIO_DBNC_8ms;
		break;
	case 9 ... 16:
		val = MAX77714_CNFG_GPIO_DBNC_16ms;
		break;
	case 17 ... 32:
		val = MAX77714_CNFG_GPIO_DBNC_32ms;
		break;
	default:
		dev_err(mgpio->dev, "Illegal value %u\n", debounce);
		return -EINVAL;
	}

	ret = regmap_update_bits(mgpio->rmap, GPIO_REG_ADDR(offset),
				 MAX77714_CNFG_GPIO_DBNC_MASK, val);//MAX77714_MASK_DBNC0
	if (ret < 0)
		dev_err(mgpio->dev, "CNFG_GPIOx_DBNC update failed: %d\n", ret);

	return ret;
}

static void max77714_gpio_set(struct gpio_chip *gc, unsigned int offset,
			      int value)
{
	struct max77714_gpio *mgpio = gpiochip_get_data(gc);
	struct max77714_chip *chip = dev_get_drvdata(mgpio->dev->parent);
	u8 val;
	int ret;

	val = (value) ? MAX77714_CNFG_GPIO_OUTPUT_VAL_HIGH :
				MAX77714_CNFG_GPIO_OUTPUT_VAL_LOW;

	ret = regmap_update_bits(mgpio->rmap, GPIO_REG_ADDR(offset),
				 MAX77714_CNFG_GPIO_OUTPUT_VAL_MASK, val);
	if (ret < 0)
		dev_err(mgpio->dev, "CNFG_GPIO_OUT update failed: %d\n", ret);
}

static int max77714_gpio_set_single_ended(struct gpio_chip *gc,
					  unsigned int offset,
					  enum single_ended_mode mode)
{
	struct max77714_gpio *mgpio = gpiochip_get_data(gc);
	struct max77714_chip *chip = dev_get_drvdata(mgpio->dev->parent);

	switch (mode) {
	case LINE_MODE_OPEN_DRAIN:
		return regmap_update_bits(mgpio->rmap, GPIO_REG_ADDR(offset),
					  MAX77714_CNFG_GPIO_DRV_MASK,
					  MAX77714_CNFG_GPIO_DRV_OPENDRAIN);
	case LINE_MODE_PUSH_PULL:
		return regmap_update_bits(mgpio->rmap, GPIO_REG_ADDR(offset),
					  MAX77714_CNFG_GPIO_DRV_MASK,
					  MAX77714_CNFG_GPIO_DRV_PUSHPULL);
	default:
		break;
	}
	
	return -ENOTSUPP;
}

static int max77714_gpio_to_irq(struct gpio_chip *gc, unsigned int offset)
{
	struct max77714_gpio *mgpio = gpiochip_get_data(gc);
	struct max77714_chip *chip = dev_get_drvdata(mgpio->dev->parent);

	return regmap_irq_get_virq(chip->gpio_irq_data, offset);
}

static int max77714_gpio_probe(struct platform_device *pdev)
{
	struct max77714_chip *chip =  dev_get_drvdata(pdev->dev.parent);
	struct max77714_gpio *mgpio;
	int gpio_irq;
	int ret;

	dev_err(&pdev->dev, "%s \n", __func__);
	gpio_irq = platform_get_irq(pdev, 0);
	if (gpio_irq <= 0) {
		dev_err(&pdev->dev, "GPIO irq not available %d\n", gpio_irq);
		return -ENODEV;
	}

	mgpio = devm_kzalloc(&pdev->dev, sizeof(*mgpio), GFP_KERNEL);
	if (!mgpio)
		return -ENOMEM;

	mgpio->rmap = chip->rmap;
	mgpio->dev = &pdev->dev;
	mgpio->gpio_irq = gpio_irq;

	mgpio->gpio_chip.label = pdev->name;
	mgpio->gpio_chip.parent = &pdev->dev;
	mgpio->gpio_chip.direction_input = max77714_gpio_dir_input;
	mgpio->gpio_chip.get = max77714_gpio_get;
	mgpio->gpio_chip.direction_output = max77714_gpio_dir_output;
	mgpio->gpio_chip.set_debounce = max77714_gpio_set_debounce;
	mgpio->gpio_chip.set = max77714_gpio_set;
	mgpio->gpio_chip.set_single_ended = max77714_gpio_set_single_ended;
	mgpio->gpio_chip.to_irq = max77714_gpio_to_irq;
	mgpio->gpio_chip.ngpio = MAX77714_GPIO_NR;
	mgpio->gpio_chip.can_sleep = 1;
	mgpio->gpio_chip.base = -1;
	mgpio->irq_base = -1;
#ifdef CONFIG_OF_GPIO
	mgpio->gpio_chip.of_node = pdev->dev.parent->of_node;
#endif

	platform_set_drvdata(pdev, mgpio);

	ret = devm_gpiochip_add_data(&pdev->dev, &mgpio->gpio_chip, mgpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "gpio_init: Failed to add max77714_gpio\n");
		return ret;
	}

	mgpio->gpio_base = mgpio->gpio_chip.base;
	ret = devm_regmap_add_irq_chip(&pdev->dev, chip->rmap, mgpio->gpio_irq,
				       IRQF_ONESHOT, mgpio->irq_base,
				       &max77714_gpio_irq_chip,
				       &chip->gpio_irq_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add gpio irq_chip %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct platform_device_id max77714_gpio_devtype[] = {
	{ .name = "max77714-gpio", },
	{},
};
MODULE_DEVICE_TABLE(platform, max77714_gpio_devtype);

static struct platform_driver max77714_gpio_driver = {
	.driver.name	= "max77714-gpio",
	.probe		= max77714_gpio_probe,
	.id_table	= max77714_gpio_devtype,
};

module_platform_driver(max77714_gpio_driver);

MODULE_DESCRIPTION("GPIO interface for MAX77714");
MODULE_AUTHOR("Daniel.Jeong <daniel.jeong@maximintegrated.com>");
MODULE_AUTHOR("Maxim LDD <opensource@maximintegrated.com>");
MODULE_ALIAS("platform:max77714-gpio");
MODULE_LICENSE("GPL v2");
