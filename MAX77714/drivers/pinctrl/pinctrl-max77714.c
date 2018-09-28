/*
 * MAX77714 pin control driver.
 *
 * Copyright (c) 2018, Maxim Integrated.  All rights reserved.
 *
 * based on MAX77620 Driver *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/mfd/max77714.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "core.h"
#include "pinconf.h"
#include "pinctrl-utils.h"

#define MAX77714_PIN_NUM 8

enum max77714_pin_ppdrv {
	MAX77714_PIN_UNCONFIG_DRV,
	MAX77714_PIN_OD_DRV,
	MAX77714_PIN_PP_DRV,
};

enum max77714_pinconf_param {
	MAX77714_ACTIVE_FPS_SOURCE = PIN_CONFIG_END + 1,
	MAX77714_ACTIVE_FPS_POWER_ON_SLOTS,
	MAX77714_ACTIVE_FPS_POWER_DOWN_SLOTS,
};

struct max77714_pin_function {
	const char *name;
	const char * const *groups;
	unsigned int ngroups;
	int mux_option;
};

static const struct pinconf_generic_params max77714_cfg_params[] = {
	{
		.property = "maxim,active-fps-source",
		.param = MAX77714_ACTIVE_FPS_SOURCE,
	}, {
		.property = "maxim,active-fps-power-up-slot",
		.param = MAX77714_ACTIVE_FPS_POWER_ON_SLOTS,
	}, {
		.property = "maxim,active-fps-power-down-slot",
		.param = MAX77714_ACTIVE_FPS_POWER_DOWN_SLOTS,
	}, 
};

enum max77714_alternate_pinmux_option {
	MAX77714_PINMUX_GPIO				= 0,
	MAX77714_PINMUX_LOW_POWER_MODE_CONTROL_IN	= 1,
	MAX77714_PINMUX_FLEXIBLE_POWER_SEQUENCER_OUT	= 2,
	MAX77714_PINMUX_32K_OUT1			= 3,
	MAX77714_PINMUX_SD0_DYNAMIC_VOLTAGE_SCALING_IN	= 4,
	MAX77714_PINMUX_SD1_DYNAMIC_VOLTAGE_SCALING_IN	= 5,
	MAX77714_PINMUX_REFERENCE_OUT			= 6,
};

struct max77714_pingroup {
	const char *name;
	const unsigned int pins[1];
	unsigned int npins;
	enum max77714_alternate_pinmux_option alt_option;
};

struct max77714_pin_info {
	enum max77714_pin_ppdrv drv_type;
	int pull_config;
};

struct max77714_fps_config {
	int active_fps_src;
	int active_power_up_slots;
	int active_power_down_slots;
};

struct max77714_pctrl_info {
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct regmap *rmap;
	int pins_current_opt[MAX77714_GPIO_NR];
	const struct max77714_pin_function *functions;
	unsigned int num_functions;
	const struct max77714_pingroup *pin_groups;
	int num_pin_groups;
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;
	struct max77714_pin_info pin_info[MAX77714_PIN_NUM];
	struct max77714_fps_config fps_config[MAX77714_PIN_NUM];
};

static const struct pinctrl_pin_desc max77714_pins_desc[] = {
	PINCTRL_PIN(MAX77714_GPIO0, "gpio0"),
	PINCTRL_PIN(MAX77714_GPIO1, "gpio1"),
	PINCTRL_PIN(MAX77714_GPIO2, "gpio2"),
	PINCTRL_PIN(MAX77714_GPIO3, "gpio3"),
	PINCTRL_PIN(MAX77714_GPIO4, "gpio4"),
	PINCTRL_PIN(MAX77714_GPIO5, "gpio5"),
	PINCTRL_PIN(MAX77714_GPIO6, "gpio6"),
	PINCTRL_PIN(MAX77714_GPIO7, "gpio7"),
};

static const char * const gpio_groups[] = {
	"gpio0",
	"gpio1",
	"gpio2",
	"gpio3",
	"gpio4",
	"gpio5",
	"gpio6",
	"gpio7",
};

#define FUNCTION_GROUP(fname, mux)			\
	{						\
		.name = fname,				\
		.groups = gpio_groups,			\
		.ngroups = ARRAY_SIZE(gpio_groups),	\
		.mux_option = MAX77714_PINMUX_##mux,	\
	}

static const struct max77714_pin_function max77714_pin_function[] = {
	FUNCTION_GROUP("gpio", GPIO),
	FUNCTION_GROUP("lpm-control-in", LOW_POWER_MODE_CONTROL_IN),
	FUNCTION_GROUP("fps-out", FLEXIBLE_POWER_SEQUENCER_OUT),
	FUNCTION_GROUP("32k-out1", 32K_OUT1),
	FUNCTION_GROUP("sd0-dvs-in", SD0_DYNAMIC_VOLTAGE_SCALING_IN),
	FUNCTION_GROUP("sd1-dvs-in", SD1_DYNAMIC_VOLTAGE_SCALING_IN),
	FUNCTION_GROUP("reference-out", REFERENCE_OUT),
};

#define MAX77714_PINGROUP(pg_name, pin_id, option) \
	{								\
		.name = #pg_name,					\
		.pins = {MAX77714_##pin_id},				\
		.npins = 1,						\
		.alt_option = MAX77714_PINMUX_##option,			\
	}

static const struct max77714_pingroup max77714_pingroups[] = {
	MAX77714_PINGROUP(gpio0, GPIO0, LOW_POWER_MODE_CONTROL_IN),
	MAX77714_PINGROUP(gpio1, GPIO1, FLEXIBLE_POWER_SEQUENCER_OUT),
	MAX77714_PINGROUP(gpio2, GPIO2, FLEXIBLE_POWER_SEQUENCER_OUT),
	MAX77714_PINGROUP(gpio3, GPIO3, FLEXIBLE_POWER_SEQUENCER_OUT),
	MAX77714_PINGROUP(gpio4, GPIO4, 32K_OUT1),
	MAX77714_PINGROUP(gpio5, GPIO5, SD0_DYNAMIC_VOLTAGE_SCALING_IN),
	MAX77714_PINGROUP(gpio6, GPIO6, SD1_DYNAMIC_VOLTAGE_SCALING_IN),
	MAX77714_PINGROUP(gpio7, GPIO7, REFERENCE_OUT),
};

static int max77714_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct max77714_pctrl_info *mpci = pinctrl_dev_get_drvdata(pctldev);

	return mpci->num_pin_groups;
}

static const char *max77714_pinctrl_get_group_name(
		struct pinctrl_dev *pctldev, unsigned int group)
{
	struct max77714_pctrl_info *mpci = pinctrl_dev_get_drvdata(pctldev);

	return mpci->pin_groups[group].name;
}

static int max77714_pinctrl_get_group_pins(
		struct pinctrl_dev *pctldev, unsigned int group,
		const unsigned int **pins, unsigned int *num_pins)
{
	struct max77714_pctrl_info *mpci = pinctrl_dev_get_drvdata(pctldev);

	*pins = mpci->pin_groups[group].pins;
	*num_pins = mpci->pin_groups[group].npins;

	return 0;
}

static const struct pinctrl_ops max77714_pinctrl_ops = {
	.get_groups_count = max77714_pinctrl_get_groups_count,
	.get_group_name = max77714_pinctrl_get_group_name,
	.get_group_pins = max77714_pinctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static int max77714_pinctrl_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct max77714_pctrl_info *mpci = pinctrl_dev_get_drvdata(pctldev);

	return mpci->num_functions;
}

static const char *max77714_pinctrl_get_func_name(struct pinctrl_dev *pctldev,
						  unsigned int function)
{
	struct max77714_pctrl_info *mpci = pinctrl_dev_get_drvdata(pctldev);

	return mpci->functions[function].name;
}

static int max77714_pinctrl_get_func_groups(struct pinctrl_dev *pctldev,
					    unsigned int function,
					    const char * const **groups,
					    unsigned int * const num_groups)
{
	struct max77714_pctrl_info *mpci = pinctrl_dev_get_drvdata(pctldev);

	*groups = mpci->functions[function].groups;
	*num_groups = mpci->functions[function].ngroups;

	return 0;
}

static int max77714_pinctrl_enable(struct pinctrl_dev *pctldev,
				   unsigned int function, unsigned int group)
{
	struct max77714_pctrl_info *mpci = pinctrl_dev_get_drvdata(pctldev);
	u8 val;
	int ret;

	if (function == MAX77714_PINMUX_GPIO) {
		val = 0;
	} else if (function == mpci->pin_groups[group].alt_option) {
		val = 1 << group;
	} else {
		dev_err(mpci->dev, "GPIO %u doesn't have function %u\n",
			group, function);
		return -EINVAL;
	}
	ret = regmap_update_bits(mpci->rmap, MAX77714_REG_AME_GPIO,
				 BIT(group), val);
	if (ret < 0)
		dev_err(mpci->dev, "REG AME GPIO update failed: %d\n", ret);

	return ret;
}

static const struct pinmux_ops max77714_pinmux_ops = {
	.get_functions_count	= max77714_pinctrl_get_funcs_count,
	.get_function_name	= max77714_pinctrl_get_func_name,
	.get_function_groups	= max77714_pinctrl_get_func_groups,
	.set_mux		= max77714_pinctrl_enable,
};

static int max77714_pinconf_get(struct pinctrl_dev *pctldev,
				unsigned int pin, unsigned long *config)
{
	struct max77714_pctrl_info *mpci = pinctrl_dev_get_drvdata(pctldev);
	struct device *dev = mpci->dev;
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned int val;
	int arg = 0;
	int ret;

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (mpci->pin_info[pin].drv_type == MAX77714_PIN_OD_DRV)
			arg = 1;
		break;

	case PIN_CONFIG_DRIVE_PUSH_PULL:
		if (mpci->pin_info[pin].drv_type == MAX77714_PIN_PP_DRV)
			arg = 1;
		break;

	case PIN_CONFIG_BIAS_PULL_UP:
		ret = regmap_read(mpci->rmap, MAX77714_REG_PUE_GPIO, &val);
		if (ret < 0) {
			dev_err(dev, "Reg PUE_GPIO read failed: %d\n", ret);
			return ret;
		}
		if (val & BIT(pin))
			arg = 1;
		break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		ret = regmap_read(mpci->rmap, MAX77714_REG_PDE_GPIO, &val);
		if (ret < 0) {
			dev_err(dev, "Reg PDE_GPIO read failed: %d\n", ret);
			return ret;
		}
		if (val & BIT(pin))
			arg = 1;
		break;

	default:
		dev_err(dev, "Properties not supported\n");
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, (u16)arg);

	return 0;
}

static int max77714_get_default_fps(struct max77714_pctrl_info *mpci,
				    int addr, int *fps)
{
	unsigned int val;
	int ret;

	ret = regmap_read(mpci->rmap, addr, &val);
	if (ret < 0) {
		dev_err(mpci->dev, "Reg PUE_GPIO read failed: %d\n", ret);
		return ret;
	}
	*fps = (val & MAX77714_MASK_FPSSRC_GPIO0) >> MAX77714_SHIFT_FPSSRC_GPIO0;

	return 0;
}

static int max77714_set_fps_param(struct max77714_pctrl_info *mpci,
				  int pin, int param)
{
	struct max77714_fps_config *fps_config = &mpci->fps_config[pin];
	int addr, ret;
	int param_val;
	int mask, shift;

	if ((pin < MAX77714_GPIO1) || (pin > MAX77714_GPIO3))
		return 0;

	addr = MAX77714_REG_FPS_GPIO1 + pin - 1;
	switch (param) {
	case MAX77714_ACTIVE_FPS_SOURCE:
		mask = MAX77714_MASK_FPSSRC_L0;
		shift = MAX77714_SHIFT_FPSSRC_L0;
		param_val = fps_config->active_fps_src;
		break;

	case MAX77714_ACTIVE_FPS_POWER_ON_SLOTS:
		mask = MAX77714_MASK_LDO0UPSLT;
		shift = MAX77714_SHIFT_LDO0UPSLT;
		param_val = fps_config->active_power_up_slots;
		break;

	case MAX77714_ACTIVE_FPS_POWER_DOWN_SLOTS:
		mask = MAX77714_MASK_LDO0DNSLT;
		shift = MAX77714_SHIFT_LDO0DNSLT;
		param_val = fps_config->active_power_down_slots;
		break;

	default:
		dev_err(mpci->dev, "Invalid parameter %d for pin %d\n",
			param, pin);
		return -EINVAL;
	}

	if (param_val < 0)
		return 0;

	ret = regmap_update_bits(mpci->rmap, addr, mask, param_val << shift);
	if (ret < 0)
		dev_err(mpci->dev, "Reg 0x%02x update failed %d\n", addr, ret);

	return ret;
}

static int max77714_pinconf_set(struct pinctrl_dev *pctldev,
				unsigned int pin, unsigned long *configs,
				unsigned int num_configs)
{
	struct max77714_pctrl_info *mpci = pinctrl_dev_get_drvdata(pctldev);
	struct device *dev = mpci->dev;
	struct max77714_fps_config *fps_config;
	int param;
	u16 param_val;
	unsigned int val;
	unsigned int pu_val;
	unsigned int pd_val;
	int addr, ret;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		param_val = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			val = param_val ? 0 : 1;
			ret = regmap_update_bits(mpci->rmap,
						 MAX77714_REG_CNFG_GPIO0 + pin,
						 MAX77714_CNFG_GPIO_DRV_MASK,
						 val);
			if (ret < 0) {
				dev_err(dev, "Reg 0x%02x update failed %d\n",
					MAX77714_REG_CNFG_GPIO0 + pin, ret);
				return ret;
			}
			mpci->pin_info[pin].drv_type = val ?
				MAX77714_PIN_PP_DRV : MAX77714_PIN_OD_DRV;
			break;

		case PIN_CONFIG_DRIVE_PUSH_PULL:
			val = param_val ? 1 : 0;
			ret = regmap_update_bits(mpci->rmap,
						 MAX77714_REG_CNFG_GPIO0 + pin,
						 MAX77714_CNFG_GPIO_DRV_MASK,
						 val);
			if (ret < 0) {
				dev_err(dev, "Reg 0x%02x update failed %d\n",
					MAX77714_REG_CNFG_GPIO0 + pin, ret);
				return ret;
			}
			mpci->pin_info[pin].drv_type = val ?
				MAX77714_PIN_PP_DRV : MAX77714_PIN_OD_DRV;
			break;

		case MAX77714_ACTIVE_FPS_SOURCE:
		case MAX77714_ACTIVE_FPS_POWER_ON_SLOTS:
		case MAX77714_ACTIVE_FPS_POWER_DOWN_SLOTS:
			if ((pin < MAX77714_GPIO1) || (pin > MAX77714_GPIO3))
				return -EINVAL;

			fps_config = &mpci->fps_config[pin];

			if ((param == MAX77714_ACTIVE_FPS_SOURCE) &&
			    (param_val == MAX77714_FPS_SRC_DEF)) {
				addr = MAX77714_REG_FPS_GPIO1 + pin - 1;
				ret = max77714_get_default_fps(
						mpci, addr,
						&fps_config->active_fps_src);
				if (ret < 0)
					return ret;
				break;
			}

			if (param == MAX77714_ACTIVE_FPS_SOURCE)
				fps_config->active_fps_src = param_val;
			else if (param == MAX77714_ACTIVE_FPS_POWER_ON_SLOTS)
				fps_config->active_power_up_slots = param_val;
			else
				fps_config->active_power_down_slots = param_val;

			ret = max77714_set_fps_param(mpci, pin, param);
			if (ret < 0)
				return ret;
			break;

		case PIN_CONFIG_BIAS_PULL_UP:
		case PIN_CONFIG_BIAS_PULL_DOWN:
			pu_val = (param == PIN_CONFIG_BIAS_PULL_UP) ?
							BIT(pin) : 0;
			pd_val = (param == PIN_CONFIG_BIAS_PULL_DOWN) ?
							BIT(pin) : 0;

			ret = regmap_update_bits(mpci->rmap,
						 MAX77714_REG_PUE_GPIO,
						 BIT(pin), pu_val);
			if (ret < 0) {
				dev_err(dev, "PUE_GPIO update failed: %d\n",
					ret);
				return ret;
			}

			ret = regmap_update_bits(mpci->rmap,
						 MAX77714_REG_PDE_GPIO,
						 BIT(pin), pd_val);
			if (ret < 0) {
				dev_err(dev, "PDE_GPIO update failed: %d\n",
					ret);
				return ret;
			}
			break;

		default:
			dev_err(dev, "Properties not supported\n");
			return -ENOTSUPP;
		}
	}

	return 0;
}

static const struct pinconf_ops max77714_pinconf_ops = {
	.pin_config_get = max77714_pinconf_get,
	.pin_config_set = max77714_pinconf_set,
};

static struct pinctrl_desc max77714_pinctrl_desc = {
	.pctlops = &max77714_pinctrl_ops,
	.pmxops = &max77714_pinmux_ops,
	.confops = &max77714_pinconf_ops,
};

static int max77714_pinctrl_probe(struct platform_device *pdev)
{
	struct max77714_chip *max77714 = dev_get_drvdata(pdev->dev.parent);
	struct max77714_pctrl_info *mpci;
	int i;

	mpci = devm_kzalloc(&pdev->dev, sizeof(*mpci), GFP_KERNEL);
	if (!mpci)
		return -ENOMEM;

	mpci->dev = &pdev->dev;
	mpci->dev->of_node = pdev->dev.parent->of_node;
	mpci->rmap = max77714->rmap;

	mpci->pins = max77714_pins_desc;
	mpci->num_pins = ARRAY_SIZE(max77714_pins_desc);
	mpci->functions = max77714_pin_function;
	mpci->num_functions = ARRAY_SIZE(max77714_pin_function);
	mpci->pin_groups = max77714_pingroups;
	mpci->num_pin_groups = ARRAY_SIZE(max77714_pingroups);
	platform_set_drvdata(pdev, mpci);

	max77714_pinctrl_desc.name = dev_name(&pdev->dev);
	max77714_pinctrl_desc.pins = max77714_pins_desc;
	max77714_pinctrl_desc.npins = ARRAY_SIZE(max77714_pins_desc);
	max77714_pinctrl_desc.num_custom_params =
				ARRAY_SIZE(max77714_cfg_params);
	max77714_pinctrl_desc.custom_params = max77714_cfg_params;

	for (i = 0; i < MAX77714_PIN_NUM; ++i) {
		mpci->fps_config[i].active_fps_src = -1;
		mpci->fps_config[i].active_power_up_slots = -1;
		mpci->fps_config[i].active_power_down_slots = -1;
	}

	mpci->pctl = devm_pinctrl_register(&pdev->dev, &max77714_pinctrl_desc,
					   mpci);
	if (IS_ERR(mpci->pctl)) {
		dev_err(&pdev->dev, "Couldn't register pinctrl driver\n");
		return PTR_ERR(mpci->pctl);
	}

	return 0;
}

static const struct platform_device_id max77714_pinctrl_devtype[] = {
	{ .name = "max77714-pinctrl", },
	{},
};
MODULE_DEVICE_TABLE(platform, max77714_pinctrl_devtype);

static struct platform_driver max77714_pinctrl_driver = {
	.driver = {
		.name = "max77714-pinctrl",
	},
	.probe = max77714_pinctrl_probe,
	.id_table = max77714_pinctrl_devtype,
};

module_platform_driver(max77714_pinctrl_driver);

MODULE_DESCRIPTION("MAX77714 pin control driver");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@maximintegrated.com>");
MODULE_AUTHOR("Maxim LDD <opensource@maximintegrated.com>");
MODULE_ALIAS("platform:max77714-pinctrl");
MODULE_LICENSE("GPL v2");
