/*
 * Maxim MAX77714 Regulator Driver
 *
 * Copyright (C) 2018 Maxim Integrated. All rights reserved.
 *
 * based on MAX77620
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/mfd/max77714.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define max77714_rails(_name)	"max77714-"#_name

/* Power Mode */
enum max77714_power_mode {
	MAX77714_POWER_MODE_DISABLE,
	MAX77714_POWER_MODE_GLPM,
	MAX77714_POWER_MODE_LPM,
	MAX77714_POWER_MODE_NORMAL
};

enum max77714_regulators {
	MAX77714_REGULATOR_ID_SD0,
	MAX77714_REGULATOR_ID_SD1,
	MAX77714_REGULATOR_ID_SD2,
	MAX77714_REGULATOR_ID_SD3,
	MAX77714_REGULATOR_ID_L0,
	MAX77714_REGULATOR_ID_L1,
	MAX77714_REGULATOR_ID_L2,
	MAX77714_REGULATOR_ID_L3,
	MAX77714_REGULATOR_ID_L4,
	MAX77714_REGULATOR_ID_L5,
	MAX77714_REGULATOR_ID_L6,
	MAX77714_REGULATOR_ID_L7,
	MAX77714_REGULATOR_ID_L8,
	MAX77714_NUM_REGS,
};

/* Regulator types */
enum max77714_regulator_type {
	MAX77714_REGULATOR_TYPE_SD,
	MAX77714_REGULATOR_TYPE_LDO,
};

struct max77714_regulator_info {
	u8 type;
	u8 fps_addr;
	u8 volt_addr;
	u8 cfg_addr;
	u8 power_mode_addr;
	u8 power_mode_mask;
	u8 power_mode_shift;
	struct regulator_desc desc;
};

struct max77714_regulator_pdata {
	struct regulator_init_data *reg_idata;
	int active_fps_src;
	int active_fps_pd_slot;
	int active_fps_pu_slot;
	int current_mode;
	int ramp_rate_setting;
};

struct max77714_regulator {
	struct device *dev;
	struct regmap *rmap;
	struct max77714_regulator_info *rinfo[MAX77714_NUM_REGS];
	struct max77714_regulator_pdata reg_pdata[MAX77714_NUM_REGS];
	int enable_power_mode[MAX77714_NUM_REGS];
	int current_power_mode[MAX77714_NUM_REGS];
	int active_fps_src[MAX77714_NUM_REGS];
};

static int max77714_regulator_get_fps_src(struct max77714_regulator *pmic,
					  int id)
{
	struct max77714_regulator_info *rinfo = pmic->rinfo[id];
	unsigned int val;
	int ret;

	ret = regmap_read(pmic->rmap, rinfo->fps_addr, &val);
	if (ret < 0) {
		dev_err(pmic->dev, "Reg 0x%02x read failed %d\n",
			rinfo->fps_addr, ret);
		return ret;
	}

	return (val & MAX77714_MASK_FPSSRC_L0) >> MAX77714_SHIFT_FPSSRC_L0;
}

static int max77714_regulator_set_fps_src(struct max77714_regulator *pmic,
					  int fps_src, int id)
{
	struct max77714_regulator_info *rinfo = pmic->rinfo[id];
	unsigned int val;
	int ret;

	if (!rinfo)
		return 0;

	switch (fps_src) {
	case MAX77714_FPS_SRC_0:
	case MAX77714_FPS_SRC_1:
	case MAX77714_FPS_SRC_NONE:
		break;

	case MAX77714_FPS_SRC_DEF:
		ret = regmap_read(pmic->rmap, rinfo->fps_addr, &val);
		if (ret < 0) {
			dev_err(pmic->dev, "Reg 0x%02x read failed %d\n",
				rinfo->fps_addr, ret);
			return ret;
		}
		ret = (val & MAX77714_MASK_FPSSRC_L0) >> MAX77714_SHIFT_FPSSRC_L0;
		pmic->active_fps_src[id] = ret;
		return 0;

	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(pmic->rmap, rinfo->fps_addr,
				 MAX77714_MASK_FPSSRC_L0,
				 fps_src << MAX77714_SHIFT_FPSSRC_L0);
	if (ret < 0) {
		dev_err(pmic->dev, "Reg 0x%02x update failed %d\n",
			rinfo->fps_addr, ret);
		return ret;
	}
	pmic->active_fps_src[id] = fps_src;

	return 0;
}

static int max77714_regulator_set_fps_slots(struct max77714_regulator *pmic,
					    int id, bool is_suspend)
{
	struct max77714_regulator_pdata *rpdata = &pmic->reg_pdata[id];
	struct max77714_regulator_info *rinfo = pmic->rinfo[id];
	unsigned int val = 0;
	unsigned int mask = 0;
	int pu = rpdata->active_fps_pu_slot;
	int pd = rpdata->active_fps_pd_slot;
	int ret = 0;

	if (!rinfo)
		return 0;

	/* FPS power up period setting */
	if (pu >= 0) {
		val |= (pu << MAX77714_SHIFT_LDO0UPSLT);
		mask |= MAX77714_MASK_LDO0UPSLT;
	}

	/* FPS power down period setting */
	if (pd >= 0) {
		val |= (pd << MAX77714_SHIFT_LDO0DNSLT);
		mask |= MAX77714_MASK_LDO0DNSLT;
	}

	if (mask) {
		ret = regmap_update_bits(pmic->rmap, rinfo->fps_addr,
					 mask, val);
		if (ret < 0) {
			dev_err(pmic->dev, "Reg 0x%02x update failed: %d\n",
				rinfo->fps_addr, ret);
			return ret;
		}
	}

	return ret;
}

static int max77714_regulator_set_power_mode(struct max77714_regulator *pmic,
					     int power_mode, int id)
{
	struct max77714_regulator_info *rinfo = pmic->rinfo[id];
	u8 mask = rinfo->power_mode_mask;
	u8 shift = rinfo->power_mode_shift;
	u8 addr;
	int ret;

	addr = rinfo->power_mode_addr;
	ret = regmap_update_bits(pmic->rmap, addr, mask, power_mode << shift);
	if (ret < 0) {
		dev_err(pmic->dev, "Regulator %d mode set failed: %d\n",
			id, ret);
		return ret;
	}
	pmic->current_power_mode[id] = power_mode;

	return ret;
}

static int max77714_regulator_get_power_mode(struct max77714_regulator *pmic,
					     int id)
{
	struct max77714_regulator_info *rinfo = pmic->rinfo[id];
	unsigned int val, addr;
	u8 mask = rinfo->power_mode_mask;
	u8 shift = rinfo->power_mode_shift;
	int ret;

	addr = rinfo->power_mode_addr;
	ret = regmap_read(pmic->rmap, addr, &val);
	if (ret < 0) {
		dev_err(pmic->dev, "Regulator %d: Reg 0x%02x read failed: %d\n",
			id, addr, ret);
		return ret;
	}

	return (val & mask) >> shift;
}

static int max77714_read_ramp_delay(struct max77714_regulator *pmic, int id)
{
	struct max77714_regulator_info *rinfo = pmic->rinfo[id];
	unsigned int rval;
	int ramp_delay;
	int ret;

	ret = regmap_read(pmic->rmap, rinfo->cfg_addr, &rval);
	if (ret < 0) {
		dev_err(pmic->dev, "Register 0x%02x read failed: %d\n",
			rinfo->cfg_addr, ret);
		return ret;
	}

	switch (rinfo->type) {
	case MAX77714_REGULATOR_TYPE_SD:

		if(id == MAX77714_REGULATOR_ID_SD2 || MAX77714_REGULATOR_ID_SD3)
			return 0;

		ramp_delay = (rval & MAX77714_MASK_SD0_SSRAMP) 
						>> MAX77714_SHIFT_SD0_SSRAMP;
		if(!ramp_delay)
			ramp_delay = 2500;
		else
			ramp_delay = 10000;
		rinfo->desc.ramp_delay = ramp_delay;
		break;
	default:
		if(!(rval & MAX77714_MASK_SS_L0))
			ramp_delay = 100000;
		else
			ramp_delay = 5000;
		rinfo->desc.ramp_delay = ramp_delay;
		break;
	}
	return 0;
}

static int max77714_set_ramp_delay(struct max77714_regulator *pmic, int id,
				  int slew_rate)
{
	struct max77714_regulator_info *rinfo = pmic->rinfo[id];
	unsigned int val;
	int ret;
	u8 mask;

	if (rinfo->type == MAX77714_REGULATOR_TYPE_SD) {
		/* This bit can be changed only by OTP to avoid the BUCKOV issue */
		return 0;
	} else {
		if (slew_rate <= 5000)
			val = MAX77714_MASK_SS_L0;
		else
			val = 0;
		mask = MAX77714_MASK_SS_L0;
	}

	ret = regmap_update_bits(pmic->rmap, rinfo->cfg_addr, mask, val);
	if (ret < 0) {
		dev_err(pmic->dev, "Regulator %d slew rate set failed: %d\n",
			id, ret);
		return ret;
	}
	return 0;
}

static int max77714_init_pmic(struct max77714_regulator *pmic, int id)
{
	struct max77714_regulator_pdata *rpdata = &pmic->reg_pdata[id];
	int ret;

	/* Update power mode */
	ret = max77714_regulator_get_power_mode(pmic, id);
	if (ret < 0)
		return ret;

	pmic->current_power_mode[id] = ret;
	pmic->enable_power_mode[id] = MAX77714_POWER_MODE_NORMAL;

	if (rpdata->active_fps_src == MAX77714_FPS_SRC_DEF) {
		ret = max77714_regulator_get_fps_src(pmic, id);
		if (ret < 0)
			return ret;
		rpdata->active_fps_src = ret;
	}

	 /* If rails are externally control of FPS then enable it always. */
	if (rpdata->active_fps_src == MAX77714_FPS_SRC_NONE) {
		ret = max77714_regulator_set_power_mode(pmic,
					pmic->enable_power_mode[id], id);
		if (ret < 0)
			return ret;
	} else {
		if (pmic->current_power_mode[id] !=
		     pmic->enable_power_mode[id]) {
			ret = max77714_regulator_set_power_mode(pmic,
					pmic->enable_power_mode[id], id);
			if (ret < 0)
				return ret;
		}
	}

	ret = max77714_regulator_set_fps_src(pmic, rpdata->active_fps_src, id);
	if (ret < 0)
		return ret;

	ret = max77714_regulator_set_fps_slots(pmic, id, false);
	if (ret < 0)
		return ret;

	if (rpdata->ramp_rate_setting) {
		ret = max77714_set_ramp_delay(pmic, id,
					     rpdata->ramp_rate_setting);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int max77714_regulator_enable(struct regulator_dev *rdev)
{
	struct max77714_regulator *pmic = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	if (pmic->active_fps_src[id] != MAX77714_FPS_SRC_NONE)
		return 0;

	return max77714_regulator_set_power_mode(pmic,
			pmic->enable_power_mode[id], id);
}

static int max77714_regulator_disable(struct regulator_dev *rdev)
{
	struct max77714_regulator *pmic = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	if (pmic->active_fps_src[id] != MAX77714_FPS_SRC_NONE)
		return 0;

	return max77714_regulator_set_power_mode(pmic,
			MAX77714_POWER_MODE_DISABLE, id);
}

static int max77714_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct max77714_regulator *pmic = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret = 1;

	if (pmic->active_fps_src[id] != MAX77714_FPS_SRC_NONE)
		return 1;

	ret = max77714_regulator_get_power_mode(pmic, id);
	if (ret < 0)
		return ret;

	if (ret != MAX77714_POWER_MODE_DISABLE)
		return 1;

	return 0;
}

static int max77714_regulator_set_mode(struct regulator_dev *rdev,
				       unsigned int mode)
{
	struct max77714_regulator *pmic = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	struct max77714_regulator_info *rinfo = pmic->rinfo[id];
	struct max77714_regulator_pdata *rpdata = &pmic->reg_pdata[id];
	bool fpwm = false;
	int power_mode;
	int ret;
	u8 val;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		fpwm = true;
		power_mode = MAX77714_POWER_MODE_NORMAL;
		break;

	case REGULATOR_MODE_NORMAL:
		power_mode = MAX77714_POWER_MODE_NORMAL;
		break;

	case REGULATOR_MODE_IDLE:
		power_mode = MAX77714_POWER_MODE_LPM;
		break;

	default:
		dev_err(pmic->dev, "Regulator %d mode %d is invalid\n",
			id, mode);
		return -EINVAL;
	}

	if (rinfo->type != MAX77714_REGULATOR_TYPE_SD)
		goto skip_fpwm;

	val = (fpwm) ? MAX77714_MASK_SD0FPWMEN : 0;
	ret = regmap_update_bits(pmic->rmap, rinfo->cfg_addr,
				 MAX77714_MASK_SD0FPWMEN, val);
	if (ret < 0) {
		dev_err(pmic->dev, "Reg 0x%02x update failed: %d\n",
			rinfo->cfg_addr, ret);
		return ret;
	}
	rpdata->current_mode = mode;

skip_fpwm:
	ret = max77714_regulator_set_power_mode(pmic, power_mode, id);
	if (ret < 0)
		return ret;

	pmic->enable_power_mode[id] = power_mode;

	return 0;
}

static unsigned int max77714_regulator_get_mode(struct regulator_dev *rdev)
{
	struct max77714_regulator *pmic = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	struct max77714_regulator_info *rinfo = pmic->rinfo[id];
	int fpwm = 0;
	int ret;
	int pm_mode, reg_mode;
	unsigned int val;

	ret = max77714_regulator_get_power_mode(pmic, id);
	if (ret < 0)
		return 0;

	pm_mode = ret;
	if (rinfo->type == MAX77714_REGULATOR_TYPE_SD) {
		ret = regmap_read(pmic->rmap, rinfo->cfg_addr, &val);
		if (ret < 0) {
			dev_err(pmic->dev, "Reg 0x%02x read failed: %d\n",
				rinfo->cfg_addr, ret);
			return ret;
		}
		fpwm = !!(val & MAX77714_MASK_SD0FPWMEN);
	}

	switch (pm_mode) {
	case MAX77714_POWER_MODE_NORMAL:
	case MAX77714_POWER_MODE_DISABLE:
		if (fpwm)
			reg_mode = REGULATOR_MODE_FAST;
		else
			reg_mode = REGULATOR_MODE_NORMAL;
		break;
	case MAX77714_POWER_MODE_LPM:
	case MAX77714_POWER_MODE_GLPM:
		reg_mode = REGULATOR_MODE_IDLE;
		break;
	default:
		return 0;
	}

	return reg_mode;
}

static int max77714_regulator_set_ramp_delay(struct regulator_dev *rdev,
					     int ramp_delay)
{
	struct max77714_regulator *pmic = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	struct max77714_regulator_pdata *rpdata = &pmic->reg_pdata[id];

	return max77714_set_ramp_delay(pmic, id, ramp_delay);
}

static int max77714_of_parse_cb(struct device_node *np,
				const struct regulator_desc *desc,
				struct regulator_config *config)
{
	struct max77714_regulator *pmic = config->driver_data;
	struct max77714_regulator_pdata *rpdata = &pmic->reg_pdata[desc->id];
	u32 pval;
	int ret;

	ret = of_property_read_u32(np, "maxim,active-fps-source", &pval);
	rpdata->active_fps_src = (!ret) ? pval : MAX77714_FPS_SRC_DEF;

	ret = of_property_read_u32(np, "maxim,active-fps-power-up-slot", &pval);
	rpdata->active_fps_pu_slot = (!ret) ? pval : -1;

	ret = of_property_read_u32(
			np, "maxim,active-fps-power-down-slot", &pval);
	rpdata->active_fps_pd_slot = (!ret) ? pval : -1;

	ret = of_property_read_u32(np, "maxim,ramp-rate-setting", &pval);
	rpdata->ramp_rate_setting = (!ret) ? pval : 0;

	return max77714_init_pmic(pmic, desc->id);
}

static struct regulator_ops max77714_regulator_ops = {
	.is_enabled = max77714_regulator_is_enabled,
	.enable = max77714_regulator_enable,
	.disable = max77714_regulator_disable,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.set_mode = max77714_regulator_set_mode,
	.get_mode = max77714_regulator_get_mode,
	.set_ramp_delay = max77714_regulator_set_ramp_delay,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_active_discharge = regulator_set_active_discharge_regmap,
};

#define MAX77714_SD_CNF2_ROVS_EN_NONE	0
#define RAIL_SD(_id, _name, _sname, _min_uV, _max_uV, _step_uV)	\
	[MAX77714_REGULATOR_ID_##_id] = {				\
		.type = MAX77714_REGULATOR_TYPE_SD,			\
		.volt_addr = MAX77714_REG_CNFG1_##_id,			\
		.cfg_addr = MAX77714_REG_CNFG2_##_id,			\
		.fps_addr = MAX77714_REG_FPS_##_id,			\
		.power_mode_addr = MAX77714_REG_BUCK_PWR_MD,		\
		.power_mode_mask = MAX77714_MASK_PWR_MD_##_id,		\
		.power_mode_shift = MAX77714_SHIFT_PWR_MD_##_id,	\
		.desc = {						\
			.name = max77714_rails(_name),			\
			.of_match = of_match_ptr(#_name),		\
			.regulators_node = of_match_ptr("regulators"),	\
			.of_parse_cb = max77714_of_parse_cb,		\
			.supply_name = _sname,				\
			.id = MAX77714_REGULATOR_ID_##_id,		\
			.ops = &max77714_regulator_ops,			\
			.n_voltages = ((_max_uV - _min_uV) / _step_uV) + 1, \
			.min_uV = _min_uV,				\
			.uV_step = _step_uV,				\
			.enable_time = 500,				\
			.vsel_mask = MAX77714_MASK_VOUT_##_id,	\
			.vsel_reg = MAX77714_REG_CNFG1_##_id,			\
			.active_discharge_off = 0,			\
			.active_discharge_on = MAX77714_SD_CFG_ADDIS_ENABLE, \
			.active_discharge_mask = MAX77714_SD_CFG_ADDIS_MASK, \
			.active_discharge_reg = MAX77714_REG_CNFG2_##_id, \
			.type = REGULATOR_VOLTAGE,			\
		},							\
	}

#define RAIL_LDO(_id, _name, _sname, _min_uV, _max_uV, _step_uV, 	\
		_pmode_addr)				\
	[MAX77714_REGULATOR_ID_##_id] = {				\
		.type = MAX77714_REGULATOR_TYPE_LDO,		\
		.volt_addr = MAX77714_REG_LDO_CNFG1_##_id,			\
		.cfg_addr = MAX77714_REG_LDO_CNFG2_##_id,			\
		.fps_addr = MAX77714_REG_FPS_##_id,			\
		.power_mode_addr = MAX77714_##_pmode_addr,	\
		.power_mode_mask = MAX77714_MASK_PWR_MD_##_id,	\
		.power_mode_shift = MAX77714_SHIFT_PWR_MD_##_id,	\
		.desc = {						\
			.name = max77714_rails(_name),			\
			.of_match = of_match_ptr(#_name),		\
			.regulators_node = of_match_ptr("regulators"),	\
			.of_parse_cb = max77714_of_parse_cb,		\
			.supply_name = _sname,				\
			.id = MAX77714_REGULATOR_ID_##_id,		\
			.ops = &max77714_regulator_ops,			\
			.n_voltages = ((_max_uV - _min_uV) / _step_uV) + 1, \
			.min_uV = _min_uV,				\
			.uV_step = _step_uV,				\
			.enable_time = 500,				\
			.vsel_mask = MAX77714_MASK_VOUT_LDO_##_id,		\
			.vsel_reg = MAX77714_REG_LDO_CNFG1_##_id,		\
			.active_discharge_off = 0,			\
			.active_discharge_on = MAX77714_MASK_ADE_##_id, \
			.active_discharge_mask = MAX77714_MASK_ADE_##_id, \
			.active_discharge_reg = MAX77714_REG_LDO_CNFG2_##_id, \
			.type = REGULATOR_VOLTAGE,			\
		},							\
	}

static struct max77714_regulator_info max77714_regs_info[MAX77714_NUM_REGS] = {
	RAIL_SD(SD0, sd0, "sd0",
				MAX77714_VOUT_MIN_UV_SDO, MAX77714_VOUT_MAX_UV_SDO,
				MAX77714_VOUT_STEP_UV_SDO),
	RAIL_SD(SD1, sd1, "sd1", 
				MAX77714_VOUT_MIN_UV_SD1, MAX77714_VOUT_MAX_UV_SD1,
				MAX77714_VOUT_STEP_UV_SD1),
	RAIL_SD(SD2, sd2, "sd2",
				MAX77714_VOUT_MIN_UV_SD2,
				MAX77714_VOUT_MAX_UV_SD2 , MAX77714_VOUT_STEP_UV_SD2),
	RAIL_SD(SD3, sd3, "sd3", 
				MAX77714_VOUT_MIN_UV_SD3, MAX77714_VOUT_MAX_UV_SD3,
				MAX77714_VOUT_STEP_UV_SD3),

	RAIL_LDO(L0, ldo0, "ldo0",
				MAX77714_VOUT_MIN_UV_LDO_L0, MAX77714_VOUT_MAX_UV_LDO_L0,
				MAX77714_VOUT_STEP_UV_LDO_L0, REG_LDO_PWR_MD0_3),
	RAIL_LDO(L1, ldo1, "ldo1",
				MAX77714_VOUT_MIN_UV_LDO_L1, MAX77714_VOUT_MAX_UV_LDO_L1,
				MAX77714_VOUT_STEP_UV_LDO_L1, REG_LDO_PWR_MD0_3),
	RAIL_LDO(L2, ldo2, "ldo2",
				MAX77714_VOUT_MIN_UV_LDO_L2, MAX77714_VOUT_MAX_UV_LDO_L2,
				MAX77714_VOUT_STEP_UV_LDO_L2, REG_LDO_PWR_MD0_3),
	RAIL_LDO(L3, ldo3, "ldo3",
				MAX77714_VOUT_MIN_UV_LDO_L3, MAX77714_VOUT_MAX_UV_LDO_L3,
				MAX77714_VOUT_STEP_UV_LDO_L3, REG_LDO_PWR_MD0_3),
	RAIL_LDO(L4, ldo4, "ldo4",
				MAX77714_VOUT_MIN_UV_LDO_L4, MAX77714_VOUT_MAX_UV_LDO_L4,
				MAX77714_VOUT_STEP_UV_LDO_L4, REG_LDO_PWR_MD4_7),
	RAIL_LDO(L5, ldo5, "ldo5",
				MAX77714_VOUT_MIN_UV_LDO_L5, MAX77714_VOUT_MAX_UV_LDO_L5,
				MAX77714_VOUT_STEP_UV_LDO_L5, REG_LDO_PWR_MD4_7),
	RAIL_LDO(L6, ldo6, "ldo6",
				MAX77714_VOUT_MIN_UV_LDO_L6, MAX77714_VOUT_MAX_UV_LDO_L6,
				MAX77714_VOUT_STEP_UV_LDO_L6, REG_LDO_PWR_MD4_7),
	RAIL_LDO(L7, ldo7, "ldo7",
				MAX77714_VOUT_MIN_UV_LDO_L7, MAX77714_VOUT_MAX_UV_LDO_L7,
				MAX77714_VOUT_STEP_UV_LDO_L7, REG_LDO_PWR_MD4_7),
	RAIL_LDO(L8, ldo8, "ldo8",
				MAX77714_VOUT_MIN_UV_LDO_L8, MAX77714_VOUT_MAX_UV_LDO_L8,
				MAX77714_VOUT_STEP_UV_LDO_L8, REG_LDO_PWR_MD8),
};


static int max77714_regulator_probe(struct platform_device *pdev)
{
	struct max77714_chip *max77714_chip = dev_get_drvdata(pdev->dev.parent);
	struct max77714_regulator_info *rinfo;
	struct device *dev = &pdev->dev;
	struct regulator_config config = { };
	struct max77714_regulator *pmic;
	int ret = 0;
	int id;

	pmic = devm_kzalloc(dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	platform_set_drvdata(pdev, pmic);
	pmic->dev = dev;
	pmic->rmap = max77714_chip->rmap;
	if (!dev->of_node)
		dev->of_node = pdev->dev.parent->of_node;

	rinfo = max77714_regs_info;
	config.regmap = pmic->rmap;
	config.dev = dev;
	config.driver_data = pmic;

	for (id = 0; id < MAX77714_NUM_REGS; id++) {
		struct regulator_dev *rdev;
		struct regulator_desc *rdesc;

		rdesc = &rinfo[id].desc;
		pmic->rinfo[id] = &max77714_regs_info[id];
		pmic->enable_power_mode[id] = MAX77714_POWER_MODE_NORMAL;

		ret = max77714_read_ramp_delay(pmic, id);
		if (ret < 0)
			return ret;

		rdev = devm_regulator_register(dev, rdesc, &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(dev, "Regulator registration %s failed: %d\n",
				rdesc->name, ret);
			return ret;
		}
	}
	return 0;
}

static const struct platform_device_id max77714_regulator_devtype[] = {
	{ .name = "max77714-pmic", },
	{},
};
MODULE_DEVICE_TABLE(platform, max77714_regulator_devtype);

static struct platform_driver max77714_regulator_driver = {
	.probe = max77714_regulator_probe,
	.id_table = max77714_regulator_devtype,
	.driver = {
		.name = "max77714-pmic",
	},
};

module_platform_driver(max77714_regulator_driver)

MODULE_AUTHOR("Daniel.Jeong <daniel.jeong@maximintegrated.com>");
MODULE_AUTHOR("Maxim LDD <opensource@maximintegrated.com>");
MODULE_DESCRIPTION("MAX77714 Regulator Driver");
MODULE_LICENSE("GPL v2");