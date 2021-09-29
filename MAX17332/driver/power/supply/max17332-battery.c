// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2021 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/mfd/max17332.h>
#include <linux/power/max17332-battery.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

static char *batt_supplied_to[] = {
	"max17332-battery",
};

static inline int max17332_raw_voltage_to_uvolts(u16 lsb)
{
	return lsb * 625 / 8; /* 78.125uV per bit */
}

static inline int max17332_raw_current_to_uamps(struct max17332_fg_chip *chip,
												int curr)
{
	return curr * 15625 / ((int)chip->rsense * 10);
}

static enum power_supply_property max17332_fg_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
};


static struct device_attribute max17332_fg_attrs[] = {
	MAX17332_FG_ATTR(max17332_program_nvm),
	MAX17332_FG_ATTR(max17332_remaining_nvm_updates),
};

static ssize_t program_nvm_memory_store(struct max17332_fg_chip *chip,
						const char *buf, size_t len)
{
	int ret = 0, i = 0;
	int size;
	u16 data;
	const u8 *ptr;
	char *fw_path;
	u16 memory_data = 0;
	u8 memory_addr = 0, try_counter = 0;
	const struct firmware *fw;

	fw_path = kstrdup(buf, GFP_ATOMIC);
	if (!fw_path)
		return -EINVAL;

	fw_path = strim(fw_path);

	if (len < 1)
		return -EINVAL;

	ret = request_firmware(&fw, fw_path, chip->dev);

	if (ret < 0 || !fw->data || !fw->size) {
		pr_err("%s : Failed to get ini file\n", __func__);
		return ret;
	}

	size = fw->size;
	ptr = fw->data;

	if (size != ((MAX17332_NVM_HIGH_ADDR - MAX17332_NVM_BASE_ADDR + 1) * 3)) {
		pr_err("%s : Firmware size is not correct!\n", __func__);
		release_firmware(fw);
		return ret;
	}

	size = (MAX17332_NVM_HIGH_ADDR - MAX17332_NVM_BASE_ADDR + 1);
	mutex_lock(&chip->lock);

nvm_block_copy:
	// Break infinite loop
	if (try_counter > 3) {
		pr_err("%s: fail to write nvm memory succesfully\n", __func__);
		ret = -EAGAIN;
		goto error;
	}
	try_counter++;

	for (i = 0; i < size; i++) {
		memory_data = (ptr[(i*3)+1] << 8) + ptr[(i*3)+2];
		memory_addr = ptr[(i*3)];
		if ((memory_addr >= REG_NROMID0_NVM) && (memory_addr <= REG_NROMID3_NVM))
			continue;

		ret = max17332_write(chip->regmap_nvm, memory_addr, memory_data);
		if (ret < 0) {
			pr_err("%s: fail to write reg 0x%X\n", __func__, memory_addr);
			goto error;
		}
	}
	return len;

	// Verify All Nonvolatile Memory Locations
	for (i = 0; i < size; i++) {
		memory_data = (ptr[(i*3)+1] << 8) + ptr[(i*3)+2];
		memory_addr = ptr[(i*3)];
		if ((memory_addr >= REG_NROMID0_NVM) && (memory_addr <= REG_NROMID3_NVM))
			continue;

		ret = max17332_read(chip->regmap_nvm, memory_addr, &data);
		if (ret < 0) {
			pr_err("%s: fail to read reg 0x%X\n", __func__, memory_addr);
			goto error;
		}
		if (data != memory_data)
			goto nvm_block_copy;
	}

	// Clear CommStat.NVError
	ret = max17332_read(chip->regmap, REG_COMMSTAT, &data);
	ret |= max17332_write(chip->regmap, REG_COMMSTAT, (data & ~(MAX17332_COMMSTAT_NVERROR)));
	if (ret < 0) {
		pr_err("%s: fail to access REG_COMMSTAT\n", __func__);
		goto error;
	}

	// Initiate Block Copy
	ret = max17332_write(chip->regmap, REG_COMMAND, MAX17332_COMMAND_COPY_NVM);
	if (ret < 0) {
		pr_err("%s: fail to initiate block copy\n", __func__);
		goto error;
	}

	// Wait t_block for Copy to Complete
	// ! Maximum time for block programming is 7360 ms
	mdelay(1000);

	// Check CommStat.NVError bit
	ret = max17332_read(chip->regmap, REG_COMMSTAT, &data);
	if (ret < 0) {
		pr_err("%s: fail to read REG_COMMSTAT\n", __func__);
		goto error;
	}

	if ((data & MAX17332_COMMSTAT_NVERROR) == MAX17332_COMMSTAT_NVERROR) {
		pr_err("%s: fail to clear CommStat.NVError\n", __func__);
		goto error;
	}

	// Initiate Full Reset
	ret = max17332_write(chip->regmap, REG_COMMAND, MAX17332_COMMAND_FULL_RESET);
	if (ret < 0) {
		pr_err("%s: fail to sent Full Reset command\n", __func__);
		goto error;
	}

	// Wait 10ms for IC to Rest
	mdelay(10);

	// Verify All Nonvolatile Memory Locations Recalled Correctly
	for (i = 0; i < size; i++) {
		memory_data = (ptr[(i*3)+1] << 8) + ptr[(i*3)+2];
		memory_addr = ptr[(i*3)];
		if ((memory_addr >= REG_NROMID0_NVM) && (memory_addr <= REG_NROMID3_NVM))
			continue;

		ret = max17332_read(chip->regmap_nvm, memory_addr, &data);
		if (ret < 0) {
			pr_err("%s: fail to read register 0x%X\n", __func__, memory_addr);
			goto error;
		}

		if (data != memory_data) {
			pr_err("%s: Failed to verify register 0x%X\n", __func__, memory_addr);
			goto error;
		}
	}

	// Clear CommStat.NVError
	ret = max17332_read(chip->regmap, REG_COMMSTAT, &data);
	ret |= max17332_write(chip->regmap, REG_COMMSTAT, (data & ~(MAX17332_COMMSTAT_NVERROR)));
	if (ret < 0) {
		pr_err("%s: fail to access REG_COMMSTAT\n", __func__);
		goto error;
	}

	// FuelGauge Reset Command
	max17332_update_bits(chip->regmap, REG_CONFIG2,
		MAX17332_CONFIG2_POR_CMD,
		MAX17332_CONFIG2_POR_CMD);

	// Wait 500 ms for POR_CMD to clear;
	mdelay(500);

	mutex_unlock(&chip->lock);
	return len;
error:
	mutex_unlock(&chip->lock);
	return ret;
}

static ssize_t remaining_nmv_updates_show(struct max17332_fg_chip *chip, char *buf)
{
	int rc = 0, ret = 0;
	u16 val = 0, logical_val = 0;
	u8 number_of_used = 0;

	// Recall indicator flags to determine remaining configuration memory writes
	ret = max17332_write(chip->regmap, REG_COMMAND, MAX17332_COMMAND_RECALL_HISTORY_REMAINING_WRITES);
	if (ret < 0) {
		pr_err("%s: fail to sent Recall History command\n", __func__);
		return ret;
	}

	// Wait t_recall
	mdelay(5);

	ret = max17332_read(chip->regmap_nvm, REG_REMAINING_UPDATES_NVM, &val);
	if (ret < 0) {
		pr_err("%s: fail to read REG_REMAINING_UPDATES_NVM\n", __func__);
		return ret;
	}

	logical_val = ((val & 0xFF) | ((val >> 8) & 0xFF));
	number_of_used = fls(logical_val);

	rc += snprintf(buf + rc, PAGE_SIZE - rc, "Numer of Updates Remaining : %d\n", (8 - number_of_used));
	return rc;
}

ssize_t max17332_fg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct max17332_fg_chip *chip = power_supply_get_drvdata(psy);
	const ptrdiff_t offset = attr - max17332_fg_attrs;
	int i = 0;

	dev_info(chip->dev, "%s\n", __func__);

	switch (offset) {
	case MAX17332_FG_REMAINING_NVM_UPDATES:
		i = remaining_nmv_updates_show(chip, buf);
		break;
	default:
		return -EINVAL;
	}
	return i;
}

ssize_t max17332_fg_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct max17332_fg_chip *chip = power_supply_get_drvdata(psy);
	const ptrdiff_t offset = attr - max17332_fg_attrs;
	int ret;

	dev_info(chip->dev, "%s\n", __func__);

	switch (offset) {
	case MAX17332_FG_PROGRAM_NVM:
		program_nvm_memory_store(chip, buf, count);
		ret = count;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}


static int max17332_fg_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < (int)ARRAY_SIZE(max17332_fg_attrs); i++) {
		rc = device_create_file(dev, &max17332_fg_attrs[i]);
		if (rc)
			goto create_attrs_failed;
	}
	return rc;

create_attrs_failed:
	dev_err(dev, "%s: failed (%d)\n", __func__, rc);
	while (i--)
		device_remove_file(dev, &max17332_fg_attrs[i]);
	return rc;
}

static int max17332_get_temperature(struct max17332_fg_chip *chip, int *temp)
{
	int ret;
	u16 val;

	ret = max17332_read(chip->regmap, REG_TEMP, &val);
	if (ret < 0) {
		pr_err("%s: fail to read REG_TEMP\n", __func__);
		return ret;
	}

	*temp = sign_extend32(val, 15);
	/* The value is converted into centigrade scale */
	/* Units of LSB = 1 / 256 degree Celsius */
	*temp = (*temp * 10) >> 8;
	return 0;
}

static int max17332_get_temperature_alert_min(struct max17332_fg_chip *chip,
											  int *temp)
{
	int ret;
	u16 val;

	ret = max17332_read(chip->regmap, REG_TALRTTH, &val);
	if (ret < 0) {
		pr_err("%s: fail to read REG_TALRTTH\n", __func__);
		return ret;
	}

	/* Convert 1DegreeC LSB to 0.1DegreeC LSB */
	*temp = sign_extend32(val & 0xff, 7) * 10;

	return 0;
}

static int max17332_get_temperature_alert_max(struct max17332_fg_chip *chip,
											  int *temp)
{
	int ret;
	u16 val;

	ret = max17332_read(chip->regmap, REG_TALRTTH, &val);
	if (ret < 0) {
		pr_err("%s: fail to read REG_TALRTTH\n", __func__);
		return ret;
	}

	/* Convert 1DegreeC LSB to 0.1DegreeC LSB */
	*temp = sign_extend32(val >> 8, 7) * 10;

	return 0;
}

static int max17332_get_battery_health(struct max17332_fg_chip *chip, int *health)
{
	int ret;
	u16 val;

	ret = max17332_read(chip->regmap, REG_PROTSTATUS, &val);
	if (ret < 0) {
		pr_err("%s: fail to read REG_PROTSTATUS\n", __func__);
		return ret;
	}

	if (val & BIT_PREQF_INT) {
		*health = POWER_SUPPLY_HEALTH_UNKNOWN;
	} else if ((val & BIT_TOOHOTC_INT) ||
			 (val & BIT_TOOHOTD_INT) ||
			 (val & BIT_DIEHOT_INT)) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if ((val & BIT_UVP_INT) ||
			 (val & BIT_PERMFAIL_INT) ||
			 (val & BIT_SHDN_INT)) {
		*health = POWER_SUPPLY_HEALTH_DEAD;
	} else if ((val & BIT_TOOCOLDC_INT) ||
			 (val & BIT_TOOCOLDD_INT)) {
		*health = POWER_SUPPLY_HEALTH_COLD;
	} else if (val & BIT_OVP_INT) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	} else if ((val & BIT_QOVFLW_INT) ||
			 (val & BIT_OCCP_INT) ||
			 (val & BIT_ODCP_INT)) {
		*health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	} else if (val & BIT_CHGWDT_INT) {
		*health = POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
	} else {
		*health = POWER_SUPPLY_HEALTH_GOOD;
	}

	return 0;
}

static int max17332_set_temp_lower_limit(struct max17332_fg_chip *chip,
										 int temp)
{
	int ret;
	u16 data;

	ret = max17332_read(chip->regmap, REG_TALRTTH, &data);
	if (ret < 0) {
		pr_err("%s: fail to read REG_TALRTTH\n", __func__);
		return ret;
	}

	/* Input in deci-centigrade, convert to centigrade */
	temp /= 10;

	data &= 0xFF00;
	data |= (temp & 0xFF);

	ret = max17332_write(chip->regmap, REG_TALRTTH, data);
	if (ret < 0) {
		pr_err("%s: fail to write REG_TALRTTH\n", __func__);
		return ret;
	}

	return 0;
}

static int max17332_set_temp_upper_limit(struct max17332_fg_chip *chip,
										 int temp)
{
	int ret;
	u16 data;

	ret = max17332_read(chip->regmap, REG_TALRTTH, &data);
	if (ret < 0) {
		pr_err("%s: fail to read REG_TALRTTH\n", __func__);
		return ret;
	}

	/* Input in deci-centigrade, convert to centigrade */
	temp /= 10;

	data &= 0xFF;
	data |= ((temp << 8) & 0xFF00);

	ret = max17332_write(chip->regmap, REG_TALRTTH, data);
	if (ret < 0) {
		pr_err("%s: fail to write REG_TALRTTH\n", __func__);
		return ret;
	}

	return 0;
}

static int max17332_set_min_capacity_alert_th(struct max17332_fg_chip *chip,
											  unsigned int th)
{
	int ret;
	u16 data;

	ret = max17332_read(chip->regmap, REG_SALRTTH, &data);
	if (ret < 0) {
		pr_err("%s: fail to read REG_SALRTTH\n", __func__);
		return ret;
	}

	data &= 0xFF00;
	data |= (th & 0xFF);

	ret = max17332_write(chip->regmap, REG_SALRTTH, data);
	if (ret < 0) {
		pr_err("%s: fail to write REG_SALRTTH\n", __func__);
		return ret;
	}

	return 0;
}

static int max17332_set_max_capacity_alert_th(struct max17332_fg_chip *chip,
											  unsigned int th)
{
	int ret;
	u16 data;

	ret = max17332_read(chip->regmap, REG_SALRTTH, &data);
	if (ret < 0) {
		pr_err("%s: fail to read REG_SALRTTH\n", __func__);
		return ret;
	}

	data &= 0xFF;
	data |= ((th & 0xFF) << 8);

	ret = max17332_write(chip->regmap, REG_SALRTTH, data);
	if (ret < 0) {
		pr_err("%s: fail to write REG_SALRTTH\n", __func__);
		return ret;
	}

	return 0;
}

static unsigned int max17332_get_vbat(struct max17332_fg_chip *chip)
{
	int vavg, vbatt, ret;
	u16 val;

	ret = max17332_read(chip->regmap, REG_AVGVCELL, &val);
	if (ret < 0) {
		pr_err("%s: fail to read REG_AVGVCELL\n", __func__);
		return ret;
	}

	/* bits [0-3] unused */
	vavg = max17332_raw_voltage_to_uvolts(val);
	/* Convert to millivolts */
	vavg /= 1000;

	ret = max17332_read(chip->regmap, REG_VCELL, &val);
	if (ret < 0) {
		pr_err("%s: fail to read REG_VCELL\n", __func__);
		return ret;
	}

	/* bits [0-3] unused */
	vbatt = max17332_raw_voltage_to_uvolts(val);
	/* Convert to millivolts */
	vbatt /= 1000;

	pr_info("%s vavg = %dmV, vbat = %dmV(0x%4x)\n",
					__func__, vavg, vbatt, val);

	return vbatt;
}

static int max17332_fg_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	u16 reg;
	struct max17332_fg_chip *chip =
		power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = max17332_read(chip->regmap, REG_VCELL, &reg);
		if (ret < 0)
			return ret;
		val->intval = max17332_raw_voltage_to_uvolts(reg);
		/* Convert to millivolts */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = max17332_read(chip->regmap, REG_AVGVCELL, &reg);
		if (ret < 0)
			return ret;
		val->intval = max17332_raw_voltage_to_uvolts(reg);
		/* Convert to millivolts */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = max17332_read(chip->regmap, REG_MAXMINVOLT, &reg);
		if (ret < 0)
			return ret;
		val->intval = reg >> 8;
		val->intval *= 20; /* Units of LSB = 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = max17332_read(chip->regmap, REG_VEMPTY, &reg);
		if (ret < 0)
			return ret;
		val->intval = reg >> 7;
		val->intval *= 10; /* Units of LSB = 10mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = max17332_read(chip->regmap, REG_VFOCV, &reg);
		if (ret < 0)
			return ret;
		val->intval = max17332_raw_voltage_to_uvolts(reg);
		/* Convert to millivolts */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = max17332_read(chip->regmap, REG_REPSOC, &reg);
		if (ret < 0)
			return ret;
		val->intval = reg >> 8; /* RepSOC LSB: 1/256 % */
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		ret = max17332_read(chip->regmap, REG_SALRTTH, &reg);
		if (ret < 0)
			return ret;
		val->intval = reg & 0xFF;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		ret = max17332_read(chip->regmap, REG_SALRTTH, &reg);
		if (ret < 0)
			return ret;
		val->intval = (reg >> 8) & 0xFF;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max17332_get_battery_health(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = max17332_get_temperature(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = max17332_get_temperature_alert_min(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = max17332_get_temperature_alert_max(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = max17332_read(chip->regmap, REG_CURRENT, &reg);
		if (ret < 0)
			return ret;
		val->intval = max17332_raw_current_to_uamps(chip, sign_extend32(reg, 15));
		/* Convert to milliamps */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = max17332_read(chip->regmap, REG_AVGCURRENT, &reg);
		if (ret < 0)
			return ret;
		val->intval = max17332_raw_current_to_uamps(chip, sign_extend32(reg, 15));
		/* Convert to milliamps */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = max17332_read(chip->regmap, REG_TTE, &reg);
		if (ret < 0)
			return ret;
		val->intval = (reg * 45) >> 3; /* TTE LSB: 5.625 sec */
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		ret = max17332_read(chip->regmap, REG_TTF, &reg);
		if (ret < 0)
			return ret;
		val->intval = (reg * 45) >> 3; /* TTF LSB: 5.625 sec */
		break;
	default:
		return -EINVAL;

	}
	return 0;
}

static int max17332_fg_set_property(struct power_supply *psy,
								 enum power_supply_property psp,
								 const union power_supply_propval *val)
{
	struct max17332_fg_chip *chip =
		power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = max17332_set_temp_lower_limit(chip, val->intval);
		if (ret < 0)
			dev_err(chip->dev, "temp alert min set fail:%d\n",
					ret);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = max17332_set_temp_upper_limit(chip, val->intval);
		if (ret < 0)
			dev_err(chip->dev, "temp alert max set fail:%d\n",
					ret);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		ret = max17332_set_min_capacity_alert_th(chip, val->intval);
		if (ret < 0)
			dev_err(chip->dev, "capacity alert min set fail:%d\n",
					ret);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		ret = max17332_set_max_capacity_alert_th(chip, val->intval);
		if (ret < 0)
			dev_err(chip->dev, "capacity alert max set fail:%d\n",
					ret);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int max17332_property_is_writeable(struct power_supply *psy,
										  enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static void max17332_set_alert_thresholds(struct max17332_fg_chip *chip)
{
	struct max17332_fg_platform_data *pdata = chip->pdata;
	u16 val;
	int ret;

	/* Set VAlrtTh */
	val = (pdata->volt_min / 20);
	val |= ((pdata->volt_max / 20) << 8);
	ret = max17332_write(chip->regmap, REG_VALRTTH, val);
	if (ret < 0) {
		pr_err("%s: fail to write REG_VALRTTH\n", __func__);
		return;
	}

	/* Set TAlrtTh */
	val = pdata->temp_min & 0xFF;
	val |= ((pdata->temp_max & 0xFF) << 8);
	ret = max17332_write(chip->regmap, REG_TALRTTH, val);
	if (ret < 0) {
		pr_err("%s: fail to write REG_TALRTTH\n", __func__);
		return;
	}

	/* Set SAlrtTh */
	val = pdata->soc_min;
	val |= (pdata->soc_max << 8);
	ret = max17332_write(chip->regmap, REG_SALRTTH, val);
	if (ret < 0) {
		pr_err("%s: fail to write REG_SALRTTH\n", __func__);
		return;
	}

	/* Set IAlrtTh */
	val = (pdata->curr_min * chip->rsense / 400) & 0xFF;
	val |= (((pdata->curr_max * chip->rsense / 400) & 0xFF) << 8);
	ret = max17332_write(chip->regmap, REG_IALRTTH, val);
	if (ret < 0) {
		pr_err("%s: fail to write REG_IALRTTH\n", __func__);
		return;
	}
}

static int max17332_fg_initialize(struct max17332_fg_chip *chip)
{
	int ret;
	u16 reg;
	u16 fgrev;

	ret = max17332_read(chip->regmap, REG_VERSION, &fgrev);
	if (ret < 0) {
		dev_err(chip->dev, "%s: fail to read REG_VERSION\n", __func__);
		return ret;
	}

	dev_info(chip->dev, "IC Version: 0x%04x\n", fgrev);

	/* Optional step - alert threshold initialization */
	max17332_set_alert_thresholds(chip);

	/* Clear Status.POR */
	ret = max17332_read(chip->regmap, REG_STATUS, &reg);
	if (ret < 0) {
		dev_err(chip->dev, "%s: fail to read REG_STATUS\n", __func__);
		return ret;
	}

	ret = max17332_write(chip->regmap, REG_STATUS,
											reg & ~BIT_STATUS_POR);
	if (ret < 0) {
		dev_err(chip->dev, "%s: fail to write REG_STATUS\n", __func__);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_OF
static int max17332_fg_parse_dt(struct max17332_fg_chip *battery)
{
	struct device *dev = battery->dev;
	struct device_node *np = of_find_node_by_name(NULL, "battery");
	struct max17332_fg_platform_data *pdata;
	int ret = 0;

	pr_info("%s start\n", __func__);
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(pdata == NULL))
		return -ENOMEM;

	pr_info("%s irq info\n", __func__);
	/* reset, irq gpio info */
	if (np == NULL) {
		pr_err("%s np NULL\n", __func__);
		return -EINVAL;
	}

	pr_info("%s talrt-min\n", __func__);
	ret = of_property_read_u32(np, "talrt-min", &pdata->temp_min);
	if (ret < 0)
		pdata->temp_min = -128; /* DegreeC */ /* Disable alert */

	pr_info("%s talrt-max\n", __func__);
	ret = of_property_read_u32(np, "talrt-max", &pdata->temp_max);
	if (ret < 0)
		pdata->temp_max = 127; /* DegreeC */ /* Disable alert */

	pr_info("%s valrt-min\n", __func__);
	ret = of_property_read_u32(np, "valrt-min", &pdata->volt_min);
	if (ret < 0)
		pdata->volt_min = 0; /* mV */ /* Disable alert */

	pr_info("%s valrt-max\n", __func__);
	ret = of_property_read_u32(np, "valrt-max", &pdata->volt_max);
	if (ret < 0)
		pdata->volt_max = 5100; /* mV */ /* Disable alert */

	pr_info("%s ialrt-min\n", __func__);
	ret = of_property_read_u32(np, "ialrt-min", &pdata->curr_min);
	if (ret < 0)
		pdata->curr_min = -5120; /* mA */ /* Disable alert */

	pr_info("%s ialrt-max\n", __func__);
	ret = of_property_read_u32(np, "ialrt-max", &pdata->curr_max);
	if (ret < 0)
		pdata->curr_max = 5080; /* mA */ /* Disable alert */

	pr_info("%s salrt-min\n", __func__);
	ret = of_property_read_u32(np, "salrt-min", &pdata->soc_min);
	if (ret < 0)
		pdata->soc_min = 0; /* Percent */ /* Disable alert */

	pr_info("%s salrt-max\n", __func__);
	ret = of_property_read_u32(np, "salrt-max", &pdata->soc_max);
	if (ret < 0)
		pdata->soc_max = 255; /* Percent */ /* Disable alert */

	battery->pdata = pdata;
	pr_info("%s finished\n", __func__);
	return 0;

}
#endif

static irqreturn_t max17332_fg_protalrt_irq_isr(int irq, void *data)
{
	struct max17332_fg_chip *chip = data;
	int ret;
	u16 val;

	dev_info(chip->dev, "Protection Interrupt Handler!\n");

	/* Check Protection Alert type */

	ret = max17332_read(chip->regmap, REG_PROGALRTS, &val);
	if (ret < 0) {
		dev_err(chip->dev, "%s: fail to read REG_PROGALRTS\n", __func__);
		return IRQ_HANDLED;
	}

	if (val & BIT_CHGWDT_INT)
		dev_info(chip->dev, "Protection Alert: Charge Watch Dog Timer!\n");
	if (val & BIT_TOOHOTC_INT)
		dev_info(chip->dev, "Protection Alert: Overtemperature for Charging!\n");
	if (val & BIT_FULL_INT)
		dev_info(chip->dev, "Protection Alert: Full Detection!\n");
	if (val & BIT_TOOCOLDC_INT)
		dev_info(chip->dev, "Protection Alert: Undertemperature!\n");
	if (val & BIT_OVP_INT)
		dev_info(chip->dev, "Protection Alert: Overvoltage!\n");
	if (val & BIT_OCCP_INT)
		dev_info(chip->dev, "Protection Alert: Overcharge Current!\n");
	if (val & BIT_QOVFLW_INT)
		dev_info(chip->dev, "Protection Alert: Q Overflow!\n");
	if (val & BIT_PERMFAIL_INT)
		dev_info(chip->dev, "Protection Alert: Permanent Failure!\n");
	if (val & BIT_DIEHOT_INT)
		dev_info(chip->dev, "Protection Alert: Overtemperature for die temperature!\n");
	if (val & BIT_TOOHOTD_INT)
		dev_info(chip->dev, "Protection Alert: Overtemperature for Discharging!\n");
	if (val & BIT_UVP_INT)
		dev_info(chip->dev, "Protection Alert: Undervoltage Protection!\n");
	if (val & BIT_ODCP_INT)
		dev_info(chip->dev, "Protection Alert: Overdischarge current!\n");

	/* Clear alerts */
	ret = max17332_write(chip->regmap, REG_PROGALRTS, val & REG_PROGALRTS_MASK);
	if (ret < 0)
		dev_err(chip->dev, "%s: fail to write REG_PROGALRTS\n", __func__);

	return IRQ_HANDLED;
}

static irqreturn_t max17332_fg_irq_isr(int irq, void *data)
{
	struct max17332_fg_chip *chip = data;
	int ret;
	u16 val;

	dev_info(chip->dev, "STATUS : Interrupt Handler!\n");

	/* Check alert type */
	ret = max17332_read(chip->regmap, REG_STATUS, &val);
	if (ret < 0) {
		dev_err(chip->dev, "%s: fail to read REG_STATUS\n", __func__);
		return IRQ_HANDLED;
	}

	if (val & BIT_STATUS_SMX)
		dev_info(chip->dev, "Alert: SOC MAX!\n");
	if (val & BIT_STATUS_SMN)
		dev_info(chip->dev, "Alert: SOC MIN!\n");
	if (val & BIT_STATUS_TMX)
		dev_info(chip->dev, "Alert: TEMP MAX!\n");
	if (val & BIT_STATUS_TMN)
		dev_info(chip->dev, "Alert: TEMP MIN!\n");
	if (val & BIT_STATUS_VMX)
		dev_info(chip->dev, "Alert: VOLT MAX!\n");
	if (val & BIT_STATUS_VMN)
		dev_info(chip->dev, "Alert: VOLT MIN!\n");
	if (val & BIT_STATUS_IMX)
		dev_info(chip->dev, "Alert: CURR MAX!\n");
	if (val & BIT_STATUS_IMN)
		dev_info(chip->dev, "Alert: CURR MIN!\n");

	if (val & BIT_STATUS_CA) {
		dev_info(chip->dev, "Alert: CHARGING!\n");

		/* Check Charging type */
		ret = max17332_read(chip->regmap, REG_CHGSTAT, &val);

		if (val & BIT_STATUS_CP)
			dev_info(chip->dev, "Charging Alert: Heat limit!\n");
		if (val & BIT_STATUS_CT)
			dev_info(chip->dev, "Charging Alert: FET Temperature limit!\n");
		if (val & BIT_STATUS_DROPOUT)
			dev_info(chip->dev, "Charging Alert: Dropout!\n");
	}

	/* Clear alerts */
	ret = max17332_write(chip->regmap, REG_STATUS, val & REG_STATUS_MASK);
	if (ret < 0)
		dev_err(chip->dev, "%s: fail to write REG_PROGALRTS\n", __func__);

	power_supply_changed(chip->battery);

	return IRQ_HANDLED;
}

static int max17332_fg_probe(struct platform_device *pdev)
{
	struct max17332_dev *max17332 = dev_get_drvdata(pdev->dev.parent);
	struct max17332_fg_platform_data *pdata =
		dev_get_platdata(max17332->dev);
	struct max17332_fg_chip *chip;
	int ret = 0;
	struct power_supply_config fg_cfg = {};

	pr_info("%s: MAX17332 Fuelgauge Driver Loading\n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		pr_err("%s: out of memory\n", __func__);
		pdata = ERR_PTR(-ENOMEM);
		return -ENOMEM;
	}

	mutex_init(&chip->lock);

	chip->dev = &pdev->dev;
	chip->max17332 = max17332;
	chip->regmap = max17332->regmap_pmic;
	chip->regmap_nvm = max17332->regmap_nvm;
	chip->pdata = pdata;
	chip->rsense = max17332->pdata->rsense;

#if defined(CONFIG_OF)
	ret = max17332_fg_parse_dt(chip);
	if (ret < 0) {
		pr_err("%s not found fuelgauge dt! ret[%d]\n",
				__func__, ret);
	}
#else
	pdata = dev_get_platdata(&pdev->dev);
#endif

	platform_set_drvdata(pdev, chip);
	chip->psy_batt_d.name     = "max17332-battery";
	chip->psy_batt_d.type     = POWER_SUPPLY_TYPE_BATTERY;
	chip->psy_batt_d.properties   = max17332_fg_battery_props;
	chip->psy_batt_d.get_property = max17332_fg_get_property;
	chip->psy_batt_d.set_property = max17332_fg_set_property;
	chip->psy_batt_d.property_is_writeable   = max17332_property_is_writeable;
	chip->psy_batt_d.num_properties =
		ARRAY_SIZE(max17332_fg_battery_props);
	fg_cfg.drv_data = chip;
	fg_cfg.supplied_to = batt_supplied_to;
	fg_cfg.of_node = max17332->dev->of_node;
	fg_cfg.num_supplicants = ARRAY_SIZE(batt_supplied_to);

	chip->battery =
		devm_power_supply_register(max17332->dev,
				&chip->psy_batt_d,
				&fg_cfg);
	if (IS_ERR(chip->battery)) {
		pr_err("Couldn't register battery rc=%ld\n",
				PTR_ERR(chip->battery));
		goto error;
	}

	if (max17332->irq > 0) {
		ret = devm_request_threaded_irq(max17332->dev, max17332->irq, NULL,
			max17332_fg_irq_isr,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED,
			"fuelgauge-irq", chip);
		if (ret) {
			pr_err("%s: Failed to Reqeust IRQ\n", __func__);
			goto error_fg_irq;
		}
		dev_info(&pdev->dev,
			"MAX17332 Fuel-Gauge irq requested %d\n", max17332->irq);
	}

	chip->fg_prot_irq = regmap_irq_get_virq(max17332->irqc_intsrc,
	  MAX17332_FG_PROT_INT);

	dev_info(&pdev->dev, "MAX17332 Fuel-Gauge prot_irq %d\n",
			chip->fg_prot_irq);

	if (chip->fg_prot_irq > 0) {
		ret = request_threaded_irq(chip->fg_prot_irq, NULL,
			max17332_fg_protalrt_irq_isr,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED,
			"fuelgauge-protalrt-irq", chip);
		if (ret) {
			pr_err("%s: Failed to Reqeust IRQ\n", __func__);
			goto error_fg_prot_irq;
		}
	}

	max17332_update_bits(chip->regmap, REG_CONFIG,
		BIT_CONFIG_ALRT_EN,
		BIT_CONFIG_ALRT_EN);

	/* Clear alerts */
	max17332_write(chip->regmap, REG_PROGALRTS, REG_PROGALRTS_MASK);
	max17332_write(chip->regmap, REG_STATUS, REG_STATUS_MASK);

	ret = max17332_fg_initialize(chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error: Initializing fuel-gauge\n");
		goto error_init;
	}

	ret = max17332_fg_create_attrs(&chip->battery->dev);
	if (ret) {
		dev_err(chip->dev,
			"%s : Failed to create_attrs\n", __func__);
	}

	pr_info("%s: Done to Load MAX17332 Fuelgauge Driver\n", __func__);
	return 0;

error_init:
	if (chip->fg_prot_irq)
		free_irq(chip->fg_prot_irq, chip);
error_fg_prot_irq:
	if (max17332->irq)
		free_irq(max17332->irq, chip);
error_fg_irq:
	power_supply_unregister(chip->battery);
error:
	kfree(chip);
	return ret;
}

static int max17332_fg_remove(struct platform_device *pdev)
{
	struct max17332_fg_chip *chip = platform_get_drvdata(pdev);

	power_supply_unregister(chip->battery);
	kfree(chip);
	return 0;
}

static const struct platform_device_id max17332_fg_id[] = {
	{ "max17332-battery", 0, },
	{ }
};
MODULE_DEVICE_TABLE(platform, max17332_fg_id);

static struct platform_driver max17332_fg_driver = {
	.driver = {
		.name = "max17332-battery",
	},
	.probe = max17332_fg_probe,
	.remove = max17332_fg_remove,
	.id_table = max17332_fg_id,
};
module_platform_driver(max17332_fg_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("opensource@maximintegrated.com ");
MODULE_DESCRIPTION("MAX17332 Fuel Gauge");
MODULE_VERSION("1.2");
