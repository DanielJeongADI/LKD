// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTC driver for Maxim MAX31341/MAX31342 RTC
 *
 * Copyright (C) 2018 Maxim Integrated
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>

#define MAX31341_CONFIG1_REG_ECLK		0x80
#define MAX31341_CONFIG1_REG_INTCN		0x40
#define MAX31341_CONFIG1_REG_CLKSEL		0x30
#define MAX31341_CONFIG1_REG_OSCONZ		0x08
#define MAX31341_CONFIG1_REG_RS			0x06
#define MAX31341_CONFIG1_REG_SWRSTN		0x01

#define MAX31341_CONFIG2_REG_DATA_RETEN		0x40
#define MAX31341_CONFIG2_REG_BREF		0x30
#define MAX31341_CONFIG2_REG_I2C_TIMEOUT_EN	0x08
#define MAX31341_CONFIG2_REG_RD_RTC		0x04
#define MAX31341_CONFIG2_REG_SET_RTC		0x02

#define MAX31341_INT_EN_REG_LOS			0x80
#define MAX31341_INT_EN_REG_DOSF		0x40
#define MAX31341_INT_EN_REG_ANA_IE		0x20
#define MAX31341_INT_EN_REG_EIE1		0x10
#define MAX31341_INT_EN_REG_EIE0		0x08
#define MAX31341_INT_EN_REG_TIE			0x04
#define MAX31341_INT_EN_REG_A2IE		0x02
#define MAX31341_INT_EN_REG_A1IE		0x01

#define MAX31341_INT_STATUS_REG_LOS		0x80
#define MAX31341_INT_STATUS_REG_OSF		0x40
#define MAX31341_INT_STATUS_REG_ANA_IF		0x20
#define MAX31341_INT_STATUS_REG_EIF1		0x10
#define MAX31341_INT_STATUS_REG_TIF		0x04
#define MAX31341_INT_STATUS_REG_A2F		0x02
#define MAX31341_INT_STATUS_REG_A1F		0x01


#define MAX31341_HOURS_REG_F_24_12		0x40
#define MAX31341_HOURS_REG_HR20_AM_PM		0x20
#define MAX31341_HOURS_REG_HOUR_BCD		0x1F

#define MAX31341_MONTH_REG_CENTURY		0x80
#define MAX31341_MONTH_REG_MONTH_BCD		0x1F

#define MAX31341_PWR_MGMT_REG_VBACK_SEL		0x08
#define MAX31341_PWR_MGMT_REG_D_MAN_SEL		0x04
#define MAX31341_PWR_MGMT_REG_D_MODE		0x03
#define MAX31341_PWR_MGMT_REG_MASK			0x0F

#define MAX31341_TRICKLE_REG_ENABLE		0x08
#define MAX31341_TRICKLE_REG_DIODE		0x04
#define MAX31341_TRICKLE_REG_RES		0x03

#define MAX31341_TMR_CFG_REG_TFS	0x03
#define MAX31341_TMR_CFG_REG_TRPT	0x04
#define MAX31341_TMR_CFG_REG_TE		0x10
#define MAX31341_TMR_CFG_REG_TPAUSE	0x20

#define MAX31341_NVRAM_SIZE	64

#define MAX31341_BREF_BIT_POS		0x04

#define REG_NOT_AVAILABLE	0xFF

enum {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	RTC_NR_TIME
};

enum {
	RTC_ALM_SEC = 0,
	RTC_ALM_MIN,
	RTC_ALM_HOUR,
	RTC_ALM_DATE,
	RTC_ALM_MONTH,
	RTC_ALM_YEAR,
	RTC_NR_ALM_TIME
};

enum {
	D_MODE_COMPARATOR = 0,
	D_MODE_POWER_MGMT_TRICKLE_CHARGER,
};

enum {
	TRICKLE_RES_3K = 0,
	TRICKLE_RES_6K = 2,
	TRICKLE_RES_11K = 3
};

enum {
	BACKUP_THRESHOLD_1P3 = 0,
	BACKUP_THRESHOLD_1P7,
	BACKUP_THRESHOLD_2P0,
	BACKUP_THRESHOLD_2P2,
	NUM_OF_BACKUP_THRESHOLD_VALS
};

struct max31341_rtc_info {
	struct i2c_client    *client;
	struct device        *dev;
	struct rtc_device    *rtc;
	struct regmap        *map;
	struct work_struct    work;
	struct work_struct    work2;

	struct bin_attribute *nvram;

	int alarm2_cnt;
	int timer_int_cnt;
	
	int additional_interrupt;

	int                   irq;
	int                   irq2;
	const u8             *regs;
};

enum chip_id {
	ID_MAX31341,
	ID_MAX31342,
};

enum register_ids {
	CONFIG1_REG = 0,
	CONFIG2_REG,
	SLEEP_CONFIG_REG,
	TIMER_CONFIG_REG,
	INT_EN_REG,
	INT_STATUS_REG,
	SECONDS_REG,
	MINUTES_REG,
	HOURS_REG,
	DAY_REG,
	DATE_REG,
	MONTH_REG,
	YEAR_REG,
	ALM1_SEC_REG,
	ALM1_MIN_REG,
	ALM1_HRS_REG,
	ALM1DAY_DATE_REG,
	ALM1_MON_REG,
	ALM1_YEAR_REG,
	ALM2_MIN_REG,
	ALM2_HRS_REG,
	ALM2DAY_DATE_REG,
	TIMER_COUNT_REG,
	TIMER_INIT_REG,
	RAM_START_REG,
	RAM_END_REG,
	PWR_MGMT_REG,
	TRICKLE_REG,
	CLOCK_SYNC_DELAY_REG,
	REVID_REG
};

enum cntdown_timer_freqsel {
	TMR_FREQ_1024HZ,
	TMR_FREQ_256HZ,
	TMR_FREQ_64HZ,
	TMR_FREQ_16HZ,
	TOTAL_CNTDOWN_FREQS
};

static const u8 max31341_regs[] = {
	[CONFIG1_REG]      = 0x00,
	[CONFIG2_REG]      = 0x01,
	[SLEEP_CONFIG_REG] = 0x02,
	[TIMER_CONFIG_REG] = 0x03,
	[INT_EN_REG]       = 0x04,
	[INT_STATUS_REG]   = 0x05,
	[SECONDS_REG]      = 0x06,
	[MINUTES_REG]      = 0x07,
	[HOURS_REG]        = 0x08,
	[DAY_REG]          = 0x09,
	[DATE_REG]         = 0x0A,
	[MONTH_REG]        = 0x0B,
	[YEAR_REG]         = 0x0C,
	[ALM1_SEC_REG]     = 0x0D,
	[ALM1_MIN_REG]     = 0x0E,
	[ALM1_HRS_REG]     = 0x0F,
	[ALM1DAY_DATE_REG] = 0x10,
	[ALM1_MON_REG]     = REG_NOT_AVAILABLE,
	[ALM1_YEAR_REG]    = REG_NOT_AVAILABLE,
	[ALM2_MIN_REG]     = 0x11,
	[ALM2_HRS_REG]     = 0x12,
	[ALM2DAY_DATE_REG] = 0x13,
	[TIMER_COUNT_REG]  = 0x14,
	[TIMER_INIT_REG]   = 0x15,
	[RAM_START_REG]    = 0x16,
	[RAM_END_REG]      = 0x55,
	[PWR_MGMT_REG]     = 0x56,
	[TRICKLE_REG]      = 0x57,
	[CLOCK_SYNC_DELAY_REG] = 0x58,
	[REVID_REG] = 0x59,
};

static const u8 max31342_regs[] = {
	[CONFIG1_REG]      = 0x00,
	[CONFIG2_REG]      = 0x01,
	[SLEEP_CONFIG_REG] = 0x02,
	[TIMER_CONFIG_REG] = 0x03,
	[INT_EN_REG]       = 0x04,
	[INT_STATUS_REG]   = 0x05,
	[SECONDS_REG]      = 0x06,
	[MINUTES_REG]      = 0x07,
	[HOURS_REG]        = 0x08,
	[DAY_REG]          = 0x09,
	[DATE_REG]         = 0x0A,
	[MONTH_REG]        = 0x0B,
	[YEAR_REG]         = 0x0C,
	[ALM1_SEC_REG]     = 0x0D,
	[ALM1_MIN_REG]     = 0x0E,
	[ALM1_HRS_REG]     = 0x0F,
	[ALM1DAY_DATE_REG] = 0x10,
	[ALM1_MON_REG]     = 0x11,
	[ALM1_YEAR_REG]    = 0x12,
	[ALM2_MIN_REG]     = 0x13,
	[ALM2_HRS_REG]     = 0x14,
	[ALM2DAY_DATE_REG] = 0x15,
	[TIMER_COUNT_REG]  = 0x16,
	[TIMER_INIT_REG]   = 0x17,
	[RAM_START_REG]    = REG_NOT_AVAILABLE,
	[RAM_END_REG]      = REG_NOT_AVAILABLE,
	[PWR_MGMT_REG]     = REG_NOT_AVAILABLE,
	[TRICKLE_REG]      = REG_NOT_AVAILABLE,
	[CLOCK_SYNC_DELAY_REG] = 0x58,
	[REVID_REG] = REG_NOT_AVAILABLE,
};

static const u8 *chip_regs[] = {
	[ID_MAX31341] = max31341_regs,
	[ID_MAX31342] = max31342_regs,
};
#ifdef SYSFS
static int register_num;
#endif
enum pwr_mgmt {
	comparator_mode,
	pm_auto_tc_on,
	pm_manual_tc_vcc,
	pm_manual_tc_ain,
	non_used_pm_mode,
	number_of_pm_modes
};

static struct {
	int mode_val;

	const char *info_str;
} pwr_mngmnt_table[number_of_pm_modes] = {
	{0x00, "Comparator Mode"},
	{0x01, "Power Management Auto and Trickle Charger"},
	{0x05, "Power Management Manual and Trickle Charger" \
		" Active Supply = Vcc"},
	{0x0D, "Power Management Manual and Trickle Charger" \
		" Active Supply = AIN, for AIN > Vcc"},
	{0xFF, "Unrecognized Power Management Mode"}
};

static struct {
	int mode_val;

	const char *info_str;
} backup_bat_threshold[NUM_OF_BACKUP_THRESHOLD_VALS] = {
	{0x00, "1.3V"},
	{0x01, "1.7V"},
	{0x02, "2.0V"},
	{0x03, "2.2V"}
};

#ifdef SYSFS
static const char *timer_freq_table[TOTAL_CNTDOWN_FREQS] = {
	"Frequency = 1024 Hz",
	"Frequency = 256 Hz",
	"Frequency = 64 Hz",
	"Frequency = 16 Hz"
};
#endif
/* sysfs interface */

static ssize_t max31341_show_tricklecharger(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int data, pwr, ret;
	char *diode = "", *schottky = "", *resistor;

	ret = regmap_read(info->map, info->regs[PWR_MGMT_REG], &pwr);
	if (ret) {
		dev_err(dev, "Error in power mgmt reg read %d\n", ret);
		return ret;
	}

	if ((pwr & MAX31341_PWR_MGMT_REG_D_MODE) !=
			D_MODE_POWER_MGMT_TRICKLE_CHARGER)
		return sprintf(buf, "disabled\n");

	ret = regmap_read(info->map, info->regs[TRICKLE_REG], &data);
	if (ret) {
		dev_err(dev, "Error in trickle reg read %d\n", ret);
		return ret;
	}

	if ((data & MAX31341_TRICKLE_REG_ENABLE) == 0)
		return sprintf(buf, "disabled\n");

	resistor = "N/A";

	switch (data & MAX31341_TRICKLE_REG_RES) {
	case TRICKLE_RES_3K:
		resistor = "3k Ohm";
		break;

	case TRICKLE_RES_6K:
		resistor = "6k Ohm";
		break;

	case TRICKLE_RES_11K:
		resistor = "11k Ohm";
		break;
	}

	if (data & MAX31341_TRICKLE_REG_DIODE)
		diode = " in series with a diode";

	schottky = " in series with a Schottky diode";

	return sprintf(buf, "%s%s%s\n", resistor, diode, schottky);
}

static DEVICE_ATTR(trickle_charger, S_IRUGO,
					max31341_show_tricklecharger, NULL);

static ssize_t max31341_show_additional_interrupts(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	char *los = "", *osf = "", *ana_if = "", *eif1 = "";

	if (info->additional_interrupt & MAX31341_INT_STATUS_REG_LOS)
		los = "LOS Int.: LOS of signal.\n";
	
	if (info->additional_interrupt & MAX31341_INT_STATUS_REG_OSF)
		osf = "OSF Int.: Oscillator stop flag.\n";
	
	if (info->additional_interrupt & MAX31341_INT_STATUS_REG_ANA_IF)
		ana_if = "Analog Int.: Analog interrupt flag/Power fail flag.\n";
	
	if (info->additional_interrupt & MAX31341_INT_STATUS_REG_EIF1)
		eif1 = "External Int.: External interrupt flag for D1\n";

	sprintf(buf, "This section shows EIF, ANA_IF, OSF and LOS interrupts.\n");

	if (info->additional_interrupt)
		strcat(buf, "\tLast Interrupts\n");
	else
		strcat(buf, "NONE\n");

	if(los[0])
		strcat(buf, los);

	if(osf[0])
		strcat(buf, osf);

	if(ana_if[0])
		strcat(buf, ana_if);

	if(eif1[0])
		strcat(buf, eif1);
	
	info->additional_interrupt = 0;
	
	return strlen(buf);
	
}

static DEVICE_ATTR(additional_interrupt, S_IRUGO,
					max31341_show_additional_interrupts, NULL);

static ssize_t max31341_write_pwr_mgmt_mode(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int l_pwr_management_mode;
	int l_parsed_count = 0, ret = 0;

	l_parsed_count = sscanf(buf, "%d", &l_pwr_management_mode);

	if (l_parsed_count != 1) {
		dev_err(info->dev, "Fix parameters! There should be only one mode.\n");
		ret = -EINVAL;
		goto out;
	}

	if (l_pwr_management_mode >= non_used_pm_mode) {
		dev_err(info->dev, "Fix parameters! Unavailable power management mode");
		ret = -EINVAL;
		goto out;
	}

	ret = regmap_write(info->map, info->regs[PWR_MGMT_REG],
			pwr_mngmnt_table[l_pwr_management_mode].mode_val);
	if (ret) {
		dev_err(dev, "Error in %d reg write: %d\n", PWR_MGMT_REG, ret);
		goto out;
	}

out:
	return (ret < 0) ? ret : count;
}

#ifdef SYSFS
static ssize_t max31341_set_alarm2(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);

	int control;
	int ret;
	int l_parsed_count = 0;
	int hours, minutes, day_date, enabled;

	if (info->irq2 <= 0)
		return -EINVAL;

	l_parsed_count = sscanf(buf, "%d %X %X %X", &enabled, &minutes,
							&hours, &day_date);

	if (l_parsed_count == 1 && !enabled)	{
		control &= ~(MAX31341_INT_EN_REG_A2IE);
		ret = regmap_write(info->map, info->regs[INT_EN_REG], control);
		if (ret)
			return ret;

		return count;
	}

	if (l_parsed_count != 4)
		return -EINVAL;

	/* clear alarm interrupt enable bit */
	ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
	if (ret)
		return ret;

	control &= ~(MAX31341_INT_EN_REG_A2IE);
	ret = regmap_write(info->map, info->regs[INT_EN_REG], control);
	if (ret)
		return ret;

	ret = regmap_write(info->map, info->regs[ALM2_MIN_REG], minutes);
	if (ret)
		return ret;

	ret = regmap_write(info->map, info->regs[ALM2_HRS_REG], hours);
	if (ret)
		return ret;

	ret = regmap_write(info->map, info->regs[ALM2DAY_DATE_REG], day_date);
	if (ret)
		return ret;

	if (enabled) {
		control |= MAX31341_INT_EN_REG_A2IE;
		ret = regmap_write(info->map, info->regs[INT_EN_REG], control);
		if (ret)
			return ret;
	}

	return count;
}

static ssize_t max31341_read_alarm2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int register_value;
	int ret = 0;
	int hours, minutes, day_date;

	ret = regmap_read(info->map, info->regs[INT_EN_REG], &register_value);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", INT_EN_REG, ret);
		return ret;
	}

	if ((register_value & MAX31341_INT_EN_REG_A2IE) == 0)
		return sprintf(buf, "Alarm2 Disabled\n");

	ret = regmap_read(info->map, info->regs[ALM2_MIN_REG], &minutes);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", ALM2_MIN_REG, ret);
		return ret;
	}

	ret = regmap_read(info->map, info->regs[ALM2_HRS_REG], &hours);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", ALM2_HRS_REG, ret);
		return ret;
	}

	ret = regmap_read(info->map, info->regs[ALM2DAY_DATE_REG], &day_date);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", ALM2DAY_DATE_REG, ret);
		return ret;
	}

	return sprintf(buf, "Reg Min.: 0x%X Reg Hours: 0x%X"\
				" Reg Day: 0x%X Alarm Count=%d\n",
				minutes, hours, day_date, info->alarm2_cnt);
}

static DEVICE_ATTR(alarm2, S_IRUGO | S_IWUSR,
				max31341_read_alarm2, max31341_set_alarm2);

static ssize_t max31341_set_timer_freq(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);

	int reg_value, enabled;
	int ret;
	int l_parsed_count = 0;
	int timer_freq, timer_period;

	l_parsed_count = sscanf(buf, "%d %d %d", &enabled, &timer_period, &timer_freq);

	if (l_parsed_count == 1 && !enabled) {
		ret = regmap_read(info->map, info->regs[TIMER_CONFIG_REG], &reg_value);
		if (ret)
			return ret;

		reg_value &= ~MAX31341_TMR_CFG_REG_TE;
		ret = regmap_write(info->map, info->regs[TIMER_CONFIG_REG], reg_value);
		if (ret)
			return ret;

		return count;
	}

	if (l_parsed_count != 3)
		return -EINVAL;

	if (timer_freq > TMR_FREQ_16HZ)
		return -EINVAL;

	ret = regmap_read(info->map, info->regs[TIMER_CONFIG_REG], &reg_value);
	if (ret)
		return ret;

	reg_value &= ~(MAX31341_TMR_CFG_REG_TE);
	ret = regmap_write(info->map, info->regs[TIMER_CONFIG_REG], reg_value);
	if (ret)
		return ret;

	reg_value |= (MAX31341_TMR_CFG_REG_TFS | timer_freq);

	ret = regmap_write(info->map, info->regs[TIMER_CONFIG_REG], reg_value);
	if (ret)
		return ret;

	ret = regmap_write(info->map, info->regs[TIMER_INIT_REG], timer_period);
	if (ret)
		return ret;

	reg_value |= MAX31341_TMR_CFG_REG_TE;
	ret = regmap_write(info->map, info->regs[TIMER_CONFIG_REG], reg_value);
	if (ret)
		return ret;

	ret = regmap_read(info->map, info->regs[INT_EN_REG], &reg_value);
	if (ret)
		return ret;
	reg_value |= (MAX31341_INT_EN_REG_TIE);

	ret = regmap_write(info->map, info->regs[INT_EN_REG], reg_value);
	if (ret)
		return ret;

	return count;
}

static ssize_t max31341_read_timer_freq(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int register_value;
	int ret = 0;
	int freq, enabled, init_value;


	ret = regmap_read(info->map, info->regs[TIMER_CONFIG_REG], &register_value);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", ALM2_MIN_REG, ret);
		return ret;
	}

	freq = register_value & MAX31341_TMR_CFG_REG_TFS;
	enabled = register_value & MAX31341_TMR_CFG_REG_TE;

	ret = regmap_read(info->map, info->regs[TIMER_INIT_REG], &init_value);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", ALM2_HRS_REG, ret);
		return ret;
	}

	return sprintf(buf, "%s\nInit Value = %d TE:%d Int Cnt = %d\n",
					timer_freq_table[freq], init_value,
					!!enabled, info->timer_int_cnt);
}

static DEVICE_ATTR(timer, S_IRUGO | S_IWUSR, max31341_read_timer_freq, max31341_set_timer_freq);

#endif
static ssize_t max31341_read_pwr_mgmt_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int register_value, backup_threshold;
	int ret = 0, i;

	ret = regmap_read(info->map, info->regs[PWR_MGMT_REG], &register_value);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", PWR_MGMT_REG, ret);
		return ret;
	}

	ret = regmap_read(info->map, info->regs[CONFIG2_REG], &backup_threshold);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", CONFIG2_REG, ret);
		return ret;
	}

	backup_threshold &= MAX31341_CONFIG2_REG_BREF;
	backup_threshold >>= MAX31341_BREF_BIT_POS;

	register_value &= MAX31341_PWR_MGMT_REG_MASK;

	for (i = 0; i < number_of_pm_modes; i++) {

		if (pwr_mngmnt_table[i].mode_val >= register_value)
			break;
	}

	return sprintf(buf, "Reg Value = 0x%02X: %s \nBackup Battery Threshold = %s\n", register_value,
			pwr_mngmnt_table[i].info_str, backup_bat_threshold[backup_threshold].info_str);
}

static DEVICE_ATTR(power_mgmt, S_IRUGO,
			max31341_read_pwr_mgmt_mode, max31341_write_pwr_mgmt_mode);

#ifdef SYSFS

static ssize_t max31341_store_register_num(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int l_register_num;
	int l_parsed_count = 0;
	int i;
	printk(KERN_ERR"File : %20s, Line : %5d, Function : %20s - Count : %4d", __FILE__, __LINE__, __func__, count);

	l_parsed_count = sscanf(buf, "0x%02X", &l_register_num);

	if (l_parsed_count != 1) {
		dev_err(info->dev, "Fix parameters! There should be only one integer.\n");
		count = -EINVAL;
		goto error;
	}

	for (i = 0; i <= REVID_REG; i++) {
		if (l_register_num == info->regs[i]) {
			l_register_num = i;
			break;
		}
	}

	if (l_register_num == RAM_END_REG || l_register_num == RAM_START_REG) {
		count = -EINVAL;
		goto error;
	}

	if (l_register_num > REVID_REG) {
		dev_err(info->dev, "Register number is too big!\n");
		count = -EINVAL;
		goto error;
	}

	if (info->regs[l_register_num] == REG_NOT_AVAILABLE) {
		dev_err(info->dev, "Register is not valid!\n");
		count = -EINVAL;
		goto error;
	}

	register_num = l_register_num;

error:
	return count;
}

static ssize_t max31341_show_register_num(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "Reg Number: 0x%02X Reg Addr.:0x%02X\n",
					register_num, info->regs[register_num]);
}
static DEVICE_ATTR(register_num, S_IRUGO | S_IWUSR,
			max31341_show_register_num, max31341_store_register_num);

static ssize_t max31341_store_register_value(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	unsigned int register_value;
	int l_parsed_count = 0;
	int ret = 0;

	l_parsed_count = sscanf(buf, "0x%02X", &register_value);
	if (l_parsed_count != 1) {
		count = -EINVAL;
		goto error;
	}

	ret = regmap_write(info->map, info->regs[register_num], register_value);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", register_num, ret);
		count = ret;
		goto error;
	}

error:
	return count;
}

static ssize_t max31341_show_register_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int register_value;
	int ret = 0;

	ret = regmap_read(info->map, info->regs[register_num], &register_value);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", register_num, ret);
		return ret;
	}

	return sprintf(buf, "Register[%3d] : 0x%02X\n", register_num, register_value);
}

static DEVICE_ATTR(register_value, S_IRUGO | S_IWUSR, max31341_show_register_value, max31341_store_register_value);
#endif
static ssize_t max31341_nvram_write(struct file *filp, struct kobject *kobj,
			struct bin_attribute *attr,
			char *buf, loff_t off, size_t count)
{
	int ret = 0;
	unsigned char address;
	struct device *dev = kobj_to_dev(kobj);
	struct max31341_rtc_info *info = dev_get_drvdata(dev);

	if ((count + off) > MAX31341_NVRAM_SIZE)
		count = MAX31341_NVRAM_SIZE - off;

	if ((count <= 0) || (off > MAX31341_NVRAM_SIZE)) {
		ret = -EINVAL;
		goto out;
	}

	address = info->regs[RAM_START_REG] + off;

	if (count <= I2C_SMBUS_BLOCK_MAX) {
		ret = regmap_bulk_write(info->map, address, buf, count);
		if (ret) {
			dev_err(dev, "Error in nvram write %d\n", ret);
			goto out;
		}
	} else {
		ret = regmap_bulk_write(info->map, address, buf, I2C_SMBUS_BLOCK_MAX);
		if (ret) {
			dev_err(dev, "Error in nvram write %d\n", ret);
			goto out;
		}

		ret = regmap_bulk_write(info->map, (address + I2C_SMBUS_BLOCK_MAX),
					buf + I2C_SMBUS_BLOCK_MAX, count - I2C_SMBUS_BLOCK_MAX);
		if (ret) {
			dev_err(dev, "Error in nvram write %d\n", ret);
			goto out;
		}
	}

out:
	return (ret < 0) ? ret : count;
}


static ssize_t max31341_nvram_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *attr,
				char *buf, loff_t off, size_t count)
{
	int ret = 0;
	unsigned char address;
	struct device *dev = kobj_to_dev(kobj);
	struct max31341_rtc_info *info = dev_get_drvdata(dev);

	if ((count + off) > MAX31341_NVRAM_SIZE)
		count = MAX31341_NVRAM_SIZE - off;

	if ((count <= 0) || (off > MAX31341_NVRAM_SIZE)) {
		count = 0;
		goto out;
	}

	address = info->regs[RAM_START_REG] + off;

	if (count <= I2C_SMBUS_BLOCK_MAX) {
		ret = regmap_bulk_read(info->map, address, buf, count);
		if (ret) {
			dev_err(dev, "Error in nvram read %d\n", ret);
			goto out;
		}
	} else {
		ret = regmap_bulk_read(info->map, address, buf, I2C_SMBUS_BLOCK_MAX);
		if (ret) {
			dev_err(dev, "Error in nvram read %d\n", ret);
			goto out;
		}

		ret = regmap_bulk_read(info->map, (address + I2C_SMBUS_BLOCK_MAX), buf + I2C_SMBUS_BLOCK_MAX, count - I2C_SMBUS_BLOCK_MAX);
		if (ret) {
			dev_err(dev, "Error in nvram read %d\n", ret);
			goto out;
		}
	}

out:
	return (ret < 0) ? ret : count;
}

static int max31341_sysfs_register(struct max31341_rtc_info *info)
{
	int err;

	if (info->regs[RAM_START_REG] != REG_NOT_AVAILABLE) {
		info->nvram = devm_kzalloc(info->dev, sizeof(struct bin_attribute),
									GFP_KERNEL);
		if (!info->nvram) {
			dev_err(info->dev, "cannot allocate memory for nvram sysfs\n");
		} else {
			info->nvram->attr.name = "userram";
			info->nvram->attr.mode = S_IRUGO | S_IWUSR;
			info->nvram->read = max31341_nvram_read;
			info->nvram->write = max31341_nvram_write;
			info->nvram->size = info->regs[RAM_END_REG] -
					    info->regs[RAM_START_REG] + 1;

			sysfs_attr_init(&info->nvram->attr);
			err = device_create_bin_file(info->dev, info->nvram);
			if (err)
				return err;
		}
	}

	sysfs_attr_init(&dev_attr_trickle_charger.attr);
	if (info->regs[TRICKLE_REG] != REG_NOT_AVAILABLE) {
		err = device_create_file(info->dev, &dev_attr_trickle_charger);
		if (err)
			return err;
	}

	sysfs_attr_init(&dev_attr_power_mgmt.attr);
	if (info->regs[PWR_MGMT_REG] != REG_NOT_AVAILABLE) {
		err = device_create_file(info->dev, &dev_attr_power_mgmt);
		if (err)
			return err;
	}

	/*sysfs_attr_init(&dev_attr_register_num.attr);
	err = device_create_file(info->dev, &dev_attr_register_num);
	if (err)
		return err;*/

	/*sysfs_attr_init(&dev_attr_alarm2.attr);
	err = device_create_file(info->dev, &dev_attr_alarm2);
	if (err)
		return err;*/

	sysfs_attr_init(&dev_attr_additional_interrupt.attr);
	err = device_create_file(info->dev, &dev_attr_additional_interrupt);
	if (err)
		return err;

	/*sysfs_attr_init(&dev_attr_register_value.attr);
	err = device_create_file(info->dev, &dev_attr_register_value);
	if (err)
		return err;*/

	/*sysfs_attr_init(&dev_attr_timer.attr);
	err = device_create_file(info->dev, &dev_attr_timer);
	if (err)
		return err;*/

	return 0;
}

static void max31341_sysfs_unregister(struct max31341_rtc_info *info)
{
	if (info->nvram)
		device_remove_bin_file(info->dev, info->nvram);

	if (info->regs[TRICKLE_REG] != REG_NOT_AVAILABLE)
		device_remove_file(info->dev, &dev_attr_trickle_charger);

	if (info->regs[PWR_MGMT_REG] != REG_NOT_AVAILABLE)
		device_remove_file(info->dev, &dev_attr_power_mgmt);

	device_remove_file(info->dev, &dev_attr_additional_interrupt);
	/*device_remove_file(info->dev, &dev_attr_register_num);
	device_remove_file(info->dev, &dev_attr_alarm2);
	device_remove_file(info->dev, &dev_attr_register_value);
	device_remove_file(info->dev, &dev_attr_timer);*/
}


static int max31341_check_rtc_status(struct max31341_rtc_info *info)
{
	unsigned int control, stat, config, timer_config, timer_init;
	int ret;

	ret = regmap_read(info->map, info->regs[INT_STATUS_REG], &stat);
	if (ret)
		return ret;

	if (stat & MAX31341_INT_STATUS_REG_OSF)
		dev_warn(info->dev,
			 "oscillator discontinuity flagged, time unreliable\n");

	ret = regmap_read(info->map, info->regs[CONFIG1_REG], &config);
	if (ret)
		return ret;

	config |= MAX31341_CONFIG1_REG_SWRSTN; /* Remove device from reset state */
	config &= ~MAX31341_CONFIG1_REG_OSCONZ; /* Enable oscillator */

	ret = regmap_write(info->map, info->regs[CONFIG1_REG], config);
	if (ret)
		return ret;

	timer_config = TMR_FREQ_16HZ & MAX31341_TMR_CFG_REG_TFS;
	timer_config |= (MAX31341_TMR_CFG_REG_TE|MAX31341_TMR_CFG_REG_TRPT);
	timer_init = 16;

	ret = regmap_write(info->map, info->regs[TIMER_INIT_REG], timer_init);
	if (ret)
		return ret;

	ret = regmap_write(info->map, info->regs[TIMER_CONFIG_REG], timer_config);
	if (ret)
		return ret;

	/* If the alarm is pending, clear it before requesting
	 * the interrupt, so an interrupt event isn't reported
	 * before everything is initialized.
	 */
	ret = regmap_read(info->map, info->regs[INT_STATUS_REG], &control);
	if (ret)
		return ret;
	
	ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
	if (ret)
		return ret;

	control &= ~(MAX31341_INT_EN_REG_A1IE | MAX31341_INT_EN_REG_A2IE
						| MAX31341_INT_EN_REG_TIE);
	control |= (MAX31341_INT_EN_REG_EIE1 | MAX31341_INT_EN_REG_DOSF
						| MAX31341_INT_EN_REG_ANA_IE 
						| MAX31341_INT_EN_REG_LOS);

	return regmap_write(info->map, info->regs[INT_EN_REG], control);
}

/*
 *
 */
static int max31341_rtc_read_time(struct device *dev, struct rtc_time *time)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int ret;
	u8 buf[RTC_NR_TIME];
	unsigned int year, month, date, hour, minute, second;
	unsigned int weekday, twelve_hr, am_pm;
	unsigned int century, add_century = 0;

	ret = regmap_bulk_read(info->map, info->regs[SECONDS_REG], buf,
			RTC_NR_TIME);
	if (ret)
		return ret;

	second = buf[RTC_SEC];
	minute = buf[RTC_MIN];
	hour = buf[RTC_HOUR];
	weekday = buf[RTC_WEEKDAY];
	date = buf[RTC_DATE];
	month = buf[RTC_MONTH];
	year = buf[RTC_YEAR];

	/* Extract additional information for AM/PM and century */

	twelve_hr = hour & MAX31341_HOURS_REG_F_24_12;
	am_pm = hour & MAX31341_HOURS_REG_HR20_AM_PM;
	century = month & MAX31341_MONTH_REG_CENTURY;

	/* Write to rtc_time structure */

	time->tm_sec = bcd2bin(second);
	time->tm_min = bcd2bin(minute);
	if (twelve_hr) {
		/* Convert to 24 hr */
		if (am_pm)
			time->tm_hour = bcd2bin(hour &
					MAX31341_HOURS_REG_HOUR_BCD) + 12;
		else
			time->tm_hour = bcd2bin(hour &
					MAX31341_HOURS_REG_HOUR_BCD);
	} else {
		time->tm_hour = bcd2bin(hour);
	}

	/* Day of the week in linux range is 0~6 while 1~7 in RTC chip */
	time->tm_wday = bcd2bin(weekday) - 1;
	time->tm_mday = bcd2bin(date);

	/* Linux tm_mon range:0~11, while month range is 1~12 in RTC chip */
	time->tm_mon = bcd2bin(month & MAX31341_MONTH_REG_MONTH_BCD) - 1;
	if (century)
		add_century = 100;

	/* Add 100 to support up to 2099 */
	time->tm_year = bcd2bin(year) + add_century + 100;

	dev_info(dev, "RTC read time ");

	if (rtc_valid_tm(time) < 0) {
		dev_err(dev, "Invalid date/time! %04d-%02d-%02d %02d:%02d:%02d",
				time->tm_year + 1900, time->tm_mon + 1,
				time->tm_mday, time->tm_hour, time->tm_min,
				time->tm_sec);
		rtc_time_to_tm(0, time);
	}

	return 0;
}

/*
 *
 */
static int max31341_rtc_set_time(struct device *dev, struct rtc_time *time)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);

	u8 buf[RTC_NR_TIME];
	unsigned int  reg;
	int ret;

	/* Extract time from rtc_time and load into max31341 */

	buf[RTC_SEC] = bin2bcd(time->tm_sec);
	buf[RTC_MIN] = bin2bcd(time->tm_min);
	buf[RTC_HOUR] = bin2bcd(time->tm_hour);
	/* Day of the week in linux range is 0~6 while 1~7 in RTC chip */
	buf[RTC_WEEKDAY] = bin2bcd(time->tm_wday + 1);
	buf[RTC_DATE] = bin2bcd(time->tm_mday); /* Date */
	/* linux tm_mon range:0~11, while month range is 1~12 in RTC chip */
	buf[RTC_MONTH] = bin2bcd(time->tm_mon + 1);
	if (time->tm_year >= 200) {
		buf[RTC_MONTH] |= MAX31341_MONTH_REG_CENTURY;
		buf[RTC_YEAR] = bin2bcd(time->tm_year - 200);
	} else if (time->tm_year >= 100) {
		buf[RTC_YEAR] = bin2bcd(time->tm_year - 100);
	} else {
		dev_info(dev, "Invalid set date! %04d-%02d-%02d %02d:%02d:%02d",
				time->tm_year + 1900, time->tm_mon + 1,
				time->tm_mday, time->tm_hour, time->tm_min,
				time->tm_sec);
		return -EINVAL;
	}

	/* RTC Startup Process from datasheet */
	/* Step 1 */

	ret = regmap_read(info->map, info->regs[CONFIG1_REG], &reg);
	if (ret)
		return ret;

	reg |= MAX31341_CONFIG1_REG_SWRSTN; /* Remove device from reset state */
	reg &= ~MAX31341_CONFIG1_REG_OSCONZ; /* Enable oscillator */

	ret = regmap_write(info->map, info->regs[CONFIG1_REG], reg);
	if (ret)
		return ret;

	/* Step 2 */
	ret = regmap_bulk_write(info->map, info->regs[SECONDS_REG], buf,
			RTC_NR_TIME);
	if (ret < 0)
		return ret;

	/* Step 3 */
	ret = regmap_read(info->map, info->regs[CONFIG2_REG], &reg);
	if (ret < 0)
		return ret;

	/* Toggle Set_RTC bit to set RTC date */
	reg |= MAX31341_CONFIG2_REG_SET_RTC;
	regmap_write(info->map, info->regs[CONFIG2_REG], reg);
	if (ret < 0)
		return ret;

	/* Step 4 */
	/* SET_RTC bit should be kept high at least 10ms  */
	msleep(10);

	/* Step 5 */
	reg &= ~MAX31341_CONFIG2_REG_SET_RTC;
	regmap_write(info->map, info->regs[CONFIG2_REG], reg);
	if (ret < 0)
		return ret;

	dev_info(dev, "RTC set time ");
	return 0;
}

/*
 *
 */
static int max31341_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	unsigned int  control, stat;
	int ret;
	u8 buf[RTC_NR_ALM_TIME];

	ret = regmap_read(info->map, info->regs[INT_STATUS_REG], &stat);
	if (ret)
		return ret;

	ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
	if (ret)
		return ret;

	ret = regmap_bulk_read(info->map, info->regs[ALM1_SEC_REG], buf,
			RTC_NR_ALM_TIME);
	if (ret)
		return ret;

	alarm->time.tm_sec = bcd2bin(buf[RTC_ALM_SEC] & 0x7F);
	alarm->time.tm_min = bcd2bin(buf[RTC_ALM_MIN] & 0x7F);
	alarm->time.tm_hour = bcd2bin(buf[RTC_ALM_HOUR] & 0x3F);
	alarm->time.tm_mday = bcd2bin(buf[RTC_ALM_DATE] & 0x3F);

	if (info->regs[ALM1_MON_REG] != REG_NOT_AVAILABLE)
		alarm->time.tm_mon = bcd2bin(buf[RTC_ALM_MONTH] & 0x1F);

	if (info->regs[ALM1_YEAR_REG] != REG_NOT_AVAILABLE)
		alarm->time.tm_year = bcd2bin(buf[RTC_ALM_YEAR]);

	alarm->enabled = !!(control & MAX31341_INT_EN_REG_A1IE);
	alarm->pending = !!(stat & MAX31341_INT_STATUS_REG_A1F);

	return 0;
}

/*
 *
 */
static int max31341_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);

	int control;
	int ret;
	u8 buf[RTC_NR_ALM_TIME];
	int nr_alm_reg = RTC_NR_ALM_TIME;

	if (info->irq <= 0)
		return -EINVAL;

	buf[RTC_ALM_SEC] = bin2bcd(alarm->time.tm_sec);
	buf[RTC_ALM_MIN] = bin2bcd(alarm->time.tm_min);
	buf[RTC_ALM_HOUR] = bin2bcd(alarm->time.tm_hour);
	buf[RTC_ALM_DATE] = bin2bcd(alarm->time.tm_mday);

	if (info->regs[ALM1_MON_REG] != REG_NOT_AVAILABLE) {
		buf[RTC_ALM_MONTH] = bin2bcd(alarm->time.tm_mon & 0x1F);
	} else {
		nr_alm_reg--; /* chip does not have alarm month register */
	}

	if (info->regs[ALM1_YEAR_REG] != REG_NOT_AVAILABLE) {
		buf[RTC_ALM_YEAR] = bin2bcd(alarm->time.tm_year);
	} else {
		nr_alm_reg--; /* chip does not have alarm year register */
	}

	/* clear alarm interrupt enable bit */
	ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
	if (ret)
		return ret;

	control &= ~(MAX31341_INT_EN_REG_A1IE);
	control &= ~(MAX31341_INT_EN_REG_TIE);
	ret = regmap_write(info->map, info->regs[INT_EN_REG], control);
	if (ret)
		return ret;

	ret = regmap_bulk_write(info->map, info->regs[ALM1_SEC_REG], buf,
			nr_alm_reg);
	if (ret)
		return ret;

	if (alarm->enabled) {
		if (info->rtc->aie_timer.enabled)
			control |= MAX31341_INT_EN_REG_A1IE;

		if (info->rtc->uie_rtctimer.enabled)
			control |= MAX31341_INT_EN_REG_TIE;

		ret = regmap_write(info->map, info->regs[INT_EN_REG], control);
		if (ret)
			return ret;
	}

	return 0;
}


/*
 *
 */
static int max31341_rtc_alarm_irq_enable(struct device *dev,
			unsigned int enabled)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int control;
	int ret;

	ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
	if (ret)
		return ret;

	if (enabled) {
		if (info->rtc->aie_timer.enabled)
			control |= (MAX31341_INT_EN_REG_A1IE);

		if (info->rtc->uie_rtctimer.enabled)
			control |= MAX31341_INT_EN_REG_TIE;
	} else {
		if (!info->rtc->aie_timer.enabled)
			control &= ~MAX31341_INT_EN_REG_A1IE;
			
		if (!info->rtc->uie_rtctimer.enabled)
			control &= ~MAX31341_INT_EN_REG_TIE;
	}

	ret = regmap_write(info->map, info->regs[INT_EN_REG], control);

	return ret;
}

static irqreturn_t max31341_irq(int irq, void *dev_id)
{
	struct max31341_rtc_info *info = dev_id;

	disable_irq_nosync(irq);
	schedule_work(&info->work);

	return IRQ_HANDLED;
}

static void max31341_work(struct work_struct *work)
{
	struct max31341_rtc_info *info = container_of(work,
			struct max31341_rtc_info, work);
	struct mutex *lock = &info->rtc->ops_lock;
	int stat, control;
	int ret;

	mutex_lock(lock);

	ret = regmap_read(info->map, info->regs[INT_STATUS_REG], &stat);
	if (ret)
		goto unlock;

	dev_info(info->dev, "Status Register %02X\n", stat);

	if (stat & MAX31341_INT_STATUS_REG_A1F) {
		ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
		if (ret) {
			dev_warn(info->dev,
					"Read Control Register error %d\n",
					ret);
		} else {
			rtc_update_irq(info->rtc, 1, RTC_AF | RTC_IRQF);
			dev_warn(info->dev,
					"RTC Alarm Occured\n");
		}
		
	}

	if (stat & MAX31341_INT_STATUS_REG_A2F) {
		ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
		if (ret) {
			dev_warn(info->dev,
					"Read Control Register error %d\n",
					ret);
		} else {
			info->alarm2_cnt++;
		}
	}

	if (stat & MAX31341_INT_STATUS_REG_TIF) {
		ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
		if (ret) {
			dev_warn(info->dev,
					"Read Control Register error %d\n",
					ret);
		} else {
			rtc_update_irq(info->rtc, 1, RTC_UF | RTC_IRQF);
			//info->timer_int_cnt++;
		}
	}
	
	if (stat & MAX31341_INT_STATUS_REG_ANA_IF) {
		info->additional_interrupt |= MAX31341_INT_STATUS_REG_ANA_IF;
	}
	
	if (stat & MAX31341_INT_STATUS_REG_EIF1) {
		info->additional_interrupt |= MAX31341_INT_STATUS_REG_EIF1;
	}
	
	if (stat & MAX31341_INT_STATUS_REG_LOS) {
		info->additional_interrupt |= MAX31341_INT_STATUS_REG_LOS;
	}
	
	if (stat & MAX31341_INT_STATUS_REG_OSF) {
		info->additional_interrupt |= MAX31341_INT_STATUS_REG_OSF;
	}
	
	enable_irq(info->irq);

unlock:
	mutex_unlock(lock);
}


static int max31341_power_management_mode_init(struct max31341_rtc_info *info)
{
	struct device_node *node = info->dev->of_node;
	u32 power_management_mode;

	if (!node)
		goto nonconfig;

	if (of_property_read_u32(node, "power-management", &power_management_mode))
		goto nonconfig;

	if (power_management_mode >= number_of_pm_modes)
		goto nonconfig;

	return regmap_write(info->map, info->regs[PWR_MGMT_REG],
				pwr_mngmnt_table[power_management_mode].mode_val);

nonconfig:
	return regmap_write(info->map, info->regs[PWR_MGMT_REG],
				pwr_mngmnt_table[pm_auto_tc_on].mode_val);
}

static int max31341_trickle_charger_init(struct max31341_rtc_info *info)
{
	struct device_node *node = info->dev->of_node;
	int trickle = 0, pwr, ret;
	u32 ohms;

	if (!node)
		return 0;

	if (of_property_read_u32(node, "trickle-resistor-ohms", &ohms))
		return 0;

	switch (ohms) {
	case 0:
		/*Trickle charger is not connected to the battery*/
		ret = regmap_write(info->map, info->regs[TRICKLE_REG], 0);
		if (ret)
			return ret;
		break;
	case 3000:
		trickle = TRICKLE_RES_3K;
		break;
	case 6000:
		trickle = TRICKLE_RES_6K;
		break;
	case 11000:
		trickle = TRICKLE_RES_11K;
		break;
	default:
		dev_warn(info->dev,
			 "Unsupported ohm value %u in dt\n", ohms);
		return 0;
	}
	trickle |= MAX31341_TRICKLE_REG_ENABLE;

	if (of_property_read_bool(node, "trickle-diode-enable"))
		trickle |= MAX31341_TRICKLE_REG_DIODE;

	ret = regmap_write(info->map, info->regs[TRICKLE_REG], trickle);
	if (ret)
		return ret;

	/* Set power mode to trickle charger */

	ret = regmap_read(info->map, info->regs[PWR_MGMT_REG], &pwr);
	if (ret)
		return ret;

	pwr &= ~MAX31341_PWR_MGMT_REG_D_MODE;
	pwr |= D_MODE_POWER_MGMT_TRICKLE_CHARGER;

	return regmap_write(info->map, info->regs[PWR_MGMT_REG], pwr);
}

static int max31341_backup_threshold_init(struct max31341_rtc_info *info)
{
	struct device_node *node = info->dev->of_node;
	int config2_reg, ret;
	u32 backup_threshold;

	if (!node)
		return 0;

	if (of_property_read_u32(node, "backup-threshold", &backup_threshold))
		return 0;

	if (backup_threshold >= NUM_OF_BACKUP_THRESHOLD_VALS)
		return 0;

	backup_threshold <<= MAX31341_BREF_BIT_POS;

	/* Set power mode to trickle charger */

	ret = regmap_read(info->map, info->regs[CONFIG2_REG], &config2_reg);
	if (ret)
		return ret;

	config2_reg &= ~MAX31341_CONFIG2_REG_BREF;
	config2_reg |= backup_threshold;

	return regmap_write(info->map, info->regs[CONFIG2_REG], config2_reg);
}

struct reg_data_s
{
	int	reg_no,
		reg_value;
};

struct alarm2_conf_s
{  
	u8 enabled;
	int minutes,
		hours,
		day_date,
		count;
};

#define MXC_RTC_REG_READ		_IOWR('p', 0x20, int)
#define MXC_RTC_REG_WRITE		_IOW('p',  0x21, struct reg_data_s)

#define MXC_RTC_PWR_MGMT_READ		_IOR('p', 0x22, int)
#define MXC_RTC_PWR_MGMT_WRITE		_IOW('p',  0x23, int)

#define MXC_RTC_ALARM2_CONF_WRITE	_IOW('p', 0x24, struct alarm2_conf_s)
#define MXC_RTC_ALARM2_CONF_READ	_IOR('p', 0x25, struct alarm2_conf_s)

#define MXC_RTC_EXT_CLK_READ		_IOR('p',  0x26, int)
#define MXC_RTC_EXT_CLK_WRITE		_IOW('p',  0x27, int)

#define MXC_RTC_DATA_RET_READ		_IOR('p',  0x28, int)
#define MXC_RTC_DATA_RET_WRITE		_IOW('p',  0x29, int)



static int max31341_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct max31341_rtc_info *info = dev_get_drvdata(dev);
	int ret, tmp, val, i;
	void __user *argp = (void __user *)arg;
	struct reg_data_s reg;
	struct alarm2_conf_s alm2;

	dev_info(dev, "ioctl has been triggered code: 0x%02X", cmd);

	switch (cmd) {
	case MXC_RTC_REG_READ:
		if (copy_from_user(&tmp, argp, sizeof(tmp)))
			return -EFAULT;

		ret = regmap_read(info->map, tmp, &val);

		if (!ret)				
			return copy_to_user(argp, &val, sizeof(val)) ? -EFAULT : 0;
		break;
	case MXC_RTC_REG_WRITE:
		if (copy_from_user(&reg, argp, sizeof(reg)))
			return -EFAULT;

		ret = regmap_write(info->map, reg.reg_no, reg.reg_value);
		break;
	case MXC_RTC_PWR_MGMT_READ:
		ret = regmap_read(info->map, info->regs[PWR_MGMT_REG], &tmp);

		tmp &= MAX31341_PWR_MGMT_REG_MASK;

		for (i = 0; i < number_of_pm_modes; i++) {
			if (pwr_mngmnt_table[i].mode_val >= tmp)
				break;
		}

		if (!ret)				
			return copy_to_user(argp, &i, sizeof(i)) ? -EFAULT : 0;
		break;
	case MXC_RTC_PWR_MGMT_WRITE:
		if (copy_from_user(&tmp, argp, sizeof(tmp)))
			return -EFAULT;

		if (tmp >= non_used_pm_mode) {
			ret = -EINVAL;
			break;
		}
	
		ret = regmap_write(info->map, info->regs[PWR_MGMT_REG],
			pwr_mngmnt_table[tmp].mode_val);
		break;
	case MXC_RTC_ALARM2_CONF_WRITE:
		if (copy_from_user(&alm2, argp, sizeof(alm2)))
			return -EFAULT;
		
		/*Clearing the waiting interrupts*/
		ret = regmap_read(info->map, info->regs[INT_STATUS_REG], &tmp);
		if (ret)
			return ret;
		
		ret = regmap_read(info->map, info->regs[INT_EN_REG], &tmp);
		if (ret)
			return ret;

		tmp &= ~(MAX31341_INT_EN_REG_A2IE);
		ret = regmap_write(info->map, info->regs[INT_EN_REG], tmp);
		if (ret)
			return ret;
			
		if (!alm2.enabled) 
			break;

		ret = regmap_write(info->map, info->regs[ALM2_MIN_REG], alm2.minutes);
		if (ret)
			return ret;

		ret = regmap_write(info->map, info->regs[ALM2_HRS_REG], alm2.hours);
		if (ret)
			return ret;

		ret = regmap_write(info->map, info->regs[ALM2DAY_DATE_REG], alm2.day_date);
		if (ret)
			return ret;

		if (alm2.enabled) {
			tmp |= MAX31341_INT_EN_REG_A2IE;
			ret = regmap_write(info->map, info->regs[INT_EN_REG], tmp);
			if (ret)
				return ret;
		}
		break;
	case MXC_RTC_ALARM2_CONF_READ:
		ret = regmap_read(info->map, info->regs[INT_EN_REG], &tmp);
		if (ret)
			return ret;

		alm2.enabled = !!(tmp & (MAX31341_INT_EN_REG_A2IE));

		ret = regmap_read(info->map, info->regs[ALM2_MIN_REG], &alm2.minutes);
		if (ret)
			return ret;

		ret = regmap_read(info->map, info->regs[ALM2_HRS_REG], &alm2.hours);
		if (ret)
			return ret;

		ret = regmap_read(info->map, info->regs[ALM2DAY_DATE_REG], &alm2.day_date);
		if (ret)
			return ret;
		
		alm2.count = info->alarm2_cnt;

		if (!ret)
			return copy_to_user(argp, &alm2, sizeof(alm2)) ? -EFAULT : 0;
		break;
	case MXC_RTC_EXT_CLK_WRITE:
		if (copy_from_user(&tmp, argp, sizeof(tmp)))
			return -EFAULT;

		ret = regmap_read(info->map, info->regs[CONFIG1_REG], &val);
		if (ret)
			return ret;

		if (tmp)
			val |= MAX31341_CONFIG1_REG_ECLK;
		else 
			val &= ~MAX31341_CONFIG1_REG_ECLK;

		ret = regmap_write(info->map, info->regs[CONFIG1_REG], val);
		if (ret)
			return ret;
		break;
	case MXC_RTC_EXT_CLK_READ:
		ret = regmap_read(info->map, info->regs[CONFIG1_REG], &val);
		if (ret)
			return ret;
		
		val &= MAX31341_CONFIG1_REG_ECLK;
		val = !!val;

		return copy_to_user(argp, &val, sizeof(val)) ? -EFAULT : 0;
	break;
	case MXC_RTC_DATA_RET_WRITE:
		if (copy_from_user(&tmp, argp, sizeof(tmp)))
			return -EFAULT;

		ret = regmap_read(info->map, info->regs[CONFIG2_REG], &val);
		if (ret)
			return ret;

		if (tmp)
			val |= MAX31341_CONFIG2_REG_DATA_RETEN;
		else 
			val &= ~MAX31341_CONFIG2_REG_DATA_RETEN;

		ret = regmap_write(info->map, info->regs[CONFIG2_REG], val);
		if (ret)
			return ret;
		break;
	case MXC_RTC_DATA_RET_READ:
		ret = regmap_read(info->map, info->regs[CONFIG2_REG], &val);
		if (ret)
			return ret;
		
		val &= MAX31341_CONFIG2_REG_DATA_RETEN;
		val = !!val;

		return copy_to_user(argp, &val, sizeof(val)) ? -EFAULT : 0;
	break;
	default:
		return -ENOIOCTLCMD;
	}

	return ret;
}

static const struct rtc_class_ops max31341_rtc_ops = {
	.read_time = max31341_rtc_read_time,
	.set_time = max31341_rtc_set_time,
	.read_alarm = max31341_rtc_read_alarm,
	.set_alarm = max31341_rtc_set_alarm,
	.alarm_irq_enable = max31341_rtc_alarm_irq_enable,
	.ioctl = max31341_ioctl
};

static int max31341_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct max31341_rtc_info *info;
	struct device_node *np = client->dev.of_node;
	static const struct regmap_config config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int ret;
	int gpio, irq2;
	const char *extra_int_pin = "extra-interrupt-gpio";


	printk(KERN_ERR"File : %20s, Line : %5d, Function : %20s - Start", __FILE__, __LINE__, __func__);
	info = devm_kzalloc(&client->dev, sizeof(struct max31341_rtc_info),
			GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	printk(KERN_ERR"max31341_probe");
	info->dev = &client->dev;
	info->irq = client->irq;

	info->irq2 = -1;

	info->regs = chip_regs[id->driver_data];

	i2c_set_clientdata(client, info);
	INIT_WORK(&info->work, max31341_work);

	info->map = devm_regmap_init_i2c(client, &config);
	if (IS_ERR(info->map)) {
		dev_err(info->dev, "%s: regmap allocation failed: %ld\n",
				__func__, PTR_ERR(info->map));
		return PTR_ERR(info->map);
	}

	info->rtc = devm_rtc_device_register(info->dev, client->name,
			&max31341_rtc_ops, THIS_MODULE);
	if (IS_ERR(info->rtc))
		return PTR_ERR(info->rtc);

	ret = max31341_check_rtc_status(info);
	if (ret)
		return ret;

	//info->rtc->uie_unsupported = 1;

	if (client->irq > 0) {
		dev_info(info->dev, "Requesting IRQ %d\n", client->irq);
		ret = devm_request_irq(info->dev, info->irq, max31341_irq, 0,
				"max31341b", info);
		if (ret) {
			dev_err(info->dev, "unable to request IRQ\n");
			return ret;
		}

		device_set_wakeup_capable(info->dev, 1);
	}


	gpio = of_get_named_gpio(np, extra_int_pin, 0);

	if (gpio_is_valid(gpio)) {
		irq2 = gpio_to_irq(gpio);

		if (irq2 <= 0) {
			dev_warn(&client->dev, "Failed to convert gpio %d"
						" to %s\n", gpio, extra_int_pin);
		} else {
			dev_info(info->dev, "Requesting IRQ2 %d\n", irq2);
			info->irq2 = irq2;
			ret = devm_request_irq(info->dev, info->irq2, max31341_irq, 0,
					"max31341b", info);

			if (ret) {
				dev_err(info->dev, "unable to request IRQ2\n");
				return ret;
			}
		}
	}

	ret = max31341_sysfs_register(info);
	if (ret)
		dev_err(info->dev, "unable to create sysfs entries\n");

	ret = max31341_trickle_charger_init(info);
	if (ret)
		dev_err(info->dev, "unable to initialize trickle charger\n");

	ret = max31341_backup_threshold_init(info);
	if (ret)
		dev_err(info->dev, "unable to initialize backup bat. threshold\n");

	ret = max31341_power_management_mode_init(info);
	if (ret)
		dev_err(info->dev, "unable to power management mode\n");

	return 0;
}


static int max31341_remove(struct i2c_client *client)
{
	struct max31341_rtc_info *info = i2c_get_clientdata(client);

	max31341_sysfs_unregister(info);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max31341_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq > 0 && device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int max31341_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq > 0 && device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif

static const struct dev_pm_ops max31341_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max31341_suspend, max31341_resume)
};

static const struct i2c_device_id max31341_id[] = {
	{ "max31341b", .driver_data = ID_MAX31341 },
	{ "max31342", .driver_data = ID_MAX31342 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max31341_id);

static struct i2c_driver max31341b_driver = {
	.driver = {
		.name = "rtc-max31341b",
		.pm	= &max31341_pm_ops,
	},
	.probe = max31341_probe,
	.remove = max31341_remove,
	.id_table = max31341_id,
};

module_i2c_driver(max31341b_driver);


MODULE_DESCRIPTION("Maxim MAX31341B RTC Driver");
MODULE_AUTHOR("Mahir Ozturk <Mahir.Ozturk@maximintegrated.com>");
MODULE_LICENSE("GPL");
