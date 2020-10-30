// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTC driver for Maxim MAX31343 RTC
 *
 * Copyright (C) 2020 Maxim Integrated
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
/*
 * TODO: Sysfs Fix = There are security concerns for user space -
 *  kernel data transfer
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

#define MAX31343_STATUS_REG_PSDECT		0x80
#define MAX31343_STATUS_REG_OSF			0x40
#define MAX31343_STATUS_REG_PFAIL		0x20
#define MAX31343_STATUS_REG_TSF			0x08
#define MAX31343_STATUS_REG_TIF			0x04
#define MAX31343_STATUS_REG_A2F			0x02
#define MAX31343_STATUS_REG_A1F			0x01

#define MAX31343_INT_EN_REG_DOSF		0x40
#define MAX31343_INT_EN_REG_PFAILE		0x20
#define MAX31343_INT_EN_REG_TSIE		0x08
#define MAX31343_INT_EN_REG_TIE			0x04
#define MAX31343_INT_EN_REG_A2IE		0x02
#define MAX31343_INT_EN_REG_A1IE		0x01

#define MAX31343_RESET_REG_SWRST		0x01

#define MAX31343_CONFIG1_REG_DATA_RET	0x20
#define MAX31343_CONFIG1_REG_I2C_TOUT	0x08
#define MAX31343_CONFIG1_REG_ENOSC	0x02

#define MAX31343_CONFIG2_REG_ENCLKO		0x02
#define MAX31343_CONFIG2_REG_CLKO_HZ	0x71
#define MAX31343_CONFIG2_REG_SQW_HZ		0x07

#define MAX31343_TIM_CONFIG_REG_TE		0x10
#define MAX31343_TIM_CONFIG_REG_TPAUSE	0x08
#define MAX31343_TIM_CONFIG_REG_TRPT	0x04
#define MAX31343_TIM_CONFIG_REG_TFS		0x03

#define MAX31343_SECONDS_REG_MASK		0x7F
#define MAX31343_MINUTES_REG_MASK		0x7F

#define MAX31343_HOURS_REG_HR_BCD		0x10
#define MAX31343_HOURS_REG_HOURS		0x0F

#define MAX31343_DAYS_REG_MASK			0x07
#define MAX31343_DATE_REG_MASK			0x3F

#define MAX31343_MONTH_REG_CENTURY		0x80
#define MAX31343_MONTH_REG_BCD			0x10
#define MAX31343_MONTH_REG_MONTH		0x0F

#define MAX31343_PWR_MGMT_REG_PFVT		0x30
#define MAX31343_PWR_MGMT_REG_D_VBACK_SEL	0x08
#define MAX31343_PWR_MGMT_REG_D_MAN_SEL		0x04
	#define MAX31343_PFVT_BIT_POS			0x04
	#define MAX31343_PWR_MODE_BIT_POS		0x02

#define MAX31343_TRICKLE_REG_TCHE		0xF0
#define MAX31343_TRICKLE_REG_D_TRICKLE	0x03
#define MAX31343_TRICKLE_REG_DIODE_EN	0x04
	#define MAX31343_TRICKLE_EN				0x50

#define MAX31343_TMR_CFG_REG_TFS	0x03
#define MAX31343_TMR_CFG_REG_TRPT	0x04
#define MAX31343_TMR_CFG_REG_TE		0x10
#define MAX31343_TMR_CFG_REG_TPAUSE	0x08

#define MAX31343_YEAR_REG_YEAR			0xFF

#define MAX31343_NVRAM_SIZE				64

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
	TRICKLE_RES_3K	= 0x00,
	TRICKLE_RES_6K	= 0x02,
	TRICKLE_RES_11K	= 0x03,
	TRICKLE_RES_NC	= 0x0A
};

enum {
	BACKUP_THRESHOLD_RES,
	BACKUP_THRESHOLD_1P8,
	BACKUP_THRESHOLD_2P0,
	BACKUP_THRESHOLD_2P4,
	NUM_OF_BACKUP_THRESHOLD_VALS
};

struct max31343_rtc_info {
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
	ID_MAX31343
};

enum register_ids {
	CONFIG1_REG = 0,
	CONFIG2_REG,
	RESET_REG,
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

static const u8 max31343_regs[] = {
	[CONFIG1_REG]		= 0x03,
	[CONFIG2_REG]		= 0x04,
	[SLEEP_CONFIG_REG]	= REG_NOT_AVAILABLE,
	[TIMER_CONFIG_REG]	= 0x05,
	[INT_EN_REG]		= 0x01,
	[INT_STATUS_REG]	= 0x00,
	[SECONDS_REG]		= 0x06,
	[MINUTES_REG]		= 0x07,
	[HOURS_REG]			= 0x08,
	[DAY_REG]			= 0x09,
	[DATE_REG]			= 0x0A,
	[MONTH_REG]			= 0x0B,
	[YEAR_REG]			= 0x0C,
	[ALM1_SEC_REG]		= 0x0D,
	[ALM1_MIN_REG]		= 0x0E,
	[ALM1_HRS_REG]		= 0x0F,
	[ALM1DAY_DATE_REG]	= 0x10,
	[ALM1_MON_REG]		= 0x11,
	[ALM1_YEAR_REG]		= 0x12,
	[ALM2_MIN_REG]		= 0x13,
	[ALM2_HRS_REG]		= 0x14,
	[ALM2DAY_DATE_REG]	= 0x15,
	[TIMER_COUNT_REG]	= 0x16,
	[TIMER_INIT_REG]	= 0x17,
	[RAM_START_REG]		= 0x22,
	[RAM_END_REG]		= 0x61,
	[PWR_MGMT_REG]		= 0x18,
	[TRICKLE_REG]		= 0x19,
	[CLOCK_SYNC_DELAY_REG] = REG_NOT_AVAILABLE,
	[REVID_REG]			= REG_NOT_AVAILABLE,
};

static const u8 *chip_regs[] = {
	[ID_MAX31343] = max31343_regs
};

enum pwr_mgmt {
	PM_AUTO,
	PM_MANUAL_ACTIVE_VCC,
	PM_MANUAL_ACTIVE_VBAT,
	NUM_OF_PWR_MGMT_MODES
};

static struct {
	int mode_val;

	const char *info_str;
	} pwr_mngmnt_table[NUM_OF_PWR_MGMT_MODES] = {
	{0x00, "Power Management Auto"},
	{MAX31343_PWR_MGMT_REG_D_MAN_SEL,
			"Power Management Manual and Active Supply = Vcc"},
	{MAX31343_PWR_MGMT_REG_D_MAN_SEL | MAX31343_PWR_MGMT_REG_D_VBACK_SEL,
"Power Management Manual and Trickle Charger Active Supply = VBAT for Vbat >= Vcc"}
};

static struct {
	int mode_val;

	const char *info_str;
} backup_bat_threshold[NUM_OF_BACKUP_THRESHOLD_VALS] = {
	{0x00, "Reserved"},
	{0x01, "1.8V"},
	{0x02, "2.0V"},
	{0x03, "2.4V"}
};

/* sysfs interface */

static ssize_t trickle_charger_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max31343_rtc_info *info = dev_get_drvdata(dev);
	int data, ret;
	char *diode = "", *schottky = "", *resistor;


	ret = regmap_read(info->map, info->regs[TRICKLE_REG], &data);
	if (ret) {
		dev_err(dev, "Error in trickle reg read %d\n", ret);
		return ret;
	}

	if (((data & MAX31343_TRICKLE_REG_TCHE)) != MAX31343_TRICKLE_EN)
		return sprintf(buf, "disabled\n");

	resistor = "N/A";

	switch (data & MAX31343_TRICKLE_REG_D_TRICKLE) {
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

	if (data & MAX31343_TRICKLE_REG_DIODE_EN)
		diode = " in series with a diode";

	schottky = " in series with a Schottky diode";

	return sprintf(buf, "%s%s%s\n", resistor, diode, schottky);
}

static DEVICE_ATTR_RO(trickle_charger);


static ssize_t power_mgmt_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max31343_rtc_info *info = dev_get_drvdata(dev);
	int register_value, backup_threshold;
	int ret = 0, i;

	ret = regmap_read(info->map, info->regs[PWR_MGMT_REG], &register_value);
	if (ret) {
		dev_err(dev, "Error in %d reg read %d\n", PWR_MGMT_REG, ret);
		return ret;
	}

	backup_threshold = register_value;
	backup_threshold &= MAX31343_PWR_MGMT_REG_PFVT;
	backup_threshold >>= MAX31343_PFVT_BIT_POS;

	register_value &= (MAX31343_PWR_MGMT_REG_D_MAN_SEL
						| MAX31343_PWR_MGMT_REG_D_VBACK_SEL);
	register_value >>= MAX31343_PWR_MODE_BIT_POS;
	for (i = 0; i < NUM_OF_PWR_MGMT_MODES; i++) {
		if (pwr_mngmnt_table[i].mode_val >= register_value)
			break;
	}

	return sprintf(buf, "Reg Value = 0x%02X: %s\n"
						"Backup Battery Threshold = %s\n",
						register_value,	pwr_mngmnt_table[i].info_str,
						backup_bat_threshold[backup_threshold].info_str);
}

static DEVICE_ATTR_RO(power_mgmt);

static ssize_t max31343_nvram_write(struct file *filp, struct kobject *kobj,
			struct bin_attribute *attr,
			char *buf, loff_t off, size_t count)
{
	int ret = 0;
	unsigned char address;
	struct device *dev = kobj_to_dev(kobj);
	struct max31343_rtc_info *info = dev_get_drvdata(dev);

	if ((count + off) > MAX31343_NVRAM_SIZE)
		count = MAX31343_NVRAM_SIZE - off;

	if ((count <= 0) || (off > MAX31343_NVRAM_SIZE)) {
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


static ssize_t max31343_nvram_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *attr,
				char *buf, loff_t off, size_t count)
{
	int ret = 0;
	unsigned char address;
	struct device *dev = kobj_to_dev(kobj);
	struct max31343_rtc_info *info = dev_get_drvdata(dev);

	if ((count + off) > MAX31343_NVRAM_SIZE)
		count = MAX31343_NVRAM_SIZE - off;

	if ((count <= 0) || (off > MAX31343_NVRAM_SIZE)) {
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

		ret = regmap_bulk_read(info->map, (address + I2C_SMBUS_BLOCK_MAX), buf
						+ I2C_SMBUS_BLOCK_MAX, count - I2C_SMBUS_BLOCK_MAX);
		if (ret) {
			dev_err(dev, "Error in nvram read %d\n", ret);
			goto out;
		}
	}

out:
	return (ret < 0) ? ret : count;
}

static int max31343_sysfs_register(struct max31343_rtc_info *info)
{
	int err;

	if (info->regs[RAM_START_REG] != REG_NOT_AVAILABLE) {
		info->nvram = devm_kzalloc(info->dev, sizeof(struct bin_attribute),
									GFP_KERNEL);
		if (info->nvram) {
			info->nvram->attr.name = "userram";
			info->nvram->attr.mode = 0644/*S_IRUGO | S_IWUSR*/;
			info->nvram->read = max31343_nvram_read;
			info->nvram->write = max31343_nvram_write;
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

	return 0;
}

static void max31343_sysfs_unregister(struct max31343_rtc_info *info)
{
	if (info->nvram)
		device_remove_bin_file(info->dev, info->nvram);

	if (info->regs[TRICKLE_REG] != REG_NOT_AVAILABLE)
		device_remove_file(info->dev, &dev_attr_trickle_charger);

	if (info->regs[PWR_MGMT_REG] != REG_NOT_AVAILABLE)
		device_remove_file(info->dev, &dev_attr_power_mgmt);
}


static int max31343_check_rtc_status(struct max31343_rtc_info *info)
{
	unsigned int control, stat, config, timer_config, timer_init;
	int ret;

	ret = regmap_read(info->map, info->regs[RESET_REG], &control);
	if (ret)
		return ret;

	/*If the device is being reset, release it to work*/
	if (control != 0) {
		control = 0;
		ret = regmap_write(info->map, info->regs[RESET_REG], control);
		if (ret)
			return ret;
	}

	ret = regmap_read(info->map, info->regs[CONFIG1_REG], &config);
	if (ret)
		return ret;

	config |= MAX31343_CONFIG1_REG_ENOSC;
	config &= ~MAX31343_CONFIG1_REG_DATA_RET;
	config |= MAX31343_CONFIG1_REG_I2C_TOUT;

	dev_warn(info->dev,
			 "Config1 Register[0x%02X] = 0x%02X\n", info->regs[CONFIG1_REG],
													config);
	ret = regmap_write(info->map, info->regs[CONFIG1_REG], config);
	if (ret)
		return ret;

	/*Interrupts are being read and also being cleared*/
	ret = regmap_read(info->map, info->regs[INT_STATUS_REG], &stat);
	if (ret)
		return ret;

	if (stat & MAX31343_STATUS_REG_OSF)
		dev_warn(info->dev,
			 "oscillator discontinuity flagged, time unreliable\n");

	timer_config = TMR_FREQ_16HZ & MAX31343_TMR_CFG_REG_TFS;
	timer_config |= (MAX31343_TMR_CFG_REG_TE|MAX31343_TMR_CFG_REG_TRPT);
	timer_init = 16;

	ret = regmap_write(info->map, info->regs[TIMER_INIT_REG], timer_init);
	if (ret)
		return ret;

	dev_warn(info->dev,
			 "Timer Conf = 0x%02X\n", timer_config);
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

	control &= ~(MAX31343_INT_EN_REG_A1IE | MAX31343_INT_EN_REG_A2IE
						| MAX31343_INT_EN_REG_TIE);

	return regmap_write(info->map, info->regs[INT_EN_REG], control);
}

/*
 *
 */
static int max31343_rtc_read_time(struct device *dev, struct rtc_time *time)
{
	struct max31343_rtc_info *info = dev_get_drvdata(dev);
	int ret;
	u8 buf[RTC_NR_TIME];
	unsigned int year, month, date, hour, minute, second;
	unsigned int weekday, century, add_century = 0;

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

	/* Extract additional information century */

	century = month & MAX31343_MONTH_REG_CENTURY;

	/* Write to rtc_time structure */
	time->tm_sec = bcd2bin(second);
	time->tm_min = bcd2bin(minute);
	time->tm_hour = bcd2bin(hour);

	/* Day of the week in linux range is 0~6 while 1~7 in RTC chip */
	time->tm_wday = bcd2bin(weekday) - 1;
	time->tm_mday = bcd2bin(date);

	/* Linux tm_mon range:0~11, while month range is 1~12 in RTC chip */
	time->tm_mon = bcd2bin(month & MAX31343_MONTH_REG_MONTH) - 1;
	if (century)
		add_century = 100;

	/* Add 100 to support up to 2099 */
	time->tm_year = bcd2bin(year) + add_century + 100;

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
static int max31343_rtc_set_time(struct device *dev, struct rtc_time *time)
{
	struct max31343_rtc_info *info = dev_get_drvdata(dev);

	u8 buf[RTC_NR_TIME];
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
		buf[RTC_MONTH] |= MAX31343_MONTH_REG_CENTURY;
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

	ret = regmap_bulk_write(info->map, info->regs[SECONDS_REG], buf,
			RTC_NR_TIME);
	if (ret)
		return ret;

	return 0;
}

/*
 *
 */
static int max31343_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct max31343_rtc_info *info = dev_get_drvdata(dev);
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

	alarm->enabled = !!(control & MAX31343_INT_EN_REG_A1IE);
	alarm->pending = !!(stat & MAX31343_STATUS_REG_A1F);

	return 0;
}

/*
 *
 */
static int max31343_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct max31343_rtc_info *info = dev_get_drvdata(dev);

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

	if (info->regs[ALM1_MON_REG] != REG_NOT_AVAILABLE)
		buf[RTC_ALM_MONTH] = bin2bcd(alarm->time.tm_mon & 0x1F);
	else
		nr_alm_reg--; /* chip does not have alarm month register */

	if (info->regs[ALM1_YEAR_REG] != REG_NOT_AVAILABLE)
		buf[RTC_ALM_YEAR] = bin2bcd(alarm->time.tm_year);
	else
		nr_alm_reg--; /* chip does not have alarm year register */

	/* clear alarm interrupt enable bit */
	ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
	if (ret)
		return ret;

	control &= ~(MAX31343_INT_EN_REG_A1IE);
	ret = regmap_write(info->map, info->regs[INT_EN_REG], control);
	if (ret)
		return ret;

	ret = regmap_bulk_write(info->map, info->regs[ALM1_SEC_REG], buf,
			nr_alm_reg);
	if (ret)
		return ret;

	if (alarm->enabled) {
		if (info->rtc->aie_timer.enabled)
			control |= MAX31343_INT_EN_REG_A1IE;

		if (info->rtc->uie_rtctimer.enabled)
			control |= MAX31343_INT_EN_REG_TIE;

		ret = regmap_write(info->map, info->regs[INT_EN_REG], control);
		if (ret)
			return ret;
	}

	return 0;
}


/*
 *
 */
static int max31343_rtc_alarm_irq_enable(struct device *dev,
			unsigned int enabled)
{
	struct max31343_rtc_info *info = dev_get_drvdata(dev);
	int control;
	int ret;

	ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
	if (ret)
		return ret;

	if (enabled) {
		if (info->rtc->aie_timer.enabled)
			control |= (MAX31343_INT_EN_REG_A1IE);

		if (info->rtc->uie_rtctimer.enabled)
			control |= MAX31343_INT_EN_REG_TIE;
	} else {
		if (!info->rtc->aie_timer.enabled)
			control &= ~MAX31343_INT_EN_REG_A1IE;

		if (!info->rtc->uie_rtctimer.enabled)
			control &= ~MAX31343_INT_EN_REG_TIE;
	}

	ret = regmap_write(info->map, info->regs[INT_EN_REG], control);

	return ret;
}

static irqreturn_t max31343_irq(int irq, void *dev_id)
{
	struct max31343_rtc_info *info = dev_id;

	disable_irq_nosync(irq);
	schedule_work(&info->work);

	return IRQ_HANDLED;
}

static void max31343_work(struct work_struct *work)
{
	struct max31343_rtc_info *info = container_of(work,
			struct max31343_rtc_info, work);
	struct mutex *lock = &info->rtc->ops_lock;
	int stat, control;
	int ret;

	mutex_lock(lock);

	ret = regmap_read(info->map, info->regs[INT_STATUS_REG], &stat);
	if (ret)
		goto unlock;

	if (stat & MAX31343_STATUS_REG_A1F) {
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

	if (stat & MAX31343_STATUS_REG_A2F) {
		ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
		if (ret) {
			dev_warn(info->dev,
					"Read Control Register error %d\n",
					ret);
		} else {
			info->alarm2_cnt++;
		}
	}

	if (stat & MAX31343_STATUS_REG_TIF) {
		ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
		if (ret) {
			dev_warn(info->dev,
					"Read Control Register error %d\n",
					ret);
		} else {
			rtc_update_irq(info->rtc, 1, RTC_UF | RTC_IRQF);
		}
	}

	enable_irq(info->irq);

unlock:
	mutex_unlock(lock);
}

static int max31343_trickle_charger_init(struct max31343_rtc_info *info)
{
	struct device_node *node = info->dev->of_node;
	int trickle = 0, ret;
	u32 ohms;

	if (!node)
		return 0;

	if (of_property_read_u32(node, "trickle-resistor-ohms", &ohms)) {
		/*Trickle charger is not connected to the battery*/
		ret = regmap_write(info->map, info->regs[TRICKLE_REG], 0);
		if (ret)
			return ret;

		return 0;
	}

	switch (ohms) {
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
		/*Trickle charger is not connected to the battery*/
		ret = regmap_write(info->map, info->regs[TRICKLE_REG], 0);
		if (ret)
			return ret;
		dev_warn(info->dev,
			 "Unsupported ohm value %u in dt\n", ohms);
		return 0;
	}
	trickle |= MAX31343_TRICKLE_EN;

	if (of_property_read_bool(node, "trickle-diode-enable"))
		trickle |= MAX31343_TRICKLE_REG_DIODE_EN;

	return regmap_write(info->map, info->regs[TRICKLE_REG], trickle);
}

static int max31343_backup_threshold_init(struct max31343_rtc_info *info)
{
	struct device_node *node = info->dev->of_node;
	int pwr_mgmt, ret;
	u32 backup_threshold, backup_th_vlt;

	ret = regmap_read(info->map, info->regs[PWR_MGMT_REG], &pwr_mgmt);
	if (ret)
		return ret;

	if (!node)
		goto nonconfig;

	if (of_property_read_u32(node, "backup-threshold", &backup_threshold))
		goto nonconfig;

	if (backup_threshold >= NUM_OF_BACKUP_THRESHOLD_VALS)
		goto nonconfig;

	backup_th_vlt = backup_bat_threshold[backup_threshold].mode_val;

	pwr_mgmt &= ~MAX31343_PWR_MGMT_REG_PFVT;
	pwr_mgmt |= (backup_th_vlt << MAX31343_PFVT_BIT_POS);

	dev_warn(info->dev, "DT Sel: %d, Bat. Thresh: 0x%02X,Reg Value: 0x%02X\n\r",
															backup_threshold,
															backup_th_vlt,
															pwr_mgmt);
	return regmap_write(info->map, info->regs[PWR_MGMT_REG], pwr_mgmt);

nonconfig:
	backup_th_vlt = backup_bat_threshold[BACKUP_THRESHOLD_2P4].mode_val;

	pwr_mgmt &= ~MAX31343_PWR_MGMT_REG_PFVT;
	pwr_mgmt |= (backup_th_vlt << MAX31343_PFVT_BIT_POS);

	return regmap_write(info->map, info->regs[PWR_MGMT_REG], pwr_mgmt);
}

static int max31343_power_management_mode_init(struct max31343_rtc_info *info)
{
	struct device_node *node = info->dev->of_node;
	u32 power_management_mode;
	int ret, pwr_mgmt;

	ret = regmap_read(info->map, info->regs[PWR_MGMT_REG], &pwr_mgmt);
	if (ret)
		return ret;

	if (!node)
		goto nonconfig;

	if (of_property_read_u32(node, "power-management", &power_management_mode))
		goto nonconfig;

	if (power_management_mode >= NUM_OF_PWR_MGMT_MODES)
		goto nonconfig;

	pwr_mgmt &= ~(MAX31343_PWR_MGMT_REG_D_MAN_SEL
					| MAX31343_PWR_MGMT_REG_D_VBACK_SEL);
	pwr_mgmt |= pwr_mngmnt_table[power_management_mode].mode_val;

	dev_warn(info->dev, "DT Sel: %d Reg Value: 0x%02X\n\r",
									power_management_mode, pwr_mgmt);
	return regmap_write(info->map, info->regs[PWR_MGMT_REG], pwr_mgmt);

nonconfig:
	pwr_mgmt &= ~(MAX31343_PWR_MGMT_REG_D_MAN_SEL
					| MAX31343_PWR_MGMT_REG_D_VBACK_SEL);
	pwr_mgmt |= pwr_mngmnt_table[PM_AUTO].mode_val;

	return regmap_write(info->map, info->regs[PWR_MGMT_REG], pwr_mgmt);
}


static const struct rtc_class_ops max31343_rtc_ops = {
	.read_time = max31343_rtc_read_time,
	.set_time = max31343_rtc_set_time,
	.read_alarm = max31343_rtc_read_alarm,
	.set_alarm = max31343_rtc_set_alarm,
	.alarm_irq_enable = max31343_rtc_alarm_irq_enable,
};

static int max31343_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct max31343_rtc_info *info;
	static const struct regmap_config config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int ret;


	info = devm_kzalloc(&client->dev, sizeof(struct max31343_rtc_info),
			GFP_KERNEL);
	if (!info)
		return -ENOMEM;


	info->dev = &client->dev;
	info->irq = client->irq;

	info->regs = chip_regs[id->driver_data];

	i2c_set_clientdata(client, info);
	INIT_WORK(&info->work, max31343_work);

	info->map = devm_regmap_init_i2c(client, &config);
	if (IS_ERR(info->map)) {
		dev_err(info->dev, "%s: regmap allocation failed: %ld\n",
				__func__, PTR_ERR(info->map));
		return PTR_ERR(info->map);
	}

	info->rtc = devm_rtc_device_register(info->dev, client->name,
			&max31343_rtc_ops, THIS_MODULE);
	if (IS_ERR(info->rtc))
		return PTR_ERR(info->rtc);

	ret = max31343_check_rtc_status(info);
	if (ret)
		return ret;

	if (client->irq > 0) {
		dev_info(info->dev, "Requesting IRQ %d\n", client->irq);
		ret = devm_request_irq(info->dev, info->irq, max31343_irq, 0,
				"max31343", info);
		if (ret) {
			dev_err(info->dev, "unable to request IRQ\n");
			return ret;
		}

		device_set_wakeup_capable(info->dev, 1);
	}


	ret = max31343_sysfs_register(info);
	if (ret)
		dev_err(info->dev, "unable to create sysfs entries\n");

	if (info->regs[TRICKLE_REG] != REG_NOT_AVAILABLE) {
		ret = max31343_trickle_charger_init(info);
		if (ret)
			dev_err(info->dev, "unable to initialize trickle charger\n");
	}

	if (info->regs[PWR_MGMT_REG] != REG_NOT_AVAILABLE) {
		ret = max31343_backup_threshold_init(info);
		if (ret)
			dev_err(info->dev, "unable to initialize backup bat. threshold\n");

		ret = max31343_power_management_mode_init(info);
		if (ret)
			dev_err(info->dev, "unable to initialize trickle charger\n");
	}

	return 0;
}


static int max31343_remove(struct i2c_client *client)
{
	struct max31343_rtc_info *info = i2c_get_clientdata(client);

	max31343_sysfs_unregister(info);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max31343_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq > 0 && device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int max31343_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq > 0 && device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif

static const struct dev_pm_ops max31343_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max31343_suspend, max31343_resume)
};

static const struct i2c_device_id max31343_id[] = {
	{ "max31343", .driver_data = ID_MAX31343 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max31343_id);

static struct i2c_driver max31343_driver = {
	.driver = {
		.name = "rtc-max31343",
		.pm	= &max31343_pm_ops,
	},
	.probe = max31343_probe,
	.remove = max31343_remove,
	.id_table = max31343_id,
};

module_i2c_driver(max31343_driver);


MODULE_DESCRIPTION("Maxim MAX31343 RTC Driver");
MODULE_AUTHOR("Can Ugur <Can.Ugur@maximintegrated.com>");
MODULE_LICENSE("GPL");
