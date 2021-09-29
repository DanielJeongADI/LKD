// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTC driver for Maxim MAX31342 RTC
 *
 * Copyright (C) 2018 Maxim Integrated
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

#define MAX31342_CONFIG1_REG_ECLK		0x80
#define MAX31342_CONFIG1_REG_INTCN		0x40
#define MAX31342_CONFIG1_REG_CLKSEL		0x30
#define MAX31342_CONFIG1_REG_OSCONZ		0x08
#define MAX31342_CONFIG1_REG_RS			0x06
#define MAX31342_CONFIG1_REG_SWRSTN		0x01

#define MAX31342_CONFIG2_REG_DATA_RETEN		0x40
#define MAX31342_CONFIG2_REG_BREF		0x30
#define MAX31342_CONFIG2_REG_I2C_TIMEOUT_EN	0x08
#define MAX31342_CONFIG2_REG_RD_RTC		0x04
#define MAX31342_CONFIG2_REG_SET_RTC		0x02

#define MAX31342_INT_EN_REG_DOSF		0x40
#define MAX31342_INT_EN_REG_ANA_IE		0x20
#define MAX31342_INT_EN_REG_EIE1		0x10
#define MAX31342_INT_EN_REG_EIE0		0x08
#define MAX31342_INT_EN_REG_TIE			0x04
#define MAX31342_INT_EN_REG_A2IE		0x02
#define MAX31342_INT_EN_REG_A1IE		0x01

#define MAX31342_INT_STATUS_REG_LOS		0x80
#define MAX31342_INT_STATUS_REG_OSF		0x40
#define MAX31342_INT_STATUS_REG_ANA_IF		0x20
#define MAX31342_INT_STATUS_REG_EIF1		0x10
#define MAX31342_INT_STATUS_REG_TIF		0x04
#define MAX31342_INT_STATUS_REG_A2F		0x02
#define MAX31342_INT_STATUS_REG_A1F		0x01


#define MAX31342_HOURS_REG_F_24_12		0x40
#define MAX31342_HOURS_REG_HR20_AM_PM		0x20
#define MAX31342_HOURS_REG_HOUR_BCD		0x1F

#define MAX31342_MONTH_REG_CENTURY		0x80
#define MAX31342_MONTH_REG_MONTH_BCD		0x1F

#define MAX31342_TMR_CFG_REG_TFS	0x03
#define MAX31342_TMR_CFG_REG_TRPT	0x04
#define MAX31342_TMR_CFG_REG_TE		0x10
#define MAX31342_TMR_CFG_REG_TPAUSE	0x20


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

struct max31342_rtc_info {
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
	ID_MAX31342
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

static const u8 max31342_regs[] = {
	[CONFIG1_REG]      = 0x00,
	[CONFIG2_REG]      = 0x01,
	[SLEEP_CONFIG_REG] = REG_NOT_AVAILABLE,
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
	[ID_MAX31342] = max31342_regs
};

/* sysfs interface */
static int max31342_check_rtc_status(struct max31342_rtc_info *info)
{
	unsigned int control, stat, config, timer_config, timer_init;
	int ret;

	ret = regmap_read(info->map, info->regs[INT_STATUS_REG], &stat);
	if (ret)
		return ret;

	if (stat & MAX31342_INT_STATUS_REG_OSF)
		dev_warn(info->dev,
			 "oscillator discontinuity flagged, time unreliable\n");

	config = 0;
	config |= MAX31342_CONFIG1_REG_INTCN;
	config |= MAX31342_CONFIG1_REG_RS;

	config |= MAX31342_CONFIG1_REG_SWRSTN; /* Remove device from reset state */
	config &= ~MAX31342_CONFIG1_REG_OSCONZ; /* Enable oscillator */

	ret = regmap_write(info->map, info->regs[CONFIG1_REG], config);
	if (ret)
		return ret;

	ret = regmap_read(info->map, info->regs[CONFIG2_REG], &config);
	if (ret)
		return ret;

	config &= ~MAX31342_CONFIG2_REG_DATA_RETEN;
	config |= MAX31342_CONFIG2_REG_I2C_TIMEOUT_EN;

	ret = regmap_write(info->map, info->regs[CONFIG2_REG], config);
	if (ret)
		return ret;

	timer_config = TMR_FREQ_16HZ & MAX31342_TMR_CFG_REG_TFS;
	timer_config |= (MAX31342_TMR_CFG_REG_TE|MAX31342_TMR_CFG_REG_TRPT);
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

	control &= ~(MAX31342_INT_EN_REG_A1IE | MAX31342_INT_EN_REG_A2IE
						| MAX31342_INT_EN_REG_TIE);

	return regmap_write(info->map, info->regs[INT_EN_REG], control);
}

/*
 *
 */
static int max31342_rtc_read_time(struct device *dev, struct rtc_time *time)
{
	struct max31342_rtc_info *info = dev_get_drvdata(dev);
	int ret;
	u8 buf[RTC_NR_TIME];
	unsigned int year, month, date, hour, minute, second;
	unsigned int weekday;
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

	century = month & MAX31342_MONTH_REG_CENTURY;

	/* Write to rtc_time structure */

	time->tm_sec = bcd2bin(second);
	time->tm_min = bcd2bin(minute);
	time->tm_hour = bcd2bin(hour);


	/* Day of the week in linux range is 0~6 while 1~7 in RTC chip */
	time->tm_wday = bcd2bin(weekday) - 1;
	time->tm_mday = bcd2bin(date);

	/* Linux tm_mon range:0~11, while month range is 1~12 in RTC chip */
	time->tm_mon = bcd2bin(month & MAX31342_MONTH_REG_MONTH_BCD) - 1;
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
static int max31342_rtc_set_time(struct device *dev, struct rtc_time *time)
{
	struct max31342_rtc_info *info = dev_get_drvdata(dev);

	u8 buf[RTC_NR_TIME];
	unsigned int  reg;
	int ret;

	/* Extract time from rtc_time and load into max31342 */

	buf[RTC_SEC] = bin2bcd(time->tm_sec);
	buf[RTC_MIN] = bin2bcd(time->tm_min);
	buf[RTC_HOUR] = bin2bcd(time->tm_hour);
	/* Day of the week in linux range is 0~6 while 1~7 in RTC chip */
	buf[RTC_WEEKDAY] = bin2bcd(time->tm_wday + 1);
	buf[RTC_DATE] = bin2bcd(time->tm_mday); /* Date */
	/* linux tm_mon range:0~11, while month range is 1~12 in RTC chip */
	buf[RTC_MONTH] = bin2bcd(time->tm_mon + 1);
	if (time->tm_year >= 200) {
		buf[RTC_MONTH] |= MAX31342_MONTH_REG_CENTURY;
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

	reg |= MAX31342_CONFIG1_REG_SWRSTN; /* Remove device from reset state */
	reg &= ~MAX31342_CONFIG1_REG_OSCONZ; /* Enable oscillator */

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
	reg |= MAX31342_CONFIG2_REG_SET_RTC;
	regmap_write(info->map, info->regs[CONFIG2_REG], reg);
	if (ret < 0)
		return ret;

	/* Step 4 */
	/* SET_RTC bit should be kept high at least 10ms  */
	msleep(20);

	/* Step 5 */
	reg &= ~MAX31342_CONFIG2_REG_SET_RTC;
	regmap_write(info->map, info->regs[CONFIG2_REG], reg);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 *
 */
static int max31342_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct max31342_rtc_info *info = dev_get_drvdata(dev);
	unsigned int  control, stat;
	int ret, century, month;
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

	if (info->regs[ALM1_YEAR_REG] != REG_NOT_AVAILABLE) {
		ret = regmap_read(info->map, info->regs[RTC_MONTH], &month);
		if (ret)
			return ret;

		century = !!(month & MAX31342_MONTH_REG_CENTURY);
		alarm->time.tm_year = bcd2bin(buf[RTC_ALM_YEAR]) + 100 + century * 100;

	}

	alarm->enabled = !!(control & MAX31342_INT_EN_REG_A1IE);
	alarm->pending = !!(stat & MAX31342_INT_STATUS_REG_A1F);

	return 0;
}

/*
 *
 */
static int max31342_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct max31342_rtc_info *info = dev_get_drvdata(dev);

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
		buf[RTC_ALM_MONTH] = bin2bcd(alarm->time.tm_mon + 1);
	else
		nr_alm_reg--; /* chip does not have alarm month register */

	if (info->regs[ALM1_YEAR_REG] != REG_NOT_AVAILABLE) {
		if (alarm->time.tm_year >= 200) {
			buf[RTC_ALM_YEAR] = bin2bcd(alarm->time.tm_year - 200);
		} else if (alarm->time.tm_year >= 100) {
			buf[RTC_ALM_YEAR] = bin2bcd(alarm->time.tm_year - 100);
		} else {
			dev_info(dev, "Invalid set alarm! %04d-%02d-%02d %02d:%02d:%02d",
					alarm->time.tm_year + 1900, alarm->time.tm_mon + 1,
					alarm->time.tm_mday, alarm->time.tm_hour, alarm->time.tm_min,
					alarm->time.tm_sec);
			return -EINVAL;
		}
	} else {
		nr_alm_reg--; /* chip does not have alarm year register */
	}

	/* clear alarm interrupt enable bit */
	ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
	if (ret)
		return ret;

	control &= ~(MAX31342_INT_EN_REG_A1IE);
	ret = regmap_write(info->map, info->regs[INT_EN_REG], control);
	if (ret)
		return ret;

	ret = regmap_bulk_write(info->map, info->regs[ALM1_SEC_REG], buf,
			nr_alm_reg);
	if (ret)
		return ret;

	if (alarm->enabled) {
		if (info->rtc->aie_timer.enabled)
			control |= MAX31342_INT_EN_REG_A1IE;

		if (info->rtc->uie_rtctimer.enabled)
			control |= MAX31342_INT_EN_REG_TIE;

		ret = regmap_write(info->map, info->regs[INT_EN_REG], control);
		if (ret)
			return ret;
	}

	return 0;
}
/*
 *
 */
static int max31342_rtc_alarm_irq_enable(struct device *dev,
			unsigned int enabled)
{
	struct max31342_rtc_info *info = dev_get_drvdata(dev);
	int control;
	int ret;

	ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
	if (ret)
		return ret;

	if (enabled) {
		if (info->rtc->aie_timer.enabled)
			control |= (MAX31342_INT_EN_REG_A1IE);

		if (info->rtc->uie_rtctimer.enabled)
			control |= MAX31342_INT_EN_REG_TIE;
	} else {
		if (!info->rtc->aie_timer.enabled)
			control &= ~MAX31342_INT_EN_REG_A1IE;

		if (!info->rtc->uie_rtctimer.enabled)
			control &= ~MAX31342_INT_EN_REG_TIE;
	}

	ret = regmap_write(info->map, info->regs[INT_EN_REG], control);

	return ret;
}

static irqreturn_t max31342_irq(int irq, void *dev_id)
{
	struct max31342_rtc_info *info = dev_id;

	disable_irq_nosync(irq);
	schedule_work(&info->work);

	return IRQ_HANDLED;
}

static void max31342_work(struct work_struct *work)
{
	struct max31342_rtc_info *info = container_of(work,
			struct max31342_rtc_info, work);
	struct mutex *lock = &info->rtc->ops_lock;
	int stat, control;
	int ret;

	mutex_lock(lock);

	ret = regmap_read(info->map, info->regs[INT_STATUS_REG], &stat);
	if (ret)
		goto unlock;

	if (stat & MAX31342_INT_STATUS_REG_A1F) {
		ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
		if (ret) {
			dev_warn(info->dev,
					"Read Control Register error %d\n",
					ret);
		} else {
			rtc_update_irq(info->rtc, 1, RTC_AF | RTC_IRQF);
		}

	}

	if (stat & MAX31342_INT_STATUS_REG_A2F) {
		ret = regmap_read(info->map, info->regs[INT_EN_REG], &control);
		if (ret) {
			dev_warn(info->dev,
					"Read Control Register error %d\n",
					ret);
		} else {
			info->alarm2_cnt++;
		}
	}

	if (stat & MAX31342_INT_STATUS_REG_TIF) {
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

static const struct rtc_class_ops max31342_rtc_ops = {
	.read_time = max31342_rtc_read_time,
	.set_time = max31342_rtc_set_time,
	.read_alarm = max31342_rtc_read_alarm,
	.set_alarm = max31342_rtc_set_alarm,
	.alarm_irq_enable = max31342_rtc_alarm_irq_enable,
};

static int max31342_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct max31342_rtc_info *info;
	static const struct regmap_config config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int ret;

	info = devm_kzalloc(&client->dev, sizeof(struct max31342_rtc_info),
			GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &client->dev;
	info->irq = client->irq;

	info->regs = chip_regs[id->driver_data];

	i2c_set_clientdata(client, info);
	INIT_WORK(&info->work, max31342_work);

	info->map = devm_regmap_init_i2c(client, &config);
	if (IS_ERR(info->map)) {
		dev_err(info->dev, "%s: regmap allocation failed: %ld\n",
				__func__, PTR_ERR(info->map));
		return PTR_ERR(info->map);
	}

	info->rtc = devm_rtc_device_register(info->dev, client->name,
			&max31342_rtc_ops, THIS_MODULE);
	if (IS_ERR(info->rtc))
		return PTR_ERR(info->rtc);

	ret = max31342_check_rtc_status(info);
	if (ret)
		return ret;

	if (client->irq > 0) {
		dev_info(info->dev, "Requesting IRQ %d\n", client->irq);
		ret = devm_request_irq(info->dev, info->irq, max31342_irq, 0,
				"max31342", info);
		if (ret) {
			dev_err(info->dev, "unable to request IRQ\n");
			return ret;
		}

		device_set_wakeup_capable(info->dev, 1);
	}

	return 0;
}


static int max31342_remove(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max31342_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq > 0 && device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int max31342_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq > 0 && device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif

static const struct dev_pm_ops max31342_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max31342_suspend, max31342_resume)
};

static const struct i2c_device_id max31342_id[] = {
	{ "max31342", .driver_data = ID_MAX31342 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max31342_id);

static struct i2c_driver max31342_driver = {
	.driver = {
		.name = "rtc-max31342",
		.pm	= &max31342_pm_ops,
	},
	.probe = max31342_probe,
	.remove = max31342_remove,
	.id_table = max31342_id,
};

module_i2c_driver(max31342_driver);


MODULE_DESCRIPTION("Maxim MAX31342 RTC Driver");
MODULE_AUTHOR("Can Ugur <Can.Ugur@maximintegrated.com>");
MODULE_LICENSE("GPL");
