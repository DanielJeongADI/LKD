/*
 * Copyright (C) 2020 Maximintegrated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17332_BATTERY_H_
#define __MAX17332_BATTERY_H_

#define REG_VALRTTH         0x01
#define REG_TALRTTH         0x02
#define REG_SALRTTH         0x03
#define REG_REPSOC          0x06
#define REG_MAXMINVOLT      0x08
#define REG_CONFIG			0x0B
#define REG_TTE             0x11
#define REG_AVGVCELL		0x19
#define REG_VCELL			0x1A
#define REG_TEMP            0x1B
#define REG_CURRENT         0x1C
#define REG_AVGCURRENT      0x1D
#define REG_TTF             0x20
#define REG_VERSION         0x21
#define REG_VEMPTY          0x3A
#define REG_CHGSTAT			0xA3
#define REG_IALRTTH         0xAC
#define REG_PROTSTATUS      0xD9
#define REG_VFOCV           0xFB

/* Config register bits for MAX17332 */
#define BIT_CONFIG_ALRT_EN		BIT(2)

#define MAX17332_BATTERY_FULL	100
#define MAX17332_BATTERY_LOW	15

struct max17332_fg_platform_data {
	int volt_min; /* in mV */
	int volt_max; /* in mV */
	int temp_min; /* in DegreC */
	int temp_max; /* in DegreeC */
	int soc_max;  /* in percent */
	int soc_min;  /* in percent */
	int curr_max; /* in mA */
	int curr_min; /* in mA */
};


enum {
	MAX17332_FG_PROGRAM_NVM = 0,
	MAX17332_FG_REMAINING_NVM_UPDATES,
};

ssize_t max17332_fg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t max17332_fg_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

#define MAX17332_FG_ATTR(_name)				\
{							\
	.attr = {.name = #_name, .mode = 0660},	\
	.show = max17332_fg_show_attrs,			\
	.store = max17332_fg_store_attrs,			\
}

struct max17332_fg_chip {
	struct device           *dev;
	struct max17332_dev     *max17332;
	struct regmap			*regmap;
	struct regmap			*regmap_nvm;
	struct attribute_group *attr_grp;

	int						fg_prot_irq;
	struct power_supply		*battery;
	struct power_supply_desc		psy_batt_d;

	/* mutex */
	struct mutex			lock;

	/* rsense */
	unsigned int rsense;

	struct max17332_fg_platform_data	*pdata;
};
#endif // __MAX17332_BATTERY_H_
