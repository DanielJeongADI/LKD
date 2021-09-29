/*
 * Copyright (C) 2020 Maximintegrated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17332_CHARGER_H_
#define __MAX17332_CHARGER_H_

/* Model Gauge Registers */
#define REG_REPCAP             0x05
#define REG_FULLCAPREP         0x10
#define REG_CYCLES             0x17
#define REG_DESIGNCAP          0x18
#define REG_QH                 0x4D

/* NVM Registers */
#define REG_NDESIGNCAP_NVM	 	0xB3
#define REG_NICHGCFG1_NVM	 	0xCE
#define REG_NVCHGCFG1_NVM	 	0xCC


/* nIChgCfg1 register bits for MAX17332 */
#define MAX17332_NICHGCFG1_ROOMCHARGINGI_POS 5
#define MAX17332_NICHGCFG1_ROOMCHARGINGI (0x3F << MAX17332_NICHGCFG1_ROOMCHARGINGI_POS)

/* nVChgCfg1 register bits for MAX17332 */
#define MAX17332_NVCHGCFG1_ROOMCHARGINGV_POS 4
#define MAX17332_NVCHGCFG1_ROOMCHARGINGV (0xFF << MAX17332_NVCHGCFG1_ROOMCHARGINGV_POS)

/* nDesignCap register bits for MAX17332 */
#define MAX17332_DESIGNCAP_QSCALE_POS 0
#define MAX17332_DESIGNCAP_QSCALE (0x7 << MAX17332_DESIGNCAP_QSCALE_POS)
#define MAX17332_DESIGNCAP_VSCALE_POS 3
#define MAX17332_DESIGNCAP_VSCALE (0x1 << MAX17332_DESIGNCAP_VSCALE_POS)
#define MAX17332_DESIGNCAP_DESIGNCAP_POS 6
#define MAX17332_DESIGNCAP_DESIGNCAP (0x3FF << MAX17332_DESIGNCAP_DESIGNCAP_POS)


enum {
	MAX17332_CHG_CURRENT = 0,
	MAX17332_CHG_EN,
	MAX17332_CHG_VOLTAGE,
};

ssize_t max17332_chg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t max17332_chg_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

#define MAX17332_CHG_ATTR(_name)				\
{							\
	.attr = {.name = #_name, .mode = 0660},	\
	.show = max17332_chg_show_attrs,			\
	.store = max17332_chg_store_attrs,			\
}

struct max17332_charger_data {
	struct device           *dev;
	struct max17332_dev     *max17332;
	struct regmap			*regmap;
	struct regmap			*regmap_nvm;

	struct power_supply		*psy_chg;
	struct power_supply_desc		psy_chg_d;

	/* mutex */
	struct mutex			lock;

	int cycles_reg_lsb_percent;
	/* rsense */
	unsigned int rsense;
};

#endif // __MAX17332_CHARGER_H_
