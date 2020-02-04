// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxim MAX77932 Dual Phase Switched Capacitor Converter
 *
 * Copyright (C) 2018 Maxim Integrated. All rights reserved.
 *
 * Author:
 *	Daniel Jeong <daniel.jeong@maximintegrated.com>
 *	Maxim LDD <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_REGULATOR_MAX77932_H
#define	__LINUX_REGULATOR_MAX77932_H

//#define __TEST_DEVICE_NODE__

#include <linux/regulator/machine.h>

/* MAX77932 Registers */
#define MAX77932_REG_INT_SRC 0x00
#define MAX77932_REG_INT_SRC_M 0x01
#define MAX77932_REG_STATUS 0x02
#define MAX77932_REG_SCC_EN 0x03
#define MAX77932_REG_SCC_CFG1 0x04
#define MAX77932_REG_SCC_CFG2 0x05
#define MAX77932_REG_OVP_UVLO 0x06
#define MAX77932_REG_OCP1 0x07
#define MAX77932_REG_OCP2 0x08
#define MAX77932_REG_OOVP 0x09
#define MAX77932_REG_SS_CFG 0x0A
#define MAX77932_REG_EN_CFG1 0x0B
#define MAX77932_REG_EN_CFG2 0x0C
#define MAX77932_REG_I2C_CFG 0x14
#define MAX77932_REG_CHIP_REV 0x15
#define MAX77932_REG_DEVICE_ID 0x16

enum max77932_reg {
	MAX77932_SCC,
	MAX77932_NUM_REGULATORS,
};

enum {
	MAX77932_IRQ_SS_FLT,
	MAX77932_IRQ_T_SHDN,
	MAX77932_IRQ_T_ALM2,
	MAX77932_IRQ_T_ALM1,
	MAX77932_IRQ_OCP,
	MAX77932_IRQ_OC_ALM,
	MAX77932_IRQ_OOVP,
	MAX77932_IRQ_IOVP,
};

struct max77932_chip {
	struct regmap *regmap;
	struct device *dev;
	struct regulator_desc regulator_desc;
	struct regulator_dev *regulator;

	int irq;
	struct regmap_irq_chip_data *irq_data;
};

#endif /* __LINUX_REGULATOR_MAX77932_H */
