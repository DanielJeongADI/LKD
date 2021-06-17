/*
 * Copyright (c) 2021 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MAX77860_REGULATOR_H
#define __LINUX_MAX77860_REGULATOR_H

struct max77860_regulator_platform_data {
	struct regulator_init_data *initdata;
	struct device_node *of_node;
};

enum max77860_regulators {
	MAX77860_SAFEOUT1 = 0,
	MAX77860_REG_MAX,
};

#endif
