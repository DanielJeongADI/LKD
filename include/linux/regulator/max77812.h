/*
 * MAX77812 Buck Regulators Driver Header File
 *
 * Copyright (C) 2017 Maxim Integrated
 *
 * This file is part of MAX77812 Linux Driver
 *
 * MAX77812 Linux Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * MAX77812 Linux Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MAX77812 Linux Driver. If not, see http://www.gnu.org/licenses/.
 */

#ifndef __MAX77812_H__
#define __MAX77812_H__

enum max77812_interrupt {
    MAX77812_INT_WDTRSTB   = 0,
    MAX77812_INT_UVLO      = 1,
    MAX77812_INT_TSHDN     = 2,
    MAX77812_INT_TJCT_140C = 3,
    MAX77812_INT_TJCT_120C = 4,
    MAX77812_INT_M4_POKn   = 5,
    MAX77812_INT_M3_POKn   = 6,
    MAX77812_INT_M2_POKn   = 7,
    MAX77812_INT_M1_POKn   = 8,
    /***/
    MAX77812_NUM_OF_INTS
};

int max77812_get_interrupt_stat (u8 int_nr);

#endif /* __MAX77812_H__ */