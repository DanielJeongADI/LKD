/*
 * MAX77696 Driver Core
 *
 * Copyright (C) 2015 Maxim Integrated
 *
 * This file is part of MAX77696 PMIC Linux Driver
 *
 * MAX77696 PMIC Linux Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * MAX77696 PMIC Linux Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MAX77696 PMIC Linux Driver. If not, see http://www.gnu.org/licenses/.
 */

#ifndef __MAX77696_CORE_H__
#define __MAX77696_CORE_H__

#include <linux/list.h>

#define MAX77696_CORE_VERSION MAX77696_DRIVER_VERSION".0"

enum {
    MAX77696_IO_PMIC = 0,
    MAX77696_IO_RTC,
    MAX77696_IO_UIC,
    MAX77696_IO_FG,
    /***/
    MAX77696_NUM_OF_IOS,
};

struct max77696_core {
    struct mutex                    lock;
    struct max77696_platform_data  *pdata;
    struct max77696_io             *io;
    struct device                  *dev;
    struct kobject                 *kobj;
    const struct attribute_group   *attr_grp;

    /* Boolean flag of debug */
    bool                            debug;

    /* Core device driver data */
    void                           *topint_drv;
    void                           *topsys_drv;

    /* Chip IO */
    struct max77696_io              chip_io[MAX77696_NUM_OF_IOS];
};

#define max77696_core_set_drvdata(_core, _drv, _data) \
        ( (_core) -> _drv = (_data))
#define max77696_core_get_drvdata(_core, _drv) \
        ( (_core) ? (_core) -> _drv : NULL)

#define max77696_core_get_io(_core, _name) \
        (&((_core) ->io[MAX77696_IO_##_name]))
#define max77696_core_set_io(_core, _name, ...) \
        __max77696_core_set_io(_core, MAX77696_IO_##_name, __VA_ARGS__)

static __always_inline
void __max77696_core_set_io (struct max77696_core *core, int index,
    struct device *iodev, void *read, void *write,
    void *bulk_read, void *bulk_write)
{
    core->chip_io[index].dev        = iodev;
    core->chip_io[index].read       = read;
    core->chip_io[index].write      = write;
    core->chip_io[index].bulk_read  = bulk_read;
    core->chip_io[index].bulk_write = bulk_write;
}

/* CORE */
extern int max77696_init (struct max77696_core *core);
extern void max77696_exit (struct max77696_core *core);
extern int max77696_suspend_core (struct max77696_core *core);
extern int max77696_resume_core (struct max77696_core *core);

/* TOPINT */
extern int max77696_topint_init (struct max77696_core *core);
extern void max77696_topint_exit (struct max77696_core *core);
extern int max77696_topint_suspend (struct max77696_core *core);
extern int max77696_topint_resume (struct max77696_core *core);

/* TOPSYS */
extern int max77696_topsys_init (struct max77696_core *core);
extern void max77696_topsys_exit (struct max77696_core *core);
extern int max77696_topsys_suspend (struct max77696_core *core);
extern int max77696_topsys_resume (struct max77696_core *core);

/* DBG */
extern int max77696_dbg_init (struct max77696_core *core);
extern void max77696_dbg_exit (struct max77696_core *core);
extern int max77696_dbg_suspend (struct max77696_core *core);
extern int max77696_dbg_resume (struct max77696_core *core);
extern int max77696_dbg_set (struct max77696_core *core, bool enable);

#define max77696_get_block_devirq(_coredev, _block) \
        __max77696_get_block_devirq(_coredev, MAX77696_BLK_##_block)
extern int __max77696_get_block_devirq (struct device *coredev,
    unsigned int block_num);

#define max77696_set_block_devirq(_coredev, _block, _devirq) \
        __max77696_set_block_devirq(_coredev,\
            MAX77696_BLK_##_block, _devirq)
extern int __max77696_set_block_devirq (struct device *coredev,
    unsigned int block_num, int devirq);

#endif /* !__MAX77696_CORE_H__ */
