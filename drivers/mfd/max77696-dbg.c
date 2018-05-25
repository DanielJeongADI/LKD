/*
 * MAX77696 Driver Debug
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

//#define DEBUG
//#define VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <linux/mfd/core.h>
#include <linux/mfd/max77696.h>
#include "max77696-core.h"

#define FILE_DESC    MAX77696_DESC" Driver Debug"
#define FILE_NAME    MAX77696_DBG_NAME
#define FILE_VERSION MAX77696_CORE_VERSION
#define FILE_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

static DEFINE_MUTEX(max77696_dbg_lock);

#define DEFINE_DBG_REG_DEV_ATTR(_module) \
static int max77696_dbg_##_module##_reg_read (struct max77696_core *core,\
    u16 addr, u16 *data)\
{\
    struct device *dev = core->dev;\
    u16 buf = 0;\
    int rc;\
    rc = max77696_read(__##_module##_io(core), addr, &buf);\
    if (unlikely(rc)) {\
        dev_err(dev,\
            "failed to read "#_module" register %02X [%d]\n", addr, rc);\
    }\
    *data = (u16)buf;\
    return rc;\
}\
static int max77696_dbg_##_module##_reg_write (struct max77696_core *core,\
    u16 addr, u16 data)\
{\
    struct device *dev = core->dev;\
    int rc;\
    rc = max77696_write(__##_module##_io(core), addr, data);\
    if (unlikely(rc)) {\
        dev_err(dev,\
            "failed to write "#_module" register %02X [%d]\n", addr, rc);\
    }\
    return rc;\
}\
static u16 max77696_dbg_##_module##_reg_addr;\
static ssize_t max77696_dbg_##_module##_reg_addr_show (struct device *dev,\
    struct device_attribute *devattr, char *buf)\
{\
    int rc;\
    mutex_lock(&max77696_dbg_lock);\
    rc  = (int)snprintf(buf, PAGE_SIZE, ""#_module" reg addr 0x%04X\n",\
        max77696_dbg_##_module##_reg_addr);\
    mutex_unlock(&max77696_dbg_lock);\
    return (ssize_t)rc;\
}\
static ssize_t max77696_dbg_##_module##_reg_addr_store (struct device *dev,\
    struct device_attribute *devattr, const char *buf, size_t count)\
{\
    mutex_lock(&max77696_dbg_lock);\
    max77696_dbg_##_module##_reg_addr = (u16)simple_strtoul(buf, NULL, 16);\
    mutex_unlock(&max77696_dbg_lock);\
    return (ssize_t)count;\
}\
static ssize_t max77696_dbg_##_module##_reg_data_show (struct device *dev,\
    struct device_attribute *devattr, char *buf)\
{\
    struct max77696_core *core = dev_get_drvdata(dev);\
    u16 data;\
    int rc, ofst;\
    mutex_lock(&max77696_dbg_lock);\
    rc = max77696_dbg_##_module##_reg_read(core,\
        max77696_dbg_##_module##_reg_addr, &data);\
    ofst = (int)snprintf(buf, PAGE_SIZE, \
        "read  "#_module" reg addr 0x%04X ",\
        max77696_dbg_##_module##_reg_addr);\
    ofst += (int)snprintf(buf + ofst, PAGE_SIZE, "data 0x%04X ", data);\
    ofst += (int)snprintf(buf + ofst, PAGE_SIZE, "rc %d\n", rc);\
    mutex_unlock(&max77696_dbg_lock);\
    return (ssize_t)ofst;\
}\
static ssize_t max77696_dbg_##_module##_reg_data_store (struct device *dev,\
    struct device_attribute *devattr, const char *buf, size_t count)\
{\
    struct max77696_core *core = dev_get_drvdata(dev);\
    u16 data;\
    int rc;\
    mutex_lock(&max77696_dbg_lock);\
    data = (u16)simple_strtoul(buf, NULL, 16);\
    rc = max77696_dbg_##_module##_reg_write(core,\
        max77696_dbg_##_module##_reg_addr, data);\
/*  printk(KERN_INFO"write "#_module" reg addr 0x%04X data 0x%04X rc %d\n",*/\
/*      max77696_dbg_##_module##_reg_addr, data, rc);*/\
    mutex_unlock(&max77696_dbg_lock);\
    return (ssize_t)count;\
}\
static DEVICE_ATTR(_module##_reg_addr, S_IWUSR|S_IRUGO,\
    max77696_dbg_##_module##_reg_addr_show,\
    max77696_dbg_##_module##_reg_addr_store);\
static DEVICE_ATTR(_module##_reg_data, S_IWUSR|S_IRUGO,\
    max77696_dbg_##_module##_reg_data_show,\
    max77696_dbg_##_module##_reg_data_store)

#define DBG_REG_DEV_ATTR(_module) \
        (&dev_attr_##_module##_reg_addr.attr),\
        (&dev_attr_##_module##_reg_data.attr)

#define __module_io(_core, _module) \
        (&((_core)->chip_io[MAX77696_IO_##_module]))

#define __pmic_io(_core)  __module_io(_core, PMIC)
#define __rtc_io(_core)   __module_io(_core, RTC )
#define __uic_io(_core)   __module_io(_core, UIC )
#define __fg_io(_core)    __module_io(_core, FG  )

DEFINE_DBG_REG_DEV_ATTR(pmic);
DEFINE_DBG_REG_DEV_ATTR(rtc);
DEFINE_DBG_REG_DEV_ATTR(uic);
DEFINE_DBG_REG_DEV_ATTR(fg);

static const struct attribute *max77696_dbg_attr[] = {
    DBG_REG_DEV_ATTR(pmic),
    DBG_REG_DEV_ATTR(rtc),
    DBG_REG_DEV_ATTR(uic),
    DBG_REG_DEV_ATTR(fg),
};

int max77696_dbg_set (struct max77696_core *core, bool enable)
{
    int i, rc = 0;

    mutex_lock(&max77696_dbg_lock);

    if (unlikely(core->debug == enable)) {
        goto out;
    }

    if (enable) {
        goto add_files;
    }
    goto remove_files;

add_files:
    for (i = 0; i < ARRAY_SIZE(max77696_dbg_attr); i++) {
        rc = sysfs_add_file_to_group(core->kobj, max77696_dbg_attr[i],
            MAX77696_CORE_ATTR_GROUP_NAME);
        if (unlikely(rc)) {
            dev_err(core->dev, "failed to add attr %s [%d]\n",
                max77696_dbg_attr[i]->name, rc);
            goto out;
        }
    }
    core->debug = true;
    goto out;

remove_files:
    for (i = 0; i < ARRAY_SIZE(max77696_dbg_attr); i++) {
        sysfs_remove_file_from_group(core->kobj, max77696_dbg_attr[i],
            MAX77696_CORE_ATTR_GROUP_NAME);
    }
    core->debug = false;

out:
    mutex_unlock(&max77696_dbg_lock);
    return rc;
}

/*******************************************************************************
 * MAX77696-Internal Services
 ******************************************************************************/

__devinit int max77696_dbg_init (struct max77696_core *core)
{
    core->debug = false;
    return 0;
}

__devexit void max77696_dbg_exit (struct max77696_core *core)
{
    max77696_dbg_set(core, false);
}

int max77696_dbg_suspend (struct max77696_core *core)
{
    return 0;
}

int max77696_dbg_resume (struct max77696_core *core)
{
    return 0;
}
