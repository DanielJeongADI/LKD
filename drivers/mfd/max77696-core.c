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

#include <linux/of.h>
#include <linux/of_platform.h>

#include <linux/mfd/core.h>
#include <linux/mfd/max77696.h>
#include "max77696-core.h"

#define FILE_DESC    MAX77696_DESC" Driver Core"
#define FILE_NAME    MAX77696_CORE_NAME
#define FILE_VERSION MAX77696_CORE_VERSION
#define FILE_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define NSUBDEV      MAX77696_NUM_OF_BLOCKS

#define CID0         0xA1
#define CID1         0xA0
#define CID2         0x9F
#define CID3         0x9E
#define CID4         0x9D
#define CID5         0x9C

#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
#define __printf_id(_core) \
        do {\
            u16 _cid5;\
            max77696_read((_core) ->io, CID5, &_cid5);\
            dev_info((_core) ->dev, "DID %02X\n", _cid5);\
        } while(0)
#elif defined(CONFIG_MAX77796)
#define __printf_id(_core) \
        do {\
            u16 _cid0, _cid1, _cid2, _cid3, _cid4, _cid5;\
            max77696_read((_core) ->io, CID0, &_cid0);\
            max77696_read((_core) ->io, CID1, &_cid1);\
            max77696_read((_core) ->io, CID2, &_cid2);\
            max77696_read((_core) ->io, CID3, &_cid3);\
            max77696_read((_core) ->io, CID4, &_cid4);\
            max77696_read((_core) ->io, CID5, &_cid5);\
            dev_info((_core) ->dev, "SN  %02X%02X%02X\n", _cid2, _cid1, _cid0);\
            dev_info((_core) ->dev, "SBT %02X\n", _cid3 & 0x7);\
            dev_info((_core) ->dev, "DRV %02X\n", _cid4);\
            dev_info((_core) ->dev, "DID %02X\n", _cid5);\
        } while(0)
#endif

#ifndef CONFIG_MAX77696_DT
static int max77696_add_subdevices (struct max77696_core *core,
    char *dev_name, void *dev_pdata, size_t dev_pdata_sz)
{
    struct device *dev = core->dev;
    struct mfd_cell subdev;
    struct resource no_res;
    int rc;

    memset(&no_res, 0x00, sizeof(no_res));
    memset(&subdev, 0x00, sizeof(struct mfd_cell));

    subdev.name           = dev_name;
    subdev.id             = 0;
    subdev.platform_data  = dev_pdata;
    subdev.pdata_size     = dev_pdata_sz;
    subdev.resources      = &no_res;
    subdev.num_resources  = 1;

    subdev.ignore_resource_conflicts = 1;

    #ifndef PLATFORM_DEVID_NONE
    #define PLATFORM_DEVID_NONE (-1)
    #endif /* !PLATFORM_DEVID_NONE */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)
    rc = mfd_add_devices(dev, PLATFORM_DEVID_NONE, &subdev, 1,
        NULL, 0);
#else /* LINUX_VERSION_CODE ... */
    rc = mfd_add_devices(dev, PLATFORM_DEVID_NONE, &subdev, 1,
        NULL, 0, NULL);
#endif /* LINUX_VERSION_CODE ... */
    if (unlikely(rc)) {
        dev_err(dev, "failed to add mfd_subdev %s [%d]\n", dev_name, rc);
        return rc;
    }
    return 0;
}
#endif /* !CONFIG_MAX77696_DT */

static ssize_t max77696_debug_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_core *core = dev_get_drvdata(dev);
    int rc;

    mutex_lock(&core->lock);

    rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", core->debug);

    mutex_unlock(&core->lock);
    return (ssize_t)rc;
}

static ssize_t max77696_debug_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct max77696_core *core = dev_get_drvdata(dev);

    mutex_lock(&core->lock);

    max77696_dbg_set(core, !!simple_strtoul(buf, NULL, 10));

    mutex_unlock(&core->lock);
    return (ssize_t)count;
}

static DEVICE_ATTR(debug, S_IWUSR | S_IRUGO,
    max77696_debug_show, max77696_debug_store);

static struct attribute *max77696_attr[] = {
    &dev_attr_debug.attr,
    NULL
};

static const struct attribute_group max77696_attr_group = {
    .name  = MAX77696_CORE_ATTR_GROUP_NAME,
    .attrs = max77696_attr,
};

struct max77696_subdev {
    bool                initialized;
    char               *compatible;
    char               *name;
    int                 irq;
    void               *pdata;
    size_t              pdata_sz;
    struct max77696_io *io;
    void               *drv_data;
};

static struct max77696_subdev max77696_subdevs[NSUBDEV];
#ifdef CONFIG_MAX77696_DT
static struct of_dev_auxdata max77696_auxdata[NSUBDEV+1];
#endif /* CONFIG_MAX77696_DT */

#define __subdev_entry(_blk_num, _member) \
        max77696_subdevs[_blk_num]._member

#define __set_subdev_entry(_blk_num, _member, _val) \
        do {\
            __subdev_entry(_blk_num, _member) = _val;\
        } while (0)

#define __init_subdev_entry(_blk, _member, _init_val) \
        __set_subdev_entry(MAX77696_BLK_##_blk, _member, _init_val)

#define __chip_io(_core, _io)  (&((_core) ->chip_io[MAX77696_IO_##_io]))

#define __INIT_SUBDEV(_blk, _name, _irq, _pdata, _pdata_sz, _io_ptr) \
        do {\
            __init_subdev_entry(_blk, initialized, true);\
            __init_subdev_entry(_blk, compatible , "maxim,"_name);\
            __init_subdev_entry(_blk, name       , _name);\
            __init_subdev_entry(_blk, irq        , _irq);\
            __init_subdev_entry(_blk, pdata      , _pdata);\
            __init_subdev_entry(_blk, pdata_sz   , _pdata_sz);\
            __init_subdev_entry(_blk, io         , _io_ptr);\
            __init_subdev_entry(_blk, drv_data   , NULL);\
        } while (0)

#define INIT_SUBDEVS(_core, _blk, _io, _pdata_member) \
        __INIT_SUBDEV(_blk, MAX77696_##_blk##_NAME, MAX77696_IRQ_##_blk,\
            _core->pdata->_pdata_member, sizeof(*_core->pdata->_pdata_member),\
            __chip_io(_core, _io))
#define INIT_SUBDEV(_core, _blk, _io, _pdata_member) \
        __INIT_SUBDEV(_blk, MAX77696_##_blk##_NAME, MAX77696_IRQ_##_blk,\
            _core->pdata->_pdata_member, sizeof(*_core->pdata->_pdata_member),\
            __chip_io(_core, _io))
#define INIT_SUBDEV_VIRT(_core, _blk, _pdata_member) \
        __INIT_SUBDEV(_blk, MAX77696_##_blk##_NAME, -EINVAL,\
            _core->pdata->_pdata_member, sizeof(*_core->pdata->_pdata_member),\
            NULL)
#define INIT_SUBDEV_CORE(_core, _blk, _irq) \
        __INIT_SUBDEV(_blk, MAX77696_##_blk##_NAME, MAX77696_IRQ_##_irq,\
            NULL, 0, _core->io)

/*******************************************************************************
 * MAX77696-internal Services
 ******************************************************************************/

__devinit int max77696_init (struct max77696_core *core)
{
    struct device *dev = core->dev;
    int i, rc;

    pr_info(FILE_DESC" "FILE_VERSION"\n");

    /* Save core IO interface */
    core->io = __chip_io(core, PMIC);

    #define MAX77696_IRQ_INVAL  (-EINVAL)
    #define MAX77696_IRQ_CLK    MAX77696_IRQ_INVAL
    #define MAX77696_IRQ_WDT    MAX77696_IRQ_INVAL
    #define MAX77696_IRQ_BBST   MAX77696_IRQ_INVAL
    #define MAX77696_IRQ_LSW    MAX77696_IRQ_INVAL
    #define MAX77696_IRQ_VDDQ   MAX77696_IRQ_INVAL
    #define MAX77696_IRQ_LED    MAX77696_IRQ_INVAL
    #define MAX77696_IRQ_CHG    MAX77696_IRQ_CHGA
    #define MAX77696_IRQ_EH     MAX77696_IRQ_CHGB

    /* Core devices */
    INIT_SUBDEV_CORE(core, CORE  , INVAL );
    INIT_SUBDEV_CORE(core, TOPINT, INVAL );
    INIT_SUBDEV_CORE(core, TOPSYS, TOPSYS);

    /* PMIC devices */
    INIT_SUBDEV(core, GPIO, PMIC, gpio_pdata );
    INIT_SUBDEV(core, CLK , PMIC, clk_pdata  );
    INIT_SUBDEV(core, WDT , PMIC, wdt_pdata  );
    INIT_SUBDEV(core, ADC , PMIC, adc_pdata  );
    INIT_SUBDEV(core, BUCK, PMIC, bucks_pdata);
#if defined(CONFIG_MAX77796)
    INIT_SUBDEV(core, BBST, PMIC, bbsts_pdata);
#endif /* CONFIG_MAX77796 */
    INIT_SUBDEV(core, LDO , PMIC, ldos_pdata );
    INIT_SUBDEV(core, LSW , PMIC, lsws_pdata );
    INIT_SUBDEV(core, EPD , PMIC, epds_pdata );
    INIT_SUBDEV(core, VDDQ, PMIC, vddq_pdata );
    INIT_SUBDEV(core, LED , PMIC, leds_pdata );
    INIT_SUBDEV(core, WLED, PMIC, bl_pdata   );
    INIT_SUBDEV(core, CHG , PMIC, chg_pdata  );
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    INIT_SUBDEV(core, EH  , RTC , eh_pdata   );
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */

    /* RTC device */
    INIT_SUBDEV(core, RTC, RTC, rtc_pdata);

    /* UIC device */
    INIT_SUBDEV(core, UIC, UIC, uic_pdata);

    /* FG device */
    INIT_SUBDEV(core, FG, FG, battery_pdata);

    /* TOPSYS-derived devices */
    INIT_SUBDEV_VIRT(core, KEY, key_pdata);

    /* Print PMIC identification */
    __printf_id(core);

    /* Device can wake up system */
    device_init_wakeup(core->dev, true);

    core->attr_grp = &max77696_attr_group;
    rc = sysfs_create_group(core->kobj, core->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        core->attr_grp = NULL;
        return rc;
    }

    /* Initialize core block TOPINT */
    rc = max77696_topint_init(core);
    if (unlikely(rc)) {
        return rc;
    }

    /* Initialize core block TOPSYS */
    rc = max77696_topsys_init(core);
    if (unlikely(rc)) {
        return rc;
    }

    /* Initialize core DEBUG */
    rc = max77696_dbg_init(core);
    if (unlikely(rc)) {
        return rc;
    }

#ifdef DEBUG
    /* List up enabled sub devices */
    for (i = 0; i < NSUBDEV; i++) {
        if (unlikely(__subdev_entry(i, initialized) != true)) {
            continue;
        }
        dev_info(dev, "subdev[%d] %s\n", i, __subdev_entry(i, name));
    }
#endif /* DEBUG */

    /* Add sub devices */
#ifdef CONFIG_MAX77696_DT
    memset(max77696_auxdata, 0x00, sizeof(max77696_auxdata));
    rc = 0;
    for (i = 0; i < NSUBDEV; i++) {
        if (unlikely(__subdev_entry(i, initialized) != true)) {
            continue;
        }

        max77696_auxdata[rc].compatible    = __subdev_entry(i, compatible);
        max77696_auxdata[rc].name          = __subdev_entry(i, name      );
        max77696_auxdata[rc].platform_data = __subdev_entry(i, pdata     );
        rc++;
    }
    rc = of_platform_populate(dev->of_node, NULL, max77696_auxdata, dev);
    if (unlikely(rc)) {
        dev_err(dev, "failed to populate of_dev [%d]\n", rc);
        return rc;
    }
#else /* CONFIG_MAX77696_DT */
    for (i = 0; i < NSUBDEV; i++) {
        if (unlikely(__subdev_entry(i, initialized) != true)) {
            continue;
        }

        rc = max77696_add_subdevices(core, __subdev_entry(i, name ),
            __subdev_entry(i, pdata), __subdev_entry(i, pdata_sz));
        if (unlikely(rc)) {
            return rc;
        }
    }
#endif /* CONFIG_MAX77696_DT */

    return 0;
}

__devexit void max77696_exit (struct max77696_core *core)
{
#ifndef CONFIG_MAX77696_DT
    mfd_remove_devices(core->dev);
#endif /* !CONFIG_MAX77696_DT */

    if (likely(core->attr_grp)) {
        sysfs_remove_group(core->kobj, core->attr_grp);
    }

    max77696_dbg_exit(core);
    max77696_topsys_exit(core);
    max77696_topint_exit(core);

    device_init_wakeup(core->dev, false);
}

int max77696_suspend_core (struct max77696_core *core)
{
    int rc;

    rc = max77696_dbg_suspend(core);
    if (unlikely(rc)) {
        goto failed_to_suspend_dbg;
    }

    rc = max77696_topsys_suspend(core);
    if (unlikely(rc)) {
        goto failed_to_suspend_topsys;
    }

    rc = max77696_topint_suspend(core);
    if (unlikely(rc)) {
        goto failed_to_suspend_topint;
    }

    return 0;

failed_to_suspend_topint:
    max77696_topsys_resume(core);
failed_to_suspend_topsys:
    max77696_dbg_resume(core);
failed_to_suspend_dbg:
    return rc;
}

int max77696_resume_core (struct max77696_core *core)
{
    max77696_topint_resume(core);
    max77696_topsys_resume(core);
    max77696_dbg_resume(core);

    return 0;
}

/*******************************************************************************
 * MAX77696-external Services
 ******************************************************************************/

struct device *max77696_dev (const char* name)
{
    struct device *dev;

    dev = bus_find_device_by_name(&platform_bus_type, NULL, name);
    if (unlikely(!dev)) {
        pr_err(FILE_NAME": device not found - %s\n", name);
        goto out;
    }

out:
    return dev;
}
EXPORT_SYMBOL(max77696_dev);

/* Return chip IRQ number (0 .. MAX77696_IRQ_NUM_OF_INTS) of max77696 block
 */
int __max77696_get_block_devirq (struct device *coredev,
    unsigned int block_num)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    int rc;

    if (unlikely(!core)) {
        pr_err(FILE_NAME": not ready\n");
        return -ENODEV;
    }

    mutex_lock(&core->lock);

    if (unlikely(block_num >= NSUBDEV)) {
        pr_err(FILE_NAME": invalid block number - %u\n", block_num);
        rc = -EINVAL;
        goto out;
    }

    rc = __subdev_entry(block_num, irq);

out:
    mutex_unlock(&core->lock);
    return rc;
}
EXPORT_SYMBOL(__max77696_get_block_devirq);

int __max77696_set_block_devirq (struct device *coredev,
    unsigned int block_num, int devirq)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    int rc = 0;

    if (unlikely(!core)) {
        pr_err(FILE_NAME": not ready\n");
        return -ENODEV;
    }

    mutex_lock(&core->lock);

    if (unlikely(block_num >= NSUBDEV)) {
        pr_err(FILE_NAME": invalid block number - %u\n", block_num);
        rc = -EINVAL;
        goto out;
    }

    __set_subdev_entry(block_num, irq, devirq);

out:
    mutex_unlock(&core->lock);
    return rc;
}
EXPORT_SYMBOL(__max77696_set_block_devirq);

struct max77696_io *__max77696_get_block_io (struct device *coredev,
    unsigned int block_num)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    struct max77696_io *io;

    if (unlikely(!core)) {
        pr_err(FILE_NAME": not ready\n");
        return ERR_PTR(-ENODEV);
    }

    mutex_lock(&core->lock);

    if (unlikely(block_num >= NSUBDEV)) {
        pr_err(FILE_NAME": invalid block number - %u\n", block_num);
        io = ERR_PTR(-EINVAL);
        goto out;
    }

    io = __subdev_entry(block_num, io);

out:
    mutex_unlock(&core->lock);
    return io;
}
EXPORT_SYMBOL(__max77696_get_block_io);
