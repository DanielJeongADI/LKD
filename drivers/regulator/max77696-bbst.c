/*
 * MAX77696 BUCK Boost Regulators Driver
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
#include <linux/delay.h>
#include <linux/i2c.h>

#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/max77696.h>

#if defined(CONFIG_MAX77796)
#define DRIVER_DESC    MAX77696_DESC" BUCK Boost Regulators Driver"
#define DRIVER_NAME    MAX77696_BBST_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define BBST_VREG_NAME               MAX77696_NAME"-bbst"
#define BBST_NREG                    MAX77696_VREG_NUM_OF_BBSTS

#define BBST_ENABLE_DELAY            800 /* us */
#define BBST_DEFAULT_MODE            REGULATOR_MODE_FAST

#define BB1CNFG                      0xC4
#define BB1VOUT                      0xC5

#define BBCNFG_OVPTH                 BITS(5,4)
#define BBCNFG_ADEN                  BIT (3)
#define BBCNFG_FPWMEN                BIT (2)

struct max77696_bbst_vid {
    int  n_volts;
    int *vtable;
    int  offset_uV, step_uV;
};

struct max77696_bbst_desc {
    struct regulator_desc    rdesc;
    s32                      ramp_up_rate;   /* uV/usec */
    s32                      ramp_down_rate; /* uV/usec */
    struct max77696_bbst_vid vid;
    struct max77696_bitdesc  vout;
    struct max77696_bitdesc  ovpth;
    struct max77696_bitdesc  ad_en;
    struct max77696_bitdesc  fpwm_en;
};

struct max77696_bbsts;
struct max77696_bbst {
    struct max77696_bbsts     *parent;
    struct max77696_io        *io;
    struct max77696_bbst_desc *desc;

    struct platform_device    *pdev;
    struct regulator_dev      *rdev;
};

struct max77696_bbsts {
    struct mutex                         lock;
    struct max77696_bbsts_platform_data *pdata;
    struct max77696_core                *core;
    struct max77696_io                  *io;
    struct device                       *dev;
    struct kobject                      *kobj;
    const struct attribute_group        *attr_grp;

    struct max77696_bbst                 bbsts[BBST_NREG];
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

#define mV_to_uV(_mV)      ((_mV) * 1000)
#define uV_to_mV(_uV)      ((_uV) / 1000)
#define V_to_uV(_V)        (mV_to_uV(_V * 1000))
#define uV_to_V(_uV)       (uV_to_mV(_uV) / 1000)

static __always_inline
int max77696_bbst_list_voltage (struct max77696_bbst *bbst, unsigned selector)
{
    struct max77696_bbst_vid *vid = &bbst->desc->vid;
    int rc;

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(&bbst->pdev->dev, "voltage control not supported\n");
        rc = 0;
        goto out;
    }

    /* table */
    if (unlikely(vid->vtable)) {
        rc = vid->vtable[min(vid->n_volts-1, (int)selector)];
        goto out;
    }

    /* linear */
    rc = vid->offset_uV + min(vid->n_volts-1, (int)selector) * vid->step_uV;

out:
    dev_vdbg(&bbst->pdev->dev, "list voltage - %u => %duV\n", selector, rc);
    return rc;
}

static __always_inline
int max77696_bbst_select_voltage (struct max77696_bbst *bbst, int uV)
{
    struct max77696_bbst_vid *vid = &bbst->desc->vid;
    int rc;

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(&bbst->pdev->dev, "voltage control not supported\n");
        rc = 0;
        goto out;
    }

    /* table */
    if (unlikely(vid->vtable)) {
        /* find index of same or nearest larger value */
        int i, diff_min;
        diff_min = INT_MAX;
        rc       = 0;
        for (i = 0; i < vid->n_volts; i++) {
            int diff;
            if (unlikely(vid->vtable[i] == uV)) {
                rc = i;
                break;
            }
            if (unlikely(vid->vtable[i] < uV)) {
                continue;
            }
            diff = vid->vtable[i] - uV;
            if (diff < diff_min) {
                diff_min = diff;
                rc       = i;
            }
        }
        goto out;
    }

    /* linear */
    rc = DIV_ROUND_UP(uV - vid->offset_uV, vid->step_uV);
    rc = min(vid->n_volts-1, max(0, rc));

out:
    dev_vdbg(&bbst->pdev->dev, "select voltage - %duV => %d\n", uV, rc);
    return rc;
}

static int max77696_bbst_op_list_voltage (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_bbst *bbst = rdev_get_drvdata(rdev);
    int rc;

    __lock(bbst->parent);

    rc  = max77696_bbst_list_voltage(bbst, selector);
    rc += rdev->constraints ? rdev->constraints->uV_offset : 0;

    __unlock(bbst->parent);
    return rc;
}

static int max77696_bbst_op_set_voltage_sel (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_bbst *bbst = rdev_get_drvdata(rdev);
    struct max77696_bbst_desc *desc = bbst->desc;
    struct max77696_bitdesc *vout_bitdesc = &desc->vout;
    int rc;

    __lock(bbst->parent);

    rc = max77696_write_bitdesc(bbst->io, vout_bitdesc, (u16)selector);
    if (unlikely(rc)) {
        dev_err(&bbst->pdev->dev, "VOUT write error [%d]\n", rc);
        goto out;
    }

    dev_dbg(&bbst->pdev->dev, "VOUT written 0x%04Xh\n", (u16)selector);

out:
    __unlock(bbst->parent);
    return rc;
}

static int max77696_bbst_op_get_voltage_sel (struct regulator_dev *rdev)
{
    struct max77696_bbst *bbst = rdev_get_drvdata(rdev);
    struct max77696_bbst_desc *desc = bbst->desc;
    struct max77696_bitdesc *vout_bitdesc = &desc->vout;
    u16 vout = 0;
    int rc;

    __lock(bbst->parent);

    rc = max77696_read_bitdesc(bbst->io, vout_bitdesc, &vout);
    if (unlikely(rc)) {
        dev_err(&bbst->pdev->dev, "VOUT read error [%d]\n", rc);
        rc = -EIO;
        goto out;
    }

    rc = (int)vout;
    dev_dbg(&bbst->pdev->dev, "VOUT read 0x%04Xh\n", vout);

out:
    __unlock(bbst->parent);
    return rc;
}

static int max77696_bbst_op_set_mode (struct regulator_dev *rdev,
    unsigned int mode)
{
    struct max77696_bbst *bbst = rdev_get_drvdata(rdev);
    struct max77696_bbst_desc *desc = bbst->desc;
    struct max77696_bitdesc *fpwm_en_bitdesc = &desc->fpwm_en;
    u16 fpwm_en;
    int rc;

    __lock(bbst->parent);

    switch (mode) {
    case REGULATOR_MODE_FAST:
        fpwm_en = true;
        break;

    case REGULATOR_MODE_NORMAL:
        fpwm_en = false;
        break;

    default:
        rc = -EINVAL;
        goto out;
    }

    rc = max77696_write_bitdesc(bbst->io, fpwm_en_bitdesc, fpwm_en);
    if (unlikely(rc)) {
        dev_err(&bbst->pdev->dev, "FPWM_EN write error [%d]\n", rc);
        goto out;
    }

out:
    __unlock(bbst->parent);
    return rc;
}

static unsigned int max77696_bbst_op_get_mode (struct regulator_dev *rdev)
{
    struct max77696_bbst *bbst = rdev_get_drvdata(rdev);
    struct max77696_bbst_desc *desc = bbst->desc;
    struct max77696_bitdesc *fpwm_en_bitdesc = &desc->fpwm_en;
    u16 fpwm_en;
    int rc;

    __lock(bbst->parent);

    rc = max77696_read_bitdesc(bbst->io, fpwm_en_bitdesc, &fpwm_en);
    if (unlikely(rc)) {
        dev_err(&bbst->pdev->dev, "FPWM_EN read error [%d]\n", rc);
        rc = 0; /* diabled */
        goto out;
    }

    rc = fpwm_en ? REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL;

out:
    __unlock(bbst->parent);
    return (unsigned int)rc;
}

static int max77696_bbst_op_enable_time (struct regulator_dev *rdev)
{
    return BBST_ENABLE_DELAY;
}

static struct regulator_ops max77696_bbst_ops = {
    /* enumerate supported voltages */
    .list_voltage         = max77696_bbst_op_list_voltage,

    /* get/set regulator voltage */
    .set_voltage_sel      = max77696_bbst_op_set_voltage_sel,
    .get_voltage_sel      = max77696_bbst_op_get_voltage_sel,

    /* enable/disable regulator */
//  .enable               = max77696_bbst_op_enable,
//  .disable              = max77696_bbst_op_disable,
//  .is_enabled           = max77696_bbst_op_is_enabled,

    /* get/set regulator operating mode (defined in consumer.h) */
    .set_mode             = max77696_bbst_op_set_mode,
    .get_mode             = max77696_bbst_op_get_mode,

    /* Time taken to enable or set voltage on the regulator */
    .enable_time          = max77696_bbst_op_enable_time,
//  .set_voltage_time_sel = max77696_bbst_op_set_voltage_time_sel,
};

#define __BBST_DESC_RDESC(_id, _n_volts) \
        {\
            .name       = BBST_VREG_NAME #_id,\
            .id         = MAX77696_VREG_BBST##_id,\
            .n_voltages = (unsigned)(_n_volts),\
            .ops        = &max77696_bbst_ops,\
            .type       = REGULATOR_VOLTAGE,\
            .owner      = THIS_MODULE,\
        }
#define __BBST_DESC_VID_TABLE(_vtable) \
        {\
            .n_volts     = (int)ARRAY_SIZE(_vtable),\
            .vtable      = _vtable,\
            .offset_uV   = 0,\
            .step_uV     = 0,\
        }
#define __BBST_DESC_VID_LINEAR(_n_volts, _offset, _step) \
        {\
            .n_volts     = _n_volts,\
            .vtable      = NULL,\
            .offset_uV   = _offset,\
            .step_uV     = _step,\
        }
#define __BBST_DESC_BITDESC(_id) \
        .vout    = MAX77696_BITDESC(BB##_id##VOUT, 0x7F         ),\
        .ovpth   = MAX77696_BITDESC(BB##_id##CNFG, BBCNFG_OVPTH ),\
        .ad_en   = MAX77696_BITDESC(BB##_id##CNFG, BBCNFG_ADEN  ),\
        .fpwm_en = MAX77696_BITDESC(BB##_id##CNFG, BBCNFG_FPWMEN)

#define MIN_UV   2600000
#define MAX_UV   4187500
#define STEP_UV  12500
#define N_VOLTS  ((((MAX_UV) - (MIN_UV)) / (STEP_UV)) + 1)

#define MAX77696_BBST_DESC(_id) \
        [MAX77696_VREG_BBST##_id] = {\
            .rdesc = __BBST_DESC_RDESC(_id, N_VOLTS),\
            .vid   = __BBST_DESC_VID_LINEAR(N_VOLTS, MIN_UV, STEP_UV),\
            __BBST_DESC_BITDESC(_id),\
        }

static struct max77696_bbst_desc max77696_bbst_descs[] = {
    MAX77696_BBST_DESC(1),
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
struct regulator_dev *max77696_bbst_register_rdev (struct device *dev)
{
    struct max77696_bbst *bbst = dev_get_drvdata(dev);
    struct regulator_config config;

    memset(&config, 0x00, sizeof(config));
    config.dev         = dev;
    config.init_data   = dev_get_platdata(dev);
    config.driver_data = bbst;
    config.of_node     = dev->of_node;

    return regulator_register(&bbst->desc->rdesc, &config);
}
#else /* LINUX_VERSION_CODE ... */
static __always_inline
struct regulator_dev *max77696_bbst_register_rdev (struct device *dev)
{
    struct max77696_bbst *bbst = dev_get_drvdata(dev);

    return regulator_register(&bbst->desc->rdesc, dev,
        dev_get_platdata(dev), bbst, dev->of_node);
}
#endif /* LINUX_VERSION_CODE ... */

static int max77696_bbst_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_bbst *bbst = dev_get_drvdata(dev);
    int rc;

    /* Register my own regulator device */
    bbst->rdev = max77696_bbst_register_rdev(dev);
    if (unlikely(IS_ERR(bbst->rdev))) {
        rc = PTR_ERR(bbst->rdev);
        dev_err(dev, "failed to register regulator [%d]\n", rc);
        bbst->rdev = NULL;
        goto abort;
    }

    dev_set_drvdata(dev, bbst);

    return 0;

abort:
    if (likely(bbst->rdev)) {
        regulator_unregister(bbst->rdev);
        bbst->rdev = NULL;
    }
    return rc;
}

static int max77696_bbst_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_bbst *bbst = dev_get_drvdata(dev);

    regulator_unregister(bbst->rdev);
    bbst->rdev = NULL;

    return 0;
}

static struct platform_driver max77696_bbst_driver = {
    .probe       = max77696_bbst_probe,
    .remove      = max77696_bbst_remove,
    .driver.name = BBST_VREG_NAME,
};

/******************************************************************************/

static __inline int max77696_bbst_init_vreg_dev (struct max77696_bbst *bbst,
    struct max77696_bbst_cfg_data *cfg_data)
{
    return 0;
}

static void max77696_bbst_unregister_vreg_dev (struct max77696_bbst *bbst)
{
    if (likely(bbst->pdev)) {
        platform_device_del(bbst->pdev);
        bbst->pdev = NULL;
    }
}

static int max77696_bbst_register_vreg_dev (struct max77696_bbst *bbst,
    struct max77696_bbst_cfg_data *cfg_data)
{
    struct device *parent_dev = bbst->parent->dev;
    struct max77696_bbst_desc *desc = bbst->desc;
    u8 bbst_id = cfg_data->bbst_id;
    int rc;

    /* just in order to make pdev bbst_id connecting to name
     *   BBST1    1
     *   ...
     */

    bbst->pdev = platform_device_alloc(BBST_VREG_NAME, bbst_id + 1);
    if (unlikely(!bbst->pdev)) {
        dev_err(parent_dev, "failed to alloc pdev for %s\n", desc->rdesc.name);
        return -ENOMEM;
    }

    dev_set_drvdata(&bbst->pdev->dev, bbst);
    bbst->pdev->dev.platform_data = &cfg_data->init_data;
    bbst->pdev->dev.parent        = parent_dev;

    rc = max77696_bbst_init_vreg_dev(bbst, cfg_data);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to initialize %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    rc = platform_device_add(bbst->pdev);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to pdev for %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    return 0;

abort:
    max77696_bbst_unregister_vreg_dev(bbst);
    return rc;
}

/******************************************************************************/

#define BBSTS_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_bbsts_attr[] = {
    NULL
};

static const struct attribute_group max77696_bbsts_attr_group = {
    .attrs = max77696_bbsts_attr,
};

static void max77696_bbsts_unregister_vreg (struct max77696_bbsts *bbsts)
{
    int i;

    for (i = 0; i < BBST_NREG; i++) {
        max77696_bbst_unregister_vreg_dev(&bbsts->bbsts[i]);
    }

    platform_driver_unregister(&max77696_bbst_driver);
}

static int max77696_bbsts_register_vreg (struct max77696_bbsts *bbsts)
{
    struct device *dev = bbsts->dev;
    int i, rc;

    rc = platform_driver_register(&max77696_bbst_driver);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register BBST vregs driver [%d]\n", rc);
        goto out;
    }

    for (i = 0; i < bbsts->pdata->num_of_cfg_data; i++) {
        struct max77696_bbst_cfg_data *cfg_data;
        struct max77696_bbst *bbst;
        int bbst_id;

        cfg_data = &bbsts->pdata->cfg_data[i];
        bbst_id  = cfg_data->bbst_id;

        if (unlikely(bbst_id >= BBST_NREG)) {
            dev_err(dev, "invalid BBST ID - %u\n", bbst_id);
            continue;
        }

        bbst         = &bbsts->bbsts[bbst_id];
        bbst->parent = bbsts;
        bbst->io     = bbsts->io;
        bbst->desc   = &max77696_bbst_descs[bbst_id];

        dev_dbg(dev, "registering BBST vreg device for %s ...\n",
            bbst->desc->rdesc.name);

        rc = max77696_bbst_register_vreg_dev(bbst, cfg_data);
        if (unlikely(rc)) {
            dev_err(dev, "failed to register vreg dev for %s [%d]\n",
                bbst->desc->rdesc.name, rc);
            goto out;
        }
    }

out:
    return rc;
}

static void *max77696_bbsts_get_platdata (struct max77696_bbsts *bbsts)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_bbsts_platform_data *pdata;
    struct device *dev = bbsts->dev;
    int i;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;
    struct device_node *cfg_np;
    size_t sz;
    int num_of_cfg_data;

    num_of_cfg_data = of_get_child_count(np);

    sz = sizeof(*pdata) + num_of_cfg_data * sizeof(*pdata->cfg_data);
    pdata = devm_kzalloc(dev, sz, GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sz);
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->cfg_data = (void*)(pdata + 1);
    pdata->num_of_cfg_data = (size_t)num_of_cfg_data;

    i = 0;
    for_each_child_of_node(np, cfg_np) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
        struct regulator_init_data *init_data =
            of_get_regulator_init_data(dev, cfg_np);
#else /* VERSION < 3.19.0 */
        struct regulator_init_data *init_data =
            of_get_regulator_init_data(dev, cfg_np, NULL);
#endif /* VERSION ... */

        of_property_u8(cfg_np, "reg", &pdata->cfg_data[i].bbst_id);

        if (likely(init_data)) {
            memcpy(&pdata->cfg_data[i].init_data,
                init_data,
                sizeof(pdata->cfg_data[i].init_data));
            devm_kfree(dev, init_data);
        }

        /* Overwrite valid modes and ops mask */
        pdata->cfg_data[i].init_data.constraints.valid_modes_mask
            = MAX77696_BBST_VALID_MODES;
        pdata->cfg_data[i].init_data.constraints.valid_ops_mask
            = MAX77696_BBST_VALID_OPS;

        i++;
    }
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "CFGDATA", "%zu", pdata->num_of_cfg_data);
    for (i = 0; i < pdata->num_of_cfg_data; i++) {
        __prop_printk(dev, "CFGDATA", "[%d] BBST%d", i,
            pdata->cfg_data[i].bbst_id + 1);
    }

out:
    return pdata;
}

static __always_inline
void max77696_bbsts_destroy (struct max77696_bbsts *bbsts)
{
    struct device *dev = bbsts->dev;

    if (likely(bbsts->attr_grp)) {
        sysfs_remove_group(bbsts->kobj, bbsts->attr_grp);
    }

    max77696_bbsts_unregister_vreg(bbsts);

#ifdef CONFIG_MAX77696_DT
    if (likely(bbsts->pdata)) {
        devm_kfree(dev, bbsts->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&bbsts->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, bbsts);
}

static __devinit int max77696_bbsts_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_bbsts *bbsts;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    bbsts = devm_kzalloc(dev, sizeof(*bbsts), GFP_KERNEL);
    if (unlikely(!bbsts)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*bbsts));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, bbsts);

    mutex_init(&bbsts->lock);
    bbsts->core  = core;
    bbsts->io    = max77696_get_block_io(dev->parent, BBST);
    bbsts->dev   = dev;
    bbsts->kobj  = &dev->kobj;

    bbsts->pdata = max77696_bbsts_get_platdata(bbsts);
    if (unlikely(IS_ERR(bbsts->pdata))) {
        rc = PTR_ERR(bbsts->pdata);
        bbsts->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Register individual BBST drivers & devices */
    rc = max77696_bbsts_register_vreg(bbsts);
    if (unlikely(rc)) {
        goto abort;
    }

    /* Create max77696-bbsts sysfs attributes */
    bbsts->attr_grp = &max77696_bbsts_attr_group;
    rc = sysfs_create_group(bbsts->kobj, bbsts->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        bbsts->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_bbsts_destroy(bbsts);
    return rc;
}

static __devexit int max77696_bbsts_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_bbsts *bbsts = dev_get_drvdata(dev);

    max77696_bbsts_destroy(bbsts);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_bbsts_suspend (struct device *dev)
{
    return 0;
}

static int max77696_bbsts_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_bbsts_pm,
    max77696_bbsts_suspend, max77696_bbsts_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_bbsts_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_bbsts_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_bbsts_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_bbsts_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_bbsts_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_bbsts_probe,
    .remove                = __devexit_p(max77696_bbsts_remove),
};

static __init int max77696_bbsts_driver_init (void)
{
    return platform_driver_register(&max77696_bbsts_driver);
}
module_init(max77696_bbsts_driver_init);

static __exit void max77696_bbsts_driver_exit (void)
{
    platform_driver_unregister(&max77696_bbsts_driver);
}
module_exit(max77696_bbsts_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
#endif /* CONFIG_MAX77796 */