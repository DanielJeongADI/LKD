/*
 * MAX77696 Load Switches Driver
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

#define DRIVER_DESC    MAX77696_DESC" Load Switches Driver"
#define DRIVER_NAME    MAX77696_LSW_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define LSW_VREG_NAME               MAX77696_NAME"-lsw"
#define LSW_NREG                    MAX77696_VREG_NUM_OF_LSWS

#define SW1_CNTRL                   0x5C
#define SW2_CNTRL                   0x5D
#define SW3_CNTRL                   0x5E
#define SW4_CNTRL                   0x5F

#define SW_CNTRL_ADE                BIT (3)
#define SW_CNTRL_RT                 BITS(2,1)
#define SW_CNTRL_OUT                BIT (0)

struct max77696_lsw_desc {
    struct regulator_desc   rdesc;
    s32                     enable_rate;    /* output rise time control */
    int                     min_uV, max_uV; /* input range */
    struct max77696_bitdesc ad_en;
    struct max77696_bitdesc rt;
    struct max77696_bitdesc out_en;
};

struct max77696_lsws;
struct max77696_lsw {
    struct max77696_lsws      *parent;
    struct max77696_io        *io;
    struct max77696_lsw_desc  *desc;

    struct platform_device    *pdev;
    struct regulator_dev      *rdev;
};

struct max77696_lsws {
    struct mutex                         lock;
    struct max77696_lsws_platform_data  *pdata;
    struct max77696_core                *core;
    struct max77696_io                  *io;
    struct device                       *dev;
    struct kobject                      *kobj;
    const struct attribute_group        *attr_grp;

    struct max77696_lsw                  lsws[LSW_NREG];
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

#define mV_to_uV(_mV)      ((_mV) * 1000)
#define uV_to_mV(_uV)      ((_uV) / 1000)
#define V_to_uV(_V)        (mV_to_uV(_V * 1000))
#define uV_to_V(_uV)       (uV_to_mV(_uV) / 1000)

static int max77696_lsw_op_enable (struct regulator_dev *rdev)
{
    struct max77696_lsw *lsw = rdev_get_drvdata(rdev);
    struct max77696_lsw_desc *desc = lsw->desc;
    struct max77696_bitdesc *out_en_bitdesc = &desc->out_en;
    int rc;

    __lock(lsw->parent);

    rc = max77696_write_bitdesc(lsw->io, out_en_bitdesc, true);
    if (unlikely(rc)) {
        dev_err(&lsw->pdev->dev, "OUT_EN write error [%d]\n", rc);
        goto out;
    }

out:
    __unlock(lsw->parent);
    return rc;
}

static int max77696_lsw_op_disable (struct regulator_dev *rdev)
{
    struct max77696_lsw *lsw = rdev_get_drvdata(rdev);
    struct max77696_lsw_desc *desc = lsw->desc;
    struct max77696_bitdesc *out_en_bitdesc = &desc->out_en;
    int rc;

    __lock(lsw->parent);

    rc = max77696_write_bitdesc(lsw->io, out_en_bitdesc, false);
    if (unlikely(rc)) {
        dev_err(&lsw->pdev->dev, "OUT_EN write error [%d]\n", rc);
        goto out;
    }

out:
    __unlock(lsw->parent);
    return rc;
}

static int max77696_lsw_op_is_enabled (struct regulator_dev *rdev)
{
    struct max77696_lsw *lsw = rdev_get_drvdata(rdev);
    struct max77696_lsw_desc *desc = lsw->desc;
    struct max77696_bitdesc *out_en_bitdesc = &desc->out_en;
    u16 out_en;
    int rc;

    __lock(lsw->parent);

    rc = max77696_read_bitdesc(lsw->io, out_en_bitdesc, &out_en);
    if (unlikely(rc)) {
        dev_err(&lsw->pdev->dev, "OUT_EN read error [%d]\n", rc);
        rc = false; /* assume not enabled */
        goto out;
    }

    rc = !!out_en;

out:
    __unlock(lsw->parent);
    return rc;
}

static int max77696_lsw_op_enable_time (struct regulator_dev *rdev)
{
    struct max77696_lsw *lsw = rdev_get_drvdata(rdev);
    struct max77696_lsw_desc *desc = lsw->desc;
    int uV, rc;

    __lock(lsw->parent);

    if (unlikely(!rdev->supply)) {
        dev_err(&lsw->pdev->dev, "no supply regulator\n");
        uV = desc->max_uV;
        goto out;
    }

    uV = regulator_get_voltage(rdev->supply);
    if (unlikely(uV < 0)) {
        dev_err(&lsw->pdev->dev, "failed to get input voltage [%d]\n", uV);
        uV = desc->max_uV;
        goto out;
    }

out:
    if (likely(desc->enable_rate > 0)) {
        rc = DIV_ROUND_UP(uV, (int)desc->enable_rate);
    } else {
        rc = 0; /* no delay */
    }

    dev_dbg(&lsw->pdev->dev, "start up delay %dusec for %uV\n", rc, uV);

    __unlock(lsw->parent);
    return rc;
}

static struct regulator_ops max77696_lsw_ops = {
    /* enumerate supported voltages */
//  .list_voltage         = max77696_lsw_op_list_voltage,

    /* get/set regulator voltage */
//  .set_voltage_sel      = max77696_lsw_op_set_voltage_sel,
//  .get_voltage_sel      = max77696_lsw_op_get_voltage_sel,

    /* enable/disable regulator */
    .enable               = max77696_lsw_op_enable,
    .disable              = max77696_lsw_op_disable,
    .is_enabled           = max77696_lsw_op_is_enabled,

    /* get/set regulator operating mode (defined in consumer.h) */
//  .set_mode             = max77696_lsw_op_set_mode,
//  .get_mode             = max77696_lsw_op_get_mode,

    /* Time taken to enable or set voltage on the regulator */
    .enable_time          = max77696_lsw_op_enable_time,
//  .set_voltage_time_sel = max77696_lsw_op_set_voltage_time_sel,

    /* the operations below are for configuration of regulator state when
     * its parent PMIC enters a global STANDBY/HIBERNATE state */
//  .set_suspend_voltage = max77696_lsw_op_set_suspend_voltage,
//  .set_suspend_enable  = max77696_lsw_op_set_suspend_enable,
//  .set_suspend_disable = max77696_lsw_op_set_suspend_disable,
//  .set_suspend_mode    = max77696_lsw_op_set_suspend_mode,
};

#define __LSW_DESC_RDESC(_id) \
        {\
            .name       = LSW_VREG_NAME #_id,\
            .id         = MAX77696_VREG_LSW##_id,\
            .n_voltages = 0,\
            .ops        = &max77696_lsw_ops,\
            .type       = REGULATOR_VOLTAGE,\
            .owner      = THIS_MODULE,\
        }
#define __LSW_DESC_BITDESC(_id) \
        .ad_en  = MAX77696_BITDESC(SW##_id##_CNTRL, SW_CNTRL_ADE),\
        .rt     = MAX77696_BITDESC(SW##_id##_CNTRL, SW_CNTRL_RT ),\
        .out_en = MAX77696_BITDESC(SW##_id##_CNTRL, SW_CNTRL_OUT)

#define MAX77696_LSW_DESC(_id, _min, _max) \
        [MAX77696_VREG_LSW##_id] = {\
            .rdesc  = __LSW_DESC_RDESC(_id),\
            .min_uV = _min,\
            .max_uV = _max,\
            __LSW_DESC_BITDESC(_id),\
        }

static struct max77696_lsw_desc max77696_lsw_descs[] = {
    MAX77696_LSW_DESC( 1, 1000000, 1600000),
    MAX77696_LSW_DESC( 2, 1000000, 1600000),
    MAX77696_LSW_DESC( 3, 1700000, 5500000),
    MAX77696_LSW_DESC( 4, 1700000, 5500000),
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
struct regulator_dev *max77696_lsw_register_rdev (struct device *dev)
{
    struct max77696_lsw *lsw = dev_get_drvdata(dev);
    struct regulator_config config;

    memset(&config, 0x00, sizeof(config));
    config.dev         = dev;
    config.init_data   = dev_get_platdata(dev);
    config.driver_data = lsw;
    config.of_node     = dev->of_node;

    return regulator_register(&lsw->desc->rdesc, &config);
}
#else /* LINUX_VERSION_CODE ... */
static __always_inline
struct regulator_dev *max77696_lsw_register_rdev (struct device *dev)
{
    struct max77696_lsw *lsw = dev_get_drvdata(dev);

    return regulator_register(&lsw->desc->rdesc, dev,
        dev_get_platdata(dev), lsw, dev->of_node);
}
#endif /* LINUX_VERSION_CODE ... */

static int max77696_lsw_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_lsw *lsw = dev_get_drvdata(dev);
    int rc;

    /* Register my own regulator device */
    lsw->rdev = max77696_lsw_register_rdev(dev);
    if (unlikely(IS_ERR(lsw->rdev))) {
        rc = PTR_ERR(lsw->rdev);
        dev_err(dev, "failed to register regulator [%d]\n", rc);
        lsw->rdev = NULL;
        goto abort;
    }

    dev_set_drvdata(dev, lsw);

    return 0;

abort:
    if (likely(lsw->rdev)) {
        regulator_unregister(lsw->rdev);
        lsw->rdev = NULL;
    }
    return rc;
}

static int max77696_lsw_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_lsw *lsw = dev_get_drvdata(dev);

    regulator_unregister(lsw->rdev);
    lsw->rdev = NULL;

    return 0;
}

static struct platform_driver max77696_lsw_driver = {
    .probe       = max77696_lsw_probe,
    .remove      = max77696_lsw_remove,
    .driver.name = LSW_VREG_NAME,
};

/******************************************************************************/

static __inline int max77696_lsw_init_vreg_dev (struct max77696_lsw *lsw,
    struct max77696_lsw_cfg_data *cfg_data)
{
    struct device *dev = &lsw->pdev->dev;
    struct max77696_lsw_desc *desc = lsw->desc;
    struct max77696_bitdesc *bitdesc;
    int rc = 0;

    /* Output rise time control */
    bitdesc = &desc->rt;
    if (likely(__check_valid_max77696_bitdesc(bitdesc))) {
        rc = max77696_write_bitdesc(lsw->io, bitdesc, cfg_data->rt);
        if (unlikely(rc)) {
            dev_err(dev, "failed to set soft-start slew rate for %s [%d]\n",
                desc->rdesc.name, rc);
            goto out;
        }
    }

    desc->enable_rate = cfg_data->rt == MAX77696_LSW_RT_300000 ? 300000 :
                        cfg_data->rt == MAX77696_LSW_RT_100000 ? 100000 :
                        cfg_data->rt == MAX77696_LSW_RT_30000  ? 30000  :
                        cfg_data->rt == MAX77696_LSW_RT_10000  ? 10000  : 0;

out:
    return rc;
}

static void max77696_lsw_unregister_vreg_dev (struct max77696_lsw *lsw)
{
    if (likely(lsw->pdev)) {
        platform_device_del(lsw->pdev);
        lsw->pdev = NULL;
    }
}

static int max77696_lsw_register_vreg_dev (struct max77696_lsw *lsw,
    struct max77696_lsw_cfg_data *cfg_data)
{
    struct device *parent_dev = lsw->parent->dev;
    struct max77696_lsw_desc *desc = lsw->desc;
    u8 lsw_id = cfg_data->lsw_id;
    int rc;

    /* just in order to make pdev lsw_id connecting to name
     *   LSW1    1
     *   LSW2    2
     *   LSW3    3
     *   ...
     *   LSW4    4
     */

    lsw->pdev = platform_device_alloc(LSW_VREG_NAME, lsw_id + 1);
    if (unlikely(!lsw->pdev)) {
        dev_err(parent_dev, "failed to alloc pdev for %s\n", desc->rdesc.name);
        return -ENOMEM;
    }

    dev_set_drvdata(&lsw->pdev->dev, lsw);
    lsw->pdev->dev.platform_data = &cfg_data->init_data;
    lsw->pdev->dev.parent        = parent_dev;

    rc = max77696_lsw_init_vreg_dev(lsw, cfg_data);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to initialize %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    rc = platform_device_add(lsw->pdev);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to pdev for %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    return 0;

abort:
    max77696_lsw_unregister_vreg_dev(lsw);
    return rc;
}

/******************************************************************************/

#define LSWS_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_lsws_attr[] = {
    NULL
};

static const struct attribute_group max77696_lsws_attr_group = {
    .attrs = max77696_lsws_attr,
};

static void max77696_lsws_unregister_vreg (struct max77696_lsws *lsws)
{
    int i;

    for (i = 0; i < LSW_NREG; i++) {
        max77696_lsw_unregister_vreg_dev(&lsws->lsws[i]);
    }

    platform_driver_unregister(&max77696_lsw_driver);
}

static int max77696_lsws_register_vreg (struct max77696_lsws *lsws)
{
    struct device *dev = lsws->dev;
    int i, rc;

    rc = platform_driver_register(&max77696_lsw_driver);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register LSW vregs driver [%d]\n", rc);
        goto out;
    }

    for (i = 0; i < lsws->pdata->num_of_cfg_data; i++) {
        struct max77696_lsw_cfg_data *cfg_data;
        struct max77696_lsw *lsw;
        int lsw_id;

        cfg_data = &lsws->pdata->cfg_data[i];
        lsw_id   = cfg_data->lsw_id;

        if (unlikely(lsw_id >= LSW_NREG)) {
            dev_err(dev, "invalid LSW ID - %u\n", lsw_id);
            continue;
        }

        lsw         = &lsws->lsws[lsw_id];
        lsw->parent = lsws;
        lsw->io     = lsws->io;
        lsw->desc   = &max77696_lsw_descs[lsw_id];

        dev_dbg(dev, "registering LSW vreg device for %s ...\n",
            lsw->desc->rdesc.name);

        rc = max77696_lsw_register_vreg_dev(lsw, cfg_data);
        if (unlikely(rc)) {
            dev_err(dev, "failed to register vreg dev for %s [%d]\n",
                lsw->desc->rdesc.name, rc);
            goto out;
        }
    }

out:
    return rc;
}

static void *max77696_lsws_get_platdata (struct max77696_lsws *lsws)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_lsws_platform_data *pdata;
    struct device *dev = lsws->dev;
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

        of_property_u8(cfg_np, "reg", &pdata->cfg_data[i].lsw_id);
        of_property_u8(cfg_np, "rt", &pdata->cfg_data[i].rt);

        if (likely(init_data)) {
            memcpy(&pdata->cfg_data[i].init_data,
                init_data,
                sizeof(pdata->cfg_data[i].init_data));
            devm_kfree(dev, init_data);
        }

        /* Overwrite valid modes and ops mask */
        pdata->cfg_data[i].init_data.constraints.valid_modes_mask
            = MAX77696_LSW_VALID_MODES;
        pdata->cfg_data[i].init_data.constraints.valid_ops_mask
            = MAX77696_LSW_VALID_OPS;

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
        __prop_printk(dev, "CFGDATA", "[%d] LSW%d", i,
            pdata->cfg_data[i].lsw_id + 1);
    }

out:
    return pdata;
}

static __always_inline
void max77696_lsws_destroy (struct max77696_lsws *lsws)
{
    struct device *dev = lsws->dev;

    if (likely(lsws->attr_grp)) {
        sysfs_remove_group(lsws->kobj, lsws->attr_grp);
    }

    max77696_lsws_unregister_vreg(lsws);

#ifdef CONFIG_MAX77696_DT
    if (likely(lsws->pdata)) {
        devm_kfree(dev, lsws->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&lsws->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, lsws);
}

static __devinit int max77696_lsws_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_lsws *lsws;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    lsws = devm_kzalloc(dev, sizeof(*lsws), GFP_KERNEL);
    if (unlikely(!lsws)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*lsws));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, lsws);

    mutex_init(&lsws->lock);
    lsws->core  = core;
    lsws->io    = max77696_get_block_io(dev->parent, LSW);
    lsws->dev   = dev;
    lsws->kobj  = &dev->kobj;

    lsws->pdata = max77696_lsws_get_platdata(lsws);
    if (unlikely(IS_ERR(lsws->pdata))) {
        rc = PTR_ERR(lsws->pdata);
        lsws->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Register individual LSW drivers & devices */
    rc = max77696_lsws_register_vreg(lsws);
    if (unlikely(rc)) {
        goto abort;
    }

    /* Create max77696-lsws sysfs attributes */
    lsws->attr_grp = &max77696_lsws_attr_group;
    rc = sysfs_create_group(lsws->kobj, lsws->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        lsws->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_lsws_destroy(lsws);
    return rc;
}

static __devexit int max77696_lsws_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_lsws *lsws = dev_get_drvdata(dev);

    max77696_lsws_destroy(lsws);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_lsws_suspend (struct device *dev)
{
    return 0;
}

static int max77696_lsws_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_lsws_pm,
    max77696_lsws_suspend, max77696_lsws_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_lsws_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_lsws_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_lsws_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_lsws_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_lsws_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_lsws_probe,
    .remove                = __devexit_p(max77696_lsws_remove),
};

static __init int max77696_lsws_driver_init (void)
{
    return platform_driver_register(&max77696_lsws_driver);
}
module_init(max77696_lsws_driver_init);

static __exit void max77696_lsws_driver_exit (void)
{
    platform_driver_unregister(&max77696_lsws_driver);
}
module_exit(max77696_lsws_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
