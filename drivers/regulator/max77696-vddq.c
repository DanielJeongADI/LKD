/*
 * MAX77696 LPDDR2 VDDQ Supply Driver
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

#define DRIVER_DESC    MAX77696_DESC" LPDDR2 VDDQ Supply Driver"
#define DRIVER_NAME    MAX77696_VDDQ_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define VDDQSET                      0x6B
#define VDDQSET_ADJ                  BITS(7,3)
#define VDDQSET_BYP_VDDOK            BIT (2)

#define VDDQ_NVOLTS                  32 /* 5-bit */

struct max77696_vddq {
    struct mutex                         lock;
    struct max77696_vddq_platform_data  *pdata;
    struct max77696_core                *core;
    struct max77696_io                  *io;
    struct device                       *dev;
    struct kobject                      *kobj;
    const struct attribute_group        *attr_grp;

    struct regulator_desc                rdesc;
    struct regulator_dev                *rdev;
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

#define mV_to_uV(_mV)      ((_mV) * 1000)
#define uV_to_mV(_uV)      ((_uV) / 1000)
#define V_to_uV(_V)        (mV_to_uV(_V * 1000))
#define uV_to_V(_uV)       (uV_to_mV(_uV) / 1000)

static const s8 max77696_vddq_adj[] = {
      0, - 4, - 8, -12, -16, -20, -24, -28,
    -32, -36, -40, -44, -48, -52, -56, -60,
      4,   8,  12,  16,  20,  24,  28,  32,
     36,  40,  44,  48,  52,  56,  60,  64,
};

static __always_inline
int max77696_vddq_calc_voltage (struct max77696_vddq *vddq, int selector)
{
    unsigned max_selector = ARRAY_SIZE(max77696_vddq_adj)-1;
    int adj, uV;

    /* VDDQOUT = 0.5 * VDDQIN * (1 + ADJ / 100) */
    adj = (int)max77696_vddq_adj[min(max_selector, (unsigned)selector)];
    uV  = vddq->pdata->input_uV * (100 + adj);
    uV  = DIV_ROUND_UP(uV, 200);

    return uV;
}

static __always_inline
int max77696_vddq_list_voltage (struct max77696_vddq *vddq, unsigned selector)
{
    int rc;

    rc = max77696_vddq_calc_voltage(vddq, (int)selector);

    dev_vdbg(vddq->dev, "list voltage - %u => %duV\n", selector, rc);
    return rc;
}

static __always_inline
int max77696_vddq_select_voltage (struct max77696_vddq *vddq, int uV)
{
    int diff_min, i, rc;

    /* find index of same or nearest larger value */

    diff_min = INT_MAX;
    rc       = 0;

    for (i = 0; i < ARRAY_SIZE(max77696_vddq_adj); i++) {
        int diff = max77696_vddq_calc_voltage(vddq, i);
        if (unlikely(diff == uV)) {
            rc = i;
            break;
        }
        if (unlikely(diff < uV)) {
            continue;
        }
        diff -= uV;
        if (diff < diff_min) {
            diff_min = diff;
            rc       = i;
        }
    }

    dev_vdbg(vddq->dev, "select voltage - %duV => %d\n", uV, rc);
    return rc;
}

static __inline int max77696_vddq_setup (struct max77696_vddq *vddq)
{
    return 0;
}

static ssize_t max77696_vddq_vddqin_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_vddq *vddq = dev_get_drvdata(dev);
    int rc;

    __lock(vddq);

    rc = (int)snprintf(buf, PAGE_SIZE, "%u uV\n", vddq->pdata->input_uV);

    __unlock(vddq);
    return (ssize_t)rc;
}

#define DEFINE_VDDQ_DEV_ATTR_RO(_name) \
static DEVICE_ATTR(_name, S_IRUGO,\
    max77696_vddq_##_name##_show, NULL)

DEFINE_VDDQ_DEV_ATTR_RO(vddqin);

#define VDDQS_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_vddq_attr[] = {
    VDDQS_DEV_ATTR(vddqin),
    NULL
};

static const struct attribute_group max77696_vddq_attr_group = {
    .attrs = max77696_vddq_attr,
};

static int max77696_vddq_op_list_voltage (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_vddq *vddq = rdev_get_drvdata(rdev);
    int rc;

    __lock(vddq);

    rc  = max77696_vddq_list_voltage(vddq, selector);
    rc += rdev->constraints ? rdev->constraints->uV_offset : 0;

    __unlock(vddq);
    return rc;
}

static int max77696_vddq_op_set_voltage_sel (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_vddq *vddq = rdev_get_drvdata(rdev);
    int rc;

    __lock(vddq);

    rc = max77696_write_reg_bit(vddq->io, VDDQSET, ADJ, (u16)selector);
    if (unlikely(rc)) {
        dev_err(vddq->dev, "ADJ write error [%d]\n", rc);
        goto out;
    }

    dev_dbg(vddq->dev, "ADJ written 0x%04Xh\n", (u16)selector);

out:
    __unlock(vddq);
    return rc;
}

static int max77696_vddq_op_get_voltage_sel (struct regulator_dev *rdev)
{
    struct max77696_vddq *vddq = rdev_get_drvdata(rdev);
    u16 adj = 0;
    int rc;

    __lock(vddq);

    rc = max77696_read_reg_bit(vddq->io, VDDQSET, ADJ, &adj);
    if (unlikely(rc)) {
        dev_err(vddq->dev, "ADJ read error [%d]\n", rc);
        goto out;
    }

    rc = (int)adj;
    dev_dbg(vddq->dev, "ADJ read 0x%04Xh\n", adj);

out:
    __unlock(vddq);
    return rc;
}

static struct regulator_ops max77696_vddq_ops = {
    /* enumerate supported voltages */
    .list_voltage         = max77696_vddq_op_list_voltage,

    /* get/set regulator voltage */
    .set_voltage_sel      = max77696_vddq_op_set_voltage_sel,
    .get_voltage_sel      = max77696_vddq_op_get_voltage_sel,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
struct regulator_dev *max77696_vddq_register_rdev (struct device *dev)
{
    struct max77696_vddq *vddq = dev_get_drvdata(dev);
    struct regulator_config config;

    memset(&config, 0x00, sizeof(config));
    config.dev         = dev;
    config.init_data   = &vddq->pdata->init_data;
    config.driver_data = vddq;
    config.of_node     = dev->of_node;

    return regulator_register(&vddq->rdesc, &config);
}
#else /* LINUX_VERSION_CODE ... */
static __always_inline
struct regulator_dev *max77696_vddq_register_rdev (struct device *dev)
{
    struct max77696_vddq *vddq = dev_get_drvdata(dev);

    return regulator_register(&vddq->rdesc, dev,
        &vddq->pdata->init_data, vddq, dev->of_node);
}
#endif /* LINUX_VERSION_CODE ... */

static void *max77696_vddq_get_platdata (struct max77696_vddq *vddq)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_vddq_platform_data *pdata;
    struct device *dev = vddq->dev;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;
    struct regulator_init_data *init_data;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*pdata));
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->input_uV = 1200000;
    of_property_u32(np, "input_uV", &pdata->input_uV);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
    init_data = of_get_regulator_init_data(dev, np);
#else /* VERSION < 3.19.0 */
    init_data = of_get_regulator_init_data(dev, np, NULL);
#endif /* VERSION ... */
    if (likely(init_data)) {
        memcpy(&pdata->init_data, init_data, sizeof(pdata->init_data));
        devm_kfree(dev, init_data);
    }

#if 0
    #define VDDQ_MIN  MAX77696_VDDQ_MIN(pdata->input_uV)
    #define VDDQ_MAX  MAX77696_VDDQ_MAX(pdata->input_uV)
    pdata->init_data.constraints.min_uV           = VDDQ_MIN;
    pdata->init_data.constraints.max_uV           = VDDQ_MAX;
    pdata->init_data.constraints.always_on        = true;
    pdata->init_data.constraints.boot_on          = true;
  //pdata->init_data.constraints.name             = "MAX77696-VDDQ";
#endif

    /* Overwrite valid modes and ops mask */
    pdata->init_data.constraints.valid_modes_mask = 0;
    pdata->init_data.constraints.valid_ops_mask   = REGULATOR_CHANGE_VOLTAGE;
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "INPUT", "%u uV", pdata->input_uV);

out:
    return pdata;
}

static __always_inline
void max77696_vddq_destroy (struct max77696_vddq *vddq)
{
    struct device *dev = vddq->dev;

    if (likely(vddq->attr_grp)) {
        sysfs_remove_group(vddq->kobj, vddq->attr_grp);
    }

    if (likely(vddq->rdev)) {
        regulator_unregister(vddq->rdev);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(vddq->pdata)) {
        devm_kfree(dev, vddq->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&vddq->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, vddq);
}

static __devinit int max77696_vddq_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_vddq *vddq;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    vddq = devm_kzalloc(dev, sizeof(*vddq), GFP_KERNEL);
    if (unlikely(!vddq)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*vddq));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, vddq);

    mutex_init(&vddq->lock);
    vddq->core  = core;
    vddq->io    = max77696_get_block_io(dev->parent, VDDQ);
    vddq->dev   = dev;
    vddq->kobj  = &dev->kobj;

    vddq->pdata = max77696_vddq_get_platdata(vddq);
    if (unlikely(IS_ERR(vddq->pdata))) {
        rc = PTR_ERR(vddq->pdata);
        vddq->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    rc = max77696_vddq_setup(vddq);
    if (unlikely(rc)) {
        dev_err(dev, "failed to setup [%d]\n", rc);
        goto abort;
    }

    vddq->rdesc.name       = DRIVER_NAME;
    vddq->rdesc.id         = 0;
    vddq->rdesc.n_voltages = VDDQ_NVOLTS;
    vddq->rdesc.ops        = &max77696_vddq_ops;
    vddq->rdesc.type       = REGULATOR_VOLTAGE;
    vddq->rdesc.owner      = THIS_MODULE;

    /* Register to regulator class */
    vddq->rdev = max77696_vddq_register_rdev(dev);
    if (unlikely(IS_ERR(vddq->rdev))) {
        rc = PTR_ERR(vddq->rdev);
        dev_err(dev, "failed to register regulator [%d]\n", rc);
        vddq->rdev = NULL;
        goto abort;
    }

    /* Create max77696-vddq sysfs attributes */
    vddq->attr_grp = &max77696_vddq_attr_group;
    rc = sysfs_create_group(vddq->kobj, vddq->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        vddq->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_vddq_destroy(vddq);
    return rc;
}

static __devexit int max77696_vddq_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_vddq *vddq = dev_get_drvdata(dev);

    max77696_vddq_destroy(vddq);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_vddq_suspend (struct device *dev)
{
    return 0;
}

static int max77696_vddq_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_vddq_pm,
    max77696_vddq_suspend, max77696_vddq_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_vddq_of_match_table[] = {
    { .compatible = "maxim,"DRIVER_NAME },
    { }
};
MODULE_DEVICE_TABLE(of, max77696_vddq_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_vddq_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_vddq_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_vddq_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_vddq_probe,
    .remove                = __devexit_p(max77696_vddq_remove),
};

static __init int max77696_vddq_driver_init (void)
{
    return platform_driver_register(&max77696_vddq_driver);
}
module_init(max77696_vddq_driver_init);

static __exit void max77696_vddq_driver_exit (void)
{
    platform_driver_unregister(&max77696_vddq_driver);
}
module_exit(max77696_vddq_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
