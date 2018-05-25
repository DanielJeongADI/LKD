/*
 * MAX77696 CLK Driver
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

#include <linux/mfd/max77696.h>

#define DRIVER_DESC    MAX77696_DESC" CLK Driver"
#define DRIVER_NAME    MAX77696_CLK_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define CLK_32KLOAD           0x7E
#define CLK_32KLOAD_32KLOAD   BITS(1,0)

#define CLK_32KSTAT           0x7F
#define CLK_32KSTAT_MODE_SET  BIT (1)
#define CLK_32KSTAT_32KSTAT   BIT (0)

struct max77696_clk {
    struct mutex                        lock;
    struct max77696_clk_platform_data  *pdata;
    struct max77696_core               *core;
    struct max77696_io                 *io;
    struct device                      *dev;
    struct kobject                     *kobj;
    const struct attribute_group       *attr_grp;
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

static __always_inline int max77696_clk_setup (struct max77696_clk *clk)
{
    struct device *dev = clk->dev;
    struct max77696_clk_platform_data *pdata = clk->pdata;
    int rc;

    rc = max77696_write_reg_bit(clk->io, CLK_32KLOAD, 32KLOAD, pdata->load_cap);
    if (unlikely(rc)) {
        dev_err(dev, "32KLOAD write error [%d]\n", rc);
        goto out;
    }

    rc = max77696_write_reg_bit(clk->io, CLK_32KSTAT, MODE_SET, pdata->op_mode);
    if (unlikely(rc)) {
        dev_err(dev, "32KSTAT write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

#define CLK_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_clk_attr[] = {
    NULL
};

static const struct attribute_group max77696_clk_attr_group = {
    .attrs = max77696_clk_attr,
};

static void *max77696_clk_get_platdata (struct max77696_clk *clk)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_clk_platform_data *pdata;
    struct device *dev = clk->dev;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*pdata));
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->load_cap = MAX77696_CLK_LOAD_CAP_22PF;
    of_property_u8(np, "load_cap", &pdata->load_cap);

    pdata->op_mode = MAX77696_CLK_OPMODE_LOW_POWER;
    of_property_u8(np, "op_mode", &pdata->op_mode);
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "LOAD CAP", "%s",
        pdata->load_cap == MAX77696_CLK_LOAD_CAP_22PF ? "22pF" :
        pdata->load_cap == MAX77696_CLK_LOAD_CAP_12PF ? "12pF" :
        pdata->load_cap == MAX77696_CLK_LOAD_CAP_10PF ? "10pF" :
        pdata->load_cap == MAX77696_CLK_LOAD_CAP_NONE ? "none" :
        "invalid");
    __prop_printk(dev, "OP MODE", "%s",
        pdata->op_mode == MAX77696_CLK_OPMODE_LOW_POWER  ? "low power"  :
        pdata->op_mode == MAX77696_CLK_OPMODE_LOW_JITTER ? "low jitter" :
        "invalid");

out:
    return pdata;
}

static __always_inline void max77696_clk_destroy (struct max77696_clk *clk)
{
    struct device *dev = clk->dev;

    if (likely(clk->attr_grp)) {
        sysfs_remove_group(clk->kobj, clk->attr_grp);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(clk->pdata)) {
        devm_kfree(dev, clk->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&clk->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, clk);
}

static __devinit int max77696_clk_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_clk *clk;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
    if (unlikely(!clk)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*clk));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, clk);

    mutex_init(&clk->lock);
    clk->core  = core;
    clk->io    = max77696_get_block_io(dev->parent, CLK);
    clk->dev   = dev;
    clk->kobj  = &dev->kobj;

    clk->pdata = max77696_clk_get_platdata(clk);
    if (unlikely(IS_ERR(clk->pdata))) {
        rc = PTR_ERR(clk->pdata);
        clk->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Create max77696-clk sysfs attributes */
    clk->attr_grp = &max77696_clk_attr_group;
    rc = sysfs_create_group(clk->kobj, clk->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        clk->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_clk_destroy(clk);
    return rc;
}

static __devexit int max77696_clk_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_clk *clk = dev_get_drvdata(dev);

    max77696_clk_destroy(clk);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_clk_suspend (struct device *dev)
{
    return 0;
}

static int max77696_clk_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_clk_pm,
    max77696_clk_suspend, max77696_clk_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_clk_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_clk_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_clk_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_clk_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_clk_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_clk_probe,
    .remove                = __devexit_p(max77696_clk_remove),
};

static __init int max77696_clk_driver_init (void)
{
    return platform_driver_register(&max77696_clk_driver);
}
module_init(max77696_clk_driver_init);

static __exit void max77696_clk_driver_exit (void)
{
    platform_driver_unregister(&max77696_clk_driver);
}
module_exit(max77696_clk_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
