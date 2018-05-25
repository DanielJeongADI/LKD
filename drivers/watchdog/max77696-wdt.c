/*
 * MAX77696 System Watchdog Timer Driver
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

#include <linux/watchdog.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    MAX77696_DESC" System Watchdog Timer Driver"
#define DRIVER_NAME    MAX77696_WDT_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define WDT_TIMEOUT_MIN         2
#define WDT_TIMEOUT_MAX         128
#define WDT_TIMEOUT_DEF         128 /* in seconds */

#define WDT_CLEAR               0b01 /* GLBLCNFG4 WDTC[1:0] */

#define WDT_CARDRESET_ERCFLAGS  \
        (MAX77696_ERCFLAG_WDPMIC_FRSTRT | MAX77696_ERCFLAG_WDPMIC_FSHDN)

struct max77696_wdt {
    struct mutex                       lock;
    struct max77696_wdt_platform_data *pdata;
    struct max77696_core              *core;
    struct device                     *dev;
    struct kobject                    *kobj;
    const struct attribute_group      *attr_grp;

    struct watchdog_device            *wdd;

    unsigned long                      ping_interval;
    struct delayed_work                ping_work;
};

#define __lock(wdt)                 mutex_lock(&((wdt)->lock))
#define __unlock(wdt)               mutex_unlock(&((wdt)->lock))

static __always_inline int max77696_wdt_disable (struct max77696_wdt *wdt)
{
    int rc;

    rc = max77696_write_topsys_config(wdt->dev->parent, WDTEN, false);
    if (unlikely(rc)) {
        dev_err(wdt->dev, "failed to disable watchdog timer [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static __inline int max77696_wdt_enable (struct max77696_wdt *wdt,
    unsigned int timeout)
{
    u16 twd;
    int rc;

    rc = max77696_write_topsys_config(wdt->dev->parent, WDTEN, false);
    if (unlikely(rc)) {
        dev_err(wdt->dev, "failed to disable watchdog timer [%d]\n", rc);
        goto out;
    }

    twd = timeout > 64 ? 3 : timeout > 16 ? 2 : timeout > 2 ? 1 : 0;
    rc = max77696_write_topsys_config(wdt->dev->parent, TWD, twd);
    if (unlikely(rc)) {
        dev_err(wdt->dev, "failed to set watchdog timer period [%d]\n", rc);
        return rc;
    }

    rc = max77696_write_topsys_config(wdt->dev->parent, WDTEN, true);
    if (unlikely(rc)) {
        dev_err(wdt->dev, "failed to enable watchdog timer [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static __always_inline int max77696_wdt_clear (struct max77696_wdt *wdt)
{
    int rc;

    rc = max77696_write_topsys_config(wdt->dev->parent, WDTC, WDT_CLEAR);
    if (unlikely(rc)) {
        dev_err(wdt->dev, "failed to clear watchdog timer [%d]\n", rc);
        goto out;
    }

    dev_dbg(wdt->dev, "watchdog timer cleared\n");

out:
    return rc;
}

#define max77696_wdt_auto_ping_stop(_wdt) \
        cancel_delayed_work_sync(&(_wdt)->ping_work)

#define max77696_wdt_auto_ping_start(_wdt) \
        if (likely((_wdt)->ping_interval > 0)) {\
            if (likely(!delayed_work_pending(&(_wdt)->ping_work))) {\
                schedule_delayed_work(&(_wdt)->ping_work,\
                    (_wdt)->ping_interval);\
            }\
        }

static void max77696_wdt_ping_work (struct work_struct *work)
{
    struct max77696_wdt *wdt =
        container_of(work, struct max77696_wdt, ping_work.work);

    __lock(wdt);

    max77696_wdt_clear(wdt);
    max77696_wdt_auto_ping_start(wdt);

    __unlock(wdt);
    return;
}

static int max77696_wdt_op_start (struct watchdog_device *wdd)
{
    struct max77696_wdt *wdt = dev_get_drvdata(wdd->parent);
    int rc;

    __lock(wdt);

    rc = max77696_wdt_enable(wdt, wdd->timeout);
    if (unlikely(rc)) {
        goto out;
    }

    max77696_wdt_auto_ping_start(wdt);

out:
    __unlock(wdt);
    return rc;
}

static int max77696_wdt_op_stop (struct watchdog_device *wdd)
{
    struct max77696_wdt *wdt = dev_get_drvdata(wdd->parent);
    int rc;

    __lock(wdt);

    rc = max77696_wdt_disable(wdt);
    if (unlikely(rc)) {
        goto out;
    }

    max77696_wdt_auto_ping_stop(wdt);

out:
    __unlock(wdt);
    return rc;
}

static int max77696_wdt_op_ping (struct watchdog_device *wdd)
{
    struct max77696_wdt *wdt = dev_get_drvdata(wdd->parent);
    int rc;

    __lock(wdt);

    rc = max77696_wdt_clear(wdt);

    __unlock(wdt);
    return rc;
}

static int max77696_wdt_set_op_timeout (struct watchdog_device *wdd,
    unsigned int timeout)
{
    struct max77696_wdt *wdt = dev_get_drvdata(wdd->parent);
    int rc = 0;

    __lock(wdt);

    if (unlikely(!test_bit(WDOG_ACTIVE, &wdd->status))) {
        /* do nothing if timer not started */
        goto out;
    }

    rc = max77696_wdt_enable(wdt, timeout);

out:
    __unlock(wdt);
    return rc;
}

static const struct watchdog_ops max77696_wdt_ops = {
    .owner        = THIS_MODULE,
    .start        = max77696_wdt_op_start,
    .stop         = max77696_wdt_op_stop,
    .ping         = max77696_wdt_op_ping,
    .set_timeout  = max77696_wdt_set_op_timeout,
};

static const struct watchdog_info max77696_wdt_ident = {
    .identity    = DRIVER_NAME,
    .options     = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
};

static struct watchdog_device max77696_wdt_device = {
    .info        = &max77696_wdt_ident,
    .ops         = &max77696_wdt_ops,
    .min_timeout = WDT_TIMEOUT_MIN,
    .max_timeout = WDT_TIMEOUT_MAX,
    .timeout     = WDT_TIMEOUT_DEF,
};

static ssize_t max77696_wdt_status_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    struct watchdog_device *wdd = wdt->wdd;
    int rc;

    __lock(wdt);
    rc = (int)snprintf(buf, PAGE_SIZE, "boot status %08X\nstatus      %08X\n",
        wdd->bootstatus, (unsigned int)wdd->status);
    __unlock(wdt);

    return (ssize_t)rc;
}

static DEVICE_ATTR(wdt_status, S_IRUGO, max77696_wdt_status_show, NULL);

static ssize_t max77696_wdt_state_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    struct watchdog_device *wdd = wdt->wdd;
    int rc;

    __lock(wdt);
    rc = (int)snprintf(buf, PAGE_SIZE, "%s\n",
        test_bit(WDOG_ACTIVE, &wdd->status) ? "enabled" : "disabled");
    __unlock(wdt);

    return (ssize_t)rc;
}

static ssize_t max77696_wdt_state_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    struct watchdog_device *wdd = wdt->wdd;
    int rc;

    __lock(wdt);

    if (simple_strtoul(buf, NULL, 10)) {
        if (!test_bit(WDOG_ACTIVE, &wdd->status)) {
            rc = max77696_wdt_enable(wdt, wdd->timeout);
            if (unlikely(rc)) {
                goto out;
            }
            set_bit(WDOG_ACTIVE, &wdd->status);
        }
    } else {
        if (test_bit(WDOG_ACTIVE, &wdd->status)) {
            rc = max77696_wdt_disable(wdt);
            if (unlikely(rc)) {
                goto out;
            }
            clear_bit(WDOG_ACTIVE, &wdd->status);
        }
    }

out:
    __unlock(wdt);
    return (ssize_t)count;
}

static DEVICE_ATTR(wdt_state, S_IWUSR | S_IRUGO,
    max77696_wdt_state_show, max77696_wdt_state_store);

static ssize_t max77696_wdt_ping_interval_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    int rc;

    __lock(wdt);
    rc = (int)snprintf(buf, PAGE_SIZE, "%u sec\n",
        (unsigned int)DIV_ROUND_UP(jiffies_to_msecs(wdt->ping_interval), 1000));
    __unlock(wdt);

    return (ssize_t)rc;
}

static ssize_t max77696_wdt_ping_interval_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    unsigned int val;

    __lock(wdt);

    max77696_wdt_auto_ping_stop(wdt);

    val                = (unsigned int)simple_strtoul(buf, NULL, 10);
    wdt->ping_interval = msecs_to_jiffies(val * 1000);

    max77696_wdt_clear(wdt);
    max77696_wdt_auto_ping_start(wdt);

    __unlock(wdt);
    return (ssize_t)count;
}

static DEVICE_ATTR(wdt_ping_interval, S_IWUSR | S_IRUGO,
    max77696_wdt_ping_interval_show, max77696_wdt_ping_interval_store);

static ssize_t max77696_wdt_timeout_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    struct watchdog_device *wdd = wdt->wdd;
    int rc;

    __lock(wdt);
    rc = (int)snprintf(buf, PAGE_SIZE, "%u sec [%u sec ... %u sec]\n",
        wdd->timeout, wdd->min_timeout, wdd->max_timeout);
    __unlock(wdt);

    return (ssize_t)rc;
}

static ssize_t max77696_wdt_timeout_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    struct watchdog_device *wdd = wdt->wdd;
    unsigned int val;

    __lock(wdt);

    val = (unsigned int)simple_strtoul(buf, NULL, 10);

	if (likely(wdd->max_timeout)) {
	    if (unlikely(val < wdd->min_timeout || val > wdd->max_timeout)) {
	        dev_err(dev, "out of range\n");
			goto out;
        }
    }

    wdd->timeout = val;

    if (unlikely(!test_bit(WDOG_ACTIVE, &wdd->status))) {
        /* do nothing if timer not started */
        goto out;
    }

    max77696_wdt_enable(wdt, wdd->timeout);

out:
    __unlock(wdt);
    return (ssize_t)count;
}

static DEVICE_ATTR(wdt_timeout, S_IWUSR | S_IRUGO,
    max77696_wdt_timeout_show, max77696_wdt_timeout_store);

static struct attribute *max77696_wdt_attr[] = {
    &dev_attr_wdt_status.attr,
    &dev_attr_wdt_state.attr,
    &dev_attr_wdt_ping_interval.attr,
    &dev_attr_wdt_timeout.attr,
    NULL
};

static const struct attribute_group max77696_wdt_attr_group = {
    .attrs = max77696_wdt_attr,
};

static void *max77696_wdt_get_platdata (struct max77696_wdt *wdt)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_wdt_platform_data *pdata;
    struct device *dev = wdt->dev;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*pdata));
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->timeout_sec = 128;
    of_property_u32(np, "timeout_sec", &pdata->timeout_sec);

    pdata->ping_interval_sec = 60;
    of_property_u32(np, "ping_interval_sec", &pdata->ping_interval_sec);
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "TIMEOUT", "%u sec", pdata->timeout_sec);
    __prop_printk(dev, "PING INTERVAL", "%u sec", pdata->ping_interval_sec);

out:
    return pdata;
}

static __always_inline void max77696_wdt_destroy (struct max77696_wdt *wdt)
{
    struct device *dev = wdt->dev;

    if (likely(wdt->attr_grp)) {
        sysfs_remove_group(wdt->kobj, wdt->attr_grp);
    }

    if (likely(wdt->wdd)) {
        watchdog_unregister_device(wdt->wdd);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(wdt->pdata)) {
        devm_kfree(dev, wdt->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&wdt->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, wdt);
}

static int __devinit max77696_wdt_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_wdt *wdt;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
    if (unlikely(!wdt)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*wdt));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, wdt);

    mutex_init(&wdt->lock);
    wdt->core  = core;
    wdt->dev   = dev;
    wdt->kobj  = &dev->kobj;

    wdt->pdata = max77696_wdt_get_platdata(wdt);
    if (unlikely(IS_ERR(wdt->pdata))) {
        rc = PTR_ERR(wdt->pdata);
        wdt->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    wdt->ping_interval = msecs_to_jiffies(wdt->pdata->ping_interval_sec * 1000);
    INIT_DELAYED_WORK(&wdt->ping_work, max77696_wdt_ping_work);

    wdt->wdd          = &max77696_wdt_device;
    wdt->wdd->parent  = dev;
    wdt->wdd->timeout =
        (wdt->pdata->timeout_sec ? wdt->pdata->timeout_sec : WDT_TIMEOUT_DEF);

    /* Translate MAX77696 recorded events to the boot status of watchdog core */
    rc = max77696_test_ercflag(dev->parent, WDT_CARDRESET_ERCFLAGS);
    if (rc > 0) {
        dev_dbg(dev, "previously reset CPU\n");
        wdt->wdd->bootstatus |= WDIOF_CARDRESET;
    }

    rc = watchdog_register_device(wdt->wdd);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register watchdog device [%d]\n", rc);
        wdt->wdd = NULL;
        goto abort;
    }

    /* Create max77696-wdt sysfs attributes */
    wdt->attr_grp = &max77696_wdt_attr_group;
    rc = sysfs_create_group(wdt->kobj, wdt->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        wdt->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_wdt_destroy(wdt);
    return rc;
}

static __devexit int max77696_wdt_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_wdt *wdt = dev_get_drvdata(dev);

    max77696_wdt_destroy(wdt);
    return 0;
}

static void max77696_wdt_shutdown (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    struct watchdog_device *wdd = wdt->wdd;

    max77696_wdt_op_stop(wdd);
}

#ifdef CONFIG_PM_SLEEP
static int max77696_wdt_suspend (struct device *dev)
{
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    struct watchdog_device *wdd = wdt->wdd;

    if (likely(test_bit(WDOG_ACTIVE, &wdd->status))) {
        max77696_wdt_op_stop(wdd);
    }

    return 0;
}

static int max77696_wdt_resume (struct device *dev)
{
    struct max77696_wdt *wdt = dev_get_drvdata(dev);
    struct watchdog_device *wdd = wdt->wdd;

    if (likely(test_bit(WDOG_ACTIVE, &wdd->status))) {
        max77696_wdt_op_start(wdd);
    }

    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_wdt_pm,
    max77696_wdt_suspend, max77696_wdt_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_wdt_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_wdt_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_wdt_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_wdt_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_wdt_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_wdt_probe,
    .remove                = __devexit_p(max77696_wdt_remove),
    .shutdown              = max77696_wdt_shutdown,
};

static __init int max77696_wdt_driver_init (void)
{
    return platform_driver_register(&max77696_wdt_driver);
}
module_init(max77696_wdt_driver_init);

static __exit void max77696_wdt_driver_exit (void)
{
    platform_driver_unregister(&max77696_wdt_driver);
}
module_exit(max77696_wdt_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
