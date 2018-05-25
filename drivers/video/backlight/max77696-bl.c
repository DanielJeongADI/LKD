/*
 * MAX77696 BL Driver
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

#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    MAX77696_DESC" BL Driver"
#define DRIVER_NAME    MAX77696_WLED_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define BL_MAX_BRIGHTNESS            0xFFF

#define LEDBST_CNTRL1                0x6C
#define LEDBST_CNTRL1_LED1EN         BIT (7)
#define LEDBST_CNTRL1_LED2EN         BIT (6)
#define LEDBST_CNTRL1_LEDPWM1EN      BIT (4)
#define LEDBST_CNTRL1_FPWM           BIT (3)
#define LEDBST_CNTRL1_LEDFOSC        BIT (2)

#define LED1CURRENT_1                0x6D
#define LED1CURRENT_2                0x6E

#if defined(CONFIG_MAX77796)
#define LED2CURRENT_1                0xB7
#define LED2CURRENT_2                0xB8
#endif /* CONFIG_MAX77796 */

#define LEDBST_INT                   0x6F
#define LEDBST_INTM                  0x70

struct max77696_bl {
    struct mutex                      lock;
    struct max77696_bl_platform_data *pdata;
    struct max77696_core             *core;
    struct max77696_io               *io;
    struct device                    *dev;
    struct kobject                   *kobj;
    const struct attribute_group     *attr_grp;

    struct backlight_device          *bl_dev;
    int                               brightness;
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

static __inline int max77696_bl_enable (struct max77696_bl *bl, bool en)
{
    max77696_write_reg_bit(bl->io, LEDBST_CNTRL1, LED1EN, !!en);
#if defined(CONFIG_MAX77796)
    max77696_write_reg_bit(bl->io, LEDBST_CNTRL1, LED2EN, !!en);
#endif /* CONFIG_MAX77796 */
    return 0;
}

static __inline int max77696_bl_set (struct max77696_bl *bl, int brightness)
{
    u8 iled1, iled2;

    brightness = min_t(int, BL_MAX_BRIGHTNESS, brightness);
    brightness = max_t(int,                 0, brightness);

    if (unlikely(bl->brightness == brightness)) {
        goto out;
    }

    if (unlikely(brightness == 0)) {
        max77696_bl_enable(bl, false);
        goto out;
    }

    /* LED1CURRENT_1 (MSB: BIT11 ~ BIT4), LED1CURRENT_2 (LSB: BIT3 ~ BIT0) */

    iled1 = (u8)(brightness >> 4);
    iled2 = (u8)(brightness & 0xf);

    max77696_write(bl->io, LED1CURRENT_2, iled2);
#if defined(CONFIG_MAX77796)
    max77696_write(bl->io, LED2CURRENT_2, iled2);
#endif /* CONFIG_MAX77796 */
    max77696_write(bl->io, LED1CURRENT_1, iled1);
#if defined(CONFIG_MAX77796)
    max77696_write(bl->io, LED2CURRENT_1, iled1);
#endif /* CONFIG_MAX77796 */

    max77696_bl_enable(bl, true);

    bl->brightness = brightness;

out:
    return 0;
}

static int max77696_bl_op_update_status (struct backlight_device *bd)
{
    struct max77696_bl *bl = bl_get_data(bd);
    int fb_blank = max(bd->props.power, bd->props.fb_blank);
    int brightness = bd->props.brightness;
    int rc;

    __lock(bl);

    if (likely(brightness > 0)) {
        if (unlikely((bd->props.state & BL_CORE_SUSPENDED) != 0)) {
            brightness = BL_MAX_BRIGHTNESS;
        } else if (unlikely(fb_blank != FB_BLANK_UNBLANK)) {
            brightness = BL_MAX_BRIGHTNESS;
        }
    }

    rc = max77696_bl_set(bl, brightness);

    __unlock(bl);
    return rc;
}

static int max77696_bl_op_get_brightness (struct backlight_device *bd)
{
    struct max77696_bl *bl = bl_get_data(bd);
    int rc;

    __lock(bl);

    /* Should we read real values from a device ??? */
    rc = bl->brightness;

    __unlock(bl);
    return rc;
}

static const struct backlight_ops max77696_bl_ops = {
    .options        = BL_CORE_SUSPENDRESUME,
    .update_status  = max77696_bl_op_update_status,
    .get_brightness = max77696_bl_op_get_brightness,
};

static const struct backlight_properties max77696_bl_props = {
    .max_brightness = BL_MAX_BRIGHTNESS,
    .type           = BACKLIGHT_RAW,
};

#define BL_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_bl_attr[] = {
    NULL
};

static const struct attribute_group max77696_bl_attr_group = {
    .attrs = max77696_bl_attr,
};

static void *max77696_bl_get_platdata (struct max77696_bl *bl)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_bl_platform_data *pdata;
    struct device *dev = bl->dev;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*pdata));
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->boot_brightness = BL_MAX_BRIGHTNESS;
    of_property_u32(np, "boot_brightness", &pdata->boot_brightness);
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "BOOT BRIGHTNESS", "%u", pdata->boot_brightness);

out:
    return pdata;
}

static __always_inline void max77696_bl_destroy (struct max77696_bl *bl)
{
    struct device *dev = bl->dev;

    if (likely(bl->attr_grp)) {
        sysfs_remove_group(bl->kobj, bl->attr_grp);
    }

    if (likely(bl->bl_dev)) {
        backlight_device_unregister(bl->bl_dev);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(bl->pdata)) {
        devm_kfree(dev, bl->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&bl->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, bl);
}

static __devinit int max77696_bl_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_bl *bl;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    bl = devm_kzalloc(dev, sizeof(*bl), GFP_KERNEL);
    if (unlikely(!bl)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*bl));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, bl);

    mutex_init(&bl->lock);
    bl->core  = core;
    bl->io    = max77696_get_block_io(dev->parent, WLED);
    bl->dev   = dev;
    bl->kobj  = &dev->kobj;

    bl->pdata = max77696_bl_get_platdata(bl);
    if (unlikely(IS_ERR(bl->pdata))) {
        rc = PTR_ERR(bl->pdata);
        bl->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    bl->bl_dev = backlight_device_register(DRIVER_NAME, bl->dev, bl,
        &max77696_bl_ops, &max77696_bl_props);
    if (unlikely(IS_ERR(bl->bl_dev))) {
        rc = PTR_ERR(bl->bl_dev);
        dev_err(bl->dev, "failed to register backlight device [%d]\n", rc);
        bl->bl_dev = NULL;
        goto abort;
    }

    /* Create max77696-bl sysfs attributes */
    bl->attr_grp = &max77696_bl_attr_group;
    rc = sysfs_create_group(bl->kobj, bl->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        bl->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);

    /* Update current */
    bl->bl_dev->props.brightness =
        min_t(int, bl->pdata->boot_brightness, BL_MAX_BRIGHTNESS);
    bl->brightness               = -1; /* invalidate */
    backlight_update_status(bl->bl_dev);

    return 0;

abort:
    max77696_bl_destroy(bl);
    return rc;
}

static __devexit int max77696_bl_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_bl *bl = dev_get_drvdata(dev);

    max77696_bl_destroy(bl);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_bl_suspend (struct device *dev)
{
    return 0;
}

static int max77696_bl_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_bl_pm,
    max77696_bl_suspend, max77696_bl_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_bl_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_bl_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_bl_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_bl_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_bl_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_bl_probe,
    .remove                = __devexit_p(max77696_bl_remove),
};

static __init int max77696_bl_driver_init (void)
{
    return platform_driver_register(&max77696_bl_driver);
}
module_init(max77696_bl_driver_init);

static __exit void max77696_bl_driver_exit (void)
{
    platform_driver_unregister(&max77696_bl_driver);
}
module_exit(max77696_bl_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
