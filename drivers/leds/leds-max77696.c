/*
 * MAX77696 LED Indicators Driver
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

#define DRIVER_DESC    MAX77696_DESC" LED Indicators Driver"
#define DRIVER_NAME    MAX77696_LED_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define LED_NLED                  MAX77696_NUM_OF_LEDS
#define LED_DEFAULT_NAME          MAX77696_NAME"-led"
#define LED_MAX_BRIGHTNESS        0x7F
#define LED_WORK_DELAY            msecs_to_jiffies(0)

#define LED_FLASHD_OFF            0x0 /* off */
#define LED_FLASHP_OFF            0xF /* on indefinitely */

#define LED_FLASHD_ON             0xF /* 640ms */
#define LED_FLASHP_ON             0x0 /* 640ms */

#define LEDIND_STAT1LEDSET        0x71
#define LEDIND_STAT2LEDSET        0x72
#define LEDIND_STAT1FLASH         0x73
#define LEDIND_STAT2FLASH         0x74

#define LEDIND_STATLEDSET(_led)   (LEDIND_STAT1LEDSET+(_led))
#define LEDIND_STATFLASH(_led)    (LEDIND_STAT1FLASH +(_led))

#define LEDIND_STATLEDSET_LEDEN   BIT (7)
#define LEDIND_STATLEDSET_LEDSET  BITS(6,0)

#define LEDIND_STATFLASH_FLASHD   BITS(7,4)
#define LEDIND_STATFLASH_FLASHP   BITS(3,0)

#define LED_NLED_BITMAP_SZ        BITS_TO_LONGS(LED_NLED)

struct max77696_leds;
struct max77696_led {
    struct max77696_leds            *parent;
    struct max77696_io              *io;
    int                              led_id;

    struct led_classdev              led_cdev;

    struct device                   *dev;
    struct kobject                  *kobj;
    const struct attribute_group    *attr_grp;

    struct delayed_work              work;
    struct max77696_led_cfg          led_cfg;
};

struct max77696_leds {
    struct mutex                        lock;
    struct max77696_leds_platform_data *pdata;
    struct max77696_core               *core;
    struct max77696_io                 *io;
    struct device                      *dev;
    struct kobject                     *kobj;
    const struct attribute_group       *attr_grp;

    struct max77696_led                 leds[LED_NLED];
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

static int max77696_led_write_config (struct max77696_led *led)
{
    struct max77696_led_cfg *cfg = &led->led_cfg;
    u16 ledset_val, flash_val, brt, flashd, flashp;
    u32 blinkd, blinkp;
    int rc;

    if (unlikely(!cfg)) {
        rc = -EINVAL;
        goto out;
    }

    blinkd = cfg->delay_on;
    blinkp = cfg->delay_on + cfg->delay_off;

    if (blinkp == 0) {
        /* turn off */
        flashd = LED_FLASHD_OFF;
        flashp = LED_FLASHP_OFF;
    } else if (blinkp == blinkd) {
        /* deactivate blinking */
        flashd = LED_FLASHD_ON;
        flashp = LED_FLASHP_ON;
    } else {
        flashd = blinkd < 320 ? DIV_ROUND_UP(blinkd    , 32)     :
                 blinkd < 640 ? DIV_ROUND_UP(blinkd-320, 64)+0xA :
                 0xF;
        flashp = blinkp <  640 ? 0x0 :
                 blinkp < 3840 ? DIV_ROUND_UP(blinkp- 640,  320)+0x0 :
                 blinkp < 5120 ? DIV_ROUND_UP(blinkp-3840,  640)+0xA :
                 blinkp < 7680 ? DIV_ROUND_UP(blinkp-5120, 1280)+0xC :
                 0xF;
    }

    if (unlikely(cfg->brightness == LED_OFF)) {
        brt = 0;
    } else {
        rc  = (int)cfg->brightness;
        brt = (u16)DIV_ROUND_UP(rc * LED_MAX_BRIGHTNESS, LED_FULL);
    }

    ledset_val = 0;
    ledset_val = BITS_SET(ledset_val, LEDIND_STATLEDSET_LEDEN , cfg->manual);
    ledset_val = BITS_SET(ledset_val, LEDIND_STATLEDSET_LEDSET, brt        );

    flash_val = 0;
    flash_val = BITS_SET(flash_val, LEDIND_STATFLASH_FLASHD, flashd);
    flash_val = BITS_SET(flash_val, LEDIND_STATFLASH_FLASHP, flashp);

    dev_vdbg(led->dev, "LEDIND_STAT 0x%02X LEDSET 0x%04X\n",
        LEDIND_STATLEDSET(led->led_id), ledset_val);

    dev_vdbg(led->dev, "LEDIND_STAT 0x%02X FLASH  0x%04X\n",
        LEDIND_STATFLASH(led->led_id), flash_val);

    rc = max77696_write(led->io, LEDIND_STATLEDSET(led->led_id), ledset_val);
    if (unlikely(rc)) {
        dev_err(led->dev,
            "LEDIND_STATLEDSET%d write error [%d]\n", led->led_id+1, rc);
        goto out;
    }

    rc = max77696_write(led->io, LEDIND_STATFLASH(led->led_id), flash_val);
    if (unlikely(rc)) {
        dev_err(led->dev,
            "LEDIND_STATFLASH%d write error [%d]\n", led->led_id+1, rc);
        goto out;
    }

out:
    return rc;
}

static __always_inline
void max77696_led_schedule_work (struct max77696_led *led)
{
    if (likely(!delayed_work_pending(&led->work))) {
        schedule_delayed_work(&led->work, LED_WORK_DELAY);
    }
}

static ssize_t max77696_led_ledset_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct max77696_led *led =
        container_of(led_cdev, struct max77696_led, led_cdev);
    u16 ledset;
    int rc;

    __lock(led->parent);

    rc = max77696_read(led->io, LEDIND_STATLEDSET(led->led_id), &ledset);
    if (unlikely(rc)) {
        goto out;
    }

    rc = (int)snprintf(buf, PAGE_SIZE, "0x%04X\n", ledset);

out:
    __unlock(led->parent);
    return (ssize_t)rc;
}

static ssize_t max77696_led_ledset_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct max77696_led *led =
        container_of(led_cdev, struct max77696_led, led_cdev);
    u16 ledset;

    __lock(led->parent);

    ledset = (u16)simple_strtoul(buf, NULL, 16);
    max77696_write(led->io, LEDIND_STATLEDSET(led->led_id), ledset);

    __unlock(led->parent);
    return (ssize_t)count;
}

static ssize_t max77696_led_statflash_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct max77696_led *led =
        container_of(led_cdev, struct max77696_led, led_cdev);
    u16 statflash;
    int rc;

    __lock(led->parent);

    rc = max77696_read(led->io, LEDIND_STATFLASH(led->led_id), &statflash);
    if (unlikely(rc)) {
        goto out;
    }

    rc = (int)snprintf(buf, PAGE_SIZE, "0x%04X\n", statflash);

out:
    __unlock(led->parent);
    return (ssize_t)rc;
}

static ssize_t max77696_led_statflash_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct max77696_led *led =
        container_of(led_cdev, struct max77696_led, led_cdev);
    u16 statflash;

    __lock(led->parent);

    statflash = (u16)simple_strtoul(buf, NULL, 16);
    max77696_write(led->io, LEDIND_STATFLASH(led->led_id), statflash);

    __unlock(led->parent);
    return (ssize_t)count;
}

static ssize_t max77696_led_manual_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct max77696_led *led =
        container_of(led_cdev, struct max77696_led, led_cdev);
    int rc;

    __lock(led->parent);

    rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", led->led_cfg.manual);

    __unlock(led->parent);
    return (ssize_t)rc;
}

static ssize_t max77696_led_manual_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct max77696_led *led =
        container_of(led_cdev, struct max77696_led, led_cdev);

    __lock(led->parent);

    led->led_cfg.manual = (bool)!!simple_strtoul(buf, NULL, 10);
    max77696_led_schedule_work(led);

    __unlock(led->parent);
    return (ssize_t)count;
}

#define DEFINE_LED_DEV_ATTR_RW(_name) \
static DEVICE_ATTR(_name, S_IWUSR | S_IRUGO,\
    max77696_led_##_name##_show, max77696_led_##_name##_store)

DEFINE_LED_DEV_ATTR_RW(ledset);
DEFINE_LED_DEV_ATTR_RW(statflash);
DEFINE_LED_DEV_ATTR_RW(manual);

#define LED_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_led_attr[] = {
    LED_DEV_ATTR(ledset),
    LED_DEV_ATTR(statflash),
    LED_DEV_ATTR(manual),
    NULL
};

static const struct attribute_group max77696_led_attr_group = {
    .attrs = max77696_led_attr,
};

static void max77696_led_op_brightness_set (struct led_classdev *led_cdev,
    enum led_brightness value)
{
    struct max77696_led *led =
        container_of(led_cdev, struct max77696_led, led_cdev);

    __lock(led->parent);

    led->led_cfg.brightness = value;
    led->led_cfg.delay_on   = 1;
    led->led_cfg.delay_off  = 0;
    max77696_led_schedule_work(led);

    __unlock(led->parent);
}

static int max77696_led_op_blink_set (struct led_classdev *led_cdev,
		unsigned long *delay_on, unsigned long *delay_off)
{
    struct max77696_led *led =
        container_of(led_cdev, struct max77696_led, led_cdev);

    __lock(led->parent);

    led->led_cfg.delay_on  = !delay_on  || *delay_on  == 0 ? 0 :
        (u32)min(0xFFFFUL, *delay_on );
    led->led_cfg.delay_off = !delay_off || *delay_off == 0 ? 0 :
        (u32)min(0xFFFFUL, *delay_off);
    max77696_led_schedule_work(led);

    __unlock(led->parent);
    return 0;
}

static void max77696_led_work (struct work_struct *work)
{
    struct max77696_led *led =
        container_of(work, struct max77696_led, work.work);
    int rc;

    __lock(led->parent);

    rc = max77696_led_write_config(led);
    if (unlikely(rc)) {
        dev_err(led->dev, "configuration failed [%d]\n", rc);
        goto out;
    }

out:
    __unlock(led->parent);
    return;
}

static __always_inline void max77696_led_destory (struct max77696_led *led)
{
    cancel_delayed_work_sync(&led->work);

    if (likely(led->attr_grp)) {
        sysfs_remove_group(led->kobj, led->attr_grp);
        led->attr_grp = NULL;
    }

    if (likely(led->led_cdev.dev)) {
        led_classdev_unregister(&led->led_cdev);
        led->led_cdev.dev = NULL;
    }

    if (likely(led->led_cdev.name)) {
        kfree(led->led_cdev.name);
        led->led_cdev.name = NULL;
    }
}

static __always_inline int max77696_led_create (struct max77696_led *led,
    struct max77696_led_init_data *init_data)
{
    struct device *parent_dev = led->parent->dev;
    int rc;

    led->led_cdev.brightness_set  = max77696_led_op_brightness_set;
    led->led_cdev.blink_set       = max77696_led_op_blink_set;
    led->led_cdev.max_brightness  = LED_FULL;
    led->led_cdev.brightness      = LED_OFF;

    if (likely(init_data && init_data->info.name)) {
        led->led_cdev.name            =
            kasprintf(GFP_KERNEL, init_data->info.name);
        led->led_cdev.flags           = init_data->info.flags;
        led->led_cdev.default_trigger = init_data->info.default_trigger;
    } else {
        led->led_cdev.name            =
            kasprintf(GFP_KERNEL, LED_DEFAULT_NAME".%d", led->led_id);
        led->led_cdev.flags           = 0;
        led->led_cdev.default_trigger = NULL;
    }

    if (unlikely(!led->led_cdev.name)) {
        dev_err(parent_dev, "failed to set LED.%d name\n", led->led_id);
        rc = -ENOMEM;
        goto abort;
    }

    rc = led_classdev_register(parent_dev, &led->led_cdev);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to register LED.%d device [%d]\n", led->led_id, rc);
        led->led_cdev.dev = NULL;
        goto abort;
    }

    led->dev  = led->led_cdev.dev;
    led->kobj = &led->dev->kobj;
    INIT_DELAYED_WORK(&led->work, max77696_led_work);

    if (likely(init_data)) {
        memcpy(&led->led_cfg, &init_data->cfg, sizeof(led->led_cfg));

        rc = max77696_led_write_config(led);
        if (unlikely(rc)) {
            dev_err(led->dev, "failed to configure initially [%d]\n", rc);
            goto abort;
        }
    }

    led->attr_grp = &max77696_led_attr_group;
    rc = sysfs_create_group(led->kobj, led->attr_grp);
    if (unlikely(rc)) {
        dev_err(led->dev, "failed to create attribute group [%d]\n", rc);
        led->attr_grp = NULL;
        goto abort;
    }

    return 0;

abort:
    max77696_led_destory(led);
    return rc;
}

/******************************************************************************/

#define LEDS_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_leds_attr[] = {
    NULL
};

static const struct attribute_group max77696_leds_attr_group = {
    .attrs = max77696_leds_attr,
};

static void *max77696_leds_get_platdata (struct max77696_leds *leds)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_leds_platform_data *pdata;
    struct device *dev = leds->dev;
    int i;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;
    struct device_node *init_np;
    size_t sz;
    int num_of_init_data;

    num_of_init_data = of_get_child_count(np);

    sz = sizeof(*pdata) + num_of_init_data * sizeof(*pdata->init_data);
    pdata = devm_kzalloc(dev, sz, GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sz);
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->init_data = (void*)(pdata + 1);
    pdata->num_of_init_data = (size_t)num_of_init_data;

    i = 0;
    for_each_child_of_node(np, init_np) {
        u32 brightness = 0;

        of_property_u32(init_np, "reg",
            (u32*)&pdata->init_data[i].led_id);

		pdata->init_data[i].info.name            =
			of_get_property(init_np, "label", NULL);
		pdata->init_data[i].info.default_trigger =
			of_get_property(init_np, "linux,default-trigger", NULL);

        pdata->init_data[i].cfg.manual =
            of_property_read_bool(init_np, "manual");
        of_property_u32(init_np, "brightness", &brightness);
        pdata->init_data[i].cfg.brightness = (enum led_brightness)brightness;
        of_property_u32(init_np, "delay_on",
            &pdata->init_data[i].cfg.delay_on);
        of_property_u32(init_np, "delay_off",
            &pdata->init_data[i].cfg.delay_off);

        i++;
    }
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "INITDATA", "%zu", pdata->num_of_init_data);
    for (i = 0; i < pdata->num_of_init_data; i++) {
        __prop_printk(dev, "INITDATA", "[%d] LED         %d",
            i, pdata->init_data[i].led_id);
        __prop_printk(dev, "INITDATA", "[%d] NAME        %s",
            i, pdata->init_data[i].info.name ? : "null");
        __prop_printk(dev, "INITDATA", "[%d] TRIGGER     %s",
            i, pdata->init_data[i].info.default_trigger ? : "null");
        __prop_printk(dev, "INITDATA", "[%d] MANUAL      %s",
            i, pdata->init_data[i].cfg.manual ? "on" : "off");
        __prop_printk(dev, "INITDATA", "[%d] BRIGHTNESS  %d",
            i, pdata->init_data[i].cfg.brightness);
        __prop_printk(dev, "INITDATA", "[%d] DEALY ON    %d msec",
            i, pdata->init_data[i].cfg.delay_on);
        __prop_printk(dev, "INITDATA", "[%d] DEALY OFF   %d msec",
            i, pdata->init_data[i].cfg.delay_off);
    }

out:
    return pdata;
}

static __always_inline void max77696_leds_destroy (struct max77696_leds *leds)
{
    struct device *dev = leds->dev;
    int i;

    if (likely(leds->attr_grp)) {
        sysfs_remove_group(leds->kobj, leds->attr_grp);
    }

    for (i = 0; i < LED_NLED; i++) {
        max77696_led_destory(&leds->leds[i]);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(leds->pdata)) {
        devm_kfree(dev, leds->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&leds->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, leds);
}

static __devinit int max77696_leds_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_leds *leds;
    int i, j, rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    leds = devm_kzalloc(dev, sizeof(*leds), GFP_KERNEL);
    if (unlikely(!leds)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*leds));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, leds);

    mutex_init(&leds->lock);
    leds->core  = core;
    leds->io    = max77696_get_block_io(dev->parent, LED);
    leds->dev   = dev;
    leds->kobj  = &dev->kobj;

    leds->pdata = max77696_leds_get_platdata(leds);
    if (unlikely(IS_ERR(leds->pdata))) {
        rc = PTR_ERR(leds->pdata);
        leds->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    for (i = 0; i < LED_NLED; i++) {
        struct max77696_led *led = &leds->leds[i];
        struct max77696_led_init_data *init_data = NULL;

        for (j = 0; j < leds->pdata->num_of_init_data; j++) {
            if (leds->pdata->init_data[j].led_id == i) {
                init_data = &leds->pdata->init_data[j];
                break;
            }
        }

        led->parent = leds;
        led->io     = leds->io;
        led->led_id = i;

        max77696_led_create(led, init_data);
    }

    /* Create max77696-leds sysfs attributes */
    leds->attr_grp = &max77696_leds_attr_group;
    rc = sysfs_create_group(leds->kobj, leds->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        leds->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_leds_destroy(leds);
    return rc;
}

static __devexit int max77696_leds_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_leds *leds = dev_get_drvdata(dev);

    max77696_leds_destroy(leds);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_leds_suspend (struct device *dev)
{
    return 0;
}

static int max77696_leds_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_leds_pm,
    max77696_leds_suspend, max77696_leds_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_leds_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_leds_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_leds_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_leds_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_leds_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_leds_probe,
    .remove                = __devexit_p(max77696_leds_remove),
};

static __init int max77696_leds_driver_init (void)
{
    return platform_driver_register(&max77696_leds_driver);
}
module_init(max77696_leds_driver_init);

static __exit void max77696_leds_driver_exit (void)
{
    platform_driver_unregister(&max77696_leds_driver);
}
module_exit(max77696_leds_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
