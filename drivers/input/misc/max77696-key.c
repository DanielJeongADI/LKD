/*
 * MAX77696 Key Input Driver
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

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif /* CONFIG_HAS_WAKELOCK */
#include <linux/input.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    MAX77696_DESC" Key Input Driver"
#define DRIVER_NAME    MAX77696_KEY_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define KEY_RECOVER_REPETITION  0

#define KEY_NKEY                MAX77696_KEY_NUM_OF_KEYS
#define KEY_NIRQ                4 /* en0.down / en0.up / en0.1sec / mrwrn */

#define KEY_DOWN_VALUE          1
#define KEY_UP_VALUE            0

#define BTNB_STTS_PRESSED       0
#define BTNB_STTS_NOT_PRESSED   1
#define JIGONB_STTS_ASSERTED    0
#define JIGONB_STTS_DEASSERTED  1

struct max77696_key {
    struct mutex                       lock;
    struct max77696_key_platform_data *pdata;
    struct max77696_core              *core;
    struct device                     *dev;
    struct kobject                    *kobj;
    struct attribute_group            *attr_grp;

    struct input_dev                  *input_dev;

    int                                irq[KEY_NIRQ];
    bool                               pressed[KEY_NKEY];

#ifdef CONFIG_HAS_WAKELOCK
    struct wake_lock                   wakelock;
#endif /* CONFIG_HAS_WAKELOCK */
};

#define __lock(_me)       mutex_lock(&(_me)->lock)
#define __unlock(_me)     mutex_unlock(&(_me)->lock)

static __always_inline int max77696_key_setup (struct max77696_key *key)
{
    struct device *dev = key->dev;
    struct max77696_key_platform_data *pdata = key->pdata;
    int rc;
    u16 val;

    /* Device wakeup initialization */
    device_init_wakeup(dev, true);

    val = pdata->mrt_sec <  6 ? pdata->mrt_sec-2                        :
          pdata->mrt_sec < 12 ? DIV_ROUND_UP(pdata->mrt_sec-6, 2)+0b100 : 0b111;
    rc  = max77696_write_topsys_config(dev->parent, MRT, val);
    if (unlikely(rc)) {
        dev_err(dev, "failed to set MRT [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static __always_inline
void max77696_key_report (struct max77696_key *key, int id, int val)
{
    bool pressed = (val == KEY_DOWN_VALUE);
    unsigned int keycode = key->pdata->keycode[id];

    if (unlikely(keycode >= KEY_MAX)) {
        dev_warn(key->dev, "keycode[%d] not assigned\n", id);
        goto out;
    }

    if (unlikely(key->pressed[id] == pressed)) {
        if (!KEY_RECOVER_REPETITION) {
            goto out;
        }

        dev_dbg(key->dev, "key[%d] repeated %s\n", id,
            pressed ? "pressing" : "releasing");

        /* Recover key event sequence by reporting missed */

        dev_dbg(key->dev, "reporting key[%d] %s with code %Xh\n", id,
            pressed ? "up" : "down", keycode);

        input_report_key(key->input_dev, keycode,
            pressed ? KEY_UP_VALUE : KEY_DOWN_VALUE);
        input_sync(key->input_dev);
    }

    dev_dbg(key->dev, "reporting key[%d] %s with code %Xh\n", id,
        pressed ? "down" : "up", keycode);

    key->pressed[id] = pressed;

    input_report_key(key->input_dev, keycode, val);
    input_sync(key->input_dev);

out:
    return;
}

static irqreturn_t max77696_key_down_isr (int irq, void *data)
{
    struct max77696_key *key = data;

    max77696_key_report(key, MAX77696_KEY_EN0, KEY_DOWN_VALUE);

    return IRQ_HANDLED;
}

static irqreturn_t max77696_key_up_isr (int irq, void *data)
{
    struct max77696_key *key = data;

    max77696_key_report(key, MAX77696_KEY_EN0, KEY_UP_VALUE);

    return IRQ_HANDLED;
}

static irqreturn_t max77696_key_1sec_isr (int irq, void *data)
{
    struct max77696_key *key = data;

    max77696_key_report(key, MAX77696_KEY_1SEC, KEY_DOWN_VALUE);
    max77696_key_report(key, MAX77696_KEY_1SEC, KEY_UP_VALUE  );

    return IRQ_HANDLED;
}

static irqreturn_t max77696_key_mrwrn_isr (int irq, void *data)
{
    struct max77696_key *key = data;

    max77696_key_report(key, MAX77696_KEY_MRWRN, KEY_DOWN_VALUE);
    max77696_key_report(key, MAX77696_KEY_MRWRN, KEY_UP_VALUE  );

    return IRQ_HANDLED;
}

struct max77696_key_irq_info {
    char          *name;
    unsigned int   topsys_irq;
    irq_handler_t  isr;
    int            key_id;
};

static struct max77696_key_irq_info max77696_key_irq_info[KEY_NIRQ] = {
    #define KEY_IRQ_INFO(_name, _topsys_irq, _isr, _key) \
            {\
                .name       = DRIVER_NAME"."_name,\
                .topsys_irq = MAX77696_TOPSYS_IRQ_##_topsys_irq,\
                .isr        = _isr,\
                .key_id     = MAX77696_KEY_##_key,\
            }
    KEY_IRQ_INFO("en0.down", EN0_RISING , max77696_key_down_isr , EN0  ),
    KEY_IRQ_INFO("en0.up"  , EN0_FALLING, max77696_key_up_isr   , EN0  ),
    KEY_IRQ_INFO("en0.1sec", EN0_1SEC   , max77696_key_1sec_isr , 1SEC ),
    KEY_IRQ_INFO("mrwrn"   , MR_WARNING , max77696_key_mrwrn_isr, MRWRN),
};

static void *max77696_key_get_platdata (struct max77696_key *key)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_key_platform_data *pdata;
    struct device *dev = key->dev;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*pdata));
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->mrt_sec = 12;
    of_property_u32(np, "mrt_sec", &pdata->mrt_sec);

    pdata->keycode[MAX77696_KEY_EN0] = KEY_MAX;
    of_property_u32(np,
        "keycode_en0", &pdata->keycode[MAX77696_KEY_EN0]);

    pdata->keycode[MAX77696_KEY_1SEC] = KEY_MAX;
    of_property_u32(np,
        "keycode_1sec", &pdata->keycode[MAX77696_KEY_1SEC]);

    pdata->keycode[MAX77696_KEY_MRWRN] = KEY_MAX;
    of_property_u32(np,
        "keycode_mrwrn", &pdata->keycode[MAX77696_KEY_MRWRN]);
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "MR TIME", "%u sec", pdata->mrt_sec);
    __prop_printk(dev,
        "KEYCODE EN0"  , "%u", pdata->keycode[MAX77696_KEY_EN0]  );
    __prop_printk(dev,
        "KEYCODE 1SEC" , "%u", pdata->keycode[MAX77696_KEY_1SEC] );
    __prop_printk(dev,
        "KEYCODE MRWRN", "%u", pdata->keycode[MAX77696_KEY_MRWRN]);

out:
    return pdata;
}

static __always_inline void max77696_key_destroy (struct max77696_key *key)
{
    struct device *dev = key->dev;
    int i;

    for (i = 0; i < KEY_NIRQ; i++) {
        if (likely(key->irq[i] > 0)) {
            devm_free_irq(dev, key->irq[i], key);
            key->irq[i] = 0;
        }
    }

    if (likely(key->input_dev)) {
        input_unregister_device(key->input_dev);
        input_free_device(key->input_dev);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(key->pdata)) {
        devm_kfree(dev, key->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

#ifdef CONFIG_HAS_WAKELOCK
    wake_lock_destroy(&key->wakelock);
#endif /* CONFIG_HAS_WAKELOCK */

    mutex_destroy(&key->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, key);
}

static __devinit int max77696_key_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_key *key;
    int i, rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    key = devm_kzalloc(dev, sizeof(*key), GFP_KERNEL);
    if (unlikely(!key)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*key));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, key);

    mutex_init(&key->lock);
    key->core  = core;
    key->dev   = dev;
    key->kobj  = &dev->kobj;

#ifdef CONFIG_HAS_WAKELOCK
    wake_lock_init(&key->wakelock, WAKE_LOCK_SUSPEND, DRIVER_NAME);
#endif /* CONFIG_HAS_WAKELOCK */

    key->pdata = max77696_key_get_platdata(key);
    if (unlikely(IS_ERR(key->pdata))) {
        rc = PTR_ERR(key->pdata);
        key->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Device setup */
    max77696_key_setup(key);

    key->input_dev = input_allocate_device();
    if (unlikely(!key->input_dev)) {
        dev_err(dev, "failed to allocate input device\n");
        rc = -ENOMEM;
        goto abort;
    }

    for (i = 0; i < KEY_NKEY; i++) {
        if (likely(key->pdata->keycode[i] < KEY_MAX)) {
            input_set_capability(key->input_dev,
                EV_KEY, key->pdata->keycode[i]);
        }
    }

    key->input_dev->name       = DRIVER_NAME;
    key->input_dev->phys       = DRIVER_NAME"/input0";
    key->input_dev->dev.parent = dev;

    rc = input_register_device(key->input_dev);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register input_dev device [%d]\n", rc);
        input_free_device(key->input_dev);
        key->input_dev = NULL;
        goto abort;
    }

    for (i = 0; i < KEY_NIRQ; i++) {
        int key_id = max77696_key_irq_info[i].key_id;

        if ((unsigned)key_id < KEY_NKEY) {
            if (key->pdata->keycode[key_id] >= KEY_MAX) {
                /* Do not use irq when reporting keycode not assigned. */
                continue;
            }
        }

        key->irq[i] = __max77696_get_topsys_irq(dev->parent,
            max77696_key_irq_info[i].topsys_irq);
        BUG_ON(key->irq[i] <= 0);

        rc = devm_request_threaded_irq(dev, (unsigned int)key->irq[i], NULL,
            max77696_key_irq_info[i].isr, IRQF_ONESHOT,
            max77696_key_irq_info[i].name, key);
        if (unlikely(rc)) {
            dev_err(dev, "failed to request IRQ(%d) for %s [%d]\n",
                key->irq[i], max77696_key_irq_info[i].name, rc);
            key->irq[i] = 0;
            goto abort;
        }

        dev_dbg(dev, "IRQ(%d) %s requested\n", key->irq[i],
            max77696_key_irq_info[i].name);

        enable_irq_wake(key->irq[i]);
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_key_destroy(key);
    return rc;
}

static __devexit int max77696_key_remove (struct platform_device *pdev)
{
    return 0;
}

static void max77696_key_shutdown (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_key *key = dev_get_drvdata(dev);

    max77696_key_destroy(key);
}

#ifdef CONFIG_PM_SLEEP
static int max77696_key_suspend (struct device *dev)
{
    return 0;
}

static int max77696_key_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_key_pm,
    max77696_key_suspend, max77696_key_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_key_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_key_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_key_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_key_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_key_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_key_probe,
    .remove                = __devexit_p(max77696_key_remove),
    .shutdown              = max77696_key_shutdown,
};

static __init int max77696_key_driver_init (void)
{
    return platform_driver_register(&max77696_key_driver);
}
module_init(max77696_key_driver_init);

static __exit void max77696_key_driver_exit (void)
{
    platform_driver_unregister(&max77696_key_driver);
}
module_exit(max77696_key_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
