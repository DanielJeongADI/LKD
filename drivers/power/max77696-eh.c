/*
 * MAX77696 Energy Harvester Driver
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
#include <linux/gpio.h>
#include <linux/i2c.h>

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>

#include <linux/power_supply.h>
#include <linux/mfd/max77696.h>

#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
#define DRIVER_DESC    MAX77696_DESC" Energy Harvester Driver"
#define DRIVER_NAME    MAX77696_EH_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define CHGB_RWC_INTERRUPT      0

#define CHGB_PSY_WORK_DELAY     (HZ/10)
#define CHGB_ACC_DET_GPIO_SEEK  msecs_to_jiffies(1000)

#define MBCCTRL1                0x36
#define MBCCTRL2                0x37

#define MBCCTRL3                0x38
#define MBCCTRL3_MBCCV          BITS(3,0)

#define MBCCTRL4                0x39
#define MBCCTRL4_MBCICHFC_EN    BIT (4)
#define MBCCTRL4_MBCICHFC       BITS(3,0)

#define MBCCTRL7                0x3A
#define MBCCTRL8                0x3B

#define CHGINB_CTL              0x3C
#define CHGINB_CTL_ACC_ILM      BIT (7)
#define CHGINB_CTL_BCHG_ENB     BIT (6)
#define CHGINB_CTL_ACC_EN       BIT (5)

#define CHGINB_STAT             0x3D
#define CHGINB_STAT_ACCFLTINTS  BIT (7)
#define CHGINB_STAT_ACCONINTS   BIT (6)
#define CHGINB_STAT_BCHGPOK     BIT (5)
#define CHGINB_STAT_BCHGON      BIT (4)

#define CHGINB_INT              0x3E
#define CHGINB_INTM             0x3F

#define CHGB_INT_ACCFLT         BIT (7) /* Accessory OL interrupt */
#define CHGB_INT_ACCON          BIT (6) /* Accessory ON interrupt */
#define CHGB_INT_BCHGPOK        BIT (5) /* CHGINB power input OK */
#define CHGB_INT_ACCLD          BIT (4) /* Accessory Loaded > 2mA interrupt */

struct max77696_eh {
    struct mutex                        lock;
    struct max77696_eh_platform_data   *pdata;
    struct max77696_core               *core;
    struct max77696_io                 *io;
    struct device                      *dev;
    struct kobject                     *kobj;
    const struct attribute_group       *attr_grp;

    int                                 irq;
    u16                                 irq_unmask;

    int                                 acc_det_gpio;
    int                                 acc_det_gpio_assert;
    unsigned long                       acc_det_debounce;

    struct power_supply                *psy;
    struct delayed_work                 psy_work;
    struct delayed_work                 acc_work;

    bool                                destroying;
    bool                                chg_present;
    bool                                chg_enable;
    bool                                acc_present;
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

static __always_inline
void max77696_eh_enable_irq (struct max77696_eh *eh, u16 irq_bits)
{
    u16 ehintm;
    int rc;

    if (unlikely((eh->irq_unmask & irq_bits) == irq_bits)) {
        /* already unmasked or null bit */
        return;
    }

    if (unlikely(!eh->irq_unmask)) {
        enable_irq(eh->irq);
        enable_irq_wake(eh->irq);
    }

    /* set enabled flag */
    eh->irq_unmask |= irq_bits;

    /* unmask irq */
    ehintm = ~eh->irq_unmask;

    rc = max77696_write(eh->io, CHGINB_INTM, ehintm);
    dev_dbg(eh->dev, "CHGINB_INTM written 0x%04X [%d]\n", ehintm, rc);

    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_INTM write error [%d]\n", rc);
    }
}

static __always_inline
void max77696_eh_disable_irq (struct max77696_eh *eh, u16 irq_bits)
{
    u16 ehintm;
    int rc;

    if (unlikely((eh->irq_unmask & irq_bits) == 0)) {
        /* already masked or null bit */
        return;
    }

    /* clear enabled flag */
    eh->irq_unmask &= ~irq_bits;

    if (unlikely(!eh->irq_unmask)) {
        disable_irq_wake(eh->irq);
        disable_irq(eh->irq);
    }

    /* mask irq */
    ehintm = ~eh->irq_unmask;

    rc = max77696_write(eh->io, CHGINB_INTM, ehintm);
    dev_dbg(eh->dev, "CHGINB_INTM written 0x%04X [%d]\n", ehintm, rc);

    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_INTM write error [%d]\n", rc);
    }
}

static __always_inline
u16 max77696_eh_read_irq (struct max77696_eh *eh)
{
    u16 ehint;
    int rc;

    rc = max77696_read(eh->io, CHGINB_INT, &ehint);
    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_INT read error [%d]\n", rc);
        return 0;
    }

    return ehint;
}

static __always_inline
void max77696_eh_ack_irq (struct max77696_eh *eh)
{
    if (CHGB_RWC_INTERRUPT) {
        max77696_write(eh->io, CHGINB_INT, ~0);
    }
}

/* Enable/Disable charger */

static int max77696_eh_chg_dis_set (struct max77696_eh* eh, int disable)
{
    int rc;

    rc = max77696_write_reg_bit(eh->io, CHGINB_CTL, BCHG_ENB, (u16)!!disable);
    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_CTL write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static int max77696_eh_chg_dis_get (struct max77696_eh* eh, int *disable)
{
    int rc;
    u16 chg_enb;

    rc = max77696_read_reg_bit(eh->io, CHGINB_CTL, BCHG_ENB, &chg_enb);
    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_CTL read error [%d]\n", rc);
        goto out;
    }

    *disable = (int)!!chg_enb;

out:
    return rc;
}

/* Set/read constant current level */

static int max77696_eh_mbcichfc_en_set (struct max77696_eh* eh, int enable)
{
    int rc;

    rc = max77696_write_reg_bit(eh->io, MBCCTRL4, MBCICHFC_EN, (u16)!!enable);
    if (unlikely(rc)) {
        dev_err(eh->dev, "MBCCTRL4 write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static int max77696_eh_mbcichfc_en_get (struct max77696_eh* eh, int *enable)
{
    int rc;
    u16 mbcichfc_en;

    rc = max77696_read_reg_bit(eh->io, MBCCTRL4, MBCICHFC_EN, &mbcichfc_en);
    if (unlikely(rc)) {
        dev_err(eh->dev, "MBCCTRL4 read error [%d]\n", rc);
        goto out;
    }

    *enable = (int)!!mbcichfc_en;

out:
    return rc;
}

static int max77696_eh_mbcichfc_set (struct max77696_eh* eh, int mA)
{
    int rc;
    u16 mbcichfc;

    if (mA > 900) {
        mbcichfc = 0xF;
    } else if (mA < 250) {
        mbcichfc = 0x0;
    } else {
        mbcichfc = (u16)DIV_ROUND_UP(mA - 200, 50);
    }

    rc = max77696_write_reg_bit(eh->io, MBCCTRL4, MBCICHFC, mbcichfc);
    if (unlikely(rc)) {
        dev_err(eh->dev, "MBCCTRL4 write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static int max77696_eh_mbcichfc_get (struct max77696_eh* eh, int *mA)
{
    int rc;
    u16 mbcichfc;

    rc = max77696_read_reg_bit(eh->io, MBCCTRL4, MBCICHFC, &mbcichfc);
    if (unlikely(rc)) {
        dev_err(eh->dev, "MBCCTRL4 read error [%d]\n", rc);
        goto out;
    }

    *mA = (int)mbcichfc * 50 + 200;

out:
    return rc;
}

/* Set/read constant voltage level */

static int max77696_eh_mbccv_set (struct max77696_eh* eh, int mV)
{
    int rc;
    u16 mbccv;

    if (mV > 4280) {
        mbccv = 0xF;
    } else if (mV < 4020) {
        mbccv = 0x0;
    } else {
        mbccv = (u16)DIV_ROUND_UP(mV - 4000, 20);
    }

    rc = max77696_write_reg_bit(eh->io, MBCCTRL3, MBCCV, mbccv);
    if (unlikely(rc)) {
        dev_err(eh->dev, "MBCCTRL3 write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static int max77696_eh_mbccv_get (struct max77696_eh* eh, int *mV)
{
    int rc;
    u16 mbccv;

    rc = max77696_read_reg_bit(eh->io, MBCCTRL3, MBCCV, &mbccv);
    if (unlikely(rc)) {
        dev_err(eh->dev, "MBCCTRL3 read error [%d]\n", rc);
        goto out;
    }

    *mV = (int)mbccv * 20 + 4000;

out:
    return rc;
}

/* Enable/Disable accessory */

static int max77696_eh_acc_en_set (struct max77696_eh* eh, int enable)
{
    int rc;

    rc = max77696_write_reg_bit(eh->io, CHGINB_CTL, ACC_EN, (u16)!!enable);
    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_CTL write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static int max77696_eh_acc_en_get (struct max77696_eh* eh, int *enable)
{
    int rc;
    u16 acc_en;

    rc = max77696_read_reg_bit(eh->io, CHGINB_CTL, ACC_EN, &acc_en);
    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_CTL read error [%d]\n", rc);
        goto out;
    }

    *enable = (int)!!acc_en;

out:
    return rc;
}

/* Set/Read accessory current */

static int max77696_eh_acc_ilm_set (struct max77696_eh* eh, int mA)
{
    int rc;

    rc = max77696_write_reg_bit(eh->io, CHGINB_CTL, ACC_ILM, (u16)(mA > 200));
    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_CTL write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static int max77696_eh_acc_ilm_get (struct max77696_eh* eh, int *mA)
{
    int rc;
    u16 acc_ilm;

    rc = max77696_read_reg_bit(eh->io, CHGINB_CTL, ACC_ILM, &acc_ilm);
    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_CTL read error [%d]\n", rc);
        goto out;
    }

    *mA = (int)(acc_ilm ? 650 : 200);

out:
    return rc;
}

#define DEFINE_MAX77696_EH_DEV_ATTR(_name, _unit) \
static ssize_t max77696_eh_##_name##_show (struct device *dev,\
    struct device_attribute *devattr, char *buf)\
{\
    struct max77696_eh *eh = dev_get_drvdata(dev);\
    int val = 0, rc;\
    __lock(eh);\
    rc = max77696_eh_##_name##_get(eh, &val);\
    if (unlikely(rc)) {\
        goto out;\
    }\
    rc = (int)snprintf(buf, PAGE_SIZE, "%d "_unit"\n", val);\
out:\
    __unlock(eh);\
    return (ssize_t)rc;\
}\
static ssize_t max77696_eh_##_name##_store (struct device *dev,\
    struct device_attribute *devattr, const char *buf, size_t count)\
{\
    struct max77696_eh *eh = dev_get_drvdata(dev);\
    int val, rc;\
    __lock(eh);\
    val = (int)simple_strtol(buf, NULL, 10);\
    rc = max77696_eh_##_name##_set(eh, val);\
    if (unlikely(rc)) {\
        goto out;\
    }\
out:\
    __unlock(eh);\
    return (ssize_t)count;\
}\
static DEVICE_ATTR(_name, S_IWUSR|S_IRUGO, max77696_eh_##_name##_show,\
    max77696_eh_##_name##_store)

DEFINE_MAX77696_EH_DEV_ATTR(chg_dis    , ""  );
DEFINE_MAX77696_EH_DEV_ATTR(mbcichfc_en, ""  );
DEFINE_MAX77696_EH_DEV_ATTR(mbcichfc   , "mA");
DEFINE_MAX77696_EH_DEV_ATTR(mbccv      , "mV");
DEFINE_MAX77696_EH_DEV_ATTR(acc_en     , ""  );
DEFINE_MAX77696_EH_DEV_ATTR(acc_ilm    , "mA");

static struct attribute *max77696_eh_attr[] = {
    &dev_attr_chg_dis.attr,
    &dev_attr_mbcichfc_en.attr,
    &dev_attr_mbcichfc.attr,
    &dev_attr_mbccv.attr,
    &dev_attr_acc_en.attr,
    &dev_attr_acc_ilm.attr,
    NULL
};

static const struct attribute_group max77696_eh_attr_group = {
    .attrs = max77696_eh_attr,
};

static irqreturn_t max77696_eh_acc_isr (int irq, void *data)
{
    struct max77696_eh *eh = data;

    if (unlikely(eh->destroying)) {
        goto out;
    }

    dev_dbg(eh->dev, "accessory detection interrupt\n");

    if (likely(!delayed_work_pending(&eh->acc_work))) {
        schedule_delayed_work(&eh->acc_work, eh->acc_det_debounce);
    }

out:
    return IRQ_HANDLED;
}

static irqreturn_t max77696_eh_isr (int irq, void *data)
{
    struct max77696_eh *eh = data;
    u16 ehint;

    if (unlikely(eh->destroying)) {
        goto out;
    }

    ehint = max77696_eh_read_irq(eh);
    max77696_eh_ack_irq(eh);
    dev_dbg(eh->dev, "EHINT 0x%04X EN 0x%04X\n", ehint, eh->irq_unmask);

    ehint &= eh->irq_unmask;

    if (ehint & CHGB_INT_ACCFLT) {
        dev_dbg(eh->dev, "ACCFLT interrupt received\n");
    }

    if (ehint & CHGB_INT_ACCON) {
        dev_dbg(eh->dev, "ACCON interrupt received\n");
    }

    if (ehint & CHGB_INT_BCHGPOK) {
        dev_dbg(eh->dev, "BCHGPOK interrupt received\n");
    }

    if (ehint & CHGB_INT_ACCLD) {
        dev_dbg(eh->dev, "ACCLD interrupt received\n");
    }

    if (likely(!delayed_work_pending(&eh->psy_work))) {
        schedule_delayed_work(&eh->psy_work, CHGB_PSY_WORK_DELAY);
    }

out:
    return IRQ_HANDLED;
}

static int max77696_eh_update_status (struct max77696_eh *eh)
{
    u16 chginb_stat;
    int rc;

    /* read CHGINB_STAT as an interrupt status */
    rc = max77696_read(eh->io, CHGINB_STAT, &chginb_stat);
    if (unlikely(rc)) {
        dev_err(eh->dev, "CHGINB_STAT read error [%d]\n", rc);
    } else {
        dev_vdbg(eh->dev, "CHGINB_STAT 0x%04X\n", chginb_stat);

        eh->chg_present = !!BITS_GET(chginb_stat, CHGINB_STAT_BCHGPOK);
        eh->chg_enable  = !!BITS_GET(chginb_stat, CHGINB_STAT_BCHGON );
        dev_dbg(eh->dev, "CHG ONLINE %d ENABLE %d\n",
            eh->chg_present, eh->chg_enable);
    }

    if (likely(eh->acc_det_gpio >= 0)) {
        eh->acc_present =
            (gpio_get_value(eh->acc_det_gpio) == eh->acc_det_gpio_assert);
        dev_vdbg(eh->dev, "ACC PRESENT %d\n", eh->acc_present);
    }

    return 0;
}

static void max77696_eh_acc_setup (struct max77696_eh *eh)
{
    struct device *dev = eh->dev;
    unsigned long acc_irqflags;
    int rc, acc_irq;

#ifdef CONFIG_MAX77696_DT
    if (unlikely(eh->acc_det_gpio == -EPROBE_DEFER)) {
        eh->acc_det_gpio = of_get_named_gpio(dev->of_node, "acc_det_gpio", 0);
        if (unlikely(eh->acc_det_gpio == -EPROBE_DEFER)) {
            dev_dbg(dev, "acc_det_gpio not ready\n");
            schedule_delayed_work(&eh->acc_work, CHGB_ACC_DET_GPIO_SEEK);
            goto out;
        }
    }
#endif /* CONFIG_MAX77696_DT */

    if (unlikely(eh->acc_det_gpio < 0)) {
        dev_dbg(dev, "acc_det_gpio not specified\n");
        goto out;
    }

    rc = devm_gpio_request(dev, (unsigned)eh->acc_det_gpio, DRIVER_NAME"-acc");
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to request GPIO(%d) [%d]\n",
            eh->acc_det_gpio, rc);
        goto out;
    }

    rc = gpio_direction_input(eh->acc_det_gpio);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to configure GPIO(%d) as input [%d]\n",
            eh->acc_det_gpio, rc);
        goto abort;
    }

    acc_irq = gpio_to_irq((unsigned)eh->acc_det_gpio);
    if (unlikely(acc_irq < 0)) {
        dev_err(dev, "failed to get irq corresponding to GPIO(%d) [%d]\n",
            eh->acc_det_gpio, acc_irq);
        goto abort;
    }

    acc_irqflags  = IRQF_ONESHOT;
    acc_irqflags |= IRQF_TRIGGER_RISING;
    acc_irqflags |= IRQF_TRIGGER_FALLING;

    rc = devm_request_threaded_irq(dev, (unsigned int)acc_irq, NULL,
        max77696_eh_acc_isr, acc_irqflags, DRIVER_NAME"-acc", eh);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to request IRQ(%d) [%d]\n", acc_irq, rc);
        goto abort;
    }

    dev_info(dev, "accessory detection: GPIO %d IRQ %d\n",
        eh->acc_det_gpio, acc_irq);
    goto out;

abort:
    devm_gpio_free(dev, (unsigned)eh->acc_det_gpio);
    eh->acc_det_gpio = -ENOTSUPP;
out:
    return;
}

static void max77696_eh_acc_work (struct work_struct *work)
{
    struct max77696_eh *eh =
        container_of(work, struct max77696_eh, acc_work.work);

    if (unlikely(eh->destroying)) {
        return;
    }

    if (unlikely(eh->acc_det_gpio < 0)) {
        max77696_eh_acc_setup(eh);
        return;
    }

    __lock(eh);
    max77696_eh_update_status(eh);
    __unlock(eh);
}

static void max77696_eh_psy_work (struct work_struct *work)
{
    struct max77696_eh *eh =
        container_of(work, struct max77696_eh, psy_work.work);

    if (unlikely(eh->destroying)) {
        return;
    }

    __lock(eh);
    max77696_eh_update_status(eh);
    __unlock(eh);
}

static int max77696_eh_setup (struct max77696_eh *eh)
{
    /* Device wakeup initialization */
    device_init_wakeup(eh->dev, true);

    eh->acc_det_gpio        = eh->pdata->acc_det_gpio;
    eh->acc_det_gpio_assert = eh->pdata->acc_det_gpio_assert;
    eh->acc_det_debounce    = msecs_to_jiffies(eh->pdata->acc_det_debounce_ms);
    max77696_eh_acc_setup(eh);

    max77696_eh_chg_dis_set(eh, (eh->pdata->mode & 1) == 0);
    max77696_eh_acc_en_set (eh, (eh->pdata->mode & 2) != 0);

    max77696_eh_acc_ilm_set(eh, eh->pdata->acc_ilimit);

    return 0;
}

static int max77696_eh_psy_get_property (struct power_supply *psy,
        enum power_supply_property psp, union power_supply_propval *val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    struct max77696_eh *eh = dev_get_drvdata(psy->dev->parent);
#else /* VERSION ... */
    struct max77696_eh *eh = power_supply_get_drvdata(psy);
#endif /* VERSION ... */
    int rc = 0;

    memset(val, 0x00, sizeof(*val));

    /* pre-process for strval */
    switch (psp) {
    case POWER_SUPPLY_PROP_MODEL_NAME:
        val->strval = MAX77696_NAME;
        return 0;

    case POWER_SUPPLY_PROP_MANUFACTURER:
        val->strval = "maxim";
        return 0;

    default:
        break;
    }

    __lock(eh);

    max77696_eh_update_status(eh);

    switch (psp) {
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = eh->chg_present;
        break;

    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = eh->chg_enable;
        break;

    case POWER_SUPPLY_PROP_STATUS:
        if (eh->chg_present == false) {
            val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
        } else if (eh->chg_enable) {
            val->intval = POWER_SUPPLY_STATUS_CHARGING;
        } else {
            val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
        break;

    default:
        rc = -EINVAL;
        goto out;
    }

out:
    dev_vdbg(eh->dev,
        "<get_property> psp %d val %d [%d]\n", psp, val->intval, rc);
    __unlock(eh);
    return rc;
}

#define max77696_eh_psy_set_property           NULL
#define max77696_eh_psy_property_is_writeable  NULL
#define max77696_eh_psy_external_power_changed NULL

static enum power_supply_property max77696_eh_psy_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_MODEL_NAME,
    POWER_SUPPLY_PROP_MANUFACTURER,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
static struct power_supply_desc max77696_eh_psy_desc = {
	.name		            = DRIVER_NAME,
	.type		            = POWER_SUPPLY_TYPE_MAINS,
    .properties             = max77696_eh_psy_props,
    .num_properties         = ARRAY_SIZE(max77696_eh_psy_props),
    .get_property           = max77696_eh_psy_get_property,
    .set_property           = max77696_eh_psy_set_property,
    .property_is_writeable  = max77696_eh_psy_property_is_writeable,
    .external_power_changed = max77696_eh_psy_external_power_changed,
};

static struct power_supply_config max77696_eh_psy_config;
#endif /* VERSION >= 4.1.0 */

static void *max77696_eh_get_platdata (struct max77696_eh *eh)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_eh_platform_data *pdata;
    struct device *dev = eh->dev;
    int i;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;
    size_t sz;
    int num_supplicants, num_supplies;

    num_supplicants = of_property_count_strings(np, "supplied_to");
    num_supplicants = max(0, num_supplicants);

    num_supplies = of_property_count_strings(np, "supplied_from");
    num_supplies = max(0, num_supplies);

    sz = sizeof(*pdata) + (num_supplicants + num_supplies) * sizeof(char*);
    pdata = devm_kzalloc(dev, sz, GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sz);
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->psy_name = DRIVER_NAME;
    of_property_read_string(np, "psy_name", (char const **)&pdata->psy_name);

    if (unlikely(num_supplicants <= 0)) {
        pdata->num_supplicants = 0;
        pdata->supplied_to     = NULL;
    } else {
        pdata->num_supplicants = (size_t)num_supplicants;
        pdata->supplied_to = (char**)(pdata + 1);
        for (i = 0; i < num_supplicants; i++) {
            of_property_read_string_index(np, "supplied_to", i,
                (char const **)&pdata->supplied_to[i]);
        }
    }

    if (unlikely(num_supplies <= 0)) {
        pdata->num_supplies  = 0;
        pdata->supplied_from = NULL;
    } else {
        pdata->num_supplies  = (size_t)num_supplies;
        pdata->supplied_from = (char**)(pdata + 1) + num_supplicants;
        for (i = 0; i < num_supplies; i++) {
            of_property_read_string_index(np, "supplied_from", i,
                (char const **)&pdata->supplied_from[i]);
        }
    }

    pdata->mode = MAX77696_EH_MODE_CHARGER;
    of_property_u8(np, "mode", &pdata->mode);

    pdata->acc_ilimit = 200;
    of_property_u32(np, "acc_ilimit", &pdata->acc_ilimit);

    pdata->acc_det_debounce_ms = 0;
    of_property_u32(np, "acc_det_debounce_ms", &pdata->acc_det_debounce_ms);

    pdata->acc_det_gpio_assert = 0;
    of_property_u32(np, "acc_det_gpio_assert", &pdata->acc_det_gpio_assert);

	pdata->acc_det_gpio = of_get_named_gpio(np, "acc_det_gpio", 0);
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "PSY NAME", "%s", pdata->psy_name);

    if (unlikely(pdata->num_supplicants <= 0)) {
        __prop_printk(dev, "SUPPLICANTS", "%s", "null");
    } else {
        for (i = 0; i < pdata->num_supplicants; i++) {
            __prop_printk(dev, "SUPPLICANTS", "%s", pdata->supplied_to[i]);
        }
    }

    if (unlikely(pdata->num_supplies <= 0)) {
        pdata->supplied_from = NULL;
        pdata->num_supplies  = 0;
        __prop_printk(dev, "SUPPLIES", "%s", "null");
    } else {
        for (i = 0; i < pdata->num_supplies; i++) {
            __prop_printk(dev, "SUPPLIES", "%s", pdata->supplied_from[i]);
        }
    }

    __prop_printk(dev, "MODE", "%d", pdata->mode);
    __prop_printk(dev, "ACC ILIMIT", "%u mA", pdata->acc_ilimit);

    if (pdata->acc_det_gpio >= 0 || pdata->acc_det_gpio == -EPROBE_DEFER) {
        if (pdata->acc_det_gpio == -EPROBE_DEFER) {
            __prop_printk(dev, "ACC DET GPIO", "%s", "(deferred)");
        } else {
            __prop_printk(dev, "ACC DET GPIO", "%d", pdata->acc_det_gpio);
        }
        __prop_printk(dev, "ACC DET DEBOUNCE", "%u msec",
            pdata->acc_det_debounce_ms);
        __prop_printk(dev, "ACC DET ASSERT", "%s",
            pdata->acc_det_gpio_assert ? "HIGH" : "LOW");
    } else {
        __prop_printk(dev, "ACC DET GPIO", "%s", "(n/a)");
    }

out:
    return pdata;
}

static __always_inline void max77696_eh_destroy (struct max77696_eh *eh)
{
    struct device *dev = eh->dev;

    eh->destroying = true;
    cancel_delayed_work_sync(&eh->psy_work);
    cancel_delayed_work_sync(&eh->acc_work);

    if (likely(eh->attr_grp)) {
        sysfs_remove_group(eh->kobj, eh->attr_grp);
    }

    if (likely(eh->irq > 0)) {
        devm_free_irq(dev, eh->irq, eh);
    }

    if (likely(eh->psy)) {
        power_supply_unregister(eh->psy);
    }

    if (likely(eh->acc_det_gpio >= 0)) {
        unsigned acc_det_gpio = (unsigned)eh->acc_det_gpio;
        unsigned acc_det_irq  = (unsigned)gpio_to_irq(acc_det_gpio);

        devm_free_irq(dev, acc_det_irq, eh);
        devm_gpio_free(dev, acc_det_gpio);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(eh->pdata)) {
        devm_kfree(dev, eh->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&eh->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, eh);
}

static __devinit int max77696_eh_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_eh *eh;
    size_t sz;
    u16 ehint;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    sz  = sizeof(*eh);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    sz += sizeof(struct power_supply);
#endif /* VERSION < 4.1.0 */

    eh = devm_kzalloc(dev, sz, GFP_KERNEL);
    if (unlikely(!eh)) {
        dev_err(dev, "out of memory (%uB requested)\n", sz);
        return -ENOMEM;
    }

    dev_set_drvdata(dev, eh);

    mutex_init(&eh->lock);
    eh->core  = core;
    eh->io    = max77696_get_block_io(dev->parent, EH);
    eh->dev   = dev;
    eh->kobj  = &dev->kobj;

    INIT_DELAYED_WORK(&eh->psy_work, max77696_eh_psy_work);
    INIT_DELAYED_WORK(&eh->acc_work, max77696_eh_acc_work);

    /* Disable all EH interrupts */
    eh->irq_unmask = 0;
    max77696_write(eh->io, CHGINB_INTM, ~0);

    /* Get EH interrupt status port address & Clear status */
    ehint = max77696_eh_read_irq(eh);
    max77696_eh_ack_irq(eh);
    dev_dbg(dev, "initial EH interrupt status: 0x%04X\n", ehint);

    eh->pdata = max77696_eh_get_platdata(eh);
    if (unlikely(IS_ERR(eh->pdata))) {
        rc = PTR_ERR(eh->pdata);
        eh->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

	rc = max77696_eh_setup(eh);
	if (unlikely(rc)) {
	    dev_err(dev, "failed to setup [%d]\n", rc);
	    goto abort;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    eh->psy                         = (struct power_supply*)(eh + 1);
    eh->psy->name                   =
        eh->pdata->psy_name ? eh->pdata->psy_name : DRIVER_NAME;
    eh->psy->type                   = POWER_SUPPLY_TYPE_MAINS;
    eh->psy->supplied_to            = eh->pdata->supplied_to;
    eh->psy->num_supplicants        = eh->pdata->num_supplicants;
    eh->psy->properties             = max77696_eh_psy_props;
    eh->psy->num_properties         = ARRAY_SIZE(max77696_eh_psy_props);
    eh->psy->get_property           = max77696_eh_psy_get_property;
    eh->psy->set_property           = max77696_eh_psy_set_property;
    eh->psy->property_is_writeable  =
        max77696_eh_psy_property_is_writeable;
    eh->psy->external_power_changed =
        max77696_eh_psy_external_power_changed;

    rc = power_supply_register(dev, eh->psy);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register power_supply class [%d]\n", rc);
        eh->psy = NULL;
        goto abort;
    }
#else /* VERSION ... */
    if (eh->pdata->psy_name) {
        max77696_eh_psy_desc.name = eh->pdata->psy_name;
    }

    max77696_eh_psy_config.drv_data        = eh;
    max77696_eh_psy_config.supplied_to     = eh->pdata->supplied_to;
    max77696_eh_psy_config.num_supplicants = eh->pdata->num_supplicants;

    eh->psy = power_supply_register(dev, &max77696_eh_psy_desc,
                        &max77696_eh_psy_config);
    if (unlikely(IS_ERR(eh->psy))) {
        rc = PTR_ERR(eh->psy);
        eh->psy = NULL;
        dev_err(dev, "failed to register power_supply class [%d]\n", rc);
        goto abort;
    }
#endif /* VERSION ... */

    /* Get EH block IRQ number */
    eh->irq = max77696_get_block_irq(dev->parent, EH);
    BUG_ON(eh->irq <= 0);

    /* Request system IRQ for EH */
    rc = devm_request_threaded_irq(dev, (unsigned int)eh->irq, NULL,
        max77696_eh_isr, IRQF_ONESHOT, DRIVER_NAME, eh);
    if (unlikely(rc)) {
        dev_err(dev, "failed to request IRQ(%d) [%d]\n", eh->irq, rc);
        eh->irq = 0;
        goto abort;
    }

    disable_irq(eh->irq);
    dev_dbg(dev, "IRQ(%d) requested\n", eh->irq);

    /* Create max77696-eh sysfs attributes */
    eh->attr_grp = &max77696_eh_attr_group;
    rc = sysfs_create_group(eh->kobj, eh->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        eh->attr_grp = NULL;
        goto abort;
    }

    /* Enable EH interrupts we need */
    max77696_eh_enable_irq(eh, CHGB_INT_BCHGPOK);

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_eh_destroy(eh);
    return rc;
}

static __devexit int max77696_eh_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_eh *eh = dev_get_drvdata(dev);

    max77696_eh_destroy(eh);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_eh_suspend (struct device *dev)
{
    return 0;
}

static int max77696_eh_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_eh_pm,
    max77696_eh_suspend, max77696_eh_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_eh_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_eh_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_eh_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_eh_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_eh_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_eh_probe,
    .remove                = __devexit_p(max77696_eh_remove),
};

static __init int max77696_eh_driver_init (void)
{
	return platform_driver_register(&max77696_eh_driver);
}
late_initcall(max77696_eh_driver_init);

static __exit void max77696_eh_driver_exit (void)
{
	platform_driver_unregister(&max77696_eh_driver);
}
module_exit(max77696_eh_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
