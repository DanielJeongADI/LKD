/*
 * MAX77696 GPIO Driver
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

#ifdef MODULE
#error "NOT IMPLEMENTED AS MODULABLE"
#endif /* MODULE */

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

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/seq_file.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    MAX77696_DESC" GPIO Driver"
#define DRIVER_NAME    MAX77696_GPIO_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define GPIO_RWC_INTERRUPT        0
#define GPIO_DEFAULT_BASE         -1
#ifdef CONFIG_SPARSE_IRQ
#define GPIO_DEFAULT_IRQ_BASE     NR_IRQS
#else /* CONFIG_SPARSE_IRQ */
#define GPIO_DEFAULT_IRQ_BASE     8
#endif /* CONFIG_SPARSE_IRQ */

#define GPIO_NGPIO                MAX77696_NUM_OF_GPIOS

#define GPIO_DIR_OUT              GPIOF_DIR_OUT
#define GPIO_DIR_IN               GPIOF_DIR_IN

#define CNFG_GPIO0                0x75
#define CNFG_GPIO1                0x76
#define CNFG_GPIO2                0x77
#define CNFG_GPIO3                0x78
#define CNFG_GPIO4                0x79
#define CNFG_GPIO(_gpio)          (CNFG_GPIO0+(_gpio))

#define CNFG_GPIO_DBNC            BITS(7,6)
#define CNFG_GPIO_REFE_IRQ        BITS(5,4)
#define CNFG_GPIO_DO              BIT (3)
#define CNFG_GPIO_DI              BIT (2)
#define CNFG_GPIO_DIR             BIT (1)
#define CNFG_GPIO_PPDRV           BIT (0)

#define PUE_GPIO                  0x7A
#define PUE_GPIO_BIAS_EN          BIT (7)

#define PDE_GPIO                  0x7B
#define AME_GPIO                  0x7C

#define GPIO_INT                  0x7D
#define GPIO_INT_BIT(_gpio)       (1 << (_gpio))

#define GPIO_BITMAP_SZ            BITS_TO_LONGS(GPIO_NGPIO)

struct max77696_gpios {
    struct mutex                        lock;
    struct max77696_gpio_platform_data *pdata;
    struct max77696_core               *core;
    struct max77696_io                 *io;
    struct device                      *dev;
    struct kobject                     *kobj;
    const struct attribute_group       *attr_grp;

    struct gpio_chip                   *gpio_chip;

    int                                 irq;
    unsigned int                        irq_base;
    unsigned int                        irq_type[GPIO_NGPIO];

    unsigned long                       wakeup_gpios[GPIO_BITMAP_SZ];
    unsigned long                       irq_enabled_gpios[GPIO_BITMAP_SZ];
    unsigned long                       irq_cfg_dirty_gpios[GPIO_BITMAP_SZ];
};

#define __lock(_me)      mutex_lock(&(_me)->lock)
#define __unlock(_me)    mutex_unlock(&(_me)->lock)

/* GPIO Configuration Structures */

struct max77696_gpio_cfg_bitdesc {
    struct max77696_bitdesc drv, dir, din, dout, intcnfg, dbnc;
    struct max77696_bitdesc puen;
    struct max77696_bitdesc pden;
    struct max77696_bitdesc amen;
};

#define GPIO_BITDESC  MAX77696_BITDESC
#define GPIO_CFG_BITDESC(_gpio) \
        [_gpio] = {\
            .drv     = GPIO_BITDESC(CNFG_GPIO(_gpio), CNFG_GPIO_PPDRV   ),\
            .dir     = GPIO_BITDESC(CNFG_GPIO(_gpio), CNFG_GPIO_DIR     ),\
            .din     = GPIO_BITDESC(CNFG_GPIO(_gpio), CNFG_GPIO_DI      ),\
            .dout    = GPIO_BITDESC(CNFG_GPIO(_gpio), CNFG_GPIO_DO      ),\
            .intcnfg = GPIO_BITDESC(CNFG_GPIO(_gpio), CNFG_GPIO_REFE_IRQ),\
            .dbnc    = GPIO_BITDESC(CNFG_GPIO(_gpio), CNFG_GPIO_DBNC    ),\
            .puen    = GPIO_BITDESC(PUE_GPIO        , 1 <<  (_gpio)     ),\
            .pden    = GPIO_BITDESC(PDE_GPIO        , 1 <<  (_gpio)     ),\
            .amen    = GPIO_BITDESC(AME_GPIO        , 3 << ((_gpio)*2)  ),\
        }

static struct max77696_gpio_cfg_bitdesc max77696_gpio_cfgs[] = {
    GPIO_CFG_BITDESC( 0), GPIO_CFG_BITDESC( 1), GPIO_CFG_BITDESC( 2),
    GPIO_CFG_BITDESC( 3), GPIO_CFG_BITDESC( 4),
};

#define __gpio_cfg_bitdesc(_gpio, _cfg) \
        (&max77696_gpio_cfgs[_gpio]._cfg)
#define __gpio_cfg_reg(_gpio, _cfg) \
        (__gpio_cfg_bitdesc(_gpio, _cfg)->reg)
#define __gpio_cfg_mask(_gpio, _cfg) \
        (__gpio_cfg_bitdesc(_gpio, _cfg)->mask)
#define __gpio_cfg_shift(_gpio, _cfg) \
        (__gpio_cfg_bitdesc(_gpio, _cfg)->shift)

#define max77696_gpio_cfg_read(_gpios, _gpio, _cfg, _val_ptr) \
        ({\
            int __rc = max77696_read_bitdesc((_gpios)->io,\
                __gpio_cfg_bitdesc(_gpio, _cfg), _val_ptr);\
            dev_vdbg((_gpios)->dev, "gpio_cfg_read: gpio-%2u " #_cfg \
                " addr 0x%04X mask 0x%04X val 0x%04X [%d]\n",\
                (unsigned)(_gpio),\
                __gpio_cfg_reg(_gpio, _cfg),\
                __gpio_cfg_mask(_gpio, _cfg),\
                *(_val_ptr),\
                __rc);\
            __rc;\
        })

#define max77696_gpio_cfg_write(_gpios, _gpio, _cfg, _val) \
        ({\
            int __rc = max77696_write_bitdesc((_gpios)->io,\
                __gpio_cfg_bitdesc(_gpio, _cfg), _val);\
            dev_vdbg((_gpios)->dev, "gpio_cfg_write: gpio-%2u " #_cfg \
                " addr %02X mask %02X val %02X [%d]\n",\
                (unsigned)(_gpio),\
                __gpio_cfg_reg(_gpio, _cfg),\
                __gpio_cfg_mask(_gpio, _cfg),\
                _val,\
                __rc);\
            __rc;\
        })

static int max77696_gpio_chip_direction_input (struct gpio_chip *chip,
    unsigned offset)
{
    struct max77696_gpios *gpios = dev_get_drvdata(chip->dev);
    int rc;

    __lock(gpios);

    rc = max77696_gpio_cfg_write(gpios, offset, dir, MAX77696_GPIO_DIR_INPUT);

    __unlock(gpios);
    return rc;
}

static int max77696_gpio_chip_get (struct gpio_chip *chip, unsigned offset)
{
    struct max77696_gpios *gpios = dev_get_drvdata(chip->dev);
    u16 din = 0;
    int rc;

    __lock(gpios);

    max77696_gpio_cfg_read(gpios, offset, din, &din);
    rc = (int)din;

    __unlock(gpios);
    return rc;
}

static int max77696_gpio_chip_direction_output (struct gpio_chip *chip,
    unsigned offset, int value)
{
    struct max77696_gpios *gpios = dev_get_drvdata(chip->dev);
    int rc;

    __lock(gpios);

    rc = max77696_gpio_cfg_write(gpios, offset, dir, MAX77696_GPIO_DIR_OUTPUT);

    __unlock(gpios);
    return rc;
}

static void max77696_chip_gpio_set (struct gpio_chip *chip,
    unsigned offset, int value)
{
    struct max77696_gpios *gpios = dev_get_drvdata(chip->dev);

    __lock(gpios);

    max77696_gpio_cfg_write(gpios, offset, dout, !!value);

    __unlock(gpios);
}

static int max77696_gpio_chip_to_irq (struct gpio_chip *chip, unsigned offset)
{
    struct max77696_gpios *gpios = dev_get_drvdata(chip->dev);

    return (int)(gpios->irq_base + offset);
}

static void max77696_gpio_chip_dbg_show (struct seq_file *s,
    struct gpio_chip *chip)
{
    struct max77696_gpios *gpios = dev_get_drvdata(chip->dev);
    const char *label;
    u16 dir = 0, lvl = 0;
    int i;

    __lock(gpios);

    for (i = 0; i < GPIO_NGPIO; i++) {
        label = gpiochip_is_requested(chip, i);

        seq_printf(s, " gpio-%-3d (%-20.20s) ",
            chip->base + i, label? label : "--");

        max77696_gpio_cfg_read(gpios, i, dir, &dir);

        if (dir) {
            max77696_gpio_cfg_read(gpios, i, din, &lvl);
        } else {
            max77696_gpio_cfg_read(gpios, i, dout, &lvl);
        }

        seq_printf(s, "%-5.5s %s\n", dir ? "in" : "out", lvl ? "hi" : "lo");
    }

    __unlock(gpios);
}

static struct gpio_chip max77696_gpio_chip = {
    .label            = DRIVER_NAME,
    .owner            = THIS_MODULE,
    .direction_input  = max77696_gpio_chip_direction_input,
    .get              = max77696_gpio_chip_get,
    .direction_output = max77696_gpio_chip_direction_output,
    .set              = max77696_chip_gpio_set,
    .to_irq           = max77696_gpio_chip_to_irq,
    .dbg_show         = max77696_gpio_chip_dbg_show,
    .ngpio            = GPIO_NGPIO,
};

static __always_inline
u8 max77696_gpio_read_irq (struct max77696_gpios *gpios)
{
    u16 gpioint;
    int rc;

    rc = max77696_read(gpios->io, GPIO_INT, &gpioint);
    if (unlikely(rc)) {
        dev_err(gpios->dev, "GPIO_INT read error [%d]\n", rc);
        return 0;
    }

    return gpioint;
}

static __always_inline
void max77696_gpio_ack_irq (struct max77696_gpios *gpios)
{
    if (GPIO_RWC_INTERRUPT) {
        max77696_write(gpios->io, GPIO_INT, ~0);
    }
}

static void max77696_gpio_sync_irq (struct max77696_gpios *gpios,
    int gpio, bool force)
{
    u16 intcnfg;
    int rc;

    if (unlikely(!force && !test_bit(gpio, gpios->irq_cfg_dirty_gpios))) {
        return;
    }

    /* Setup intcnfg */
    intcnfg = MAX77696_GPIO_INTCNFG_DISABLE;
    if (likely(test_bit(gpio, gpios->irq_enabled_gpios))) {
        if (gpios->irq_type[gpio] & IRQ_TYPE_EDGE_RISING) {
            intcnfg |= MAX77696_GPIO_INTCNFG_RISING_EDGE;
        }
        if (gpios->irq_type[gpio] & IRQ_TYPE_EDGE_FALLING) {
            intcnfg |= MAX77696_GPIO_INTCNFG_FALLING_EDGE;
        }
    }

    /* write intcnfg bit */
    rc = max77696_gpio_cfg_write(gpios, gpio, intcnfg, intcnfg);
    if (unlikely(rc)) {
        goto out;
    }

    /* clear dirty */
    clear_bit(gpio, gpios->irq_cfg_dirty_gpios);

out:
    return;
}

static void max77696_gpio_irq_mask (struct irq_data *data)
{
    struct max77696_gpios *gpios = irq_data_get_irq_chip_data(data);
    unsigned int gpio = data->irq - gpios->irq_base;

    if (unlikely(!test_bit(gpio, gpios->irq_enabled_gpios))) {
        /* already masked */
        return;
    }

    /* clear enabled flag */
    clear_bit(gpio, gpios->irq_enabled_gpios);

    /* set dirty */
    set_bit(gpio, gpios->irq_cfg_dirty_gpios);
}

static void max77696_gpio_irq_unmask (struct irq_data *data)
{
    struct max77696_gpios *gpios = irq_data_get_irq_chip_data(data);
    unsigned int gpio = data->irq - gpios->irq_base;

    if (unlikely(test_bit(gpio, gpios->irq_enabled_gpios))) {
        /* already unmasked */
        return;
    }

    /* set enabled flag */
    set_bit(gpio, gpios->irq_enabled_gpios);

    /* set dirty */
    set_bit(gpio, gpios->irq_cfg_dirty_gpios);
}

static void max77696_gpio_irq_bus_lock (struct irq_data *data)
{
    struct max77696_gpios *me = irq_data_get_irq_chip_data(data);

    __lock(me);
}

/*
 * genirq core code can issue chip->mask/unmask from atomic context.
 * This doesn't work for slow busses where an access needs to sleep.
 * bus_sync_unlock() is therefore called outside the atomic context,
 * syncs the current irq mask state with the slow external controller
 * and unlocks the bus.
 */

static void max77696_gpio_irq_bus_sync_unlock (struct irq_data *data)
{
    struct max77696_gpios *gpios = irq_data_get_irq_chip_data(data);
    int i;

    //disable_irq(gpios->irq);

    for (i = 0; i < GPIO_NGPIO; i++) {
        max77696_gpio_sync_irq(gpios, i, false);
    }

    //if (likely(!bitmap_empty(gpios->irq_enabled_gpios, GPIO_NGPIO))) {
    //    enable_irq(gpios->irq);
    //}

    __unlock(gpios);
}

static int max77696_gpio_irq_set_type (struct irq_data *data, unsigned int type)
{
    struct max77696_gpios *gpios = irq_data_get_irq_chip_data(data);
    int gpio = (int)(data->irq - gpios->irq_base);
    int rc = 0;

    if (unlikely(type & ~IRQ_TYPE_EDGE_BOTH)) {
        dev_err(gpios->dev, "gpio %d: unsupported irq type %d\n", gpio, type);
        rc = -EINVAL;
        goto out;
    }

    if (unlikely(gpios->irq_type[gpio] == type)) {
        goto out;
    }

    gpios->irq_type[gpio] = type;
    set_bit(gpio, gpios->irq_cfg_dirty_gpios);

    if (likely(test_bit(gpio, gpios->irq_enabled_gpios))) {
        max77696_gpio_sync_irq(gpios, gpio, true);
    }

out:
    return rc;
}

static int max77696_gpio_irq_set_wake (struct irq_data *data, unsigned int on)
{
    struct max77696_gpios *me = irq_data_get_irq_chip_data(data);
    unsigned int gpio = data->irq - me->irq_base;

    if (on) {
        if (unlikely(bitmap_empty(me->wakeup_gpios, GPIO_NGPIO))) {
            enable_irq_wake(me->irq);
        }
        set_bit  (gpio, me->wakeup_gpios);
    } else {
        clear_bit(gpio, me->wakeup_gpios);
        if (unlikely(bitmap_empty(me->wakeup_gpios, GPIO_NGPIO))) {
            disable_irq_wake(me->irq);
        }
    }

    return 0;
}

static struct irq_chip max77696_gpio_irq_chip = {
    .name                = DRIVER_NAME,
//  .flags               = IRQCHIP_SET_TYPE_MASKED,
    .irq_mask            = max77696_gpio_irq_mask,
    .irq_unmask          = max77696_gpio_irq_unmask,
    .irq_bus_lock        = max77696_gpio_irq_bus_lock,
    .irq_bus_sync_unlock = max77696_gpio_irq_bus_sync_unlock,
    .irq_set_type        = max77696_gpio_irq_set_type,
    .irq_set_wake        = max77696_gpio_irq_set_wake,
};

#define GPIO_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_gpio_attr[] = {
    NULL
};

static const struct attribute_group max77696_gpio_attr_group = {
    .attrs = max77696_gpio_attr,
};

static irqreturn_t max77696_gpio_isr (int irq, void *data)
{
    struct max77696_gpios *gpios = data;
    int i;
    u16 gpioint;

    gpioint = max77696_gpio_read_irq(gpios);
    max77696_gpio_ack_irq(gpios);
    dev_dbg(gpios->dev, "GPIOINT 0x%04X\n", gpioint);

    for (i = 0; i < GPIO_NGPIO; i++) {
        u16 irq_bit = GPIO_INT_BIT(i);

        if (unlikely(!test_bit(i, gpios->irq_enabled_gpios))) {
            continue;
        }

        if (unlikely((gpioint & irq_bit) == 0)) {
            continue;
        }

        dev_dbg(gpios->dev, "handle GPIO%d IRQ %u\n", i,
            gpios->irq_base + (unsigned int)i);
        handle_nested_irq(gpios->irq_base + (unsigned int)i);
    }

    return IRQ_HANDLED;
}

static void max77696_gpio_dump_cfg (struct max77696_gpios *me,
    struct max77696_gpio_cfg_data *cfg_data)
{
    if (cfg_data->direction) {
        dev_dbg(me->dev, "gpi-%2u func-%u %s %s dbnc-%u intcnfg-%u\n",
            cfg_data->gpio, cfg_data->alter_mode,
            cfg_data->pullup_en ? "pu" : "--",
            cfg_data->pulldn_en ? "pd" : "--",
            cfg_data->u.input.dbnc, cfg_data->u.input.intcnfg);
    } else {
        dev_dbg(me->dev, "gpo-%2u func-%u %s %s drv-%u  do-%u\n",
            cfg_data->gpio, cfg_data->alter_mode,
            cfg_data->pullup_en ? "pu" : "--",
            cfg_data->pulldn_en ? "pd" : "--",
            cfg_data->u.output.drive, cfg_data->u.output.level);
    }
}

static void max77696_gpio_write_cfg (struct max77696_gpios *me,
    struct max77696_gpio_cfg_data *cfg_data)
{
    max77696_gpio_dump_cfg(me, cfg_data);

    if (unlikely(cfg_data->gpio >= GPIO_NGPIO)) {
        dev_err(me->dev, "invalid gpio number %u\n", cfg_data->gpio);
        goto out;
    }

    /* Write alter mode */
    max77696_gpio_cfg_write(me, cfg_data->gpio, amen, cfg_data->alter_mode);

    /* Write pull mode */
    max77696_gpio_cfg_write(me, cfg_data->gpio, puen, !!cfg_data->pullup_en);
    max77696_gpio_cfg_write(me, cfg_data->gpio, pden, !!cfg_data->pulldn_en);

    /* Write dir-depend config */
    if (cfg_data->direction) {
        max77696_gpio_cfg_write(me, cfg_data->gpio, dir,
            MAX77696_GPIO_DIR_INPUT);
        max77696_gpio_cfg_write(me, cfg_data->gpio, dbnc,
            cfg_data->u.input.dbnc);
        max77696_gpio_cfg_write(me, cfg_data->gpio, intcnfg,
            cfg_data->u.input.intcnfg);

        /* save irq type */
        if (cfg_data->u.input.intcnfg & MAX77696_GPIO_INTCNFG_FALLING_EDGE) {
            me->irq_type[cfg_data->gpio] |= IRQ_TYPE_EDGE_FALLING;
        }
        if (cfg_data->u.input.intcnfg & MAX77696_GPIO_INTCNFG_RISING_EDGE) {
            me->irq_type[cfg_data->gpio] |= IRQ_TYPE_EDGE_RISING;
        }
    } else {
        max77696_gpio_cfg_write(me, cfg_data->gpio, dir,
            MAX77696_GPIO_DIR_OUTPUT);
        max77696_gpio_cfg_write(me, cfg_data->gpio, drv,
            cfg_data->u.output.drive);
      //max77696_gpio_cfg_write(me, cfg_data->gpio, dout,
      //    cfg_data->u.output.level);
    }

out:
    return;
}

static void *max77696_gpio_get_platdata (struct max77696_gpios *gpios)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_gpio_platform_data *pdata;
    struct device *dev = gpios->dev;
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

    pdata->gpio_base = 0;
    of_property_u32(np, "gpio_base", (u32*)&pdata->gpio_base);

    pdata->irq_base = 0;
    of_property_u32(np, "irq_base", &pdata->irq_base);

    i = 0;
    for_each_child_of_node(np, cfg_np) {
        of_property_u32(cfg_np, "reg",
            (u32*)&pdata->cfg_data[i].gpio);

        of_property_u8(cfg_np, "alter_mode",
            &pdata->cfg_data[i].alter_mode);

        pdata->cfg_data[i].pullup_en =
            (u8)of_property_read_bool(cfg_np, "pullup_en");
        pdata->cfg_data[i].pulldn_en =
            (u8)of_property_read_bool(cfg_np, "pulldn_en");

        of_property_u8(cfg_np, "direction",
            &pdata->cfg_data[i].direction);

        if (pdata->cfg_data[i].direction == MAX77696_GPIO_DIR_OUTPUT) {
            of_property_u8(cfg_np, "drive",
                &pdata->cfg_data[i].u.output.drive);
            of_property_u8(cfg_np, "level",
                &pdata->cfg_data[i].u.output.level);
        } else {
            of_property_u8(cfg_np, "dbnc",
                &pdata->cfg_data[i].u.input.dbnc);
            of_property_u8(cfg_np, "intcnfg",
                &pdata->cfg_data[i].u.input.intcnfg);
        }

        i++;
    }
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    if (pdata->gpio_base <= 0) {
        pdata->gpio_base = GPIO_DEFAULT_BASE;
    }
    if (pdata->irq_base <= 0) {
        pdata->irq_base = GPIO_DEFAULT_IRQ_BASE;
    }

    __prop_printk(dev, "GPIOBASE", "%d", pdata->gpio_base);
    __prop_printk(dev, "IRQBASE", "%d", pdata->irq_base);

    __prop_printk(dev, "CFGDATA", "%zu", pdata->num_of_cfg_data);
    for (i = 0; i < pdata->num_of_cfg_data; i++) {
        max77696_gpio_dump_cfg(gpios, &pdata->cfg_data[i]);
    }

out:
    return pdata;
}

static __always_inline void max77696_gpio_destroy (struct max77696_gpios *gpios)
{
    struct device *dev = gpios->dev;
    int i;

    if (likely(gpios->attr_grp)) {
        sysfs_remove_group(gpios->kobj, gpios->attr_grp);
    }

    if (likely(gpios->irq > 0)) {
        devm_free_irq(dev, gpios->irq, gpios);
    }

    if (likely(gpios->irq_base > 0)) {
        for (i = 0; i < GPIO_NGPIO; i++) {
            unsigned int irq = gpios->irq_base + i;

            irq_set_handler(irq, NULL);
            irq_set_chip_data(irq, NULL);
        }

        irq_free_descs(gpios->irq_base, GPIO_NGPIO);
    }

    if (likely(gpios->gpio_chip)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
        int rc = gpiochip_remove(gpios->gpio_chip);
        BUG_ON(rc);
#else /* VERSION < 3.18.0 */
        gpiochip_remove(gpios->gpio_chip);
#endif /* VERSION ... */
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(gpios->pdata)) {
        devm_kfree(dev, gpios->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&gpios->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, gpios);
}

static __devinit int max77696_gpio_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_gpios *gpios;
    u16 gpioint;
    int i, rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    gpios = devm_kzalloc(dev, sizeof(*gpios), GFP_KERNEL);
    if (unlikely(!gpios)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*gpios));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, gpios);

    mutex_init(&gpios->lock);
    gpios->core  = core;
    gpios->io    = max77696_get_block_io(dev->parent, GPIO);
    gpios->dev   = dev;
    gpios->kobj  = &dev->kobj;

    /* Disable all GPIO interrupts */
    for (i = 0; i < GPIO_NGPIO; i++) {
        gpios->irq_type[i] = IRQ_TYPE_NONE;
        max77696_gpio_sync_irq(gpios, i, true);
    }

    /* Get GPIO interrupt status port address & Clear status */
    gpioint = max77696_gpio_read_irq(gpios);
    max77696_gpio_ack_irq(gpios);
    dev_dbg(dev, "initial GPIO interrupt status: 0x%04X\n", gpioint);

    gpios->pdata = max77696_gpio_get_platdata(gpios);
    if (unlikely(IS_ERR(gpios->pdata))) {
        rc = PTR_ERR(gpios->pdata);
        gpios->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Device wakeup initialization */
    device_init_wakeup(dev, true);

    /* Initialize configurations */
    for (i = 0; i < gpios->pdata->num_of_cfg_data; i++) {
        max77696_gpio_write_cfg(gpios, &gpios->pdata->cfg_data[i]);
    }

    gpios->gpio_chip = &max77696_gpio_chip;

    gpios->gpio_chip->base  = gpios->pdata->gpio_base;
    gpios->gpio_chip->ngpio = GPIO_NGPIO;
    gpios->gpio_chip->dev   = dev;

    rc = gpiochip_add(gpios->gpio_chip);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to add gpiochip [%d]\n", rc);
        gpios->gpio_chip = NULL;
        goto abort;
    }

    dev_dbg(dev, "GPIO base %u\n", gpios->gpio_chip->base);

    rc = irq_alloc_descs(-1, gpios->pdata->irq_base, GPIO_NGPIO, 0);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to alloc irq_descs [%d]\n", rc);
        goto abort;
    }

    gpios->irq_base = (unsigned int)rc;
    dev_dbg(dev, "IRQ base %u\n", gpios->irq_base);

    for (i = 0; i < GPIO_NGPIO; i++) {
        unsigned int irq = gpios->irq_base + i;

        irq_set_chip_data(irq, gpios);
        irq_set_chip_and_handler(irq, &max77696_gpio_irq_chip,
            handle_simple_irq);
        irq_set_nested_thread(irq, true);

#ifdef CONFIG_ARM
        /*
         * ARM needs us to explicitly flag the IRQ as VALID,
         * once we do so, it will also set the noprobe.
         */
        set_irq_flags(irq, IRQF_VALID);
#else
        irq_set_noprobe(irq);
#endif
    }

    /* Get GPIO block IRQ number */
    gpios->irq = max77696_get_block_irq(dev->parent, GPIO);
    BUG_ON(gpios->irq <= 0);

    /* Request system IRQ for GPIO */
    rc = devm_request_threaded_irq(dev, (unsigned int)gpios->irq, NULL,
        max77696_gpio_isr, IRQF_ONESHOT, DRIVER_NAME, gpios);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to request IRQ(%d) [%d]\n", gpios->irq, rc);
        gpios->irq = 0;
        goto abort;
    }

    dev_dbg(dev, "IRQ(%d) requested\n", gpios->irq);

    //disable_irq(gpios->irq);

    /* Create max77696-gpio sysfs attributes */
    gpios->attr_grp = &max77696_gpio_attr_group;
    rc = sysfs_create_group(gpios->kobj, gpios->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        gpios->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_gpio_destroy(gpios);
    return rc;
}

static __devexit int max77696_gpio_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_gpios *gpios = dev_get_drvdata(dev);

    max77696_gpio_destroy(gpios);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_gpio_suspend (struct device *dev)
{
    return 0;
}

static int max77696_gpio_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_gpio_pm,
    max77696_gpio_suspend, max77696_gpio_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_gpio_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_gpio_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_gpio_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_gpio_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_gpio_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_gpio_probe,
    .remove                = __devexit_p(max77696_gpio_remove),
};

static __init int max77696_gpio_driver_init (void)
{
    return platform_driver_register(&max77696_gpio_driver);
}
subsys_initcall(max77696_gpio_driver_init);

#if 0
static __exit void max77696_gpio_driver_exit (void)
{
    platform_driver_unregister(&max77696_gpio_driver);
}
module_exit(max77696_gpio_driver_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
