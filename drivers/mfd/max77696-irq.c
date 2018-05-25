/*
 * MAX77696 TOPINT Support
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

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77696.h>
#include "max77696-core.h"

#define FILE_DESC    MAX77696_DESC" IRQ"
#define FILE_NAME    MAX77696_TOPINT_NAME
#define FILE_VERSION MAX77696_CORE_VERSION
#define FILE_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define TOPINT_THREADED_ISR       1
#define TOPINT_RWC_INTERRUPT      0

#define TOPINT_NBANK              2
#define TOPINT_NIRQ               MAX77696_NUM_OF_IRQS

#define INTTOP1                   0xA2
#define INTTOP1M                  0xA3

#define INTTOP1_UIC_INT           BIT(0)
#define INTTOP1_LDO_INT           BIT(1)
#define INTTOP1_CHGA_INT          BIT(2)
#define INTTOP1_RTC_INT           BIT(3)
#define INTTOP1_GPIO_INT          BIT(4)
#define INTTOP1_FG_INT            BIT(5)
#define INTTOP1_BUCK_INT          BIT(6)
#define INTTOP1_TOPSYS_INT        BIT(7)

#define INTTOP2                   0xA4
#define INTTOP2M                  0xA5

#define INTTOP2_CHGB_INT          BIT(4)
#define INTTOP2_EPD_INT           BIT(5)
#define INTTOP2_WLED_INT          BIT(6)
#define INTTOP2_ADC_INT           BIT(7)

#define TOPINT_BANK_BITMAP_SZ     BITS_TO_LONGS(TOPINT_NBANK)
#define TOPINT_IRQ_BITMAP_SZ      BITS_TO_LONGS(TOPINT_NIRQ)

struct max77696_topint {
    struct mutex                           lock;
    struct max77696_topint_platform_data  *pdata;
    struct max77696_core                  *core;
    struct max77696_io                    *io;
    struct device                         *dev;
    struct kobject                        *kobj;

    int           irq;
    unsigned int  irq_base;
    u16           irq_unmask[TOPINT_NBANK];
    spinlock_t    irq_lock;

    unsigned long wakeup_irqs[TOPINT_IRQ_BITMAP_SZ];
    unsigned long enabled_irqs[TOPINT_IRQ_BITMAP_SZ];
    unsigned long cfg_dirty_banks[TOPINT_BANK_BITMAP_SZ];
};

#define __lock(_me)       mutex_lock(&(_me)->lock)
#define __unlock(_me)     mutex_unlock(&(_me)->lock)

/* BANK descriptor */
struct max77696_topint_bank_desc {
    u16 sts_reg;
    u16 msk_reg;
};

#define __TOPINT_BANK_DESC(_bank, _stsreg, _mskreg) \
    [_bank] = { .sts_reg = _stsreg, .msk_reg = _mskreg }

#define TOPINT_BANK_DESC(_bank) \
        __TOPINT_BANK_DESC(_bank-1, INTTOP##_bank, INTTOP##_bank##M)

static const struct max77696_topint_bank_desc max77696_topint_bank_descs[] = {
    TOPINT_BANK_DESC(1),
    TOPINT_BANK_DESC(2),
};

#define __topint_bank_desc_sts_reg(_bank) \
        (max77696_topint_bank_descs[_bank].sts_reg)
#define __topint_bank_desc_msk_reg(_bank) \
        (max77696_topint_bank_descs[_bank].msk_reg)

/* IRQ descriptor */
struct max77696_topint_irq_desc {
    char *name;
    char *devname;
    int   bank;
    u16   bitmask;   /* corresponding bit mask of irq for both status and mask
                      * registers
                      */
};

#define __TOPINT_IRQ_DESC(_id, _bank, _bitmask) \
        [MAX77696_IRQ_##_id] = {\
            .name    = #_id,\
            .devname = FILE_NAME "." #_id,\
            .bank    = _bank,\
            .bitmask = _bitmask,\
        }
#define TOPINT_IRQ_DESC(_id, _bank) \
        __TOPINT_IRQ_DESC(_id, _bank-1, INTTOP##_bank##_##_id##_INT)

static const struct max77696_topint_irq_desc max77696_topint_irq_descs[] = {
    /* Interrupt Bank #1 */
    TOPINT_IRQ_DESC(UIC   , 1),
    TOPINT_IRQ_DESC(LDO   , 1),
    TOPINT_IRQ_DESC(CHGA  , 1),
    TOPINT_IRQ_DESC(RTC   , 1),
    TOPINT_IRQ_DESC(GPIO  , 1),
    TOPINT_IRQ_DESC(FG    , 1),
    TOPINT_IRQ_DESC(BUCK  , 1),
    TOPINT_IRQ_DESC(TOPSYS, 1),
    /* Interrupt Bank #2 */
    TOPINT_IRQ_DESC(CHGB,   2),
    TOPINT_IRQ_DESC(EPD,    2),
    TOPINT_IRQ_DESC(WLED,   2),
    TOPINT_IRQ_DESC(ADC,    2),
};

#define __topint_irq_desc_name(_irq) \
        (max77696_topint_irq_descs[_irq].name)
#define __topint_irq_desc_devname(_irq) \
        (max77696_topint_irq_descs[_irq].devname)
#define __topint_irq_desc_bank(_irq) \
        (max77696_topint_irq_descs[_irq].bank)
#define __topint_irq_desc_bitmask(_irq) \
        (max77696_topint_irq_descs[_irq].bitmask)

static __always_inline
u16 max77696_topint_read_irq (struct max77696_topint *topint, int bank)
{
    u16 banksts;
    int rc;

    rc = max77696_read(topint->io, __topint_bank_desc_sts_reg(bank), &banksts);
    if (unlikely(rc)) {
        dev_err(topint->dev, "bank(%d) status read error [%d]\n", bank, rc);
        return 0;
    }

    return banksts;
}

static __always_inline
void max77696_topint_ack_irq (struct max77696_topint *topint,
    int bank, u16 bits)
{
    if (TOPINT_RWC_INTERRUPT) {
        max77696_write(topint->io, __topint_bank_desc_sts_reg(bank), bits);
    }
}

static void max77696_topint_sync_irq (struct max77696_topint *topint,
    int bank, bool force)
{
    u16 bankmsk;
    int rc;

    if (unlikely(!force && !test_bit(bank, topint->cfg_dirty_banks))) {
        return;
    }

    if (likely(max77696_core_get_drvdata(topint->core, topsys_drv))) {
        u16 irq_pin_dis = !!bitmap_empty(topint->enabled_irqs, TOPINT_NIRQ);
        rc = max77696_write_topsys_config(topint->core->dev, IRQ_PIN_DIS,
            irq_pin_dis);
        dev_dbg(topint->dev, "IRQ pin is %s [%d]\n",
            irq_pin_dis ? "disabled" : "enabled", rc);
    }

    /* write an irq masking register */
    bankmsk = ~topint->irq_unmask[bank];
    rc = max77696_write(topint->io, __topint_bank_desc_msk_reg(bank), bankmsk);
    if (unlikely(rc)) {
        dev_err(topint->dev, "bank(%d) mask write error [%d]\n", bank, rc);
        goto out;
    }

    dev_dbg(topint->dev, "wrote bank(%d) mask 0x%04X\n", bank, bankmsk);

    /* clear dirty */
    clear_bit(bank, topint->cfg_dirty_banks);

out:
    return;
}

static void max77696_topint_irq_mask (struct irq_data *data)
{
    struct max77696_topint *topint = irq_data_get_irq_chip_data(data);
    unsigned int irq = data->irq - topint->irq_base;
    u16 irq_bank = __topint_irq_desc_bank(irq);
    u16 irq_bit = __topint_irq_desc_bitmask(irq);

    if (unlikely(!test_bit(irq, topint->enabled_irqs))) {
        /* already masked */
        dev_dbg(topint->dev,
            "IRQ %s(%u) already disabled\n", __topint_irq_desc_name(irq), irq);
        return;
    }

    /* clear enabled flag */
    clear_bit(irq, topint->enabled_irqs);

    /* mask irq */
    topint->irq_unmask[irq_bank] &= ~irq_bit;

    /* set dirty */
    set_bit(irq_bank, topint->cfg_dirty_banks);

    dev_dbg(topint->dev,
        "IRQ %s(%u) disabled\n", __topint_irq_desc_name(irq), irq);
}

static void max77696_topint_irq_unmask (struct irq_data *data)
{
    struct max77696_topint *topint = irq_data_get_irq_chip_data(data);
    unsigned int irq = data->irq - topint->irq_base;
    u16 irq_bank = __topint_irq_desc_bank(irq);
    u16 irq_bit = __topint_irq_desc_bitmask(irq);

    if (unlikely(test_bit(irq, topint->enabled_irqs))) {
        /* already unmasked */
        dev_dbg(topint->dev,
            "IRQ %s(%u) already enabled\n", __topint_irq_desc_name(irq), irq);
        return;
    }

    /* set enabled flag */
    set_bit(irq, topint->enabled_irqs);

    /* unmask irq */
    topint->irq_unmask[irq_bank] |= irq_bit;

    /* set dirty */
    set_bit(irq_bank, topint->cfg_dirty_banks);

    dev_dbg(topint->dev,
        "IRQ %s(%u) enabled\n", __topint_irq_desc_name(irq), irq);
}

static void max77696_topint_irq_bus_lock (struct irq_data *data)
{
    struct max77696_topint *topint = irq_data_get_irq_chip_data(data);

    __lock(topint);
}

/*
 * genirq core code can issue chip->mask/unmask from atomic context.
 * This doesn't work for slow busses where an access needs to sleep.
 * bus_sync_unlock() is therefore called outside the atomic context,
 * syncs the current irq mask state with the slow external controller
 * and unlocks the bus.
 */

static void max77696_topint_irq_bus_sync_unlock (struct irq_data *data)
{
    struct max77696_topint *topint = irq_data_get_irq_chip_data(data);
    int i;

    for (i = 0; i < TOPINT_NBANK; i++) {
        max77696_topint_sync_irq(topint, i, false);
    }

    __unlock(topint);
}

static
int max77696_topint_irq_set_type (struct irq_data *data, unsigned int type)
{
    struct max77696_topint *topint = irq_data_get_irq_chip_data(data);

    if (unlikely(type & ~(IRQ_TYPE_EDGE_BOTH | IRQ_TYPE_LEVEL_MASK))) {
        dev_err(topint->dev, "unsupported IRQ type %d\n", type);
        return -EINVAL;
    }

    return 0;
}

static int max77696_topint_irq_set_wake (struct irq_data *data, unsigned int on)
{
    struct max77696_topint *topint = irq_data_get_irq_chip_data(data);
    unsigned int irq = data->irq - topint->irq_base;

    if (on) {
        if (unlikely(bitmap_empty(topint->wakeup_irqs, TOPINT_NIRQ))) {
            if (likely(topint->irq >= 0)) {
                enable_irq_wake((unsigned int)topint->irq);
                dev_dbg(topint->dev, "IRQ %s(%u) base %d wakeup enabled\n",
                    __topint_irq_desc_name(irq), irq, topint->irq);
            }
        }
        set_bit  (irq, topint->wakeup_irqs);
    } else {
        clear_bit(irq, topint->wakeup_irqs);
        if (unlikely(bitmap_empty(topint->wakeup_irqs, TOPINT_NIRQ))) {
            if (likely(topint->irq >= 0)) {
                disable_irq_wake((unsigned int)topint->irq);
                dev_dbg(topint->dev, "IRQ %s(%u) base %d wakeup enabled\n",
                    __topint_irq_desc_name(irq), irq, topint->irq);
            }
        }
    }

    return 0;
}

/* IRQ chip operations
 */
static struct irq_chip max77696_topint_irq_chip = {
    .name                = FILE_NAME,
//  .flags               = IRQCHIP_SET_TYPE_MASKED,
    .irq_mask            = max77696_topint_irq_mask,
    .irq_unmask          = max77696_topint_irq_unmask,
    .irq_bus_lock        = max77696_topint_irq_bus_lock,
    .irq_bus_sync_unlock = max77696_topint_irq_bus_sync_unlock,
    .irq_set_type        = max77696_topint_irq_set_type,
    .irq_set_wake        = max77696_topint_irq_set_wake,
};

static irqreturn_t max77696_topint_thread (int irq, void *data)
{
    struct max77696_topint *topint = data;
    u16 banksts[TOPINT_NBANK];
    int i;

    dev_dbg(topint->dev, "%s(%d, %p) entered\n", __func__, irq, data);

    /* Read all first-level IRQ status */
    for (i = 0; i < TOPINT_NBANK; i++) {
        banksts[i] = max77696_topint_read_irq(topint, i);
        max77696_topint_ack_irq(topint, i, ~0);
        dev_dbg(topint->dev, "bank(%d) status 0x%04X\n", i, banksts[i]);
    }

    /* Check IRQ bits */
    for (i = 0; i < TOPINT_NIRQ; i++) {
        u16 irq_bank = __topint_irq_desc_bank(i);
        u16 irq_bit  = __topint_irq_desc_bitmask(i);

        /* Comment out below -
         * to clear interrupt status register even if not enabled */
#if 0
        if (unlikely(!test_bit(i, topint->enabled_irqs))) {
            continue;
        }
#endif

        if (unlikely((banksts[irq_bank] & irq_bit) == 0)) {
            continue;
        }

        dev_dbg(topint->dev, "handle IRQ %s(%u)\n", __topint_irq_desc_name(i),
            topint->irq_base + (unsigned int)i);
        handle_nested_irq(topint->irq_base + (unsigned int)i);
    }

    return IRQ_HANDLED;
}

static ssize_t max77696_topint_inttop_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_core *core = dev_get_drvdata(dev);
    struct max77696_topint *topint =
        max77696_core_get_drvdata(core, topint_drv);
    u16 inttop[2], inttopm[2];
    int rc = 0;

    __lock(topint);

    max77696_read(topint->io, INTTOP1, &inttop[0]);
    max77696_read(topint->io, INTTOP2, &inttop[1]);

    rc += (int)snprintf(buf + rc, PAGE_SIZE, "INTTOP  0x%02X%02X\n",
        inttop[1] & 0xFF, inttop[0] & 0xFF);

    max77696_read(topint->io, INTTOP1M, &inttopm[0]);
    max77696_read(topint->io, INTTOP2M, &inttopm[1]);

    rc += (int)snprintf(buf + rc, PAGE_SIZE, "INTTOPM 0x%02X%02X\n",
        inttopm[1] & 0xFF, inttopm[0] & 0xFF);

    __unlock(topint);
    return (ssize_t)rc;
}

static DEVICE_ATTR(inttop, S_IRUGO, max77696_topint_inttop_show, NULL);

#define TOPINT_DEV_ATTR(_name) (&dev_attr_##_name.attr)

static const struct attribute *max77696_topint_attr[] = {
    TOPINT_DEV_ATTR(inttop),
};

static void max77696_topint_sysfs_remove (struct max77696_core *core)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(max77696_topint_attr); i++) {
        sysfs_remove_file_from_group(core->kobj, max77696_topint_attr[i],
            MAX77696_CORE_ATTR_GROUP_NAME);
    }
}

static void max77696_topint_sysfs_add (struct max77696_core *core)
{
    int i, rc;

    for (i = 0; i < ARRAY_SIZE(max77696_topint_attr); i++) {
        rc = sysfs_add_file_to_group(core->kobj, max77696_topint_attr[i],
            MAX77696_CORE_ATTR_GROUP_NAME);
        if (unlikely(rc)) {
            dev_err(core->dev, "failed to add attr %s [%d]\n",
                max77696_topint_attr[i]->name, rc);
        }
    }
}

static __always_inline
void max77696_topint_destroy (struct max77696_topint *topint)
{
    struct max77696_core *core = topint->core;
    struct device *dev = core->dev;

    max77696_topint_sysfs_remove(core);

    if (likely(topint->irq >= 0)) {
        devm_free_irq(dev, topint->irq, topint);
    }

    if (likely(topint->irq_base > 0)) {
        int i;

        for (i = 0; i < TOPINT_NIRQ; i++) {
            unsigned int irq = topint->irq_base + i;

            irq_set_handler(irq, NULL);
            irq_set_chip_data(irq, NULL);
        }

        irq_free_descs(topint->irq_base, TOPINT_NIRQ);
    }

    mutex_destroy(&topint->lock);
    max77696_core_set_drvdata(core, topint_drv, NULL);
    devm_kfree(dev, topint);
}

/*******************************************************************************
 * MAX77696-internal Services
 ******************************************************************************/

/* Initialize and register IRQ channel on driver probe
 */
__devinit int max77696_topint_init (struct max77696_core *core)
{
    struct device *dev = core->dev;
    struct max77696_topint_platform_data *pdata = core->pdata->topint_pdata;
    struct max77696_topint *topint;
    unsigned long irq_flags;
    int i, rc;

    pr_info(FILE_DESC" "FILE_VERSION"\n");

    if (unlikely(!pdata)) {
        dev_err(dev, "platform data is missing\n");
        return -EINVAL;
    }

    if (unlikely(pdata->irq < 0)) {
        dev_err(dev, "required resource is missing\n");
        return -EINVAL;
    }

    topint = devm_kzalloc(dev, sizeof(*topint), GFP_KERNEL);
    if (unlikely(!topint)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*topint));
        return -ENOMEM;
    }

    max77696_core_set_drvdata(core, topint_drv, topint);

    mutex_init(&topint->lock);
    topint->pdata = pdata;
    topint->core  = core;
    topint->io    = max77696_get_block_io(dev, TOPINT);
    topint->dev   = dev;
    topint->kobj  = &dev->kobj;

    spin_lock_init(&topint->irq_lock);

    /* Disable all TOP interrupts */
    bitmap_zero(topint->enabled_irqs, TOPINT_NIRQ);
    for (i = 0; i < TOPINT_NBANK; i++) {
        topint->irq_unmask[i] = 0;
        max77696_topint_sync_irq(topint, i, true);
    }

    /* Get TOP interrupt status port address & Clear status */
    for (i = 0; i < TOPINT_NBANK; i++) {
        u16 banksts;

        banksts = max77696_topint_read_irq(topint, i);
        max77696_topint_ack_irq(topint, i, ~0);

        dev_dbg(dev, "initial TOPINT bank(%d) status: 0x%04X\n", i, banksts);
    }

    rc = irq_alloc_descs(-1, pdata->irq_base, TOPINT_NIRQ, 0);
    if (unlikely(rc <= 0)) {
        dev_err(dev, "failed to alloc irq_descs [%d]\n", rc);
        goto abort;
    }

    topint->irq_base = (unsigned int)rc;
    dev_dbg(dev, "TOPINT IRQ %u-%u\n", topint->irq_base,
        topint->irq_base + TOPINT_NIRQ - 1);

    for (i = 0; i < TOPINT_NIRQ; i++) {
        unsigned int irq = topint->irq_base + (unsigned int)i;

        irq_set_chip_data(irq, topint);
        irq_set_chip_and_handler(irq, &max77696_topint_irq_chip,
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

    /* Save Level 0 IRQ (Chip IRQ) */
    topint->irq = pdata->irq;

    irq_flags  = pdata->irq_trigger;
    irq_flags |= IRQF_ONESHOT;

    BUG_ON(!TOPINT_THREADED_ISR);
    rc = devm_request_threaded_irq(dev, (unsigned int)topint->irq, NULL,
        max77696_topint_thread, irq_flags, FILE_NAME, topint);
    if (unlikely(rc)) {
        dev_err(dev, "failed to request core IRQ(%d) [%d]\n", topint->irq, rc);
        topint->irq = -1;
        goto abort;
    }

    max77696_topint_sysfs_add(core);

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_topint_destroy(topint);
    return rc;
}

/* Free IRQ channel on driver exit
 */
void max77696_topint_exit (struct max77696_core *core)
{
    struct max77696_topint *topint =
        max77696_core_get_drvdata(core, topint_drv);

    max77696_topint_destroy(topint);
    return;
}

int max77696_topint_suspend (struct max77696_core *core)
{
    return 0;
}

int max77696_topint_resume (struct max77696_core *core)
{
    return 0;
}

/*******************************************************************************
 * MAX77696-external Services
 ******************************************************************************/

/* Return system IRQ number of max77696 block
 */
int __max77696_get_block_irq (struct device *coredev,
    unsigned int block_num)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    struct max77696_topint *topint =
        max77696_core_get_drvdata(core, topint_drv);
    int rc;

    if (unlikely(!topint || IS_ERR(topint))) {
        pr_err(FILE_NAME": not ready\n");
        return -ENODEV;
    }

    __lock(topint);

    rc = __max77696_get_block_devirq(coredev, block_num);
    if (unlikely((unsigned)rc >= TOPINT_NIRQ)) {
        pr_err(FILE_NAME": invalid TOPINT IRQ number - %d\n", rc);
        rc = -EINVAL;
        goto out;
    }

    rc += (int)topint->irq_base;

out:
    __unlock(topint);
    return rc;
}
EXPORT_SYMBOL(__max77696_get_block_irq);

int __max77696_set_block_irq (struct device *coredev,
    unsigned int block_num, int irq)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    struct max77696_topint *topint =
        max77696_core_get_drvdata(core, topint_drv);
    int rc;

    if (unlikely(!topint || IS_ERR(topint))) {
        pr_err(FILE_NAME": not ready\n");
        return -ENODEV;
    }

    __lock(topint);

    if (unlikely(irq <= (int)topint->irq_base)) {
        pr_err(FILE_NAME": invalid TOPINT IRQ number - %d\n", irq);
        rc = -EINVAL;
        goto out;
    }

    irq -= (int)topint->irq_base;

    rc = __max77696_set_block_devirq(coredev, block_num, irq);
    if (unlikely(rc)) {
        goto out;
    }

out:
    __unlock(topint);
    return rc;
}
EXPORT_SYMBOL(__max77696_set_block_irq);
