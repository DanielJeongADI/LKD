/*
 * MAX77696 TOPSYS Support
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

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/irq.h>
#include <linux/mfd/max77696.h>
#include "max77696-core.h"

#define FILE_DESC    MAX77696_DESC" TOPSYS Support"
#define FILE_NAME    MAX77696_TOPSYS_NAME
#define FILE_VERSION MAX77696_CORE_VERSION
#define FILE_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define TOPSYS_RWC_INTERRUPT         0

#define TOPSYS_NBANK                 1
#define TOPSYS_NIRQ                  MAX77696_NUM_OF_TOPSYS_IRQS

#define GLBLCNFG0                    0x00
#define GLBLCNFG0_PTP                BIT (6) /* Pull The Plug */
#define GLBLCNFG0_FSENT              BIT (5) /* Factory Ship Enter */
#define GLBLCNFG0_FRSTRT             BIT (4) /* Full Restart */
#define GLBLCNFG0_PRSTRT             BIT (3) /* Partial Restart */
#define GLBLCNFG0_FSHDN              BIT (2) /* Full Shutdown */
#define GLBLCNFG0_PSHDN              BIT (1) /* Partial Shutdown */
#define GLBLCNFG0_SFTPDRR            BIT (0) /* Software Power Down
                                              * Register Reset */

#define GLBLCNFG1                    0x01
#define GLBLCNFG1_GLBL_LPM           BIT (6)
//      GLBLCNFG1_MREN               BIT (5)
#define GLBLCNFG1_MRT                BITS(4,2)
#define GLBLCNFG1_EN0DLY             BIT (1)
#define GLBLCNFG1_STBYEN             BIT (0)

#define GLBLCNFG2                    0x02
#define GLBLCNFG2_WDTEN              BIT (6)
#define GLBLCNFG2_TWD                BITS(5,4)
#define GLBLCNFG2_RTCAWK             BIT (3)
#define GLBLCNFG2_WDWK               BIT (2)
#define GLBLCNFG2_MROWK              BIT (1)
#define GLBLCNFG2_UICWK_EDGE         BIT (0)

#define GLBLCNFG3                    0x03
#define GLBLCNFG3_LBHYST             BITS(4,3)
#define GLBLCNFG3_LBDAC              BITS(2,0)

#define GLBLCNFG4                    0x04
#define GLBLCNFG4_WDTC               BITS(1,0)

#define GLBLCNFG5                    0xAF
#define GLBLCNFG5_NRSO_DEL           BITS(1,0)
#define GLBLCNFG5_POOT               BITS(3,2)

#define GLBLCNFG6                    0xB6

#define GLBLINT                      0x05
#define GLBLINTM                     0x06
#define GLBLINTM_IRQ_PIN_DIS         BIT (0)

#define GLBLINT_EN0_RISING           BIT (7)
#define GLBLINT_EN0_FALLING          BIT (6)
#define GLBLINT_EN0_1SEC             BIT (5)
#define GLBLINT_MR_WARNING           BIT (4)
#define GLBLINT_BATT_LOW             BIT (3)
#define GLBLINT_THERM_ALARM_0        BIT (2)
#define GLBLINT_THERM_ALARM_1        BIT (1)

#define GLBLSTAT                     0x07

#define ERCFLAG0                     0xAA
#define ERCFLAG0_WDPMIC_FSHDN        BIT (0)
#define ERCFLAG0_WDPMIC_FRSTRT       BIT (1)
#define ERCFLAG0_MR_FSHDN            BIT (2)
#define ERCFLAG0_MR_FRSTRT           BIT (3)
#define ERCFLAG0_SFT_PSHDN           BIT (4)
#define ERCFLAG0_SFT_PRSTRT          BIT (5)
#define ERCFLAG0_SFT_FSHDN           BIT (6)
#define ERCFLAG0_SFT_FRSTRT          BIT (7)

#define ERCFLAG1                     0xAB
#define ERCFLAG1_LBMOK_FSHDN         BIT (0)
#define ERCFLAG1_SYS1UVLO_FSHDN      BIT (1)
#define ERCFLAG1_TOVLO_FSHDN         BIT (2)
#define ERCFLAG1_RSTIN_PRSTRT        BIT (3)

#define TOPSYS_INT_BITMAP_SZ         BITS_TO_LONGS(TOPSYS_NIRQ)

struct max77696_topsys {
    struct mutex                          lock;
    struct max77696_topsys_platform_data *pdata;
    struct max77696_core                 *core;
    struct max77696_io                   *io;
    struct device                        *dev;
    struct kobject                       *kobj;

    int                                   irq;
    unsigned int                          irq_base;
    u16                                   irq_unmask;

    unsigned long                         wakeup_irqs[TOPSYS_INT_BITMAP_SZ];
    unsigned long                         enabled_irqs[TOPSYS_INT_BITMAP_SZ];
    unsigned long                         cfg_dirty_irqs[TOPSYS_INT_BITMAP_SZ];

    u16                                   ercflag;
};

#define __lock(_me)       mutex_lock(&(_me)->lock)
#define __unlock(_me)     mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

static u16 max77696_ercflag = 0;
static __init int max77696_topsys_setup_ercflag (char *str)
{
    /* ercflag0, ercflag1 */
    int ercflag[3];
    get_options(str, 3, ercflag);
    max77696_ercflag = ((u16)ercflag[2] << 8) | ercflag[1];
    return 1;
}
__setup(MAX77696_NAME"_ercflag=", max77696_topsys_setup_ercflag);

static const char *ercflag0_bit_string[] = {
    [0] = "PMIC System Watchdog Full Shutdown",
    [1] = "PMIC System Watchdog Full Restart",
    [2] = "Manual Reset Full Shutdown",
    [3] = "Manual Reset Partial Restart",
    [4] = "Software Partial Shutdown",
    [5] = "Software Partial Restart",
    [6] = "Software Full Shutdown",
    [7] = "Software Full Restart",
};

static const char *ercflag1_bit_string[] = {
    [0] = "Low-Battery Monitor Not Okay Full Shutdown",
    [1] = "System 1 Undervoltage Full Shutdown",
    [2] = "Thermal Overload Full Shutdown",
    [3] = "Reset Input Partial Restart",
};

/* TOPSYS_IRQ descriptor */
struct max77696_topsys_irq_desc {
    char *name;
    u16   bitmask;   /* corresponding bit mask of irq for both status and mask
                      * registers
                      */
};

#define __TOPSYS_IRQ_DESC(_id, _bitmask) \
        [MAX77696_TOPSYS_IRQ_##_id] = {\
            .name    = #_id,\
            .bitmask = _bitmask,\
        }
#define TOPSYS_IRQ_DESC(_id) \
        __TOPSYS_IRQ_DESC(_id, GLBLINT_##_id)

static struct max77696_topsys_irq_desc max77696_topsys_irq_descs[] = {
    TOPSYS_IRQ_DESC(THERM_ALARM_1),
    TOPSYS_IRQ_DESC(THERM_ALARM_0),
    TOPSYS_IRQ_DESC(BATT_LOW     ),
    TOPSYS_IRQ_DESC(MR_WARNING   ),
    TOPSYS_IRQ_DESC(EN0_1SEC     ),
    TOPSYS_IRQ_DESC(EN0_FALLING  ),
    TOPSYS_IRQ_DESC(EN0_RISING   ),
};

#define __topsys_irq_desc_name(_irq) \
        (max77696_topsys_irq_descs[_irq].name)
#define __topsys_irq_desc_bitmask(_irq) \
        (max77696_topsys_irq_descs[_irq].bitmask)

static __always_inline
u16 max77696_topsys_read_irq (struct max77696_topsys *topsys)
{
    u16 glblint;
    int rc;

    rc = max77696_read(topsys->io, GLBLINT, &glblint);
    if (unlikely(rc)) {
        dev_err(topsys->dev, "GLBLINT read error [%d]\n", rc);
        return 0;
    }

    return glblint;
}

static __always_inline
void max77696_topsys_ack_irq (struct max77696_topsys *topsys)
{
    if (TOPSYS_RWC_INTERRUPT) {
        max77696_write(topsys->io, GLBLINT, ~0);
    }
}

static void max77696_topsys_irq_mask (struct irq_data *data)
{
    struct max77696_topsys *topsys = irq_data_get_irq_chip_data(data);
    unsigned int irq = data->irq - topsys->irq_base;
    u16 irq_bit = __topsys_irq_desc_bitmask(irq);

    if (unlikely(!test_bit(irq, topsys->enabled_irqs))) {
        /* already masked */
        return;
    }

    /* clear enabled flag */
    clear_bit(irq, topsys->enabled_irqs);

    /* mask irq */
    topsys->irq_unmask &= ~irq_bit;

    /* set dirty */
    set_bit(irq, topsys->cfg_dirty_irqs);
}

static void max77696_topsys_irq_unmask (struct irq_data *data)
{
    struct max77696_topsys *topsys = irq_data_get_irq_chip_data(data);
    unsigned int irq = data->irq - topsys->irq_base;
    u16 irq_bit = __topsys_irq_desc_bitmask(irq);

    if (unlikely(test_bit(irq, topsys->enabled_irqs))) {
        /* already unmasked */
        return;
    }

    /* set enabled flag */
    set_bit(irq, topsys->enabled_irqs);

    /* unmask irq */
    topsys->irq_unmask |= irq_bit;

    /* set dirty */
    set_bit(irq, topsys->cfg_dirty_irqs);
}

static void max77696_topsys_irq_bus_lock (struct irq_data *data)
{
    struct max77696_topsys *topsys = irq_data_get_irq_chip_data(data);

    __lock(topsys);
}

/*
 * genirq core code can issue chip->mask/unmask from atomic context.
 * This doesn't work for slow busses where an access needs to sleep.
 * bus_sync_unlock() is therefore called outside the atomic context,
 * syncs the current irq mask state with the slow external controller
 * and unlocks the bus.
 */

static void max77696_topsys_irq_bus_sync_unlock (struct irq_data *data)
{
    struct max77696_topsys *topsys = irq_data_get_irq_chip_data(data);
    u16 irq_pin_dis, glblintm = 0;
    int rc;

    if (unlikely(bitmap_empty(topsys->cfg_dirty_irqs, TOPSYS_NIRQ))) {
        goto out;
    }

//  disable_irq(topsys->irq);

    rc = max77696_read(topsys->io, GLBLINTM, &glblintm);
    if (unlikely(rc)) {
        dev_err(topsys->dev, "GLBLINTM read error [%d]\n", rc);
        goto out;
    }

    dev_dbg(topsys->dev, "read  GLBLINTM 0x%04X\n", glblintm);
    irq_pin_dis = BITS_GET(glblintm, GLBLINTM_IRQ_PIN_DIS);
    glblintm    = ~topsys->irq_unmask;
    glblintm    = BITS_SET(glblintm, GLBLINTM_IRQ_PIN_DIS, irq_pin_dis);

    rc = max77696_write(topsys->io, GLBLINTM, glblintm);
    if (unlikely(rc)) {
        dev_err(topsys->dev, "GLBLINTM write error [%d]\n", rc);
        goto out;
    }

    dev_dbg(topsys->dev, "wrote GLBLINTM 0x%04X\n", glblintm);

    /* clear dirty flags */
    bitmap_zero(topsys->cfg_dirty_irqs, TOPSYS_NIRQ);

//  if (likely(!bitmap_empty(topsys->enabled_irqs, TOPSYS_NIRQ))) {
//      enable_irq(topsys->irq);
//  }

out:
    __unlock(topsys);
}

static int max77696_topsys_irq_set_type (struct irq_data *data,
        unsigned int type)
{
    struct max77696_topsys *topsys = irq_data_get_irq_chip_data(data);

    if (unlikely(type & ~(IRQ_TYPE_EDGE_BOTH | IRQ_TYPE_LEVEL_MASK))) {
        dev_err(topsys->dev, "unsupported irq type %d\n", type);
        return -EINVAL;
    }

    return 0;
}

static int max77696_topsys_irq_set_wake (struct irq_data *data,
    unsigned int on)
{
    struct max77696_topsys *topsys = irq_data_get_irq_chip_data(data);
    unsigned int irq = data->irq - topsys->irq_base;

    if (on) {
        if (unlikely(bitmap_empty(topsys->wakeup_irqs, TOPSYS_NIRQ))) {
            enable_irq_wake(topsys->irq);
        }
        set_bit  (irq, topsys->wakeup_irqs);
    } else {
        clear_bit(irq, topsys->wakeup_irqs);
        if (unlikely(bitmap_empty(topsys->wakeup_irqs, TOPSYS_NIRQ))) {
            disable_irq_wake(topsys->irq);
        }
    }

    return 0;
}

/* IRQ chip operations
 */
static struct irq_chip max77696_topsys_irq_chip = {
    .name                = FILE_NAME,
//  .flags               = IRQCHIP_SET_TYPEED,
    .irq_mask            = max77696_topsys_irq_mask,
    .irq_unmask          = max77696_topsys_irq_unmask,
    .irq_bus_lock        = max77696_topsys_irq_bus_lock,
    .irq_bus_sync_unlock = max77696_topsys_irq_bus_sync_unlock,
    .irq_set_type        = max77696_topsys_irq_set_type,
    .irq_set_wake        = max77696_topsys_irq_set_wake,
};

static irqreturn_t max77696_topsys_isr (int irq, void *data)
{
    struct max77696_topsys *topsys = data;
    u16 glblint;
    int i;

    glblint = max77696_topsys_read_irq(topsys);
    max77696_topsys_ack_irq(topsys);
    dev_dbg(topsys->dev, "GLBLINT 0x%04X\n", glblint);

    for (i = 0; i < TOPSYS_NIRQ; i++) {
        u16 irq_bit = __topsys_irq_desc_bitmask(i);

        if (unlikely(!test_bit(i, topsys->enabled_irqs))) {
            /* if the irq is disabled, then ignore a below process */
            continue;
        }

        if (unlikely((glblint & irq_bit) == 0)) {
            continue;
        }

        dev_dbg(topsys->dev, "handle IRQ %s(%u)\n", __topsys_irq_desc_name(i),
            topsys->irq_base + (unsigned int)i);
        handle_nested_irq(topsys->irq_base + (unsigned int)i);
    }

    return IRQ_HANDLED;
}

/*** TOPSYS Attributes ***/

#define DEFINE_TOPSYS_CNFG_DEV_ATTR(_cnfg) \
static ssize_t max77696_topsys_##_cnfg##_show (struct device *dev,\
    struct device_attribute *devattr, char *buf)\
{\
    struct max77696_core *core = dev_get_drvdata(dev);\
    u16 cnfg = 0;\
    int rc;\
    rc = max77696_read_topsys_config(core->dev, _cnfg, &cnfg);\
    if (unlikely(rc)) {\
        dev_err(dev, ""#_cnfg" read error [%d]\n", rc);\
        rc = (int)snprintf(buf, PAGE_SIZE, "error [%d]\n", rc);\
        goto out;\
    }\
    rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", cnfg);\
out:\
    return (ssize_t)rc;\
}\
static ssize_t max77696_topsys_##_cnfg##_store (struct device *dev,\
    struct device_attribute *devattr, const char *buf, size_t count)\
{\
    struct max77696_core *core = dev_get_drvdata(dev);\
    u16 cnfg;\
    int rc;\
    cnfg = (u16)simple_strtoul(buf, NULL, 10);\
    rc = max77696_write_topsys_config(core->dev, _cnfg, cnfg);\
    if (unlikely(rc)) {\
        dev_err(dev, ""#_cnfg" write error [%d]\n", rc);\
        goto out;\
    }\
out:\
    return (ssize_t)count;\
}\
static DEVICE_ATTR(_cnfg, S_IWUSR|S_IRUGO, max77696_topsys_##_cnfg##_show,\
    max77696_topsys_##_cnfg##_store)

  DEFINE_TOPSYS_CNFG_DEV_ATTR(IRQ_PIN_DIS);
  DEFINE_TOPSYS_CNFG_DEV_ATTR(PTP        );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(FSENT      );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(FRSTRT     );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(PRSTRT     );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(FSHDN      );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(PSHDN      );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(SFTPDRR    );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(GLBL_LPM   );
//DEFINE_TOPSYS_CNFG_DEV_ATTR(MREN       );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(MRT        );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(EN0DLY     );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(STBYEN     );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(WDTEN      );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(TWD        );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(RTCAWK     );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(WDWK       );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(MROWK      );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(UICWK_EDGE );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(LBHYST     );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(LBDAC      );
  DEFINE_TOPSYS_CNFG_DEV_ATTR(WDTC       );

static ssize_t max77696_topsys_dump_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_core *core = dev_get_drvdata(dev);
    struct max77696_topsys *topsys =
        max77696_core_get_drvdata(core, topsys_drv);
    u16 val;
    int rc = 0;

    __lock(topsys);

    max77696_read(topsys->io, GLBLSTAT, &val);
    rc += (int)snprintf(buf + rc, PAGE_SIZE, "GLBLSTAT  0x%04X\n", val);

    max77696_read(topsys->io, GLBLINTM, &val);
    rc += (int)snprintf(buf + rc, PAGE_SIZE, "GLBLINTM  0x%04X\n", val);

    max77696_read(topsys->io, GLBLCNFG0, &val);
    rc += (int)snprintf(buf + rc, PAGE_SIZE, "GLBLCNFG0 0x%04X\n", val);

    max77696_read(topsys->io, GLBLCNFG1, &val);
    rc += (int)snprintf(buf + rc, PAGE_SIZE, "GLBLCNFG1 0x%04X\n", val);

    max77696_read(topsys->io, GLBLCNFG2, &val);
    rc += (int)snprintf(buf + rc, PAGE_SIZE, "GLBLCNFG2 0x%04X\n", val);

    max77696_read(topsys->io, GLBLCNFG3, &val);
    rc += (int)snprintf(buf + rc, PAGE_SIZE, "GLBLCNFG3 0x%04X\n", val);

    max77696_read(topsys->io, GLBLCNFG4, &val);
    rc += (int)snprintf(buf + rc, PAGE_SIZE, "GLBLCNFG4 0x%04X\n", val);

    max77696_read(topsys->io, GLBLCNFG5, &val);
    rc += (int)snprintf(buf + rc, PAGE_SIZE, "GLBLCNFG5 0x%04X\n", val);

    max77696_read(topsys->io, GLBLCNFG6, &val);
    rc += (int)snprintf(buf + rc, PAGE_SIZE, "GLBLCNFG6 0x%04X\n", val);

    __unlock(topsys);
    return (ssize_t)rc;
}

static DEVICE_ATTR(dump, S_IRUGO, max77696_topsys_dump_show, NULL);

static ssize_t max77696_topsys_ercflag_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_core *core = dev_get_drvdata(dev);
    struct max77696_topsys *topsys =
        max77696_core_get_drvdata(core, topsys_drv);
    u16 ercflag0, ercflag1;
    int i, rc = 0;

    __lock(topsys);

    ercflag0 = ((topsys->ercflag >> 0) & 0xFF);
    ercflag1 = ((topsys->ercflag >> 8) & 0xFF);

    rc += (int)snprintf(buf + rc, PAGE_SIZE, "ERCFLAG0 0x%04X\n",
        ercflag0);

    for (i = 0; i < ARRAY_SIZE(ercflag0_bit_string); i++) {
        if (unlikely(!ercflag0_bit_string[i])) {
            continue;
        }
        rc += (int)snprintf(buf + rc, PAGE_SIZE, "  %s %s\n",
            (ercflag0 & (1 << i)) ? "o" : "-", ercflag0_bit_string[i]);
    }

    rc += (int)snprintf(buf + rc, PAGE_SIZE, "ERCFLAG1 0x%04X\n",
        ercflag1);

    for (i = 0; i < ARRAY_SIZE(ercflag1_bit_string); i++) {
        if (unlikely(!ercflag1_bit_string[i])) {
            continue;
        }
        rc += (int)snprintf(buf + rc, PAGE_SIZE, "  %s %s\n",
            (ercflag1 & (1 << i)) ? "o" : "-", ercflag1_bit_string[i]);
    }

    __unlock(topsys);
    return (ssize_t)rc;
}

static DEVICE_ATTR(ercflag, S_IRUGO, max77696_topsys_ercflag_show, NULL);

#define TOPSYS_DEV_ATTR(_name)      (&dev_attr_##_name.attr)
#define TOPSYS_CNFG_DEV_ATTR(_cnfg) TOPSYS_DEV_ATTR(_cnfg)

static const struct attribute *max77696_topsys_attr[] = {
    TOPSYS_DEV_ATTR(dump   ),
    TOPSYS_DEV_ATTR(ercflag),

    TOPSYS_CNFG_DEV_ATTR(IRQ_PIN_DIS),
    TOPSYS_CNFG_DEV_ATTR(PTP        ),
    TOPSYS_CNFG_DEV_ATTR(FSENT      ),
    TOPSYS_CNFG_DEV_ATTR(FRSTRT     ),
    TOPSYS_CNFG_DEV_ATTR(PRSTRT     ),
    TOPSYS_CNFG_DEV_ATTR(FSHDN      ),
    TOPSYS_CNFG_DEV_ATTR(PSHDN      ),
    TOPSYS_CNFG_DEV_ATTR(SFTPDRR    ),
    TOPSYS_CNFG_DEV_ATTR(GLBL_LPM   ),
  //TOPSYS_CNFG_DEV_ATTR(MREN       ),
    TOPSYS_CNFG_DEV_ATTR(MRT        ),
    TOPSYS_CNFG_DEV_ATTR(EN0DLY     ),
    TOPSYS_CNFG_DEV_ATTR(STBYEN     ),
    TOPSYS_CNFG_DEV_ATTR(WDTEN      ),
    TOPSYS_CNFG_DEV_ATTR(TWD        ),
    TOPSYS_CNFG_DEV_ATTR(RTCAWK     ),
    TOPSYS_CNFG_DEV_ATTR(WDWK       ),
    TOPSYS_CNFG_DEV_ATTR(MROWK      ),
    TOPSYS_CNFG_DEV_ATTR(UICWK_EDGE ),
    TOPSYS_CNFG_DEV_ATTR(LBHYST     ),
    TOPSYS_CNFG_DEV_ATTR(LBDAC      ),
    TOPSYS_CNFG_DEV_ATTR(WDTC       ),
};

static void max77696_topsys_sysfs_remove (struct max77696_core *core)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(max77696_topsys_attr); i++) {
        sysfs_remove_file_from_group(core->kobj, max77696_topsys_attr[i],
            MAX77696_CORE_ATTR_GROUP_NAME);
    }
}

static void max77696_topsys_sysfs_add (struct max77696_core *core)
{
    int i, rc;

    for (i = 0; i < ARRAY_SIZE(max77696_topsys_attr); i++) {
        rc = sysfs_add_file_to_group(core->kobj, max77696_topsys_attr[i],
            MAX77696_CORE_ATTR_GROUP_NAME);
        if (unlikely(rc)) {
            dev_err(core->dev, "failed to add attr %s [%d]\n",
                max77696_topsys_attr[i]->name, rc);
        }
    }
}

/*** TOPSYS Init/Exit ***/

static __always_inline
void max77696_topsys_destroy (struct max77696_topsys *topsys)
{
    struct max77696_core *core = topsys->core;
    struct device *dev = topsys->dev;
    int i;

    max77696_topsys_sysfs_remove(core);

    if (likely(topsys->irq > 0)) {
        devm_free_irq(dev, topsys->irq, topsys);
    }

    if (likely(topsys->irq_base > 0)) {
        for (i = 0; i < TOPSYS_NIRQ; i++) {
            unsigned int irq = topsys->irq_base + i;

            irq_set_handler(irq, NULL);
            irq_set_chip_data(irq, NULL);
        }

        irq_free_descs(topsys->irq_base, TOPSYS_NIRQ);
    }

    mutex_destroy(&topsys->lock);
    max77696_core_set_drvdata(core, topsys_drv, NULL);
    devm_kfree(dev, topsys);
}

/*******************************************************************************
 * MAX77696-internal Services
 ******************************************************************************/

__devinit int max77696_topsys_init (struct max77696_core *core)
{
    struct device *dev = core->dev;
    struct max77696_topsys_platform_data *pdata = core->pdata->topsys_pdata;
    struct max77696_topsys *topsys;
    u16 topsysint, ercflag0, ercflag1;
    int i, rc;

    pr_info(FILE_DESC" "FILE_VERSION"\n");

    if (unlikely(!pdata)) {
        dev_err(dev, "platform data is missing\n");
        return -EINVAL;
    }

    topsys = devm_kzalloc(dev, sizeof(*topsys), GFP_KERNEL);
    if (unlikely(!topsys)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*topsys));
        return -ENOMEM;
    }

    max77696_core_set_drvdata(core, topsys_drv, topsys);

    mutex_init(&topsys->lock);
    topsys->pdata = pdata;
    topsys->core  = core;
    topsys->io    = max77696_get_block_io(dev, TOPSYS);
    topsys->dev   = dev;
    topsys->kobj  = &dev->kobj;

    /* Disable all TOPSYS interrupts */
    bitmap_zero(topsys->enabled_irqs, TOPSYS_NIRQ);
    topsys->irq_unmask = 0;
    max77696_write(topsys->io, GLBLINTM, ~0);

    /* Get TOPSYS interrupt status port address & Clear status */
    topsysint = max77696_topsys_read_irq(topsys);
    max77696_topsys_ack_irq(topsys);
    dev_dbg(dev, "initial TOPSYS interrupt status: 0x%04X\n", topsysint);

    for (i = 0; i < TOPSYS_NIRQ; i++) {
        dev_dbg(dev, "    %-13s  %s\n", __topsys_irq_desc_name(i),
            (topsysint & __topsys_irq_desc_bitmask(i))? "o" : "-");
    }

    rc = irq_alloc_descs(-1, pdata->irq_base, TOPSYS_NIRQ, 0);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to alloc irq_descs [%d]\n", rc);
        goto abort;
    }

    topsys->irq_base = (unsigned int)rc;
    dev_dbg(dev, "TOPSYS IRQ %u-%u\n", topsys->irq_base,
        topsys->irq_base + TOPSYS_NIRQ - 1);

    for (i = 0; i < TOPSYS_NIRQ; i++) {
        unsigned int irq = topsys->irq_base + i;

        irq_set_chip_data(irq, topsys);
        irq_set_chip_and_handler(irq, &max77696_topsys_irq_chip,
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

    /* Get TOPSYS block IRQ number */
    topsys->irq = max77696_get_block_irq(dev, TOPSYS);
    BUG_ON(topsys->irq <= 0);

    /* Request system IRQ for TOPSYS */
    rc = devm_request_threaded_irq(dev, (unsigned int)topsys->irq, NULL,
        max77696_topsys_isr, IRQF_ONESHOT, FILE_NAME, topsys);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to request IRQ(%d) [%d]\n", topsys->irq, rc);
        topsys->irq = 0;
        goto abort;
    }

    dev_dbg(dev, "IRQ(%d) requested\n", topsys->irq);

    //disable_irq(topsys->irq);

    max77696_topsys_sysfs_add(core);

    max77696_read(topsys->io, ERCFLAG0, &ercflag0);
    max77696_read(topsys->io, ERCFLAG1, &ercflag1);
    topsys->ercflag |= ((ercflag1 << 8) | ercflag0);
    topsys->ercflag |= max77696_ercflag;
    dev_info(dev, "ERCFLAG [1] 0x%04X [0] 0x%04X\n", ercflag1, ercflag0);

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_topsys_destroy(topsys);
    return rc;
}

__devexit void max77696_topsys_exit (struct max77696_core *core)
{
    struct max77696_topsys *topsys =
        max77696_core_get_drvdata(core, topsys_drv);

    max77696_topsys_destroy(topsys);
    return;
}

int max77696_topsys_suspend (struct max77696_core *core)
{
    return 0;
}

int max77696_topsys_resume (struct max77696_core *core)
{
    return 0;
}

/*******************************************************************************
 * MAX77696-external Services
 ******************************************************************************/

/* Return system IRQ number of max77696 TOPSYS interrupt
 */
int __max77696_get_topsys_irq (struct device *coredev, unsigned int topsys_irq)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    struct max77696_topsys *topsys =
        max77696_core_get_drvdata(core, topsys_drv);
    int rc;

    if (unlikely(!topsys || IS_ERR(topsys))) {
        pr_err(FILE_NAME": not ready\n");
        return -ENODEV;
    }

    __lock(topsys);

    if (unlikely(topsys_irq >= TOPSYS_NIRQ)) {
        pr_err(FILE_NAME": invalid TOPSYS IRQ number - %u\n", topsys_irq);
        rc = -EINVAL;
        goto out;
    }

    rc = (int)topsys->irq_base + (int)topsys_irq;

out:
    __unlock(topsys);
    return rc;
}
EXPORT_SYMBOL(__max77696_get_topsys_irq);

#define TOPSYS_STAT_BITDESC(_sts, _sts_reg) \
        [MAX77696_TOPSYS_STAT_##_sts] =\
            MAX77696_BITDESC(_sts_reg, _sts_reg##_##_sts)

static const struct max77696_bitdesc max77696_topsys_stat_bitdesc[] = {
};

#define __topsys_stat_bitdesc(_sts) (&max77696_topsys_stat_bitdesc[_sts])

int __max77696_read_topsys_stat (struct device *coredev, int sts, u16 *val)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    struct max77696_topsys *topsys =
        max77696_core_get_drvdata(core, topsys_drv);
    int rc;

    if (unlikely(!topsys || IS_ERR(topsys))) {
        pr_err(FILE_NAME": not ready\n");
        return -ENODEV;
    }

    __lock(topsys);

    rc = max77696_read_bitdesc(topsys->io, __topsys_stat_bitdesc(sts), val);
    if (unlikely(rc)) {
        pr_err(FILE_NAME": failed to read TOPSYS status(%d) [%d]\n", sts, rc);
        goto out;
    }

    pr_debug(FILE_NAME": TOPSYS status(%d) read = 0x%04Xh\n", sts, *val);

out:
    __unlock(topsys);
    return rc;
}
EXPORT_SYMBOL(__max77696_read_topsys_stat);

#define TOPSYS_CNFG_BITDESC(_cfg, _cfg_reg) \
        [MAX77696_TOPSYS_CNFG_##_cfg] = \
            MAX77696_BITDESC(_cfg_reg, _cfg_reg##_##_cfg)

static const struct max77696_bitdesc max77696_topsys_cnfg_bitdesc[] = {
    TOPSYS_CNFG_BITDESC(IRQ_PIN_DIS, GLBLINTM ),

    TOPSYS_CNFG_BITDESC(PTP        , GLBLCNFG0),
    TOPSYS_CNFG_BITDESC(FSENT      , GLBLCNFG0),
    TOPSYS_CNFG_BITDESC(FRSTRT     , GLBLCNFG0),
    TOPSYS_CNFG_BITDESC(PRSTRT     , GLBLCNFG0),
    TOPSYS_CNFG_BITDESC(FSHDN      , GLBLCNFG0),
    TOPSYS_CNFG_BITDESC(PSHDN      , GLBLCNFG0),
    TOPSYS_CNFG_BITDESC(SFTPDRR    , GLBLCNFG0),

    TOPSYS_CNFG_BITDESC(GLBL_LPM   , GLBLCNFG1),
//  TOPSYS_CNFG_BITDESC(MREN       , GLBLCNFG1),
    TOPSYS_CNFG_BITDESC(MRT        , GLBLCNFG1),
    TOPSYS_CNFG_BITDESC(EN0DLY     , GLBLCNFG1),
    TOPSYS_CNFG_BITDESC(STBYEN     , GLBLCNFG1),

    TOPSYS_CNFG_BITDESC(WDTEN      , GLBLCNFG2),
    TOPSYS_CNFG_BITDESC(TWD        , GLBLCNFG2),
    TOPSYS_CNFG_BITDESC(RTCAWK     , GLBLCNFG2),
    TOPSYS_CNFG_BITDESC(WDWK       , GLBLCNFG2),
    TOPSYS_CNFG_BITDESC(MROWK      , GLBLCNFG2),
    TOPSYS_CNFG_BITDESC(UICWK_EDGE , GLBLCNFG2),

    TOPSYS_CNFG_BITDESC(LBHYST     , GLBLCNFG3),
    TOPSYS_CNFG_BITDESC(LBDAC      , GLBLCNFG3),

    TOPSYS_CNFG_BITDESC(WDTC       , GLBLCNFG4),
};

#define __topsys_cnfg_bitdesc(_cfg) (&max77696_topsys_cnfg_bitdesc[_cfg])

int __max77696_read_topsys_config (struct device *coredev, int cfg, u16 *val)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    struct max77696_topsys *topsys =
        max77696_core_get_drvdata(core, topsys_drv);
    int rc;

    if (unlikely(!topsys || IS_ERR(topsys))) {
        pr_err(FILE_NAME": not ready\n");
        return -ENODEV;
    }

    __lock(topsys);

    rc = max77696_read_bitdesc(topsys->io, __topsys_cnfg_bitdesc(cfg), val);
    if (unlikely(rc)) {
        pr_err(FILE_NAME": failed to read TOPSYS config(%d) [%d]\n", cfg, rc);
        goto out;
    }

    pr_debug(FILE_NAME": TOPSYS config(%d) read = 0x%04Xh\n", cfg, *val);

out:
    __unlock(topsys);
    return rc;
}
EXPORT_SYMBOL(__max77696_read_topsys_config);

int __max77696_write_topsys_config (struct device *coredev, int cfg, u16 val)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    struct max77696_topsys *topsys =
        max77696_core_get_drvdata(core, topsys_drv);
    int rc;

    if (unlikely(!topsys || IS_ERR(topsys))) {
        pr_err(FILE_NAME": not ready\n");
        return -ENODEV;
    }

    __lock(topsys);

    rc = max77696_write_bitdesc(topsys->io, __topsys_cnfg_bitdesc(cfg), val);
    if (unlikely(rc)) {
        pr_err(FILE_NAME": failed to write TOPSYS config(%d) [%d]\n", cfg, rc);
        goto out;
    }

    switch (cfg) {
    case MAX77696_TOPSYS_CNFG_FSENT:
        /* At least 32msec delay to complete some housekeeping tasks before the
         * FSENT state starts turning regulators off
         */
        __msleep(50);
        break;
    }

out:
    __unlock(topsys);
    return rc;
}
EXPORT_SYMBOL(__max77696_write_topsys_config);

int max77696_test_ercflag (struct device *coredev, u16 ercflags)
{
    struct max77696_core *core = dev_get_drvdata(coredev);
    struct max77696_topsys *topsys =
        max77696_core_get_drvdata(core, topsys_drv);
    int rc;

    if (unlikely(!topsys || IS_ERR(topsys))) {
        pr_err(FILE_NAME": not ready\n");
        return -ENODEV;
    }

    __lock(topsys);

    rc = (int)(topsys->ercflag & ercflags);

    __unlock(topsys);
    return rc;
}
EXPORT_SYMBOL(max77696_test_ercflag);
