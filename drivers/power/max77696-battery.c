/*
 * MAX77696 FuelGauge Driver
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

#include <linux/power_supply.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    MAX77696_DESC" FG Driver"
#define DRIVER_NAME    MAX77696_FG_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define FG_LPM_WORKAROUNDS

#define FG_RWC_INTERRUPT        0
#define FG_INIT_DELAY           msecs_to_jiffies(0)
#define FG_DEFAULT_RSENSE       10     /* milli-ohms */
#define FG_MODEL_SCALING        1
#define FG_SOC_FULL             0x5F00 /*   95 %  (for STATUS_FULL) */
#define FG_VCELL_DEAD           0xA000 /* 3200 mV (for HEALTH_DEAD) */
#define FG_VCELL_OVER           0xE100 /* 4500 mV (for HEALTH_OVER) */

#define GAUGE_DEFAULT_SNS_RESISTOR_UOHM    10000 /* micro-ohms */
#define GAUGE_DEFAULT_UPD_INTERVAL_MSEC    5000
#define GAUGE_DEFAULT_UPD_INTERVAL_JIFFIES \
        msecs_to_jiffies(GAUGE_DEFAULT_UPD_INTERVAL_MSEC)

#define STATUS                0x00
#define STATUS_POR            BIT ( 1)
#define STATUS_BST            BIT ( 3)
#define STATUS_VMN            BIT ( 8)
#define STATUS_TMN            BIT ( 9)
#define STATUS_SMN            BIT (10)
#define STATUS_BI             BIT (11)
#define STATUS_VMX            BIT (12)
#define STATUS_TMX            BIT (13)
#define STATUS_SMX            BIT (14)
#define STATUS_BR             BIT (15)
#define VALRT_TH              0x01
#define TALRT_TH              0x02
#define SALRT_TH              0x03
#define ATRATE                0x04
#define REMCAPREP             0x05
#define SOCREP                0x06
#define AGE                   0x07
#define TEMP                  0x08
#define VCELL                 0x09
#define CURRENT               0x0A
#define AVGCURRENT            0x0B
#define SOCMIX                0x0D
#define SOCAV                 0x0E
#define REMCAPMIX             0x0F
#define FULLCAP               0x10
#define TTE                   0x11
#define QRESIDUAL00           0x12
#define FULLSOCTHR            0x13
#define AVGTA                 0x16
#define CYCLES                0x17
#define DESIGNCAP             0x18
#define AVGVCELL              0x19
#define MINMAXTEMP            0x1A
#define MINMAXVOLT            0x1B
#define MINMAXCURR            0x1C
#define CONFIG                0x1D
#define CONFIG_BER            BIT ( 0)
#define CONFIG_BEI            BIT ( 1)
#define CONFIG_AEN            BIT ( 2)
#define CONFIG_FTHRM          BIT ( 3)
#define CONFIG_ETHRM          BIT ( 4)
#define CONFIG_ALSH           BIT ( 5)
#define CONFIG_I2CSH          BIT ( 6)
#define CONFIG_SHDN           BIT ( 7)
#define CONFIG_TEX            BIT ( 8)
#define CONFIG_TEN            BIT ( 9)
#define CONFIG_AINSH          BIT (10)
#define CONFIG_ALRTP          BIT (11)
#define CONFIG_VS             BIT (12)
#define CONFIG_TS             BIT (13)
#define CONFIG_SS             BIT (14)
#define ICHGTERM              0x1E
#define REMCAPAV              0x1F
#define VERSION               0x21
#define QRESIDUAL10           0x22
#define FULLCAPNOM            0x23
#define TEMPNOM               0x24
#define TEMPLIM               0x25
#define AIN                   0x27
#define LEARNCFG              0x28
#define FILTERCFG             0x29
#define RELAXCFG              0x2A
#define MISCCFG               0x2B
#define TGAIN                 0x2C
#define TOFF                  0x2D
#define CGAIN                 0x2E
#define COFF                  0x2F
#define QRESIDUAL20           0x32
#define FULLCAP0              0x35
#define IAVG_EMPTY            0x36
#define FCTC                  0x37
#define RCOMP0                0x38
#define TEMPCO                0x39
#define V_EMPTY               0x3A
#define FSTAT                 0x3D
#define TIMER                 0x3E
#define SHDNTIMER             0x3F
#define QRESIDUAL30           0x42
#define DQACC                 0x45
#define DPACC                 0x46
#define VFSOC0                0x48
#define QH0                   0x4C
#define QH                    0x4D
#define FG_INT                0xA7
#define FG_INT_TMN            BIT (0)
#define FG_INT_TMX            BIT (1)
#define FG_INT_VMN            BIT (2)
#define FG_INT_VMX            BIT (3)
#define FG_INT_SMN            BIT (4)
#define FG_INT_SMX            BIT (5)
#define FG_INT_M              0xA8
#define VFOCV                 0xFB
#define VFSOC                 0xFF

#define DEGREE_SIGN_UTF8      "\xC2\xB0"
#define DEGREE_SIGN           DEGREE_SIGN_UTF8

#define OHM_SIGN_UTF8         "\xE2\x84\xA6"
#define OHM_SIGN              OHM_SIGN_UTF8

struct max77696_fg {
    struct mutex                           lock;
    struct max77696_battery_platform_data *pdata;
    struct max77696_core                  *core;
    struct max77696_io                    *io;
    struct device                         *dev;
    struct kobject                        *kobj;
    const struct attribute_group          *attr_grp;

    int                                    irq;
    u16                                    irq_unmask;

    struct delayed_work                    init_work;
    struct power_supply                   *psy;

    u16                                    capacity; /* mAh */
    u16                                    r_sns;    /* mOhm */

    u16                                    status;
    u16                                    vcell;
    u16                                    soc;
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

static __always_inline
void max77696_fg_enable_irq (struct max77696_fg *fg, u16 irq_bits)
{
    u16 fgintm;
    int rc;

    if (unlikely((fg->irq_unmask & irq_bits) == irq_bits)) {
        /* already unmasked or null bit */
        return;
    }

    if (unlikely(!fg->irq_unmask)) {
        enable_irq(fg->irq);
        enable_irq_wake(fg->irq);
    }

    /* set enabled flag */
    fg->irq_unmask |= irq_bits;

    /* unmask irq */
    fgintm = ~fg->irq_unmask;

    rc = max77696_write(fg->io, FG_INT_M, fgintm);
    dev_dbg(fg->dev, "written FG_INT_M 0x%04X [%d]\n", fgintm, rc);

    if (unlikely(rc)) {
        dev_err(fg->dev, "FG_INT_M write error [%d]\n", rc);
    }
}

static __always_inline
void max77696_fg_disable_irq (struct max77696_fg *fg, u16 irq_bits)
{
    u16 fgintm;
    int rc;

    if (unlikely((fg->irq_unmask & irq_bits) == 0)) {
        /* already masked or null bit */
        return;
    }

    /* clear enabled flag */
    fg->irq_unmask &= ~irq_bits;

    if (unlikely(!fg->irq_unmask)) {
        disable_irq_wake(fg->irq);
        disable_irq(fg->irq);
    }

    /* mask irq */
    fgintm = ~fg->irq_unmask;

    rc = max77696_write(fg->io, FG_INT_M, fgintm);
    dev_dbg(fg->dev, "written FG_INT_M 0x%04X [%d]\n", fgintm, rc);

    if (unlikely(rc)) {
        dev_err(fg->dev, "FG_INT_M write error [%d]\n", rc);
    }
}

static __always_inline u16 max77696_fg_read_irq (struct max77696_fg *fg)
{
    u16 fgint;
    int rc;

    rc = max77696_read(fg->io, FG_INT, &fgint);
    if (unlikely(rc)) {
        dev_err(fg->dev, "FG_INT read error [%d]\n", rc);
        return 0;
    }

    return fgint;
}

static __always_inline void max77696_fg_ack_irq (struct max77696_fg *fg)
{
    if (FG_RWC_INTERRUPT) {
        max77696_write(fg->io, FG_INT, ~0);
    }
}

static int max77696_fg_setup (struct max77696_fg *fg)
{
    return 0;
}

#define FG_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_fg_attr[] = {
    NULL
};

static const struct attribute_group max77696_fg_attr_group = {
    .attrs = max77696_fg_attr,
};

/* returns battery index. if battery not present, returns negative */
static __always_inline int max77696_fg_detect_battery (struct max77696_fg *fg)
{
    return 0; /* always present */
}

static __always_inline struct max77696_battery_ini *
max77696_fg_get_battery_ini (struct max77696_fg *fg)
{
    /* TODO: */
    int battery_index = max77696_fg_detect_battery(fg);
    return &fg->pdata->ini[battery_index];
}

static int max77696_fg_update_status (struct max77696_fg *fg)
{
    u16 status, vcell, soc;
    int rc;

    rc = max77696_read(fg->io, STATUS, &status);
    if (unlikely(rc)) {
        dev_err(fg->dev, "STATUS read error [%d]\n", rc);
        goto out;
    }

    rc = max77696_read(fg->io, VCELL, &vcell);
    if (unlikely(rc)) {
        dev_err(fg->dev, "VCELL read error [%d]\n", rc);
        goto out;
    }

#ifdef FG_LPM_WORKAROUNDS
    rc = max77696_read(fg->io, SOCAV, &soc);
#else /* FG_LPM_WORKAROUNDS */
    rc = max77696_read(fg->io, SOCREP, &soc);
#endif /* FG_LPM_WORKAROUNDS */
    if (unlikely(rc)) {
        dev_err(fg->dev, "SOC read error [%d]\n", rc);
        goto out;
    }

    dev_dbg(fg->dev, "STATUS 0x%04X VCELL 0x%04X SOC 0x%04X\n",
        status, vcell, soc);

    fg->status = status;
    fg->vcell  = vcell;
    fg->soc    = soc;

out:
    return rc;
}

static irqreturn_t max77696_fg_isr (int irq, void *data)
{
    struct max77696_fg *fg = data;
    u16 fgint;

    fgint = max77696_fg_read_irq(fg);
    max77696_fg_ack_irq(fg);
    dev_dbg(fg->dev, "FGINT 0x%04X EN 0x%04X\n", fgint, fg->irq_unmask);

//  fgint &= fg->irq_unmask;

    if (fgint & FG_INT_VMN) {
        dev_dbg(fg->dev, "VMN interrupt received\n");
    }

    if (fgint & FG_INT_VMX) {
        dev_dbg(fg->dev, "VMX interrupt received\n");
    }

    if (fgint & FG_INT_TMN) {
        dev_dbg(fg->dev, "TMN interrupt received\n");
    }

    if (fgint & FG_INT_TMX) {
        dev_dbg(fg->dev, "TMX interrupt received\n");
    }

    if (fgint & FG_INT_SMN) {
        dev_dbg(fg->dev, "SMN interrupt received\n");
    }

    if (fgint & FG_INT_SMX) {
        dev_dbg(fg->dev, "SMX interrupt received\n");
    }

/*out:*/
    if (likely(fg->psy)) {
        power_supply_changed(fg->psy);
    }
    return IRQ_HANDLED;
}

/* priority: FULL > CHARGING > NOT_CHARGING > DISCHARGING */
#define __psy_status_priority(_status) \
        (((_status) == POWER_SUPPLY_STATUS_FULL        ) ? 4 :\
         ((_status) == POWER_SUPPLY_STATUS_CHARGING    ) ? 3 :\
         ((_status) == POWER_SUPPLY_STATUS_NOT_CHARGING) ? 2 :\
         ((_status) == POWER_SUPPLY_STATUS_DISCHARGING ) ? 1 : 0)

static int max77696_fg_get_psy_status(struct max77696_fg *fg)
{
    int i, rc, supply_status;

    if (unlikely((fg->status & STATUS_BST) != 0)) {
        dev_dbg(fg->dev, "battery not present\n");
	    rc = POWER_SUPPLY_STATUS_UNKNOWN;
	    goto out;
    }

    supply_status = POWER_SUPPLY_STATUS_UNKNOWN;

    for (i = 0; i < fg->pdata->num_supplies; i++) {
        struct power_supply *psy =
            power_supply_get_by_name(fg->pdata->supplied_from[i]);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
        const struct power_supply *psy_desc = psy;
#else /* VERSION ... */
        const struct power_supply_desc *psy_desc = psy ? psy->desc : NULL;
#endif /* VERSION ... */
        union power_supply_propval psp_val;

        if (unlikely(!psy_desc)) {
            continue;
        }

        if (unlikely(!psy_desc->get_property)) {
            continue;
        }

        rc = psy_desc->get_property(psy, POWER_SUPPLY_PROP_STATUS, &psp_val);
        if (unlikely(rc)) {
            dev_vdbg(fg->dev, "%s status unknown [%d]\n", psy_desc->name, rc);
            continue;
        }

        dev_vdbg(fg->dev, "%-16s status %d\n", psy_desc->name, psp_val.intval);

        if (unlikely(__psy_status_priority(supply_status) <
                     __psy_status_priority(psp_val.intval))) {
            supply_status = psp_val.intval;
        }
    }

    rc = supply_status;
    dev_dbg(fg->dev, "all supplies status %s(%d)\n",
        rc == POWER_SUPPLY_STATUS_FULL         ? "FULL"         :
        rc == POWER_SUPPLY_STATUS_CHARGING     ? "CHARGING"     :
        rc == POWER_SUPPLY_STATUS_NOT_CHARGING ? "NOT_CHARGING" :
        rc == POWER_SUPPLY_STATUS_DISCHARGING  ? "DISCHARGING"  : "UNKNOWN",
        rc);

    /* notify STATUS_FULL if SOC >= FG_SOC_FULL threshold */
    if (unlikely(fg->soc >= FG_SOC_FULL)) {
#if 1
        /* notify STATUS_FULL only if charger is present */
        if (supply_status == POWER_SUPPLY_STATUS_CHARGING ||
            supply_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
#endif
        {
            rc = POWER_SUPPLY_STATUS_FULL;
        }
    }

out:
    return rc;
}

static int max77696_fg_psy_get_property (struct power_supply *psy,
        enum power_supply_property psp, union power_supply_propval *val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    struct max77696_fg *fg = dev_get_drvdata(psy->dev->parent);
#else /* VERSION ... */
    struct max77696_fg *fg = power_supply_get_drvdata(psy);
#endif /* VERSION ... */
    u16 data;
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

    __lock(fg);

    max77696_fg_update_status(fg);

    switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	    val->intval = max77696_fg_get_psy_status(fg);
	    break;

	case POWER_SUPPLY_PROP_HEALTH:
	    if (unlikely((fg->status & STATUS_BST) != 0)) {
	        dev_dbg(fg->dev, "battery not present\n");
    	    val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
    	    rc = 0;
    	    goto out;
        }
	    val->intval = POWER_SUPPLY_HEALTH_GOOD;
        /* check temperature */
        if (fg->status & STATUS_TMX) {
            dev_dbg(fg->dev, "battery overheat\n");
            val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
        }
        if (fg->status & STATUS_TMN) {
            dev_dbg(fg->dev, "battery cold\n");
            val->intval = POWER_SUPPLY_HEALTH_COLD;
        }
        /* check voltage */
        if (fg->vcell >= FG_VCELL_OVER) {
            dev_dbg(fg->dev, "battery overvoltage\n");
            val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        }
        if (fg->vcell <= FG_VCELL_DEAD) {
            dev_dbg(fg->dev, "battery dead\n");
            val->intval = POWER_SUPPLY_HEALTH_DEAD;
        }
	    break;

	case POWER_SUPPLY_PROP_PRESENT:
	    val->intval = 1; /* always present */
	    break;

    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;

    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        rc = max77696_read(fg->io, CYCLES, &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "CYCLES read error [%d]\n", rc);
            goto out;
        }
        val->intval = (int)data / 100;
        break;

    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        /* LSB = 625 [uV] ; Don't care lower 3 bits */
        val->intval = (int)(fg->vcell >> 3) * 625;
        break;

    case POWER_SUPPLY_PROP_VOLTAGE_AVG:
        rc = max77696_read(fg->io, AVGVCELL, &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "AVGVCELL read error [%d]\n", rc);
            goto out;
        }
        /* LSB = 625 [uV] ; Don't care lower 3 bits */
        val->intval = (int)(data >> 3) * 625;
        break;

    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        val->intval = (int)fg->capacity * 1000; /* mAh -> uAh */
        break;

    case POWER_SUPPLY_PROP_CHARGE_FULL: /* Last Measured Discharge */
        rc = max77696_read(fg->io, FULLCAPNOM, &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "FULLCAPNOM read error [%d]\n", rc);
            goto out;
        }
        /* LSB = 5uV / Rsense (0.5mAh when Rsense = 10mOhm)
         */
        val->intval  = DIV_ROUND_UP((int)data * 5, fg->r_sns);
        val->intval *= 1000; /* mAh -> uAh */
        break;

    case POWER_SUPPLY_PROP_CHARGE_NOW: /* Nominal Available Charge */
        rc = max77696_read(fg->io, REMCAPREP, &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "REMCAPREP read error [%d]\n", rc);
            goto out;
        }
        /* LSB = 5uV / Rsense (0.5mAh when Rsense = 10mOhm)
         */
        val->intval  = DIV_ROUND_UP((int)data * 5, fg->r_sns);
        val->intval *= 1000; /* mAh -> uAh */
        break;

    case POWER_SUPPLY_PROP_CAPACITY:
        /* LSB = 1 / 256 [%] */
        val->intval = DIV_ROUND_UP((int)fg->soc, 256);
        break;

    case POWER_SUPPLY_PROP_TEMP:
        rc = max77696_read(fg->io, TEMP, &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "TEMP read error [%d]\n", rc);
            goto out;
        }
        /* LSB = 10 / 256 [tenths of degree Celsius] */
        val->intval = DIV_ROUND_UP((int)(s16)data * 10, 256);
        break;

    case POWER_SUPPLY_PROP_CURRENT_NOW:
        rc = max77696_read(fg->io, CURRENT, &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "CURRENT read error [%d]\n", rc);
            goto out;
        }
        /* LSB = 1.5625uV / Rsense (0.15625mA where Rsense = 10mOhm)
         */
        val->intval  = DIV_ROUND_UP((int)(s16)data * 100, 64 * fg->r_sns);
        val->intval *= 1000; /* mA -> uA */
        break;

    case POWER_SUPPLY_PROP_CURRENT_AVG:
        rc = max77696_read(fg->io, AVGCURRENT, &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "AVGCURRENT read error [%d]\n", rc);
            goto out;
        }
        /* LSB = 1.5625uV / Rsense (0.15625mA where Rsense = 10mOhm)
         */
        val->intval  = DIV_ROUND_UP((int)(s16)data * 100, 64 * fg->r_sns);
        val->intval *= 1000; /* mA -> uA */
        break;

    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
        rc = max77696_read(fg->io, TTE, &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "TTE read error [%d]\n", rc);
            goto out;
        }
        /* LSB = 5.625 [sec] */
        val->intval = DIV_ROUND_UP((int)data * 5625, 1000);
        break;

    default:
        rc = -EINVAL;
        goto out;
    }

out:
    dev_vdbg(fg->dev,
        "<get_property> psp %d val %d [%d]\n", psp, val->intval, rc);
    __unlock(fg);
    return rc;
}

#define max77696_fg_psy_set_property           NULL
#define max77696_fg_psy_property_is_writeable  NULL
#define max77696_fg_psy_external_power_changed NULL

static enum power_supply_property max77696_fg_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CYCLE_COUNT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_VOLTAGE_AVG,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_CHARGE_FULL, /* Last Measured Discharge */
    POWER_SUPPLY_PROP_CHARGE_NOW,  /* Nominal Available Charge */
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CURRENT_AVG,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
static struct power_supply_desc max77696_fg_psy_desc = {
	.name		            = DRIVER_NAME,
	.type		            = POWER_SUPPLY_TYPE_BATTERY,
    .properties             = max77696_fg_psy_props,
    .num_properties         = ARRAY_SIZE(max77696_fg_psy_props),
    .get_property           = max77696_fg_psy_get_property,
    .set_property           = max77696_fg_psy_set_property,
    .property_is_writeable  = max77696_fg_psy_property_is_writeable,
    .external_power_changed = max77696_fg_psy_external_power_changed,
};

static struct power_supply_config max77696_fg_psy_config;
#endif /* VERSION >= 4.1.0 */

static void *max77696_fg_get_platdata (struct max77696_fg *fg)
{
    struct max77696_battery_platform_data *pdata;
    struct device *dev = fg->dev;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;
    struct device_node *ini_np, *ini_child_np;
    size_t sz;
    int rc, i, num_supplicants, num_supplies, num_ini;

    num_supplicants = of_property_count_strings(np, "supplied_to");
    num_supplicants = max(0, num_supplicants);

    num_supplies = of_property_count_strings(np, "supplied_from");
    num_supplies = max(0, num_supplies);

    ini_np = of_get_child_by_name(np, "ini");
    if (unlikely(!ini_np)) {
        dev_err(dev, "ini data not specified\n");
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }

    num_ini = of_get_child_count(ini_np);
    if (unlikely(num_ini <= 0)) {
        dev_err(dev, "ini data is empty\n");
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }

    sz = sizeof(*pdata) +
        (num_supplicants + num_supplies) * sizeof(char*) +
        num_ini * sizeof(*pdata->ini);
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

    pdata->r_sns = FG_DEFAULT_RSENSE;
    of_property_u16(np, "r_sns", &pdata->r_sns);

    pdata->valrt_max = 5100;
    of_property_u32(np, "valrt_max", (u32*)&pdata->valrt_max);

    pdata->valrt_min = 3600;
    of_property_u32(np, "valrt_min", (u32*)&pdata->valrt_min);

    pdata->talrt_max = 0;
    of_property_u32(np, "talrt_max", (u32*)&pdata->talrt_max);

    pdata->talrt_min = 0;
    of_property_u32(np, "talrt_min", (u32*)&pdata->talrt_min);

    pdata->salrt_max = 0;
    of_property_u32(np, "salrt_max", (u32*)&pdata->salrt_max);

    pdata->salrt_min = 0;
    of_property_u32(np, "salrt_min", (u32*)&pdata->salrt_min);

    pdata->ini = (void*)((char**)(pdata + 1) + num_supplicants + num_supplies);

    i = 0;
    for_each_child_of_node(ini_np, ini_child_np) {
        struct max77696_battery_ini *ini = &pdata->ini[i++];

        #define of_property_read_battery_ini(_prop) \
                do {\
                    rc = of_property_u16(ini_child_np, #_prop, &ini->_prop);\
                    if (unlikely(rc)) {\
                        dev_err(dev, "ini[%d]["#_prop"] not specified [%d]\n",\
                            i, rc);\
                        goto abort;\
                    }\
                } while (0)

        of_property_read_battery_ini(learncfg  );
        of_property_read_battery_ini(capacity  );
        of_property_read_battery_ini(misccfg   );
        of_property_read_battery_ini(ichgterm  );
        of_property_read_battery_ini(filtercfg );
        of_property_read_battery_ini(fullsocthr);
        of_property_read_battery_ini(relaxcfg  );
        of_property_read_battery_ini(dq_acc    );
        of_property_read_battery_ini(dp_acc    );
        of_property_read_battery_ini(qrtable00 );
        of_property_read_battery_ini(qrtable10 );
        of_property_read_battery_ini(qrtable20 );
        of_property_read_battery_ini(qrtable30 );
        of_property_read_battery_ini(rcomp0    );
        of_property_read_battery_ini(tempco    );
        of_property_read_battery_ini(v_empty   );
        of_property_read_battery_ini(iavg_empty);

        rc = of_property_read_u16_array(ini_child_np, "modeldata",
            ini->modeldata, ARRAY_SIZE(ini->modeldata));
        if (unlikely(rc)) {
            dev_err(dev, "ini[%d][modeldata] not specified [%d]\n", i, rc);
            goto abort;
        }

        ini->title = "untitled";
        of_property_read_string(ini_child_np, "title",
            (char const **)&ini->title);
        dev_dbg(dev, "ini[%d] %s data parsed\n", i, ini->title);
    }

    /* all done successfully */
    goto out;

abort:
    devm_kfree(dev, pdata);
    return ERR_PTR(rc);
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

out:
    return pdata;
}

static __always_inline void max77696_fg_destroy (struct max77696_fg *fg)
{
    if (likely(fg->attr_grp)) {
        sysfs_remove_group(fg->kobj, fg->attr_grp);
    }

    if (likely(fg->irq > 0)) {
        free_irq(fg->irq, fg);
    }

    if (likely(fg->psy)) {
        power_supply_unregister(fg->psy);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(fg->pdata)) {
        devm_kfree(fg->dev, fg->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&fg->lock);
    dev_set_drvdata(fg->dev, NULL);
    devm_kfree(fg->dev, fg);
}

#define MODEL_LOCK_0          0x62
#define MODEL_LOCK_1          0x63
#define MODEL_MEM_BASE        0x80
#define MODEL_MEM_ADDR(_ofst) (0x80+(_ofst))

#define MODEL_UNLOCK_CODE_0   0x0059
#define MODEL_UNLOCK_CODE_1   0x00C4
#define MODEL_LOCK_CODE_0     0x0000
#define MODEL_LOCK_CODE_1     0x0000

#define MODEL_ROWS            3
#define MODEL_COLS            16
#define MODEL_WORDSZ          2

#define MODEL_WORDS           (MODEL_ROWS * MODEL_COLS)
#define MODEL_ROW_BYTES       (MODEL_COLS * MODEL_WORDSZ)
#define MODEL_BYTES           (MODEL_ROWS * MODEL_ROW_BYTES)

static __always_inline int max77696_fg_unlock_model (struct max77696_fg *fg)
{
    u16 data;
    int i, rc;

    max77696_write(fg->io, MODEL_LOCK_0, MODEL_UNLOCK_CODE_0);
    max77696_write(fg->io, MODEL_LOCK_1, MODEL_UNLOCK_CODE_1);

    /* Any non-zero will be read if model is unlocked */

    for (i = 0; i < MODEL_WORDS; i++) {
        rc = max77696_read(fg->io, MODEL_MEM_ADDR(i), &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "model(%d) read error [%d]\n", i, rc);
            goto out;
        }

        if (data) {
            rc = 0;
            goto out;
        }
    }

    dev_err(fg->dev, "failed to unlock model\n");
    rc = -EIO;

out:
    return rc;
}

static __always_inline int max77696_fg_lock_model (struct max77696_fg *fg)
{
    u16 data;
    int i, rc;

    max77696_write(fg->io, MODEL_LOCK_0, MODEL_LOCK_CODE_0);
    max77696_write(fg->io, MODEL_LOCK_1, MODEL_LOCK_CODE_1);

    /* All zero will be read if model is locked */

    for (i = 0; i < MODEL_WORDS; i++) {
        rc = max77696_read(fg->io, MODEL_MEM_ADDR(i), &data);
        if (unlikely(rc)) {
            dev_err(fg->dev, "model(%d) read error [%d]\n", i, rc);
            goto out;
        }

        if (data) {
            dev_err(fg->dev, "failed to lock model\n");
            rc = -EIO;
            goto out;
        }
    }

    rc = 0;

out:
    return rc;
}

static __always_inline
int max77696_fg_read_model (struct max77696_fg *fg, u16* model)
{
    int row, col, rc;

    for (row = 0; row < MODEL_ROWS; row++) {
        for (col = 0; col < MODEL_COLS; col++) {
            rc = max77696_read(fg->io, MODEL_MEM_ADDR(row * MODEL_COLS + col),
                &model[row * MODEL_COLS + col]);
            if (unlikely(rc)) {
                dev_err(fg->dev, "model row (%d/%d) read error [%d]\n",
                    row, MODEL_ROWS-1, rc);
                goto out;
            }
        }
    }

#ifdef VERBOSE_DEBUG
    do {
        char *str = devm_kzalloc(fg->dev,
            sizeof("0000 ") * MODEL_COLS + 1, GFP_KERNEL);

        for (row = 0; row < MODEL_ROWS; row++) {
            int ofst = 0;

            for (col = 0; col < MODEL_COLS; col++) {
                ofst += (int)sprintf(str + ofst, "%04X ",
                    model[row * MODEL_COLS + col]);
            }

            printk(DRIVER_NAME": write_model[%d] %s\n", row, str);
        }

        devm_kfree(fg->dev, str);
    } while (0);
#endif /* VERBOSE_DEBUG */

out:
    return rc;
}

static __always_inline
int max77696_fg_write_model (struct max77696_fg *fg, const u16* model)
{
    int row, col, rc;

#ifdef VERBOSE_DEBUG
    do {
        char *str = devm_kzalloc(fg->dev,
            sizeof("0000 ") * MODEL_COLS + 1, GFP_KERNEL);

        for (row = 0; row < MODEL_ROWS; row++) {
            int ofst = 0;

            for (col = 0; col < MODEL_COLS; col++) {
                ofst += (int)sprintf(str + ofst, "%04X ",
                    model[row * MODEL_COLS + col]);
            }

            printk(DRIVER_NAME": write_model[%d] %s\n", row, str);
        }

        devm_kfree(fg->dev, str);
    } while (0);
#endif /* VERBOSE_DEBUG */

    for (row = 0; row < MODEL_ROWS; row++) {
        for (col = 0; col < MODEL_COLS; col++) {
            rc = max77696_write(fg->io, MODEL_MEM_ADDR(row * MODEL_COLS + col),
                model[row * MODEL_COLS + col]);
            if (unlikely(rc)) {
                dev_err(fg->dev, "model row (%d/%d) write error [%d]\n",
                    row, MODEL_ROWS-1, rc);
                goto out;
            }
        }
    }

out:
    return rc;
}

static int max77696_fg_load_model (struct max77696_fg *fg, const u16* model)
{
    struct device *dev = fg->dev;
    u16 *wbuf, *rbuf;
    int trial, rc;

    wbuf = devm_kzalloc(dev, MODEL_BYTES, GFP_KERNEL);
    rbuf = devm_kzalloc(dev, MODEL_BYTES, GFP_KERNEL);

    if (unlikely(!wbuf || !rbuf)) {
        dev_err(dev, "out of memory (%uB requested)\n", MODEL_BYTES);
        rc = -ENOMEM;
        goto out;
    }

    memcpy(wbuf, model, MODEL_BYTES);

    /* Unlock Model Access
     */

    rc = max77696_fg_unlock_model(fg);
    if (unlikely(rc)) {
        goto abort;
    }

    dev_dbg(dev, "model is unlocked\n");

    /* Write the Custom Model
     */

    rc = max77696_fg_write_model(fg, wbuf);
    if (unlikely(rc)) {
        goto abort;
    }

    dev_dbg(dev, "model is written\n");

    /* Verify the Custom Model
     */

    rc = max77696_fg_read_model(fg, rbuf);
    if (unlikely(rc)) {
        goto abort;
    }

    rc = memcmp(wbuf, rbuf, MODEL_BYTES);
    if (unlikely(rc)) {
        dev_err(dev, "failed to verify model written\n");
        rc = -EIO;
        goto abort;
    }

    dev_dbg(dev, "model is verified\n");

    /* Lock Model Access
     */

    for (trial = 0; true; trial++) {
        if (trial >= 3) {
            rc = -EIO;
            goto out;
        }

        rc = max77696_fg_lock_model(fg);
        if (likely(!rc)) {
            dev_dbg(dev, "model is locked\n");
            goto out;
        }

        dev_dbg(dev, "try again to lock model access %d\n", trial + 1);
    }

abort:
    max77696_fg_lock_model(fg);
out:
    if (likely(rbuf)) {
        devm_kfree(dev, rbuf);
    }
    if (likely(wbuf)) {
        devm_kfree(dev, wbuf);
    }
    return rc;
}

#define VFSOC0_LOCK           0x60
#define VFSOC0_LOCK_CODE      0x0000
#define VFSOC0_UNLOCK_CODE    0x0080

static __always_inline int max77696_fg_por_init (struct max77696_fg *fg)
{
    #define __LogParam(_Name, _Value) \
            dev_dbg(fg->dev, "%-15s 0x%04X\n", _Name, _Value)
    #define __WriteRegister(_RegisterAddress, _RegisterValueToWrite) \
            max77696_write(fg->io, _RegisterAddress, _RegisterValueToWrite)
    #define __ReadRegister(_RegisterAddress) \
            ({\
                u16 __RegisterValueRead = 0;\
                int __rc = max77696_read(fg->io, _RegisterAddress,\
                    &__RegisterValueRead);\
                if (unlikely(__rc)) {\
                    dev_err(fg->dev, "failed to read register 0x%04X [%d]\n",\
                        _RegisterAddress, __rc);\
                    BUG();\
                }\
                __RegisterValueRead;\
            })
    #define __WriteAndVerifyRegister(_RegisterAddress, _RegisterValueToWrite) \
            ({\
                int __try = 0, __max_try = 5;\
                u16 __RegisterValueRead;\
                do {\
                    __WriteRegister(_RegisterAddress, _RegisterValueToWrite);\
                    __msleep(3);\
                    __RegisterValueRead = __ReadRegister(_RegisterAddress);\
                    if (likely(_RegisterValueToWrite == __RegisterValueRead)) {\
                        break;\
                    }\
                } while (++__try < __max_try);\
                (__try < __max_try ? 0 : -EIO);\
            })

    struct device *dev = fg->dev;
	struct max77696_battery_ini *ini;
	u16 status, r_sns, model_scaling;
	u16 capacity, vf_fullcap, vfsoc, qh, remcap, repcap;
    int rc = 0;

    /*
     * INITIALIZE REGISTERS TO RECOMMENDED CONFIGURATION
     * -------------------------------------------------
     * The MAX77696 Fuel Gauge should be initialized prior to being used.
     * The following three registers should be written to these values in order
     * for the MAX77696 Fuel Gauge to perform at its best. These values are
     * written to RAM, so they must be written to the device any time that power
     * is applied or restored to the device. Some registers are updated
     * internally, so it is necessary to verify that the register was written
     * correctly to prevent data collisions.
     */

    /* STEP 0. Check for POR
     * ---------------------------------------------------------------------- */

    status = __ReadRegister(STATUS);
    __LogParam("STATUS", status);

    if (unlikely((status & STATUS_POR) == 0)) {
        goto out;
    }

    dev_info(dev, "POR detected\n");

    ini           = max77696_fg_get_battery_ini(fg);
    r_sns         = fg->r_sns;
    model_scaling = FG_MODEL_SCALING;

    dev_dbg(dev, "INI     %s\n"            , ini->title   );
    dev_dbg(dev, "SENSE R %u m"OHM_SIGN"\n", r_sns        );
    dev_dbg(dev, "SCALING %u\n"            , model_scaling);

    /* STEP 1. Delay at least 500ms
     * ---------------------------------------------------------------------- */

    /* After Power up, the MAX77696 Fuel Gauge requires 500ms in order to
     * perform signal debouncing and initial SOC reporting.
     */
    __msleep(500);

    /* STEP 2. Initialize configuration
     * ---------------------------------------------------------------------- */

    __WriteRegister(CONFIG    , 0x2210         );
    __WriteRegister(FILTERCFG , ini->filtercfg );
    __WriteRegister(RELAXCFG  , ini->relaxcfg  );
    __WriteRegister(LEARNCFG  , ini->learncfg  );
    __WriteRegister(FULLSOCTHR, ini->fullsocthr);
    __WriteRegister(IAVG_EMPTY, ini->iavg_empty);

    /*
     * LOAD CUSTOM MODEL AND PARAMETERS
     * --------------------------------
     * The custom model that is stored in the MAX77696 Fuel Gauge is also
     * written to RAM and so it must be written to the device any time that
     * power is applied or restored to the device. When the device is powered
     * on, the host software must first unlock write access to the model, write
     * the model, verify the model was written properly, and then lock access to
     * the model. After the model is loaded correctly, simply write a few
     * registers with customized parameters that will be provided by Maxim.
     */

    /* STEP 4. ~ STEP 9.
     * ---------------------------------------------------------------------- */

    rc = max77696_fg_load_model(fg, ini->modeldata);
    if (unlikely(rc)) {
        dev_err(dev, "failed to load custom model [%d]\n", rc);
//      goto out;
    }

    /* STEP 10. Write custom parameters
     * ---------------------------------------------------------------------- */

    __WriteAndVerifyRegister(RCOMP0     , ini->rcomp0   );
    __WriteAndVerifyRegister(TEMPCO     , ini->tempco   );
    __WriteRegister         (ICHGTERM   , ini->ichgterm );
    /* TODO: set the TGAIN and TOFF when using thermistor */
  //__WriteRegister         (TGAIN      , ini->tgain    );
  //__WriteRegister         (TOFF       , ini->toff     );
    __WriteAndVerifyRegister(V_EMPTY    , ini->v_empty  );
    __WriteAndVerifyRegister(QRESIDUAL00, ini->qrtable00);
    __WriteAndVerifyRegister(QRESIDUAL10, ini->qrtable10);
    __WriteAndVerifyRegister(QRESIDUAL20, ini->qrtable20);
    __WriteAndVerifyRegister(QRESIDUAL30, ini->qrtable30);

    /* STEP 11. Update full capacity parameters
     * ---------------------------------------------------------------------- */

    /* Capacity LSB = 5uV / Rsense
     * (0.5mAh where Rsense = 10mOhm)
     */
    capacity = (u16)DIV_ROUND_UP((u32)fg->capacity * r_sns, 5);
    __LogParam("CAPACITY", capacity);
    __WriteAndVerifyRegister(FULLCAP, capacity);

    /* If the INI does not have a VF_FullCap value, the Capacity should be used
     * for both VF_FullCap and Capacity.
     */
    vf_fullcap = capacity;
    __LogParam("VF_FULLCAP", vf_fullcap);
    __WriteRegister         (DESIGNCAP , vf_fullcap);
    __WriteAndVerifyRegister(FULLCAPNOM, vf_fullcap);

    /* STEP 13. Delay at least 350ms
     * ---------------------------------------------------------------------- */

    /* Delay to allow VFSOC to be calculated from the new configuration */
    __msleep(350);

    /* STEP 14. Write VFSOC and QH values
     * ---------------------------------------------------------------------- */

    vfsoc = __ReadRegister(VFSOC);
    __LogParam("VFSOC", vfsoc);
    __WriteRegister(VFSOC0_LOCK, VFSOC0_UNLOCK_CODE);
    __WriteAndVerifyRegister(VFSOC0, vfsoc);
    qh = __ReadRegister(QH);
    __LogParam("QH", qh);
    __WriteRegister(QH0, qh);
    __WriteRegister(VFSOC0_LOCK, VFSOC0_LOCK_CODE);

    /* STEP 15.5. Advance to Coulomb-Counter Mode
     * ---------------------------------------------------------------------- */

    /* Advancing the cycles register to a higher values makes the fuelgauge
     * behave more like a coulomb counter. MAX77696 Fuel Gauge supports quicker
     * insertion error healing by supporting starting from a lower learn stage.
     * To Advance to Coulomb-Counter Mode, simply write the Cycles register to
     * a value of 96%.
     */
    __WriteAndVerifyRegister(CYCLES, 0x0060);

    /* STEP 16. Load new capacity parameters
     * ---------------------------------------------------------------------- */

    remcap = (u16)DIV_ROUND_UP((u32)vfsoc * vf_fullcap, 25600);
    __LogParam("REMCAP", remcap);
    __WriteAndVerifyRegister(REMCAPMIX, remcap);

    repcap = (u16)((u32)remcap *
        DIV_ROUND_UP(capacity, vf_fullcap * model_scaling));
    __LogParam("REPCAP", repcap);
    __WriteAndVerifyRegister(REMCAPREP, repcap);

    __WriteAndVerifyRegister(DPACC, ini->dp_acc);
    __WriteAndVerifyRegister(DQACC, ini->dq_acc);

    __WriteAndVerifyRegister(FULLCAP   , capacity  );
    __WriteRegister         (DESIGNCAP , vf_fullcap);
    __WriteAndVerifyRegister(FULLCAPNOM, vf_fullcap);

    /* Update SOC register with new SOC */
    __WriteRegister(SOCREP, vfsoc);

    /* STEP 17. Initialization complete
     * ---------------------------------------------------------------------- */

#ifdef FG_LPM_WORKAROUNDS
    /* WORKAROUND for LPM:
     * Adjust MixRate to maximum by turning bits 0x03E0 on (along with existing
     * values) in the MiscCfg register.
     */
    do {
        u16 misccfg = __ReadRegister(MISCCFG);
        __WriteRegister(MISCCFG, misccfg | 0x03E0);
    } while (0);
#endif /* FG_LPM_WORKAROUNDS */

    /* Clear the POR bit to indicate that the custom model and parameters were
     * successfully loaded.
     */
    max77696_write(fg->io, STATUS, status & ~STATUS_POR);
    dev_info(dev, "POR cleared\n");

out:
    return rc;
}

static void max77696_fg_init_work (struct work_struct *work)
{
    struct max77696_fg *fg =
        container_of(work, struct max77696_fg, init_work.work);
    struct device *dev = fg->dev;
    int rc;

    /* Device wakeup initialization */
    device_init_wakeup(dev, true);

    rc = max77696_fg_por_init(fg);
	if (unlikely(rc)) {
	    dev_err(dev, "failed to POR init [%d]\n", rc);
	    goto abort;
	}

    rc = max77696_fg_setup(fg);
	if (unlikely(rc)) {
	    dev_err(dev, "failed to setup [%d]\n", rc);
	    goto abort;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    fg->psy                         = (struct power_supply*)(fg + 1);
    fg->psy->name                   =
        fg->pdata->psy_name ? fg->pdata->psy_name : DRIVER_NAME;
    fg->psy->type                   = POWER_SUPPLY_TYPE_BATTERY;
    fg->psy->supplied_to            = fg->pdata->supplied_to;
    fg->psy->num_supplicants        = fg->pdata->num_supplicants;
    fg->psy->properties             = max77696_fg_psy_props;
    fg->psy->num_properties         = ARRAY_SIZE(max77696_fg_psy_props);
    fg->psy->get_property           = max77696_fg_psy_get_property;
    fg->psy->set_property           = max77696_fg_psy_set_property;
    fg->psy->property_is_writeable  = max77696_fg_psy_property_is_writeable;
    fg->psy->external_power_changed = max77696_fg_psy_external_power_changed;

    rc = power_supply_register(dev, fg->psy);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register power_supply class [%d]\n", rc);
        fg->psy = NULL;
        goto abort;
    }
#else /* VERSION ... */
    if (fg->pdata->psy_name) {
        max77696_fg_psy_desc.name = fg->pdata->psy_name;
    }

    max77696_fg_psy_config.drv_data        = fg;
    max77696_fg_psy_config.supplied_to     = fg->pdata->supplied_to;
    max77696_fg_psy_config.num_supplicants = fg->pdata->num_supplicants;

	fg->psy = power_supply_register(dev, &max77696_fg_psy_desc,
						&max77696_fg_psy_config);
	if (unlikely(IS_ERR(fg->psy))) {
        rc = PTR_ERR(fg->psy);
        fg->psy = NULL;
        dev_err(dev, "failed to register power_supply class [%d]\n", rc);
        goto abort;
	}
#endif /* VERSION ... */

    /* Get FG block IRQ number */
    fg->irq = max77696_get_block_irq(dev->parent, FG);
    BUG_ON(fg->irq <= 0);

    /* Request system IRQ for FG */
    rc = devm_request_threaded_irq(dev, (unsigned int)fg->irq, NULL,
        max77696_fg_isr, IRQF_ONESHOT, DRIVER_NAME, fg);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to request IRQ(%d) [%d]\n", fg->irq, rc);
        fg->irq = 0;
        goto abort;
    }

    dev_dbg(dev, "IRQ(%d) requested\n", fg->irq);

    disable_irq(fg->irq);

    /* Create max77696-fg sysfs attributes */
    fg->attr_grp = &max77696_fg_attr_group;
    rc = sysfs_create_group(fg->kobj, fg->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        fg->attr_grp = NULL;
        goto abort;
    }

    return;

abort:
    if (likely(fg->attr_grp)) {
        sysfs_remove_group(fg->kobj, fg->attr_grp);
        fg->attr_grp = NULL;
    }
    if (likely(fg->irq > 0)) {
        devm_free_irq(dev, fg->irq, fg);
        fg->irq = 0;
    }
    if (likely(fg->psy)) {
        power_supply_unregister(fg->psy);
        fg->psy = NULL;
    }
    return;
}

static __devinit int max77696_fg_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_fg *fg;
    size_t sz;
    u16 fgint;
    int i, rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    sz  = sizeof(*fg);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    sz += sizeof(struct power_supply);
#endif /* VERSION < 4.1.0 */

    fg = devm_kzalloc(dev, sz, GFP_KERNEL);
    if (unlikely(!fg)) {
        dev_err(dev, "out of memory (%uB requested)\n", sz);
        return -ENOMEM;
    }

    dev_set_drvdata(dev, fg);

    mutex_init(&fg->lock);
    fg->core  = core;
    fg->io    = max77696_get_block_io(dev->parent, FG);
    fg->dev   = dev;
    fg->kobj  = &dev->kobj;

    /* Disable all FG interrupts */
    fg->irq_unmask = 0;
    max77696_write(fg->io, FG_INT_M, ~0);

    /* Get FG interrupt status port address & Clear status */
    fgint = max77696_fg_read_irq(fg);
    max77696_fg_ack_irq(fg);
    dev_dbg(dev, "initial FG interrupt status: 0x%04X\n", fgint);

    fg->pdata = max77696_fg_get_platdata(fg);
    if (unlikely(IS_ERR(fg->pdata))) {
        rc = PTR_ERR(fg->pdata);
        fg->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    __prop_printk(dev, "PSY NAME", "%s", fg->pdata->psy_name);

    if (unlikely(fg->pdata->num_supplicants <= 0)) {
        __prop_printk(dev, "SUPPLICANTS", "%s", "null");
    } else {
        for (i = 0; i < fg->pdata->num_supplicants; i++) {
            __prop_printk(dev, "SUPPLICANTS", "%s", fg->pdata->supplied_to[i]);
        }
    }

    if (unlikely(fg->pdata->num_supplies <= 0)) {
        __prop_printk(dev, "SUPPLIES", "%s", "null");
    } else {
        for (i = 0; i < fg->pdata->num_supplies; i++) {
            __prop_printk(dev, "SUPPLIES", "%s", fg->pdata->supplied_from[i]);
        }
    }

    __prop_printk(dev, "R SENSE", "%u m"OHM_SIGN, fg->pdata->r_sns);
    __prop_printk(dev, "V ALRT", "%5d %5d mV",
        fg->pdata->valrt_min, fg->pdata->valrt_max);
    __prop_printk(dev, "T ALRT", "%5d %5d "DEGREE_SIGN"C",
        fg->pdata->talrt_min, fg->pdata->talrt_max);
    __prop_printk(dev, "S ALRT", "%5d %5d %%",
        fg->pdata->salrt_min, fg->pdata->salrt_max);

    if (unlikely(!fg->pdata->ini)) {
        dev_err(dev, "ModelGauge could not be configured\n");
        rc = -EINVAL;
        goto abort;
    }

    fg->capacity = fg->pdata->ini->capacity;
    fg->r_sns    = fg->pdata->r_sns;

    pr_debug("%s() successfully done\n", __func__);

    INIT_DELAYED_WORK(&fg->init_work, max77696_fg_init_work);
    schedule_delayed_work(&fg->init_work, FG_INIT_DELAY);

    return 0;

abort:
    max77696_fg_destroy(fg);
    return rc;
}

static __devexit int max77696_fg_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_fg *fg = dev_get_drvdata(dev);

    max77696_fg_destroy(fg);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_fg_suspend (struct device *dev)
{
    return 0;
}

static int max77696_fg_resume (struct device *dev)
{
#ifdef FG_LPM_WORKAROUNDS
    /* WORKAROUND for LPM:
     * Writing dPAcc and dQAcc to the initialization values after switching
     * power states. It prevents changing FullCapNom, keeping AvCap and AvSOC
     * accurate.
     */
    do {
        struct max77696_fg *fg = dev_get_drvdata(dev);
        struct max77696_battery_ini *ini = max77696_fg_get_battery_ini(fg);

        dev_dbg(dev, "restoring DPACC 0x%04X\n", ini->dp_acc);
        max77696_write(fg->io, DPACC, ini->dp_acc);
        dev_dbg(dev, "restoring DQACC 0x%04X\n", ini->dq_acc);
        max77696_write(fg->io, DQACC, ini->dq_acc);
    } while (0);
#endif /* FG_LPM_WORKAROUNDS */

    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_fg_pm,
    max77696_fg_suspend, max77696_fg_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_fg_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_fg_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_fg_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_fg_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_fg_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_fg_probe,
    .remove                = __devexit_p(max77696_fg_remove),
};

static __init int max77696_fg_driver_init (void)
{
    return platform_driver_register(&max77696_fg_driver);
}
module_init(max77696_fg_driver_init);

static __exit void max77696_fg_driver_exit (void)
{
    platform_driver_unregister(&max77696_fg_driver);
}
module_exit(max77696_fg_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
