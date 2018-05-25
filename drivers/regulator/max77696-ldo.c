/*
 * MAX77696 LDO Regulators Driver
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

#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    MAX77696_DESC" LDO Regulators Driver"
#define DRIVER_NAME    MAX77696_LDO_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define LDO_VREG_NAME               MAX77696_NAME"-ldo"
#define LDO_NREG                    MAX77696_VREG_NUM_OF_LDOS

#define LDO_DEFAULT_MODE            REGULATOR_MODE_NORMAL

#define LDO1_CNFG1                  0x43
#define LDO1_CNFG2                  0x44
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
#define LDO2_CNFG1                  0x45
#define LDO2_CNFG2                  0x46
#define LDO3_CNFG1                  0x47
#define LDO3_CNFG2                  0x48
#define LDO4_CNFG1                  0x49
#define LDO4_CNFG2                  0x4A
#define LDO5_CNFG1                  0x4B
#define LDO5_CNFG2                  0x4C
#define LDO6_CNFG1                  0x4D
#define LDO6_CNFG2                  0x4E
#define LDO7_CNFG1                  0x4F
#define LDO7_CNFG2                  0x50
#define LDO8_CNFG1                  0x51
#define LDO8_CNFG2                  0x52
#define LDO9_CNFG1                  0x53
#define LDO9_CNFG2                  0x54
#define LDO10_CNFG1                 0x55
#define LDO10_CNFG2                 0x56
#elif defined(CONFIG_MAX77796)
#define LDO2_CNFG1                  0x4D
#define LDO2_CNFG2                  0x4E
#define LDO3_CNFG1                  0x4F
#define LDO3_CNFG2                  0x50
#define LDO4_CNFG1                  0x55 /* SAFEL_CNFG1 */
#define LDO4_CNFG2                  0x56 /* SAFEL_CNFG2 */
#endif

#define LDO_INT1                    0x57
#define LDO_INT2                    0x58
#define LDO_INT1M                   0x59
#define LDO_INT2M                   0x5A
#define LDO_CNFG3                   0x5B

#define LDO_CNFG1_PWRMD             BITS(7,6)
#define LDO_CNFG1_TV                BITS(5,0)

#define LDO_CNFG2_OVCLMP_EN         BIT (7)
#define LDO_CNFG2_ALPM_EN           BIT (6)
#define LDO_CNFG2_COMP              BITS(5,4)
#define LDO_CNFG2_POK               BIT (3)
#define LDO_CNFG2_IMON_EN           BIT (2)
#define LDO_CNFG2_ADE               BIT (1)
#define LDO_CNFG2_RAMP              BIT (0)

#define LDO_CNFG3_L_IMON_TF         BITS(5,3)
#define LDO_CNFG3_L_IMON_EN         BIT (2)
#define LDO_CNFG3_SBIASEN           BIT (1)
#define LDO_CNFG3_BIASEN            BIT (0)

#define LDO_PWRMD_OFF               0b00 /* Off */
#define LDO_PWRMD_DSM               0b01 /* Dynamic Standby Mode */
#define LDO_PWRMD_FSM               0b10 /* Forced Standby Mode */
#define LDO_PWRMD_NOR               0b11 /* Normal Operation */

#define LDO_BITMAP_SZ               BITS_TO_LONGS(LDO_NREG)

struct max77696_ldo_vid {
    int  n_volts;
    int *vtable;
    int  offset_uV, step_uV;
};

struct max77696_ldo_desc {
    struct regulator_desc   rdesc;
    s32                     enable_rate; /* Soft-Start slew rate */
    struct max77696_ldo_vid vid;
    struct max77696_bitdesc pwrmd;
    struct max77696_bitdesc tv;
    struct max77696_bitdesc ovclmp_en;
    struct max77696_bitdesc alpm_en;
    struct max77696_bitdesc comp;
    struct max77696_bitdesc pok;
    struct max77696_bitdesc imon_en;
    struct max77696_bitdesc ad_en;
    struct max77696_bitdesc ramp;
};

struct max77696_ldos;
struct max77696_ldo {
    struct max77696_ldos      *parent;
    struct max77696_io        *io;
    struct max77696_ldo_desc  *desc;

    struct platform_device    *pdev;
    struct regulator_dev      *rdev;

    unsigned int               mode;
};

struct max77696_ldos {
    struct mutex                         lock;
    struct max77696_ldos_platform_data  *pdata;
    struct max77696_core                *core;
    struct max77696_io                  *io;
    struct device                       *dev;
    struct kobject                      *kobj;
    const struct attribute_group        *attr_grp;

    unsigned long                        imon_enabled_ldos[LDO_BITMAP_SZ];

    struct max77696_ldo                  ldos[LDO_NREG];
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

#define mV_to_uV(_mV)      ((_mV) * 1000)
#define uV_to_mV(_uV)      ((_uV) / 1000)
#define V_to_uV(_V)        (mV_to_uV(_V * 1000))
#define uV_to_V(_uV)       (uV_to_mV(_uV) / 1000)

static __always_inline
bool max77696_ldo_is_enabled (struct max77696_ldo *ldo)
{
    struct device *dev = &ldo->pdev->dev;
    struct max77696_ldo_desc *desc = ldo->desc;
    struct max77696_bitdesc *pwrmd_bitdesc = &desc->pwrmd;
    u16 pwrmd;
    int rc;

    rc = max77696_read_bitdesc(ldo->io, pwrmd_bitdesc, &pwrmd);
    if (unlikely(rc)) {
        dev_err(dev, "PWRMD read error [%d]\n", rc);
        return false; /* assume not enabled */
    }

    return (pwrmd != LDO_PWRMD_OFF);
}

static __always_inline
int max77696_ldo_set_mode (struct max77696_ldo *ldo, unsigned int mode)
{
    struct device *dev = &ldo->pdev->dev;
    struct max77696_ldo_desc *desc = ldo->desc;
    struct max77696_bitdesc *pwrmd_bitdesc = &desc->pwrmd;
    u16 pwrmd;
    int rc;

    switch (mode) {
    case REGULATOR_MODE_FAST:
    case REGULATOR_MODE_NORMAL:
        pwrmd = LDO_PWRMD_NOR;
        break;

    case REGULATOR_MODE_IDLE:
        pwrmd = LDO_PWRMD_DSM;
        break;

    case REGULATOR_MODE_STANDBY:
        pwrmd = LDO_PWRMD_FSM;
        break;

    default:
        rc = -EINVAL;
        goto out;
    }

    rc = max77696_write_bitdesc(ldo->io, pwrmd_bitdesc, pwrmd);
    if (unlikely(rc)) {
        dev_err(dev, "PWRMD write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static __always_inline
int max77696_ldo_list_voltage (struct max77696_ldo *ldo, unsigned selector)
{
    struct max77696_ldo_vid *vid = &ldo->desc->vid;
    int rc;

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(&ldo->pdev->dev, "voltage control not supported\n");
        rc = 0;
        goto out;
    }

    /* table */
    if (unlikely(vid->vtable)) {
        rc = vid->vtable[min(vid->n_volts-1, (int)selector)];
        goto out;
    }

    /* linear */
    rc = vid->offset_uV + min(vid->n_volts-1, (int)selector) * vid->step_uV;

out:
    dev_vdbg(&ldo->pdev->dev, "list voltage - %u => %duV\n", selector, rc);
    return rc;
}

static __always_inline
int max77696_ldo_select_voltage (struct max77696_ldo *ldo, int uV)
{
    struct max77696_ldo_vid *vid = &ldo->desc->vid;
    int rc;

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(&ldo->pdev->dev, "voltage control not supported\n");
        rc = 0;
        goto out;
    }

    /* table */
    if (unlikely(vid->vtable)) {
        /* find index of same or nearest larger value */
        int i, diff_min;
        diff_min = INT_MAX;
        rc       = 0;
        for (i = 0; i < vid->n_volts; i++) {
            int diff;
            if (unlikely(vid->vtable[i] == uV)) {
                rc = i;
                break;
            }
            if (unlikely(vid->vtable[i] < uV)) {
                continue;
            }
            diff = vid->vtable[i] - uV;
            if (diff < diff_min) {
                diff_min = diff;
                rc       = i;
            }
        }
        goto out;
    }

    /* linear */
    rc = DIV_ROUND_UP(uV - vid->offset_uV, vid->step_uV);
    rc = min(vid->n_volts-1, max(0, rc));

out:
    dev_vdbg(&ldo->pdev->dev, "select voltage - %duV => %d\n", uV, rc);
    return rc;
}

static __always_inline
int max77696_ldo_set_voltage_sel (struct max77696_ldo *ldo,
    unsigned selector)
{
    struct device *dev = &ldo->pdev->dev;
    struct max77696_ldo_desc *desc = ldo->desc;
    struct max77696_bitdesc *tv_bitdesc = &desc->tv;
    int rc;

    rc = max77696_write_bitdesc(ldo->io, tv_bitdesc, (u16)selector);
    if (unlikely(rc)) {
        dev_err(dev, "TV write error [%d]\n", rc);
        goto out;
    }

    dev_dbg(dev, "TV written 0x%04Xh\n", (u16)selector);

out:
    return rc;
}

static __always_inline
int max77696_ldo_get_voltage_sel (struct max77696_ldo *ldo)
{
    struct device *dev = &ldo->pdev->dev;
    struct max77696_ldo_desc *desc = ldo->desc;
    struct max77696_bitdesc *tv_bitdesc = &desc->tv;
    u16 tv = 0;
    int rc;

    rc = max77696_read_bitdesc(ldo->io, tv_bitdesc, &tv);
    if (unlikely(rc)) {
        dev_err(dev, "TV read error [%d]\n", rc);
        rc = -EIO;
        goto out;
    }

    rc = (int)tv;
    dev_dbg(dev, "TV read 0x%04Xh\n", tv);

out:
    return rc;
}

static int max77696_ldo_op_list_voltage (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_ldo *ldo = rdev_get_drvdata(rdev);
    int rc;

    __lock(ldo->parent);

    rc  = max77696_ldo_list_voltage(ldo, selector);
    rc += rdev->constraints ? rdev->constraints->uV_offset : 0;

    __unlock(ldo->parent);
    return rc;
}

static int max77696_ldo_op_set_voltage_sel (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_ldo *ldo = rdev_get_drvdata(rdev);
    int rc;

    __lock(ldo->parent);

    rc = max77696_ldo_set_voltage_sel(ldo, selector);

    __unlock(ldo->parent);
    return rc;
}

static int max77696_ldo_op_get_voltage_sel (struct regulator_dev *rdev)
{
    struct max77696_ldo *ldo = rdev_get_drvdata(rdev);
    int rc;

    __lock(ldo->parent);

    rc = max77696_ldo_get_voltage_sel(ldo);

    __unlock(ldo->parent);
    return rc;
}

static int max77696_ldo_op_enable (struct regulator_dev *rdev)
{
    struct max77696_ldo *ldo = rdev_get_drvdata(rdev);
    unsigned int mode = ldo->mode ? ldo->mode : LDO_DEFAULT_MODE;
    int rc;

    __lock(ldo->parent);

    rc = max77696_ldo_set_mode(ldo, mode);
    if (unlikely(rc)) {
        dev_err(&ldo->pdev->dev, "enabling failed [%d]\n", rc);
        goto out;
    }

out:
    __unlock(ldo->parent);
    return rc;
}

static int max77696_ldo_op_disable (struct regulator_dev *rdev)
{
    struct max77696_ldo *ldo = rdev_get_drvdata(rdev);
    struct max77696_ldo_desc *desc = ldo->desc;
    struct max77696_bitdesc *pwrmd_bitdesc = &desc->pwrmd;
    int rc;

    __lock(ldo->parent);

    rc = max77696_write_bitdesc(ldo->io, pwrmd_bitdesc, LDO_PWRMD_OFF);
    if (unlikely(rc)) {
        dev_err(&ldo->pdev->dev, "disabling failed [%d]\n", rc);
        goto out;
    }

out:
    __unlock(ldo->parent);
    return rc;
}

static int max77696_ldo_op_is_enabled (struct regulator_dev *rdev)
{
    struct max77696_ldo *ldo = rdev_get_drvdata(rdev);
    int rc;

    __lock(ldo->parent);

    rc = max77696_ldo_is_enabled(ldo);

    __unlock(ldo->parent);
    return rc;
}

static int max77696_ldo_op_set_mode (struct regulator_dev *rdev,
    unsigned int mode)
{
    struct max77696_ldo *ldo = rdev_get_drvdata(rdev);
    int rc;

    __lock(ldo->parent);

    if (unlikely(!mode)) {
        rc = -EINVAL;
        goto out;
    }

    ldo->mode = mode;

    if (unlikely(!max77696_ldo_is_enabled(ldo))) {
        rc = 0;
        goto out;
    }

    rc = max77696_ldo_set_mode(ldo, ldo->mode);
    if (unlikely(rc)) {
        dev_err(&ldo->pdev->dev, "failed to set mode [%d]\n", rc);
        goto out;
    }

out:
    __unlock(ldo->parent);
    return rc;
}

static unsigned int max77696_ldo_op_get_mode (struct regulator_dev *rdev)
{
    struct max77696_ldo *ldo = rdev_get_drvdata(rdev);
    struct max77696_ldo_desc *desc = ldo->desc;
    struct max77696_bitdesc *pwrmd_bitdesc = &desc->pwrmd;
    u16 pwrmd;
    int rc;

    __lock(ldo->parent);

    rc = max77696_read_bitdesc(ldo->io, pwrmd_bitdesc, &pwrmd);
    if (unlikely(rc)) {
        dev_err(&ldo->pdev->dev, "PWRMD read error [%d]\n", rc);
        rc = 0; /* diabled */
        goto out;
    }

    switch (pwrmd) {
    case LDO_PWRMD_FSM:
        rc = REGULATOR_MODE_STANDBY;
        break;

    case LDO_PWRMD_DSM:
        rc = REGULATOR_MODE_IDLE;
        break;

    case LDO_PWRMD_NOR:
        rc = REGULATOR_MODE_NORMAL;
        break;

    default:
        rc = 0; /* diabled */
        break;
    }

out:
    __unlock(ldo->parent);
    return (unsigned int)rc;
}

static int max77696_ldo_op_enable_time (struct regulator_dev *rdev)
{
    struct max77696_ldo *ldo = rdev_get_drvdata(rdev);
    struct max77696_ldo_desc *desc = ldo->desc;
    int uV, selector, rc;

    __lock(ldo->parent);

    selector = max77696_ldo_get_voltage_sel(ldo);
    if (unlikely(selector < 0)) {
        dev_err(&ldo->pdev->dev,
            "failed to read current TV setting [%d]\n", selector);
        uV = desc->vid.offset_uV + (desc->vid.n_volts - 1) * desc->vid.step_uV;
        goto out;
    }

    uV = max77696_ldo_list_voltage(ldo, (unsigned)selector);

out:
    if (likely(desc->enable_rate > 0)) {
        rc = DIV_ROUND_UP(uV, (int)desc->enable_rate);
    } else {
        rc = 0; /* no delay */
    }

    dev_dbg(&ldo->pdev->dev, "start up delay %dusec for %uV\n", rc, uV);

    __unlock(ldo->parent);
    return rc;
}

static struct regulator_ops max77696_ldo_ops = {
    /* enumerate supported voltages */
    .list_voltage         = max77696_ldo_op_list_voltage,

    /* get/set regulator voltage */
    .set_voltage_sel      = max77696_ldo_op_set_voltage_sel,
    .get_voltage_sel      = max77696_ldo_op_get_voltage_sel,

    /* enable/disable regulator */
    .enable               = max77696_ldo_op_enable,
    .disable              = max77696_ldo_op_disable,
    .is_enabled           = max77696_ldo_op_is_enabled,

    /* get/set regulator operating mode (defined in consumer.h) */
    .set_mode             = max77696_ldo_op_set_mode,
    .get_mode             = max77696_ldo_op_get_mode,

    /* Time taken to enable or set voltage on the regulator */
    .enable_time          = max77696_ldo_op_enable_time,
//  .set_voltage_time_sel = max77696_ldo_op_set_voltage_time_sel,

    /* the operations below are for configuration of regulator state when
     * its parent PMIC enters a global STANDBY/HIBERNATE state */
//  .set_suspend_voltage = max77696_ldo_op_set_suspend_voltage,
//  .set_suspend_enable  = max77696_ldo_op_set_suspend_enable,
//  .set_suspend_disable = max77696_ldo_op_set_suspend_disable,
//  .set_suspend_mode    = max77696_ldo_op_set_suspend_mode,
};

#define __LDO_DESC_RDESC(_id, _n_volts) \
        {\
            .name       = LDO_VREG_NAME #_id,\
            .id         = MAX77696_VREG_LDO##_id,\
            .n_voltages = (unsigned)(_n_volts),\
            .ops        = &max77696_ldo_ops,\
            .type       = REGULATOR_VOLTAGE,\
            .owner      = THIS_MODULE,\
        }
#define __LDO_DESC_VID_TABLE(_vtable) \
        {\
            .n_volts     = (int)ARRAY_SIZE(_vtable),\
            .vtable      = _vtable,\
            .offset_uV   = 0,\
            .step_uV     = 0,\
        }
#define __LDO_DESC_VID_LINEAR(_n_volts, _offset, _step) \
        {\
            .n_volts     = _n_volts,\
            .vtable      = NULL,\
            .offset_uV   = _offset,\
            .step_uV     = _step,\
        }
#define __LDO_DESC_BITDESC(_id) \
        .pwrmd     = MAX77696_BITDESC(LDO##_id##_CNFG1, LDO_CNFG1_PWRMD    ),\
        .tv        = MAX77696_BITDESC(LDO##_id##_CNFG1, LDO_CNFG1_TV       ),\
        .ovclmp_en = MAX77696_BITDESC(LDO##_id##_CNFG2, LDO_CNFG2_OVCLMP_EN),\
        .alpm_en   = MAX77696_BITDESC(LDO##_id##_CNFG2, LDO_CNFG2_ALPM_EN  ),\
        .comp      = MAX77696_BITDESC(LDO##_id##_CNFG2, LDO_CNFG2_COMP     ),\
        .pok       = MAX77696_BITDESC(LDO##_id##_CNFG2, LDO_CNFG2_POK      ),\
        .imon_en   = MAX77696_BITDESC(LDO##_id##_CNFG2, LDO_CNFG2_IMON_EN  ),\
        .ad_en     = MAX77696_BITDESC(LDO##_id##_CNFG2, LDO_CNFG2_ADE      ),\
        .ramp      = MAX77696_BITDESC(LDO##_id##_CNFG2, LDO_CNFG2_RAMP     )

#define TYPE1_MIN_UV   800000
#define TYPE1_MAX_UV   2375000
#define TYPE1_STEP_UV  25000
#define TYPE1_N_VOLTS  ((((TYPE1_MAX_UV)-(TYPE1_MIN_UV))/(TYPE1_STEP_UV))+1)

#define MAX77696_LDO_DESC_TYPE1(_id) \
        [MAX77696_VREG_LDO##_id] = {\
            .rdesc = __LDO_DESC_RDESC(_id, TYPE1_N_VOLTS),\
            .vid   = __LDO_DESC_VID_LINEAR(TYPE1_N_VOLTS,\
                TYPE1_MIN_UV, TYPE1_STEP_UV),\
            __LDO_DESC_BITDESC(_id),\
        }

#define TYPE2_MIN_UV   800000
#define TYPE2_MAX_UV   3950000
#define TYPE2_STEP_UV  50000
#define TYPE2_N_VOLTS  ((((TYPE2_MAX_UV)-(TYPE2_MIN_UV))/(TYPE2_STEP_UV))+1)

#define MAX77696_LDO_DESC_TYPE2(_id) \
        [MAX77696_VREG_LDO##_id] = {\
            .rdesc = __LDO_DESC_RDESC(_id, TYPE2_N_VOLTS),\
            .vid   = __LDO_DESC_VID_LINEAR(TYPE2_N_VOLTS,\
                TYPE2_MIN_UV, TYPE2_STEP_UV),\
            __LDO_DESC_BITDESC(_id),\
        }

#define TYPE3_MIN_UV   2200000
#define TYPE3_MAX_UV   5350000
#define TYPE3_STEP_UV  50000
#define TYPE3_N_VOLTS  ((((TYPE3_MAX_UV)-(TYPE3_MIN_UV))/(TYPE3_STEP_UV))+1)

#define MAX77696_LDO_DESC_TYPE3(_id) \
        [MAX77696_VREG_LDO##_id] = {\
            .rdesc = __LDO_DESC_RDESC(_id, TYPE3_N_VOLTS),\
            .vid   = __LDO_DESC_VID_LINEAR(TYPE3_N_VOLTS,\
                TYPE3_MIN_UV, TYPE3_STEP_UV),\
            __LDO_DESC_BITDESC(_id),\
        }

static struct max77696_ldo_desc max77696_ldo_descs[] = {
    MAX77696_LDO_DESC_TYPE2( 1),
    MAX77696_LDO_DESC_TYPE2( 2),
    MAX77696_LDO_DESC_TYPE2( 3),
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    MAX77696_LDO_DESC_TYPE1( 4),
    MAX77696_LDO_DESC_TYPE1( 5),
    MAX77696_LDO_DESC_TYPE2( 6),
    MAX77696_LDO_DESC_TYPE2( 7),
    MAX77696_LDO_DESC_TYPE1( 8),
    MAX77696_LDO_DESC_TYPE1( 9),
    MAX77696_LDO_DESC_TYPE3(10),
#elif defined(CONFIG_MAX77796)
    MAX77696_LDO_DESC_TYPE3( 4),
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
struct regulator_dev *max77696_ldo_register_rdev (struct device *dev)
{
    struct max77696_ldo *ldo = dev_get_drvdata(dev);
    struct regulator_config config;

    memset(&config, 0x00, sizeof(config));
    config.dev         = dev;
    config.init_data   = dev_get_platdata(dev);
    config.driver_data = ldo;
    config.of_node     = dev->of_node;

    return regulator_register(&ldo->desc->rdesc, &config);
}
#else /* LINUX_VERSION_CODE ... */
static __always_inline
struct regulator_dev *max77696_ldo_register_rdev (struct device *dev)
{
    struct max77696_ldo *ldo = dev_get_drvdata(dev);

    return regulator_register(&ldo->desc->rdesc, dev,
        dev_get_platdata(dev), ldo, dev->of_node);
}
#endif /* LINUX_VERSION_CODE ... */

static int max77696_ldo_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_ldo *ldo = dev_get_drvdata(dev);
    int rc;

    /* Register my own regulator device */
    ldo->rdev = max77696_ldo_register_rdev(dev);
    if (unlikely(IS_ERR(ldo->rdev))) {
        rc = PTR_ERR(ldo->rdev);
        dev_err(dev, "failed to register regulator [%d]\n", rc);
        ldo->rdev = NULL;
        goto abort;
    }

    dev_set_drvdata(dev, ldo);

    return 0;

abort:
    if (likely(ldo->rdev)) {
        regulator_unregister(ldo->rdev);
        ldo->rdev = NULL;
    }
    return rc;
}

static int max77696_ldo_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_ldo *ldo = dev_get_drvdata(dev);

    regulator_unregister(ldo->rdev);
    ldo->rdev = NULL;

    return 0;
}

static struct platform_driver max77696_ldo_driver = {
    .probe       = max77696_ldo_probe,
    .remove      = max77696_ldo_remove,
    .driver.name = LDO_VREG_NAME,
};

/******************************************************************************/

static __inline int max77696_ldo_init_vreg_dev (struct max77696_ldo *ldo,
    struct max77696_ldo_cfg_data *cfg_data)
{
    struct device *dev = &ldo->pdev->dev;
    struct max77696_ldo_desc *desc = ldo->desc;
    struct max77696_bitdesc *bitdesc;
    int rc = 0;

    /* Soft-start slew rate selection */
    bitdesc = &desc->ramp;
    if (likely(__check_valid_max77696_bitdesc(bitdesc))) {
        rc = max77696_write_bitdesc(ldo->io, bitdesc, cfg_data->sssr);
        if (unlikely(rc)) {
            dev_err(dev, "failed to set soft-start slew rate for %s [%d]\n",
                desc->rdesc.name, rc);
            goto out;
        }
    }

    desc->enable_rate = cfg_data->sssr == MAX77696_LDO_SSSR_100000 ? 100000 :
                        cfg_data->sssr == MAX77696_LDO_SSSR_5000   ? 5000   : 0;

out:
    return rc;
}

static void max77696_ldo_unregister_vreg_dev (struct max77696_ldo *ldo)
{
    if (likely(ldo->pdev)) {
        platform_device_del(ldo->pdev);
        ldo->pdev = NULL;
    }
}

static int max77696_ldo_register_vreg_dev (struct max77696_ldo *ldo,
    struct max77696_ldo_cfg_data *cfg_data)
{
    struct device *parent_dev = ldo->parent->dev;
    struct max77696_ldo_desc *desc = ldo->desc;
    u8 ldo_id = cfg_data->ldo_id;
    int rc;

    /* just in order to make pdev ldo_id connecting to name
     *   LDO1    1
     *   LDO2    2
     *   LDO3    3
     *   ...
     *   LDO9    9
     *   LDO10  10
     */

    ldo->pdev = platform_device_alloc(LDO_VREG_NAME, ldo_id + 1);
    if (unlikely(!ldo->pdev)) {
        dev_err(parent_dev, "failed to alloc pdev for %s\n", desc->rdesc.name);
        return -ENOMEM;
    }

    dev_set_drvdata(&ldo->pdev->dev, ldo);
    ldo->pdev->dev.platform_data = &cfg_data->init_data;
    ldo->pdev->dev.parent        = parent_dev;

    rc = max77696_ldo_init_vreg_dev(ldo, cfg_data);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to initialize %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    rc = platform_device_add(ldo->pdev);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to pdev for %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    return 0;

abort:
    max77696_ldo_unregister_vreg_dev(ldo);
    return rc;
}

/******************************************************************************/

#define LDOS_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_ldos_attr[] = {
    NULL
};

static const struct attribute_group max77696_ldos_attr_group = {
    .attrs = max77696_ldos_attr,
};

static void max77696_ldos_unregister_vreg (struct max77696_ldos *ldos)
{
    int i;

    for (i = 0; i < LDO_NREG; i++) {
        max77696_ldo_unregister_vreg_dev(&ldos->ldos[i]);
    }

    platform_driver_unregister(&max77696_ldo_driver);
}

static int max77696_ldos_register_vreg (struct max77696_ldos *ldos)
{
    struct device *dev = ldos->dev;
    int i, rc;

    rc = platform_driver_register(&max77696_ldo_driver);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register LDO vregs driver [%d]\n", rc);
        goto out;
    }

    for (i = 0; i < ldos->pdata->num_of_cfg_data; i++) {
        struct max77696_ldo_cfg_data *cfg_data;
        struct max77696_ldo *ldo;
        int ldo_id;

        cfg_data = &ldos->pdata->cfg_data[i];
        ldo_id   = cfg_data->ldo_id;

        if (unlikely(ldo_id >= LDO_NREG)) {
            dev_err(dev, "invalid LDO ID - %u\n", ldo_id);
            continue;
        }

        ldo         = &ldos->ldos[ldo_id];
        ldo->parent = ldos;
        ldo->io     = ldos->io;
        ldo->desc   = &max77696_ldo_descs[ldo_id];

        dev_dbg(dev, "registering LDO vreg device for %s ...\n",
            ldo->desc->rdesc.name);

        rc = max77696_ldo_register_vreg_dev(ldo, cfg_data);
        if (unlikely(rc)) {
            dev_err(dev, "failed to register vreg dev for %s [%d]\n",
                ldo->desc->rdesc.name, rc);
            goto out;
        }
    }

out:
    return rc;
}

static void *max77696_ldos_get_platdata (struct max77696_ldos *ldos)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_ldos_platform_data *pdata;
    struct device *dev = ldos->dev;
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

    i = 0;
    for_each_child_of_node(np, cfg_np) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
        struct regulator_init_data *init_data =
            of_get_regulator_init_data(dev, cfg_np);
#else /* VERSION < 3.19.0 */
        struct regulator_init_data *init_data =
            of_get_regulator_init_data(dev, cfg_np, NULL);
#endif /* VERSION ... */

        of_property_u8(cfg_np, "reg", &pdata->cfg_data[i].ldo_id);
        of_property_u8(cfg_np, "sssr", &pdata->cfg_data[i].sssr);

        if (likely(init_data)) {
            memcpy(&pdata->cfg_data[i].init_data,
                init_data,
                sizeof(pdata->cfg_data[i].init_data));
            devm_kfree(dev, init_data);
        }

        /* Overwrite valid modes and ops mask */
        pdata->cfg_data[i].init_data.constraints.valid_modes_mask
            = MAX77696_LDO_VALID_MODES;
        pdata->cfg_data[i].init_data.constraints.valid_ops_mask
            = MAX77696_LDO_VALID_OPS;

        i++;
    }
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "CFGDATA", "%zu", pdata->num_of_cfg_data);
    for (i = 0; i < pdata->num_of_cfg_data; i++) {
        __prop_printk(dev, "CFGDATA", "[%d] LDO%d", i,
            pdata->cfg_data[i].ldo_id + 1);
    }

out:
    return pdata;
}

static __always_inline
void max77696_ldos_destroy (struct max77696_ldos *ldos)
{
    struct device *dev = ldos->dev;

    if (likely(ldos->attr_grp)) {
        sysfs_remove_group(ldos->kobj, ldos->attr_grp);
    }

    max77696_ldos_unregister_vreg(ldos);

#ifdef CONFIG_MAX77696_DT
    if (likely(ldos->pdata)) {
        devm_kfree(dev, ldos->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&ldos->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, ldos);
}

static __devinit int max77696_ldos_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_ldos *ldos;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    ldos = devm_kzalloc(dev, sizeof(*ldos), GFP_KERNEL);
    if (unlikely(!ldos)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*ldos));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, ldos);

    mutex_init(&ldos->lock);
    ldos->core  = core;
    ldos->io    = max77696_get_block_io(dev->parent, LDO);
    ldos->dev   = dev;
    ldos->kobj  = &dev->kobj;

    ldos->pdata = max77696_ldos_get_platdata(ldos);
    if (unlikely(IS_ERR(ldos->pdata))) {
        rc = PTR_ERR(ldos->pdata);
        ldos->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Register individual LDO drivers & devices */
    rc = max77696_ldos_register_vreg(ldos);
    if (unlikely(rc)) {
        goto abort;
    }

    /* Create max77696-ldos sysfs attributes */
    ldos->attr_grp = &max77696_ldos_attr_group;
    rc = sysfs_create_group(ldos->kobj, ldos->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        ldos->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_ldos_destroy(ldos);
    return rc;
}

static __devexit int max77696_ldos_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_ldos *ldos = dev_get_drvdata(dev);

    max77696_ldos_destroy(ldos);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_ldos_suspend (struct device *dev)
{
    return 0;
}

static int max77696_ldos_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_ldos_pm,
    max77696_ldos_suspend, max77696_ldos_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_ldos_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_ldos_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_ldos_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_ldos_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_ldos_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_ldos_probe,
    .remove                = __devexit_p(max77696_ldos_remove),
};

static __init int max77696_ldos_driver_init (void)
{
    return platform_driver_register(&max77696_ldos_driver);
}
module_init(max77696_ldos_driver_init);

static __exit void max77696_ldos_driver_exit (void)
{
    platform_driver_unregister(&max77696_ldos_driver);
}
module_exit(max77696_ldos_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);

/*******************************************************************************
 * MAX77696-external Services
 ******************************************************************************/

int max77696_enable_ldo_imon (struct device *coredev, u8 ldo_id, bool en)
{
    struct device *dev = max77696_dev(DRIVER_NAME);
    struct max77696_ldos *ldos = dev_get_drvdata(dev);
    struct max77696_ldo *ldo;
    struct max77696_ldo_desc *desc;
    struct max77696_bitdesc *imon_en_bitdesc;
    int rc = 0;

    if (unlikely(!ldos)) {
        pr_err(DRIVER_NAME": not ready\n");
        return -ENODEV;
    }

    __lock(ldos);

    if (unlikely(ldo_id >= LDO_NREG)) {
        dev_err(dev, "invalid LDO ID - %u\n", ldo_id);
        rc = -EINVAL;
        goto out;
    }

    ldo             = &ldos->ldos[ldo_id];
    desc            = ldo->desc;
    imon_en_bitdesc = &desc->imon_en;

    if (en) {
        goto enable;
    }
    goto disable;

enable:
    if (unlikely(test_bit(ldo_id, ldos->imon_enabled_ldos))) {
        /* already enabled */
        goto out;
    }

    if (likely(bitmap_empty(ldos->imon_enabled_ldos, LDO_NREG))) {
        rc = max77696_write_reg_bit(ldos->io, LDO_CNFG3, L_IMON_EN, true);
        if (unlikely(rc)) {
            dev_err(ldos->dev, "L_IMON_EN write error [%d]\n", rc);
            goto out;
        }
    }

    rc = max77696_write_bitdesc(ldo->io, imon_en_bitdesc, true);
    if (unlikely(rc)) {
        dev_err(&ldo->pdev->dev, "IMON_EN write error [%d]\n", rc);
        goto out;
    }

    set_bit(ldo_id, ldos->imon_enabled_ldos);
    goto out;

disable:
    if (unlikely(!test_bit(ldo_id, ldos->imon_enabled_ldos))) {
        /* already disabled */
        goto out;
    }

    rc = max77696_write_bitdesc(ldo->io, imon_en_bitdesc, false);
    if (unlikely(rc)) {
        dev_err(&ldo->pdev->dev, "IMON_EN write error [%d]\n", rc);
        goto out;
    }

    clear_bit(ldo_id, ldos->imon_enabled_ldos);

    if (likely(bitmap_empty(ldos->imon_enabled_ldos, LDO_NREG))) {
        rc = max77696_write_reg_bit(ldos->io, LDO_CNFG3, L_IMON_EN, false);
        if (unlikely(rc)) {
            dev_err(ldos->dev, "L_IMON_EN write error [%d]\n", rc);
            goto out;
        }
    }

out:
    __unlock(ldos);
    return rc;
}
EXPORT_SYMBOL(max77696_enable_ldo_imon);
