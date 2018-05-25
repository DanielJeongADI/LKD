/*
 * MAX77696 BUCK Regulators Driver
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

#define DRIVER_DESC    MAX77696_DESC" BUCK Regulators Driver"
#define DRIVER_NAME    MAX77696_BUCK_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define BUCK_VREG_NAME               MAX77696_NAME"-buck"
#define BUCK_NREG                    MAX77696_VREG_NUM_OF_BUCKS

#define BUCK_ENABLE_DELAY            250 /* us */
#define BUCK_FSR_TIME                1   /* us */
#define BUCK_DEFAULT_MODE            REGULATOR_MODE_FAST

#define VOUT1                        0x31
#define VOUT1DVS                     0x32
#define VOUT2                        0x33
#define VOUT2DVS                     0x34
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
#define VOUT3                        0x35
#define VOUT4                        0x36
#define VOUT5                        0x37
#define VOUT6                        0x38
#elif defined(CONFIG_MAX77796)
#define VOUT3                        0x37
#define VOUT4                        0x38
#endif
#define VOUTCNFG1                    0x39
#define VOUTCNFG2                    0x3A
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
#define VOUTCNFG3                    0x3B
#define VOUTCNFG4                    0x3C
#define VOUTCNFG5                    0x3D
#define VOUTCNFG6                    0x3E
#elif defined(CONFIG_MAX77796)
#define VOUTCNFG3                    0x3D
#define VOUTCNFG4                    0x3E
#endif
#define BUCKFREQ                     0x3F
#define BUCKINT                      0x40
#define BUCKINTM                     0x41
#define BUCKINTS                     0x42

#define VOUTCNFG_BUCKRSR             BITS(7,6)
#define VOUTCNFG_BUCKPWRMD           BITS(5,4)
#define VOUTCNFG_BUCKADEN            BIT (3)
#define VOUTCNFG_BUCKFPWMEN          BIT (2)
#define VOUTCNFG_BUCKIMONEN          BIT (1)
#define VOUTCNFG_BUCKFSREN           BIT (0)

#define BUCK_PWRMD_OFF               0b00 /* Off */
#define BUCK_PWRMD_DSM               0b01 /* Dynamic Standby Mode */
#define BUCK_PWRMD_FSM               0b10 /* Forced Standby Mode */
#define BUCK_PWRMD_NOR               0b11 /* Normal Operation */

struct max77696_buck_vid {
    int  n_volts;
    int *vtable;
    int  offset_uV, step_uV;
};

struct max77696_buck_desc {
    struct regulator_desc    rdesc;
    s32                      ramp_up_rate;   /* uV/usec */
    s32                      ramp_down_rate; /* uV/usec */
    struct max77696_buck_vid vid_nor, vid_sus;
    struct max77696_bitdesc  vout;
    struct max77696_bitdesc  vout_dvs;
    struct max77696_bitdesc  rsr;
    struct max77696_bitdesc  pwrmd;
    struct max77696_bitdesc  ad_dis;
    struct max77696_bitdesc  fpwm_en;
    struct max77696_bitdesc  imon_en;
    struct max77696_bitdesc  fsr_dis;
};

struct max77696_bucks;
struct max77696_buck {
    struct max77696_bucks     *parent;
    struct max77696_io        *io;
    struct max77696_buck_desc *desc;

    struct platform_device    *pdev;
    struct regulator_dev      *rdev;

    unsigned int               mode;
};

struct max77696_bucks {
    struct mutex                         lock;
    struct max77696_bucks_platform_data *pdata;
    struct max77696_core                *core;
    struct max77696_io                  *io;
    struct device                       *dev;
    struct kobject                      *kobj;
    const struct attribute_group        *attr_grp;

    struct max77696_buck                 bucks[BUCK_NREG];
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

#define mV_to_uV(_mV)      ((_mV) * 1000)
#define uV_to_mV(_uV)      ((_uV) / 1000)
#define V_to_uV(_V)        (mV_to_uV(_V * 1000))
#define uV_to_V(_uV)       (uV_to_mV(_uV) / 1000)

static __always_inline
bool max77696_buck_is_enabled (struct max77696_buck *buck)
{
    struct device *dev = &buck->pdev->dev;
    struct max77696_buck_desc *desc = buck->desc;
    struct max77696_bitdesc *pwrmd_bitdesc = &desc->pwrmd;
    u16 pwrmd;
    int rc;

    rc = max77696_read_bitdesc(buck->io, pwrmd_bitdesc, &pwrmd);
    if (unlikely(rc)) {
        dev_err(dev, "PWRMD read error [%d]\n", rc);
        return false; /* assume not enabled */
    }

    return (pwrmd != BUCK_PWRMD_OFF);
}

static __always_inline
int max77696_buck_set_mode (struct max77696_buck *buck, unsigned int mode)
{
    struct device *dev = &buck->pdev->dev;
    struct max77696_buck_desc *desc = buck->desc;
    struct max77696_bitdesc *fpwm_en_bitdesc = &desc->fpwm_en;
    struct max77696_bitdesc *pwrmd_bitdesc = &desc->pwrmd;
    u16 fpwm_en, pwrmd;
    int rc;

    switch (mode) {
    case REGULATOR_MODE_FAST:
        fpwm_en = true;
        pwrmd   = BUCK_PWRMD_NOR;
        break;

    case REGULATOR_MODE_NORMAL:
        fpwm_en = false;
        pwrmd   = BUCK_PWRMD_NOR;
        break;

    case REGULATOR_MODE_IDLE:
        fpwm_en = false;
        pwrmd   = BUCK_PWRMD_DSM;
        break;

    case REGULATOR_MODE_STANDBY:
        fpwm_en = false;
        pwrmd   = BUCK_PWRMD_FSM;
        break;

    default:
        rc = -EINVAL;
        goto out;
    }

    rc = max77696_write_bitdesc(buck->io, fpwm_en_bitdesc, fpwm_en);
    if (unlikely(rc)) {
        dev_err(dev, "FPWM_EN write error [%d]\n", rc);
        goto out;
    }

    rc = max77696_write_bitdesc(buck->io, pwrmd_bitdesc, pwrmd);
    if (unlikely(rc)) {
        dev_err(dev, "PWRMD write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static __always_inline
int max77696_buck_list_voltage (struct max77696_buck *buck,
    struct max77696_buck_vid *vid, unsigned selector)
{
    int rc;

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(&buck->pdev->dev, "voltage control not supported\n");
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
    dev_vdbg(&buck->pdev->dev, "list voltage - %u => %duV\n", selector, rc);
    return rc;
}

static __always_inline
int max77696_buck_select_voltage (struct max77696_buck *buck,
    struct max77696_buck_vid *vid, int uV)
{
    int rc;

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(&buck->pdev->dev, "voltage control not supported\n");
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
    dev_vdbg(&buck->pdev->dev, "select voltage - %duV => %d\n", uV, rc);
    return rc;
}

static __always_inline
int max77696_buck_set_voltage_sel (struct max77696_buck *buck,
    unsigned selector)
{
    struct device *dev = &buck->pdev->dev;
    struct max77696_buck_desc *desc = buck->desc;
    struct max77696_bitdesc *vout_bitdesc = &desc->vout;
    int rc;

    rc = max77696_write_bitdesc(buck->io, vout_bitdesc, (u16)selector);
    if (unlikely(rc)) {
        dev_err(dev, "VOUT write error [%d]\n", rc);
        goto out;
    }

    dev_dbg(dev, "VOUT written 0x%04Xh\n", (u16)selector);

out:
    return rc;
}

static __always_inline
int max77696_buck_get_voltage_sel (struct max77696_buck *buck)
{
    struct device *dev = &buck->pdev->dev;
    struct max77696_buck_desc *desc = buck->desc;
    struct max77696_bitdesc *vout_bitdesc = &desc->vout;
    u16 vout = 0;
    int rc;

    rc = max77696_read_bitdesc(buck->io, vout_bitdesc, &vout);
    if (unlikely(rc)) {
        dev_err(dev, "VOUT read error [%d]\n", rc);
        rc = -EIO;
        goto out;
    }

    rc = (int)vout;
    dev_dbg(dev, "VOUT read 0x%04Xh\n", vout);

out:
    return rc;
}

static int max77696_buck_op_list_voltage (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    struct max77696_buck_desc *desc = buck->desc;
    int rc;

    __lock(buck->parent);

    rc  = max77696_buck_list_voltage(buck, &desc->vid_nor, selector);
    rc += rdev->constraints ? rdev->constraints->uV_offset : 0;

    __unlock(buck->parent);
    return rc;
}

static int max77696_buck_op_set_voltage_sel (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    int rc;

    __lock(buck->parent);

    rc = max77696_buck_set_voltage_sel(buck, selector);

    __unlock(buck->parent);
    return rc;
}

static int max77696_buck_op_get_voltage_sel (struct regulator_dev *rdev)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    int rc;

    __lock(buck->parent);

    rc = max77696_buck_get_voltage_sel(buck);

    __unlock(buck->parent);
    return rc;
}

static int max77696_buck_op_enable (struct regulator_dev *rdev)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    unsigned int mode = buck->mode ? buck->mode : BUCK_DEFAULT_MODE;
    int rc;

    __lock(buck->parent);

    rc = max77696_buck_set_mode(buck, mode);
    if (unlikely(rc)) {
        dev_err(&buck->pdev->dev, "enabling failed [%d]\n", rc);
        goto out;
    }

out:
    __unlock(buck->parent);
    return rc;
}

static int max77696_buck_op_disable (struct regulator_dev *rdev)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    struct max77696_buck_desc *desc = buck->desc;
    struct max77696_bitdesc *pwrmd_bitdesc = &desc->pwrmd;
    int rc;

    __lock(buck->parent);

    rc = max77696_write_bitdesc(buck->io, pwrmd_bitdesc, BUCK_PWRMD_OFF);
    if (unlikely(rc)) {
        dev_err(&buck->pdev->dev, "disabling failed [%d]\n", rc);
        goto out;
    }

out:
    __unlock(buck->parent);
    return rc;
}

static int max77696_buck_op_is_enabled (struct regulator_dev *rdev)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    int rc;

    __lock(buck->parent);

    rc = max77696_buck_is_enabled(buck);

    __unlock(buck->parent);
    return rc;
}

static int max77696_buck_op_set_mode (struct regulator_dev *rdev,
    unsigned int mode)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    int rc;

    __lock(buck->parent);

    if (unlikely(!mode)) {
        rc = -EINVAL;
        goto out;
    }

    buck->mode = mode;

    if (unlikely(!max77696_buck_is_enabled(buck))) {
        rc = 0;
        goto out;
    }

    rc = max77696_buck_set_mode(buck, buck->mode);
    if (unlikely(rc)) {
        dev_err(&buck->pdev->dev, "failed to set mode [%d]\n", rc);
        goto out;
    }

out:
    __unlock(buck->parent);
    return rc;
}

static unsigned int max77696_buck_op_get_mode (struct regulator_dev *rdev)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    struct max77696_buck_desc *desc = buck->desc;
    struct max77696_bitdesc *fpwm_en_bitdesc = &desc->fpwm_en;
    struct max77696_bitdesc *pwrmd_bitdesc = &desc->pwrmd;
    u16 fpwm_en, pwrmd;
    int rc;

    __lock(buck->parent);

    rc = max77696_read_bitdesc(buck->io, fpwm_en_bitdesc, &fpwm_en);
    if (unlikely(rc)) {
        dev_err(&buck->pdev->dev, "FPWM_EN read error [%d]\n", rc);
        rc = 0; /* diabled */
        goto out;
    }

    rc = max77696_read_bitdesc(buck->io, pwrmd_bitdesc, &pwrmd);
    if (unlikely(rc)) {
        dev_err(&buck->pdev->dev, "PWRMD read error [%d]\n", rc);
        rc = 0; /* diabled */
        goto out;
    }

    switch (pwrmd) {
    case BUCK_PWRMD_FSM:
        rc = REGULATOR_MODE_STANDBY;
        break;

    case BUCK_PWRMD_DSM:
        rc = REGULATOR_MODE_IDLE;
        break;

    case BUCK_PWRMD_NOR:
        rc = fpwm_en ? REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL;
        break;

    default:
        rc = 0; /* diabled */
        break;
    }

out:
    __unlock(buck->parent);
    return (unsigned int)rc;
}

static int max77696_buck_op_enable_time (struct regulator_dev *rdev)
{
    return BUCK_ENABLE_DELAY;
}

static int max77696_buck_op_set_voltage_time_sel (struct regulator_dev *rdev,
    unsigned int old_selector, unsigned int new_selector)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    struct max77696_buck_desc *desc = buck->desc;
    int old_uV, new_uV, rc = 0; /* no delay */

    __lock(buck->parent);

    old_uV = max77696_buck_list_voltage(buck, &desc->vid_nor, old_selector);
    new_uV = max77696_buck_list_voltage(buck, &desc->vid_nor, new_selector);

    if (unlikely(old_uV == new_uV)) {
        goto out;
    }

    if (old_uV < new_uV) {
        /* rising */
        if (likely(desc->ramp_up_rate > 0)) {
            rc = DIV_ROUND_UP(new_uV - old_uV, (int)desc->ramp_up_rate);
        }
    } else /*if (old_uV > new_uV)*/ {
        /* falling */
        if (likely(desc->ramp_down_rate > 0)) {
            rc = DIV_ROUND_UP(old_uV - new_uV, (int)desc->ramp_down_rate);
        }
    }

out:
    dev_dbg(&buck->pdev->dev, "ramp delay %dusec for %duV -> %uV\n", rc, old_uV,
        new_uV);
    __unlock(buck->parent);
    return rc;
}

static int max77696_buck_op_set_suspend_voltage (struct regulator_dev *rdev,
    int uV)
{
    struct max77696_buck *buck = rdev_get_drvdata(rdev);
    struct max77696_buck_desc *desc = buck->desc;
    struct max77696_bitdesc *vout_dvs_bitdesc = &desc->vout_dvs;
    int selector, rc;

    __lock(buck->parent);

    if (unlikely(!__check_valid_max77696_bitdesc(vout_dvs_bitdesc))) {
        dev_err(&buck->pdev->dev, "not support DVS\n");
        rc = -ENOTSUPP;
        goto out;
    }

    selector = max77696_buck_select_voltage(buck, &desc->vid_sus, uV);
    if (unlikely(selector < 0)) {
        dev_err(&buck->pdev->dev, "invalid suspend voltage - %duV\n", uV);
        rc = -EINVAL;
        goto out;
    }

    rc = max77696_write_bitdesc(buck->io, vout_dvs_bitdesc, (u16)selector);
    if (unlikely(rc)) {
        dev_err(&buck->pdev->dev, "VOUTDVS write error [%d]\n", rc);
        goto out;
    }

    dev_dbg(&buck->pdev->dev, "VOUTDVS written 0x%04Xh\n", (u16)selector);

out:
    __unlock(buck->parent);
    return rc;
}

static struct regulator_ops max77696_buck_ops = {
    /* enumerate supported voltages */
    .list_voltage         = max77696_buck_op_list_voltage,

    /* get/set regulator voltage */
    .set_voltage_sel      = max77696_buck_op_set_voltage_sel,
    .get_voltage_sel      = max77696_buck_op_get_voltage_sel,

    /* enable/disable regulator */
    .enable               = max77696_buck_op_enable,
    .disable              = max77696_buck_op_disable,
    .is_enabled           = max77696_buck_op_is_enabled,

    /* get/set regulator operating mode (defined in consumer.h) */
    .set_mode             = max77696_buck_op_set_mode,
    .get_mode             = max77696_buck_op_get_mode,

    /* Time taken to enable or set voltage on the regulator */
    .enable_time          = max77696_buck_op_enable_time,
    .set_voltage_time_sel = max77696_buck_op_set_voltage_time_sel,

    /* the operations below are for configuration of regulator state when
     * its parent PMIC enters a global STANDBY/HIBERNATE state */
    .set_suspend_voltage = max77696_buck_op_set_suspend_voltage,
//  .set_suspend_enable  = max77696_buck_op_set_suspend_enable,
//  .set_suspend_disable = max77696_buck_op_set_suspend_disable,
//  .set_suspend_mode    = max77696_buck_op_set_suspend_mode,
};

#define __BUCK_DESC_RDESC(_id, _n_volts) \
        {\
            .name       = BUCK_VREG_NAME #_id,\
            .id         = MAX77696_VREG_BUCK##_id,\
            .n_voltages = (unsigned)(_n_volts),\
            .ops        = &max77696_buck_ops,\
            .type       = REGULATOR_VOLTAGE,\
            .owner      = THIS_MODULE,\
        }
#define __BUCK_DESC_VID_TABLE(_vtable) \
        {\
            .n_volts     = (int)ARRAY_SIZE(_vtable),\
            .vtable      = _vtable,\
            .offset_uV   = 0,\
            .step_uV     = 0,\
        }
#define __BUCK_DESC_VID_LINEAR(_n_volts, _offset, _step) \
        {\
            .n_volts     = _n_volts,\
            .vtable      = NULL,\
            .offset_uV   = _offset,\
            .step_uV     = _step,\
        }
#define __BUCK_DESC_BITDESC(_id) \
        .vout     = MAX77696_BITDESC(VOUT##_id     , 0xFF               ),\
        .vout_dvs = MAX77696_BITDESC(VOUT##_id##DVS, 0xFF               ),\
        .rsr      = MAX77696_BITDESC(VOUTCNFG##_id , VOUTCNFG_BUCKRSR   ),\
        .pwrmd    = MAX77696_BITDESC(VOUTCNFG##_id , VOUTCNFG_BUCKPWRMD ),\
        .ad_dis   = MAX77696_BITDESC(VOUTCNFG##_id , VOUTCNFG_BUCKADEN  ),\
        .fpwm_en  = MAX77696_BITDESC(VOUTCNFG##_id , VOUTCNFG_BUCKFPWMEN),\
        .imon_en  = MAX77696_BITDESC(VOUTCNFG##_id , VOUTCNFG_BUCKIMONEN),\
        .fsr_dis  = MAX77696_BITDESC(VOUTCNFG##_id , VOUTCNFG_BUCKFSREN )

#define MIN_UV   600000
#define MAX_UV   3387500
#define STEP_UV  12500
#define N_VOLTS  ((((MAX_UV) - (MIN_UV)) / (STEP_UV)) + 1)

#define MAX77696_BUCK_DESC(_id) \
        [MAX77696_VREG_BUCK##_id] = {\
            .rdesc   = __BUCK_DESC_RDESC(_id, N_VOLTS),\
            .vid_nor = __BUCK_DESC_VID_LINEAR(N_VOLTS, MIN_UV, STEP_UV),\
            .vid_sus = __BUCK_DESC_VID_LINEAR(N_VOLTS, MIN_UV, STEP_UV),\
            __BUCK_DESC_BITDESC(_id),\
        }

#define VOUT3DVS  MAX77696_REG_RSVD
#define VOUT4DVS  MAX77696_REG_RSVD
#define VOUT5DVS  MAX77696_REG_RSVD
#define VOUT6DVS  MAX77696_REG_RSVD

static struct max77696_buck_desc max77696_buck_descs[] = {
    MAX77696_BUCK_DESC(1),
    MAX77696_BUCK_DESC(2),
    MAX77696_BUCK_DESC(3),
    MAX77696_BUCK_DESC(4),
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    MAX77696_BUCK_DESC(5),
    MAX77696_BUCK_DESC(6),
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
struct regulator_dev *max77696_buck_register_rdev (struct device *dev)
{
    struct max77696_buck *buck = dev_get_drvdata(dev);
    struct regulator_config config;

    memset(&config, 0x00, sizeof(config));
    config.dev         = dev;
    config.init_data   = dev_get_platdata(dev);
    config.driver_data = buck;
    config.of_node     = dev->of_node;

    return regulator_register(&buck->desc->rdesc, &config);
}
#else /* LINUX_VERSION_CODE ... */
static __always_inline
struct regulator_dev *max77696_buck_register_rdev (struct device *dev)
{
    struct max77696_buck *buck = dev_get_drvdata(dev);

    return regulator_register(&buck->desc->rdesc, dev,
        dev_get_platdata(dev), buck, dev->of_node);
}
#endif /* LINUX_VERSION_CODE ... */

static int max77696_buck_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_buck *buck = dev_get_drvdata(dev);
    int rc;

    /* Register my own regulator device */
    buck->rdev = max77696_buck_register_rdev(dev);
    if (unlikely(IS_ERR(buck->rdev))) {
        rc = PTR_ERR(buck->rdev);
        dev_err(dev, "failed to register regulator [%d]\n", rc);
        buck->rdev = NULL;
        goto abort;
    }

    dev_set_drvdata(dev, buck);

    return 0;

abort:
    if (likely(buck->rdev)) {
        regulator_unregister(buck->rdev);
        buck->rdev = NULL;
    }
    return rc;
}

static int max77696_buck_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_buck *buck = dev_get_drvdata(dev);

    regulator_unregister(buck->rdev);
    buck->rdev = NULL;

    return 0;
}

static struct platform_driver max77696_buck_driver = {
    .probe       = max77696_buck_probe,
    .remove      = max77696_buck_remove,
    .driver.name = BUCK_VREG_NAME,
};

/******************************************************************************/

static __inline int max77696_buck_init_vreg_dev (struct max77696_buck *buck,
    struct max77696_buck_cfg_data *cfg_data)
{
    struct device *dev = &buck->pdev->dev;
    struct max77696_buck_desc *desc = buck->desc;
    struct max77696_bitdesc *bitdesc;
    int rc = 0;

    /* Rising slew rate selection */
    bitdesc = &desc->rsr;
    if (likely(__check_valid_max77696_bitdesc(bitdesc))) {
        rc = max77696_write_bitdesc(buck->io, bitdesc, cfg_data->rsr);
        if (unlikely(rc)) {
            dev_err(dev, "failed to set rising slew rate for %s [%d]\n",
                desc->rdesc.name, rc);
            goto out;
        }
    }

    desc->ramp_up_rate = cfg_data->rsr == MAX77696_BUCK_RSR_12500 ? 12500 :
                         cfg_data->rsr == MAX77696_BUCK_RSR_25000 ? 25000 :
                         cfg_data->rsr == MAX77696_BUCK_RSR_50000 ? 50000 : 0;

    /* Falling slew rate selection */
    bitdesc = &desc->fsr_dis;
    if (likely(__check_valid_max77696_bitdesc(bitdesc))) {
        rc = max77696_write_bitdesc(buck->io, bitdesc, cfg_data->fsr);
        if (unlikely(rc)) {
            dev_err(dev, "failed to set falling slew rate for %s [%d]\n",
                desc->rdesc.name, rc);
            goto out;
        }
    }

    desc->ramp_down_rate = cfg_data->fsr == MAX77696_BUCK_FSR_DISABLE ? 0 :
        DIV_ROUND_UP(12500, BUCK_FSR_TIME);

    /* DVS setting */
    if (likely(cfg_data->dvs_uV > 0)) {
        bitdesc = &desc->vout_dvs;
        if (likely(__check_valid_max77696_bitdesc(bitdesc))) {
            int selector = max77696_buck_select_voltage(buck, &desc->vid_sus,
                cfg_data->dvs_uV);
            if (unlikely(selector < 0)) {
                dev_err(dev, "invalid DVS output - %duV\n", cfg_data->dvs_uV);
                rc = -EINVAL;
                goto out;
            }
            rc = max77696_write_bitdesc(buck->io, bitdesc, (u16)selector);
            if (unlikely(rc)) {
                dev_err(dev, "failed to set DVS output for %s [%d]\n",
                    desc->rdesc.name, rc);
                goto out;
            }
        }
    }

out:
    return rc;
}

static void max77696_buck_unregister_vreg_dev (struct max77696_buck *buck)
{
    if (likely(buck->pdev)) {
        platform_device_del(buck->pdev);
        buck->pdev = NULL;
    }
}

static int max77696_buck_register_vreg_dev (struct max77696_buck *buck,
    struct max77696_buck_cfg_data *cfg_data)
{
    struct device *parent_dev = buck->parent->dev;
    struct max77696_buck_desc *desc = buck->desc;
    u8 buck_id = cfg_data->buck_id;
    int rc;

    /* just in order to make pdev buck_id connecting to name
     *   BUCK1    1
     *   BUCK2    2
     *   BUCK3    3
     *   ...
     *   BUCK5    5
     *   BUCK6    6
     */

    buck->pdev = platform_device_alloc(BUCK_VREG_NAME, buck_id + 1);
    if (unlikely(!buck->pdev)) {
        dev_err(parent_dev, "failed to alloc pdev for %s\n", desc->rdesc.name);
        return -ENOMEM;
    }

    dev_set_drvdata(&buck->pdev->dev, buck);
    buck->pdev->dev.platform_data = &cfg_data->init_data;
    buck->pdev->dev.parent        = parent_dev;

    rc = max77696_buck_init_vreg_dev(buck, cfg_data);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to initialize %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    rc = platform_device_add(buck->pdev);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to pdev for %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    return 0;

abort:
    max77696_buck_unregister_vreg_dev(buck);
    return rc;
}

/******************************************************************************/

#define BUCKS_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_bucks_attr[] = {
    NULL
};

static const struct attribute_group max77696_bucks_attr_group = {
    .attrs = max77696_bucks_attr,
};

static void max77696_bucks_unregister_vreg (struct max77696_bucks *bucks)
{
    int i;

    for (i = 0; i < BUCK_NREG; i++) {
        max77696_buck_unregister_vreg_dev(&bucks->bucks[i]);
    }

    platform_driver_unregister(&max77696_buck_driver);
}

static int max77696_bucks_register_vreg (struct max77696_bucks *bucks)
{
    struct device *dev = bucks->dev;
    int i, rc;

    rc = platform_driver_register(&max77696_buck_driver);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register BUCK vregs driver [%d]\n", rc);
        goto out;
    }

    for (i = 0; i < bucks->pdata->num_of_cfg_data; i++) {
        struct max77696_buck_cfg_data *cfg_data;
        struct max77696_buck *buck;
        int buck_id;

        cfg_data = &bucks->pdata->cfg_data[i];
        buck_id  = cfg_data->buck_id;

        if (unlikely(buck_id >= BUCK_NREG)) {
            dev_err(dev, "invalid BUCK ID - %u\n", buck_id);
            continue;
        }

        buck         = &bucks->bucks[buck_id];
        buck->parent = bucks;
        buck->io     = bucks->io;
        buck->desc   = &max77696_buck_descs[buck_id];

        dev_dbg(dev, "registering BUCK vreg device for %s ...\n",
            buck->desc->rdesc.name);

        rc = max77696_buck_register_vreg_dev(buck, cfg_data);
        if (unlikely(rc)) {
            dev_err(dev, "failed to register vreg dev for %s [%d]\n",
                buck->desc->rdesc.name, rc);
            goto out;
        }
    }

out:
    return rc;
}

static void *max77696_bucks_get_platdata (struct max77696_bucks *bucks)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_bucks_platform_data *pdata;
    struct device *dev = bucks->dev;
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

        of_property_u8(cfg_np, "reg", &pdata->cfg_data[i].buck_id);
        of_property_u8(cfg_np, "rsr", &pdata->cfg_data[i].rsr);
        pdata->cfg_data[i].fsr = (u8)of_property_read_bool(cfg_np, "fsr_en");
        of_property_u32(cfg_np, "dvs_uV", (u32*)&pdata->cfg_data[i].dvs_uV);

        if (likely(init_data)) {
            memcpy(&pdata->cfg_data[i].init_data,
                init_data,
                sizeof(pdata->cfg_data[i].init_data));
            devm_kfree(dev, init_data);
        }

        /* Overwrite valid modes and ops mask */
        pdata->cfg_data[i].init_data.constraints.valid_modes_mask
            = MAX77696_BUCK_VALID_MODES;
        pdata->cfg_data[i].init_data.constraints.valid_ops_mask
            = MAX77696_BUCK_VALID_OPS;

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
        __prop_printk(dev, "CFGDATA", "[%d] BUCK%d", i,
            pdata->cfg_data[i].buck_id + 1);
    }

out:
    return pdata;
}

static __always_inline
void max77696_bucks_destroy (struct max77696_bucks *bucks)
{
    struct device *dev = bucks->dev;

    if (likely(bucks->attr_grp)) {
        sysfs_remove_group(bucks->kobj, bucks->attr_grp);
    }

    max77696_bucks_unregister_vreg(bucks);

#ifdef CONFIG_MAX77696_DT
    if (likely(bucks->pdata)) {
        devm_kfree(dev, bucks->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&bucks->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, bucks);
}

static __devinit int max77696_bucks_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_bucks *bucks;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    bucks = devm_kzalloc(dev, sizeof(*bucks), GFP_KERNEL);
    if (unlikely(!bucks)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*bucks));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, bucks);

    mutex_init(&bucks->lock);
    bucks->core  = core;
    bucks->io    = max77696_get_block_io(dev->parent, BUCK);
    bucks->dev   = dev;
    bucks->kobj  = &dev->kobj;

    bucks->pdata = max77696_bucks_get_platdata(bucks);
    if (unlikely(IS_ERR(bucks->pdata))) {
        rc = PTR_ERR(bucks->pdata);
        bucks->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Register individual BUCK drivers & devices */
    rc = max77696_bucks_register_vreg(bucks);
    if (unlikely(rc)) {
        goto abort;
    }

    /* Create max77696-bucks sysfs attributes */
    bucks->attr_grp = &max77696_bucks_attr_group;
    rc = sysfs_create_group(bucks->kobj, bucks->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        bucks->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_bucks_destroy(bucks);
    return rc;
}

static __devexit int max77696_bucks_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_bucks *bucks = dev_get_drvdata(dev);

    max77696_bucks_destroy(bucks);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_bucks_suspend (struct device *dev)
{
    return 0;
}

static int max77696_bucks_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_bucks_pm,
    max77696_bucks_suspend, max77696_bucks_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_bucks_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_bucks_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_bucks_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_bucks_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_bucks_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_bucks_probe,
    .remove                = __devexit_p(max77696_bucks_remove),
};

static __init int max77696_bucks_driver_init (void)
{
    return platform_driver_register(&max77696_bucks_driver);
}
module_init(max77696_bucks_driver_init);

static __exit void max77696_bucks_driver_exit (void)
{
    platform_driver_unregister(&max77696_bucks_driver);
}
module_exit(max77696_bucks_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);

/*******************************************************************************
 * MAX77696-external Services
 ******************************************************************************/

int max77696_enable_buck_imon (struct device *coredev, u8 buck_id, bool en)
{
    struct device *dev = max77696_dev(DRIVER_NAME);
    struct max77696_bucks *bucks = dev_get_drvdata(dev);
    struct max77696_buck *buck;
    struct max77696_buck_desc *desc;
    struct max77696_bitdesc *imon_en_bitdesc;
    int rc;

    if (unlikely(!bucks)) {
        pr_err(DRIVER_NAME": not ready\n");
        return -ENODEV;
    }

    __lock(bucks);

    if (unlikely(buck_id >= BUCK_NREG)) {
        dev_err(dev, "invalid BUCK ID - %u\n", buck_id);
        rc = -EINVAL;
        goto out;
    }

    buck            = &bucks->bucks[buck_id];
    desc            = buck->desc;
    imon_en_bitdesc = &desc->imon_en;

    rc = max77696_write_bitdesc(buck->io, imon_en_bitdesc, !!en);
    if (unlikely(rc)) {
        dev_err(&buck->pdev->dev, "IMON_EN write error [%d]\n", rc);
        goto out;
    }

out:
    __unlock(bucks);
    return rc;
}
EXPORT_SYMBOL(max77696_enable_buck_imon);
