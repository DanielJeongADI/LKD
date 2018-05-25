/*
 * MAX77696 EPD Supplies Driver
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

#define DRIVER_DESC    MAX77696_DESC" EPD Supplies Driver"
#define DRIVER_NAME    MAX77696_EPD_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,13,0)
#define INIT_COMPLETION(x)          reinit_completion(&(x))
#endif

#define EPD_RWC_INTERRUPT           0
#define EPD_PWR_CHANGE_TIMEOUT      msecs_to_jiffies(5000)

#define EPD_VREG_NAME               MAX77696_NAME"-epd"
#define EPD_NREG                    MAX77696_VREG_NUM_OF_EPDS

#define EPDCNFG                     0x60
#define EPDCNFG_EPDEN               BIT (7)
#define EPDCNFG_VCOMEN              BIT (6)
#define EPDCNFG_SSHVINP             BIT (5)
#define EPDCNFG_SSVEE               BIT (4)
#define EPDCNFG_SSVNEG              BIT (3)
#define EPDCNFG_SSVPOS              BIT (2)
#define EPDCNFG_SSVDDH              BIT (1)

#define EPDINTS                     0x61
#define EPDINT                      0x62
#define EPDINTM                     0x63

#define EPDVCOM                     0x64
#define EPDVCOM_VCOM                BITS(7,0)
#define EPDVEE                      0x65
#define EPDVEE_VEE                  BITS(7,0)
#define EPDVNEG                     0x66
#define EPDVNEG_VNEG                BITS(7,0)
#define EPDVPOS                     0x67
#define EPDVPOS_VPOS                BITS(7,0)
#define EPDVDDH                     0x68
#define EPDVDDH_VDDH                BITS(7,0)

#define EPDSEQ                      0x69

#define EPDOKINTS                   0x6A
#define EPDOKINT                    0xB2
#define EPDOKINTM                   0xB3
#define EPDOKINT_EPDPOK             BIT (7) /* EPD Power-OK interrupt */
#define EPDOKINT_EPDPDN             BIT (6) /* EPD Powered Down interrupt */

#define EPDVCOMR                    0xB4
#define EPDVCOMR_VCOMR              BIT (0)
#define EPDDIS                      0xB5

struct max77696_epd_vid {
    int  n_volts;
    int *vtable;
    int  offset_uV, step_uV;
};

struct max77696_epd_desc {
    struct regulator_desc   rdesc;
    u16                     pok_irq_bits; /* Power OK interrupt */
    u16                     pdn_irq_bits; /* Powered down interrupt */
    struct max77696_epd_vid vid;
    struct max77696_bitdesc en;
    struct max77696_bitdesc vout;
};

struct max77696_epds;
struct max77696_epd {
    struct max77696_epds      *parent;
    struct max77696_io        *io;
    struct max77696_epd_desc  *desc;

    struct platform_device    *pdev;
    struct regulator_dev      *rdev;
};

struct max77696_epds {
    struct mutex                         lock;
    struct max77696_epds_platform_data  *pdata;
    struct max77696_core                *core;
    struct max77696_io                  *io;
    struct device                       *dev;
    struct kobject                      *kobj;
    const struct attribute_group        *attr_grp;

    unsigned int                         irq;
    u16                                  irq_unmask;
    u16                                  pok_irq_unmask;
    struct completion                    pwr_changed;

    struct max77696_epd                  epds[EPD_NREG];
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

#define mV_to_uV(_mV)      ((_mV) * 1000)
#define uV_to_mV(_uV)      ((_uV) / 1000)
#define V_to_uV(_V)        (mV_to_uV(_V * 1000))
#define uV_to_V(_uV)       (uV_to_mV(_uV) / 1000)

static __always_inline
void max77696_epds_enable_pok_irq (struct max77696_epds *epds, u16 irq_bits)
{
    u16 epdokintm;
    int rc;

    if (unlikely((epds->pok_irq_unmask & irq_bits) == irq_bits)) {
        /* already unmasked or null bit */
        return;
    }

    if (unlikely(!epds->irq_unmask && !epds->pok_irq_unmask)) {
        enable_irq(epds->irq);
      //enable_irq_wake(epds->irq);
    }

    /* set enabled flag */
    epds->pok_irq_unmask |= irq_bits;

    /* unmask irq */
    epdokintm = ~epds->pok_irq_unmask;

    rc = max77696_write(epds->io, EPDOKINTM, epdokintm);
    dev_dbg(epds->dev, "EPDOKINTM written 0x%04X [%d]\n", epdokintm, rc);

    if (unlikely(rc)) {
        dev_err(epds->dev, "EPDOKINTM write error [%d]\n", rc);
    }
}

static __always_inline
void max77696_epds_disable_pok_irq (struct max77696_epds *epds, u16 irq_bits)
{
    u16 epdokintm;
    int rc;

    if (unlikely((epds->pok_irq_unmask & irq_bits) == 0)) {
        /* already masked or null bit */
        return;
    }

    /* clear enabled flag */
    epds->pok_irq_unmask &= ~irq_bits;

    if (unlikely(!epds->irq_unmask && !epds->pok_irq_unmask)) {
      //disable_irq_wake(epds->irq);
        disable_irq(epds->irq);
    }

    /* mask irq */
    epdokintm = ~epds->pok_irq_unmask;

    rc = max77696_write(epds->io, EPDOKINTM, epdokintm);
    dev_dbg(epds->dev, "EPDOKINTM written 0x%04X [%d]\n", epdokintm, rc);

    if (unlikely(rc)) {
        dev_err(epds->dev, "EPDOKINTM write error [%d]\n", rc);
    }
}

static __always_inline
u16 max77696_epds_read_pok_irq (struct max77696_epds *epds)
{
    u16 epdokint;
    int rc;

    rc = max77696_read(epds->io, EPDOKINT, &epdokint);
    if (unlikely(rc)) {
        dev_err(epds->dev, "EPDOKINT read error [%d]\n", rc);
        return 0;
    }

    return epdokint;
}

static __always_inline
void max77696_epds_ack_pok_irq (struct max77696_epds *epds)
{
    if (EPD_RWC_INTERRUPT) {
        max77696_write(epds->io, EPDOKINT, ~0);
    }
}

static __always_inline
u16 max77696_epds_read_pok_status (struct max77696_epds *epds)
{
    u16 epdokints;
    int rc;

    rc = max77696_read(epds->io, EPDOKINTS, &epdokints);
    if (unlikely(rc)) {
        dev_err(epds->dev, "EPDOKINTS read error [%d]\n", rc);
        return 0;
    }

    return epdokints;
}

static __always_inline
int max77696_epd_wait_for_power_change (struct max77696_epd *epd, bool pwrup)
{
    struct max77696_epd_desc *desc = epd->desc;
    struct completion *completion = &epd->parent->pwr_changed;
    u16 epdokints, irq_bit = pwrup ? desc->pok_irq_bits : desc->pdn_irq_bits;
    int rc;

    INIT_COMPLETION(*completion);
    max77696_epds_enable_pok_irq(epd->parent, irq_bit);

    epdokints = max77696_epds_read_pok_status(epd->parent);
//  dev_dbg(&epd->pdev->dev, "EPDOKINTS@0 0x%04X\n", epdokints);
    dev_dbg(&epd->pdev->dev, "EPDOKINTS 0x%04X\n", epdokints);

    if (unlikely((epdokints & irq_bit) == irq_bit)) {
        dev_dbg(&epd->pdev->dev, "EPD power %s already done\n",
            pwrup ? "up" : "down");
        rc = 0;
        goto out;
    }

    rc = wait_for_completion_interruptible_timeout(completion,
        EPD_PWR_CHANGE_TIMEOUT);
    if (likely(rc <= 0)) {
        dev_err(&epd->pdev->dev, "timeout to wait for EPD power %s [%d]\n",
            pwrup ? "up" : "down", rc);

        epdokints = max77696_epds_read_pok_status(epd->parent);
//      dev_dbg(&epd->pdev->dev, "EPDOKINTS@1 0x%04X\n", epdokints);
        if (unlikely((epdokints & irq_bit) == irq_bit)) {
            rc = 0;
            goto out;
        }

        rc = -ETIME;
        goto out;
    }

    dev_dbg(&epd->pdev->dev, "EPD power %s successfully done\n",
        pwrup ? "up" : "down");

    rc = 0;

out:
    /* Disable IRQ after completion */
    max77696_epds_disable_pok_irq(epd->parent, irq_bit);
    return rc;
}

static int max77696_epd_op_list_voltage (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_epd *epd = rdev_get_drvdata(rdev);
    struct max77696_epd_vid *vid = &epd->desc->vid;
    int rc;

    __lock(epd->parent);

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(&epd->pdev->dev, "voltage control not supported\n");
        rc = 0;
        goto out;
    }

    /* table */
    if (unlikely(vid->vtable)) {
        rc = vid->vtable[min(vid->n_volts-1, (int)selector)];
    }
    /* linear */
    else {
        rc = vid->offset_uV + min(vid->n_volts-1, (int)selector) * vid->step_uV;
    }

    dev_vdbg(&epd->pdev->dev, "list voltage: 0x%04X => %9d uV\n", selector, rc);
    rc  = MAX77696_EPD_VID(rc);
    rc += rdev->constraints ? rdev->constraints->uV_offset : 0;

out:
    __unlock(epd->parent);
    return rc;
}

static int max77696_epd_op_map_voltage (struct regulator_dev *rdev,
    int min_uV, int max_uV)
{
    struct max77696_epd *epd = rdev_get_drvdata(rdev);
    struct max77696_epd_vid *vid = &epd->desc->vid;
    int rc;

    __lock(epd->parent);

    min_uV -= rdev->constraints ? rdev->constraints->uV_offset : 0;
    max_uV -= rdev->constraints ? rdev->constraints->uV_offset : 0;
    min_uV  = MAX77696_EPD_UV(min_uV);
    max_uV  = MAX77696_EPD_UV(max_uV);

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(&epd->pdev->dev, "voltage control not supported\n");
        rc = 0;
        goto out;
    }

    /* table */
    if (unlikely(vid->vtable)) {
        /* find index of same or closest value */
        int i, diff_min;
        diff_min = INT_MAX;
        rc       = 0;
        for (i = 0; i < vid->n_volts; i++) {
            int diff;
            if (unlikely(vid->vtable[i] == min_uV)) {
                rc = i;
                break;
            }
            if (unlikely(vid->vtable[i] < min_uV)) {
                continue;
            }
            diff = vid->vtable[i] - min_uV;
            diff = abs(diff);
            if (diff < diff_min) {
                diff_min = diff;
                rc       = i;
            }
        }
    }
    /* linear */
    else {
        if (vid->step_uV < 0) {
            rc = (min_uV - vid->offset_uV) / vid->step_uV;
        } else {
            rc = DIV_ROUND_UP(min_uV - vid->offset_uV, vid->step_uV);
        }
        rc = min(vid->n_volts-1, max(0, rc));
    }

    dev_vdbg(&epd->pdev->dev, "select voltage: %9d uV => 0x%04X\n", min_uV, rc);

out:
    __unlock(epd->parent);
    return rc;
}

static int max77696_epd_op_set_voltage_sel (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_epd *epd = rdev_get_drvdata(rdev);
    struct max77696_epd_desc *desc = epd->desc;
    struct max77696_bitdesc *vout_bitdesc = &desc->vout;
    int rc;

    __lock(epd->parent);

    rc = max77696_write_bitdesc(epd->io, vout_bitdesc, (u16)selector);
    if (unlikely(rc)) {
        dev_err(&epd->pdev->dev, "setting voltage failed [%d]\n", rc);
        goto out;
    }

    dev_dbg(&epd->pdev->dev, "voltage set 0x%04X\n", (u16)selector);

out:
    __unlock(epd->parent);
    return rc;
}

static int max77696_epd_op_get_voltage_sel (struct regulator_dev *rdev)
{
    struct max77696_epd *epd = rdev_get_drvdata(rdev);
    struct max77696_epd_desc *desc = epd->desc;
    struct max77696_bitdesc *vout_bitdesc = &desc->vout;
    u16 vout = 0;
    int rc;

    __lock(epd->parent);

    rc = max77696_read_bitdesc(epd->io, vout_bitdesc, &vout);
    if (unlikely(rc)) {
        dev_err(&epd->pdev->dev, "getting voltage failed [%d]\n", rc);
        rc = -EIO;
        goto out;
    }

    rc = (int)vout;
    dev_dbg(&epd->pdev->dev, "voltage get 0x%04X\n", vout);

out:
    __unlock(epd->parent);
    return rc;
}

static int max77696_epd_op_enable (struct regulator_dev *rdev)
{
    struct max77696_epd *epd = rdev_get_drvdata(rdev);
    struct max77696_epd_desc *desc = epd->desc;
    struct max77696_bitdesc *en_bitdesc = &desc->en;
    int rc;

    __lock(epd->parent);

    rc = max77696_write_bitdesc(epd->io, en_bitdesc, true);
    if (unlikely(rc)) {
        dev_err(&epd->pdev->dev, "enabling failed [%d]\n", rc);
        goto out;
    }

    rc = max77696_epd_wait_for_power_change(epd, true);
    if (unlikely(rc)) {
        goto out;
    }

out:
    __unlock(epd->parent);
    return rc;
}

static int max77696_epd_op_disable (struct regulator_dev *rdev)
{
    struct max77696_epd *epd = rdev_get_drvdata(rdev);
    struct max77696_epd_desc *desc = epd->desc;
    struct max77696_bitdesc *en_bitdesc = &desc->en;
    int rc;

    __lock(epd->parent);

    rc = max77696_write_bitdesc(epd->io, en_bitdesc, false);
    if (unlikely(rc)) {
        dev_err(&epd->pdev->dev, "disabling failed [%d]\n", rc);
        goto out;
    }

    rc = max77696_epd_wait_for_power_change(epd, false);
    if (unlikely(rc)) {
        goto out;
    }

out:
    __unlock(epd->parent);
    return rc;
}

static int max77696_epd_op_is_enabled (struct regulator_dev *rdev)
{
    struct max77696_epd *epd = rdev_get_drvdata(rdev);
    struct max77696_epd_desc *desc = epd->desc;
    struct max77696_bitdesc *en_bitdesc = &desc->en;
    u16 en;
    int rc;

    __lock(epd->parent);

    rc = max77696_read_bitdesc(epd->io, en_bitdesc, &en);
    if (unlikely(rc)) {
        dev_err(&epd->pdev->dev, "EN read error [%d]\n", rc);
        rc = false; /* assume not enabled */
        goto out;
    }

    rc = !!en;

out:
    __unlock(epd->parent);
    return rc;
}

static struct regulator_ops max77696_epd_ops = {
    /* enumerate supported voltages */
    .list_voltage         = max77696_epd_op_list_voltage,

    /* get/set regulator voltage */
    .map_voltage          = max77696_epd_op_map_voltage,
    .set_voltage_sel      = max77696_epd_op_set_voltage_sel,
    .get_voltage_sel      = max77696_epd_op_get_voltage_sel,
};

static struct regulator_ops max77696_epd_disp_ops = {
    /* enable/disable regulator */
    .enable               = max77696_epd_op_enable,
    .disable              = max77696_epd_op_disable,
    .is_enabled           = max77696_epd_op_is_enabled,
};

static int max77696_epd_vcom_op_set_voltage_sel (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77696_epd *epd = rdev_get_drvdata(rdev);
    struct max77696_epd_desc *desc = epd->desc;
    struct max77696_bitdesc *vout_bitdesc = &desc->vout;
    u16 vcomr, vout;
    int rc;

    __lock(epd->parent);

    vcomr = (u16)(selector >> 8);
    vout  = (u16)(selector & 0xFF);

    rc = max77696_write_reg_bit(epd->io, EPDVCOMR, VCOMR, vcomr);
    if (unlikely(rc)) {
        dev_err(&epd->pdev->dev, "EPDVCOMR write error [%d]\n", rc);
        rc = -EIO;
        goto out;
    }

    rc = max77696_write_bitdesc(epd->io, vout_bitdesc, vout);
    if (unlikely(rc)) {
        dev_err(&epd->pdev->dev, "setting voltage failed [%d]\n", rc);
        goto out;
    }

    dev_dbg(&epd->pdev->dev, "voltage set 0x%04X\n", (u16)selector);

out:
    __unlock(epd->parent);
    return rc;
}

static int max77696_epd_vcom_op_get_voltage_sel (struct regulator_dev *rdev)
{
    struct max77696_epd *epd = rdev_get_drvdata(rdev);
    struct max77696_epd_desc *desc = epd->desc;
    struct max77696_bitdesc *vout_bitdesc = &desc->vout;
    u16 vcomr, vout = 0;
    int rc;

    __lock(epd->parent);

    rc = max77696_read_bitdesc(epd->io, vout_bitdesc, &vout);
    if (unlikely(rc)) {
        dev_err(&epd->pdev->dev, "getting voltage failed [%d]\n", rc);
        rc = -EIO;
        goto out;
    }

    rc = max77696_read_reg_bit(epd->io, EPDVCOMR, VCOMR, &vcomr);
    if (unlikely(rc)) {
        dev_err(&epd->pdev->dev, "EPDVCOMR read error [%d]\n", rc);
        rc = -EIO;
        goto out;
    }

    vout |= (vcomr << 8);

    rc = (int)vout;
    dev_dbg(&epd->pdev->dev, "voltage get 0x%04X\n", vout);

out:
    __unlock(epd->parent);
    return rc;
}

static struct regulator_ops max77696_epd_vcom_ops = {
    /* enumerate supported voltages */
    .list_voltage         = max77696_epd_op_list_voltage,

    /* get/set regulator voltage */
    .map_voltage          = max77696_epd_op_map_voltage,
    .set_voltage_sel      = max77696_epd_vcom_op_set_voltage_sel,
    .get_voltage_sel      = max77696_epd_vcom_op_get_voltage_sel,

    /* enable/disable regulator */
    .enable               = max77696_epd_op_enable,
    .disable              = max77696_epd_op_disable,
    .is_enabled           = max77696_epd_op_is_enabled,
};

#define max77696_epd_vee_ops   max77696_epd_ops
#define max77696_epd_vneg_ops  max77696_epd_ops
#define max77696_epd_vpos_ops  max77696_epd_ops
#define max77696_epd_vddh_ops  max77696_epd_ops

#define __EPD_DESC_RDESC(_id, _n_volts) \
        {\
            .name       = EPD_VREG_NAME #_id,\
            .id         = MAX77696_VREG_EPD_##_id,\
            .n_voltages = (unsigned)(_n_volts),\
            .ops        = EPD_##_id##_OPS,\
            .type       = REGULATOR_VOLTAGE,\
            .owner      = THIS_MODULE,\
        }
#define __EPD_DESC_VID_TABLE(_vtable) \
        {\
            .n_volts     = (int)ARRAY_SIZE(_vtable),\
            .vtable      = _vtable,\
            .offset_uV   = 0,\
            .step_uV     = 0,\
        }
#define __EPD_DESC_VID_LINEAR(_n_volts, _offset, _step) \
        {\
            .n_volts     = _n_volts,\
            .vtable      = NULL,\
            .offset_uV   = _offset,\
            .step_uV     = _step,\
        }
#define __EPD_DESC_BITDESC(_id) \
        .en   = MAX77696_BITDESC(EPDCNFG , EPDCNFG_##_id##EN),\
        .vout = MAX77696_BITDESC(EPD##_id, EPD##_id##_##_id )

#define N_VOLTS(_min, _max, _step) \
        (((_max) - (_min)) / (_step) + 1)

#define MAX77696_EPD_DESC(_id, _min, _max, _step) \
        [MAX77696_VREG_EPD_##_id] = {\
            .rdesc        = __EPD_DESC_RDESC(_id, N_VOLTS(_min, _max, _step)),\
            .pok_irq_bits = EPDOKINT_##_id##POK,\
            .pdn_irq_bits = EPDOKINT_##_id##PDN,\
            .vid          = __EPD_DESC_VID_LINEAR(N_VOLTS(_min, _max, _step),\
                _min, _step),\
            __EPD_DESC_BITDESC(_id),\
        }

#define EPD_DISP_OPS      (&max77696_epd_disp_ops)
#define EPD_VCOM_OPS      (&max77696_epd_vcom_ops)
#define EPD_VEE_OPS       (&max77696_epd_vee_ops)
#define EPD_VNEG_OPS      (&max77696_epd_vneg_ops)
#define EPD_VPOS_OPS      (&max77696_epd_vpos_ops)
#define EPD_VDDH_OPS      (&max77696_epd_vddh_ops)

#define EPDCNFG_DISPEN    EPDCNFG_EPDEN
#define EPDDISP           MAX77696_REG_RSVD
#define EPDDISP_DISP      0
#define EPDOKINT_DISPPOK  EPDOKINT_EPDPOK
#define EPDOKINT_DISPPDN  EPDOKINT_EPDPDN

#define EPDOKINT_VCOMPOK  0
#define EPDOKINT_VCOMPDN  0

#define EPDCNFG_VEEEN     BIT_RSVD
#define EPDOKINT_VEEPOK   0
#define EPDOKINT_VEEPDN   0

#define EPDCNFG_VNEGEN    BIT_RSVD
#define EPDOKINT_VNEGPOK  0
#define EPDOKINT_VNEGPDN  0

#define EPDCNFG_VPOSEN    BIT_RSVD
#define EPDOKINT_VPOSPOK  0
#define EPDOKINT_VPOSPDN  0

#define EPDCNFG_VDDHEN    BIT_RSVD
#define EPDOKINT_VDDHPOK  0
#define EPDOKINT_VDDHPDN  0

static struct max77696_epd_desc max77696_epd_descs[] = {
    /* EPD Power Switch */
    MAX77696_EPD_DESC(DISP,         0,         0,       1),
    /* EPD Power Supplies */
    MAX77696_EPD_DESC(VCOM,         0, - 5058900, -  9900),
    MAX77696_EPD_DESC(VEE , -15000000, -28020000, -420000),
    MAX77696_EPD_DESC(VNEG, - 1620000,   1605750,   12650),
    MAX77696_EPD_DESC(VPOS,   8000000,  18000000,  500000),
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    MAX77696_EPD_DESC(VDDH,  15000000,  29500000,  467742),
#elif defined(CONFIG_MAX77796)
    MAX77696_EPD_DESC(VDDH,  15000000,  30500000,  500000),
#endif
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
struct regulator_dev *max77696_epd_register_rdev (struct device *dev)
{
    struct max77696_epd *epd = dev_get_drvdata(dev);
    struct regulator_config config;

    memset(&config, 0x00, sizeof(config));
    config.dev         = dev;
    config.init_data   = dev_get_platdata(dev);
    config.driver_data = epd;
    config.of_node     = dev->of_node;

    return regulator_register(&epd->desc->rdesc, &config);
}
#else /* LINUX_VERSION_CODE ... */
static __always_inline
struct regulator_dev *max77696_epd_register_rdev (struct device *dev)
{
    struct max77696_epd *epd = dev_get_drvdata(dev);

    return regulator_register(&epd->desc->rdesc, dev,
        dev_get_platdata(dev), epd, dev->of_node);
}
#endif /* LINUX_VERSION_CODE ... */

static int max77696_epd_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_epd *epd = dev_get_drvdata(dev);
    int rc;

    /* Register my own regulator device */
    epd->rdev = max77696_epd_register_rdev(dev);
    if (unlikely(IS_ERR(epd->rdev))) {
        rc = PTR_ERR(epd->rdev);
        dev_err(dev, "failed to register regulator [%d]\n", rc);
        epd->rdev = NULL;
        goto abort;
    }

    dev_set_drvdata(dev, epd);

    return 0;

abort:
    if (likely(epd->rdev)) {
        regulator_unregister(epd->rdev);
        epd->rdev = NULL;
    }
    return rc;
}

static int max77696_epd_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_epd *epd = dev_get_drvdata(dev);

    regulator_unregister(epd->rdev);
    epd->rdev = NULL;

    return 0;
}

static struct platform_driver max77696_epd_driver = {
    .probe       = max77696_epd_probe,
    .remove      = max77696_epd_remove,
    .driver.name = EPD_VREG_NAME,
};

/******************************************************************************/

static __inline int max77696_epd_init_vreg_dev (struct max77696_epd *epd,
    struct max77696_epd_cfg_data *cfg_data)
{
    return 0;
}

static void max77696_epd_unregister_vreg_dev (struct max77696_epd *epd)
{
    if (likely(epd->pdev)) {
        platform_device_del(epd->pdev);
        epd->pdev = NULL;
    }
}

static int max77696_epd_register_vreg_dev (struct max77696_epd *epd,
    struct max77696_epd_cfg_data *cfg_data)
{
    struct device *parent_dev = epd->parent->dev;
    struct max77696_epd_desc *desc = epd->desc;
    u8 epd_id = cfg_data->epd_id;
    int rc;

    /* just in order to make pdev epd_id connecting to name
     *   EPD1    1
     *   EPD2    2
     *   EPD3    3
     *   ...
     *   EPD9    9
     *   EPD10  10
     */

    epd->pdev = platform_device_alloc(EPD_VREG_NAME, epd_id + 1);
    if (unlikely(!epd->pdev)) {
        dev_err(parent_dev, "failed to alloc pdev for %s\n", desc->rdesc.name);
        return -ENOMEM;
    }

    dev_set_drvdata(&epd->pdev->dev, epd);
    epd->pdev->dev.platform_data = &cfg_data->init_data;
    epd->pdev->dev.parent        = parent_dev;

    rc = max77696_epd_init_vreg_dev(epd, cfg_data);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to initialize %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    rc = platform_device_add(epd->pdev);
    if (unlikely(rc)) {
        dev_err(parent_dev,
            "failed to pdev for %s [%d]\n", desc->rdesc.name, rc);
        goto abort;
    }

    return 0;

abort:
    max77696_epd_unregister_vreg_dev(epd);
    return rc;
}

/******************************************************************************/

static irqreturn_t max77696_epds_isr (int irq, void *data)
{
    struct max77696_epds *epds = data;
    u16 epdint = 0, epdints = 0, epdokint = 0, epdokints = 0;

    max77696_read(epds->io, EPDINT   , &epdint   );
    max77696_read(epds->io, EPDINTS  , &epdints  );

    max77696_read(epds->io, EPDOKINT , &epdokint );
    max77696_read(epds->io, EPDOKINTS, &epdokints);

    dev_dbg(epds->dev, "EPDINT   0x%04X EN 0x%04X STS 0x%04X\n",
        epdint  , epds->irq_unmask    , epdints  );
    dev_dbg(epds->dev, "EPDOKINT 0x%04X EN 0x%04X STS 0x%04X\n",
        epdokint, epds->pok_irq_unmask, epdokints);

    if (likely((epdokint & epds->pok_irq_unmask) != 0)) {
        complete(&epds->pwr_changed);
    }

    return IRQ_HANDLED;
}

#define EPDS_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_epds_attr[] = {
    NULL
};

static const struct attribute_group max77696_epds_attr_group = {
    .attrs = max77696_epds_attr,
};

static void max77696_epds_unregister_vreg (struct max77696_epds *epds)
{
    int i;

    for (i = 0; i < EPD_NREG; i++) {
        max77696_epd_unregister_vreg_dev(&epds->epds[i]);
    }

    platform_driver_unregister(&max77696_epd_driver);
}

static int max77696_epds_register_vreg (struct max77696_epds *epds)
{
    struct device *dev = epds->dev;
    int i, rc;

    rc = platform_driver_register(&max77696_epd_driver);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register EPD vregs driver [%d]\n", rc);
        goto out;
    }

    for (i = 0; i < epds->pdata->num_of_cfg_data; i++) {
        struct max77696_epd_cfg_data *cfg_data;
        struct max77696_epd *epd;
        int epd_id;

        cfg_data = &epds->pdata->cfg_data[i];
        epd_id   = cfg_data->epd_id;

        if (unlikely(epd_id >= EPD_NREG)) {
            dev_err(dev, "invalid EPD ID - %u\n", epd_id);
            continue;
        }

        epd         = &epds->epds[epd_id];
        epd->parent = epds;
        epd->io     = epds->io;
        epd->desc   = &max77696_epd_descs[epd_id];

        dev_dbg(dev, "registering EPD vreg device for %s ...\n",
            epd->desc->rdesc.name);

        rc = max77696_epd_register_vreg_dev(epd, cfg_data);
        if (unlikely(rc)) {
            dev_err(dev, "failed to register vreg dev for %s [%d]\n",
                epd->desc->rdesc.name, rc);
            goto out;
        }
    }

out:
    return rc;
}

static void *max77696_epds_get_platdata (struct max77696_epds *epds)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_epds_platform_data *pdata;
    struct device *dev = epds->dev;
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

        of_property_u8(cfg_np, "reg", &pdata->cfg_data[i].epd_id);

        if (likely(init_data)) {
            memcpy(&pdata->cfg_data[i].init_data,
                init_data,
                sizeof(pdata->cfg_data[i].init_data));
            devm_kfree(dev, init_data);
        }

        /* Overwrite valid modes */
        pdata->cfg_data[i].init_data.constraints.valid_modes_mask
            = MAX77696_EPD_VALID_MODES;

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
        __prop_printk(dev, "CFGDATA", "[%d] EPD%d", i,
            pdata->cfg_data[i].epd_id);
    }

out:
    return pdata;
}

static __always_inline
void max77696_epds_destroy (struct max77696_epds *epds)
{
    struct device *dev = epds->dev;

    if (likely(epds->attr_grp)) {
        sysfs_remove_group(epds->kobj, epds->attr_grp);
    }

    max77696_epds_unregister_vreg(epds);

    if (likely(epds->irq > 0)) {
        devm_free_irq(dev, epds->irq, epds);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(epds->pdata)) {
        devm_kfree(dev, epds->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&epds->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, epds);
}

static __devinit int max77696_epds_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_epds *epds;
    u16 epdokint;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    epds = devm_kzalloc(dev, sizeof(*epds), GFP_KERNEL);
    if (unlikely(!epds)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*epds));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, epds);

    mutex_init(&epds->lock);
    epds->core  = core;
    epds->io    = max77696_get_block_io(dev->parent, EPD);
    epds->dev   = dev;
    epds->kobj  = &dev->kobj;

    init_completion(&epds->pwr_changed);

    /* Disable all EPD interrupts */
    epds->irq_unmask     = 0;
    epds->pok_irq_unmask = 0;
    max77696_write(epds->io, EPDINTM  , ~0);
    max77696_write(epds->io, EPDOKINTM, ~0);

    /* Get EPD interrupt status port address & Clear status */
    epdokint = max77696_epds_read_pok_irq(epds);
    max77696_epds_ack_pok_irq(epds);
    dev_dbg(dev, "initial EPD POK interrupt status: 0x%04X\n", epdokint);

    epds->pdata = max77696_epds_get_platdata(epds);
    if (unlikely(IS_ERR(epds->pdata))) {
        rc = PTR_ERR(epds->pdata);
        epds->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Get EPD block IRQ number */
    epds->irq = max77696_get_block_irq(dev->parent, EPD);
    BUG_ON(epds->irq <= 0);

    /* Request system IRQ for EPD */
    rc = devm_request_threaded_irq(dev, (unsigned int)epds->irq, NULL,
        max77696_epds_isr, IRQF_ONESHOT, DRIVER_NAME, epds);
    if (unlikely(rc)) {
        dev_err(dev, "failed to request IRQ(%d) [%d]\n", epds->irq, rc);
        epds->irq = 0;
        goto abort;
    }

    dev_dbg(dev, "IRQ(%d) requested\n", epds->irq);

    disable_irq(epds->irq);

    /* Register individual EPD drivers & devices */
    rc = max77696_epds_register_vreg(epds);
    if (unlikely(rc)) {
        goto abort;
    }

    /* Create max77696-epds sysfs attributes */
    epds->attr_grp = &max77696_epds_attr_group;
    rc = sysfs_create_group(epds->kobj, epds->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        epds->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_epds_destroy(epds);
    return rc;
}

static __devexit int max77696_epds_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_epds *epds = dev_get_drvdata(dev);

    max77696_epds_destroy(epds);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_epds_suspend (struct device *dev)
{
    return 0;
}

static int max77696_epds_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_epds_pm,
    max77696_epds_suspend, max77696_epds_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_epds_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_epds_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_epds_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_epds_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_epds_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_epds_probe,
    .remove                = __devexit_p(max77696_epds_remove),
};

static __init int max77696_epds_driver_init (void)
{
    return platform_driver_register(&max77696_epds_driver);
}
module_init(max77696_epds_driver_init);

static __exit void max77696_epds_driver_exit (void)
{
    platform_driver_unregister(&max77696_epds_driver);
}
module_exit(max77696_epds_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
