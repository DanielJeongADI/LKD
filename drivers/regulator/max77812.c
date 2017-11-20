/*
 * MAX77812 Buck Regulators Driver
 *
 * Copyright (C) 2017 Maxim Integrated
 *
 * This file is part of MAX77812 Linux Driver
 *
 * MAX77812 Linux Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * MAX77812 Linux Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MAX77812 Linux Driver. If not, see http://www.gnu.org/licenses/.
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
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>

#include <linux/irq.h>
#include <linux/regmap.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/max77812.h>

#define DRIVER_DESC    "MAX77812 Buck Regulators Driver"
#define DRIVER_NAME    "max77812"
#define DRIVER_VERSION "1.70205"
#define DRIVER_AUTHOR  "Maxim Integrated"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
#ifndef __devinit
#define __devinit
#endif
#ifndef __devexit
#define __devexit
#endif
#ifndef __devexit_p
#define __devexit_p(x)  (x)
#endif
#endif /* LINUX_VERSION_CODE ... */

#undef  __CONST_FFS
#define __CONST_FFS(_x) \
        ((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ?  0 :  1) :\
                                      ((_x) & 0x04 ?  2 :  3)) :\
                       ((_x) & 0x30 ? ((_x) & 0x10 ?  4 :  5) :\
                                      ((_x) & 0x40 ?  6 :  7)))

#undef  FFS
#if 0
#define FFS(_x) \
        ((_x) ? (__builtin_constant_p(_x) ? __CONST_FFS(_x) : __ffs(_x)) : 0)
#else
#define FFS(_x) ((_x) ? __CONST_FFS(_x) : 0)
#endif

#undef  BITS
#define BITS(_end, _start) \
        ((BIT(_end) - BIT(_start)) + BIT(_end))

#undef  BIT_RSVD
#define BIT_RSVD 0

#ifdef CONFIG_OF
static __always_inline int of_property_u8 (const struct device_node *np,
    const char *propname, u8 *out_value)
{
    u32 buf;
    int rc = of_property_read_u32(np, propname, &buf);

    if (likely(!rc)) {
        *out_value = (u8)buf;
    }

    return rc;
}

static __always_inline int of_property_u16 (const struct device_node *np,
    const char *propname, u16 *out_value)
{
    u32 buf;
    int rc = of_property_read_u32(np, propname, &buf);

    if (likely(!rc)) {
        *out_value = (u16)buf;
    }

    return rc;
}

static __always_inline int of_property_u32 (const struct device_node *np,
    const char *propname, u32 *out_value)
{
    return of_property_read_u32(np, propname, out_value);
}
#endif /* CONFIG_OF */

#define BUCK_VREG_NAME               "max77812-buck"

enum {
    BUCK1 = 0,
    BUCK2,
    BUCK3,
    BUCK4,
    NUM_OF_BUCKS,
};

#define REG_RESET                    0x00

#define INT_SRC                      0x01
#define INT_SRC_M                    0x02
#define INT_SRC_BUCK_INT             BIT (1)
#define INT_SRC_TOPSYS_INT           BIT (0)

#define TOPSYS_INT                   0x03
#define TOPSYS_INT_M                 0x04
#define TOPSYS_INT_WDTRSTB_INT       BIT (4)
#define TOPSYS_INT_UVLO_INT          BIT (3)
#define TOPSYS_INT_TSHDN_INT         BIT (2)
#define TOPSYS_INT_TJCT_140C_INT     BIT (1)
#define TOPSYS_INT_TJCT_120C_INT     BIT (0)

#define TOPSYS_STAT                  0x05
#define TOPSYS_STAT_UVLO             BIT (3)
#define TOPSYS_STAT_TSHDN            BIT (2)
#define TOPSYS_STAT_TJCT_140C        BIT (1)
#define TOPSYS_STAT_TJCT_120C        BIT (0)

#define EN_CTRL                      0x06
#define EN_CTRL_EN_M4_LPM            BIT (7)
#define EN_CTRL_EN_M4                BIT (6)
#define EN_CTRL_EN_M3_LPM            BIT (5)
#define EN_CTRL_EN_M3                BIT (4)
#define EN_CTRL_EN_M2_LPM            BIT (3)
#define EN_CTRL_EN_M2                BIT (2)
#define EN_CTRL_EN_M1_LPM            BIT (1)
#define EN_CTRL_EN_M1                BIT (0)

#define STUP_SEQ1                    0x07
#define STUP_SEQ1_DLY_STEP           BIT (7)
#define STUP_SEQ2                    STUP_SEQ1
#define STUP_SEQ3                    0x08
#define STUP_SEQ4                    0x09
#define STUP_SEQX_STUP_DLY           BITS(4,0)

#define SHDN_SEQ1                    0x0A
#define SHDN_SEQ2                    0x0B
#define SHDN_SEQ3                    0x0C
#define SHDN_SEQ4                    0x0D
#define SHDN_SEQX_SHDN_DLY           BITS(4,0)

#define WDTRSTB_DEB                  0x0E
#define WDTRSTB_DEB_WDT_DEB          BITS(2,0)

/* GPI Registers */
#define GPI_FUNC                     0x0F
#define GPI_FUNC_GPI1_FUNC           BITS(7,4)
#define GPI_FUNC_GPI0_FUNC           BITS(3,0)
#define GPI_DEB1                     0x10
#define GPI_DEB1_LPM_DEB             BITS(7,4)
#define GPI_DEB1_EN_DEB              BITS(3,0)
#define GPI_DEB2                     0x11
#define GPI_DEB2_GPI1_DEB            BITS(7,4)
#define GPI_DEB2_GPI0_DEB            BITS(3,0)
#define GPI_PD_CTRL                  0x12
#define GPI_PD_CTRL_LPM_PD           BIT (3)
#define GPI_PD_CTRL_EN_PD            BIT (2)
#define GPI_PD_CTRL_GPI1_PD          BIT (1)
#define GPI_PD_CTRL_GPI0_PD          BIT (0)

/* Protection Configuration Register */
#define PROT_CFG                     0x13
#define PROT_CFG_TSHDN_EN            BIT (7)
#define PROT_CFG_UVLO_F              BITS(2,0)

#define I2C_CFG                      0x15

#define BUCK_INT                     0x20
#define BUCK_INT_M                   0x21
#define BUCK_INT_M4_POKn_INT         BIT (3)
#define BUCK_INT_M3_POKn_INT         BIT (2)
#define BUCK_INT_M2_POKn_INT         BIT (1)
#define BUCK_INT_M1_POKn_INT         BIT (0)

#define BUCK_STAT                    0x22
#define BUCK_STAT_M4_POKn            BIT (3)
#define BUCK_STAT_M3_POKn            BIT (2)
#define BUCK_STAT_M2_POKn            BIT (1)
#define BUCK_STAT_M1_POKn            BIT (0)

/* Buck Master Output Voltage Setting Registers */
#define M1_VOUT                      0x23
#define M2_VOUT                      0x24
#define M3_VOUT                      0x25
#define M4_VOUT                      0x26

/* Buck Master Default Output Voltage Setting Registers */
#define M1_VOUT_D                    0x27
#define M2_VOUT_D                    0x28
#define M3_VOUT_D                    0x29
#define M4_VOUT_D                    0x2A

/* Buck Master Sleep Mode Output Voltage Setting Registers */
#define M1_VOUT_S                    0x2B
#define M2_VOUT_S                    0x2C
#define M3_VOUT_S                    0x2D
#define M4_VOUT_S                    0x2E

/* Buck Master Configuration Registers */
#define M1_CFG                       0x2F
#define M2_CFG                       0x30
#define M3_CFG                       0x31
#define M4_CFG                       0x32
#define MX_CFG_AD                    BIT (7)
#define MX_CFG_ILIM                  BITS(6,4)
#define MX_CFG_FPWM                  BIT (1)
#define MX_CFG_FSREN                 BIT (0)

/* Buck Global Configuration Registers */
#define GLB_CFG1                     0x33
#define GLB_CFG1_B_SD_SR             BITS(6,4)
#define GLB_CFG1_B_SS_SR             BITS(2,0)
#define GLB_CFG2                     0x34
#define GLB_CFG2_B_RD_SR             BITS(6,4)
#define GLB_CFG2_B_RU_SR             BITS(2,0)
#define GLB_CFG3                     0x35
#define GLB_CFG3_B_ISHARE_EN         BIT (4)
#define GLB_CFG3_B_BDBK_EN           BIT (3)
#define GLB_CFG3_B_PETR_EN           BIT (1)
#define GLB_CFG3_B_NETR_EN           BIT (0)

/* GPI Functions */
#define GPI_FUNC_GLB_EN              0x0
#define GPI_FUNC_M1_EN               0x1
#define GPI_FUNC_M2_EN               0x2
#define GPI_FUNC_M3_EN               0x3
#define GPI_FUNC_M4_EN               0x4
#define GPI_FUNC_GLB_VSEL            0x5
#define GPI_FUNC_M1_VSEL             0x6
#define GPI_FUNC_M2_VSEL             0x7
#define GPI_FUNC_M3_VSEL             0x8
#define GPI_FUNC_M4_VSEL             0x9
#define GPI_FUNC_GLB_LPM             0xA
#define GPI_FUNC_M1_LPM              0xB
#define GPI_FUNC_M2_LPM              0xC
#define GPI_FUNC_M3_LPM              0xD
#define GPI_FUNC_M4_LPM              0xE
#define GPI_FUNC_NO_FUNC             0xF

/* Buck Control Options for En/Dis */
#define BUCK_EN_CTRL_BIT             BIT(0)
#define BUCK_EN_CTRL_GPIO            BIT(1)

/* Buck Control Options for LPM */
#define BUCK_LPM_CTRL_BIT            BIT(0)
#define BUCK_LPM_CTRL_GPIO           BIT(1)

/* Buck Master Current Limit Setting Options (PMOS Peak / NMOS Valley) */
#define BUCK_ILIM_0                  0 /* 3.0A / 2.0A */
#define BUCK_ILIM_1                  1 /* 3.6A / 2.4A */
#define BUCK_ILIM_2                  2 /* 4.2A / 2.8A */
#define BUCK_ILIM_3                  3 /* 4.8A / 3.2A */
#define BUCK_ILIM_4                  4 /* 5.4A / 3.6A */
#define BUCK_ILIM_5                  5 /* 6.0A / 4.0A */
#define BUCK_ILIM_6                  6 /* 6.6A / 4.4A */
#define BUCK_ILIM_7                  7 /* 7.2A / 4.8A */

struct max77812_bitdesc {
    u8 reg, mask, shift;
};

#define __BITDESC(_reg, _bit) \
        { .reg = _reg, .mask = _bit, .shift = (u8)FFS(_bit), }
#define BITDESC(_reg, _bit)   \
        (&((const struct max77812_bitdesc)__BITDESC(_reg, _bit)))

#define BUCK_INTBANK_TOPSYS   0
#define BUCK_INTBANK_BUCK     1
/* Number of interrupt banks */
#define BUCK_NUM_OF_INTBANKS  2

/* Number of interrupts */
#define BUCK_NUM_OF_INTS      MAX77812_NUM_OF_INTS

/* Number of GPIs */
#define NUM_OF_GPIS           2

/* GPIO Names */
#define GPIO_NAME_GLB_EN      DRIVER_NAME"-en"
#define GPIO_NAME_GLB_LPM     DRIVER_NAME"-lpm"
#define GPIO_NAME_GPIx        DRIVER_NAME"-gpi"

struct max77812_intbank_desc {
    const char              *name;
    struct max77812_bitdesc  intr;
    struct max77812_bitdesc  mask;
    u8                       src_intr_reg;
    u8                       src_mask_reg;
};

struct max77812_int_desc {
    const char              *name;
    u8                       bank;
    u8                       intr_bit;
    u8                       mask_bit;
    struct max77812_bitdesc  stat;
};

struct max77812_gpix_desc {
    const char              *name;
    struct max77812_bitdesc  func;
    struct max77812_bitdesc  deb;
    struct max77812_bitdesc  pd_en;
};

struct max77812_gpix {
    struct max77812_gpix_desc *desc;

    bool                       gpio_owner;
    int                        gpio;
};

struct max77812_buck_vid {
    int  n_volts;
    int *vtable;
    int  offset_uV, step_uV;
};

struct max77812_buck_desc {
    struct regulator_desc    rdesc;
    struct max77812_buck_vid vid;

    u16                      gpi_func_mask_en;
    u16                      gpi_func_mask_lpm;
    u16                      gpi_func_mask_vsel;

    /* STAT */
//  struct max77812_bitdesc  pokn;

    /* EN_CTRL */
    struct max77812_bitdesc  en;
    struct max77812_bitdesc  lpm_en;

    /* Mx_VOUT */
    struct max77812_bitdesc  vout;
    struct max77812_bitdesc  vout_d;
    struct max77812_bitdesc  vout_s;

    /* Mx_CFG */
    struct max77812_bitdesc  ad_en;
    struct max77812_bitdesc  ilim;
    struct max77812_bitdesc  fpwm_en;
    struct max77812_bitdesc  fsr_en;

    /* STUP/SHDN_SEQx */
    struct max77812_bitdesc  stup_dly;
    struct max77812_bitdesc  shdn_dly;
};

struct max77812_buck {
    struct list_head               link;
    struct device                 *dev;
    u8                             id; /* BUCK1(0) ... BUCK4(3) */
    struct max77812_buck_desc     *desc;

    bool                           no_idle_mode;

    u8                             en_ctrl_mask;
    u8                             lpm_ctrl_mask;

//  u8                             stup_dly_cnt;
//  u8                             shdn_dly_cnt;

    struct regulator_init_data    *rinitdata;
    struct regulator_dev          *rdev;

    unsigned int                   mode;
};

struct max77812;

struct max77812_io {
    struct regmap *regmap;
    int (*read)(struct max77812*, u8, u8*);
    int (*write)(struct max77812*, u8, u8);
    int (*update_bits)(struct max77812*, u8, u8, u8);
};

struct max77812 {
    struct mutex                         lock;
    struct device                       *dev;
    struct pinctrl                      *pinctrl;
    struct platform_driver              *buck_drv;
    const struct attribute_group        *attr_grp;

    struct max77812_io                  *io;

    u8                                   phase;
    bool                                 glb_en_ctrl_gpio;
    int                                  glb_en_gpio;
    bool                                 glb_lpm_ctrl_gpio;
    int                                  glb_lpm_gpio;

    struct max77812_gpix                 gpi[NUM_OF_GPIS];

//  u8                                   dly_step_ms;
//  u16                                  sd_sr;
//  u16                                  ss_sr;
//  u16                                  rd_sr;
//  u16                                  ru_sr;

    int                                  irq;
    int                                  irq_base;

    u8                                   intbank_to_sync;
    u8                                   intbank_unmask[BUCK_NUM_OF_INTBANKS];

    struct list_head                     bucks;
};

#define __lock(_max77812)        mutex_lock(&(_max77812)->lock)
#define __unlock(_max77812)      mutex_unlock(&(_max77812)->lock)

/* will be initialized during probe() */
static struct bus_type *max77812_bus_type = NULL;

static __always_inline struct max77812 *__find_max77812 (void)
{
    struct device *dev = NULL;

    if (likely(max77812_bus_type)) {
        dev = bus_find_device_by_name(max77812_bus_type, NULL, DRIVER_NAME);
    }

    return dev ? dev_get_drvdata(dev) : NULL;
}

// dly_step bit => msec
static __always_inline int __list_delay_step (u8 dly_step)
{
    return dly_step ? 2 : 1;
}

// slew_rate bit => usec
static __always_inline int __list_slew_rate (u8 slew_rate)
{
    return slew_rate == 0 ?  1250 :
           slew_rate == 1 ?  2500 :
           slew_rate == 2 ?  5000 :
           slew_rate == 3 ? 10000 :
           slew_rate == 4 ? 20000 :
           slew_rate == 5 ? 40000 :
                            60000;
}

static __always_inline int max77812_read (struct max77812 *max77812,
    u8 reg, u8 *val)
{
    return max77812->io->read(max77812, reg, val);
}

static __always_inline int max77812_write (struct max77812 *max77812,
    u8 reg, u8 val)
{
    return max77812->io->write(max77812, reg, val);
}

static __always_inline int max77812_read_bits (struct max77812 *max77812,
    u8 reg, u8 mask, u8* val)
{
    int rc;

    rc = max77812_read(max77812, reg, val);
    if (likely(!rc)) {
        *val &= mask;
    }

    return rc;
}

static __always_inline int max77812_update_bits (struct max77812 *max77812,
    u8 reg, u8 mask, u8 val)
{
    return max77812->io->update_bits(max77812, reg, mask, val);
}

static __always_inline int max77812_read_bitdesc (struct max77812 *max77812,
    const struct max77812_bitdesc *desc, u8 *bitval)
{
    int rc;

    rc = max77812_read_bits(max77812, desc->reg, desc->mask, bitval);
    if (likely(!rc)) {
        *bitval >>= desc->shift;
    }

    return rc;
}

static __always_inline int max77812_write_bitdesc (struct max77812 *max77812,
    const struct max77812_bitdesc *desc, u8 bitval)
{
    return max77812_update_bits(max77812, desc->reg, desc->mask,
        (u8)(bitval << desc->shift));
}

static __always_inline void max77812_select_pinctrl (struct max77812 *max77812,
    const char *state)
{
    struct device *dev = max77812->dev;

    if (likely(max77812->pinctrl)) {
        devm_pinctrl_put(max77812->pinctrl);
        max77812->pinctrl = NULL;
    }

    if (unlikely(!state)) {
        dev_dbg(dev, "pinctrl released\n");
        goto out;
    }

    max77812->pinctrl = devm_pinctrl_get_select(dev, state);

    if (unlikely(IS_ERR(max77812->pinctrl))) {
        dev_err(dev, "failed to select pinctrl %s [%ld]\n", state,
            PTR_ERR(max77812->pinctrl));
        max77812->pinctrl = NULL;
        goto out;
    }

    dev_dbg(dev, "pinctrl state %s selected\n", state);

out:
    return;
}

static int max77812_init_prop (struct max77812 *max77812,
    struct device_node *np, const char *propname,
    const struct max77812_bitdesc *bitdesc, u8 *val)
{
    int rc;

    if (of_property_u8(np, propname, val)) {
        rc = max77812_read_bitdesc(max77812, bitdesc, val);
        if (unlikely(rc)) {
            dev_err(max77812->dev,
                "failed to get prop '%s' [%d]\n", propname, rc);
        }
    } else {
        rc = max77812_write_bitdesc(max77812, bitdesc, *val);
        if (unlikely(rc)) {
            dev_err(max77812->dev,
                "failed to set prop '%s' [%d]\n", propname, rc);
        }
    }

    return rc;
}

#define UVLO_STAT_BITDESC        BITDESC(TOPSYS_STAT, TOPSYS_STAT_UVLO     )
#define TSHDN_STAT_BITDESC       BITDESC(TOPSYS_STAT, TOPSYS_STAT_TSHDN    )
#define TJCT_140C_STAT_BITDESC   BITDESC(TOPSYS_STAT, TOPSYS_STAT_TJCT_140C)
#define TJCT_120C_STAT_BITDESC   BITDESC(TOPSYS_STAT, TOPSYS_STAT_TJCT_120C)

#define DLY_STEP_BITDESC         BITDESC(STUP_SEQ1  , STUP_SEQ1_DLY_STEP   )
#define WDT_DEB_BITDESC          BITDESC(WDTRSTB_DEB, WDTRSTB_DEB_WDT_DEB  )
#define TSHDN_EN_BITDESC         BITDESC(PROT_CFG   , PROT_CFG_TSHDN_EN    )
#define UVLO_F_BITDESC           BITDESC(PROT_CFG   , PROT_CFG_UVLO_F      )
#define SD_SR_BITDESC            BITDESC(GLB_CFG1   , GLB_CFG1_B_SD_SR     )
#define SS_SR_BITDESC            BITDESC(GLB_CFG1   , GLB_CFG1_B_SS_SR     )
#define RD_SR_BITDESC            BITDESC(GLB_CFG2   , GLB_CFG2_B_RD_SR     )
#define RU_SR_BITDESC            BITDESC(GLB_CFG2   , GLB_CFG2_B_RU_SR     )
#define ISHARE_EN_BITDESC        BITDESC(GLB_CFG3   , GLB_CFG3_B_ISHARE_EN )
#define BDBK_EN_BITDESC          BITDESC(GLB_CFG3   , GLB_CFG3_B_BDBK_EN   )
#define PETR_EN_BITDESC          BITDESC(GLB_CFG3   , GLB_CFG3_B_PETR_EN   )
#define NETR_EN_BITDESC          BITDESC(GLB_CFG3   , GLB_CFG3_B_NETR_EN   )

#define GPI_LPM_DEB_BITDESC      BITDESC(GPI_DEB1   , GPI_DEB1_LPM_DEB     )
#define GPI_LPM_PD_BITDESC       BITDESC(GPI_PD_CTRL, GPI_PD_CTRL_LPM_PD   )
#define GPI_EN_DEB_BITDESC       BITDESC(GPI_DEB1   , GPI_DEB1_EN_DEB      )
#define GPI_EN_PD_BITDESC        BITDESC(GPI_PD_CTRL, GPI_PD_CTRL_EN_PD    )

static struct max77812_intbank_desc max77812_intbank_descs[] = {
    #define INTBANK_DESC(_name) \
            [BUCK_INTBANK_##_name] = {\
                .name         = #_name,\
                .intr         = __BITDESC(INT_SRC  , INT_SRC_##_name##_INT),\
                .mask         = __BITDESC(INT_SRC_M, INT_SRC_##_name##_INT),\
                .src_intr_reg = _name##_INT,\
                .src_mask_reg = _name##_INT_M,\
            }
    INTBANK_DESC(TOPSYS),
    INTBANK_DESC(BUCK  ),
};

static struct max77812_int_desc max77812_int_descs[] = {
    #define TOPSYSINT_DESC(_name) \
            [MAX77812_INT_##_name] = {\
                .name     = "TOPSYS_" #_name,\
                .bank     = BUCK_INTBANK_TOPSYS,\
                .intr_bit = TOPSYS_INT_##_name##_INT,\
                .mask_bit = TOPSYS_INT_##_name##_INT,\
                .stat     = __BITDESC(TOPSYS_STAT , TOPSYS_STAT_##_name),\
            }
    #define TOPSYS_STAT_WDTRSTB BIT_RSVD
    TOPSYSINT_DESC(WDTRSTB  ),
    TOPSYSINT_DESC(UVLO     ),
    TOPSYSINT_DESC(TSHDN    ),
    TOPSYSINT_DESC(TJCT_140C),
    TOPSYSINT_DESC(TJCT_120C),

    #define BUCKINT_DESC(_nr) \
            [MAX77812_INT_M##_nr##_POKn] = {\
                .name     = "BUCK" #_nr "_POKn",\
                .bank     = BUCK_INTBANK_BUCK,\
                .intr_bit = BUCK_INT_M##_nr##_POKn_INT,\
                .mask_bit = BUCK_INT_M##_nr##_POKn_INT,\
                .stat     = __BITDESC(BUCK_STAT , BUCK_STAT_M##_nr##_POKn),\
            }

    BUCKINT_DESC(1),
    BUCKINT_DESC(2),
    BUCKINT_DESC(3),
    BUCKINT_DESC(4),
};

static void max77812_irq_mask (struct irq_data *data)
{
    struct max77812 *max77812 = irq_data_get_irq_chip_data(data);
    struct device *dev = max77812->dev;
    struct max77812_int_desc *int_desc;
    u8 *intbank_unmask;
    int sub_irq;

    dev_dbg(dev, "irq_mask(%u)\n", data->irq);

    sub_irq = data->irq - (unsigned int)max77812->irq_base;
    if (unlikely(sub_irq < 0 || sub_irq >= BUCK_NUM_OF_INTS)) {
        dev_err(dev, "invalid irq nr - %u\n", data->irq);
        goto out;
    }

    int_desc       = &max77812_int_descs[sub_irq];
    intbank_unmask = &max77812->intbank_unmask[int_desc->bank];

    if (unlikely((*intbank_unmask & int_desc->mask_bit) == 0)) {
        /* already masked */
        dev_dbg(dev, "irq %s(%u) already disabled\n",
            int_desc->name, data->irq);
        goto out;
    }

    *intbank_unmask &= ~int_desc->mask_bit;

    max77812->intbank_to_sync |= (1 << int_desc->bank);

    dev_dbg(dev, "irq %s(%u) will be disabled\n", int_desc->name, data->irq);

out:
    return;
}

static void max77812_irq_unmask (struct irq_data *data)
{
    struct max77812 *max77812 = irq_data_get_irq_chip_data(data);
    struct device *dev = max77812->dev;
    struct max77812_int_desc *int_desc;
    u8 *intbank_unmask;
    int sub_irq;

    dev_dbg(dev, "irq_unmask(%u)\n", data->irq);

    sub_irq = data->irq - (unsigned int)max77812->irq_base;
    if (unlikely(sub_irq < 0 || sub_irq >= BUCK_NUM_OF_INTS)) {
        dev_err(dev, "invalid irq nr - %u\n", data->irq);
        goto out;
    }

    int_desc       = &max77812_int_descs[sub_irq];
    intbank_unmask = &max77812->intbank_unmask[int_desc->bank];

    if (unlikely((*intbank_unmask & int_desc->mask_bit) != 0)) {
        /* already masked */
        dev_dbg(dev, "irq %s(%u) already enabled\n",
            int_desc->name, data->irq);
        goto out;
    }

    *intbank_unmask |= int_desc->mask_bit;

    max77812->intbank_to_sync |= (1 << int_desc->bank);

    dev_dbg(dev, "irq %s(%u) will be enabled\n", int_desc->name, data->irq);

out:
    return;
}

static void max77812_irq_bus_lock (struct irq_data *data)
{
    struct max77812 *max77812 = irq_data_get_irq_chip_data(data);
    struct device *dev = max77812->dev;

    __lock(max77812);
    dev_vdbg(dev, "irq_bus_lock()\n");
}

/*
 * genirq core code can issue chip->mask/unmask from atomic context.
 * This doesn't work for slow busses where an access needs to sleep.
 * bus_sync_unlock() is therefore called outside the atomic context,
 * syncs the current irq mask state with the slow external controller
 * and unlocks the bus.
 */

static void max77812_irq_bus_sync_unlock (struct irq_data *data)
{
    struct max77812 *max77812 = irq_data_get_irq_chip_data(data);
    struct device *dev = max77812->dev;
    int i, rc;

    for (i = 0; i < BUCK_NUM_OF_INTBANKS; i++) {
        struct max77812_intbank_desc *intbank_desc = &max77812_intbank_descs[i];
        u8 src_unmask = max77812->intbank_unmask[i];
        u8 src_mask = (u8)(~src_unmask);

        dev_vdbg(dev, "irq %s source mask to sync: 0x%02X\n",
            intbank_desc->name, src_mask);

        if (unlikely(!(max77812->intbank_to_sync & (1 << i)))) {
            continue;
        }

        rc = max77812_write(max77812, intbank_desc->src_mask_reg, src_mask);
        if (unlikely(rc)) {
            dev_err(dev, "failed to write irq %s mask register 0x%02X [%d]\n",
                intbank_desc->name, intbank_desc->src_mask_reg, rc);
        }

        if (src_unmask) {
            dev_dbg(dev, "unmasking irq %s\n", intbank_desc->name);
            rc = max77812_write_bitdesc(max77812, &intbank_desc->mask, 0);
        } else {
            dev_dbg(dev, "masking irq %s\n", intbank_desc->name);
            rc = max77812_write_bitdesc(max77812, &intbank_desc->mask, 1);
        }

        if (unlikely(rc)) {
            dev_err(dev, "failed to write INT_SRC_M for irq %s [%d]\n",
                intbank_desc->name, rc);
        }

        max77812->intbank_to_sync &= ~(1 << i);
    }

    dev_vdbg(dev, "irq_bus_sync_unlock()\n");
    __unlock(max77812);
}

static int max77812_irq_set_type (struct irq_data *data, unsigned int type)
{
    struct max77812 *max77812 = irq_data_get_irq_chip_data(data);
    struct device *dev = max77812->dev;

    dev_dbg(dev, "irq_set_type(0x%08X)\n", type);

    return 0;
}

/* IRQ chip operations
 */
static struct irq_chip max77812_irq_chip = {
    .name                = DRIVER_NAME,
    .irq_mask            = max77812_irq_mask,
    .irq_unmask          = max77812_irq_unmask,
    .irq_bus_lock        = max77812_irq_bus_lock,
    .irq_bus_sync_unlock = max77812_irq_bus_sync_unlock,
    .irq_set_type        = max77812_irq_set_type,
//  .irq_set_wake        = max77812_irq_set_wake,
};

static irqreturn_t max77812_irq_thread (int irq, void *data)
{
    struct max77812 *max77812 = data;
    struct device *dev = max77812->dev;
    u8 intbank_src[BUCK_NUM_OF_INTBANKS];
    int i, rc;

    dev_dbg(dev, "%s(%d, %p) entered\n", __func__, irq, data);

    for (i = 0; i < BUCK_NUM_OF_INTBANKS; i++) {
        struct max77812_intbank_desc *intbank_desc = &max77812_intbank_descs[i];

        rc = max77812_read(max77812,
            intbank_desc->src_intr_reg, &intbank_src[i]);
        if (likely(!rc)) {
            dev_dbg(dev, "irq %s source: 0x%02X\n", intbank_desc->name,
                intbank_src[i]);
        } else {
            dev_err(dev, "failed to read irq %s source register 0x%02X [%d]\n",
                intbank_desc->name, intbank_src[i], rc);
            intbank_src[0] = 0;
        }
    }

    for (i = 0; i < BUCK_NUM_OF_INTS; i++) {
        struct max77812_int_desc *int_desc = &max77812_int_descs[i];
        unsigned int sub_irq = (unsigned int)(max77812->irq_base + i);

        if (likely(intbank_src[int_desc->bank] & int_desc->intr_bit)) {
            dev_dbg(dev, "handle interrupt %s(%u)\n", int_desc->name, sub_irq);
            handle_nested_irq(sub_irq);
        }
    }

    rc = IRQ_HANDLED;

    dev_dbg(dev, "%s() exiting %d\n", __func__, rc);
    return (irqreturn_t)rc;
}

static struct max77812_gpix_desc max77812_gpix_descs[] = {
    #define GPIx_BITDESC(_nr) \
            [_nr] = {\
                .name  = GPIO_NAME_GPIx #_nr,\
                .func  = __BITDESC(GPI_FUNC   , GPI_FUNC_GPI##_nr##_FUNC ),\
                .deb   = __BITDESC(GPI_DEB2   , GPI_DEB2_GPI##_nr##_DEB  ),\
                .pd_en = __BITDESC(GPI_PD_CTRL, GPI_PD_CTRL_GPI##_nr##_PD),\
            }
    GPIx_BITDESC(0),
    GPIx_BITDESC(1),
};

static __always_inline struct max77812_gpix *max77812_lookup_gpi (
    struct max77812 *max77812, u16 func_mask)
{
    struct device *dev = max77812->dev;
    int i, rc;

    for (i = 0; i < NUM_OF_GPIS; i++) {
        struct max77812_gpix *gpix = &max77812->gpi[i];
        u8 func;

        rc = max77812_read_bitdesc(max77812, &gpix->desc->func, &func);
        if (unlikely(rc)) {
            dev_err(dev, "failed to read GPI%d_FUNC [%d]\n", i, rc);
            continue;
        }

        if (unlikely((func_mask & ((u16)1 << func)) == 0)) {
            continue;
        }

        dev_vdbg(dev, "gpi %d func 0x%04X maskable\n", i, func_mask);

        return gpix;
    }

    dev_vdbg(dev, "gpi func 0x%04X maskable not found\n", func_mask);

    return NULL;
}

// Return value: a value of a gpi has func_mask.
//               Negative if any errors.
static __always_inline int max77812_get_gpi_value (
    struct max77812 *max77812, u16 func_mask)
{
    struct device *dev = max77812->dev;
    struct max77812_gpix *gpix;
    int rc;

    gpix = max77812_lookup_gpi(max77812, func_mask);
    if (unlikely(!gpix)) {
        rc = -EINVAL;
        goto out;
    }

    if (unlikely(gpix->gpio < 0)) {
        dev_vdbg(dev, "%s not assigned to any gpio\n", gpix->desc->name);
        rc = -EINVAL;
        goto out;
    }

    rc = gpio_get_value((unsigned)gpix->gpio);

    dev_dbg(dev, "get %d %s (gpio %d)\n", rc, gpix->desc->name, gpix->gpio);

out:
    return rc;
}

static __always_inline int max77812_set_gpi_value (
    struct max77812 *max77812, u16 func_mask, int value)
{
    struct device *dev = max77812->dev;
    struct max77812_gpix *gpix;
    int rc;

    gpix = max77812_lookup_gpi(max77812, func_mask);
    if (unlikely(!gpix)) {
        rc = -EINVAL;
        goto out;
    }

    if (unlikely(gpix->gpio < 0)) {
        dev_vdbg(dev, "%s not assigned to any gpio\n", gpix->desc->name);
        rc = -EINVAL;
        goto out;
    }

    if (unlikely(!gpix->gpio_owner)) {
        dev_vdbg(dev, "%s not have ownership of gpio %d\n",
            gpix->desc->name, gpix->gpio);
        rc = -EINVAL;
        goto out;
    }

    dev_dbg(dev, "set %d %s (gpio %d)\n", value, gpix->desc->name, gpix->gpio);

    gpio_set_value((unsigned)gpix->gpio, value);

    /* all done successfully */
    rc = 0;

out:
    return rc;
}

static __always_inline
int max77812_buck_list_voltage (struct max77812_buck *buck,
    struct max77812_buck_vid *vid, unsigned selector)
{
    struct device *dev = buck->dev;
    int rc;

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(dev, "voltage control not supported\n");
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

out:
    dev_vdbg(dev, "list voltage - %u => %duV\n", selector, rc);
    return rc;
}

static __always_inline
int max77812_buck_select_voltage (struct max77812_buck *buck,
    struct max77812_buck_vid *vid, int uV)
{
    struct device *dev = buck->dev;
    int rc;

    if (unlikely(vid->n_volts <= 0)) {
        dev_warn(dev, "voltage control not supported\n");
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
    }
    /* linear */
    else {
        rc = DIV_ROUND_UP(uV - vid->offset_uV, vid->step_uV);
        rc = min(vid->n_volts-1, max(0, rc));
    }

out:
    dev_vdbg(dev, "select voltage - %duV => %d\n", uV, rc);
    return rc;
}

// Return value: a time of delay in msec.
//               Negative if any errors.
static __always_inline
int max77812_buck_delay_count (struct max77812_buck *buck, u8 dly_cnt)
{
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    u8 dly_step;
    int rc;

    rc = max77812_read_bitdesc(max77812, DLY_STEP_BITDESC, &dly_step);
    if (unlikely(rc)) {
        dev_err(dev, "failed to read DLY_STEP [%d]\n", rc);
        goto out;
    }

    rc = (int)dly_cnt * __list_delay_step(dly_step);

out:
    return rc;
}

// Return value: a time for slew rate in usec.
//               Zero or positive values always.
static __always_inline int max77812_buck_slew_time (struct max77812_buck *buck,
    const struct max77812_bitdesc *bitdesc, int diff_uV)
{
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    u8 sr;
    int rc;

    rc = max77812_read_bitdesc(max77812, bitdesc, &sr);
    if (unlikely(rc)) {
        dev_err(dev, "failed to read SR [%d]\n", rc);
        sr = 7;
    }

    return DIV_ROUND_UP(abs(diff_uV), (int)__list_slew_rate(sr));
}

static __always_inline
int max77812_buck_enable (struct max77812_buck *buck, bool en)
{
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    int cnt = 0, rc;

    if (likely(max77812->glb_en_ctrl_gpio)) {
        if (likely(max77812->glb_en_gpio >= 0)) {
            dev_dbg(dev, "set %u %s (gpio %d)\n",
                en, GPIO_NAME_GLB_EN, max77812->glb_en_gpio);

            gpio_set_value((unsigned)max77812->glb_en_gpio, !!en);
            cnt++;
        }

        rc = max77812_set_gpi_value(max77812, 1 << GPI_FUNC_GLB_EN, !!en);
        if (likely(!rc)) {
            cnt++;
        }
    }

    if (likely(buck->en_ctrl_mask & BUCK_EN_CTRL_GPIO)) {
        rc = max77812_set_gpi_value(max77812, desc->gpi_func_mask_en, !!en);
        if (likely(!rc)) {
            cnt++;
        }
    }

    if (likely(buck->en_ctrl_mask & BUCK_EN_CTRL_BIT)) {
        rc = max77812_write_bitdesc(max77812, &desc->en, !!en);
        if (likely(!rc)) {
            cnt++;
        } else {
            dev_err(dev, "failed to write EN [%d]\n", rc);
        }
    }

    if (unlikely(!cnt)) {
        dev_err(dev, "no control done for %sabling\n", en ? "en" : "dis");
        rc = -EINVAL;
        goto out;
    }

    dev_dbg(dev, "%s initiated\n", en ? "start up" : "shut down");

    /* all done successfully */
    rc = 0;

out:
    return rc;
}

// Return value: a value indicating if the buck is in LPM.
//               Assuming false if any errors.
static __always_inline bool max77812_buck_is_lpm (struct max77812_buck *buck)
{
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    u8 lpm_en;
    int rc;

    if (unlikely(max77812->glb_lpm_gpio >= 0)) {
        rc = gpio_get_value((unsigned)max77812->glb_lpm_gpio);

        dev_dbg(dev, "get %u %s (gpio %d)\n",
            rc, GPIO_NAME_GLB_LPM, max77812->glb_lpm_gpio);

        if (unlikely(rc >= 0 && rc)) {
            rc = true;
            goto out;
        }
    }

    rc = max77812_get_gpi_value(max77812,
        desc->gpi_func_mask_lpm | (1 << GPI_FUNC_GLB_LPM));
    if (unlikely(rc >= 0 && rc)) {
        rc = true;
        goto out;
    }

    rc = max77812_read_bitdesc(max77812, &desc->lpm_en, &lpm_en);
    if (unlikely(rc)) {
        dev_err(dev, "failed to read LPM_EN [%d]\n", rc);
        rc = false;
        goto out;
    }

    rc = !!lpm_en;

out:
    return (bool)rc;
}

static __always_inline int max77812_buck_set_mode (struct max77812_buck *buck,
    unsigned int mode)
{
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    struct max77812_buck_desc *desc = buck->desc;
    u16 fpwm_en, lpm_en;
    int lpm_cnt = 0, rc;

    switch (mode) {
    case REGULATOR_MODE_FAST:
        fpwm_en = 1;
        lpm_en  = 0;
        break;

    case REGULATOR_MODE_NORMAL:
        fpwm_en = 0;
        lpm_en  = 0;
        break;

    case REGULATOR_MODE_IDLE:
        fpwm_en = 0;
        lpm_en  = 1;
        break;

    case REGULATOR_MODE_STANDBY:
        fpwm_en = 0;
        lpm_en  = 1;
        break;

    default:
        rc = -EINVAL;
        goto out;
    }

    rc = max77812_write_bitdesc(max77812, &desc->fpwm_en, fpwm_en);
    if (unlikely(rc)) {
        dev_err(dev, "failed to write FPWM_EN [%d]\n", rc);
        goto out;
    }

    if (likely(max77812->glb_lpm_ctrl_gpio)) {
        if (likely(max77812->glb_lpm_gpio >= 0)) {
            dev_dbg(dev, "set %u %s (gpio %d)\n",
                lpm_en, GPIO_NAME_GLB_LPM, max77812->glb_lpm_gpio);

            gpio_set_value((unsigned)max77812->glb_lpm_gpio, lpm_en);
            lpm_cnt++;
        }

        rc = max77812_set_gpi_value(max77812, 1 << GPI_FUNC_GLB_LPM, lpm_en);
        if (likely(!rc)) {
            lpm_cnt++;
        }
    }

    if (likely(buck->lpm_ctrl_mask & BUCK_LPM_CTRL_GPIO)) {
        rc = max77812_set_gpi_value(max77812, desc->gpi_func_mask_lpm, lpm_en);
        if (likely(!rc)) {
            lpm_cnt++;
        }
    }

    if (likely(buck->lpm_ctrl_mask & BUCK_LPM_CTRL_BIT)) {
        rc = max77812_write_bitdesc(max77812, &desc->lpm_en, lpm_en);
        if (likely(!rc)) {
            lpm_cnt++;
        } else {
            dev_err(dev, "failed to write LPM_EN[%d]\n", rc);
        }
    }

    if (unlikely(!lpm_cnt)) {
        dev_err(dev, "no control done for LPM\n");
        rc = -EINVAL;
        goto out;
    }

    dev_dbg(dev, "new mode %u (fpwm %s / lpm %s)\n", mode,
        fpwm_en ? "enabled" : "disabled",
        lpm_en  ? "enabled" : "disabled");

    /* all done successfully */
    rc = 0;

out:
    return rc;
}

// Return value: a list of REGULATOR_MODE of the buck.
//               Negative if any errors.
static __always_inline int max77812_buck_get_mode (struct max77812_buck *buck)
{
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    u8 fpwm_en;
    int rc;

    if (unlikely(max77812_buck_is_lpm(buck))) {
        rc = REGULATOR_MODE_STANDBY;
        if (!buck->no_idle_mode) {
            rc |= REGULATOR_MODE_IDLE;
        }
        goto out;
    }

    rc = max77812_read_bitdesc(max77812, &desc->fpwm_en, &fpwm_en);
    if (unlikely(rc)) {
        dev_err(dev, "failed to read FPWM_EN [%d]\n", rc);
        goto out;
    }

    rc = fpwm_en ? REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL;

out:
    return rc;
}

static __always_inline
const struct max77812_bitdesc *max77812_buck_get_current_vout (
    struct max77812_buck *buck)
{
    struct device *dev = buck->dev;
    struct max77812_buck_desc *desc = buck->desc;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    struct max77812_bitdesc *vout;
    int rc;

    rc = max77812_get_gpi_value(max77812,
        desc->gpi_func_mask_vsel | (1 << GPI_FUNC_GLB_VSEL));

    if (rc < 0) {
        dev_vdbg(dev, "active vout VOUT\n");
        vout = &desc->vout;
    } else if (rc != 0) {
        dev_vdbg(dev, "active vout VOUT_D\n");
        vout = &desc->vout_d;
    } else {
        dev_vdbg(dev, "active vout VOUT_S\n");
        vout = &desc->vout_s;
    }

    return vout;
}

// Return value: the real output voltage (uV).
//               Zero if any errors.
static __always_inline
int max77812_buck_get_current_voltage (struct max77812_buck *buck)
{
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    const struct max77812_bitdesc *vout_bitdesc;
    u8 vout;
    int rc;

    vout_bitdesc = max77812_buck_get_current_vout(buck);

    rc = max77812_read_bitdesc(max77812, vout_bitdesc, &vout);
    if (unlikely(rc)) {
        dev_err(dev, "failed to read VOUT_x [%d]\n", rc);
        rc = 0;
        goto out;
    }

    rc = max77812_buck_list_voltage(buck, &desc->vid, vout);

out:
    return rc;
}

static int max77812_buck_op_list_voltage (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    int rc;

    __lock(max77812);

    rc  = max77812_buck_list_voltage(buck, &desc->vid, selector);

    /* constraint offset */
    rc += rdev->constraints ? rdev->constraints->uV_offset : 0;

    __unlock(max77812);
    return rc;
}

static int max77812_buck_op_set_voltage_sel (struct regulator_dev *rdev,
    unsigned selector)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    int rc;

    __lock(max77812);

    rc = max77812_write_bitdesc(max77812, &desc->vout, (u8)selector);
    if (unlikely(rc)) {
        dev_err(dev, "failed to write VOUT [%d]\n", rc);
        goto out;
    }

#if 1
    rc = max77812_write_bitdesc(max77812, &desc->vout_d, (u8)selector);
    if (unlikely(rc)) {
        dev_err(dev, "failed to write VOUT_D [%d]\n", rc);
        goto out;
    }
#endif

    dev_dbg(dev, "new VOUT %duV (selected 0x%02X)\n",
        max77812_buck_list_voltage(buck, &desc->vid, selector), (u8)selector);

out:
    __unlock(max77812);
    return rc;
}

static int max77812_buck_op_get_voltage_sel (struct regulator_dev *rdev)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    u8 vout;
    int rc;

    __lock(max77812);

    if (max77812_buck_get_current_vout(buck) == &desc->vout_d) {
        rc = max77812_read_bitdesc(max77812, &desc->vout_d, &vout);
        if (unlikely(rc)) {
            dev_err(dev, "failed to read VOUT_D [%d]\n", rc);
            goto out;
        }
    } else {
        rc = max77812_read_bitdesc(max77812, &desc->vout, &vout);
        if (unlikely(rc)) {
            dev_err(dev, "failed to read VOUT [%d]\n", rc);
            goto out;
        }
    }

    rc = (int)vout;

    dev_dbg(dev, "current VOUT(_D) %duV (selected 0x%02X)\n",
        max77812_buck_list_voltage(buck, &desc->vid, vout), vout);

out:
    __unlock(max77812);
    return rc;
}

static int max77812_buck_op_enable (struct regulator_dev *rdev)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    int rc;

    __lock(max77812);

    rc = max77812_buck_enable(buck, 1);

    __unlock(max77812);
    return rc;
}

static int max77812_buck_op_disable (struct regulator_dev *rdev)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    int rc;

    __lock(max77812);

    rc = max77812_buck_enable(buck, 0);
    if (unlikely(rc)) {
        goto out;
    }

#if 1
    do {
        u8 shdn_dly_cnt;
        int delay, shdn_dly, slew_time, uV;

        rc = max77812_read_bitdesc(max77812, &desc->shdn_dly, &shdn_dly_cnt);
        if (unlikely(rc)) {
            dev_err(dev, "failed to read SHDN_DLY [%d]\n", rc);
            shdn_dly_cnt = 31;
        }

        shdn_dly = max77812_buck_delay_count(buck, shdn_dly_cnt);
        shdn_dly = shdn_dly >= 0 ? shdn_dly : 31 * 2;
        dev_dbg(dev, "shut down delay %dms\n", shdn_dly);

        uV        = max77812_buck_get_current_voltage(buck);
        slew_time = max77812_buck_slew_time(buck, SD_SR_BITDESC, uV);
        dev_dbg(dev, "shut down slew time %dus for %duV\n", slew_time, uV);

        delay = shdn_dly * 1000 + slew_time;

        if (likely(delay > 1000)) {
        	mdelay(delay / 1000);
        	delay = delay % 1000;
        }

        udelay(delay);
    } while (0);
#endif

    /* all done successfully */
    rc = 0;

out:
    __unlock(max77812);
    return rc;
}

static int max77812_buck_op_is_enabled (struct regulator_dev *rdev)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    u8 en;
    int rc;

    __lock(max77812);

    if (unlikely(max77812->glb_en_gpio >= 0)) {
        rc = gpio_get_value((unsigned)max77812->glb_en_gpio);

        dev_dbg(dev, "get %u %s (gpio %d)\n",
            rc, GPIO_NAME_GLB_EN, max77812->glb_en_gpio);

        if (unlikely(rc >= 0 && rc)) {
            rc = true;
            goto out;
        }
    }

    rc = max77812_get_gpi_value(max77812,
        desc->gpi_func_mask_en | (1 << GPI_FUNC_GLB_EN));
    if (unlikely(rc >= 0 && rc)) {
        rc = true;
        goto out;
    }

    rc = max77812_read_bitdesc(max77812, &desc->en, &en);
    if (unlikely(rc)) {
        dev_err(dev, "failed to read EN [%d]\n", rc);
        rc = false;
        goto out;
    }

    rc = !!en;

out:
    dev_dbg(dev, "current state %s\n", rc ? "enabled" : "disabled");

    __unlock(max77812);
    return rc;
}

static int max77812_buck_op_set_mode (struct regulator_dev *rdev,
    unsigned int mode)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    int rc;

    __lock(max77812);

    rc = max77812_buck_set_mode(buck, mode);
    if (unlikely(rc)) {
        goto out;
    }

    buck->mode = mode;

out:
    __unlock(max77812);
    return rc;
}

static unsigned int max77812_buck_op_get_mode (struct regulator_dev *rdev)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    int rc;

    __lock(max77812);

    rc = max77812_buck_get_mode(buck) & (int)buck->mode;

    dev_dbg(dev, "current mode %d\n", rc);

    __unlock(max77812);
    return rc;
}

static int max77812_buck_op_enable_time (struct regulator_dev *rdev)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    u8 stup_dly_cnt;
    int stup_dly, uV, slew_time, rc;

    __lock(max77812);

    if (unlikely(buck->id == BUCK1)) {
        stup_dly_cnt = 0;
    } else {
        rc = max77812_read_bitdesc(max77812, &desc->stup_dly, &stup_dly_cnt);
        if (unlikely(rc)) {
            dev_err(dev, "failed to read STUP_DLY [%d]\n", rc);
            stup_dly_cnt = 31;
        }
    }

    stup_dly = max77812_buck_delay_count(buck, stup_dly_cnt);
    stup_dly = stup_dly >= 0 ? stup_dly : 31 * 2;
    dev_dbg(dev, "start up delay %dms\n", stup_dly);

    uV        = max77812_buck_get_current_voltage(buck);
    slew_time = max77812_buck_slew_time(buck, SS_SR_BITDESC, uV);
    dev_dbg(dev, "start up slew time %dus for %duV\n", slew_time, uV);

    rc = stup_dly * 1000 + slew_time;

    dev_dbg(dev, "enable time %dus\n", rc);

    __unlock(max77812);
    return rc;
}

static int max77812_buck_op_set_voltage_time_sel (struct regulator_dev *rdev,
    unsigned int old_selector, unsigned int new_selector)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    int old_uV, new_uV, rc = 0; /* no delay */

    __lock(max77812);

    old_uV = max77812_buck_list_voltage(buck, &desc->vid, old_selector);
    new_uV = max77812_buck_list_voltage(buck, &desc->vid, new_selector);

    if (unlikely(old_uV == new_uV)) {
        goto out;
    }

    if (old_uV < new_uV) {
        /* ramp-up */
        rc = max77812_buck_slew_time(buck, RU_SR_BITDESC, new_uV - old_uV);
    } else /*if (old_uV > new_uV)*/ {
        /* ramp-down */
        rc = max77812_buck_slew_time(buck, RD_SR_BITDESC, old_uV - new_uV);
    }

out:
    dev_dbg(dev, "ramp time %dus for %duV -> %uV\n", rc, old_uV, new_uV);
    __unlock(max77812);
    return rc;
}

static int max77812_buck_op_set_suspend_voltage (struct regulator_dev *rdev,
    int uV)
{
    struct max77812_buck *buck = rdev_get_drvdata(rdev);
    struct max77812_buck_desc *desc = buck->desc;
    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    int selector, rc;

    __lock(max77812);

    selector = max77812_buck_select_voltage(buck, &desc->vid, uV);
    if (unlikely(selector < 0)) {
        dev_err(dev, "invalid suspend voltage - %duV\n", uV);
        rc = -EINVAL;
        goto out;
    }

    rc = max77812_write_bitdesc(max77812, &desc->vout_s, (u8)selector);
    if (unlikely(rc)) {
        dev_err(dev, "failed to write VOUT_S [%d]\n", rc);
        goto out;
    }

    dev_dbg(dev, "new VOUT_S %duV (selected 0x%02X)\n", uV, (u8)selector);

out:
    __unlock(max77812);
    return rc;
}

static struct regulator_ops max77812_buck_ops = {
    /* enumerate supported voltages */
    .list_voltage         = max77812_buck_op_list_voltage,

    /* get/set regulator voltage */
    .set_voltage_sel      = max77812_buck_op_set_voltage_sel,
    .get_voltage_sel      = max77812_buck_op_get_voltage_sel,

    /* enable/disable regulator */
    .enable               = max77812_buck_op_enable,
    .disable              = max77812_buck_op_disable,
    .is_enabled           = max77812_buck_op_is_enabled,

    /* get/set regulator operating mode (defined in consumer.h) */
    .set_mode             = max77812_buck_op_set_mode,
    .get_mode             = max77812_buck_op_get_mode,

    /* Time taken to enable or set voltage on the regulator */
    .enable_time          = max77812_buck_op_enable_time,
    .set_voltage_time_sel = max77812_buck_op_set_voltage_time_sel,

    /* the operations below are for configuration of regulator state when
     * its parent PMIC enters a global STANDBY/HIBERNATE state */
    .set_suspend_voltage = max77812_buck_op_set_suspend_voltage,
//  .set_suspend_enable  = max77812_buck_op_set_suspend_enable,
//  .set_suspend_disable = max77812_buck_op_set_suspend_disable,
//  .set_suspend_mode    = max77812_buck_op_set_suspend_mode,
};

#define __BUCK_DESC_RDESC(_nr, _n_volts) \
        {\
            .name       = BUCK_VREG_NAME #_nr,\
            .id         = BUCK##_nr,\
            .n_voltages = (unsigned)(_n_volts),\
            .ops        = &max77812_buck_ops,\
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
#define __BUCK_DESC_GPI_FUNC(_nr) \
        .gpi_func_mask_en   = 1 << GPI_FUNC_M##_nr##_EN,\
        .gpi_func_mask_lpm  = 1 << GPI_FUNC_M##_nr##_LPM,\
        .gpi_func_mask_vsel = 1 << GPI_FUNC_M##_nr##_VSEL

#define __BUCK_DESC_BITDESC(_nr) \
        .en       = __BITDESC(EN_CTRL        , EN_CTRL_EN_M##_nr      ),\
        .lpm_en   = __BITDESC(EN_CTRL        , EN_CTRL_EN_M##_nr##_LPM),\
        .vout     = __BITDESC(M##_nr##_VOUT  , 0xFF                   ),\
        .vout_d   = __BITDESC(M##_nr##_VOUT_D, 0xFF                   ),\
        .vout_s   = __BITDESC(M##_nr##_VOUT_S, 0xFF                   ),\
        .ad_en    = __BITDESC(M##_nr##_CFG   , MX_CFG_AD              ),\
        .ilim     = __BITDESC(M##_nr##_CFG   , MX_CFG_ILIM            ),\
        .fpwm_en  = __BITDESC(M##_nr##_CFG   , MX_CFG_FPWM            ),\
        .fsr_en   = __BITDESC(M##_nr##_CFG   , MX_CFG_FSREN           ),\
        .stup_dly = __BITDESC(STUP_SEQ##_nr  , STUP_SEQX_STUP_DLY     ),\
        .shdn_dly = __BITDESC(SHDN_SEQ##_nr  , SHDN_SEQX_SHDN_DLY     )

#define MIN_UV    250000 /* 0.250V */
#define MAX_UV   1510000 /* 1.151V */
#define STEP_UV     5000 /* 0.005V */
#define N_VOLTS  ((((MAX_UV) - (MIN_UV)) / (STEP_UV)) + 1)

static struct max77812_buck_desc max77812_buck_descs[] = {
    #define BUCK_DESC(_nr) \
            [BUCK##_nr] = {\
                .rdesc = __BUCK_DESC_RDESC(_nr, N_VOLTS),\
                .vid   = __BUCK_DESC_VID_LINEAR(N_VOLTS, MIN_UV, STEP_UV),\
                __BUCK_DESC_GPI_FUNC(_nr),\
                __BUCK_DESC_BITDESC(_nr),\
            }
    BUCK_DESC(1),
    BUCK_DESC(2),
    BUCK_DESC(3),
    BUCK_DESC(4),
};

static __always_inline
struct regulator_dev *max77812_buck_alloc_rdev (struct max77812_buck *buck)
{
    struct device *dev = buck->dev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    struct regulator_config config;

    memset(&config, 0x00, sizeof(config));
    config.dev         = dev;
    config.init_data   = buck->rinitdata;
    config.driver_data = buck;
    config.of_node     = dev->of_node;

    return regulator_register(&buck->desc->rdesc, &config);
#else /* LINUX_VERSION_CODE ... */
    return regulator_register(&buck->desc->rdesc, dev, buck->rinitdata, buck,
        dev->of_node);
#endif /* LINUX_VERSION_CODE ... */
}

static __always_inline int max77812_buck_config (struct max77812_buck *buck)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property: " _prop, ##__VA_ARGS__)

    struct device *dev = buck->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    struct max77812_buck_desc *desc = buck->desc;
    u8 tmp;
    int rc;

    buck->no_idle_mode = of_find_property(dev->of_node, "no_idle_mode", NULL);
    __prop_printk(dev, "idle_mode", "%s",
        buck->no_idle_mode ? "disabled" : "enabled");

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
    buck->rinitdata = of_get_regulator_init_data(dev, dev->of_node);
#else /* VERSION < 3.19.0 */
    buck->rinitdata = of_get_regulator_init_data(dev, dev->of_node, NULL);
#endif /* VERSION ... */

    buck->rinitdata->constraints.valid_modes_mask =
        REGULATOR_MODE_FAST   |
        REGULATOR_MODE_NORMAL |
//      REGULATOR_MODE_IDLE   |
        REGULATOR_MODE_STANDBY;
    buck->rinitdata->constraints.valid_ops_mask   =
        REGULATOR_CHANGE_VOLTAGE |
        REGULATOR_CHANGE_STATUS  |
        REGULATOR_CHANGE_MODE    |
        REGULATOR_CHANGE_DRMS;

    if (!buck->no_idle_mode) {
        buck->rinitdata->constraints.valid_modes_mask |= REGULATOR_MODE_IDLE;
    }

    if (of_property_u8(dev->of_node, "en_ctrl_mask", &buck->en_ctrl_mask)) {
        buck->en_ctrl_mask = BUCK_EN_CTRL_BIT;
    }

    __prop_printk(dev, "en_ctrl_mask", "0x%02X", buck->en_ctrl_mask);

    if (of_property_u8(dev->of_node, "lpm_ctrl_mask", &buck->lpm_ctrl_mask)) {
        buck->lpm_ctrl_mask = BUCK_LPM_CTRL_BIT;
    }

    __prop_printk(dev, "lpm_ctrl_mask", "0x%02X", buck->lpm_ctrl_mask);

    if (likely(buck->id != BUCK1)) {
        /* BUCK1 does not support STUP_DLY */

        rc = max77812_init_prop(max77812, dev->of_node,
            "stup_dly", &desc->stup_dly, &tmp);
        if (unlikely(rc)) {
            goto out;
        }

        __prop_printk(dev, "stup_dly", "%u x DLYSTEP", tmp);

        rc = max77812_init_prop(max77812, dev->of_node,
            "shdn_dly", &desc->shdn_dly, &tmp);
        if (unlikely(rc)) {
            goto out;
        }
    }

    __prop_printk(dev, "shdn_dly", "%u x DLYSTEP", tmp);

    rc = max77812_init_prop(max77812, dev->of_node,
        "ad_en", &desc->ad_en, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "ad_en", "%s", tmp ? "enabled" : "disabled");

    rc = max77812_init_prop(max77812, dev->of_node,
        "ilim", &desc->ilim, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "ilim", "%s",
        tmp == 0 ? "3.0A / 2.0A" : tmp == 1 ? "3.6A / 2.4A" :
        tmp == 2 ? "4.2A / 2.8A" : tmp == 3 ? "4.8A / 3.2A" :
        tmp == 4 ? "5.4A / 3.6A" : tmp == 5 ? "6.0A / 4.0A" :
        tmp == 6 ? "6.6A / 4.4A" : "7.2A / 4.8A");

    rc = max77812_init_prop(max77812, dev->of_node,
        "fpwm_en", &desc->fpwm_en, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "fpwm_en", "%s", tmp ? "enabled" : "disabled");

    rc = max77812_init_prop(max77812, dev->of_node,
        "fsr_en", &desc->fsr_en, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "fsr_en", "%s", tmp ? "enabled" : "disabled");

out:
    return rc;
}

static char *max77812_buck_of_device_names[] = {
    #define BUCK_OF_DEV_NAME(_nr) \
            [BUCK##_nr] = BUCK_VREG_NAME "." #_nr
    BUCK_OF_DEV_NAME(1),
    BUCK_OF_DEV_NAME(2),
    BUCK_OF_DEV_NAME(3),
    BUCK_OF_DEV_NAME(4),
};

static __always_inline void max77812_buck_destroy (struct max77812_buck *buck)
{
    struct device *dev = buck->dev;

    if (likely(buck->rdev)) {
        regulator_unregister(buck->rdev);
    }

    list_del(&buck->link);

    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, buck);
}

static int max77812_buck_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev->parent);
    struct max77812_buck *buck;
    int nr, rc;

    dev_vdbg(dev, "probing %s ...\n", dev_name(dev));

    for (nr = 0; nr < NUM_OF_BUCKS &&
        strcasecmp(dev_name(dev), max77812_buck_of_device_names[nr]);
        nr++);

    if (unlikely(nr >= NUM_OF_BUCKS)) {
        dev_err(dev, "invalid BUCK number - %s\n", dev_name(dev));
        return -ENODEV;
    }

    buck = devm_kzalloc(dev, sizeof(*buck), GFP_KERNEL);
    if (unlikely(!buck)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*buck));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, buck);

    INIT_LIST_HEAD(&buck->link);
    buck->dev  = dev;
    buck->id   = nr;
    buck->desc = &max77812_buck_descs[nr];
    buck->mode = REGULATOR_MODE_NORMAL;

    list_add_tail(&buck->link, &max77812->bucks);

    /* MAX77812 BUCKx Config */
    rc = max77812_buck_config(buck);
    if (unlikely(rc)) {
        dev_err(dev, "failed to initialize [%d]\n", rc);
        goto abort;
    }

    /* Register regulator device */
    buck->rdev = max77812_buck_alloc_rdev(buck);
    if (unlikely(IS_ERR(buck->rdev))) {
        rc = PTR_ERR(buck->rdev);
        dev_err(dev, "failed to register regulator [%d]\n", rc);
        buck->rdev = NULL;
        goto abort;
    }

    /* all done successfully */
    return 0;

abort:
    max77812_buck_destroy(buck);
    return rc;
}

static int max77812_buck_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77812_buck *buck = dev_get_drvdata(dev);

    max77812_buck_destroy(buck);

    return 0;
}

static int max77812_buck_pm_suspend (struct device *dev)
{
    return 0;
}

static int max77812_buck_pm_resume (struct device *dev)
{
    return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int max77812_buck_pm_idle (struct device *dev)
{
    return 0;
}
#else /* CONFIG_PM_RUNTIME */
#define max77812_buck_pm_idle NULL
#endif /* CONFIG_PM_RUNTIME */

static UNIVERSAL_DEV_PM_OPS(max77812_buck_pm,\
    max77812_buck_pm_suspend, max77812_buck_pm_resume, max77812_buck_pm_idle);

static struct of_device_id max77812_buck_of_device_ids[NUM_OF_BUCKS+1] = {
    #define BUCK_OF_DEV_ID(_nr) \
            [BUCK##_nr] = { .compatible = "maxim," BUCK_VREG_NAME #_nr }
    BUCK_OF_DEV_ID(1),
    BUCK_OF_DEV_ID(2),
    BUCK_OF_DEV_ID(3),
    BUCK_OF_DEV_ID(4),
    { }
};

static struct platform_driver max77812_buck_driver = {
    .probe                 = max77812_buck_probe,
    .remove                = max77812_buck_remove,
    .driver.name           = BUCK_VREG_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77812_buck_pm,
    .driver.of_match_table = of_match_ptr(max77812_buck_of_device_ids),
};

#define BUCK_RO_BIT_DEV_ATTR(_name, _bitdesc) \
static ssize_t max77812_attr_##_name##_show (struct device *dev,\
    struct device_attribute *attr, char *buf)\
{\
	struct max77812 *max77812 = dev_get_drvdata(dev);\
	u8 val;\
	int rc;\
	__lock(max77812);\
	rc = max77812_read_bitdesc(max77812, _bitdesc, &val);\
	if (rc) {\
        rc = (int)snprintf(buf, PAGE_SIZE, "I/O exception [%d]\n", rc);\
	} else {\
    	rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", val);\
    }\
    __unlock(max77812);\
	return rc;\
}\
static DEVICE_ATTR(_name, S_IRUGO, max77812_attr_##_name##_show, NULL)

#define BUCK_RW_BIT_DEV_ATTR(_name, _bitdesc) \
static ssize_t max77812_attr_##_name##_show (struct device *dev,\
    struct device_attribute *attr, char *buf)\
{\
	struct max77812 *max77812 = dev_get_drvdata(dev);\
	u8 val;\
	int rc;\
	__lock(max77812);\
	rc = max77812_read_bitdesc(max77812, _bitdesc, &val);\
	if (rc) {\
        rc = (int)snprintf(buf, PAGE_SIZE, "I/O exception [%d]\n", rc);\
	} else {\
    	rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", val);\
    }\
    __unlock(max77812);\
	return rc;\
}\
static ssize_t max77812_attr_##_name##_store (struct device *dev,\
    struct device_attribute *devattr, const char *buf, size_t count)\
{\
	struct max77812 *max77812 = dev_get_drvdata(dev);\
    int val;\
    __lock(max77812);\
    val = (int)simple_strtol(buf, NULL, 10);\
    max77812_write_bitdesc(max77812, _bitdesc, (u8)val);\
    __unlock(max77812);\
    return (ssize_t)count;\
}\
static DEVICE_ATTR(_name, S_IWUSR|S_IRUGO, max77812_attr_##_name##_show,\
    max77812_attr_##_name##_store)

BUCK_RO_BIT_DEV_ATTR(M1_POKn, BITDESC(BUCK_STAT, BUCK_STAT_M1_POKn));
BUCK_RO_BIT_DEV_ATTR(M2_POKn, BITDESC(BUCK_STAT, BUCK_STAT_M2_POKn));
BUCK_RO_BIT_DEV_ATTR(M3_POKn, BITDESC(BUCK_STAT, BUCK_STAT_M3_POKn));
BUCK_RO_BIT_DEV_ATTR(M4_POKn, BITDESC(BUCK_STAT, BUCK_STAT_M4_POKn));

BUCK_RO_BIT_DEV_ATTR(UVLO_Stat     , UVLO_STAT_BITDESC     );
BUCK_RO_BIT_DEV_ATTR(TSHDN_Stat    , TSHDN_STAT_BITDESC    );
BUCK_RO_BIT_DEV_ATTR(TJCT_140C_Stat, TJCT_140C_STAT_BITDESC);
BUCK_RO_BIT_DEV_ATTR(TJCT_120C_Stat, TJCT_120C_STAT_BITDESC);

BUCK_RW_BIT_DEV_ATTR(GPI0_Func, BITDESC(GPI_FUNC, GPI_FUNC_GPI0_FUNC));
BUCK_RW_BIT_DEV_ATTR(GPI1_Func, BITDESC(GPI_FUNC, GPI_FUNC_GPI1_FUNC));

static struct attribute *max77812_attrs[] = {
    #define BUCK_DEV_ATTR(_name) (&dev_attr_##_name.attr)

    BUCK_DEV_ATTR(M1_POKn),
    BUCK_DEV_ATTR(M2_POKn),
    BUCK_DEV_ATTR(M3_POKn),
    BUCK_DEV_ATTR(M4_POKn),

    BUCK_DEV_ATTR(UVLO_Stat     ),
    BUCK_DEV_ATTR(TSHDN_Stat    ),
    BUCK_DEV_ATTR(TJCT_140C_Stat),
    BUCK_DEV_ATTR(TJCT_120C_Stat),

    BUCK_DEV_ATTR(GPI0_Func),
    BUCK_DEV_ATTR(GPI1_Func),

    NULL
};

static const struct attribute_group max77812_attr_group = {
    .attrs = max77812_attrs,
};

static void max77812_unregister_vregs (struct max77812 *max77812)
{
    if (likely(max77812->buck_drv)) {
        platform_driver_unregister(max77812->buck_drv);
    }
}

static struct of_dev_auxdata max77812_of_dev_auxdatas[NUM_OF_BUCKS+1];

static int max77812_register_vregs (struct max77812 *max77812)
{
    const u8 buck_mask_per_phase[] = {
        [0] = (1 << BUCK1),
        [1] = (1 << BUCK1) | (1 << BUCK4),
        [2] = (1 << BUCK1) | (1 << BUCK3),
        [3] = (1 << BUCK1) | (1 << BUCK3) | (1 << BUCK4),
    };

    struct device *dev = max77812->dev;
    struct of_device_id *of_device_id;
    struct of_dev_auxdata *of_dev_auxdata;
    u8 buck_mask;
    int i, rc;

    buck_mask = max77812->phase >= 4 ? 0xFF :
        buck_mask_per_phase[max77812->phase];

    memset(max77812_buck_of_device_ids, 0x00,
        sizeof(max77812_buck_of_device_ids));
    of_device_id = &max77812_buck_of_device_ids[0];

    memset(max77812_of_dev_auxdatas, 0x00,
        sizeof(max77812_of_dev_auxdatas));
    of_dev_auxdata = &max77812_of_dev_auxdatas[0];

    for (i = 0; i < NUM_OF_BUCKS; i++) {
        if (unlikely(!(buck_mask & (1 << i)))) {
            continue;
        }

        snprintf(of_device_id->compatible, ARRAY_SIZE(of_device_id->compatible),
            "maxim," BUCK_VREG_NAME "%d", i + 1);

        of_dev_auxdata->compatible = of_device_id->compatible;
        of_device_id++;

        of_dev_auxdata->name = max77812_buck_of_device_names[i];
        of_dev_auxdata++;

        dev_vdbg(dev, "BUCK%d added\n", i + 1);
    }

    /* Register BUCK driver */
    max77812->buck_drv = &max77812_buck_driver;
    rc = platform_driver_register(&max77812_buck_driver);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register BUCK vregs driver [%d]\n", rc);
        max77812->buck_drv = NULL;
        goto abort;
    }

    /* Populate all children matching the table */
    rc = of_platform_populate(dev->of_node, NULL, max77812_of_dev_auxdatas, dev);
    if (unlikely(rc)) {
        dev_err(dev, "failed to populate BUCK vreg devices [%d]\n", rc);
        goto abort;
    }

    /* all done successfully */
    return 0;

abort:
    max77812_unregister_vregs(max77812);
    return rc;
}

static __always_inline void max77812_destroy (struct max77812 *max77812)
{
    struct device *dev = max77812->dev;
    int i;

    if (likely(max77812->attr_grp)) {
        sysfs_remove_group(&dev->kobj, max77812->attr_grp);
    }

    max77812_unregister_vregs(max77812);

    for (i = 0; i < NUM_OF_GPIS; i++) {
        if (likely(max77812->gpi[i].gpio_owner && max77812->gpi[i].gpio >= 0)) {
            devm_gpio_free(dev, (unsigned)max77812->gpi[i].gpio);
        }
    }

    if (likely(max77812->glb_lpm_ctrl_gpio && max77812->glb_lpm_gpio >= 0)) {
        devm_gpio_free(dev, (unsigned)max77812->glb_lpm_gpio);
    }

    if (likely(max77812->glb_en_ctrl_gpio && max77812->glb_en_gpio >= 0)) {
        devm_gpio_free(dev, (unsigned)max77812->glb_en_gpio);
    }

    if (likely(max77812->irq_base > 0)) {
        int i;

        for (i = 0; i < BUCK_NUM_OF_INTS; i++) {
            unsigned int sub_irq = (unsigned int)(max77812->irq_base + i);

            irq_set_handler(sub_irq, NULL);
            irq_set_chip_data(sub_irq, NULL);
        }

        irq_free_descs((unsigned int)max77812->irq_base, BUCK_NUM_OF_INTS);
    }

    if (likely(max77812->irq > 0)) {
        devm_free_irq(dev, (unsigned int)max77812->irq, max77812);
    }

    max77812_select_pinctrl(max77812, NULL);

    mutex_destroy(&max77812->lock);

    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, max77812);
}

static __always_inline int max77812_global_config (struct max77812 *max77812)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-29s" _fmt "\n", "property: " _prop, ##__VA_ARGS__)

    struct device *dev = max77812->dev;
    u8 tmp;
    int rc;

    /* phase config */

    __prop_printk(dev, "phase", "cfg2=%u cfg1=%u cfg0=%u",
        !!(max77812->phase & 0x4), !!(max77812->phase & 0x2),
        !!(max77812->phase & 0x1));

    /* EN/LPM gpio */

    max77812->glb_en_ctrl_gpio =
        of_find_property(dev->of_node, "glb_en_ctrl_gpio", NULL);
    __prop_printk(dev, "glb_en_ctrl_gpio", "%s",
        max77812->glb_en_ctrl_gpio ? "enabled" : "disabled");

    max77812->glb_en_gpio = of_get_named_gpio(dev->of_node, "glb_en_gpio", 0);
    __prop_printk(dev, "glb_en_gpio", "%d", max77812->glb_en_gpio);

    if (likely(max77812->glb_en_ctrl_gpio && max77812->glb_en_gpio >= 0)) {
        rc = devm_gpio_request(dev, (unsigned)max77812->glb_en_gpio,
            GPIO_NAME_GLB_EN);
        if (unlikely(rc)) {
            dev_err(dev, "failed to request gpio %d for %s [%d]\n",
                max77812->glb_en_gpio, GPIO_NAME_GLB_EN, rc);
            max77812->glb_en_gpio = rc;
        }
    }

    max77812->glb_lpm_ctrl_gpio =
        of_find_property(dev->of_node, "glb_lpm_ctrl_gpio", NULL);
    __prop_printk(dev, "glb_lpm_ctrl_gpio", "%s",
        max77812->glb_lpm_ctrl_gpio ? "enabled" : "disabled");

    max77812->glb_lpm_gpio = of_get_named_gpio(dev->of_node, "glb_lpm_gpio", 0);
    __prop_printk(dev, "glb_lpm_gpio", "%d", max77812->glb_lpm_gpio);

    if (likely(max77812->glb_lpm_ctrl_gpio && max77812->glb_lpm_gpio >= 0)) {
        rc = devm_gpio_request(dev, (unsigned)max77812->glb_lpm_gpio,
            GPIO_NAME_GLB_LPM);
        if (unlikely(rc)) {
            dev_err(dev, "failed to request gpio %d for %s [%d]\n",
                max77812->glb_lpm_gpio, GPIO_NAME_GLB_LPM, rc);
            max77812->glb_lpm_gpio = rc;
        }
    }

    /* STUP_SEQ1.DLY_STEP bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "dly_step", DLY_STEP_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "dly_step", "%ums", __list_delay_step(tmp));

    /* WDTRSTB_DEB.WDT_DEB bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "wdt_deb", WDT_DEB_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "wdt_deb", "%dns",
        tmp == 0 ?     0 : tmp == 1 ?  800 : tmp == 2 ?  1600 :
        tmp == 3 ?  3200 : tmp == 4 ? 6400 : tmp == 5 ? 12800 :
        tmp == 6 ? 25600 : 51200);

    /* PROT_CFG.TSHDN_EN bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "tshdn_en", TSHDN_EN_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "tshdn", "%s", tmp ? "enabled" : "disabled");

    /* PROT_CFG.UVLO_F bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "uvlo_f", UVLO_F_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "uvlo_f", "%s",
        tmp == 0 ? "1.95V" : tmp == 1 ? "2.05V" : tmp == 2 ? "2.15V" :
        tmp == 3 ? "2.25V" : tmp == 4 ? "2.35V" : tmp == 5 ? "2.45V" :
        tmp == 6 ? "2.55V" : "disabled");

    /* GLB_CFG1.B_SD_SR bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "sd_sr", SD_SR_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "sd_sr", "%uuV/us", __list_slew_rate(tmp));

    /* GLB_CFG1.B_SS_SR bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "ss_sr", SS_SR_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "ss_sr", "%uuV/us", __list_slew_rate(tmp));

    /* GLB_CFG2.B_RD_SR bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "rd_sr", RD_SR_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "rd_sr", "%uuV/us", __list_slew_rate(tmp));

    /* GLB_CFG2.B_RU_SR bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "ru_sr", RU_SR_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "ru_sr", "%uuV/us", __list_slew_rate(tmp));

    /* GLB_CFG3.B_ISHARE_EN */

    rc = max77812_init_prop(max77812, dev->of_node,
        "ishare_en", ISHARE_EN_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "ishare", "%s", tmp ? "enabled" : "disabled");

    /* GLB_CFG3.B_BDBK_EN */

    rc = max77812_init_prop(max77812, dev->of_node,
        "bdbk_en", BDBK_EN_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "bdbk", "%s", tmp ? "enabled" : "disabled");

    /* GLB_CFG3.B_PETR_EN */

    rc = max77812_init_prop(max77812, dev->of_node,
        "petr_en", PETR_EN_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "petr", "%s", tmp ? "enabled" : "disabled");

    /* GLB_CFG3.B_NETR_EN */

    rc = max77812_init_prop(max77812, dev->of_node,
        "netr_en", NETR_EN_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "netr", "%s", tmp ? "enabled" : "disabled");

out:
    return rc;
}

static __always_inline int max77812_gpix_config (struct max77812 *max77812,
    struct device_node *np, u8 nr)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property: " _prop, ##__VA_ARGS__)

    struct device *dev = max77812->dev;
    struct max77812_gpix *gpix = &max77812->gpi[nr];
    u8 tmp;
    int rc;

    gpix->desc = &max77812_gpix_descs[nr];

    gpix->gpio_owner = of_find_property(np, "gpio_owner", NULL);
    __prop_printk(dev, "gpi/gpio_owner", "[%u] %s",
        nr, gpix->gpio_owner ? "enabled" : "disabled");

    gpix->gpio = of_get_named_gpio(np, "gpio", 0);
    __prop_printk(dev, "gpi/sys_gpio", "[%u] %d", nr, gpix->gpio);

    if (likely(gpix->gpio_owner && gpix->gpio >= 0)) {
        rc = devm_gpio_request(dev, (unsigned)gpix->gpio, gpix->desc->name);
        if (unlikely(rc)) {
            dev_err(dev, "failed to request gpio %d for %s [%d]\n",
                gpix->gpio, gpix->desc->name, rc);
            gpix->gpio = rc;
        }
    }

    rc = max77812_init_prop(max77812, np, "func", &gpix->desc->func, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "gpi/func", "[%u] %u", nr, tmp);

    rc = max77812_init_prop(max77812, np, "deb", &gpix->desc->deb, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "gpi/deb", "[%u] %uus", nr, (u16)tmp * 64);

    rc = max77812_init_prop(max77812, np, "pd_en", &gpix->desc->pd_en, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "gpi/pd_en", "[%u] %s",
        nr, tmp ? "enabled" : "disabled");

out:
    return rc;
}

static __always_inline int max77812_gpi_config (struct max77812 *max77812)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property: " _prop, ##__VA_ARGS__)

    struct device *dev = max77812->dev;
    u8 tmp;
    int i, rc;

    /* GPI_DEB1.LPM_DEB bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "gpi_lpm_deb", GPI_LPM_DEB_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "gpi_lpm_deb", "%uus", (u16)tmp * 64);

    /* GPI_PD_CTRL.LPM_PD bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "gpi_lpm_pd_en", GPI_LPM_PD_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "gpi_lpm_pd", "%s", tmp ? "enabled" : "disabled");

    /* GPI_DEB1.EN_DEB bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "gpi_en_deb", GPI_EN_DEB_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "gpi_en_deb", "%uus", (u16)tmp * 64);

    /* GPI_PD_CTRL.EN_PD bit */

    rc = max77812_init_prop(max77812, dev->of_node,
        "gpi_en_pd_en", GPI_EN_PD_BITDESC, &tmp);
    if (unlikely(rc)) {
        goto out;
    }

    __prop_printk(dev, "gpi_en_pd", "%s", tmp ? "enabled" : "disabled");

    /* GPIx config */

    for (i = 0; i < NUM_OF_GPIS; i++) {
        struct device_node *gpix_np = NULL;
        struct device_node *child;

        for_each_child_of_node(dev->of_node, child) {
            u8 nr;

            if (unlikely(of_node_cmp(child->name, "gpi"))) {
                continue;
            }

            rc = of_property_u8(child, "reg", &nr);
            if (unlikely(rc)) {
                dev_warn(dev, "invalid GPIx config found\n");
                continue;
            }

            if (unlikely(nr == i)) {
                gpix_np = child;
                break;
            }
        }

        dev_dbg(dev, "gpi%d_config() ---\n", i);
        max77812_gpix_config(max77812, gpix_np, i);
    }

    /* all done successfully */
    rc = 0;

out:
    return rc;
}

static __always_inline int max77812_interrupt_config (struct max77812 *max77812,
    int irq)
{
    struct device *dev = max77812->dev;
    struct max77812_intbank_desc *intbank_desc;
    int i, rc;

    if (unlikely(irq <= 0)) {
        dev_dbg(dev, "irq disabled\n");
        rc = 0;
        goto out;
    }

    /* Disable all interrupts */
    for (i = 0; i < BUCK_NUM_OF_INTBANKS; i++) {
        intbank_desc = &max77812_intbank_descs[i];

        max77812->intbank_unmask[i] = 0;

        rc = max77812_write_bitdesc(max77812, &intbank_desc->mask, 1);
        if (unlikely(rc)) {
            dev_err(dev, "failed to reset irq %s mask [%d]\n",
                intbank_desc->name, rc);
            goto out;
        }
    }

    /* Clear all interrupt status  */
    for (i = 0; i < BUCK_NUM_OF_INTBANKS; i++) {
        u8 intbank_src;

        intbank_desc = &max77812_intbank_descs[i];

        rc = max77812_read(max77812, intbank_desc->src_intr_reg, &intbank_src);
        if (unlikely(rc)) {
            dev_err(dev, "failed to clear irq %s source register 0x%02X [%d]\n",
                intbank_desc->name, intbank_desc->src_intr_reg, rc);
            goto out;
        }

        dev_dbg(dev, "initial irq %s source: 0x%02X\n", intbank_desc->name,
            intbank_src);
    }

    /* Request HW IRQ */
    rc = devm_request_threaded_irq(dev, (unsigned int)irq, NULL,
        max77812_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
        DRIVER_NAME"-irq", max77812);
    if (unlikely(rc)) {
        dev_err(dev, "failed to request irq %d [%d]\n", irq, rc);
        goto out;
    }

    max77812->irq = irq;
    dev_dbg(dev, "irq %d requested\n", max77812->irq);

    /* Initialize irq_desc for sub-interrupts */

    rc = irq_alloc_descs(-1, 0, BUCK_NUM_OF_INTS, 0);
    if (unlikely(rc <= 0)) {
        dev_err(dev, "failed to alloc irq_desc [%d]\n", rc);
        goto out;
    }

    max77812->irq_base = rc;
    dev_dbg(dev, "irq_desc allocated: %d-%d\n", max77812->irq_base,
        max77812->irq_base + BUCK_NUM_OF_INTS - 1);

    for (i = 0; i < BUCK_NUM_OF_INTS; i++) {
        unsigned int sub_irq = (unsigned int)(max77812->irq_base + i);

        irq_set_chip_data(sub_irq, max77812);
        irq_set_chip_and_handler(sub_irq,
            &max77812_irq_chip, handle_simple_irq);
        irq_set_nested_thread(sub_irq, true);
        irq_set_noprobe(sub_irq);
    }

    /* all done successfully */
    rc = 0;

out:
    return rc;
}

static __devinit int max77812_do_probe (struct device *dev,
    struct max77812_io *io, u8 phase, int irq)
{
    struct max77812 *max77812;
    u8 r14;
    int rc;

    max77812 = devm_kzalloc(dev, sizeof(*max77812), GFP_KERNEL);
    if (unlikely(!max77812)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*max77812));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, max77812);

    mutex_init(&max77812->lock);
    max77812->dev = dev;
    max77812->io  = io;
    INIT_LIST_HEAD(&max77812->bucks);

    rc = max77812_read(max77812, 0x14, &r14);
    if (unlikely(rc)) {
        dev_err(dev, "device not found\n");
        goto abort;
    }

    dev_dbg(dev, "device info (R14) 0x%X-%u\n", (r14 >> 3) & 0xF, r14 & 0x7);

    /* MAX77812 Phase Config */
    max77812->phase = phase;

    /* MAX77812 PinCtrl */
    max77812_select_pinctrl(max77812, PINCTRL_STATE_DEFAULT);

    /* MAX77812 Global Config */
    dev_dbg(dev, "global_config() ---\n");
    rc = max77812_global_config(max77812);
    if (unlikely(rc)) {
        goto abort;
    }

    /* MAX77812 GPIx Config */
    dev_dbg(dev, "gpi_config() ---\n");
    rc = max77812_gpi_config(max77812);
    if (unlikely(rc)) {
        goto abort;
    }

    /* MAX77812 IRQ Config */
    dev_dbg(dev, "interrupt_config() ---\n");
    rc = max77812_interrupt_config(max77812, irq);
    if (unlikely(rc)) {
        goto abort;
    }

    /* Register platform drivers & devices for BUCKs */
    dev_dbg(dev, "register_vregs() ---\n");
    rc = max77812_register_vregs(max77812);
    if (unlikely(rc)) {
        goto abort;
    }

    /* Create max77812 sysfs attributes */
    dev_dbg(dev, "sysfs_create_group() ---\n");
    max77812->attr_grp = &max77812_attr_group;
    rc = sysfs_create_group(&dev->kobj, max77812->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        max77812->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77812_destroy(max77812);
    return rc;
}

static int max77812_pm_suspend (struct device *dev)
{
    struct max77812 *max77812 = dev_get_drvdata(dev);
    int rc;

    max77812_select_pinctrl(max77812, PINCTRL_STATE_SLEEP);

    rc = 0;

//out:
    dev_dbg(dev, "%s() returns %d\n", __func__, rc);
    return rc;
}

static int max77812_pm_resume (struct device *dev)
{
    struct max77812 *max77812 = dev_get_drvdata(dev);
    int rc;

    max77812_select_pinctrl(max77812, PINCTRL_STATE_DEFAULT);

    rc = 0;

//out:
    dev_dbg(dev, "%s() returns %d\n", __func__, rc);
    return rc;
}

#ifdef CONFIG_PM_RUNTIME
static int max77812_pm_idle (struct device *dev)
{
    struct max77812 *max77812 = dev_get_drvdata(dev);
    int rc;

    max77812_select_pinctrl(max77812, PINCTRL_STATE_IDLE);

    rc = 0;

//out:
    dev_dbg(dev, "%s() returns %d\n", __func__, rc);
    return rc;
}
#else /* CONFIG_PM_RUNTIME */
#define max77812_pm_idle NULL
#endif /* CONFIG_PM_RUNTIME */

static UNIVERSAL_DEV_PM_OPS(max77812_pm,\
    max77812_pm_suspend, max77812_pm_resume, max77812_pm_idle);

#ifdef CONFIG_OF
static struct of_device_id max77812_of_device_ids[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77812_of_device_ids);
#endif /* CONFIG_OF */

static struct regmap_config max77812_regmap_config = {
    .name              = DRIVER_NAME"-regmap",
//  .read_flag_mask    = ?,
//  .write_flag_mask   = ?,
//  .reg_bits          = ?,
//  .reg_format_endian = ?,
    .val_bits          = 8,
};

static struct max77812_io max77812_io;

#ifdef CONFIG_MAX77812_I2C
static int max77812_i2c_read (struct max77812 *max77812, u8 reg, u8 *val)
{
    struct regmap *regmap = max77812->io->regmap;
    unsigned int buf;
    int rc;

    rc = regmap_read(regmap, (unsigned int)reg, &buf);

    *val = (u8)buf;

    return rc;
}

static int max77812_i2c_write (struct max77812 *max77812, u8 reg, u8 val)
{
    struct regmap *regmap = max77812->io->regmap;

    return regmap_write(regmap, (unsigned int)reg, (unsigned int)val);
}

static int max77812_i2c_update_bits (struct max77812 *max77812,
    u8 reg, u8 mask, u8 val)
{
    struct regmap *regmap = max77812->io->regmap;

    return regmap_update_bits(regmap,
        (unsigned int)reg, (unsigned int)mask, (unsigned int)val);
}

static __devinit int max77812_i2c_probe (struct i2c_client *i2c,
    const struct i2c_device_id *id)
{
    struct device *dev = &i2c->dev;
    struct regmap *regmap;
    u8 phase;
    int rc;

    pr_info(DRIVER_DESC" (I2C) "DRIVER_VERSION"\n");

    max77812_regmap_config.reg_bits = 8;

    regmap = devm_regmap_init_i2c(i2c, &max77812_regmap_config);
    if (unlikely(IS_ERR(regmap))) {
        rc = PTR_ERR(regmap);
        dev_err(dev, "failed to init regmap_i2c [%d]\n", rc);
        goto out;
    }

    max77812_io.regmap      = regmap;
    max77812_io.read        = max77812_i2c_read;
    max77812_io.write       = max77812_i2c_write;
    max77812_io.update_bits = max77812_i2c_update_bits;

    switch (i2c->addr) {
    case 0x30:
    case 0x38:
        phase = 0;
        break;

    case 0x31:
    case 0x39:
        phase = 1;
        break;

    case 0x32:
    case 0x3A:
        phase = 2;
        break;

    case 0x33:
    case 0x3B:
        phase = 3;
        break;

    case 0x34:
    case 0x3C:
        phase = 4;
        break;

    default:
        dev_err(dev,
            "phase config not supported for addr 0x%02X of i2c client\n",
            (u8)i2c->addr);
        rc = -ENOTSUPP;
        goto out;
    }

    rc = max77812_do_probe(dev, &max77812_io, phase, i2c->irq);
    if (unlikely(rc)) {
        goto out;
    }

    max77812_bus_type = &i2c_bus_type;

out:
    return rc;
}

static __devexit int max77812_i2c_remove (struct i2c_client *i2c)
{
    struct device *dev = &i2c->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev);

    max77812_destroy(max77812);

    return 0;
}

static struct i2c_device_id max77812_i2c_device_ids[] = {
    { DRIVER_NAME, 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, max77812_i2c_device_ids);

static struct i2c_driver max77812_i2c_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77812_pm,
    .driver.of_match_table = of_match_ptr(max77812_of_device_ids),
    .probe                 = max77812_i2c_probe,
    .remove                = __devexit_p(max77812_i2c_remove),
    .id_table              = max77812_i2c_device_ids,
};
#endif /* CONFIG_MAX77812_I2C */

#ifdef CONFIG_MAX77812_SPI
static int max77812_spi_read (struct max77812 *max77812, u8 reg, u8 *val)
{
    // SPI Single Read Frame
    // 0 | 0 | RSVD[3:0] | ADDR[9:0] | PKT_LEN[7:0] | DATA[7:0] | ...

    struct regmap *regmap = max77812->io->regmap;
    unsigned int head = /*(0 << 23) |*/ (reg << 8) | 1, buf;
    int rc;

    rc = regmap_read(regmap, head, &buf);

    *val = (u8)buf;

    return rc;
}

static int max77812_spi_write (struct max77812 *max77812, u8 reg, u8 val)
{
    // SPI Single Write Frame
    // 1 | 0 | RSVD[3:0] | ADDR[9:0] | PKT_LEN[7:0] | DATA[7:0] | ...

    struct regmap *regmap = max77812->io->regmap;
    unsigned int head = /*(1 << 23) |*/ (reg << 8) | 1;

    return regmap_write(regmap, head, (unsigned int)val);
}

static int max77812_spi_update_bits (struct max77812 *max77812,
    u8 reg, u8 mask, u8 val)
{
    u8 orig;
    int rc;

    rc = max77812_spi_read(max77812, reg, &orig);
    if (likely(!rc)) {
        val = (orig & ~mask) | (val & mask);
        rc  = max77812_spi_write(max77812, reg, val);
    }

    return rc;
}

static __devinit int max77812_spi_probe (struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    struct regmap *regmap;
    u8 phase;
    int irq, rc;

    pr_info(DRIVER_DESC" (SPI) "DRIVER_VERSION"\n");

    max77812_regmap_config.read_flag_mask    = 0x00;
    max77812_regmap_config.write_flag_mask   = 0x80;
    max77812_regmap_config.reg_bits          = 24;
    max77812_regmap_config.reg_format_endian = REGMAP_ENDIAN_BIG;

    regmap = devm_regmap_init_spi(spi, &max77812_regmap_config);
    if (unlikely(IS_ERR(regmap))) {
        rc = PTR_ERR(regmap);
        dev_err(dev, "failed to init regmap_i2c [%d]\n", rc);
        goto out;
    }

    max77812_io.regmap      = regmap;
    max77812_io.read        = max77812_spi_read;
    max77812_io.write       = max77812_spi_write;
    max77812_io.update_bits = max77812_spi_update_bits;

    if (unlikely(of_property_u8(dev->of_node, "phase", &phase))) {
        phase = 4;
    }

    irq = irq_of_parse_and_map(dev->of_node, 0);

    rc = max77812_do_probe(dev, &max77812_io, phase, irq);
    if (unlikely(rc)) {
        goto out;
    }

    max77812_bus_type = &spi_bus_type;

out:
    return rc;
}

static __devexit int max77812_spi_remove (struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    struct max77812 *max77812 = dev_get_drvdata(dev);

    max77812_destroy(max77812);

    return 0;
}

static struct spi_device_id max77812_spi_device_ids[] = {
    { DRIVER_NAME, 0 },
    { },
};
MODULE_DEVICE_TABLE(spi, max77812_spi_device_ids);

static struct spi_driver max77812_spi_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77812_pm,
    .driver.of_match_table = of_match_ptr(max77812_of_device_ids),
    .probe                 = max77812_spi_probe,
    .remove                = __devexit_p(max77812_spi_remove),
    .id_table              = max77812_spi_device_ids,
};
#endif /* CONFIG_MAX77812_SPI */

static __init int max77812_driver_init (void)
{
    int rc;

#ifdef CONFIG_MAX77812_I2C
    rc = i2c_add_driver(&max77812_i2c_driver);
    if (unlikely(rc)) {
        pr_err(DRIVER_NAME": failed to add i2c driver [%d]\n", rc);
        goto out;
    }
    pr_debug(DRIVER_NAME"-i2c: registered\n");
#endif /* CONFIG_MAX77812_I2C */

#ifdef CONFIG_MAX77812_SPI
    rc = spi_register_driver(&max77812_spi_driver);
    if (unlikely(rc)) {
        pr_err(DRIVER_NAME": failed to add spi driver [%d]\n", rc);
        goto out;
    }
    pr_debug(DRIVER_NAME"-spi: registered\n");
#endif /* CONFIG_MAX77812_SPI */

    /* all done successfully */
    rc = 0;

out:
    return rc;
}
arch_initcall(max77812_driver_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

int max77812_get_interrupt_stat (u8 int_nr)
{
    struct max77812 *max77812 = __find_max77812();
    struct device *dev;
    struct max77812_int_desc *int_desc;
    u8 stat;
    int rc;

    if (unlikely(!max77812)) {
        pr_err(DRIVER_NAME": device not found\n");
        return -ENODEV;
    }

    __lock(max77812);

    dev = max77812->dev;

    if (unlikely(int_nr >= MAX77812_NUM_OF_INTS)) {
        dev_err(dev, "invalid interrupt number - %u\n", int_nr);
        rc = -EINVAL;
        goto out;
    }

    int_desc = &max77812_int_descs[int_nr];

    if (unlikely(int_desc->stat.mask == BIT_RSVD)) {
        dev_err(dev, "interrupt %s stat not available\n", int_desc->name);
        rc = -ENOTSUPP;
        goto out;
    }

    rc = max77812_read_bitdesc(max77812, &int_desc->stat, &stat);
    if (unlikely(rc)) {
        dev_err(dev, "failed to read interrupt %s stat [%d]\n",
            int_desc->name, rc);
        goto out;
    }

    rc = (int)stat;

    dev_dbg(dev, "interrupt %s stat: %s\n", int_desc->name,
        stat ? "asserted" : "not asserted");

out:
    __unlock(max77812);
    return rc;
}
EXPORT_SYMBOL(max77812_get_interrupt_stat);

