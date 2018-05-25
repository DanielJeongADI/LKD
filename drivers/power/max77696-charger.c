/*
 * MAX77696 Charger Driver
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

#define DRIVER_DESC    MAX77696_DESC" Charger Driver"
#define DRIVER_NAME    MAX77696_CHG_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define CHG_RWC_INTERRUPT       0

#define CHG_INT                 0x08
#define CHG_INT_MASK            0x09

#define CHG_INT_OK              0x0A
#define CHG_INT_OK_CHGINA       BIT (6)
#define CHG_INT_OK_CHG          BIT (4)
#define CHG_INT_OK_BAT          BIT (3)
#define CHG_INT_OK_THM          BIT (2)
#define CHG_INT_OK_SYS2         BIT (0)

#define CHG_DTLS_00             0x0B
#define CHG_DTLS_00_CHGINA_DTLS BITS(6,5)
#define CHG_DTLS_00_THM_DTLS    BITS(2,0)

#define CHG_DTLS_01             0x0C
#define CHG_DTLS_01_TREG        BIT (7)
#define CHG_DTLS_01_BAT_DTLS    BITS(6,4)
#define CHG_DTLS_01_CHG_DTLS    BITS(3,0)

#define CHG_DTLS_02             0x0D
#define CHG_DTLS_02_SYS2_DTLS   BITS(3,0)

#define CHG_INT_CHGINA          BIT (6)
#define CHG_INT_CHG             BIT (4)
#define CHG_INT_BAT             BIT (3)
#define CHG_INT_THM             BIT (2)
#define CHG_INT_SYS2            BIT (0)
#define CHG_INT_ALL             0x5D

#define CHG_CNFG_00             0x0F
#define CHG_CNFG_00_WDTEN       BIT (4)
#define CHG_CNFG_00_MODE        BITS(3,0)

#define CHG_CNFG_01             0x10
#define CHG_CNFG_01_PQEN        BIT (7)
#define CHG_CNFG_01_CHG_RSTRT   BITS(5,4)
#define CHG_CNFG_01_FCHGTIME    BITS(2,0)

#define CHG_CNFG_02             0x11
#define CHG_CNFG_02_OTGA_ILIM   BIT (7)
#define CHG_CNFG_02_CHG_CC      BITS(5,0)

#define CHG_CNFG_03             0x12
#define CHG_CNFG_03_TO_TIME     BITS(5,3)
#define CHG_CNFG_03_TO_ITH      BITS(2,0)

#define CHG_CNFG_04             0x13
#define CHG_CNFG_04_MINVSYS1    BITS(7,5)
#define CHG_CNFG_04_CHG_CV_PRM  BITS(4,0)

#define CHG_CNFG_05             0x14
#define CHG_CNFG_05_CHG_CV_JTA  BITS(4,0)

#define CHG_CNFG_06             0x15
#define CHG_CNFG_06_CHGPROT     BITS(3,2)
#define CHG_CNFG_06_WDTCLR      BITS(1,0)

#define CHG_CNFG_07             0x16
#define CHG_CNFG_07TEMP         BITS(6,5)

#define CHG_CNFG_08             0x17

#define CHG_CNFG_09             0x18
#define CHG_CNFG_09_CHGA_ICL    BIT (7)
#define CHG_CNFG_09_CHGINA_ILIM BITS(6,0)

#define CHG_CNFG_10             0x19

#define CHG_CNFG_11             0x1A
#define CHG_CNFG_11_VSYS2SET    BITS(6,0)

#define CHG_CNFG_12             0x1B
#define CHG_CNFG_12_CHG_LPM     BIT (7)
#define CHG_CNFG_12_VCHGIN      BITS(4,3)
#define CHG_CNFG_12_B2SOVRC     BITS(2,0)

#define CHG_CNFG_13             0x1C

#define CHG_CNFG_14             0x1D
#define CHG_CNFG_14_JEITA       BIT (7)
#define CHG_CNFG_14_T4          BITS(6,5)
#define CHG_CNFG_14_T3          BITS(4,3)
#define CHG_CNFG_14_T2          BIT (2)
#define CHG_CNFG_14_T1          BITS(1,0)

struct max77696_chg {
    struct mutex                        lock;
    struct max77696_chg_platform_data  *pdata;
    struct max77696_core               *core;
    struct max77696_io                 *io;
    struct device                      *dev;
    struct kobject                     *kobj;
    const struct attribute_group       *attr_grp;

    int                                 irq;
    u16                                 irq_unmask;

    struct power_supply                *psy;

    int                                 present;
    int                                 enabled;
    int                                 status;
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

enum {
    CHGINA_DTLS_UVLO    = 0b00,
    CHGINA_DTLS_SYSUVLO = 0b01,
    CHGINA_DTLS_OVLO    = 0b10,
    CHGINA_DTLS_VALID   = 0b11,
};

static char* max77696_chg_chgina_details[] = {
    [CHGINA_DTLS_UVLO   ] = "Vchgina < Vchgina_uvlo",
    [CHGINA_DTLS_SYSUVLO] = "Vchgina < Vmbat + Vchgina2sys1",
    [CHGINA_DTLS_OVLO   ] = "Vchgina > Vchgina_ovlo",
    [CHGINA_DTLS_VALID  ] = "Vchgina okay",
};

enum {
    THM_DTLS_NORMAL = 0b000,
    THM_DTLS_COOL   = 0b001,
    THM_DTLS_COLD   = 0b011,
    THM_DTLS_WARM   = 0b100,
    THM_DTLS_ERROR  = 0b101,
    THM_DTLS_HOT    = 0b110,
};

static char* max77696_chg_thm_details[] = {
    [THM_DTLS_NORMAL] = "normal; T2 < T < T3",
    [THM_DTLS_COOL  ] = "cool; T1 < T < T2",
    [THM_DTLS_COLD  ] = "cold; T < T1",
    [THM_DTLS_WARM  ] = "warm; T3 < T < T4",
    [THM_DTLS_ERROR ] = "no temperature",
    [THM_DTLS_HOT   ] = "hot; T > T4",
};

enum {
	BAT_DTLS_NO_BATTERY    = 0b000,
	BAT_DTLS_DEAD          = 0b001,
	BAT_DTLS_TIMER_FAULT   = 0b010,
	BAT_DTLS_OKAY          = 0b011,
	BAT_DTLS_OKAY_LOW      = 0b100,
	BAT_DTLS_OVERCURRENT   = 0b110,
};

static char* max77696_chg_bat_details[] = {
    [BAT_DTLS_NO_BATTERY ] = "no battery",
    [BAT_DTLS_DEAD       ] = "Vmbat < Vpqlb",
    [BAT_DTLS_TIMER_FAULT] = "too long charging",
    [BAT_DTLS_OKAY       ] = "okay; Vmbat > Vsysmin",
    [BAT_DTLS_OKAY_LOW   ] = "okay; Vmbat < Vsysmin",
    [BAT_DTLS_OVERCURRENT] = "overcurrent",
};

enum {
    TREG_DTLS_BELOW = 0,
    TREG_DTLS_OVER  = 1,
};

static char* max77696_chg_treg_details[] = {
    [TREG_DTLS_BELOW] = "Tj < REGTEMP threshold",
    [TREG_DTLS_OVER ] = "Tj > REGTEMP threshold",
};

enum {
	CHG_DTLS_PREQUAL        = 0b0000,
	CHG_DTLS_FASTCHARGE_CC  = 0b0001,
	CHG_DTLS_FASTCHARGE_CV  = 0b0010,
	CHG_DTLS_TOPOFF         = 0b0011,
	CHG_DTLS_DONE           = 0b0100,
	CHG_DTLS_HIGH_TEMP      = 0b0101,
	CHG_DTLS_TIMER_FAULT    = 0b0110,
	CHG_DTLS_THERM_SUSPEND  = 0b0111,
	CHG_DTLS_INPUT_INVALID  = 0b1000,
	CHG_DTLS_TJ_SHDN        = 0b1010,
	CHG_DTLS_WDT_EXPIRED    = 0b1011,
};

static char* max77696_chg_chg_details[] = {
    [CHG_DTLS_PREQUAL       ] = "in prequalification mode",
    [CHG_DTLS_FASTCHARGE_CC ] = "in fast-charge constant current mode",
    [CHG_DTLS_FASTCHARGE_CV ] = "in fast-charge constant voltage mode",
    [CHG_DTLS_TOPOFF        ] = "in top-off mode",
    [CHG_DTLS_DONE          ] = "in done mode",
    [CHG_DTLS_HIGH_TEMP     ] = "in high temperature charging mode",
    [CHG_DTLS_TIMER_FAULT   ] = "in timer fault mode",
    [CHG_DTLS_THERM_SUSPEND ] = "in thermistor suspend mode",
    [CHG_DTLS_INPUT_INVALID ] = "off; invalid input and/or disabled",
    [CHG_DTLS_TJ_SHDN       ] = "off; T > Tj_shdn",
    [CHG_DTLS_WDT_EXPIRED   ] = "off; watchdog timer expired",
};

static __always_inline
void max77696_chg_enable_irq (struct max77696_chg *chg, u16 irq_bits)
{
    u16 chgintm;
    int rc;

    if (unlikely((chg->irq_unmask & irq_bits) == irq_bits)) {
        /* already unmasked or null bit */
        return;
    }

    if (unlikely(!chg->irq_unmask)) {
        enable_irq(chg->irq);
        enable_irq_wake(chg->irq);
    }

    /* set chg_present flag */
    chg->irq_unmask |= irq_bits;

    /* unmask irq */
    chgintm = ~chg->irq_unmask;

    rc = max77696_write(chg->io, CHG_INT_MASK, chgintm);
    dev_dbg(chg->dev, "CHG_INT_MASK written 0x%04X [%d]\n", chgintm, rc);

    if (unlikely(rc)) {
        dev_err(chg->dev, "CHG_INT_MASK write error [%d]\n", rc);
    }
}

static __always_inline
void max77696_chg_disable_irq (struct max77696_chg *chg, u16 irq_bits)
{
    u16 chgintm;
    int rc;

    if (unlikely((chg->irq_unmask & irq_bits) == 0)) {
        /* already masked or null bit */
        return;
    }

    /* clear chg_present flag */
    chg->irq_unmask &= ~irq_bits;

    if (unlikely(!chg->irq_unmask)) {
        disable_irq_wake(chg->irq);
        disable_irq(chg->irq);
    }

    /* mask irq */
    chgintm = ~chg->irq_unmask;

    rc = max77696_write(chg->io, CHG_INT_MASK, chgintm);
    dev_dbg(chg->dev, "CHG_INT_MASK written 0x%04X [%d]\n", chgintm, rc);

    if (unlikely(rc)) {
        dev_err(chg->dev, "CHG_INT_MASK write error [%d]\n", rc);
    }
}

static __always_inline
u16 max77696_chg_read_irq (struct max77696_chg *chg)
{
    u16 chgint;
    int rc;

    rc = max77696_read(chg->io, CHG_INT, &chgint);
    if (unlikely(rc)) {
        dev_err(chg->dev, "CHG_INT read error [%d]\n", rc);
        return 0;
    }

    return chgint;
}

static __always_inline
void max77696_chg_ack_irq (struct max77696_chg *chg)
{
    if (CHG_RWC_INTERRUPT) {
        max77696_write(chg->io, CHG_INT, ~0);
    }
}

enum {
    CFG_CHGPROT = 0,
    CFG_CHG_CV_PRM,
    CFG_CHG_CC,
    CFG_CHGINA_ILIM,
};

static struct max77696_bitdesc max77696_chg_cfg_bitdescs[] = {
    #define CFG_BITDESC(_cfg_bit, _cfg_reg) \
            [CFG_##_cfg_bit] =\
                MAX77696_BITDESC(_cfg_reg, _cfg_reg##_##_cfg_bit)

    CFG_BITDESC(CHGPROT    , CHG_CNFG_06),
    CFG_BITDESC(CHG_CV_PRM , CHG_CNFG_04),
    CFG_BITDESC(CHG_CC     , CHG_CNFG_02),
    CFG_BITDESC(CHGINA_ILIM, CHG_CNFG_09),
};
#define __cfg_bitdesc(_cfg) (&max77696_chg_cfg_bitdescs[CFG_##_cfg])

#define CHGPROT_UNLOCK  3
#define CHGPROT_LOCK    0

static __always_inline
int max77696_chg_unlock (struct max77696_chg *chg)
{
    int rc;

    rc = max77696_write_bitdesc(chg->io,
        __cfg_bitdesc(CHGPROT), CHGPROT_UNLOCK);
    if (unlikely(rc)) {
        dev_err(chg->dev, "failed to unlock [%d]\n", rc);
    }

    return rc;
}

static __always_inline
int max77696_chg_lock (struct max77696_chg *chg)
{
    int rc;

    rc = max77696_write_bitdesc(chg->io,
        __cfg_bitdesc(CHGPROT), CHGPROT_LOCK);
    if (unlikely(IS_ERR_VALUE(rc))) {
        dev_err(chg->dev, "failed to lock [%d]\n", rc);
    }

    return rc;
}

#define max77696_chg_read_config(_chg, _cfg, _val_ptr) \
        ({\
            int __rc = max77696_read_bitdesc((_chg)->io, __cfg_bitdesc(_cfg),\
                _val_ptr);\
            if (unlikely(__rc)) {\
                dev_err(_chg->dev, "read config "#_cfg" error [%d]\n", __rc);\
            } else {\
                dev_vdbg(_chg->dev, "read config "#_cfg": %Xh\n", *(_val_ptr));\
            }\
            __rc;\
        })
#define max77696_chg_write_config(_chg, _cfg, _val) \
        ({\
            int __rc = max77696_chg_unlock(_chg);\
            if (likely(!__rc)) {\
                __rc = max77696_write_bitdesc((_chg)->io, __cfg_bitdesc(_cfg),\
                    _val);\
                if (unlikely(__rc)) {\
                    dev_err(_chg->dev,\
                        "write config "#_cfg" error [%d]\n", __rc);\
                } else {\
                    dev_vdbg(_chg->dev, "write config "#_cfg": %Xh\n", _val);\
                }\
                max77696_chg_lock(_chg);\
            }\
            __rc;\
        })

static __inline int max77696_chg_read_cc (struct max77696_chg *chg, int *uA)
{
    int rc;
    u16 chg_cc;

    rc = max77696_chg_read_config(chg, CHG_CC, &chg_cc);
    if (unlikely(rc)) {
        goto out;
    }

    *uA = max_t(int, 2, chg_cc) * 100 * 1000 / 3;

out:
    return rc;
}

static __inline int max77696_chg_write_cc (struct max77696_chg *chg, int uA)
{
    u16 chg_cc;
    int rc;

    chg_cc = uA < 0x02 * 100 * 1000 / 3 ? 0x02 :
             uA < 0x3F * 100 * 1000 / 3 ? DIV_ROUND_UP(3 * uA, 100 * 1000) :
             0x3F;

    rc = max77696_chg_write_config(chg, CHG_CC, chg_cc);
    if (unlikely(rc)) {
        goto out;
    }

    dev_dbg(chg->dev, "CC write 0x%04X = %duA\n", chg_cc, uA);

out:
    return rc;
}

static __inline
int max77696_chg_read_chgina_ilim (struct max77696_chg *chg, int *uA)
{
    int rc;
    u16 chgina_ilim;

    rc = max77696_chg_read_config(chg, CHGINA_ILIM, &chgina_ilim);
    if (unlikely(rc)) {
        goto out;
    }

    *uA = max_t(int, 3, chgina_ilim) * 20 * 1000;

out:
    return rc;
}

static __inline
int max77696_chg_write_chgina_ilim (struct max77696_chg *chg, int uA)
{
    u16 chgina_ilim;
    int rc;

    chgina_ilim = uA < 0x03 * 20 * 1000 ? 0x03 :
                  uA < 0x7F * 20 * 1000 ? DIV_ROUND_UP(uA, 25 * 1000) :
                  0x7F;

    rc = max77696_chg_write_config(chg, CHGINA_ILIM, chgina_ilim);
    if (unlikely(rc)) {
        goto out;
    }

    dev_dbg(chg->dev, "CHGINA ILIM write 0x%04X = %duA\n",
        chgina_ilim, uA);

out:
    return rc;
}

static int max77696_chg_setup (struct max77696_chg *chg)
{
    /* Device wakeup initialization */
    device_init_wakeup(chg->dev, true);

    return 0;
}

#define DEFINE_CHARGER_DETAIL_DEV_ATTR(_name, _reg, _bit) \
static ssize_t max77696_chg_##_name##_show (struct device *dev,\
		struct device_attribute *attr, char *buf)\
{\
	struct max77696_chg *chg = dev_get_drvdata(dev);\
	u16 val;\
	int rc;\
	__lock(chg);\
	rc = max77696_read_reg_bit(chg->io, CHG_##_reg, _bit, &val);\
	if (unlikely(rc)) {\
        rc = (int)snprintf(buf, PAGE_SIZE, "I/O exception [%d]", rc);\
	    goto out;\
	}\
	rc = (int)snprintf(buf, PAGE_SIZE, "0x%04X ", val);\
	if (likely(val < ARRAY_SIZE(max77696_chg_##_name##s))) {\
	    if (likely(max77696_chg_##_name##s[val])) {\
        	rc += (int)snprintf(buf+rc, PAGE_SIZE-rc,\
        	    max77696_chg_##_name##s[val]);\
        }\
	}\
out:\
    rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "\n");\
    __unlock(chg);\
	return rc;\
}\
static DEVICE_ATTR(_name, S_IRUGO, max77696_chg_##_name##_show, NULL)

DEFINE_CHARGER_DETAIL_DEV_ATTR(chgina_detail, DTLS_00, CHGINA_DTLS);
DEFINE_CHARGER_DETAIL_DEV_ATTR(thm_detail   , DTLS_00, THM_DTLS   );
DEFINE_CHARGER_DETAIL_DEV_ATTR(treg_detail  , DTLS_01, TREG       );
DEFINE_CHARGER_DETAIL_DEV_ATTR(bat_detail   , DTLS_01, BAT_DTLS   );
DEFINE_CHARGER_DETAIL_DEV_ATTR(chg_detail   , DTLS_01, CHG_DTLS   );

static ssize_t max77696_chg_sys2_detail_show (struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77696_chg *chg = dev_get_drvdata(dev);
	u16 sys2dtls;
	int rc;

	__lock(chg);

	rc = max77696_read_reg_bit(chg->io, CHG_DTLS_02, SYS2_DTLS, &sys2dtls);
	if (unlikely(rc)) {
        rc = (int)snprintf(buf, PAGE_SIZE, "I/O exception [%d]", rc);
	    goto out;
	}

	rc = (int)snprintf(buf, PAGE_SIZE, "0x%04X ", sys2dtls);

    if (unlikely(!sys2dtls)) {
    	rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "OK");
    	goto out;
    }

    /* OTGILIM */
    if (sys2dtls & 0b0001) {
    	rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "OTGILIM");
    } else {
        rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "-");
    }

    /* BSTILIM */
    if (sys2dtls & 0b0010) {
        rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, " / BSTILIM");
    } else {
        rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, " / -");
    }

    /* VCHGINLIM */
    if (sys2dtls & 0b1000) {
        rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, " / VCHGINLIM");
    } else {
        rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, " / -");
    }

out:
    rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "\n");
    __unlock(chg);
	return rc;
}
static DEVICE_ATTR(sys2_detail, S_IRUGO, max77696_chg_sys2_detail_show, NULL);

static ssize_t max77696_chg_all_detail_show (struct device *dev,
		struct device_attribute *attr, char *buf)
{
    int rc = 0;

    rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "CHGINA: ");
    rc += max77696_chg_chgina_detail_show(dev, attr, buf+rc);

    rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "THM:    ");
    rc += max77696_chg_thm_detail_show(dev, attr, buf+rc);

    rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "TREG:   ");
    rc += max77696_chg_treg_detail_show(dev, attr, buf+rc);

    rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "BAT:    ");
    rc += max77696_chg_bat_detail_show(dev, attr, buf+rc);

    rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "CHG:    ");
    rc += max77696_chg_chg_detail_show(dev, attr, buf+rc);

    rc += (int)snprintf(buf+rc, PAGE_SIZE-rc, "SYS2:   ");
    rc += max77696_chg_sys2_detail_show(dev, attr, buf+rc);

	return rc;
}
static DEVICE_ATTR(all_detail, S_IRUGO, max77696_chg_all_detail_show, NULL);

#define CHG_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_chg_attr[] = {
    CHG_DEV_ATTR(chgina_detail),
    CHG_DEV_ATTR(thm_detail   ),
    CHG_DEV_ATTR(treg_detail  ),
    CHG_DEV_ATTR(bat_detail   ),
    CHG_DEV_ATTR(chg_detail   ),
    CHG_DEV_ATTR(sys2_detail  ),
    CHG_DEV_ATTR(all_detail   ),

    NULL
};

static const struct attribute_group max77696_chg_attr_group = {
    .attrs = max77696_chg_attr,
};

static int max77696_chg_update_status (struct max77696_chg *chg)
{
    u16 chgdtls;
    int rc;

    rc = max77696_read_reg_bit(chg->io, CHG_DTLS_01, CHG_DTLS, &chgdtls);
    if (unlikely(rc)) {
        dev_err(chg->dev, "CHG_DTLS_01 read error [%d]\n", rc);
        goto out;
    }

    dev_dbg(chg->dev, "CHG_DTLS 0x%04X\n", chgdtls );

	if (likely(chgdtls< ARRAY_SIZE(max77696_chg_chg_details))) {
	    if (likely(max77696_chg_chg_details[chgdtls])) {
	        dev_dbg(chg->dev, "CHG_DTLS charger is %s\n",
	            max77696_chg_chg_details[chgdtls]);
        }
	}

    switch (chgdtls) {
    case CHG_DTLS_PREQUAL:
    case CHG_DTLS_FASTCHARGE_CC:
    case CHG_DTLS_FASTCHARGE_CV:
        chg->present = 1;
        chg->enabled = 1;
        chg->status  = POWER_SUPPLY_STATUS_CHARGING;
        break;

    case CHG_DTLS_TOPOFF:
    case CHG_DTLS_DONE:
        chg->present = 1;
        chg->enabled = 1;
        chg->status  = POWER_SUPPLY_STATUS_FULL;
        break;

    case CHG_DTLS_INPUT_INVALID:
        chg->present = 0;
        chg->enabled = 0;
        chg->status  = POWER_SUPPLY_STATUS_DISCHARGING;
        break;

    case CHG_DTLS_HIGH_TEMP:
    case CHG_DTLS_TIMER_FAULT:
    case CHG_DTLS_THERM_SUSPEND:
    case CHG_DTLS_TJ_SHDN:
    case CHG_DTLS_WDT_EXPIRED:
        dev_err(chg->dev, "abnormal charger status - 0x%04X\n", chgdtls);
        chg->present = 1;
        chg->enabled = 0;
        chg->status  = POWER_SUPPLY_STATUS_NOT_CHARGING;
        break;

    default:
        dev_err(chg->dev, "unknown charger status - 0x%04X\n", chgdtls);
        chg->present = 0;
        chg->enabled = 0;
        chg->status  = POWER_SUPPLY_STATUS_UNKNOWN;
        break;
    }

out:
    return rc;
}

static irqreturn_t max77696_chg_isr (int irq, void *data)
{
    struct max77696_chg *chg = data;
    u16 chgint, chgok;

    chgint = max77696_chg_read_irq(chg);
    max77696_chg_ack_irq(chg);
    dev_dbg(chg->dev, "CHGINT 0x%04X EN 0x%04X\n", chgint, chg->irq_unmask);

    max77696_read(chg->io, CHG_INT_OK, &chgok);
    dev_dbg(chg->dev, "CHGOK  0x%04X\n", chgok);

    chgint &= chg->irq_unmask;

    if (chgint & CHG_INT_CHGINA) {
        dev_dbg(chg->dev, "CHGINA interrupt received\n");
    }

    if (chgint & CHG_INT_CHG) {
        dev_dbg(chg->dev, "CHG interrupt received\n");
    }

    if (chgint & CHG_INT_BAT) {
        dev_dbg(chg->dev, "BAT interrupt received\n");
    }

    if (chgint & CHG_INT_THM) {
        dev_dbg(chg->dev, "THM interrupt received\n");
    }

    if (chgint & CHG_INT_SYS2) {
        dev_dbg(chg->dev, "SYS2 interrupt received\n");
    }

    if (likely(chg->psy)) {
        power_supply_changed(chg->psy);
    }
    return IRQ_HANDLED;
}

static int max77696_chg_psy_get_property (struct power_supply *psy,
        enum power_supply_property psp, union power_supply_propval *val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    struct max77696_chg *chg = dev_get_drvdata(psy->dev->parent);
#else /* VERSION ... */
    struct max77696_chg *chg = power_supply_get_drvdata(psy);
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

    __lock(chg);

    max77696_chg_update_status(chg);

    switch (psp) {
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = chg->present;
        break;

    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = chg->enabled;
        break;

    case POWER_SUPPLY_PROP_STATUS:
        val->intval = chg->status;
        break;

    default:
        rc = -EINVAL;
        goto out;
    }

out:
    dev_vdbg(chg->dev,
        "<get_property> psp %d val %d [%d]\n", psp, val->intval, rc);
    __unlock(chg);
    return rc;
}

#define max77696_chg_psy_set_property           NULL
#define max77696_chg_psy_property_is_writeable  NULL
#define max77696_chg_psy_external_power_changed NULL

static enum power_supply_property max77696_chg_psy_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_MODEL_NAME,
    POWER_SUPPLY_PROP_MANUFACTURER,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
static struct power_supply_desc max77696_chg_psy_desc = {
	.name		            = DRIVER_NAME,
	.type		            = POWER_SUPPLY_TYPE_MAINS,
    .properties             = max77696_chg_psy_props,
    .num_properties         = ARRAY_SIZE(max77696_chg_psy_props),
    .get_property           = max77696_chg_psy_get_property,
    .set_property           = max77696_chg_psy_set_property,
    .property_is_writeable  = max77696_chg_psy_property_is_writeable,
    .external_power_changed = max77696_chg_psy_external_power_changed,
};

static struct power_supply_config max77696_chg_psy_config;
#endif /* VERSION >= 4.1.0 */

static void *max77696_chg_get_platdata (struct max77696_chg *chg)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_chg_platform_data *pdata;
    struct device *dev = chg->dev;
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

out:
    return pdata;
}

static __always_inline void max77696_chg_destroy (struct max77696_chg *chg)
{
    struct device *dev = chg->dev;

    if (likely(chg->attr_grp)) {
        sysfs_remove_group(chg->kobj, chg->attr_grp);
    }

    if (likely(chg->irq > 0)) {
        devm_free_irq(dev, chg->irq, chg);
    }

    if (likely(chg->psy)) {
        power_supply_unregister(chg->psy);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(chg->pdata)) {
        devm_kfree(dev, chg->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&chg->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, chg);
}

static __devinit int max77696_chg_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_chg *chg;
    size_t sz;
    u16 chgint;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    sz  = sizeof(*chg);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    sz += sizeof(struct power_supply);
#endif /* VERSION < 4.1.0 */

    chg = devm_kzalloc(dev, sz, GFP_KERNEL);
    if (unlikely(!chg)) {
        dev_err(dev, "out of memory (%uB requested)\n", sz);
        return -ENOMEM;
    }

    dev_set_drvdata(dev, chg);

    mutex_init(&chg->lock);
    chg->core  = core;
    chg->io    = max77696_get_block_io(dev->parent, CHG);
    chg->dev   = dev;
    chg->kobj  = &dev->kobj;

    /* Disable all CHG interrupts */
    chg->irq_unmask = 0;
    max77696_write(chg->io, CHG_INT_MASK, ~0);

    /* Get CHG interrupt status port address & Clear status */
    chgint = max77696_chg_read_irq(chg);
    max77696_chg_ack_irq(chg);
    dev_dbg(dev, "initial CHG interrupt status: 0x%04X\n", chgint);

    chg->pdata = max77696_chg_get_platdata(chg);
    if (unlikely(IS_ERR(chg->pdata))) {
        rc = PTR_ERR(chg->pdata);
        chg->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

	rc = max77696_chg_setup(chg);
	if (unlikely(rc)) {
	    dev_err(dev, "failed to setup [%d]\n", rc);
	    goto abort;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
    chg->psy                         = (struct power_supply*)(chg + 1);
    chg->psy->name                   =
        chg->pdata->psy_name ? chg->pdata->psy_name : DRIVER_NAME;
    chg->psy->type                   = POWER_SUPPLY_TYPE_MAINS;
    chg->psy->supplied_to            = chg->pdata->supplied_to;
    chg->psy->num_supplicants        = chg->pdata->num_supplicants;
    chg->psy->properties             = max77696_chg_psy_props;
    chg->psy->num_properties         = ARRAY_SIZE(max77696_chg_psy_props);
    chg->psy->get_property           = max77696_chg_psy_get_property;
    chg->psy->set_property           = max77696_chg_psy_set_property;
    chg->psy->property_is_writeable  =
        max77696_chg_psy_property_is_writeable;
    chg->psy->external_power_changed =
        max77696_chg_psy_external_power_changed;

    rc = power_supply_register(dev, chg->psy);
    if (unlikely(rc)) {
        dev_err(dev, "failed to register power_supply class [%d]\n", rc);
        chg->psy = NULL;
        goto abort;
    }
#else /* VERSION ... */
    if (chg->pdata->psy_name) {
        max77696_chg_psy_desc.name = chg->pdata->psy_name;
    }

    max77696_chg_psy_config.drv_data        = chg;
    max77696_chg_psy_config.supplied_to     = chg->pdata->supplied_to;
    max77696_chg_psy_config.num_supplicants = chg->pdata->num_supplicants;

    chg->psy = power_supply_register(dev, &max77696_chg_psy_desc,
                        &max77696_chg_psy_config);
    if (unlikely(IS_ERR(chg->psy))) {
        rc = PTR_ERR(chg->psy);
        chg->psy = NULL;
        dev_err(dev, "failed to register power_supply class [%d]\n", rc);
        goto abort;
    }
#endif /* VERSION ... */

    /* Get CHG block IRQ number */
    chg->irq = max77696_get_block_irq(dev->parent, CHG);
    BUG_ON(chg->irq <= 0);

    /* Request system IRQ for CHG */
    rc = devm_request_threaded_irq(dev, (unsigned int)chg->irq, NULL,
        max77696_chg_isr, IRQF_ONESHOT, DRIVER_NAME, chg);
    if (unlikely(rc)) {
        dev_err(dev, "failed to request IRQ(%d) [%d]\n", chg->irq, rc);
        chg->irq = 0;
        goto abort;
    }

    disable_irq(chg->irq);
    dev_dbg(dev, "IRQ(%d) requested\n", chg->irq);

    /* Create max77696-chg sysfs attributes */
    chg->attr_grp = &max77696_chg_attr_group;
    rc = sysfs_create_group(chg->kobj, chg->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        chg->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_chg_destroy(chg);
    return rc;
}

static __devexit int max77696_chg_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_chg *chg = dev_get_drvdata(dev);

    max77696_chg_destroy(chg);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_chg_suspend (struct device *dev)
{
    return 0;
}

static int max77696_chg_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_chg_pm,
    max77696_chg_suspend, max77696_chg_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_chg_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_chg_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_chg_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_chg_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_chg_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_chg_probe,
    .remove                = __devexit_p(max77696_chg_remove),
};

static __init int max77696_chg_driver_init (void)
{
    return platform_driver_register(&max77696_chg_driver);
}

static __exit void max77696_chg_driver_exit (void)
{
    platform_driver_unregister(&max77696_chg_driver);
}

module_init(max77696_chg_driver_init);
module_exit(max77696_chg_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
