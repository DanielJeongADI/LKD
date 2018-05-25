/*
 * MAX77696 USB Interface Circuit Driver
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

#define DRIVER_DESC    MAX77696_DESC" UIC Driver"
#define DRIVER_NAME    MAX77696_UIC_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define UIC_RWC_INTERRUPT       0
#define UIC_PSY_WORK_DELAY      msecs_to_jiffies(0)

#define DEVICEID                0x00
#define DEVICEID_CHIPID         BITS(7,4)
#define DEVICEID_CHIPREV        BITS(3,0)

#define INT1                    0x01
#define INT2                    0x02

#define STATUS1                 0x03
#define STATUS1_DCDTMR          BIT (7)
#define STATUS1_CHGDETACT       BIT (6)
#define STATUS1_VBVOLT          BIT (4)
#define STATUS1_CHGTYP          BITS(3,0)

#define STATUS2                 0x04
#define STATUS2_ADCERROR        BIT (7)
#define STATUS2_ENUSTAT         BIT (6)
#define STATUS2_ADC             BITS(4,0)

#define INTMASK1                0x05
#define INTMASK2                0x06

#define SYSCTRL1                0x07
#define SYSCTRL1_INT_TYP        BIT (7)
#define SYSCTRL1_INT_DLY        BIT (6)
#define SYSCTRL1_INT_POL        BIT (5)
#define SYSCTRL1_INT_EN         BIT (4)
#define SYSCTRL1_USBSWC         BITS(3,2)
#define SYSCTRL1_LOW_POW        BIT (0)

#define SYSCTRL2                0x08
#define SYSCTRL2_ADC_DEB        BITS(4,3)
#define SYSCTRL2_DCDCPL         BIT (2)
#define SYSCTRL2_IDAUTOSWC      BIT (1)
#define SYSCTRL2_ADC_EN         BIT (0)

#define CDETCTRL                0x09
#define CDETCTRL_SFEN           BITS(7,6)
#define CDETCTRL_CDPDET         BIT (5)
#define CDETCTRL_DCHKTM         BIT (4)
#define CDETCTRL_DCD2SCT        BIT (3)
#define CDETCTRL_DCDEN          BIT (2)
#define CDETCTRL_CHGTYPMAN      BIT (1)
#define CDETCTRL_CHGDETEN       BIT (0)

#define MANCTRL                 0x0A
#define MANCTRL_OIDSET          BITS(7,6)
#define MANCTRL_FIDSET          BITS(5,4)
#define MANCTRL_MANSET          BIT (3)
#define MANCTRL_ISET            BITS(2,0)

#define CHGCTRL                 0x0B
#define CHGCTRL_AUTH500         BIT (7)
#define CHGCTRL_ENUMSUB         BIT (6)
#define CHGCTRL_DCPSET          BITS(5,4)
#define CHGCTRL_ENUMEN          BIT (3)
#define CHGCTRL_IMAX            BITS(2,0)

#define ENUCTRL                 0x0C
#define ENUCTRL_ENUMTM          BITS(1,0)

#define INTSTS                  0x0D
#define INTSTS_UICWK            BIT (7)
#define INTSTS_CHG_DET_ACT      BIT (6)
#define INTSTS_FACTORY          BIT (5)
#define INTSTS_DB               BIT (4)
#define INTSTS_SFEN             BIT (3)
#define INTSTS_ISET             BITS(2,0)

/*** Interrupt-2 Bits ***/
#define INT2_ADCERROR           BIT (23)
#define INT2_ENUSTAT            BIT (22)
//                              BIT (21)
//                              BIT (20)
//                              BIT (19)
//                              BIT (18)
//                              BIT (17)
#define INT2_ADC                BIT (16)

/*** Interrupt-1 Bits ***/
#define INT1_DCDTMR             BIT ( 7)
#define INT1_CHGDETACTRISE      BIT ( 6)
#define INT1_CHGDETACTFALL      BIT ( 5)
//                              BIT ( 4)
//                              BIT ( 3)
//                              BIT ( 2)
#define INT1_VBVOLT             BIT ( 1)
#define INT1_CHGTYP             BIT ( 0)

struct max77696_uic {
    struct mutex                       lock;
    struct max77696_uic_platform_data *pdata;
    struct max77696_core              *core;
    struct max77696_io                *io;
    struct device                     *dev;
    struct kobject                    *kobj;
    const struct attribute_group      *attr_grp;

    struct delayed_work                psy_work;

    unsigned int                       irq;
    u32                                irq_unmask;

    struct power_supply               *psy_chg;
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

static __always_inline
void max77696_uic_enable_irq (struct max77696_uic *uic, u32 irq_bits)
{
    u16 intmask;
    int rc;

    if (unlikely((uic->irq_unmask & irq_bits) == irq_bits)) {
        /* already unmasked or null bit */
        return;
    }

    if (unlikely(!uic->irq_unmask)) {
        max77696_write_reg_bit(uic->io, SYSCTRL1, INT_EN, true);

        enable_irq(uic->irq);
      //enable_irq_wake(uic->irq);
    }

    /* set enabled flag */
    uic->irq_unmask |= irq_bits;

    intmask = (u16)((uic->irq_unmask >>  0) & 0xFFFF);
    rc = max77696_write(uic->io, INTMASK1, intmask);
    dev_dbg(uic->dev, "written INTMASK1 0x%04X [%d]\n", intmask, rc);

    intmask = (u16)((uic->irq_unmask >> 16) & 0xFFFF);
    rc = max77696_write(uic->io, INTMASK2, intmask);
    dev_dbg(uic->dev, "written INTMASK2 0x%04X [%d]\n", intmask, rc);
}

static __always_inline
void max77696_uic_disable_irq (struct max77696_uic *uic, u32 irq_bits)
{
    u16 intmask;
    int rc;

    if (unlikely((uic->irq_unmask & irq_bits) == 0)) {
        /* already masked or null bit */
        return;
    }

    /* clear enabled flag */
    uic->irq_unmask &= ~irq_bits;

    if (unlikely(!uic->irq_unmask)) {
        max77696_write_reg_bit(uic->io, SYSCTRL1, INT_EN, false);

      //disable_irq_wake(uic->irq);
        disable_irq(uic->irq);
    }

    intmask = (u16)((uic->irq_unmask >>  0) & 0xFFFF);
    rc = max77696_write(uic->io, INTMASK1, intmask);
    dev_dbg(uic->dev, "written INTMASK1 0x%04X [%d]\n", intmask, rc);

    intmask = (u16)((uic->irq_unmask >> 16) & 0xFFFF);
    rc = max77696_write(uic->io, INTMASK2, intmask);
    dev_dbg(uic->dev, "written INTMASK2 0x%04X [%d]\n", intmask, rc);
}

static __always_inline u32 max77696_uic_read_irq (struct max77696_uic *uic)
{
    u16 int1, int2;
    int rc;

    rc = max77696_read(uic->io, INT1, &int1);
    if (unlikely(rc)) {
        dev_err(uic->dev, "INT1 read error [%d]\n", rc);
        int1 = 0;
    }

    rc = max77696_read(uic->io, INT2, &int2);
    if (unlikely(rc)) {
        dev_err(uic->dev, "INT2 read error [%d]\n", rc);
        int2 = 0;
    }

    return ((u32)int2 << 16) | int1;
}

static __always_inline void max77696_uic_ack_irq (struct max77696_uic *uic)
{
    if (UIC_RWC_INTERRUPT) {
        max77696_write(uic->io, INT1, ~0);
        max77696_write(uic->io, INT2, ~0);
    }
}

static int max77696_uic_set_charge_current (struct max77696_uic *uic, int cc_mA)
{
    /* UIC automatically configure CC value for charger */
    return 0;
}

static __always_inline int max77696_uic_setup (struct max77696_uic *uic)
{
    return 0;
}

/* The following lists will be propulated with the UIC device parameters
 * and read/write function pointers
 */

#define DEFINE_UIC_DEV_ATTR_RO(_name, _reg, _bit) \
static ssize_t max77696_uic_##_name##_show (struct device *dev,\
    struct device_attribute *devattr, char *buf)\
{\
    struct max77696_uic *uic = dev_get_drvdata(dev);\
    struct max77696_bitdesc bitdesc = MAX77696_BITDESC(_reg, _reg##_##_bit);\
    u16 val;\
    int rc;\
    __lock(uic);\
    rc = max77696_read_bitdesc(uic->io, &bitdesc, &val);\
    if (unlikely(rc)) {\
        dev_err(dev, ""#_reg" read error [%d]\n", rc);\
        goto out;\
    }\
    rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", val);\
out:\
    __unlock(uic);\
    return (ssize_t)rc;\
}\
static DEVICE_ATTR(_name, S_IRUGO, max77696_uic_##_name##_show, NULL)

#define DEFINE_UIC_DEV_ATTR_RW(_name, _reg, _bit) \
static ssize_t max77696_uic_##_name##_show (struct device *dev,\
    struct device_attribute *devattr, char *buf)\
{\
    struct max77696_uic *uic = dev_get_drvdata(dev);\
    struct max77696_bitdesc bitdesc = MAX77696_BITDESC(_reg, _reg##_##_bit);\
    u16 val;\
    int rc;\
    __lock(uic);\
    rc = max77696_read_bitdesc(uic->io, &bitdesc, &val);\
    if (unlikely(rc)) {\
        dev_err(dev, ""#_reg" read error [%d]\n", rc);\
        goto out;\
    }\
    rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", val);\
out:\
    __unlock(uic);\
    return (ssize_t)rc;\
}\
static ssize_t max77696_uic_##_name##_store (struct device *dev,\
    struct device_attribute *devattr, const char *buf, size_t count)\
{\
    struct max77696_uic *uic = dev_get_drvdata(dev);\
    struct max77696_bitdesc bitdesc = MAX77696_BITDESC(_reg, _reg##_##_bit);\
    u16 val;\
    int rc;\
    __lock(uic);\
    val = (u16)simple_strtoul(buf, NULL, 10);\
    rc = max77696_write_bitdesc(uic->io, &bitdesc, val);\
    if (unlikely(rc)) {\
        dev_err(dev, ""#_reg" write error [%d]\n", rc);\
        goto out;\
    }\
out:\
    __unlock(uic);\
    return (ssize_t)count;\
}\
static DEVICE_ATTR(_name, S_IWUSR|S_IRUGO, max77696_uic_##_name##_show,\
    max77696_uic_##_name##_store)

DEFINE_UIC_DEV_ATTR_RO(chipid   , DEVICEID, CHIPID   );
DEFINE_UIC_DEV_ATTR_RO(chiprev  , DEVICEID, CHIPREV  );
DEFINE_UIC_DEV_ATTR_RO(dcdtmr   , STATUS1,  DCDTMR   );
DEFINE_UIC_DEV_ATTR_RO(chgdetact, STATUS1,  CHGDETACT);
DEFINE_UIC_DEV_ATTR_RO(vbvolt   , STATUS1,  VBVOLT   );
DEFINE_UIC_DEV_ATTR_RO(chgtyp   , STATUS1,  CHGTYP   );
DEFINE_UIC_DEV_ATTR_RO(adcerror , STATUS2,  ADCERROR );
DEFINE_UIC_DEV_ATTR_RO(enustat  , STATUS2,  ENUSTAT  );
DEFINE_UIC_DEV_ATTR_RO(adc      , STATUS2,  ADC      );
DEFINE_UIC_DEV_ATTR_RW(usbswc   , SYSCTRL1, USBSWC   );
DEFINE_UIC_DEV_ATTR_RW(low_pow  , SYSCTRL1, LOW_POW  );
DEFINE_UIC_DEV_ATTR_RW(adc_deb  , SYSCTRL2, ADC_DEB  );
DEFINE_UIC_DEV_ATTR_RW(dcdcpl   , SYSCTRL2, DCDCPL   );
DEFINE_UIC_DEV_ATTR_RW(idautoswc, SYSCTRL2, IDAUTOSWC);
DEFINE_UIC_DEV_ATTR_RW(adc_en   , SYSCTRL2, ADC_EN   );
DEFINE_UIC_DEV_ATTR_RW(sfen     , CDETCTRL, SFEN     );
DEFINE_UIC_DEV_ATTR_RW(cdpdet   , CDETCTRL, CDPDET   );
DEFINE_UIC_DEV_ATTR_RW(dchktm   , CDETCTRL, DCHKTM   );
DEFINE_UIC_DEV_ATTR_RW(dcd2sct  , CDETCTRL, DCD2SCT  );
DEFINE_UIC_DEV_ATTR_RW(dcden    , CDETCTRL, DCDEN    );
DEFINE_UIC_DEV_ATTR_RW(chgtypman, CDETCTRL, CHGTYPMAN);
DEFINE_UIC_DEV_ATTR_RW(chgdeten , CDETCTRL, CHGDETEN );

#define UIC_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute *max77696_uic_attr[] = {
    UIC_DEV_ATTR(chipid   ),
    UIC_DEV_ATTR(chiprev  ),
    UIC_DEV_ATTR(dcdtmr   ),
    UIC_DEV_ATTR(chgdetact),
    UIC_DEV_ATTR(vbvolt   ),
    UIC_DEV_ATTR(chgtyp   ),
    UIC_DEV_ATTR(adcerror ),
    UIC_DEV_ATTR(enustat  ),
    UIC_DEV_ATTR(adc      ),
    UIC_DEV_ATTR(usbswc   ),
    UIC_DEV_ATTR(low_pow  ),
    UIC_DEV_ATTR(adc_deb  ),
    UIC_DEV_ATTR(dcdcpl   ),
    UIC_DEV_ATTR(idautoswc),
    UIC_DEV_ATTR(adc_en   ),
    UIC_DEV_ATTR(sfen     ),
    UIC_DEV_ATTR(cdpdet   ),
    UIC_DEV_ATTR(dchktm   ),
    UIC_DEV_ATTR(dcd2sct  ),
    UIC_DEV_ATTR(dcden    ),
    UIC_DEV_ATTR(chgtypman),
    UIC_DEV_ATTR(chgdeten ),
    NULL
};

static const struct attribute_group max77696_uic_attr_group = {
    .attrs = max77696_uic_attr,
};

#define CHGTYP_SDP            0b0001 /* USB SDP */
#define CHGTYP_CDP            0b0010 /* USB CDP */
#define CHGTYP_DCP            0b0011 /* USB DCP */
#define CHGTYP_APPLE_500MA    0b0100 /* Apple  500mA max */
#define CHGTYP_APPLE_1500MA   0b0101 /* Apple 1500mA max */
#define CHGTYP_APPLE_2000MA   0b0110 /* Apple 2000mA max */
#define CHGTYP_OTH_0          0b0111 /* Other */
#define CHGTYP_SELFENUM_500MA 0b1001 /* Self-enumerated 500mA max */
#define CHGTYP_OTH_1          0b1100 /* Other */

static void max77696_uic_psy_work (struct work_struct *work)
{
    struct max77696_uic *uic =
        container_of(work, struct max77696_uic, psy_work.work);
    u16 chgtyp;
    int cc_mA, rc;

    __lock(uic);

    rc = max77696_read_reg_bit(uic->io, STATUS1, CHGTYP, &chgtyp);
    if (unlikely(rc)) {
        dev_err(uic->dev, "STATUS1 read error [%d]\n", rc);
        goto out;
    }

    switch (chgtyp) {
    case CHGTYP_DCP:
    case CHGTYP_APPLE_1500MA:
        cc_mA = 1500;
        break;

    case CHGTYP_APPLE_2000MA:
        cc_mA = 2000;
        break;

    default:
        cc_mA =  500;
        break;
    }

    rc = max77696_uic_set_charge_current(uic, cc_mA);
    if (unlikely(rc)) {
        dev_err(uic->dev, "failed to set charge current [%d]\n", rc);
        goto out;
    }

out:
    __unlock(uic);
    return;
}

static irqreturn_t max77696_uic_isr (int irq, void *data)
{
    struct max77696_uic *uic = data;
    u32 uicint;

    uicint = max77696_uic_read_irq(uic);
    max77696_uic_ack_irq(uic);
    dev_dbg(uic->dev, "UICINT 0x%08X EN 0x%08X\n", uicint, uic->irq_unmask);

    uicint &= uic->irq_unmask;

    if (unlikely(!uicint)) {
        goto out;
    }

    if (unlikely(delayed_work_pending(&uic->psy_work))) {
        goto out;
    }

    schedule_delayed_work(&uic->psy_work, UIC_PSY_WORK_DELAY);

out:
    return IRQ_HANDLED;
}

static void *max77696_uic_get_platdata (struct max77696_uic *uic)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_uic_platform_data *pdata;
    struct device *dev = uic->dev;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*pdata));
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->int_type = MAX77696_UIC_INTTYP_LEVEL;
    of_property_u16(np, "int_type", &pdata->int_type);

    pdata->int_delay = MAX77696_UIC_INTDLY_2TICKS;
    of_property_u16(np, "int_delay", &pdata->int_delay);

    pdata->int_polarity = MAX77696_UIC_INTPOL_ACTIVE_LOW;
    of_property_u16(np, "int_polarity", &pdata->int_polarity);

    pdata->chg_psy_name = NULL;
    of_property_read_string(np,
        "chg_psy_name", (char const**)&pdata->chg_psy_name);
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "INT TYPE"    , "%u", pdata->int_type    );
    __prop_printk(dev, "INT DELAY"   , "%u", pdata->int_delay   );
    __prop_printk(dev, "INT POLARITY", "%u", pdata->int_polarity);
    __prop_printk(dev, "CHARGER PSY" , "%s", pdata->chg_psy_name);

out:
    return pdata;
}

static __always_inline void max77696_uic_destroy (struct max77696_uic *uic)
{
    struct device *dev = uic->dev;

    if (likely(uic->attr_grp)) {
        sysfs_remove_group(uic->kobj, uic->attr_grp);
    }

    if (likely(uic->irq > 0)) {
        devm_free_irq(dev, uic->irq, uic);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(uic->pdata)) {
        devm_kfree(dev, uic->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&uic->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, uic);
}

static __devinit int max77696_uic_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_uic *uic;
    u16 chipid = 0, chiprev = 0;
    u32 uicint;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    uic = devm_kzalloc(dev, sizeof(*uic), GFP_KERNEL);
    if (unlikely(!uic)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*uic));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, uic);

    mutex_init(&uic->lock);
    uic->core  = core;
    uic->io    = max77696_get_block_io(dev->parent, UIC);
    uic->dev   = dev;
    uic->kobj  = &dev->kobj;

    INIT_DELAYED_WORK(&(uic->psy_work), max77696_uic_psy_work);

    /* Disable all UIC interrupts */
    uic->irq_unmask = 0;
    max77696_write(uic->io, INTMASK1, 0);
    max77696_write(uic->io, INTMASK2, 0);

    /* Get UIC interrupt status port address & Clear status */
    uicint = max77696_uic_read_irq(uic);
    max77696_uic_ack_irq(uic);
    dev_dbg(dev, "initial UIC interrupt status: 0x%08X\n", uicint);

    /* Read & Show UIC version information */
    max77696_read_reg_bit(uic->io, DEVICEID, CHIPID , &chipid );
    max77696_read_reg_bit(uic->io, DEVICEID, CHIPREV, &chiprev);
    dev_info(uic->dev, "ID 0x%04X Revision 0x%04X\n", chipid, chiprev);

    uic->pdata = max77696_uic_get_platdata(uic);
    if (unlikely(IS_ERR(uic->pdata))) {
        rc = PTR_ERR(uic->pdata);
        uic->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Device initialization */
    max77696_uic_setup(uic);

    /* Get UIC block IRQ number */
    uic->irq = max77696_get_block_irq(dev->parent, UIC);
    BUG_ON(uic->irq <= 0);

    /* Request system IRQ for UIC */
    rc = devm_request_threaded_irq(dev, (unsigned int)uic->irq, NULL,
        max77696_uic_isr, IRQF_ONESHOT, DRIVER_NAME, uic);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to request IRQ(%d) [%d]\n", uic->irq, rc);
        uic->irq = 0;
        goto abort;
    }

    dev_dbg(dev, "IRQ(%d) requested\n", uic->irq);

    disable_irq(uic->irq);

    /* Create max77696-uic sysfs attributes */
    uic->attr_grp = &max77696_uic_attr_group;
    rc = sysfs_create_group(uic->kobj, uic->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        uic->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);

    /* Enable using UIC interrupts */
    uicint  = INT1_CHGTYP;
    max77696_uic_enable_irq(uic, uicint);
    return 0;

abort:
    max77696_uic_destroy(uic);
    return rc;
}

static __devexit int max77696_uic_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_uic *uic = dev_get_drvdata(dev);

    max77696_uic_destroy(uic);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_uic_suspend (struct device *dev)
{
    return 0;
}

static int max77696_uic_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_uic_pm,
    max77696_uic_suspend, max77696_uic_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_uic_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_uic_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_uic_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_uic_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_uic_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_uic_probe,
    .remove                = __devexit_p(max77696_uic_remove),
};

static __init int max77696_uic_driver_init (void)
{
    return platform_driver_register(&max77696_uic_driver);
}
module_init(max77696_uic_driver_init);

static __exit void max77696_uic_driver_exit (void)
{
    platform_driver_unregister(&max77696_uic_driver);
}
module_exit(max77696_uic_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
