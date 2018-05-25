/*
 * MAX77696 ADC Driver
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

#include <linux/regulator/consumer.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    MAX77696_DESC" ADC Driver"
#define DRIVER_NAME    MAX77696_ADC_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define ADC_PRINT_RAW              MAX77696_ADC_PRINT_RAW
#define ADC_PRINT_SCALED           MAX77696_ADC_PRINT_SCALED
#define ADC_PRINT_FULL             MAX77696_ADC_PRINT_FULL

#define ADC_NCH                    MAX77696_NUM_OF_ADC_CHS
#define ADC_CONV_TIMEOUT           msecs_to_jiffies(10)
#define ADC_RESOLUTION             12
#define ADC_RESOLUTION_MASK        ((1 << ADC_RESOLUTION) - 1)

#define ADCCNTL                    0x26
#define ADCCNTL_ADCCONV            BIT (4)
#define ADCCNTL_ADCAVG             BITS(3,2)
#define ADCCNTL_ADCREFEN           BIT (1)
#define ADCCNTL_ADCEN              BIT (0)

#define ADCDLY                     0x27
#define ADCDLY_ADCDLY              BITS(4,0)

#define ADCSEL0                    0x28
#define ADCSEL1                    0x29

#define ADCCHSEL                   0x2A
#define ADCCHSEL_ADCCH             BITS(4,0)

#define ADCDATAL                   0x2B
#define ADCDATAH                   0x2C

#define ADCINT                     0x2E
#define ADCINTM                    0x2F

#define ADCICNFG                   0x30
#define ADCICNFG_IADCMUX           BITS(3,2)
#define ADCICNFG_IADC              BITS(1,0)

#define DEGREE_SIGN_UTF8           "\xC2\xB0"
#define DEGREE_SIGN                DEGREE_SIGN_UTF8

#define OHM_SIGN_UTF8              "\xE2\x84\xA6"
#define OHM_SIGN                   OHM_SIGN_UTF8

#define UNIT_EMPTY                 ""
#define UNIT_TEMP                  DEGREE_SIGN"C"
#define UNIT_VOLT                  "V"
#define UNIT_AMPS                  "A"
#define UNIT_OHMS                  OHM_SIGN

struct max77696_adc {
    struct mutex                        lock;
    struct max77696_adc_platform_data  *pdata;
    struct max77696_core               *core;
    struct max77696_io                 *io;
    struct device                      *dev;
    struct kobject                     *kobj;
    const struct attribute_group       *attr_grp;

    struct regulator                   *vref;

    struct device                      *hwmon;
    u8                                  print_fmt;
};

#define __lock(_me)        mutex_lock(&(_me)->lock)
#define __unlock(_me)      mutex_unlock(&(_me)->lock)

#define __msleep(_msec)    msleep_interruptible(_msec)

/*** ADC channel declaration ***/

struct max77696_adc_channel_desc {
    char                    *name;
    u8                       phy_ch;
    char                    *unit_str;

    /* Conversion Formula:
     *   MILLI-SCALED = (multiple * RAW + offset) / (10 ^ tens_place)
     */
    s32                      offset;
    s32                      multiple;
    s32                      tens_place;

    /* Common BitDesc */
    struct max77696_bitdesc  conv_req;
    struct max77696_bitdesc  data_h, data_l;

    int (*setup) (struct max77696_adc *adc, u8 ch);
    int (*release) (struct max77696_adc *adc, u8 ch);
};

#define max77696_adc_channel_setup_null   NULL
#define max77696_adc_channel_release_null NULL

#ifdef CONFIG_REGULATOR_MAX77696
static
int max77696_adc_channel_setup_imon_buck (struct max77696_adc *adc, u8 ch)
{
    u8 buck = (u8)(ch - MAX77696_ADC_CH_IMONB1);
    return max77696_enable_buck_imon(adc->dev->parent, buck, true);
}

static
int max77696_adc_channel_release_imon_buck (struct max77696_adc *adc, u8 ch)
{
    u8 buck = (u8)(ch - MAX77696_ADC_CH_IMONB1);
    return max77696_enable_buck_imon(adc->dev->parent, buck, false);
}

static
int max77696_adc_channel_setup_imon_ldo (struct max77696_adc *adc, u8 ch)
{
    u8 ldo = (u8)(ch - MAX77696_ADC_CH_IMONL1);
    return max77696_enable_ldo_imon(adc->dev->parent, ldo, true);
}

static
int max77696_adc_channel_release_imon_ldo (struct max77696_adc *adc, u8 ch)
{
    u8 ldo = (u8)(ch - MAX77696_ADC_CH_IMONL1);
    return max77696_enable_ldo_imon(adc->dev->parent, ldo, false);
}
#endif /* CONFIG_REGULATOR_MAX77696 */

#define ADC_CHANNEL_DESC(_ch) (&max77696_adc_channel_descs[_ch])
static struct max77696_adc_channel_desc max77696_adc_channel_descs[] =
{
    #define ADC_CHANNEL(_id, _phys, _ofst, _n, _m, _unit, _gate) \
            [MAX77696_ADC_CH_##_id] =\
                {\
                    .name       = #_id,\
                    .phy_ch     = _phys,\
                    .unit_str   = UNIT_##_unit,\
                    .offset     = _ofst,\
                    .multiple   = _n,\
                    .tens_place = _m,\
                    .conv_req   = MAX77696_BITDESC(ADCCNTL , ADCCNTL_ADCCONV),\
                    .data_h     = MAX77696_BITDESC(ADCDATAH, ~0             ),\
                    .data_l     = MAX77696_BITDESC(ADCDATAL, ~0             ),\
                    .setup      = max77696_adc_channel_setup_##_gate,\
                    .release    = max77696_adc_channel_release_##_gate,\
                }

    #define LSB(_n, _d)  DIV_ROUND_UP(_n, _d)

    #define kilo         ( 3)
    #define milli        (-3)
    #define micro        (-6)
    #define nano         (-9)

    /* FORMULA uV = 8192000 * code / 4095
     *   -> LSB = 8192000 / 4095
     */
    ADC_CHANNEL(VSYS2  , 0, 0, LSB(8192000, 4095), micro, VOLT, null     ),
    ADC_CHANNEL(VCHGINA, 4, 0, LSB(8192000, 4095), micro, VOLT, null     ),

    /* FORMULA uV = 2500000 * code / 4095
     *   -> LSB = 2500000 / 4095
     */
#ifdef CONFIG_REGULATOR_MAX77696
    ADC_CHANNEL(IMONL1 , 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL2 , 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL3 , 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    ADC_CHANNEL(IMONL4 , 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL5 , 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL6 , 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL7 , 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL8 , 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL9 , 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL10, 6, 0, LSB(2500000, 4095), micro, VOLT, imon_ldo ),
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
    ADC_CHANNEL(IMONB1 , 7, 0, LSB(2500000, 4095), micro, VOLT, imon_buck),
    ADC_CHANNEL(IMONB2 , 7, 0, LSB(2500000, 4095), micro, VOLT, imon_buck),
    ADC_CHANNEL(IMONB3 , 8, 0, LSB(2500000, 4095), micro, VOLT, imon_buck),
    ADC_CHANNEL(IMONB4 , 8, 0, LSB(2500000, 4095), micro, VOLT, imon_buck),
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    ADC_CHANNEL(IMONB5 ,10, 0, LSB(2500000, 4095), micro, VOLT, imon_buck),
    ADC_CHANNEL(IMONB6 ,10, 0, LSB(2500000, 4095), micro, VOLT, imon_buck),
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
#endif /* CONFIG_REGULATOR_MAX77696 */
    ADC_CHANNEL(AIN0   , 9, 0, LSB(2500000, 4095), micro, VOLT, null     ),
    ADC_CHANNEL(AIN1   ,11, 0, LSB(2500000, 4095), micro, VOLT, null     ),
    ADC_CHANNEL(AIN2   ,12, 0, LSB(2500000, 4095), micro, VOLT, null     ),
    ADC_CHANNEL(AIN3   ,13, 0, LSB(2500000, 4095), micro, VOLT, null     ),

    /* FORMULA uV = 5120000 * code / 4095
     *   -> LSB = 5120000 / 4095
     */
    ADC_CHANNEL(VSYS1  , 3, 0, LSB(5120000, 4095), micro, VOLT, null     ),

    /* FORMULA uA = 2580000 * code / 4095
     *   -> LSB = 2580000 / 4095
     */
    ADC_CHANNEL(ICHGINA, 5, 0, LSB(2580000, 4095), micro, AMPS, null     ),

    /* FORMULA mC = (2500 * code / 4095 - 0.7527) / 2.79e-3
     *   -> LSB    =  2500   / 4095 / 2.79e-3 =     218.817423118498387315592...
     *   -> offset = - 752.7        / 2.79e-3 = -269784.946236559139784946237...
     */
    ADC_CHANNEL(TDIE, 1, -269785, 219, milli, TEMP, null),
};

#define __adc_ch_desc(_ch) \
        (&max77696_adc_channel_descs[_ch])

static __always_inline
s32 max77696_adc_calc_milli_scaled (struct max77696_adc *adc, u8 ch, u16 raw)
{
    struct max77696_adc_channel_desc *ch_desc = __adc_ch_desc(ch);
    s32 milli_scaled, tens_place;

    tens_place   = ch_desc->tens_place;
    milli_scaled = (s32)raw * ch_desc->multiple + ch_desc->offset;

    for ( ; tens_place < -3; tens_place += 3) {
        milli_scaled = DIV_ROUND_UP(milli_scaled, 1000);
    }

    for ( ; tens_place > -3; tens_place--) {
        milli_scaled *= 10;
    }

    return milli_scaled;
}

static __always_inline
u16 max77696_adc_calc_raw (struct max77696_adc *adc, u8 ch, s32 milli_scaled)
{
    struct max77696_adc_channel_desc *ch_desc = __adc_ch_desc(ch);
    s32 raw, tens_place;

    tens_place = ch_desc->tens_place;

    for ( ; tens_place > -3; tens_place -= 3) {
        milli_scaled = DIV_ROUND_UP(milli_scaled, 1000);
    }

    for ( ; tens_place < -3; tens_place++) {
        milli_scaled *= 10;
    }

    if (unlikely(milli_scaled <= ch_desc->offset)) {
        return 0;
    }

    raw = DIV_ROUND_UP(milli_scaled - ch_desc->offset, ch_desc->multiple);

    return (u16)(raw >= ADC_RESOLUTION_MASK ? ADC_RESOLUTION_MASK : raw);
}

static __always_inline
int max77696_adc_enable_vref (struct max77696_adc *adc, bool en)
{
    int rc;

    if (unlikely(!adc->pdata->vref_name)) {
        dev_vdbg(adc->dev, "no external Vref control\n");
        rc = 0;
        goto out;
    }

    if (unlikely(!adc->vref)) {
        adc->vref = devm_regulator_get(adc->dev, adc->pdata->vref_name);
        if (unlikely(!adc->vref || IS_ERR(adc->vref))) {
            rc = adc->vref ? PTR_ERR(adc->vref) : -ENOMEM;
            dev_err(adc->dev, "failed to get Vref regulator [%d]\n", rc);
            adc->vref = NULL;
            goto out;
        }
    }

    rc = en ? regulator_enable(adc->vref) : regulator_disable(adc->vref);
    if (unlikely(rc)) {
        dev_err(adc->dev,
            "failed to %s Vref [%d]\n", en ? "enable" : "disable", rc);
        goto out;
    }

out:
    return rc;
}

static __always_inline
int max77696_adc_channel_setup_conv (struct max77696_adc *adc, u8 ch)
{
    struct max77696_adc_channel_desc *ch_desc = __adc_ch_desc(ch);
    u8 adcsel[2];
    int rc;

    /* Enable ADC by setting the ADC Enable (ADCEN) bit to 1
     * which in turn forces the ADC reference on
     * even if ADCREFEN bit is set to 0.
     */
    rc = max77696_write_reg_bit(adc->io, ADCCNTL, ADCEN, true);
    if (unlikely(rc)) {
        dev_err(adc->dev, "ADCCNTL write error [%d]\n", rc);
        goto out;
    }

    /* Select ADC ch to be converted. */

    ch_desc->phy_ch = ch_desc->phy_ch;

    if (ch_desc->phy_ch > 7) {
        adcsel[0] = 0;
        adcsel[1] = (1 << (ch_desc->phy_ch - 8));
    } else {
        adcsel[0] = (1 << (ch_desc->phy_ch    ));
        adcsel[1] = 0;
    }

    dev_vdbg(adc->dev, "ADCSEL0 0x%02X ADCSEL1 0x%02X\n", adcsel[0], adcsel[1]);

    rc = max77696_bulk_write(adc->io, ADCSEL0, adcsel, 2);
    if (unlikely(rc)) {
        dev_err(adc->dev, "ADCSEL write error [%d]\n", rc);
        goto out;
    }

    /* Setup ADC ch for conversion */
    if (likely(ch_desc->setup)) {
        rc = ch_desc->setup(adc, ch);
    }

out:
    return rc;
}

static __always_inline
void max77696_adc_channel_release_conv (struct max77696_adc *adc, u8 ch)
{
    struct max77696_adc_channel_desc *ch_desc = __adc_ch_desc(ch);

    if (likely(ch_desc->release)) {
        ch_desc->release(adc, ch);
    }
}

static int max77696_adc_channel_convert (struct max77696_adc *adc,
    u8 ch, u16 *raw, s32 *milli_scaled)
{
    struct max77696_adc_channel_desc *ch_desc = __adc_ch_desc(ch);
    unsigned long timeout;
    int rc;
    s32 _milli_scaled;
    u16 _raw, data_l = 0, data_h = 0, adc_busy;

    rc = max77696_adc_enable_vref(adc, true);
    if (unlikely(rc)) {
        dev_err(adc->dev, "failed to enable Vref for ADC %s [%d]\n",
            ch_desc->name, rc);
        goto out;
    }

    rc = max77696_adc_channel_setup_conv(adc, ch);
    if (unlikely(rc)) {
        dev_err(adc->dev, "failed to setup ADC %s [%d]\n", ch_desc->name, rc);
        goto disable_vref;
    }

    /* Initiate ADC conversion sequence
     * by setting the conversion request bit to 1.
     */
    rc = max77696_write_bitdesc(adc->io, &ch_desc->conv_req, true);
    if (unlikely(rc)) {
        dev_err(adc->dev, "failed to start ADC %s [%d]\n", ch_desc->name, rc);
        goto release_channel;
    }

    /* Check availability of ADC raw by inspecting the conversion request bit.
     * This bit is automatically cleared to 0 when an ADC conversion sequence
     * has completed.
     */

    timeout = jiffies + ADC_CONV_TIMEOUT;

    do {
        if (unlikely(time_after(jiffies, timeout))) {
            dev_err(adc->dev, "timeout to convert ADC %s\n", ch_desc->name);
            rc = -ETIME;
            goto release_channel;
        }
        __msleep(1);
        adc_busy = 0;
        max77696_read_bitdesc(adc->io, &ch_desc->conv_req, &adc_busy);
    } while (likely(adc_busy));

    /* Read ADC conversion result. */

    rc = max77696_write_reg_bit(adc->io, ADCCHSEL, ADCCH, ch_desc->phy_ch);
    if (unlikely(rc)) {
        dev_err(adc->dev, "ADCCHSEL write error [%d]\n", rc);
        goto release_channel;
    }

    rc = max77696_read_bitdesc(adc->io, &ch_desc->data_l, &data_l);
    if (unlikely(rc)) {
        dev_err(adc->dev, "DATAL read error [%d]\n", rc);
        goto release_channel;
    }

    rc = max77696_read_bitdesc(adc->io, &ch_desc->data_h, &data_h);
    if (unlikely(rc)) {
        dev_err(adc->dev, "DATAH read error [%d]\n", rc);
        goto release_channel;
    }

    data_h &= 0xFF;
    data_l &= 0xFF;
    dev_vdbg(adc->dev, "DATAH 0x%04X DATAL 0x%04X\n", data_h, data_l);

    _raw           = (data_h << 8) | data_l;
    _raw          &= ADC_RESOLUTION_MASK;
    _milli_scaled  = max77696_adc_calc_milli_scaled(adc, ch, _raw);

    if (likely(raw)) {
        *raw = _raw;
    }

    if (likely(milli_scaled)) {
        *milli_scaled = _milli_scaled;
    }

release_channel:
    max77696_adc_channel_release_conv(adc, ch);
disable_vref:
    max77696_adc_enable_vref(adc, false);
out:
    return rc;
}

static __always_inline int max77696_adc_setup (struct max77696_adc *adc)
{
    adc->print_fmt = adc->pdata->print_fmt;
    return 0;
}

static ssize_t max77696_adc_channel_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_adc *adc = dev_get_drvdata(dev);
    struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
    u8 ch;
    u16 raw = 0;
    s32 milli_scaled_i = 0, milli_scaled_f = 0;
    int rc;

    __lock(adc);

    ch = (u8)attr->index;

    rc = max77696_adc_channel_convert(adc, ch, &raw, &milli_scaled_i);
    if (unlikely(rc)) {
        goto out;
    }

    milli_scaled_f = (milli_scaled_i % 1000);
    milli_scaled_i = (milli_scaled_i / 1000);

    if (likely(adc->print_fmt & ADC_PRINT_RAW)) {
        rc += (int)snprintf(buf+rc, PAGE_SIZE, "%u\n", raw);
    }

    if (likely(adc->print_fmt & ADC_PRINT_SCALED)) {
        rc += (int)snprintf(buf+rc, PAGE_SIZE, "%d\n",
            milli_scaled_i + (milli_scaled_f >= 500));
    }

    if (likely(adc->print_fmt & ADC_PRINT_FULL)) {
        rc += (int)snprintf(buf+rc, PAGE_SIZE, "%d.%03d %s\n",
            milli_scaled_i, milli_scaled_f, __adc_ch_desc(ch)->unit_str);
    }

out:
    __unlock(adc);
    return (ssize_t)rc;
}

#define DEFINE_ADC_CH_DEV_ATTR(_name, _ch) \
        SENSOR_DEVICE_ATTR(_name, S_IRUGO,\
            max77696_adc_channel_show, NULL, MAX77696_ADC_CH_##_ch)

static DEFINE_ADC_CH_DEV_ATTR(vsys2  , VSYS2  );
static DEFINE_ADC_CH_DEV_ATTR(tdie   , TDIE   );
static DEFINE_ADC_CH_DEV_ATTR(vsys1  , VSYS1  );
static DEFINE_ADC_CH_DEV_ATTR(vchgina, VCHGINA);
static DEFINE_ADC_CH_DEV_ATTR(ichgina, ICHGINA);
#ifdef CONFIG_REGULATOR_MAX77696
static DEFINE_ADC_CH_DEV_ATTR(imonl1 , IMONL1 );
static DEFINE_ADC_CH_DEV_ATTR(imonl2 , IMONL2 );
static DEFINE_ADC_CH_DEV_ATTR(imonl3 , IMONL3 );
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
static DEFINE_ADC_CH_DEV_ATTR(imonl4 , IMONL4 );
static DEFINE_ADC_CH_DEV_ATTR(imonl5 , IMONL5 );
static DEFINE_ADC_CH_DEV_ATTR(imonl6 , IMONL6 );
static DEFINE_ADC_CH_DEV_ATTR(imonl7 , IMONL7 );
static DEFINE_ADC_CH_DEV_ATTR(imonl8 , IMONL8 );
static DEFINE_ADC_CH_DEV_ATTR(imonl9 , IMONL9 );
static DEFINE_ADC_CH_DEV_ATTR(imonl10, IMONL10);
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
static DEFINE_ADC_CH_DEV_ATTR(imonb1 , IMONB1 );
static DEFINE_ADC_CH_DEV_ATTR(imonb2 , IMONB2 );
static DEFINE_ADC_CH_DEV_ATTR(imonb3 , IMONB3 );
static DEFINE_ADC_CH_DEV_ATTR(imonb4 , IMONB4 );
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
static DEFINE_ADC_CH_DEV_ATTR(imonb5 , IMONB5 );
static DEFINE_ADC_CH_DEV_ATTR(imonb6 , IMONB6 );
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
#endif /* CONFIG_REGULATOR_MAX77696 */
static DEFINE_ADC_CH_DEV_ATTR(ain0   , AIN0   );
static DEFINE_ADC_CH_DEV_ATTR(ain1   , AIN1   );
static DEFINE_ADC_CH_DEV_ATTR(ain2   , AIN2   );
static DEFINE_ADC_CH_DEV_ATTR(ain3   , AIN3   );

#define ADC_CH_DEV_ATTR(_name) \
        (&sensor_dev_attr_##_name.dev_attr.attr)

#define ADC_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_adc_attr[] = {
    ADC_CH_DEV_ATTR(vsys2  ),
    ADC_CH_DEV_ATTR(tdie   ),
    ADC_CH_DEV_ATTR(vsys1  ),
    ADC_CH_DEV_ATTR(vchgina),
    ADC_CH_DEV_ATTR(ichgina),
#ifdef CONFIG_REGULATOR_MAX77696
    ADC_CH_DEV_ATTR(imonl1 ),
    ADC_CH_DEV_ATTR(imonl2 ),
    ADC_CH_DEV_ATTR(imonl3 ),
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    ADC_CH_DEV_ATTR(imonl4 ),
    ADC_CH_DEV_ATTR(imonl5 ),
    ADC_CH_DEV_ATTR(imonl6 ),
    ADC_CH_DEV_ATTR(imonl7 ),
    ADC_CH_DEV_ATTR(imonl8 ),
    ADC_CH_DEV_ATTR(imonl9 ),
    ADC_CH_DEV_ATTR(imonl10),
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
    ADC_CH_DEV_ATTR(imonb1 ),
    ADC_CH_DEV_ATTR(imonb2 ),
    ADC_CH_DEV_ATTR(imonb3 ),
    ADC_CH_DEV_ATTR(imonb4 ),
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    ADC_CH_DEV_ATTR(imonb5 ),
    ADC_CH_DEV_ATTR(imonb6 ),
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
#endif /* CONFIG_REGULATOR_MAX77696 */
    ADC_CH_DEV_ATTR(ain0   ),
    ADC_CH_DEV_ATTR(ain1   ),
    ADC_CH_DEV_ATTR(ain2   ),
    ADC_CH_DEV_ATTR(ain3   ),

    NULL
};

static const struct attribute_group max77696_adc_attr_group = {
    .attrs = max77696_adc_attr,
};

static void *max77696_adc_get_platdata (struct max77696_adc *adc)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_adc_platform_data *pdata;
    struct device *dev = adc->dev;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*pdata));
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->print_fmt = MAX77696_ADC_PRINT_ALL;
    of_property_u8(np, "print_fmt", &pdata->print_fmt);

    pdata->vref_name = NULL;
    of_property_read_string(np, "vref_name", (char const**)&pdata->vref_name);
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "PRINT FMT", "0x%02X", pdata->print_fmt);
    __prop_printk(dev, "VREF SUPPLY", "%s",
        pdata->vref_name ? pdata->vref_name : "null");

out:
    return pdata;
}

static __always_inline void max77696_adc_destroy (struct max77696_adc *adc)
{
    struct device *dev = adc->dev;

    if (likely(adc->attr_grp)) {
        sysfs_remove_group(adc->kobj, adc->attr_grp);
    }

    if (likely(adc->hwmon)) {
        hwmon_device_unregister(adc->hwmon);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(adc->pdata)) {
        devm_kfree(dev, adc->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&adc->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, adc);
}

static __devinit int max77696_adc_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_adc *adc;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    adc = devm_kzalloc(dev, sizeof(*adc), GFP_KERNEL);
    if (unlikely(!adc)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*adc));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, adc);

    mutex_init(&adc->lock);
    adc->core  = core;
    adc->io    = max77696_get_block_io(dev->parent, ADC);
    adc->dev   = dev;
    adc->kobj  = &dev->kobj;

    adc->pdata = max77696_adc_get_platdata(adc);
    if (unlikely(IS_ERR(adc->pdata))) {
        rc = PTR_ERR(adc->pdata);
        adc->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Device initialization */
    max77696_adc_setup(adc);

    /* Register adc to HWMON class */
    adc->hwmon = hwmon_device_register(dev);
    if (unlikely(IS_ERR(adc->hwmon))) {
        rc = PTR_ERR(adc->hwmon);
        dev_err(dev, "failed to register hwmon device [%d]\n", rc);
        adc->hwmon = NULL;
        goto abort;
    }

    /* Create max77696-adc sysfs attributes */
    adc->attr_grp = &max77696_adc_attr_group;
    rc = sysfs_create_group(adc->kobj, adc->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        adc->attr_grp = NULL;
        goto abort;
    }

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_adc_destroy(adc);
    return rc;
}

static __devexit int max77696_adc_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_adc *adc = dev_get_drvdata(dev);

    max77696_adc_destroy(adc);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_adc_suspend (struct device *dev)
{
    return 0;
}

static int max77696_adc_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_adc_pm,
    max77696_adc_suspend, max77696_adc_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_adc_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_adc_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_adc_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_adc_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_adc_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_adc_probe,
    .remove                = __devexit_p(max77696_adc_remove),
};

static __init int max77696_adc_driver_init (void)
{
    return platform_driver_register(&max77696_adc_driver);
}
module_init(max77696_adc_driver_init);

static __exit void max77696_adc_driver_exit (void)
{
    platform_driver_unregister(&max77696_adc_driver);
}
module_exit(max77696_adc_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
