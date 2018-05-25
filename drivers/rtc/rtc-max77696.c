/*
 * MAX77696 Real Time Clock Driver
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

#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    MAX77696_DESC" Real Time Clock Driver"
#define DRIVER_NAME    MAX77696_RTC_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

#define RTC_RWC_INTERRUPT           1

#define RTC_UPDATE_TIMEOUT          msecs_to_jiffies(10)
#define RTC_MASTER                  0

#define RTC_YEAR_BASE               2000
#define RTC_YEAR_OFFSET             (RTC_YEAR_BASE - 1900)

#define RTCINT                      0x00
#define RTCINTM                     0x01

#define RTCINT_WTSR                 BIT (5)
#define RTCINT_RTC1S                BIT (4)
#define RTCINT_SMPL                 BIT (3)
#define RTCINT_RTCA2                BIT (2)
#define RTCINT_RTCA1                BIT (1)
#define RTCINT_RTC60S               BIT (0)

#define RTCCNTLM                    0x02
#define RTCCNTLM_HRMODEM            BIT (1)
#define RTCCNTLM_BCDM               BIT (0)

#define RTCCNTL                     0x03
#define RTCCNTL_HRMODE              BIT (1)
#define RTCCNTL_BCD                 BIT (0)

#define RTCUPDATE0                  0x04
#define RTCUPDATE0_RBUDR            BIT (4)
#define RTCUPDATE0_RTCWAKE          BIT (3)
#define RTCUPDATE0_FREEZE_SEC       BIT (2)
#define RTCUPDATE0_UDR              BIT (0)

#define RTCSMPL                     0x06
#define RTCSMPL_SMPLEN              BIT (7)
#define RTCSMPL_WTSR                BIT (6)
#define RTCSMPL_SMPLT               BITS(3,2)
#define RTCSMPL_WTSRT               BITS(1,0)

#define RTCSEC                      0x07
#define RTCMIN                      0x08
#define RTCHOUR                     0x09
#define RTCDOW                      0x0A
#define RTCMONTH                    0x0B
#define RTCYEAR                     0x0C
#define RTCDOM                      0x0D

#define RTCAE1                      0x0E
#define RTCSECA1                    0x0F
#define RTCMINA1                    0x10
#define RTCHOURA1                   0x11
#define RTCDOWA1                    0x12
#define RTCMONTHA1                  0x13
#define RTCYEARA1                   0x14
#define RTCDOMA1                    0x15

#define RTCAE2                      0x16
#define RTCSECA2                    0x17
#define RTCMINA2                    0x18
#define RTCHOURA2                   0x19
#define RTCDOWA2                    0x1A
#define RTCMONTHA2                  0x1B
#define RTCYEARA2                   0x1C
#define RTCDOMA2                    0x1D

#define RTCHOUR_AMPM_MASK           BIT (6)
#define RTCHOUR_AM                  0x00
#define RTCHOUR_PM                  0x40

#define RTCAE_SEC                   BIT (0)
#define RTCAE_MIN                   BIT (1)
#define RTCAE_HOUR                  BIT (2)
#define RTCAE_DOW                   BIT (3)
#define RTCAE_MONTH                 BIT (4)
#define RTCAE_YEAR                  BIT (5)
#define RTCAE_DOM                   BIT (6)
#define RTCAE_ALL                   0x7F

#define RTC_HRMODE_12H              0
#define RTC_HRMODE_24H              1

#define RTC_BCD_OFF                 0
#define RTC_BCD_ON                  1

/* Default mode of RTC data registers */
#define RTC_BCD_MODE                RTC_BCD_OFF
#define RTC_HOUR_MODE               RTC_HRMODE_12H
#define RTC_DATA_MODE               (RTC_BCD_MODE|RTC_HOUR_MODE)

struct max77696_rtc {
    struct mutex                       lock;
    struct max77696_rtc_platform_data *pdata;
    struct max77696_core              *core;
    struct max77696_io                *io;
    struct device                     *dev;
    struct kobject                    *kobj;
    const struct attribute_group      *attr_grp;

    struct rtc_device                 *rtc_dev[2];
    struct rtc_device                 *rtc_master;

    int                                irq;
    u16                                irq_unmask;
};

#define __lock(_me)      mutex_lock(&(_me)->lock)
#define __unlock(_me)    mutex_unlock(&(_me)->lock)

#define __msleep(_msec)  msleep_interruptible(_msec)

#define RTC_SEC_MASK   (0x7F)
#define RTC_MIN_MASK   (0x7F)
#define RTC_HOUR_MASK  (0x3F)
#define RTC_DOW_MASK   (0x7F)
#define RTC_MONTH_MASK (0x1F)
#define RTC_YEAR_MASK  (0xFF)
#define RTC_DOM_MASK   (0x3F)

struct max77696_rtc_time_reg {
    u8 sec;
    u8 min;
    u8 hour;
    u8 dow;
    u8 month;
    u8 year;
    u8 dom;
};

#define RTC_TIME_REG     RTCSEC
#define RTC_TIME_REG_A1  RTCSECA1
#define RTC_TIME_REG_A2  RTCSECA2

static char *dow_short[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

/* Convert MAX77696 RTC time register --> struct rtc_time */
static void max77696_rtc_format_reg2time (struct max77696_rtc *rtc,
    const struct max77696_rtc_time_reg *tm_reg, struct rtc_time *tm)
{
    u8 dow = tm_reg->dow & RTC_DOW_MASK;

    tm->tm_year = (int)(tm_reg->year & RTC_YEAR_MASK) + RTC_YEAR_OFFSET;
    tm->tm_mon  = (int)(tm_reg->month & RTC_MONTH_MASK) - 1;
    tm->tm_mday = (int)(tm_reg->dom & RTC_DOM_MASK);
    tm->tm_wday = (int)(dow ? (__fls(dow)) : 0);
    tm->tm_hour = (int)(tm_reg->hour & RTC_HOUR_MASK);
    tm->tm_min  = (int)(tm_reg->min & RTC_MIN_MASK);
    tm->tm_sec  = (int)(tm_reg->sec & RTC_SEC_MASK);

    if (RTC_HOUR_MODE == RTC_HRMODE_12H) {
        tm->tm_hour %= 12;
        tm->tm_hour += ((tm_reg->hour & RTCHOUR_AMPM_MASK)? 12 : 0);
    }
}

/* Convert struct rtc_time --> MAX77696 RTC registers */
static void max77696_rtc_format_time2reg (struct max77696_rtc *rtc,
    const struct rtc_time *tm, struct max77696_rtc_time_reg *tm_reg)
{
    tm_reg->year  = (u16)(tm->tm_year - RTC_YEAR_OFFSET) & RTC_YEAR_MASK;
    tm_reg->month = (u16)(tm->tm_mon + 1) & RTC_MONTH_MASK;
    tm_reg->dom   = (u16)(tm->tm_mday) & RTC_DOM_MASK;
    tm_reg->dow   = (u16)(1 << tm->tm_wday) & RTC_DOW_MASK;
    tm_reg->hour  = (u16)(tm->tm_hour) & RTC_HOUR_MASK;
    tm_reg->min   = (u16)(tm->tm_min) & RTC_MIN_MASK;
    tm_reg->sec   = (u16)(tm->tm_sec) & RTC_SEC_MASK;

    if (RTC_HOUR_MODE == RTC_HRMODE_12H) {
        if (tm->tm_hour > 12) {
            tm_reg->hour |= RTCHOUR_PM;
            tm_reg->hour -= 12;
        } else if (tm->tm_hour == 12) {
            tm_reg->hour |= RTCHOUR_PM;
        }
    }
}

static __always_inline
void max77696_rtc_write_irq_mask (struct max77696_rtc *rtc)
{
    u16 rtcintm;
    int rc;

    rtcintm = ~rtc->irq_unmask;

    rc = max77696_write(rtc->io, RTCINTM, rtcintm);
    dev_dbg(rtc->dev, "written RTCINTM 0x%04X [%d]\n", rtcintm, rc);

    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCINTM write error [%d]\n", rc);
    }
}

static __always_inline
void max77696_rtc_enable_irq (struct max77696_rtc *rtc, u16 irq_bits)
{
    if (unlikely((rtc->irq_unmask & irq_bits) == irq_bits)) {
        /* already unmasked or null bit */
        return;
    }

    if (unlikely(!rtc->irq_unmask)) {
        enable_irq(rtc->irq);
        enable_irq_wake(rtc->irq);
    }

    /* set enabled flag */
    rtc->irq_unmask |= irq_bits;

    /* write irq mask */
    max77696_rtc_write_irq_mask(rtc);
}

static __always_inline
void max77696_rtc_disable_irq (struct max77696_rtc *rtc, u16 irq_bits)
{
    if (unlikely((rtc->irq_unmask & irq_bits) == 0)) {
        /* already masked or null bit */
        return;
    }

    /* clear enabled flag */
    rtc->irq_unmask &= ~irq_bits;

    if (unlikely(!rtc->irq_unmask)) {
        disable_irq_wake(rtc->irq);
        disable_irq_nosync(rtc->irq);
    }

    /* write irq mask */
    max77696_rtc_write_irq_mask(rtc);
}

static __always_inline u8 max77696_rtc_read_irq (struct max77696_rtc *rtc)
{
    u16 rtcint;
    int rc;

    rc = max77696_read(rtc->io, RTCINT, &rtcint);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCINT read error [%d]\n", rc);
        return 0;
    }

    return rtcint;
}

static __always_inline void max77696_rtc_ack_irq (struct max77696_rtc *rtc)
{
    if (RTC_RWC_INTERRUPT) {
        max77696_write(rtc->io, RTCINT, ~0);
    }
}

#define max77696_rtc_init_data_mode(_rtc) \
        max77696_write(_rtc->io, RTCCNTL, RTC_DATA_MODE)

static __always_inline int max77696_rtc_unfreeze_sec (struct max77696_rtc *rtc)
{
    u16 rtcupdate0 = 0;
    int rc;

    rc = max77696_read(rtc->io, RTCUPDATE0, &rtcupdate0);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCUPDATE0 read error [%d]\n", rc);
        goto out;
    }

    if (unlikely(rtcupdate0 & RTCUPDATE0_FREEZE_SEC)) {
        rc = max77696_write(rtc->io, RTCUPDATE0,
            rtcupdate0 & ~RTCUPDATE0_FREEZE_SEC);
        dev_err(rtc->dev, "FREEZE_SEC cleared [%d]\n", rc);
    }

out:
    return rc;
}

static int max77696_rtc_sync_read_buffer (struct max77696_rtc *rtc)
{
    unsigned long timeout;
    u16 rbudr;
    int rc;

    rc = max77696_write_reg_bit(rtc->io, RTCUPDATE0, RBUDR, true);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCUPDATE0 write error [%d]\n", rc);
        goto out;
    }

    timeout = jiffies + RTC_UPDATE_TIMEOUT;

    do {
        if (unlikely(time_after(jiffies, timeout))) {			
            dev_err(rtc->dev, "time out to xfer read buffer\n");
            rc = -ETIME;
            goto out;
        }

        __msleep(1);

        rbudr = 1;
        max77696_read_reg_bit(rtc->io, RTCUPDATE0, RBUDR, &rbudr);
    } while (likely(rbudr));

out:
    return rc;
}

static int max77696_rtc_commit_write_buffer (struct max77696_rtc *rtc)
{
    unsigned long timeout;
    u16 udr;
    int rc;

    rc = max77696_write_reg_bit(rtc->io, RTCUPDATE0, UDR, true);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCUPDATE0 write error [%d]\n", rc);
        goto out;
    }

    timeout = jiffies + RTC_UPDATE_TIMEOUT;

    do {
        if (unlikely(time_after(jiffies, timeout))) {
            dev_err(rtc->dev, "time out to xfer write buffer\n");
            rc = -ETIME;
            goto out;
        }

        __msleep(1);

        udr = 1;
        max77696_read_reg_bit(rtc->io, RTCUPDATE0, UDR, &udr);
    } while (likely(udr));

out:
    return rc;
}

#define max77696_rtc_dump_rtc_time(_rtc, _type, _tm) \
        dev_info(_rtc->dev,\
            _type ": %04d-%02d-%02d %s %02d:%02d:%02d\n",\
            (_tm)->tm_year+1900, (_tm)->tm_mon+1, (_tm)->tm_mday,\
            dow_short[(_tm)->tm_wday],\
            (_tm)->tm_hour, (_tm)->tm_min, (_tm)->tm_sec)

static int max77696_rtc_read_time_reg (struct max77696_rtc *rtc,
    u16 reg, bool cached, struct rtc_time *tm)
{
    struct max77696_rtc_time_reg tm_reg;
    int rc;

    if (likely(cached)) {
        /* Sync "Read Buffers" */
        rc = max77696_rtc_sync_read_buffer(rtc);
        if (unlikely(rc)) {
            dev_err(rtc->dev, "failed to sync read buffer [%d]\n", rc);
            goto out;
        }
    }

    rc = max77696_bulk_read(rtc->io, reg, (void*)&tm_reg, sizeof(tm_reg));
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTC time registers read error [%d]\n", rc);
        goto out;
    }

    max77696_rtc_format_reg2time(rtc, &tm_reg, tm);

out:
    return rc;
}

static int max77696_rtc_write_time_reg (struct max77696_rtc *rtc,
    u16 reg, bool cached, const struct rtc_time *tm)
{
    struct max77696_rtc_time_reg tm_reg;
    int rc;

    max77696_rtc_format_time2reg(rtc, tm, &tm_reg);

    rc = max77696_rtc_init_data_mode(rtc);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "failed to initialize data mode [%d]\n", rc);
        goto out;
    }

    rc = max77696_bulk_write(rtc->io, reg, (void*)&tm_reg, sizeof(tm_reg));
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTC time registers write error [%d]\n", rc);
        goto out;
    }

    if (likely(cached)) {
        /* Commit "Write Buffers" */
        rc = max77696_rtc_commit_write_buffer(rtc);
        if (unlikely(rc)) {
            dev_err(rtc->dev, "failed to commit write buffer [%d]\n", rc);
            goto out;
        }
    }

    /* WORKAROUND: Clear FREEZE_SEC bit */
    max77696_rtc_unfreeze_sec(rtc);

out:
    return rc;
}

#define max77696_rtc_read_timekeeper_counters(_rtc, _tm) \
        max77696_rtc_read_time_reg(_rtc, RTC_TIME_REG, true, _tm)
#define max77696_rtc_write_timekeeper_counters(_rtc, _tm) \
        max77696_rtc_write_time_reg(_rtc, RTC_TIME_REG, true, _tm)

#define max77696_rtc_read_alarm0_time(_rtc, _tm) \
        max77696_rtc_read_time_reg(_rtc, RTC_TIME_REG_A1, false, _tm)
#define max77696_rtc_write_alarm0_time(_rtc, _tm) \
        max77696_rtc_write_time_reg(_rtc, RTC_TIME_REG_A1, false, _tm)

#define max77696_rtc_read_alarm1_time(_rtc, _tm) \
        max77696_rtc_read_time_reg(_rtc, RTC_TIME_REG_A2, false, _tm)
#define max77696_rtc_write_alarm1_time(_rtc, _tm) \
        max77696_rtc_write_time_reg(_rtc, RTC_TIME_REG_A2, false, _tm)

static int max77696_rtc_smplen_read (struct max77696_rtc *rtc, bool *en)
{
    u16 smplen;
    int rc;

    rc = max77696_read_reg_bit(rtc->io, RTCSMPL, SMPLEN, &smplen);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCSMPL read error [%d]\n", rc);
        goto out;
    }

    *en = (bool)(!!smplen);

out:
    return rc;
}

static int max77696_rtc_smplen_write (struct max77696_rtc *rtc, bool en)
{
    int rc;

    rc = max77696_write_reg_bit(rtc->io, RTCSMPL, SMPLEN, !!en);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCSMPL write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static int max77696_rtc_smplt_read (struct max77696_rtc *rtc, u32 *msec)
{
    u16 smplt;
    int rc;

    rc = max77696_read_reg_bit(rtc->io, RTCSMPL, SMPLT, &smplt);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCSMPL read error [%d]\n", rc);
        goto out;
    }

    *msec = ((u32)smplt + 1) * 500;

out:
    return rc;
}

static int max77696_rtc_smplt_write (struct max77696_rtc *rtc, u32 msec)
{
    u16 smplt;
    int rc;

    smplt = msec > 1500 ? 3 :
            msec > 1000 ? 2 :
            msec >  500 ? 1 : 0;

    rc = max77696_write_reg_bit(rtc->io, RTCSMPL, SMPLT, smplt);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCSMPL write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static __always_inline int max77696_rtc_setup (struct max77696_rtc *rtc)
{
    struct max77696_rtc_platform_data *pdata = rtc->pdata;

    /* Device wakeup initialization */
    device_init_wakeup(rtc->dev, true);

    if (likely(pdata->smpl_msec > 0)) {
        max77696_rtc_smplt_write (rtc, pdata->smpl_msec);
        max77696_rtc_smplen_write(rtc, true);
    } else {
        max77696_rtc_smplen_write(rtc, false);
    }

    return 0;
}

static irqreturn_t max77696_rtc_isr (int irq, void *data)
{
    struct max77696_rtc *rtc = data;
    u8 rtcint, rtc_events[2] = { 0, 0 };

    rtcint = max77696_rtc_read_irq(rtc);
    max77696_rtc_ack_irq(rtc);
    dev_dbg(rtc->dev, "RTCINT %02X EN %02X\n", rtcint, rtc->irq_unmask);

    #if 0 /* is it neccessary? */
    if (likely(rtcint & (RTCINT_RTCA1 | RTCINT_RTCA2))) {
        /* WORKAROUND: Clear FREEZE_SEC bit */
        max77696_rtc_unfreeze_sec(rtc);
    }
    #endif

    rtcint &= rtc->irq_unmask;

    if (rtcint & RTCINT_RTC60S) {
        rtc_events[RTC_MASTER] |= RTC_PF;
    }

    if (rtcint & RTCINT_RTCA1) {
        rtc_events[0] |= RTC_AF;
    }

    if (rtcint & RTCINT_RTCA2) {
        rtc_events[1] |= RTC_AF;
    }

    if (rtcint & RTCINT_RTC1S) {
        rtc_events[RTC_MASTER] |= RTC_UF;
    }

    if (likely(rtc_events[0])) {
        rtc_update_irq(rtc->rtc_dev[0], 1, RTC_IRQF | rtc_events[0]);
    }

    if (likely(rtc_events[1])) {
        rtc_update_irq(rtc->rtc_dev[1], 1, RTC_IRQF | rtc_events[1]);
    }

    return IRQ_HANDLED;
}

static int max77696_rtc_op_read_time (struct device *dev, struct rtc_time *tm)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    int rc;

    __lock(rtc);

    rc = max77696_rtc_read_timekeeper_counters(rtc, tm);
    if (unlikely(rc)) {
        goto out;
    }

    #ifdef DEBUG
    max77696_rtc_dump_rtc_time(rtc, "read_time", tm);
    #endif /* DEBUG */

out:
    __unlock(rtc);
    return rc;
}

static int max77696_rtc_op_set_time (struct device *dev, struct rtc_time *tm)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    int rc;

    __lock(rtc);

    #ifdef DEBUG
    max77696_rtc_dump_rtc_time(rtc, "write_time", tm);
    #endif /* DEBUG */

    rc = max77696_rtc_write_timekeeper_counters(rtc, tm);

    __unlock(rtc);
    return rc;
}

static int max77696_rtc_op_read_alarm0 (struct device *dev,
    struct rtc_wkalrm *alrm)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    struct rtc_time *tm = &(alrm->time);
    int rc;

    __lock(rtc);

    rc = max77696_rtc_read_alarm0_time(rtc, tm);
    if (unlikely(rc)) {
        goto out;
    }

    #ifdef DEBUG
    max77696_rtc_dump_rtc_time(rtc, "read_alarm0", tm);
    #endif /* DEBUG */

out:
    __unlock(rtc);
    return rc;
}

static int max77696_rtc_op_set_alarm0 (struct device *dev,
    struct rtc_wkalrm *alrm)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    struct rtc_time *tm = &(alrm->time);
    int rc = 0;

    __lock(rtc);

    /* Disable RTC alarm interrupt */
    max77696_rtc_disable_irq(rtc, RTCINT_RTCA1);

    if (unlikely(!alrm->enabled)) {
        goto out;
    }

    #ifdef DEBUG
    max77696_rtc_dump_rtc_time(rtc, "set_alarm0", tm);
    #endif /* DEBUG */

    /* Set new alaram */
    rc = max77696_rtc_write_alarm0_time(rtc, tm);
    if (unlikely(rc)) {
        goto out;
    }

    /* Enable alarm bits */
    rc = max77696_write(rtc->io, RTCAE1, RTCAE_ALL & ~RTCAE_DOW);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCAE1 write error [%d]\n", rc);
        goto out;
    }

    /* Enable RTC alarm interrupt */
    max77696_rtc_enable_irq(rtc, RTCINT_RTCA1);

out:
    __unlock(rtc);
    return rc;
}

static int max77696_rtc_op_alarm0_irq_enable (struct device *dev,
    unsigned int enabled)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);

    __lock(rtc);

    if (enabled) {
        dev_dbg(dev, "alarm0_irq enable\n");
        max77696_rtc_enable_irq (rtc, RTCINT_RTCA1);
    } else {
        dev_dbg(dev, "alarm0_irq disable\n");
        max77696_rtc_disable_irq(rtc, RTCINT_RTCA1);
    }

    __unlock(rtc);
    return 0;
}

static const struct rtc_class_ops max77696_rtc_ops0 = {
    .read_time        = max77696_rtc_op_read_time,
    .set_time         = max77696_rtc_op_set_time,
    .read_alarm       = max77696_rtc_op_read_alarm0,
    .set_alarm        = max77696_rtc_op_set_alarm0,
    .alarm_irq_enable = max77696_rtc_op_alarm0_irq_enable,
};

static int max77696_rtc_op_read_alarm1 (struct device *dev,
    struct rtc_wkalrm *alrm)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    struct rtc_time *tm = &(alrm->time);
    int rc;

    __lock(rtc);

    rc = max77696_rtc_read_alarm1_time(rtc, tm);
    if (unlikely(rc)) {
        goto out;
    }

    #ifdef DEBUG
    max77696_rtc_dump_rtc_time(rtc, "read_alarm1", tm);
    #endif /* DEBUG */

out:
    __unlock(rtc);
    return rc;
}

static int max77696_rtc_op_set_alarm1 (struct device *dev,
    struct rtc_wkalrm *alrm)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    struct rtc_time *tm = &(alrm->time);
    int rc = 0;

    __lock(rtc);

    /* Disable RTC alarm interrupt */
    max77696_rtc_disable_irq(rtc, RTCINT_RTCA2);

    if (unlikely(!alrm->enabled)) {
        goto out;
    }

    #ifdef DEBUG
    max77696_rtc_dump_rtc_time(rtc, "set_alarm1", tm);
    #endif /* DEBUG */

    /* Set new alaram */
    rc = max77696_rtc_write_alarm1_time(rtc, tm);
    if (unlikely(rc)) {
        goto out;
    }

    /* Enable alarm bits */
    rc = max77696_write(rtc->io, RTCAE2, RTCAE_ALL & ~RTCAE_DOW);
    if (unlikely(rc)) {
        dev_err(rtc->dev, "RTCAE1 write error [%d]\n", rc);
        goto out;
    }

    /* Enable RTC alarm interrupt */
    max77696_rtc_enable_irq(rtc, RTCINT_RTCA2);

out:
    __unlock(rtc);
    return rc;
}

static int max77696_rtc_op_alarm1_irq_enable (struct device *dev,
    unsigned int enabled)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);

    __lock(rtc);

    if (enabled) {
        dev_dbg(dev, "alarm1_irq enable\n");
        max77696_rtc_enable_irq (rtc, RTCINT_RTCA2);
    } else {
        dev_dbg(dev, "alarm1_irq disable\n");
        max77696_rtc_disable_irq(rtc, RTCINT_RTCA2);
    }

    __unlock(rtc);
    return 0;
}

static const struct rtc_class_ops max77696_rtc_ops1 = {
    .read_time        = max77696_rtc_op_read_time,
    .set_time         = max77696_rtc_op_set_time,
    .read_alarm       = max77696_rtc_op_read_alarm1,
    .set_alarm        = max77696_rtc_op_set_alarm1,
    .alarm_irq_enable = max77696_rtc_op_alarm1_irq_enable,
};

static ssize_t max77696_rtc_smplen_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    bool smplen;
    int rc;

    __lock(rtc);

    rc = max77696_rtc_smplen_read(rtc, &smplen);
    if (unlikely(rc)) {
        goto out;
    }

    rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", smplen);

out:
    __unlock(rtc);
    return (ssize_t)rc;
}

static ssize_t max77696_rtc_smplen_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    bool smplen;
    int rc;

    __lock(rtc);

    smplen = (bool)simple_strtoul(buf, NULL, 10);

    rc = max77696_rtc_smplen_write(rtc, smplen);
    if (unlikely(rc)) {
        goto out;
    }

    /* mask/unmask irq */
    if (smplen) {
        max77696_rtc_enable_irq (rtc, RTCINT_SMPL);
    } else {
        max77696_rtc_disable_irq(rtc, RTCINT_SMPL);
    }

out:
    __unlock(rtc);
    return (ssize_t)count;
}

static DEVICE_ATTR(rtc_smplen, S_IWUSR | S_IRUGO,
    max77696_rtc_smplen_show, max77696_rtc_smplen_store);

static ssize_t max77696_rtc_smplt_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    u32 smplt_msec;
    int rc;

    __lock(rtc);

    rc = max77696_rtc_smplt_read(rtc, &smplt_msec);
    if (unlikely(rc)) {
        goto out;
    }

    rc = (int)snprintf(buf, PAGE_SIZE, "%u msec\n", smplt_msec);

out:
    __unlock(rtc);
    return (ssize_t)rc;
}

static ssize_t max77696_rtc_smplt_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct max77696_rtc *rtc = dev_get_drvdata(dev);
    u32 smplt_msec;

    __lock(rtc);

    smplt_msec = (u32)simple_strtoul(buf, NULL, 10);
    max77696_rtc_smplt_write(rtc, smplt_msec);

    __unlock(rtc);
    return (ssize_t)count;
}

static DEVICE_ATTR(rtc_smplt, S_IWUSR | S_IRUGO,
    max77696_rtc_smplt_show, max77696_rtc_smplt_store);

#define RTC_DEV_ATTR(_name) \
        (&dev_attr_##_name.attr)

static struct attribute* max77696_rtc_attr[] = {
    RTC_DEV_ATTR(rtc_smplen),
    RTC_DEV_ATTR(rtc_smplt),
    NULL
};

static const struct attribute_group max77696_rtc_attr_group = {
    .attrs = max77696_rtc_attr,
};

static void *max77696_rtc_get_platdata (struct max77696_rtc *rtc)
{
    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    struct max77696_rtc_platform_data *pdata;
    struct device *dev = rtc->dev;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*pdata));
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->smpl_msec = 500;
    of_property_u32(np, "smpl_msec", &pdata->smpl_msec);
#else /* CONFIG_MAX77696_DT */
    pdata = dev_get_platdata(dev);
    if (unlikely(!pdata)) {
        pdata = ERR_PTR(-EINVAL);
        goto out;
    }
#endif /* CONFIG_MAX77696_DT */

    __prop_printk(dev, "SMPL TIMER", "%u msec", pdata->smpl_msec);

out:
    return pdata;
}

static __always_inline void max77696_rtc_destroy (struct max77696_rtc *rtc)
{
    struct device *dev = rtc->dev;

    if (likely(rtc->attr_grp)) {
        sysfs_remove_group(rtc->kobj, rtc->attr_grp);
    }

    if (likely(rtc->irq > 0)) {
        devm_free_irq(dev, rtc->irq, rtc);
    }

    rtc->rtc_master = NULL;

    if (likely(rtc->rtc_dev[1])) {
        devm_rtc_device_unregister(dev, rtc->rtc_dev[1]);
    }

    if (likely(rtc->rtc_dev[0])) {
        devm_rtc_device_unregister(dev, rtc->rtc_dev[0]);
    }

#ifdef CONFIG_MAX77696_DT
    if (likely(rtc->pdata)) {
        devm_kfree(dev, rtc->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&rtc->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, rtc);
}

static __devinit int max77696_rtc_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev->parent);
    struct max77696_rtc *rtc;
    u16 rtcint;
    int rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
    if (unlikely(!rtc)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*rtc));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, rtc);

    mutex_init(&rtc->lock);
    rtc->core  = core;
    rtc->io    = max77696_get_block_io(dev->parent, RTC);
    rtc->dev   = dev;
    rtc->kobj  = &dev->kobj;

    /* Disable all RTC interrupts */
    rtc->irq_unmask = 0;
    max77696_write(rtc->io, RTCINTM, ~0);

    /* Get RTC interrupt status port address & Clear status */
    rtcint = max77696_rtc_read_irq(rtc);
    max77696_rtc_ack_irq(rtc);
    dev_dbg(dev, "initial RTC interrupt status: %04X\n", rtcint);

    rtc->pdata = max77696_rtc_get_platdata(rtc);
    if (unlikely(IS_ERR(rtc->pdata))) {
        rc = PTR_ERR(rtc->pdata);
        rtc->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto abort;
    }

    /* Device initialization */
    max77696_rtc_setup(rtc);

    rtc->rtc_dev[0] = devm_rtc_device_register(dev, DRIVER_NAME".0",
        &max77696_rtc_ops0, THIS_MODULE);
    if (unlikely(IS_ERR(rtc->rtc_dev[0]))) {
        rc = PTR_ERR(rtc->rtc_dev[0]);
        rtc->rtc_dev[0] = NULL;
        dev_err(dev, "failed to register rtc0 [%d]\n", rc);
        goto abort;
    }

    rtc->rtc_dev[1] = devm_rtc_device_register(dev, DRIVER_NAME".1",
        &max77696_rtc_ops1, THIS_MODULE);
    if (unlikely(IS_ERR(rtc->rtc_dev[1]))) {
        rc = PTR_ERR(rtc->rtc_dev[1]);
        rtc->rtc_dev[1] = NULL;
        dev_err(dev, "failed to register rtc1 [%d]\n", rc);
        goto abort;
    }

    rtc->rtc_master = rtc->rtc_dev[RTC_MASTER];

    /* Get RTC block IRQ number */
    rtc->irq = max77696_get_block_irq(dev->parent, RTC);
    BUG_ON(rtc->irq <= 0);

    /* Request system IRQ for RTC */
    rc = devm_request_threaded_irq(dev, (unsigned int)rtc->irq, NULL,
        max77696_rtc_isr, IRQF_ONESHOT, DRIVER_NAME, rtc);
    if (unlikely(rc < 0)) {
        dev_err(dev, "failed to request IRQ(%d) [%d]\n", rtc->irq, rc);
        rtc->irq = 0;
        goto abort;
    }

    dev_dbg(dev, "IRQ(%d) requested\n", rtc->irq);

    disable_irq(rtc->irq);

    /* Create max77696-rtc sysfs attributes */
    rtc->attr_grp = &max77696_rtc_attr_group;
    rc = sysfs_create_group(rtc->kobj, rtc->attr_grp);
    if (unlikely(rc)) {
        dev_err(dev, "failed to create attribute group [%d]\n", rc);
        rtc->attr_grp = NULL;
        goto abort;
    }

    #if 1
    do {
        struct rtc_time tm;
        max77696_rtc_read_timekeeper_counters(rtc, &tm);
        max77696_rtc_dump_rtc_time(rtc, "initial time", &tm);
    } while (0);
    #endif

    pr_debug("%s() successfully done\n", __func__);
    return 0;

abort:
    max77696_rtc_destroy(rtc);
    return rc;
}

static __devexit int max77696_rtc_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_rtc *rtc = dev_get_drvdata(dev);

    max77696_rtc_destroy(rtc);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_rtc_suspend (struct device *dev)
{
    return 0;
}

static int max77696_rtc_resume (struct device *dev)
{
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_rtc_pm,
    max77696_rtc_suspend, max77696_rtc_resume);

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_uic_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_uic_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_rtc_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_rtc_pm,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_uic_of_match_table,
#endif /* CONFIG_MAX77696_DT */
    .probe                 = max77696_rtc_probe,
    .remove                = __devexit_p(max77696_rtc_remove),
};

static __init int max77696_rtc_driver_init (void)
{
    return platform_driver_register(&max77696_rtc_driver);
}
module_init(max77696_rtc_driver_init);

static __exit void max77696_rtc_driver_exit (void)
{
    platform_driver_unregister(&max77696_rtc_driver);
}
module_exit(max77696_rtc_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
