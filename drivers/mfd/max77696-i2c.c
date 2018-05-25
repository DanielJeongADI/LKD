/*
 * MAX77696 I2C Support
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

#ifdef MODULE
#error "NOT IMPLEMENTED AS MODULABLE"
#endif /* MODULE */

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
#include <linux/gpio.h>
#include <linux/i2c.h>

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>

#include <linux/mfd/core.h>
#include <linux/mfd/max77696.h>
#include "max77696-core.h"

#ifdef CONFIG_SPARSE_IRQ
#define TOPINT_DEFAULT_IRQ_BASE      NR_IRQS
#define TOPSYS_DEFAULT_IRQ_BASE      NR_IRQS
#else /* CONFIG_SPARSE_IRQ */
#define TOPINT_DEFAULT_IRQ_BASE      8
#define TOPSYS_DEFAULT_IRQ_BASE      8
#endif /* CONFIG_SPARSE_IRQ */

#define DRIVER_DESC    MAX77696_DESC" I2C"
#define DRIVER_NAME    MAX77696_NAME
#define DRIVER_ALIAS   "platform:"DRIVER_NAME
#define DRIVER_VERSION MAX77696_CORE_VERSION
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maximintegrated.com>"

/*******************************************************************************
 * Macro converting byte-order master -> slave
 */

#define __m2s_8(_addr)      ((u8)(_addr))
#define __m2s_16be(_addr)   (__cpu_to_be16(_addr))
#define __m2s_16le(_addr)   (__cpu_to_le16(_addr))

/*******************************************************************************
 * Macro making i2c msg and xfer
 */

#define __build_w_msg(_msg, _client, _buf, _buflen) \
        do {\
            (_msg) ->addr  = (_client) ->addr;\
            (_msg) ->flags = (_client) ->flags & I2C_M_TEN;\
            (_msg) ->len   = _buflen;\
            (_msg) ->buf   = (void*)(_buf);\
        } while (0)
#define __build_r_msg(_msg, _client, _buf, _buflen) \
        do {\
            __build_w_msg(_msg, _client, _buf, _buflen);\
            (_msg) ->flags |= I2C_M_RD;\
        } while (0)
#define __transfer(_client, _msg, _msglen) \
        ({\
            int __rc = i2c_transfer((_client) ->adapter, _msg, _msglen);\
            (__rc == _msglen) ? 0 : __rc;\
        })

/*******************************************************************************
 * Macro reading from/writing to, each protocol, each size of addr & data
 *
 * addr8 ----+- data8
 *           +- data16be
 *           +- data16le
 *
 * addr16be -+- data8
 *           +- data16be
 *           +- data16le
 *
 * addr16le -+- data8
 *           +- data16be
 *           +- data16le
 */

#define __a8_seq_readv(_io, _addr, _dstp, _len) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[2];\
            u8 __addr = __m2s_8(_addr);\
            __build_w_msg(&__msg[0], __client, &__addr, 1);\
            __build_r_msg(&__msg[1], __client, _dstp, _len);\
            __transfer(__client, __msg, 2);\
        })
#define __a16_seq_readv(_io, _addr, _dstp, _len, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[2];\
            u16 __addr = __m2s_16##_order(_addr);\
            __build_w_msg(&__msg[0], __client, &__addr, 2);\
            __build_r_msg(&__msg[1], __client, _dstp, _len);\
            __transfer(__client, __msg, 2);\
        })
#define __a8_seq_writev(_io, _addr, _srcp, _len) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[(_len) + 1];\
            __buf[0] = __m2s_8(_addr);\
            memcpy(&__buf[1], _srcp, _len);\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a16_seq_writev(_io, _addr, _srcp, _len, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[(_len) + 2];\
            *((u16*)&__buf[0]) = __m2s_16##_order(_addr);\
            memcpy(&__buf[2], _srcp, _len);\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a8_seq_write8(_io, _addr, _src8) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[2];\
            __buf[0] = __m2s_8(_addr);\
            __buf[1] = __m2s_8(_src8);\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a8_seq_write16(_io, _addr, _src16, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[3];\
            __buf[0] = __m2s_8(_addr);\
            *((u16*)&__buf[1]) = __m2s_16##_order(_src16);\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a16_seq_write8(_io, _addr, _src8, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[3];\
            *((u16*)&__buf[0]) = __m2s_16##_order(_addr);\
            __buf[2] = __m2s_8(_src8);\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a16_seq_write16(_io, _addr, _src16, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[4];\
            *((u16*)&__buf[0]) = __m2s_16##_order(_addr);\
            *((u16*)&__buf[2]) = __m2s_16##_order(_src16);\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a8_pair_writev_x1(_io, _addr, _srcp, _len) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[(_len) << 1];\
            u16 __addri = 0, __srci = 0;\
            int __bufi = 0, __rc;\
            while (__srci < (_len)) {\
                __buf[__bufi++] = __m2s_8((_addr) + __addri++);\
                __buf[__bufi++] = __m2s_8((_srcp)[__srci++]);\
            }\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a8_pair_writev_x2(_io, _addr, _srcp, _len, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[((_len) >> 1) + (_len)];\
            u16 __addri = 0, __srci = 0;\
            int __bufi = 0, __rc;\
            BUG_ON((_len) % 2);\
            while (__srci < (_len)) {\
                u16 __src16 = (__force u16)*((u16*)&(_srcp)[__srci]);\
                __srci += 2;\
                __buf[__bufi++] = __m2s_8((_addr) + __addri++);\
                *((u16*)&__buf[__bufi]) = __m2s_16##_order(__src16);\
                __bufi += 2;\
            }\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a16x1_pair_writev_x1(_io, _addr, _srcp, _len, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[((_len) << 1) + (_len)];\
            u16 __addri = 0, __srci = 0;\
            int __bufi = 0, __rc;\
            while (__srci < (_len)) {\
                *((u16*)&__buf[__bufi]) =\
                    __m2s_16##_order((_addr) + __addri++);\
                __bufi += 2;\
                __buf[__bufi++] = __m2s_8((_srcp)[__srci++]);\
            }\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a16x1_pair_writev_x2(_io, _addr, _srcp, _len, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[((_len) << 1) + (_len)];\
            u16 __addri = 0, __srci = 0;\
            int __bufi = 0, __rc;\
            while (__srci < (_len)) {\
                u16 __src16 = (__force u16)*((u16*)&(_srcp)[__srci]);\
                __srci += 2;\
                *((u16*)&__buf[__bufi]) =\
                    __m2s_16##_order((_addr) + __addri++);\
                __bufi += 2;\
                *((u16*)&__buf[__bufi]) = __m2s_16##_order(__src16);\
                __bufi += 2;\
            }\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })

#define __a16x2_pair_writev_x1(_io, _addr, _srcp, _len, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[((_len) << 1) + (_len)];\
            u16 __addri = 0, __srci = 0;\
            int __bufi = 0, __rc;\
            while (__srci < (_len)) {\
                *((u16*)&__buf[__bufi]) =\
                    __m2s_16##_order((_addr) + __addri);\
                __addri += 2;\
                __bufi += 2;\
                __buf[__bufi++] = __m2s_8((_srcp)[__srci++]);\
            }\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })
#define __a16x2_pair_writev_x2(_io, _addr, _srcp, _len, _order) \
        ({\
            struct i2c_client *__client = to_i2c_client((_io) ->dev);\
            struct i2c_msg __msg[1];\
            u8 __buf[((_len) << 1) + (_len)];\
            u16 __addri = 0, __srci = 0;\
            int __bufi = 0, __rc;\
            while (__srci < (_len)) {\
                u16 __src16 = (__force u16)*((u16*)&(_srcp)[__srci]);\
                __srci += 2;\
                *((u16*)&__buf[__bufi]) =\
                    __m2s_16##_order((_addr) + __addri);\
                __addri += 2;\
                __bufi += 2;\
                *((u16*)&__buf[__bufi]) = __m2s_16##_order(__src16);\
                __bufi += 2;\
            }\
            __build_w_msg(&__msg[0], __client, __buf, sizeof(__buf));\
            __transfer(__client, __msg, 1);\
        })

/*******************************************************************************
 * R/W interfaces for types of I2C protocols
 *
 * seq ----+- read
 *         +- write
 *
 * pair ---+- x1 write
 *         +- x2 write
 *
 * single -+- read
 *         +- write
 */

/* Reading multiple bytes from sequential registers */

static __inline int seq_read_addr8 (struct max77696_io *io,
    u16 addr, u8 *dst, u16 len)
{
    return __a8_seq_readv(io, addr, dst, len);
}

static __inline int seq_read_addr16_be (struct max77696_io *io,
    u16 addr, u8 *dst, u16 len)
{
    return __a16_seq_readv(io, addr, dst, len, be);
}

static __inline int seq_read_addr16_le (struct max77696_io *io,
    u16 addr, u8 *dst, u16 len)
{
    return __a16_seq_readv(io, addr, dst, len, le);
}

/* Writing multiple bytes to sequential registers */

static __inline int seq_write_addr8 (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a8_seq_writev(io, addr, src, len);
}

static __inline int seq_write_addr16_be (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16_seq_writev(io, addr, src, len, be);
}

static __inline int seq_write_addr16_le (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16_seq_writev(io, addr, src, len, le);
}

#if 0
/* Writing multiple bytes using register-data pairs */

static __inline int pair_write_addr8_data8 (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a8_pair_writev_x1(io, addr, src, len);
}

static __inline int pair_write_addr8_data16_be (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a8_pair_writev_x2(io, addr, src, len, be);
}

static __inline int pair_write_addr8_data16_le (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a8_pair_writev_x2(io, addr, src, len, le);
}

static __inline int pair_write_addr16x1_data8_be (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16x1_pair_writev_x1(io, addr, src, len, be);
}

static __inline int pair_write_addr16x1_data8_le (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16x1_pair_writev_x1(io, addr, src, len, le);
}

static __inline int pair_write_addr16x1_data16_be (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16x1_pair_writev_x2(io, addr, src, len, be);
}

static __inline int pair_write_addr16x1_data16_le (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16x1_pair_writev_x2(io, addr, src, len, le);
}

static __inline int pair_write_addr16x2_data8_be (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16x2_pair_writev_x1(io, addr, src, len, be);
}

static __inline int pair_write_addr16x2_data8_le (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16x2_pair_writev_x1(io, addr, src, len, le);
}

static __inline int pair_write_addr16x2_data16_be (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16x2_pair_writev_x2(io, addr, src, len, be);
}

static __inline int pair_write_addr16x2_data16_le (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return __a16x2_pair_writev_x2(io, addr, src, len, le);
}
#endif

/* Reading a word from a single register */

static __inline int single_read_addr8_data8 (struct max77696_io *io,
    u16 addr, u16 *val)
{
    *val = 0;
    return __a8_seq_readv(io, addr, val, 1);
}

static __inline int single_read_addr8_data16_be (struct max77696_io *io,
    u16 addr, u16 *val)
{
    int rc = __a8_seq_readv(io, addr, (u8*)val, 2);
    if (likely(!rc)) {
        *val = __be16_to_cpup(val);
    }
    return rc;
}

static __inline int single_read_addr8_data16_le (struct max77696_io *io,
    u16 addr, u16 *val)
{
    int rc = __a8_seq_readv(io, addr, (u8*)val, 2);
    if (likely(!rc)) {
        *val = __le16_to_cpup(val);
    }
    return rc;
}

static __inline int single_read_addr16_data8_be (struct max77696_io *io,
    u16 addr, u16 *val)
{
    *val = 0;
    return __a16_seq_readv(io, addr, val, 1, be);
}

static __inline int single_read_addr16_data8_le (struct max77696_io *io,
    u16 addr, u16 *val)
{
    *val = 0;
    return __a16_seq_readv(io, addr, val, 1, le);
}

static __inline int single_read_addr16_data16_be (struct max77696_io *io,
    u16 addr, u16 *val)
{
    int rc = __a16_seq_readv(io, addr, (u8*)val, 2, be);
    if (likely(!rc)) {
        *val = __be16_to_cpup(val);
    }
    return rc;
}

static __inline int single_read_addr16_data16_le (struct max77696_io *io,
    u16 addr, u16 *val)
{
    int rc = __a16_seq_readv(io, addr, (u8*)val, 2, le);
    if (likely(!rc)) {
        *val = __le16_to_cpup(val);
    }
    return rc;
}

/* Writing a word to a single register */

static __inline int single_write_addr8_data8 (struct max77696_io *io,
    u16 addr, u16 val)
{
    return __a8_seq_write8(io, addr, val);
}

static __inline int single_write_addr8_data16_be (struct max77696_io *io,
    u16 addr, u16 val)
{
    return __a8_seq_write16(io, addr, val, be);
}

static __inline int single_write_addr8_data16_le (struct max77696_io *io,
    u16 addr, u16 val)
{
    return __a8_seq_write16(io, addr, val, le);
}

static __inline int single_write_addr16_data8_be (struct max77696_io *io,
    u16 addr, u16 val)
{
    return __a16_seq_write8(io, addr, val, be);
}

static __inline int single_write_addr16_data8_le (struct max77696_io *io,
    u16 addr, u16 val)
{
    return __a16_seq_write8(io, addr, val, le);
}

static __inline int single_write_addr16_data16_be (struct max77696_io *io,
    u16 addr, u16 val)
{
    return __a16_seq_write16(io, addr, val, be);
}

static __inline int single_write_addr16_data16_le (struct max77696_io *io,
    u16 addr, u16 val)
{
    return __a16_seq_write16(io, addr, val, le);
}

/******************************************************************************/

#define MAX77696_I2C_NAME  MAX77696_NAME"-i2c"

/* MAX77696 I2C Slave Addresses
 *   PMIC   7'h3C
 *   RTC    7'h68
 *   UIC    7'h35
 *   FG     7'h34
 */
#define PMIC_I2C_ADDR      0x3C
#define RTC_I2C_ADDR       0x68
#define UIC_I2C_ADDR       0x35
#define FG_I2C_ADDR        0x34

#define PMIC_I2C_NAME      MAX77696_I2C_NAME".pmic"
#define RTC_I2C_NAME       MAX77696_I2C_NAME".rtc"
#define UIC_I2C_NAME       MAX77696_I2C_NAME".uic"
#define FG_I2C_NAME        MAX77696_I2C_NAME".fg"

struct max77696_i2c_info {
    u16                   addresses[MAX77696_NUM_OF_IOS+1];
    struct i2c_device_id  i2cdevids[MAX77696_NUM_OF_IOS+1];
    struct i2c_board_info boardinfo[MAX77696_NUM_OF_IOS  ];
};

#define MAX77696_I2C_INFO(_name) \
    .addresses[MAX77696_IO_##_name] = _name##_I2C_ADDR,\
    .i2cdevids[MAX77696_IO_##_name] = { .name = _name##_I2C_NAME, },\
    .boardinfo[MAX77696_IO_##_name] =\
        { I2C_BOARD_INFO(_name##_I2C_NAME, _name##_I2C_ADDR), }

static struct max77696_i2c_info max77696_i2c_info = {
    MAX77696_I2C_INFO(PMIC),
    MAX77696_I2C_INFO(RTC ),
    MAX77696_I2C_INFO(UIC ),
    MAX77696_I2C_INFO(FG  ),
    .addresses[MAX77696_NUM_OF_IOS] = I2C_CLIENT_END,
    .i2cdevids[MAX77696_NUM_OF_IOS] = { .name = { 0 }, },
};

static struct max77696_core *max77696_core;

static __always_inline int max77696_i2c_io_index (struct i2c_client *client)
{
    int i;
    for (i = 0; i < MAX77696_NUM_OF_IOS; i++) {
        if (max77696_i2c_info.addresses[i] == client->addr) {
            goto found;
        }
    }
    BUG();
found:
    return i;
}

static __always_inline void max77696_i2c_set_io (struct i2c_client *client,
    void *read, void *write, void *bulk_read, void *bulk_write)
{
    struct max77696_core *core = i2c_get_clientdata(client);
    __max77696_core_set_io(core, max77696_i2c_io_index(client),
        &client->dev, read, write, bulk_read, bulk_write);
    pr_debug("%s io ready\n", client->name);
}

static __devinit int max77696_i2c_probe (struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct max77696_core *core;
    int i, rc = 0;

    pr_debug("%s attached\n", client->name);

    core = max77696_core;
    if (unlikely(!core)) {
        dev_err(&client->dev, "core pointer is missing\n");
        return -EINVAL;
    }

#ifdef CONFIG_MAX77696_DT
    if (unlikely(core->pdata->intb_gpio == -EPROBE_DEFER)) {

        core->pdata->intb_gpio =
            of_get_named_gpio(core->dev->of_node, "intb_gpio", 0);

        if (unlikely(core->pdata->intb_gpio == -EPROBE_DEFER)) {
            dev_dbg(core->dev, "intb_gpio not ready\n");
            rc = -EPROBE_DEFER;
            goto out;
        }

        if (unlikely(core->pdata->intb_gpio < 0)) {
            dev_err(core->dev, "intb_gpio not specified [%d]\n",
                core->pdata->intb_gpio);
            rc = core->pdata->intb_gpio;
            goto out;
        }
    }
#endif /* CONFIG_MAX77696_DT */

    i2c_set_clientdata(client, core);

    switch ((u8)client->addr) {
    case PMIC_I2C_ADDR:   /* 8-bit address /  8-bit data */
    case RTC_I2C_ADDR:    /* 8-bit address /  8-bit data */
    case UIC_I2C_ADDR:    /* 8-bit address /  8-bit data */
        max77696_i2c_set_io(client,
            single_read_addr8_data8 ,  /* read */
            single_write_addr8_data8,  /* write */
            seq_read_addr8          ,  /* bulk read */
            seq_write_addr8         ); /* bulk write */
        break;

    case FG_I2C_ADDR:     /* 8-bit address / 16-bit data */
        max77696_i2c_set_io(client,
            single_read_addr8_data16_le ,  /* read */
            single_write_addr8_data16_le,  /* write */
            seq_read_addr16_le          ,  /* bulk read */
            seq_write_addr16_le         ); /* bulk write */
        break;

    default:
        BUG();
        return -ENODEV;
    }

    /* Wait for all probe */
    for (i = 0; i < ARRAY_SIZE(core->chip_io); i++) {
        if (likely(!core->chip_io[i].dev)) {
            goto out;
        }
    }

    /* Request INTB gpio */
	rc = devm_gpio_request_one(core->dev, (unsigned)core->pdata->intb_gpio,
	    GPIOF_IN, MAX77696_NAME"-intb");
	if (unlikely(rc)) {
        dev_err(core->dev, "failed to request INTB gpio [%d]\n", rc);
        goto out;
    }

    core->pdata->topint_pdata->irq = gpio_to_irq(core->pdata->intb_gpio);
    dev_info(core->dev, "IRQ %d GPIO %d\n", core->pdata->topint_pdata->irq,
        core->pdata->intb_gpio);

    /* Complete initialization */
    rc = max77696_init(core);
    if (unlikely(rc)) {
        goto out;
    }

    pr_debug("%s() successfully done\n", __func__);

out:
    return rc;
}

static __devexit int max77696_i2c_remove (struct i2c_client *client)
{
    return 0;
}

static int max77696_i2c_detect (struct i2c_client *client,
    struct i2c_board_info *boardinfo)
{
	struct i2c_adapter *adapter = client->adapter;
    struct max77696_core *core;
    char *name;
	int rc;

    core = max77696_core;
    if (unlikely(!core)) {
        rc = -ENODEV;
        goto out;
    }

    name = max77696_i2c_info.i2cdevids[max77696_i2c_io_index(client)].name;
    strlcpy(boardinfo->type, name, I2C_NAME_SIZE);

	pr_debug(DRIVER_NAME": detected chip %s at adapter %d address 0x%02X\n",
		 boardinfo->type, i2c_adapter_id(adapter), client->addr);
    rc = 0;

out:
    return rc;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_i2c_suspend (struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct max77696_core *core = i2c_get_clientdata(client);

    return max77696_suspend_core(core);
}

static int max77696_i2c_resume (struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct max77696_core *core = i2c_get_clientdata(client);

    return max77696_resume_core(core);
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_i2c_pm, max77696_i2c_suspend,
    max77696_i2c_resume);

static struct i2c_driver max77696_i2c_driver = {
    .driver.name           = MAX77696_I2C_NAME,
    .driver.owner          = THIS_MODULE,
    .driver.pm             = &max77696_i2c_pm,
    .class                 = I2C_CLASS_HWMON,
    .probe                 = max77696_i2c_probe,
    .remove                = __devexit_p(max77696_i2c_remove),
    .id_table              = max77696_i2c_info.i2cdevids,
    .detect                = max77696_i2c_detect,
    .address_list          = max77696_i2c_info.addresses,
};

/***
 *** MAX77696 DRIVER MAIN ENTRY
 ***/

static void *max77696_get_platdata (struct max77696_core *core)
{
    struct device *dev = core->dev;
    struct max77696_platform_data *pdata;

#ifdef CONFIG_MAX77696_DT
    struct device_node *np = dev->of_node;
    size_t sz;
    int rc;

    sz = sizeof(*pdata) +
        sizeof(*pdata->topint_pdata) + sizeof(*pdata->topsys_pdata);
    pdata = devm_kzalloc(dev, sz, GFP_KERNEL);
    if (unlikely(!pdata)) {
        dev_err(dev, "out of memory (%uB requested)\n", sz);
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pdata->topint_pdata = (void*)(pdata + 1);
    pdata->topsys_pdata = (void*)(pdata->topint_pdata + 1);

	pdata->intb_gpio = of_get_named_gpio(np, "intb_gpio", 0);
    if (unlikely(pdata->intb_gpio != -EPROBE_DEFER && pdata->intb_gpio < 0)) {
        dev_err(dev, "intb_gpio not specified [%d]\n", pdata->intb_gpio);
        rc = pdata->intb_gpio;
        goto abort;
    }

    pdata->io_busnum = -EPROBE_DEFER;
    of_property_u32(np, "i2c_busnum", (u32*)&pdata->io_busnum);

    pdata->topint_pdata->irq_trigger = IRQ_TYPE_EDGE_FALLING;
    of_property_u32(np, "topint_irq_trigger",
        (u32*)&pdata->topint_pdata->irq_trigger);

    pdata->topint_pdata->irq_base = 0;
    of_property_u32(np, "topint_irq_base", &pdata->topint_pdata->irq_base);

    pdata->topsys_pdata->irq_base = 0;
    of_property_u32(np, "topsys_irq_base", &pdata->topsys_pdata->irq_base);

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

static __init int max77696_probe (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core;
    struct i2c_adapter *i2c_adapter;
    int i, rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
    if (unlikely(!core)) {
        dev_err(dev, "out of memory (%uB requested)\n", sizeof(*core));
        return -ENOMEM;
    }

    dev_set_drvdata(dev, core);

    mutex_init(&core->lock);
    core->dev   = dev;
    core->kobj  = &dev->kobj;

    core->pdata = max77696_get_platdata(core);
    if (unlikely(IS_ERR(core->pdata))) {
        rc = PTR_ERR(core->pdata);
        core->pdata = NULL;
        dev_err(dev, "failed to get platform data [%d]\n", rc);
        goto out;
    }

    #undef  __prop_printk
    #define __prop_printk(_dev, _prop, _fmt, ...) \
            dev_dbg(_dev, "%-26s" _fmt "\n", "property:" _prop, ##__VA_ARGS__)

    if (core->pdata->intb_gpio == -EPROBE_DEFER) {
        __prop_printk(dev, "INTB GPIO", "%s", "(deferred)");
    } else {
        __prop_printk(dev, "INTB GPIO", "%d", core->pdata->intb_gpio);
    }

    if (core->pdata->io_busnum < 0) {
        __prop_printk(dev, "I2C BUS", "%s", "(non-static)");
    } else {
        __prop_printk(dev, "I2C BUS", "%d", core->pdata->io_busnum);
    }

    if (core->pdata->topint_pdata->irq_base <= 0) {
        core->pdata->topint_pdata->irq_base = TOPINT_DEFAULT_IRQ_BASE;
    }
    if (core->pdata->topsys_pdata->irq_base <= 0) {
        core->pdata->topsys_pdata->irq_base = TOPSYS_DEFAULT_IRQ_BASE;
    }

    __prop_printk(dev, "TOPINT IRQTYPE", "%lu",
        core->pdata->topint_pdata->irq_trigger);
    __prop_printk(dev, "TOPINT IRQBASE", "%u",
        core->pdata->topint_pdata->irq_base);
    __prop_printk(dev, "TOPSYS IRQBASE", "%u",
        core->pdata->topsys_pdata->irq_base);

    /* Device detection callback for automatic device creation */
    if (core->pdata->io_busnum < 0) {
        max77696_i2c_driver.detect = max77696_i2c_detect;
    } else {
        max77696_i2c_driver.detect = NULL;
    }

    /* Register I2C driver */
    rc = i2c_add_driver(&max77696_i2c_driver);
    if (unlikely(rc)) {
        dev_err(dev, "failed to add i2c driver [%d]\n", rc);
        goto out;
    }

    /* Save 'core' pointer */
    max77696_core = core;

    /* Rest initialization will be done at max77696_i2c_probe() */
    pr_debug("%s() successfully done\n", __func__);

    /* Register I2C board info */
    if (unlikely(core->pdata->io_busnum < 0)) {
        goto out;
    }

    i2c_adapter = i2c_get_adapter(core->pdata->io_busnum);
    if (!i2c_adapter) {
        rc = i2c_register_board_info(core->pdata->io_busnum,
            max77696_i2c_info.boardinfo, MAX77696_NUM_OF_IOS);
        if (unlikely(rc)) {
            dev_err(dev, "failed to register i2c boards [%d]\n", rc);
            //goto out;
        }
    } else {
        for (i = 0; i < MAX77696_NUM_OF_IOS; i++) {
            dev_dbg(dev, "attaching %s to %s\n",
                max77696_i2c_info.boardinfo[i].type, i2c_adapter->name);
            i2c_new_device(i2c_adapter, &max77696_i2c_info.boardinfo[i]);
        }
        i2c_put_adapter(i2c_adapter);
    }

out:
    return rc;
}

static __exit int max77696_remove (struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77696_core *core = dev_get_drvdata(dev);

    /* Clear 'core' pointer */
    max77696_core = NULL;

    i2c_del_driver(&max77696_i2c_driver);

    max77696_exit(core);

#ifdef CONFIG_MAX77696_DT
    if (likely(core->pdata)) {
        devm_kfree(core->dev, core->pdata);
    }
#endif /* CONFIG_MAX77696_DT */

    mutex_destroy(&core->lock);
    dev_set_drvdata(dev, NULL);
    devm_kfree(dev, core);

    return 0;
}

#ifdef CONFIG_MAX77696_DT
static struct of_device_id max77696_of_match_table[] = {
	{ .compatible = "maxim,"DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, max77696_of_match_table);
#endif /* CONFIG_MAX77696_DT */

static struct platform_driver max77696_driver = {
    .driver.name           = DRIVER_NAME,
    .driver.owner          = THIS_MODULE,
#ifdef CONFIG_MAX77696_DT
    .driver.of_match_table = max77696_of_match_table,
#endif /* CONFIG_MAX77696_DT */
  //.probe                 = max77696_probe,
  //.remove                = __exit_p(max77696_remove),
};

static __init int max77696_driver_init (void)
{
    int rc;

    max77696_driver.probe  = max77696_probe;
    max77696_driver.remove = __exit_p(max77696_remove);

    rc = platform_driver_register(&max77696_driver);

    pr_debug(DRIVER_DESC" registered [%d]\n", rc);
    return rc;
}
arch_initcall(max77696_driver_init);

#if 0
static __exit void max77696_driver_exit (void)
{
    platform_driver_unregister(&max77696_driver);
}
module_exit(max77696_driver_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS(DRIVER_ALIAS);
