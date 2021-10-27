/*
 * Copyright (c) 2021 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77975_H__
#define __MAX77975_H__

#define DRV_NAME "max77975"

/* detail register bit description */
#define SIOP_INPUT_LIMIT_CURRENT                1200
#define SIOP_CHARGING_LIMIT_CURRENT             1000
#define SLOW_CHARGING_CURRENT_STANDARD          400

#define IRQ_WORK_DELAY              0

#define ENABLE 1
#define DISABLE 0

/* CHIP ID */
#define BIT_ID				BITS(7, 0)

/* CHIP REVISION */
#define BIT_REVISION			BITS(3, 0)
#define BIT_VERSION				BITS(7, 4)

/* CHG_INT, CHG_INT_MASK and CHG_INT_OK */
#define BIT_BYP					BIT(0)
#define BIT_DISQBAT				BIT(1)
#define BIT_BAT					BIT(3)
#define BIT_CHG					BIT(4)
#define BIT_INLIM				BIT(5)
#define BIT_CHGIN				BIT(6)
#define BIT_AICL				BIT(7)

/* CHG_DTLS_00 */
#define BIT_SPSN_DTLS			BITS(2, 1)
#define BIT_CHGIN_DTLS			BITS(6, 5)

/* CHG_DTLS_01 */
#define BIT_CHG_DTLS			BITS(3, 0)
#define BIT_BAT_DTLS			BITS(6, 4)
#define BIT_TREG				BIT(7)

/* CHG_DTLS_02 */
#define BIT_BYP_DTLS			BITS(3, 0)

/* CHG_CNFG_01 */
#define BIT_MODE				BITS(3, 0)
#define BIT_MODE_CHARGER		BIT(0)
#define BIT_MODE_OTG			BIT(1)
#define BIT_MODE_BUCK			BIT(2)
#define BIT_MODE_BOOST			BIT(3)

/* CHG_CNFG_01 */
#define BIT_FCHGTIME			BITS(2, 0)
#define BIT_CHG_RSTRT			BITS(5, 4)
#define BIT_WDTEN				BIT(6)
#define BIT_TEN					BIT(7)

/* CHG_CNFG_02 */
#define BIT_CHG_CC				BITS(6, 0)

/* CHG_CNFG_03 */
#define BIT_TO_ITH				BITS(3, 0)
#define BIT_TO_TIME				BITS(6, 4)

/* CHG_CNFG_04 */
#define BIT_CHG_CV_PRM			BITS(4, 0)
#define BIT_MINSYS				BITS(6, 5)
#define BIT_SYS_TRACK_DIS		BIT(7)

/* CHG_CNFG_06 */
#define BIT_WDTCLR				BITS(1, 0)
#define BIT_CHGPROT				BITS(3, 2)

/* CHG_CNFG_09 for MAX77975 */
#define BIT_CHGIN_ILIM_MAX77975	BITS(5, 0)
#define BIT_INLIM_CLK			BITS(7,6)

/* CHG_CNFG_09 for MAX77985 */
#define BIT_CHGIN_ILIM_MAX77985	BITS(6, 0)

/* CHG_CNFG_12 */
#define BIT_DIS_AICL			BIT( 0)
#define BIT_VCHGIN_REG			BITS(5,4)
#define BIT_DEEP_SUSP_DIS		BIT(6)
#define BIT_BYPDISCHG_EN		BIT(7)

enum {
	CHG_DTLS_PREQUAL = 0,
	CHG_DTLS_FASTCHARGE_CC,
	CHG_DTLS_FASTCHARGE_CV,
	CHG_DTLS_TOPOFF,
	CHG_DTLS_DONE,
	CHG_DTLS_RESERVED_05,
	CHG_DTLS_OFF_TIMER_FAULT,
	CHG_DTLS_OFF_SUSPEND,
	CHG_DTLS_OFF_INPUT_INVALID,
	CHG_DTLS_RESERVED_09,
	CHG_DTLS_OFF_JUCTION_TEMP,
	CHG_DTLS_OFF_WDT_EXPIRED,
	CHG_DTLS_CHG_CC_CV_REDUCED,
	CHG_DTLS_BATT_REMOVED,
	CHG_DTLS_SUSPEND_PIN,
};

enum chip_id {
	ID_MAX77975,
	ID_MAX77985,
};

enum chip_version {
	VERSION_A = 0x1010,
	VERSION_B,
};

enum chip_revision {
	PASS1 = 0x01,
	PASS2,
	PASS3,
	PASS4,
};

enum register_ids {
	REG_CHIP_ID = 0,
	REG_CHIP_REVISION,
	REG_OTP_REVISION,
	REG_TOP_INT,
	REG_TOP_INT_MASK,
	REG_TOP_CTRL,
	REG_SW_RESET,
	REG_SM_CTRL,
	REG_I2C_CONFIG,
	REG_CHG_INT,
	REG_CHG_INT_MASK,
	REG_CHG_INT_OK,
	REG_CHG_DETAILS_00,
	REG_CHG_DETAILS_01,
	REG_CHG_DETAILS_02,
	REG_CHG_CNFG_00,
	REG_CHG_CNFG_01,
	REG_CHG_CNFG_02,
	REG_CHG_CNFG_03,
	REG_CHG_CNFG_04,
	REG_CHG_CNFG_05,
	REG_CHG_CNFG_06,
	REG_CHG_CNFG_07,
	REG_CHG_CNFG_08,
	REG_CHG_CNFG_09,
	REG_CHG_CNFG_10,
	REG_CHG_CNFG_11,
	REG_CHG_CNFG_12,
	REG_CHG_CNFG_13,
	REG_STAT_CNFG
};

/* Register addresses  */
static const u8 max77975_regs[] = {
	[REG_CHIP_ID] = 0x00,
	[REG_CHIP_REVISION] = 0x01,
	[REG_OTP_REVISION] = 0x02,
	[REG_TOP_INT] = 0x03,
	[REG_TOP_INT_MASK] = 0x04,
	[REG_TOP_CTRL] = 0x05,
	[REG_SW_RESET] = 0x50,
	[REG_SM_CTRL] = 0x51,
	[REG_I2C_CONFIG] = 0x40,
	[REG_CHG_INT] = 0x10,
	[REG_CHG_INT_MASK] = 0x11,
	[REG_CHG_INT_OK] = 0x12,
	[REG_CHG_DETAILS_00] = 0x13,
	[REG_CHG_DETAILS_01] = 0x14,
	[REG_CHG_DETAILS_02] = 0x15,
	[REG_CHG_CNFG_00] = 0x16,
	[REG_CHG_CNFG_01] = 0x17,
	[REG_CHG_CNFG_02] = 0x18,
	[REG_CHG_CNFG_03] = 0x19,
	[REG_CHG_CNFG_04] = 0x1A,
	[REG_CHG_CNFG_05] = 0x1B,
	[REG_CHG_CNFG_06] = 0x1C,
	[REG_CHG_CNFG_07] = 0x1D,
	[REG_CHG_CNFG_08] = 0x1E,
	[REG_CHG_CNFG_09] = 0x1F,
	[REG_CHG_CNFG_10] = 0x20,
	[REG_CHG_CNFG_11] = 0x21,
	[REG_CHG_CNFG_12] = 0x22,
	[REG_CHG_CNFG_13] = 0x23,
	[REG_STAT_CNFG] = 0x24,
};

#define GET_TO_ITH(X)	(X < 1 ? 0 : ((X+2)*50))	/* mA */

#define SET_TO_ITH(X)	(X < 150 ? 0x00 : (X/50)-2)	/* mA */

#undef  __CONST_FFS
#define __CONST_FFS(_x) \
        ((_x) & 0x00FF ?\
            ((_x) & 0x000F ? ((_x) & 0x0003 ? ((_x) & 0x0001 ?  0 :  1) :\
                                              ((_x) & 0x0004 ?  2 :  3)) :\
                             ((_x) & 0x0030 ? ((_x) & 0x0010 ?  4 :  5) :\
                                              ((_x) & 0x0040 ?  6 :  7))) :\
            ((_x) & 0x0F00 ? ((_x) & 0x0300 ? ((_x) & 0x0100 ?  8 :  9) :\
                                              ((_x) & 0x0400 ? 10 : 11)) :\
                             ((_x) & 0x3000 ? ((_x) & 0x1000 ? 12 : 13) :\
                                              ((_x) & 0x4000 ? 14 : 15))))

#undef  FFS
#define FFS(_x) ((_x) ? __CONST_FFS(_x) : 0)

#define BITS(_end, _start) ((BIT(_end) - BIT(_start)) + BIT(_end))

struct max77975_charger {
	struct device           	*dev;
	struct regmap			*regmap;
    struct power_supply		*psy_chg;
	struct power_supply_desc	psy_chg_d;
	kernel_ulong_t driver_data;

    const u8        *regs;

	int				details_0;
	int				details_1;
	int				details_2;

	struct delayed_work		irq_work;

	int				irq;
	int				irq_gpio;

	/* mutex */
	struct mutex			lock;

	int 			chip_id;
	int 			chip_version;
	int				chip_revision;
	int				present;
	int				health;
	int				status;
	int				charge_type;

	struct max77975_charger_platform_data *pdata;
};

struct max77975_charger_platform_data {
	int fast_charge_timer;
	int fast_charge_current;
	int termination_voltage;
	int topoff_timer;
	int topoff_current;
	int restart_threshold;
	int input_current_limit;
};

#endif
