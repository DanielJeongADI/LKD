/*
 * Copyright (c) 2019 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __MAX77960_CHARGER_H
#define __MAX77960_CHARGER_H __FILE__
#include <linux/mfd/core.h>
#include <linux/regulator/machine.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>

#define ENABLE 1
#define DISABLE 0

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


enum {
	CHG_INT_BYP_I,
	CHG_INT_BATP_I,
	CHG_INT_BAT_I,
	CHG_INT_CHG_I,
	CHG_INT_WCIN_I,
	CHG_INT_CHGIN_I,
	CHG_INT_AICL_I,
};


/*
 * Register address
 */
#define	MAX77960_CID				0x00
#define	MAX77960_SWRST				0x01
#define	MAX77960_TOP_INT			0x02
#define	MAX77960_TOP_INT_MASK		0x03
#define	MAX77960_TOP_INT_OK			0x04
#define	MAX77960_CHG_INT			0x10
#define	MAX77960_CHG_INT_MASK		0x11
#define	MAX77960_CHG_INT_OK			0x12

#define MAX77960_CHG_DTLS_00    0x13
#define MAX77960_CHG_DTLS_01    0x14
#define MAX77960_CHG_DTLS_02    0x15


#define	MAX77960_CHG_CNFG_00		0x16
#define	MAX77960_CHG_CNFG_01		0x17

#define	MAX77960_CHG_CNFG_02		0x18
#define	MAX77960_CHG_CNFG_03		0x19

#define	MAX77960_CHG_CNFG_04		0x1A
#define	MAX77960_CHG_CNFG_05		0x1B
#define	MAX77960_CHG_CNFG_06		0x1C
#define	MAX77960_CHG_CNFG_07		0x1D

#define	MAX77960_CHG_CNFG_08		0x1E
#define	MAX77960_CHG_CNFG_09		0x1F
#define	MAX77960_CHG_CNFG_10		0x20

/*
 * MAX77960_TOP_INT
 */
#define BIT_TSHDN_I				BIT(2)
#define BIT_SYSOVLO_I			BIT(1)
#define BIT_SYSUVLO_I			BIT(0)

/*
 * MAX77960_TOP_INT_MASK
 */
#define BIT_TSHDN_M				BIT(2)
#define BIT_SYSOVLO_M			BIT(1)
#define BIT_SYSUVLO_M			BIT(0)

/*
 * MAX77960_TOP_INT_OK
 */
#define BIT_TSHDN_OK			BIT(2)
#define BIT_SYSOVLO_OK			BIT(1)
#define BIT_SYSUVLO_OK			BIT(0)

/*
 * MAX77960_CHG_INT
 */
#define BIT_AICL_I				BIT(7)
#define BIT_CHGIN_I				BIT(6)
#define BIT_B2SOVRC_I			BIT(5)
#define BIT_CHG_I				BIT(4)
#define BIT_BAT_I				BIT(3)
#define BIT_CHGINILIM_I		    BIT(2)
#define BIT_DISQBAT_I			BIT(1)
#define BIT_OTG_PLIM_I		    BIT(0)

/*
 * MAX77960_CHG_INT_MASK
 */
#define BIT_AICL_M				BIT(7)
#define BIT_CHGIN_M				BIT(6)
#define BIT_B2SOVRC_M			BIT(5)
#define BIT_CHG_M				BIT(4)
#define BIT_BAT_M				BIT(3)
#define BIT_CHGINILIM_M		    BIT(2)
#define BIT_DISQBAT_M			BIT(1)
#define BIT_OTG_PLIM_M		    BIT(0)

/*
 * MAX77960_CHG_INT_OK
 */
#define BIT_AICL_OK				BIT(7)
#define BIT_CHGIN_OK			BIT(6)
#define BIT_B2SOVRC_OK		    BIT(5)
#define BIT_CHG_OK				BIT(4)
#define BIT_BAT_OK				BIT(3)
#define BIT_CHGINILIM_OK	    BIT(2)
#define BIT_DISQBAT_OK		    BIT(1)
#define BIT_OTG_PLIM_OK		    BIT(0)

/*
 * MAX77960_CHG_DTSL00
 */
#define BIT_CHGIN_DTLS		    BITS(6, 5)
#define BIT_OTG_DTLS			BITS(4, 3)
#define BIT_QB_DTLS				BIT(0)

/*
 * MAX77960_CHG_DTSL01
 */
#define BIT_TREG				BITS(6, 5)
#define BIT_BAT_DTLS			BITS(6, 4)
#define BIT_CHG_DTLS			BITS(3, 0)


/*
 * MAX77960_CHG_DTSL02
 */
#define BIT_THM_DTLS			BITS(6, 4)
#define BIT_APP_MODE_DTLS		BIT(3)
#define BIT_FSW_DTLS			BITS(2, 1)
#define BIT_NUM_CELL_DTLS		BIT(0)


/*
 * MAX77960_CHG_CNFG_00
 */
#define BIT_COMM_MODE			BIT(7)
#define BIT_DISIBS				BIT(6)
#define BIT_STBY_EN				BIT(5)
#define BIT_WDTEN				BIT(4)
#define BIT_MODE				BITS(3, 0)


/*
 * MAX77960_CHG_CNFG_01
 */
#define BIT_PQEN				BIT(7)
#define BIT_LPM					BIT(6)
#define BIT_CHG_RSTRT			BITS(5, 4)
#define BIT_STAT_EN				BIT(3)
#define BIT_FCHGTIME			BITS(2, 0)


/*
 * MAX77960_CHG_CNFG_02
 */
#define BIT_CHGCC				BITS(5, 0)

/*
 * MAX77960_CHG_CNFG_03
 */
#define BIT_SYS_TRACK_DIS		BIT(7)
#define BIT_B2SOVRC_DTC			BIT(6)
#define BIT_TO_TIME				BITS(5, 3)
#define BIT_TO_ITH				BITS(2, 0)


/*
 * MAX77960_CHG_CNFG_04
 */
#define BIT_CHG_CV_PRM			BITS(5, 0)

/*
 * MAX77960_CHG_CNFG_05
 */
#define BIT_ITRICKLE			BITS(5, 4)
#define BIT_B2SOVRC				BITS(3, 0)

/*
 * MAX77960_CHG_CNFG_06
 */
#define BIT_SLOWLX				BITS(6, 5)
#define BIT_CHGPROT				BITS(3, 2)
#define BIT_WDTCLR				BITS(1, 0)

/*
 * MAX77960_CHG_CNFG_07
 */
#define BIT_JEITA_EN			BIT(7)
#define BIT_REGTEMP				BITS(6, 3)
#define BIT_VCHGCV_COOL			BIT(2)
#define BIT_ICHGCC_COOL			BIT(1)
#define BIT_FSHIP_MODE			BIT(0)



/*
 * MAX77960_CHG_CNFG_08
 */
#define BIT_CHGIN_ILIM			BITS(6, 0)

/*
 * MAX77960_CHG_CNFG_09
 */
#define BIT_INLIM_CLK			BITS(7, 6)
#define BIT_OTG_ILIM			BITS(5, 3)
#define BIT_MINVSYS				BITS(2, 0)

/*
 * MAX77960_CHG_CNFG_10
 */
#define BIT_VCHGIN_REG			BITS(5, 1)
#define BIT_DISKIP				BIT(0)



/*
 * MAX77960_CHG_CNFG_00 Mode Select
 */
#define MAX77960_MODE_DEFAULT   0x04
#define MAX77960_MODE_CHGR      0x01
#define MAX77960_MODE_OTG       0x02
#define MAX77960_MODE_BUCK      0x04
#define MAX77960_MODE_BOOST 		0x08

/* MAX77960_CHG_CNFG_02 Fast Charging Current */
#define MAX77960_CHG_CC         0x3F

/* irq */
#define IRQ_DEBOUNCE_TIME       20      /* msec */

/* charger type detection */
#define DET_ERR_RETRY   5
#define DET_ERR_DELAY   200

/* soft charging */
#define SOFT_CHG_START_CURR     100     /* mA */
#define SOFT_CHG_START_DUR      100     /* ms */
#define SOFT_CHG_CURR_STEP      100     /* mA */
#define SOFT_CHG_STEP_DUR       20      /* ms */

#define DEFAULT_AC_CURRENT	1600	/* mA */
#define DEFAULT_USB_CURRENT	500	/* mA */

/* Cable State */
enum {
  /* ta remove */
  CHARGING_TA_NONE = 0,
  /* ta insert */
  CHARGING_TA_INSERTED,
};

enum {
	POWER_SUPPLY_VBUS_UNKNOWN = 0,
	POWER_SUPPLY_VBUS_UVLO,
	POWER_SUPPLY_VBUS_WEAK,
	POWER_SUPPLY_VBUS_OVLO,
	POWER_SUPPLY_VBUS_GOOD,
};

/* Battery State */
enum {
	MAX77960_BAT_NOBAT			= 0x0,
	MAX77960_BAT_PREQUALIFICATION,
	MAX77960_BAT_TIMER_EXPIRED,
	MAX77960_BAT_GOOD,
	MAX77960_BAT_LOWVOLTAGE,
	MAX77960_BAT_OVERVOLTAGE,
	MAX77960_BAT_OVERCURRENT,
	MAX77960_BAT_RESERVED,
};

enum {
	CHG_DTLS_PREQUAL,
	CHG_DTLS_FASTCHARGE_CC,
	CHG_DTLS_FASTCHARGE_CV,
	CHG_DTLS_TOPOFF,
	CHG_DTLS_DONE,
	CHG_DTLS_RESEVRED_05,
	CHG_DTLS_OFF_TIMER_FAULT,
	CHG_DTLS_OFF_SUSPEND,
	CHG_DTLS_OFF_INPUT_INVALID,
	CHG_DTLS_RESERVED_09,
	CHG_DTLS_OFF_JUCTION_TEMP,
	CHG_DTLS_OFF_WDT_EXPIRED,
};


enum max77960_irq {
  MAX77960_OTG_INT = 0,
  MAX77960_DISQBAT_INT,
  MAX77960_CHGINLIM_INT,
  MAX77960_BAT_INT,
  MAX77960_CHGER_INT,
  MAX77960_B2SOVRC_INT,
  MAX77960_CHGIN_INT,
  MAX77960_AICL_INT,
};

enum max77960_irq_source {
      CHG_INT,
      MAX77958_IRQ_GROUP_NR,
};

struct max77960_irq_data {
  int mask;
  enum max77960_irq_source group;
};


struct max77960_charger_data {
	struct device *dev;

	struct regmap *regmap;

	struct regmap_irq_chip_data	*irqc_chg;

	struct power_supply	*psy_chg;

	struct workqueue_struct *wqueue;
	struct work_struct	chgin_work;
	struct delayed_work	isr_work;
	struct delayed_work	recovery_work;	/*  softreg recovery work */
	struct delayed_work	chgin_init_work;	/*  chgin init work */

	/* mutex */
	struct mutex irq_lock;
	struct mutex ops_lock;
	struct mutex i2c_lock;

	unsigned int is_charging;
	unsigned int charging_type;
	unsigned int battery_state;
	unsigned int battery_present;
	unsigned int cable_type;
	unsigned int charging_current_max;
	unsigned int charging_current;
	unsigned int input_current_limit;
	int topoff_timer;
	int topoff_current;
	int fast_charge_current;
	unsigned int vbus_state;
	unsigned int chg_float_voltage;
	unsigned long chg_irq_attr;
	int						details_0;
	int						details_1;
	int						details_2;

	int	aicl_on;
	int	status;
	int	siop_level;
	int uvlo_attach_flag;
	int uvlo_attach_cable_type;

	int irq;
	int irq_gpio;

	int	irq_bypass;
	int	irq_battery;
	int	irq_chg;
	int	irq_chgin;

	/* software regulation */
	bool		soft_reg_state;
	int		soft_reg_current;

	/* unsufficient power */
	bool		reg_loop_deted;

	int		soft_reg_recovery_cnt;

	int pmic_ver;
	int input_curr_limit_step;
	int charging_curr_step;

};


#endif
