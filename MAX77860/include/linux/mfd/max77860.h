/*
 * Copyright (c) 2021 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <opensource@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77860_MFD_H__
#define __MAX77860_MFD_H__

#define __TEST_DEVICE_NODE__

/* MAX77860 Top Devices */
#define MAX77860_NAME			"max77860"

/* MAX77860 PMIC Devices */
#define MAX77860_REGULATOR_NAME		MAX77860_NAME "-regulator"
#define MAX77860_CHARGER_NAME		MAX77860_NAME "-charger"
#define MAX77860_USBC_NAME		MAX77860_NAME "-usbc"
#define MAX77860_ADC_NAME		MAX77860_NAME "-adc"

/* Register map */
/* PMIC */
#define REG_PMICID		0x00

/* Interrupt register & mask bit */
/* Interrupt Sources */
#define REG_INTSRC		0x22
#define REG_INTSRCMASK		0x23
#define BIT_CHGR_INT		BIT(0)
#define BIT_SYS_INT		BIT(1)
#define BIT_USBC_INT		BIT(3)
#define BIT_B2SOVRC_INT		BIT(5)
#define BIT_SLAVE_INT		BIT(6)

/* System Interrupts */
#define REG_SYSINTSRC		0x24
#define REG_SYSINTMASK		0x26
#define BIT_T120C_INT		BIT(0)
#define BIT_T140C_INT		BIT(1)
#define BIT_LOWSYS_INT		BIT(3)
#define BIT_SYSUVLO_INT		BIT(4)
#define BIT_SYSOVLO_INT		BIT(5)
#define BIT_TSHDN_INT		BIT(6)

/* Charger Interrupts */
#define REG_CHARGER_INT		0xB0
#define REG_CHARGER_INT_MASK	0xB1
#define BIT_CHG_BYP_I		BIT(0)
#define BIT_CHG_BAT2SOC_I	BIT(1)
#define BIT_CHG_BATP_I		BIT(2)
#define BIT_CHG_BAT_I		BIT(3)
#define BIT_CHG_CHG_I		BIT(4)
#define BIT_CHG_TOPOFF_I	BIT(5)
#define BIT_CHG_CHGIN_I		BIT(6)
#define BIT_CHG_AICL_I		BIT(7)

/* BC_INT Interrupts */
#define REG_BC_INT		0x00
#define REG_BC_INT_MASK		0x02
#define BIT_CHG_TYPEI		BIT(0)
#define BIT_DCDTMOI		BIT(1)
#define BIT_PRCHG_TYPI		BIT(2)
#define BIT_CHG_TYP_RUNRI	BIT(3)
#define BIT_CHG_TYP_RUNFI	BIT(4)
#define BIT_DNVDATREFI		BIT(5)
#define BIT_DXOVPI		BIT(6)
#define BIT_VBUSDETI		BIT(7)

/* CC_INT Interrupts */
#define REG_CC_INT		0x01
#define REG_CC_INT_MASK		0x03
#define	BIT_CC_STATI		BIT(0)
#define	BIT_CC_VCN_STATI	BIT(1)
#define	BIT_CC_ISTATI		BIT(2)
#define BIT_CC_PINSTATI		BIT(3)
#define BIT_DET_ABRTI		BIT(5)
#define BIT_VSAFE0VI		BIT(6)

/* BC Status 1 */
#define REG_BC_STATUS1		0x04
#define BIT_CHG_TYPE		BITS(1, 0)
#define BIT_DCDTMO		BIT(2)
#define BIT_PRCHG_TYP		BITS(5, 3)
#define BIT_CHG_TYP_RUN		BIT(6)
#define BIT_VBUSDET		BIT(7)

/* BC Status 2 */
#define REG_BC_STATUS2		0x05
#define BIT_DNVDATREF		BIT(0)
#define BIT_DXOVP		BIT(1)

/* CC Status 1 */
#define REG_CC_STATUS1		0x06
#define BIT_CC_PINSTAT		BITS(7, 6)
#define BIT_CC_ISTAT		BITS(5, 4)
#define BIT_CC_VCN_STAT		BIT(3)
#define BIT_CC_STAT		BITS(2, 0)

/* CC Status 2 */
#define REG_CC_STATUS2		0x07
#define BIT_VSAFE0V_S		BIT(3)
#define BIT_DET_ABRT		BIT(2)

/* CC Control1 */
#define REG_CC_CONTROL1		0x0A
#define BIT_CC_SRC_CUR_CH	BIT(7)
#define BIT_CC_SRC_CUR		BITS(6, 5)
#define BIT_CC_SRC_SNK		BIT(4)
#define BIT_CC_SNK_SRC		BIT(3)
#define BIT_CC_DBG_EN		BIT(2)
#define BIT_CC_AUD_EN		BIT(1)
#define BIT_CC_DET_EN		BIT(0)

/* ADC Registers */
#define REG_ADC_CONFIG1		0x50
#define BITS_MEAS_ADC_COUNT	BITS(1, 0)
#define BIT_CH1_OFFSET_CAL_EN	BIT(2)
#define BIT_CH3_OFFSET_CAL_EN	BIT(3)
#define BIT_CH4_OFFSET_CAL_EN	BIT(4)
#define BITS_ADC_FILTER		BITS(6, 5)
#define BIT_VBUS_HV_RANGE	BIT(7)

#define REG_ADC_CONFIG2		0x51
#define REG_ADC_CONFIG3		0x52
#define REG_ADC_DATA_CH0	0x53
#define REG_ADC_DATA_CH1	0x54
#define REG_ADC_DATA_CH2	0x55
#define REG_ADC_DATA_CH3	0x56
#define REG_ADC_DATA_CH4	0x57
#define REG_ADC_DATA_CH5	0x58
#define REG_ADC_DATA_CH6	0x59
#define REG_ADC_DATA_CH7	0x5A
#define REG_ADC_OFFSET_CH1	0x5B
#define REG_ADC_OFFSET_CH3	0x5C
#define REG_ADC_OFFSET_CH4	0x5D

/* Chip Interrupts */
enum {
	MAX77860_CHGR_INT = 0,
	MAX77860_SYS_INT,
	MAX77860_USBC_INT,
	MAX77860_B2SOVRC_INT,
	MAX77860_SLAVE_INT,

	MAX77860_SYS_IRQ_START,
	MAX77860_SYS_IRQ_T120C = MAX77860_SYS_IRQ_START,
	MAX77860_SYS_IRQ_T140C,
	MAX77860_SYS_IRQ_LOWSYS,
	MAX77860_SYS_IRQ_UVLO,
	MAX77860_SYS_IRQ_OVLO,
	MAX77860_SYS_IRQ_TSHDN,

	MAX77860_CHG_IRQ_START,
	MAX77860_CHG_IRQ_BYP_I = MAX77860_CHG_IRQ_START,
	MAX77860_CHG_IRQ_BAT2SOC_I,
	MAX77860_CHG_IRQ_BATP_I,
	MAX77860_CHG_IRQ_BAT_I,
	MAX77860_CHG_IRQ_CHG_I,
	MAX77860_CHG_IRQ_TOPOFF_I,
	MAX77860_CHG_IRQ_CHGIN_I,
	MAX77860_CHG_IRQ_AICL_CHGINI_I,

	MAX77860_BC_IRQ_START,
	MAX77860_BC_IRQ_CHGTYPE_I = MAX77860_BC_IRQ_START,
	MAX77860_BC_IRQ_DCDTMO_I,
	MAX77860_BC_IRQ_PR_CHG_TYPE_I,
	MAX77860_BC_IRQ_CHGTYP_RUNR_I,
	MAX77860_BC_IRQ_CHGTYP_RUNF_I,
	MAX77860_BC_IRQ_DNVDATREF_I,
	MAX77860_BC_IRQ_DXOVP_I,
	MAX77860_BC_IRQ_VBUSDET_I,

	MAX77860_CC_IRQ_START,
	MAX77860_CC_IRQ_CC_STAT_I = MAX77860_CC_IRQ_START,
	MAX77860_CC_IRQ_CC_VCN_STAT_I,
	MAX77860_CC_IRQ_CC_ISTAT_I,
	MAX77860_CC_IRQ_CC_PIN_STAT_I,
	MAX77860_CC_IRQ_DETABRT_I,
	MAX77860_CC_IRQ_VSAFE0V_I,

	MAX77860_NUM_OF_INTS,
};

enum max77860_chg_type {
	MAX77860_CHG_NOTHING,
	MAX77860_CHG_SDP, /* USB cable attached */
	MAX77860_CHG_CDP, /* Charging downstream port */
	MAX77860_CHG_DCP, /* Dedicated charger */
};

enum max77860_pr_chg_type {
	MAX77860_PR_CHG_UNKNOWN,
	MAX77860_PR_CHG_SAMSUNG_2A,
	MAX77860_PR_CHG_APPLE_0_5A,
	MAX77860_PR_CHG_APPLE_1A,
	MAX77860_PR_CHG_APPLE_2A,
	MAX77860_PR_CHG_APPLE_12W,
	MAX77860_PR_CHG_3A_DCP,
	MAX77860_PR_CHG_RFU,
};

enum max77860_cc_stat_type {
	MAX77860_CC_NO_CONN,
	MAX77860_CC_UFP,
	MAX77860_CC_DFP,
	MAX77860_CC_AUDIO,
	MAX77860_CC_DEBUG,
	MAX77860_CC_ERROR,
	MAX77860_CC_DISABLED,
	MAX77860_CC_RFU,
};

enum max77860_cc_i_stat_type {
	MAX77860_CC_I_NOT_UFP,
	MAX77860_CC_I_500_MA,
	MAX77860_CC_I_1_5A,
	MAX77860_CC_I_3A,
};

enum max77860_cc_pin_stat_type {
	MAX77860_CC_PIN_NO_DET,
	MAX77860_CC_PIN_CC1,
	MAX77860_CC_PIN_CC2,
	MAX77860_CC_PIN_RFU,
};

/*******************************************************************************
 * Useful Macros
 ******************************************************************************/

#undef  __CONST_FFS
#define __CONST_FFS(_x) \
		((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
						((_x) & 0x04 ? 2 : 3)) :\
		((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
						((_x) & 0x40 ? 6 : 7)))

#undef  FFS
#define FFS(_x) \
		((_x) ? __CONST_FFS(_x) : 0)

#undef  BIT_RSVD
#define BIT_RSVD  0

#undef  BITS
#define BITS(_end, _start) \
	((BIT(_end) - BIT(_start)) + BIT(_end))

#undef  __BITS_GET
#define __BITS_GET(_word, _mask, _shift) \
	(((_word) & (_mask)) >> (_shift))

#undef  BITS_GET
#define BITS_GET(_word, _bit) \
	__BITS_GET(_word, _bit, FFS(_bit))

#undef  __BITS_SET
#define __BITS_SET(_word, _mask, _shift, _val) \
	(((_word) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))

#undef  BITS_SET
#define BITS_SET(_word, _bit, _val) \
	__BITS_SET(_word, _bit, FFS(_bit), _val)

#undef  BITS_MATCH
#define BITS_MATCH(_word, _bit) \
	(((_word) & (_bit)) == (_bit))

/*******************************************************************************
 * Sub Modules Support
 ******************************************************************************/
enum {
	MAX77860_DEV_REGULATOR = 0,
	MAX77860_DEV_CHARGER,
	MAX77860_DEV_USBC,
	MAX77860_DEV_NUM_OF_DEVICES,
};

struct max77860_dev {
	void				*pdata;
	struct mutex			lock;	/* device mutex */
	struct device			*dev;

	int				irq;
	int				irq_gpio;

	struct regmap_irq_chip_data	*irqc_intsrc;
	struct regmap_irq_chip_data	*irqc_sys;
	struct regmap_irq_chip_data	*irqc_chg;
	struct regmap_irq_chip_data	*irqc_usbc;

	struct i2c_client	*pmic;		/* 0xCC , CLOGIC/SAFELDOS */
	struct i2c_client	*chg;		/* 0xD2, CHARGER */
	struct i2c_client	*usbc;		/* 0x4A, USBC */

	struct regmap		*regmap_pmic;	/* CLOGIC/SAFELDOS */
	struct regmap		*regmap_chg;	/* CHARGER */
	struct regmap		*regmap_usbc;	/* USBC */
};

/*******************************************************************************
 * Chip IO
 ******************************************************************************/
int max77860_read(struct regmap *regmap, u8 addr, u8 *val);
int max77860_write(struct regmap *regmap, u8 addr, u8 val);
int max77860_bulk_read(struct regmap *regmap, u8 addr, u8 *dst, u16 len);
int max77860_bulk_write(struct regmap *regmap, u8 addr, const u8 *src, u16 len);

/*******************************************************************************
 * Interrupt
 ******************************************************************************/
int max77860_irq_init(struct max77860_dev *max77860);
void max77860_irq_exit(struct max77860_dev *max77860);
int max77860_irq_resume(struct max77860_dev *max77860);

int max77860_map_irq(struct max77860_dev *max77860, int irq);

#endif /* !__MAX77860_MFD_H__ */
