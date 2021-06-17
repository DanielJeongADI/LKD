/*
 * This header provides macros for MAXIM MAX77860 device bindings.
 *
 * Copyright (C) 2021 Maxim Integrated. All rights reserved.
 *
 * Author:
 *	Maxim LKD <opensource@maximintegrated.com>
 */

#ifndef _DT_BINDINGS_MFD_MAX77860_H
#define _DT_BINDINGS_MFD_MAX77860_H

/* MAX77860 TOP */
#define TOP_INTSRCMASK 23
#define TOP_SYSINTMASK 26
#define TOP_SAFEOUTCTRL c6

/* MAX77860 PMIC Charger */
#define PMIC_CHG_INT_MASK b1
#define PMIC_CHG_CNFG_00 b7
#define PMIC_CHG_CNFG_01 b8
#define PMIC_CHG_CNFG_02 b9
#define PMIC_CHG_CNFG_03 ba
#define PMIC_CHG_CNFG_04 bb
#define PMIC_CHG_CNFG_05 bc
#define PMIC_CHG_CNFG_06 bd
#define PMIC_CHG_CNFG_07 be
#define PMIC_CHG_CNFG_08 bf
#define PMIC_CHG_CNFG_09 c0
#define PMIC_CHG_CNFG_10 c1
#define PMIC_CHG_CNFG_11 c2
#define PMIC_CHG_CNFG_12 c3

#endif //_DT_BINDINGS_MFD_MAX77860_H
