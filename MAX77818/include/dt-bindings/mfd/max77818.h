/*
 * This header provides macros for MAXIM MAX77714 device bindings.
 *
 * Copyright (C) 2019 Maxim Integrated. All rights reserved.
 *
 * Author:
 *	Daniel Jeong <daniel.jeong@maximintegrated.com>
 *	Maxim LDD <opensource@maximintegrated.com>
 */

#ifndef _DT_BINDINGS_MFD_MAX77818_H
#define _DT_BINDINGS_MFD_MAX77818_H

/* MAX77818 TOP */
#define TOP_INTSRCMASK 23
#define TOP_SYSINTMASK 26
#define TOP_SAFEOUTCTRL c6

/* MAX77818 PMIC Charger */
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

/* MAX77818 FG */
#define FG_STATUS 00
#define FG_VALRT_Th 01
#define FG_TALRT_Th 02
#define FG_SALRT_Th 03
#define FG_AtRate 04

#define FG_QRTable00 12
#define FG_FullSOCThr 13
#define FG_RFAST 15

#define FG_DesignCap 18

#define FG_CONFIG 1d
#define FG_RemCapAv 1e

#define FG_QRTable10 22

#define FG_AIN 27
#define FG_LearnCFG 28
#define FG_FilterCFG 29
#define FG_RelaxCFG 2a
#define FG_MiscCFG 2b
#define FG_TGAIN 2c
#define FG_TOFF 2d
#define FG_CGAIN 2e
#define FG_COFF 2f
#define FG_QRTable20 32

#define FG_FullCapRep 35
#define FG_Iave_empty 36
#define FG_RCOMP0 38
#define FG_TempCo 39
#define FG_V_EMPTY 3A
#define FG_TaskPeriod 3C
#define FG_SHDNTIMER 3F
#define FG_DischargeTH 40
#define FG_QRTable30 42
#define FG_dQ_acc 45
#define FG_dP_acc 46
#define FG_ConvgCFG 49

#define FG_CMD 60
#define FG_TUL1 62
#define FG_TUL2 63
#define FG_OTUL1 6b
#define FG_OTUL2 6c

#define FG_TALRT_Th2 b2
#define FG_CV_MixCap b6
#define FG_CURVE b9
#define FG_HibCFG ba
#define FG_Config2 bb
#define FG_ChargeSgtate0 d1
#define FG_ChargeSgtate1 d2
#define FG_ChargeSgtate2 d3
#define FG_ChargeSgtate3 d4
#define FG_ChargeSgtate4 d5
#define FG_ChargeSgtate5 d6
#define FG_ChargeSgtate6 d7
#define FG_ChargeSgtate7 d8
#define FG_JEITA_Volt d9
#define FG_JEITA_Curr da
#define FG_SmartChgCfg db

#endif //_DT_BINDINGS_MFD_MAX77818_H
