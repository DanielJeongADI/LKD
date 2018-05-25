/*
 * MAX77696 MFD Driver Header File
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

#ifndef __MAX77696_H__
#define __MAX77696_H__

#include <linux/version.h>
#include <linux/leds.h>
#include <linux/regulator/machine.h>

/* MAX77696 variant might be defined by Kconfig */
//      CONFIG_MAX77696
//      CONFIG_MAX77697
//      CONFIG_MAX77796

#ifdef CONFIG_OF
#define CONFIG_MAX77696_DT
#endif /* CONFIG_OF */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#ifndef __devinit
#define __devinit
#endif
#ifndef __devexit
#define __devexit
#endif
#ifndef __devexit_p
#define __devexit_p(x)  (x)
#endif
#endif /* LINUX_VERSION_CODE ... */

#define MAX77696_DRIVER_VERSION            "2.13"

/* Driver Core */
#if defined(CONFIG_MAX77696)
#define MAX77696_DESC                      "MAX77696"
#define MAX77696_NAME                      "max77696"
#elif defined(CONFIG_MAX77697)
#define MAX77696_DESC                      "MAX77697"
#define MAX77696_NAME                      "max77697"
#elif defined(CONFIG_MAX77796)
#define MAX77696_DESC                      "MAX77796"
#define MAX77696_NAME                      "max77796"
#else
#error "MAX77696 VARIANT NOT DEFINED"
#endif
#define MAX77696_CORE_NAME                 MAX77696_NAME"-core"
#define MAX77696_TOPINT_NAME               MAX77696_NAME"-irq"
#define MAX77696_TOPSYS_NAME               MAX77696_NAME"-topsys"
#define MAX77696_DBG_NAME                  MAX77696_NAME"-dbg"

/* PMIC (7'h3C) */
#define MAX77696_GPIO_NAME                 MAX77696_NAME"-gpio"
#define MAX77696_CLK_NAME                  MAX77696_NAME"-clk"
#define MAX77696_WDT_NAME                  MAX77696_NAME"-wdt"
#define MAX77696_ADC_NAME                  MAX77696_NAME"-adc"
#define MAX77696_BUCK_NAME                 MAX77696_NAME"-bucks"
#define MAX77696_BBST_NAME                 MAX77696_NAME"-bbsts"
#define MAX77696_LDO_NAME                  MAX77696_NAME"-ldos"
#define MAX77696_LSW_NAME                  MAX77696_NAME"-lsws"
#define MAX77696_EPD_NAME                  MAX77696_NAME"-epds"
#define MAX77696_VDDQ_NAME                 MAX77696_NAME"-vddq"
#define MAX77696_LED_NAME                  MAX77696_NAME"-leds"
#define MAX77696_WLED_NAME                 MAX77696_NAME"-bl"
#define MAX77696_CHG_NAME                  MAX77696_NAME"-charger"

/* RTC (7'h68) */
#define MAX77696_RTC_NAME                  MAX77696_NAME"-rtc"
#define MAX77696_EH_NAME                   MAX77696_NAME"-eh"

/* UIC (7'h35) */
#define MAX77696_UIC_NAME                  MAX77696_NAME"-uic"

/* FG (7'h34) */
#define MAX77696_FG_NAME                   MAX77696_NAME"-battery"

/* TOPSYS-derived */
#define MAX77696_KEY_NAME                  MAX77696_NAME"-key"

/* MAX77696 Core Dev Attr Group Name */
#define MAX77696_CORE_ATTR_GROUP_NAME      MAX77696_CORE_NAME

enum {
    /* core */
    MAX77696_BLK_CORE = 0,
    MAX77696_BLK_TOPINT,
    MAX77696_BLK_TOPSYS,
    /* pmic */
    MAX77696_BLK_GPIO,
    MAX77696_BLK_CLK,
    MAX77696_BLK_WDT,
    MAX77696_BLK_BUCK,
    MAX77696_BLK_BBST,
    MAX77696_BLK_LDO,
    MAX77696_BLK_LSW,
    MAX77696_BLK_EPD,
    MAX77696_BLK_VDDQ,
    MAX77696_BLK_LED,
    MAX77696_BLK_WLED,
    MAX77696_BLK_ADC,
    MAX77696_BLK_CHG,
    MAX77696_BLK_EH,
    /* rtc */
    MAX77696_BLK_RTC,
    /* uic */
    MAX77696_BLK_UIC,
    /* fuelgauge */
    MAX77696_BLK_FG,
    /* topsys-derived */
    MAX77696_BLK_KEY,
    /***/
    MAX77696_NUM_OF_BLOCKS
};

/*******************************************************************************
 * Top Interrupts
 ******************************************************************************/

/* First-level (Zero-depth) Interrupts */
enum {
    MAX77696_IRQ_TOPSYS    =  0,
    MAX77696_IRQ_BUCK,    /*  1 */
    MAX77696_IRQ_FG,      /*  2 */
    MAX77696_IRQ_GPIO,    /*  3 */
    MAX77696_IRQ_RTC,     /*  4 */
    MAX77696_IRQ_CHGA,    /*  5 */
    MAX77696_IRQ_LDO,     /*  6 */
    MAX77696_IRQ_UIC,     /*  7 */
    MAX77696_IRQ_ADC,     /*  8 */
    MAX77696_IRQ_WLED,    /*  9 */
    MAX77696_IRQ_EPD,     /* 10 */
    MAX77696_IRQ_CHGB,    /* 11 */
    /***/
    MAX77696_NUM_OF_IRQS,
};

struct max77696_topint_platform_data {
    int            irq;
    unsigned long  irq_trigger;
    unsigned int   irq_base;
};

/*******************************************************************************
 * Top System Management
 ******************************************************************************/

/* TOPSYS Interrupts */
enum {
    MAX77696_TOPSYS_IRQ_THERM_ALARM_1   = 0,
    MAX77696_TOPSYS_IRQ_THERM_ALARM_0, /* 1 */
    MAX77696_TOPSYS_IRQ_BATT_LOW,      /* 2 */
    MAX77696_TOPSYS_IRQ_MR_WARNING,    /* 3 */
    MAX77696_TOPSYS_IRQ_EN0_1SEC,      /* 4 */
    MAX77696_TOPSYS_IRQ_EN0_FALLING,   /* 5 */
    MAX77696_TOPSYS_IRQ_EN0_RISING,    /* 6 */
    /***/
    MAX77696_NUM_OF_TOPSYS_IRQS,
};

/* TOPSYS status */

enum {
    /***/
    MAX77696_TOPSYS_NUM_OF_STATS,
};

/* TOPSYS config */

enum {
    MAX77696_TOPSYS_CNFG_IRQ_PIN_DIS,

    MAX77696_TOPSYS_CNFG_PTP,
    MAX77696_TOPSYS_CNFG_FSENT,
    MAX77696_TOPSYS_CNFG_FRSTRT,
    MAX77696_TOPSYS_CNFG_PRSTRT,
    MAX77696_TOPSYS_CNFG_FSHDN,
    MAX77696_TOPSYS_CNFG_PSHDN,
    MAX77696_TOPSYS_CNFG_SFTPDRR,

    MAX77696_TOPSYS_CNFG_GLBL_LPM,
//  MAX77696_TOPSYS_CNFG_MREN,
    MAX77696_TOPSYS_CNFG_MRT,
    MAX77696_TOPSYS_CNFG_EN0DLY,
    MAX77696_TOPSYS_CNFG_STBYEN,

    MAX77696_TOPSYS_CNFG_WDTEN,
    MAX77696_TOPSYS_CNFG_TWD,
    MAX77696_TOPSYS_CNFG_RTCAWK,
    MAX77696_TOPSYS_CNFG_WDWK,
    MAX77696_TOPSYS_CNFG_MROWK,
    MAX77696_TOPSYS_CNFG_UICWK_EDGE,

    MAX77696_TOPSYS_CNFG_LBHYST,
    MAX77696_TOPSYS_CNFG_LBDAC,

    MAX77696_TOPSYS_CNFG_WDTC,

    /***/
    MAX77696_NUM_OF_TOPSYS_CNFGS,
};

struct max77696_topsys_platform_data {
    unsigned int irq_base;
};

/*******************************************************************************
 * GPIOs
 ******************************************************************************/

/* Number of GPIOs */
#define MAX77696_NUM_OF_GPIOS               5

/* GPIO alternate mode */
#define MAX77696_GPIO_MODE_STDGPIO          0b00 /* Standard GPI or GPO */
#define MAX77696_GPIO_MODE_0                0b01
#define MAX77696_GPIO_MODE_1                0b10
#define MAX77696_GPIO_MODE_2                0b11

/* GPIO direction */
#define MAX77696_GPIO_DIR_OUTPUT            0
#define MAX77696_GPIO_DIR_INPUT             1

/* GPIO output drive configuration */
#define MAX77696_GPIO_DRIVE_OPENDRAIN       0
#define MAX77696_GPIO_DRIVE_PUSHPULL        1

/* GPIO output level configuration */
#define MAX77696_GPIO_LEVEL_LOW             0
#define MAX77696_GPIO_LEVEL_HIGH            1

/* GPIO input debounce configuration */
#define MAX77696_GPIO_DBNC_0_MSEC           0b00
#define MAX77696_GPIO_DBNC_8_MSEC           0b01
#define MAX77696_GPIO_DBNC_16_MSEC          0b10
#define MAX77696_GPIO_DBNC_32_MSEC          0b11

/* GPIO input interrupt configuration */
#define MAX77696_GPIO_INTCNFG_DISABLE       0b00
#define MAX77696_GPIO_INTCNFG_FALLING_EDGE  0b01
#define MAX77696_GPIO_INTCNFG_RISING_EDGE   0b10
#define MAX77696_GPIO_INTCNFG_BOTH_EDGE     \
        (MAX77696_GPIO_INTCNFG_FALLING_EDGE|MAX77696_GPIO_INTCNFG_RISING_EDGE)

struct max77696_gpio_cfg_data {
    unsigned gpio;  /* 0 <= n < MAX77696_NUM_OF_GPIOS */

    u8 alter_mode;  /* GPIO Alternate Mode Enable (see datasheet for details) */
    u8 pullup_en;   /* GPIO Pull-Up Enable */
    u8 pulldn_en;   /* GPIO Pull-Dn Enable */
    u8 direction;   /* GPIO Direction */

    union {
        struct {
            u8 drive;    /* Output drive configuration */
            u8 level;    /* Output level (0:LOW / 1:HIGH) */
        } output;
        struct {
            u8 dbnc;     /* Input debounce configuration */
            u8 intcnfg;  /* Input interrupt configuration */
        } input;
    } u;
};

struct max77696_gpio_platform_data {
    int                            gpio_base;
    unsigned int                   irq_base;
    struct max77696_gpio_cfg_data *cfg_data;
    size_t                         num_of_cfg_data;
};

/*******************************************************************************
 * 32KHz Clock Buffer
 ******************************************************************************/

/* Load capacitance selection */
#define MAX77696_CLK_LOAD_CAP_22PF         0 /* 22pF per node */
#define MAX77696_CLK_LOAD_CAP_12PF         1 /* 12pF per node */
#define MAX77696_CLK_LOAD_CAP_10PF         2 /* 10pF per node */
#define MAX77696_CLK_LOAD_CAP_NONE         3 /* No internal load cap selected */

/* Operation mode
 *   Whenever the main battery is plugged in and VSYS > VSYSMIN, and the
 *   MODE_SEL bit is set to 1, the crystal driver goes into a high current, high
 *   accuracy state that meets the 15ns cycle by cycle jitter and 45%. 55% duty
 *   cycle spec. As soon as VSYS < VSYSMIN, the oscillator automatically goes
 *   into a low power state where it does not meet the jitter and duty cycle
 *   specification. The assumption is that when VSYS < VSYSMIN, the radio is
 *   off, and tight jitter and duty cycle spec is not required. This prevents
 *   the battery from being too deeply discharge, since the supply current is
 *   reduced.
 */
#define MAX77696_CLK_OPMODE_LOW_POWER      0 /* Low Power */
#define MAX77696_CLK_OPMODE_LOW_JITTER     1 /* Low Jitter */

struct max77696_clk_platform_data {
    u8 load_cap; /* Load capacitance selection */
    u8 op_mode;  /* Operation mode             */
};

/*******************************************************************************
 * Watchdog Timer
 ******************************************************************************/

struct max77696_wdt_platform_data {
    u32 timeout_sec;
    u32 ping_interval_sec; /* If watchdog daemon is used, then set this zero */
};

/*******************************************************************************
 * ADC
 ******************************************************************************/

enum {
    MAX77696_ADC_CH_VSYS2 = 0,
    MAX77696_ADC_CH_TDIE,
    MAX77696_ADC_CH_VSYS1,
    MAX77696_ADC_CH_VCHGINA,
    MAX77696_ADC_CH_ICHGINA,
    MAX77696_ADC_CH_IMONL1,
    MAX77696_ADC_CH_IMONL2,
    MAX77696_ADC_CH_IMONL3,
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    MAX77696_ADC_CH_IMONL4,
    MAX77696_ADC_CH_IMONL5,
    MAX77696_ADC_CH_IMONL6,
    MAX77696_ADC_CH_IMONL7,
    MAX77696_ADC_CH_IMONL8,
    MAX77696_ADC_CH_IMONL9,
    MAX77696_ADC_CH_IMONL10,
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
    MAX77696_ADC_CH_IMONB1,
    MAX77696_ADC_CH_IMONB2,
    MAX77696_ADC_CH_IMONB3,
    MAX77696_ADC_CH_IMONB4,
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    MAX77696_ADC_CH_IMONB5,
    MAX77696_ADC_CH_IMONB6,
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
    MAX77696_ADC_CH_AIN0,
    MAX77696_ADC_CH_AIN1,
    MAX77696_ADC_CH_AIN2,
    MAX77696_ADC_CH_AIN3,
    /***/
    MAX77696_NUM_OF_ADC_CHS,
};

#define MAX77696_ADC_PRINT_RAW    0b00000001 /* integer */
#define MAX77696_ADC_PRINT_SCALED 0b00000010 /* integer (milli-scaled) */
#define MAX77696_ADC_PRINT_FULL   0b00000100 /* double + unit string */
#define MAX77696_ADC_PRINT_ALL    0b11111111

struct max77696_adc_platform_data {
    u8    print_fmt; /* cf. MAX77696_ADC_PRINT_... */
    char *vref_name; /* name of ADC Vref regulator */
};

/*******************************************************************************
 * BUCK Regulators
 ******************************************************************************/

#define MAX77696_BUCK_VALID_MODES \
        REGULATOR_MODE_FAST |\
        REGULATOR_MODE_NORMAL |\
        REGULATOR_MODE_IDLE |\
        REGULATOR_MODE_STANDBY
#define MAX77696_BUCK_VALID_OPS \
        REGULATOR_CHANGE_VOLTAGE |\
        REGULATOR_CHANGE_STATUS |\
        REGULATOR_CHANGE_MODE |\
        REGULATOR_CHANGE_DRMS

enum {
    MAX77696_VREG_BUCK1 = 0,
    MAX77696_VREG_BUCK2,
    MAX77696_VREG_BUCK3,
    MAX77696_VREG_BUCK4,
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    MAX77696_VREG_BUCK5,
    MAX77696_VREG_BUCK6,
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
    /***/
    MAX77696_VREG_NUM_OF_BUCKS,
};

/* Rising Slew Rate */
#define MAX77696_BUCK_RSR_12500    0b00 /* 12.5 mV/us */
#define MAX77696_BUCK_RSR_25000    0b01 /* 25   mV/us */
#define MAX77696_BUCK_RSR_50000    0b10 /* 50   mV/us */
#define MAX77696_BUCK_RSR_DISABLE  0b11 /* no control */

/* Falling Slew Rate */
#define MAX77696_BUCK_FSR_ENABLE   0
#define MAX77696_BUCK_FSR_DISABLE  1

struct max77696_buck_cfg_data {
    u8                         buck_id;  /* MAX77696_VREG_BUCKx */
    u8                         rsr;      /* Rising Slew Rate */
    u8                         fsr;      /* Falling Slew Rate */
    int                        dvs_uV;   /* Vout in uV when DVS */
    struct regulator_init_data init_data;
};

struct max77696_bucks_platform_data {
    struct max77696_buck_cfg_data *cfg_data;
    size_t                         num_of_cfg_data;
};

#if defined(CONFIG_MAX77796)
/*******************************************************************************
 * BUCK Boost Regulators
 ******************************************************************************/

#define MAX77696_BBST_VALID_MODES \
        REGULATOR_MODE_FAST |\
        REGULATOR_MODE_NORMAL
#define MAX77696_BBST_VALID_OPS \
        REGULATOR_CHANGE_VOLTAGE |\
        REGULATOR_CHANGE_MODE |\
        REGULATOR_CHANGE_DRMS

enum {
    MAX77696_VREG_BBST1 = 0,
    /***/
    MAX77696_VREG_NUM_OF_BBSTS,
};

struct max77696_bbst_cfg_data {
    u8                         bbst_id;  /* MAX77696_VREG_BBSTx */
    struct regulator_init_data init_data;
};

struct max77696_bbsts_platform_data {
    struct max77696_bbst_cfg_data *cfg_data;
    size_t                         num_of_cfg_data;
};
#endif /* CONFIG_MAX77796 */

/*******************************************************************************
 * LDO Regulators
 ******************************************************************************/

#define MAX77696_LDO_VALID_MODES \
        REGULATOR_MODE_NORMAL |\
        REGULATOR_MODE_IDLE |\
        REGULATOR_MODE_STANDBY
#define MAX77696_LDO_VALID_OPS \
        REGULATOR_CHANGE_VOLTAGE |\
        REGULATOR_CHANGE_STATUS |\
        REGULATOR_CHANGE_MODE |\
        REGULATOR_CHANGE_DRMS

enum {
    MAX77696_VREG_LDO1 = 0,
    MAX77696_VREG_LDO2,
    MAX77696_VREG_LDO3,
    MAX77696_VREG_LDO4,
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    MAX77696_VREG_LDO5,
    MAX77696_VREG_LDO6,
    MAX77696_VREG_LDO7,
    MAX77696_VREG_LDO8,
    MAX77696_VREG_LDO9,
    MAX77696_VREG_LDO10,
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
    /***/
    MAX77696_VREG_NUM_OF_LDOS,
};

/* Soft-Start Slew Rate */
#define MAX77696_LDO_SSSR_100000  0 /* 100 mV/us */
#define MAX77696_LDO_SSSR_5000    1 /*   5 mV/us */

struct max77696_ldo_cfg_data {
    u8                         ldo_id;   /* MAX77696_VREG_LDOx */
    u8                         sssr;     /* Soft-Start Slew Rate */
    struct regulator_init_data init_data;
};

struct max77696_ldos_platform_data {
    struct max77696_ldo_cfg_data *cfg_data;
    size_t                        num_of_cfg_data;
};

/*******************************************************************************
 * Load Switches
 ******************************************************************************/

#define MAX77696_LSW_VALID_MODES \
        0
#define MAX77696_LSW_VALID_OPS \
        REGULATOR_CHANGE_STATUS

enum {
    MAX77696_VREG_LSW1 = 0,
    MAX77696_VREG_LSW2,
    MAX77696_VREG_LSW3,
    MAX77696_VREG_LSW4,
    /***/
    MAX77696_VREG_NUM_OF_LSWS,
};

/* Output Rise Time */
#define MAX77696_LSW_RT_300000  0b00 /* 300 mV/us */
#define MAX77696_LSW_RT_100000  0b01 /* 100 mV/us */
#define MAX77696_LSW_RT_30000   0b10 /*  30 mV/us */
#define MAX77696_LSW_RT_10000   0b11 /*  10 mV/us */

struct max77696_lsw_cfg_data {
    u8                         lsw_id;    /* MAX77696_VREG_LSWx */
    u8                         rt;        /* Output Rise Time */
    struct regulator_init_data init_data;
};

struct max77696_lsws_platform_data {
    struct max77696_lsw_cfg_data *cfg_data;
    size_t                        num_of_cfg_data;
};

/*******************************************************************************
 * EPD Supplies
 ******************************************************************************/

#define MAX77696_EPD_VID_OFFSET  (50*1000*1000)

/* micro-volts    -> EPD voltage ID */
#define MAX77696_EPD_VID(_uV)    ((_uV)  + MAX77696_EPD_VID_OFFSET)

/* EPD voltage ID -> micro-volts    */
#define MAX77696_EPD_UV(_vid)    ((_vid) - MAX77696_EPD_VID_OFFSET)

#define MAX77696_EPD_VALID_MODES \
        0

enum {
    MAX77696_VREG_EPD_DISP = 0,
    MAX77696_VREG_EPD_VCOM,
    MAX77696_VREG_EPD_VEE,
    MAX77696_VREG_EPD_VNEG,
    MAX77696_VREG_EPD_VPOS,
    MAX77696_VREG_EPD_VDDH,
    /***/
    MAX77696_VREG_NUM_OF_EPDS,
};

struct max77696_epd_cfg_data {
    u8                         epd_id;    /* MAX77696_VREG_EPD_x */
    struct regulator_init_data init_data;
};

struct max77696_epds_platform_data {
    struct max77696_epd_cfg_data *cfg_data;
    size_t                        num_of_cfg_data;
};

/*******************************************************************************
 * LPDDR2 VREFDQ Supply
 ******************************************************************************/

#define MAX77696_VDDQ_MIN(_input_uV) \
        (((_input_uV)/2)-((60*((_input_uV)/2))/100))
#define MAX77696_VDDQ_MAX(_input_uV) \
        (((_input_uV)/2)+((64*((_input_uV)/2))/100))

struct max77696_vddq_platform_data {
    u32                        input_uV;
    struct regulator_init_data init_data;
};

/*******************************************************************************
 * Charger
 ******************************************************************************/

struct max77696_chg_platform_data {
    char    *psy_name;
    char   **supplied_to;
    size_t   num_supplicants;
	char   **supplied_from;
	size_t   num_supplies;
};

#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
/*******************************************************************************
 * Energy Harvester
 ******************************************************************************/

#define MAX77696_EH_MODE_SHUTDOWN   0b00
#define MAX77696_EH_MODE_CHARGER    0b01
#define MAX77696_EH_MODE_ACCESSORY  0b10
#define MAX77696_EH_MODE_AUTODETECT 0b11

struct max77696_eh_platform_data {
    char    *psy_name;
    char   **supplied_to;
    size_t   num_supplicants;
	char   **supplied_from;
	size_t   num_supplies;

    u8       mode;
    u32      acc_ilimit;          /* 200 or 650 mA */
    u32      acc_det_debounce_ms;
    int      acc_det_gpio_assert; /* HIGH(1) or LOW (0) */
    int      acc_det_gpio;
};
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */

/*******************************************************************************
 * LED Indicators
 ******************************************************************************/

enum {
    MAX77696_LED_GREEN = 0,
    MAX77696_LED_AMBER,
    /***/
    MAX77696_NUM_OF_LEDS,
};

struct max77696_led_cfg {
    bool                manual;
    enum led_brightness brightness;
    u32                 delay_on;
    u32                 delay_off;
};

struct max77696_led_init_data {
    int                     led_id; /* 0...MAX77696_NUM_OF_LEDS */
    struct led_info         info;
    struct max77696_led_cfg cfg;
};

struct max77696_leds_platform_data {
    struct max77696_led_init_data *init_data;
    size_t                         num_of_init_data;
};

/*******************************************************************************
 * Backlighting: WLED Boost Regulator
 ******************************************************************************/

struct max77696_bl_platform_data {
    u32 boot_brightness;
};

/*******************************************************************************
 * Real-Time Clock
 ******************************************************************************/

struct max77696_rtc_platform_data {
    u32  smpl_msec; /* SMPL timer duration ; msec */
};

/*******************************************************************************
 * UIC
 ******************************************************************************/

/* UIC Interrupt Type */
#define MAX77696_UIC_INTTYP_LEVEL           0 /* Interrupt is level triggered */
#define MAX77696_UIC_INTTYP_EDGE            1 /* Interrupt is  edge triggered */

/* UIC Interrupt Delay */
#define MAX77696_UIC_INTDLY_2TICKS          0 /* 2 x 60KHz-clock ticks */
#define MAX77696_UIC_INTDLY_4TICKS          1 /* 4 x 60KHz-clock ticks */

/* UIC Interrupt Polarity */
#define MAX77696_UIC_INTPOL_ACTIVE_LOW      0
#define MAX77696_UIC_INTPOL_ACTIVE_HIGH     1

struct max77696_uic_platform_data {
    u16   int_type;
    u16   int_delay;    /* valid only if int_type = 1 */
    u16   int_polarity;
    char *chg_psy_name; /* psy class dev name for max77696-charger */
};

/*******************************************************************************
 * FuelGauge
 ******************************************************************************/

struct max77696_battery_ini {
    char *title;
    u16   learncfg;
    u16   capacity; /* mAh */
    u16   misccfg;
    u16   ichgterm;
    u16   filtercfg;
    u16   fullsocthr;
    u16   relaxcfg;
    u16   dq_acc;
    u16   dp_acc;
    u16   qrtable00;
    u16   qrtable10;
    u16   qrtable20;
    u16   qrtable30;
    u16   rcomp0;
    u16   tempco;
    u16   v_empty;
    u16   iavg_empty;
    u16   modeldata[3*16];
};

struct max77696_battery_platform_data {
    char                         *psy_name;
    char                        **supplied_to;
    size_t                        num_supplicants;
	char                        **supplied_from;
	size_t                        num_supplies;

    u16                           r_sns;      /* milli-ohms; default 10 */

    /* The Alert Threshold values
     *   - Over/Under voltage       VALRT threshold violation (upper or lower)
     *   - Over/Under temperature   TALRT threshold violation (upper or lower)
     *   - Over/Under SOC           SALRT threshold violation (upper or lower)
     * (the alert will be disabled if max and min are same)
     */
    s32                           valrt_max, valrt_min; /* 5100 ~    0 [mV] */
    s32                           talrt_max, talrt_min; /*  127 ~ -128 [ C] */
    s32                           salrt_max, salrt_min; /*  255 ~    0 [ %] */

	struct max77696_battery_ini  *ini;
	size_t                        num_ini;
};

/*******************************************************************************
 * Key Input
 ******************************************************************************/

enum {
    MAX77696_KEY_EN0 = 0,
    MAX77696_KEY_1SEC,
    MAX77696_KEY_MRWRN,
    /***/
    MAX77696_KEY_NUM_OF_KEYS,
};

struct max77696_key_platform_data {
    u32          mrt_sec;  /* 2s ... 12s */
    unsigned int keycode[MAX77696_KEY_NUM_OF_KEYS];
};

/*******************************************************************************
 * Platform Data
 ******************************************************************************/

struct max77696_platform_data {
    int                                    intb_gpio;
    int                                    io_busnum;

    struct max77696_topint_platform_data  *topint_pdata;
    struct max77696_topsys_platform_data  *topsys_pdata;

    struct max77696_gpio_platform_data    *gpio_pdata;
    struct max77696_clk_platform_data     *clk_pdata;
    struct max77696_wdt_platform_data     *wdt_pdata;
    struct max77696_adc_platform_data     *adc_pdata;
    struct max77696_bucks_platform_data   *bucks_pdata;
#if defined(CONFIG_MAX77796)
    struct max77696_bbsts_platform_data   *bbsts_pdata;
#endif /* CONFIG_MAX77796 */
    struct max77696_ldos_platform_data    *ldos_pdata;
    struct max77696_lsws_platform_data    *lsws_pdata;
    struct max77696_epds_platform_data    *epds_pdata;
    struct max77696_vddq_platform_data    *vddq_pdata;
    struct max77696_chg_platform_data     *chg_pdata;
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    struct max77696_eh_platform_data      *eh_pdata;
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
    struct max77696_leds_platform_data    *leds_pdata;
    struct max77696_bl_platform_data      *bl_pdata;

    struct max77696_rtc_platform_data     *rtc_pdata;
    struct max77696_uic_platform_data     *uic_pdata;
    struct max77696_battery_platform_data *battery_pdata;

    struct max77696_key_platform_data     *key_pdata;
};

/*******************************************************************************
 * Useful Macros
 ******************************************************************************/

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
#if 0
#define FFS(_x) \
        ((_x) ? (__builtin_constant_p(_x) ? __CONST_FFS(_x) : __ffs(_x)) : 0)
#else
#define FFS(_x) ((_x) ? __CONST_FFS(_x) : 0)
#endif

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

#ifdef CONFIG_OF
static __always_inline int of_property_u8 (const struct device_node *np,
    const char *propname, u8 *out_value)
{
    u32 buf;
    int rc = of_property_read_u32(np, propname, &buf);
    if (likely(!rc)) {
        *out_value = (u8)buf;
    }
    return rc;
}

static __always_inline int of_property_u16 (const struct device_node *np,
    const char *propname, u16 *out_value)
{
    u32 buf;
    int rc = of_property_read_u32(np, propname, &buf);
    if (likely(!rc)) {
        *out_value = (u16)buf;
    }
    return rc;
}

static __always_inline int of_property_u32 (const struct device_node *np,
    const char *propname, u32 *out_value)
{
    return of_property_read_u32(np, propname, out_value);
}
#endif /* CONFIG_OF */

/*******************************************************************************
 * Chip IO
 ******************************************************************************/

struct max77696_io {
    struct device *dev;

    int (*read) (struct max77696_io*, u16, u16*);
    int (*write) (struct max77696_io*, u16, u16);
    int (*bulk_read) (struct max77696_io*, u16, u8*, u16);
    int (*bulk_write) (struct max77696_io*, u16, const u8*, u16);
};

static __always_inline int max77696_read (struct max77696_io *io,
    u16 addr, u16 *val)
{
    return io->read(io, addr, val);
}

static __always_inline int max77696_write (struct max77696_io *io,
    u16 addr, u16 val)
{
    return io->write(io, addr, val);
}

static __inline int max77696_masked_read (struct max77696_io *io,
    u16 addr, u16 mask, u16 shift, u16 *val)
{
    u16 buf = 0;
    int rc;

    if (unlikely(!mask)) {
        /* no actual access */
        *val = 0;
        rc   = 0;
        goto out;
    }

    rc = max77696_read(io, addr, &buf);
    if (likely(!rc)) {
        *val = __BITS_GET(buf, mask, shift);
    }

out:
    return rc;
}

static __inline int max77696_masked_write (struct max77696_io *io,
    u16 addr, u16 mask, u16 shift, u16 val)
{
    u16 buf = 0;
    int rc;

    if (unlikely(!mask)) {
        /* no actual access */
        rc = 0;
        goto out;
    }

    rc = max77696_read(io, addr, &buf);
    if (likely(!rc)) {
        rc = max77696_write(io, addr, __BITS_SET(buf, mask, shift, val));
    }

out:
    return rc;
}

static __always_inline int max77696_bulk_read (struct max77696_io *io,
    u16 addr, u8 *dst, u16 len)
{
    return io->bulk_read(io, addr, dst, len);
}

static __always_inline int max77696_bulk_write (struct max77696_io *io,
    u16 addr, const u8 *src, u16 len)
{
    return io->bulk_write(io, addr, src, len);
}

/*** Simplifying bitwise configurations for individual subdevice drivers ***/

#ifndef MAX77696_REG_RSVD
#define MAX77696_REG_RSVD  0
#endif

struct max77696_bitdesc {
    u16 reg;
    u16 mask, shift;
};

#define MAX77696_BITDESC(_reg, _bit) \
        { .reg = _reg, .mask = _bit, .shift = (u16)FFS(_bit), }

#define MAX77696_BITDESC_INVALID \
        MAX77696_BITDESC(MAX77696_REG_RSVD, BIT_RSVD)

#define __check_valid_max77696_bitdesc(_bitdesc) \
        ((_bitdesc)->mask != BIT_RSVD &&\
         (_bitdesc)->reg  != MAX77696_REG_RSVD)

static __always_inline int max77696_read_bitdesc (struct max77696_io *io,
    const struct max77696_bitdesc *desc, u16 *val)
{
    return max77696_masked_read(io,
        desc->reg, desc->mask, desc->shift, val);
}

static __always_inline int max77696_write_bitdesc (struct max77696_io *io,
    const struct max77696_bitdesc *desc, u16 val)
{
    return max77696_masked_write(io,
        desc->reg, desc->mask, desc->shift, val);
}

#define max77696_read_reg_bit(_io, _reg, _bit, _val_ptr) \
        max77696_masked_read(_io,\
            _reg, _reg##_##_bit, (u16)FFS(_reg##_##_bit), _val_ptr)

#define max77696_write_reg_bit(_io, _reg, _bit, _val) \
        max77696_masked_write(_io,\
            _reg, _reg##_##_bit, (u16)FFS(_reg##_##_bit), _val)

/*******************************************************************************
 * MAX77696-external Services
 ******************************************************************************/

extern struct device *max77696_dev (const char* name);

extern int max77696_get_version (struct device *coredev);

#define max77696_get_block_irq(_coredev, _block) \
        __max77696_get_block_irq(_coredev, MAX77696_BLK_##_block)
extern int __max77696_get_block_irq (struct device *coredev,
    unsigned int block_num);

#define max77696_set_block_irq(_coredev, _block, _irq) \
        __max77696_set_block_irq(_coredev, MAX77696_BLK_##_block, _irq)
extern int __max77696_set_block_irq (struct device *coredev,
    unsigned int block_num, int irq);

#define max77696_get_block_io(_coredev, _block) \
        __max77696_get_block_io(_coredev, MAX77696_BLK_##_block)
extern struct max77696_io *__max77696_get_block_io (struct device *coredev,
    unsigned int block_num);

#define max77696_get_topsys_irq(_coredev, _topsys_irq) \
        __max77696_get_topsys_irq(_coredev, MAX77696_TOPSYS_IRQ_##_topsys_irq)
extern int __max77696_get_topsys_irq (struct device *coredev,
    unsigned int topsys_irq);

#define max77696_read_topsys_stat(_coredev, _sts, _val) \
        __max77696_read_topsys_stat(_coredev, MAX77696_TOPSYS_STAT_##_sts, _val)
extern int __max77696_read_topsys_stat (struct device *coredev,
    int sts, u16 *val);

#define max77696_read_topsys_config(_coredev, _cfg, _val) \
        __max77696_read_topsys_config(_coredev,\
            MAX77696_TOPSYS_CNFG_##_cfg, _val)
extern int __max77696_read_topsys_config (struct device *coredev,
    int cfg, u16 *val);

#define max77696_write_topsys_config(_coredev, _cfg, _val) \
        __max77696_write_topsys_config(_coredev,\
            MAX77696_TOPSYS_CNFG_##_cfg, _val)
extern int __max77696_write_topsys_config (struct device *coredev,
    int cfg, u16 val);

enum {
    MAX77696_ERCFLAG_WDPMIC_FSHDN   = (1<< 0),
    MAX77696_ERCFLAG_WDPMIC_FRSTRT  = (1<< 1),
    MAX77696_ERCFLAG_MR_FSHDN       = (1<< 2),
    MAX77696_ERCFLAG_MR_FRSTRT      = (1<< 3),
    MAX77696_ERCFLAG_SFT_PSHDN      = (1<< 4),
    MAX77696_ERCFLAG_SFT_PRSTRT     = (1<< 5),
    MAX77696_ERCFLAG_SFT_FSHDN      = (1<< 6),
    MAX77696_ERCFLAG_SFT_FRSTRT     = (1<< 7),
    MAX77696_ERCFLAG_LBMOK_FSHDN    = (1<< 8),
    MAX77696_ERCFLAG_SYS1UVLO_FSHDN = (1<< 9),
    MAX77696_ERCFLAG_TOVLO_FSHDN    = (1<<10),
    MAX77696_ERCFLAG_RSTIN_PRSTRT   = (1<<11),
};

extern int max77696_test_ercflag (struct device *coredev,
    u16 ercflags);

extern int max77696_enable_buck_imon (struct device *coredev,
    u8 buck_id, bool en);

extern int max77696_enable_ldo_imon (struct device *coredev,
    u8 ldo_id, bool en);

#endif /* __MAX77696_H__ */
