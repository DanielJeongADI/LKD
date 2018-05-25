#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input.h>

#include <linux/mfd/max77696.h>

#ifndef CONFIG_MAX77696_DT
#define MAX77696_IO_BUSNUM       1
#define MAX77696_INTB_GPIO       0
#define MAX77696_EH_ACC_DET_GPIO -ENOTSUPP

#define MAX77696_NR_IRQS         (MAX77696_NUM_OF_IRQS+\
                                  MAX77696_NUM_OF_TOPSYS_IRQS+\
                                  MAX77696_NUM_OF_GPIOS)
#ifdef CONFIG_SPARSE_IRQ
#define MAX77696_TOPINT_IRQBASE  NR_IRQS
#else
#define MAX77696_TOPINT_IRQBASE  (NR_IRQS-MAX77696_NR_IRQS)//IRQ_BOARD_START
#endif
#define MAX77696_TOPSYS_IRQBASE  \
        (MAX77696_TOPINT_IRQBASE+MAX77696_NUM_OF_IRQS)
#define MAX77696_GPIO_IRQBASE    \
        (MAX77696_TOPSYS_IRQBASE+MAX77696_NUM_OF_TOPSYS_IRQS)

#define MAX77696_GPIO_BASE       -1

#define MAX77696_VDDQIN_UV       mV_to_uV(1200)

#define MAX77696_CHG_PSY         MAX77696_CHG_NAME
#define MAX77696_EH_PSY          MAX77696_EH_NAME
#define MAX77696_FG_PSY          MAX77696_FG_NAME

#define mV_to_uV(_mV)            ((_mV) * 1000)
#define uV_to_mV(_uV)            ((_uV) / 1000)
#define V_to_uV(_V)              (mV_to_uV(_V * 1000))
#define uV_to_V(_uV)             (uV_to_mV(_uV) / 1000)

/*******************************************************************************
 * Top Interrupts
 ******************************************************************************/

static struct max77696_topint_platform_data topint_pdata = {
    .irq         = -1,
    .irq_trigger = IRQ_TYPE_EDGE_FALLING,
    .irq_base    = MAX77696_TOPINT_IRQBASE,
};

/*******************************************************************************
 * Top System Management
 ******************************************************************************/

static struct max77696_topsys_platform_data topsys_pdata = {
    .irq_base = MAX77696_TOPSYS_IRQBASE,
};

/*******************************************************************************
 * GPIOs
 ******************************************************************************/

#define MAX77696_GPIO_CFG_OUTPUT(_gpio, _mode, _pu, _pd, _drv, _lvl) \
        {\
            .gpio       = _gpio,\
            .alter_mode = MAX77696_GPIO_MODE_##_mode,\
            .pullup_en  = !!(_pu),\
            .pulldn_en  = !!(_pd),\
            .direction  = MAX77696_GPIO_DIR_OUTPUT,\
            .u.output   = {\
                .drive = MAX77696_GPIO_DRIVE_##_drv,\
                .level = MAX77696_GPIO_LEVEL_##_lvl,\
            },\
        }

#define MAX77696_GPIO_CFG_INPUT(_gpio, _mode, _pu, _pd, _dbnc, _intcnfg) \
        {\
            .gpio       = _gpio,\
            .alter_mode = MAX77696_GPIO_MODE_##_mode,\
            .pullup_en  = !!(_pu),\
            .pulldn_en  = !!(_pd),\
            .direction  = MAX77696_GPIO_DIR_INPUT,\
            .u.input    = {\
                .dbnc    = MAX77696_GPIO_DBNC_##_dbnc,\
                .intcnfg = MAX77696_GPIO_INTCNFG_##_intcnfg,\
            },\
        }

static struct max77696_gpio_cfg_data gpio_cfg_data[] = {
#if 0
    MAX77696_GPIO_CFG_INPUT (0, STDGPIO, false, false, 32_MSEC, DISABLE),
    MAX77696_GPIO_CFG_INPUT (3, STDGPIO, false, false,  0_MSEC, DISABLE),
#endif
};

static struct max77696_gpio_platform_data gpio_pdata = {
    .gpio_base       = MAX77696_GPIO_BASE,
    .irq_base        = MAX77696_GPIO_IRQBASE,
    .cfg_data        = gpio_cfg_data,
    .num_of_cfg_data = ARRAY_SIZE(gpio_cfg_data),
};

/*******************************************************************************
 * 32KHz Clock Buffer
 ******************************************************************************/

static struct max77696_clk_platform_data clk_pdata = {
    .load_cap = MAX77696_CLK_LOAD_CAP_22PF,
    .op_mode  = MAX77696_CLK_OPMODE_LOW_POWER,
};

/*******************************************************************************
 * Watchdog Timer
 ******************************************************************************/

static struct max77696_wdt_platform_data wdt_pdata = {
    .timeout_sec       = 128,
    .ping_interval_sec = 60,
};

/*******************************************************************************
 * ADC
 ******************************************************************************/

static struct max77696_adc_platform_data adc_pdata = {
    .print_fmt = MAX77696_ADC_PRINT_FULL,
    .vref_name = NULL,
};

/*******************************************************************************
 * Regulators
 ******************************************************************************/

#define VREG_CONSUMERS_NAME(_id) vreg_consumers_##_id
#define VREG_CONSUMERS(_id)      \
        static struct regulator_consumer_supply VREG_CONSUMERS_NAME(_id)[]

#define VREG_NAME(_id) \
        "MAX77696-"#_id
#define VREG_REPRESENTATIVE(_id) \
        REGULATOR_SUPPLY(VREG_NAME(_id), NULL)

/*******************************************************************************
 * BUCK Regulators
 ******************************************************************************/

VREG_CONSUMERS(BUCK1) = {
    VREG_REPRESENTATIVE(BUCK1),
};
VREG_CONSUMERS(BUCK2) = {
    VREG_REPRESENTATIVE(BUCK2),
};
VREG_CONSUMERS(BUCK3) = {
    VREG_REPRESENTATIVE(BUCK3),
};
VREG_CONSUMERS(BUCK4) = {
    VREG_REPRESENTATIVE(BUCK4),
};
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
VREG_CONSUMERS(BUCK5) = {
    VREG_REPRESENTATIVE(BUCK5),
};
VREG_CONSUMERS(BUCK6) = {
    VREG_REPRESENTATIVE(BUCK6),
};
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */

#define BUCK_INIT(_id, _rsr, _fsr_en, _dvs_uV,\
            _min_uV, _max_uV, _apply, _boot_on, _always_on) \
        {\
            .buck_id   = MAX77696_VREG_##_id,\
            .rsr       = MAX77696_BUCK_RSR_##_rsr,\
            .fsr       = MAX77696_BUCK_FSR_##_fsr_en,\
            .dvs_uV    = _dvs_uV,\
            .init_data = {\
                .constraints = {\
                    .valid_modes_mask = MAX77696_BUCK_VALID_MODES,\
                    .valid_ops_mask   = MAX77696_BUCK_VALID_OPS,\
                    .min_uV           = _min_uV,\
                    .max_uV           = _max_uV,\
                    .apply_uV         = _apply,\
                    .always_on        = _always_on,\
                    .boot_on          = _boot_on,\
                    .name             = VREG_NAME(_id),\
                },\
                .num_consumer_supplies =\
                    ARRAY_SIZE(VREG_CONSUMERS_NAME(_id)),\
                .consumer_supplies     = VREG_CONSUMERS_NAME(_id),\
                .supply_regulator      = NULL,\
            },\
        }

static struct max77696_buck_cfg_data buck_cfg_data[] = {
    #define NA 0
    #define  V *1000*1000
    #define mV *1000
    #define uV *1

/*            id     rsr      fsr_en    dvs        min        max      apply  boot_on  always_on
 *                   [uV/us]            [uV]       [uV]       [uV]
 *            -----  -------  --------  --------   --------   -------  -----  -------  --------- */
    BUCK_INIT(BUCK1,   25000,   ENABLE,  1000000,    600000,  3387500, false,   false,     true ),
    BUCK_INIT(BUCK2,   25000,   ENABLE,  1000000,    600000,  3387500, false,   false,     true ),
    BUCK_INIT(BUCK3,   25000,   ENABLE,        0,    600000,  3387500, false,   false,     true ),
    BUCK_INIT(BUCK4,   25000,   ENABLE,        0,    600000,  3387500, false,   false,     true ),
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    BUCK_INIT(BUCK5,   25000,   ENABLE,        0,    600000,  3387500, false,   false,     true ),
    BUCK_INIT(BUCK6,   25000,   ENABLE,        0,    600000,  3387500, false,   false,     true ),
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
};

static struct max77696_bucks_platform_data bucks_pdata = {
    .cfg_data        = buck_cfg_data,
    .num_of_cfg_data = ARRAY_SIZE(buck_cfg_data),
};

#if defined(CONFIG_MAX77796)
/*******************************************************************************
 * BUCK Boost Regulators
 ******************************************************************************/

VREG_CONSUMERS(BBST1) = {
    VREG_REPRESENTATIVE(BBST1),
};

#define BBST_INIT(_id,\
            _min_uV, _max_uV, _apply, _boot_on, _always_on, _supply) \
        {\
            .bbst_id   = MAX77696_VREG_##_id,\
            .init_data = {\
                .constraints = {\
                    .valid_modes_mask = MAX77696_BBST_VALID_MODES,\
                    .valid_ops_mask   = MAX77696_BBST_VALID_OPS,\
                    .min_uV           = _min_uV,\
                    .max_uV           = _max_uV,\
                    .apply_uV         = _apply,\
                    .always_on        = _always_on,\
                    .boot_on          = _boot_on,\
                    .name             = VREG_NAME(_id),\
                },\
                .num_consumer_supplies =\
                    ARRAY_SIZE(VREG_CONSUMERS_NAME(_id)),\
                .consumer_supplies     = VREG_CONSUMERS_NAME(_id),\
                .supply_regulator      = _supply,\
            },\
        }

static struct max77696_bbst_cfg_data bbst_cfg_data[] = {
    #define NA 0
    #define  V *1000*1000
    #define mV *1000
    #define uV *1

/*            id     min        max      apply  boot_on  always_on  supply
 *                   [uV]       [uV]
 *            -----  --------   -------  -----  -------  ---------  ---------------- */
    BBST_INIT(BBST1,  2600000,  4187500, false,   false,      true, NULL),//VREG_NAME(BUCK1)),
};

static struct max77696_bbsts_platform_data bbsts_pdata = {
    .cfg_data        = bbst_cfg_data,
    .num_of_cfg_data = ARRAY_SIZE(bbst_cfg_data),
};
#endif /* CONFIG_MAX77796 */

/*******************************************************************************
 * LDO Regulators
 ******************************************************************************/

VREG_CONSUMERS(LDO1) = {
    VREG_REPRESENTATIVE(LDO1),
};
VREG_CONSUMERS(LDO2) = {
    VREG_REPRESENTATIVE(LDO2),
};
VREG_CONSUMERS(LDO3) = {
    VREG_REPRESENTATIVE(LDO3),
};
VREG_CONSUMERS(LDO4) = {
    VREG_REPRESENTATIVE(LDO4),
};
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
VREG_CONSUMERS(LDO5) = {
    VREG_REPRESENTATIVE(LDO5),
};
VREG_CONSUMERS(LDO6) = {
    VREG_REPRESENTATIVE(LDO6),
};
VREG_CONSUMERS(LDO7) = {
    VREG_REPRESENTATIVE(LDO7),
};
VREG_CONSUMERS(LDO8) = {
    VREG_REPRESENTATIVE(LDO8),
};
VREG_CONSUMERS(LDO9) = {
    VREG_REPRESENTATIVE(LDO9),
};
VREG_CONSUMERS(LDO10) = {
    VREG_REPRESENTATIVE(LDO10),
};
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */

#define LDO_INIT(_id, _sssr,\
            _min_uV, _max_uV, _apply, _boot_on, _always_on, _supply)\
        {\
            .ldo_id    = MAX77696_VREG_##_id,\
            .sssr      = MAX77696_LDO_SSSR_##_sssr,\
            .init_data = {\
                .constraints = {\
                    .valid_modes_mask = MAX77696_LDO_VALID_MODES,\
                    .valid_ops_mask   = MAX77696_LDO_VALID_OPS,\
                    .min_uV           = _min_uV,\
                    .max_uV           = _max_uV,\
                    .apply_uV         = _apply,\
                    .always_on        = _always_on,\
                    .boot_on          = _boot_on,\
                    .name             = VREG_NAME(_id),\
                },\
                .num_consumer_supplies =\
                    ARRAY_SIZE(VREG_CONSUMERS_NAME(_id)),\
                .consumer_supplies     = VREG_CONSUMERS_NAME(_id),\
                .supply_regulator      = _supply,\
            },\
        }

static struct max77696_ldo_cfg_data ldo_cfg_data[] = {
    #define NA 0
    #define  V *1000*1000
    #define mV *1000
    #define uV *1

/*           id    sssr     min      max      apply   boot_on  always_on  supply
 *                 [uV/us]  [uV]     [uV]
 *           ----  -------  -------  -------  ------  -------  ---------  ---------------- */
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    LDO_INIT(LDO1 , 100000,  800 mV, 3950 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO2 , 100000,  800 mV, 3950 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO3 , 100000,  800 mV, 3950 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO4 , 100000,  800 mV, 2325 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO5 , 100000,  800 mV, 2325 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO6 ,   5000,  800 mV, 3950 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO7 , 100000,  800 mV, 3950 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO8 , 100000,  800 mV, 2325 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO9 , 100000,  800 mV, 2325 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO10, 100000, 2200 mV, 5350 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
#elif defined(CONFIG_MAX77796)
    LDO_INIT(LDO1 , 100000,  800 mV, 3950 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO2 ,   5000,  800 mV, 3950 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO3 ,   5000,  800 mV, 3950 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
    LDO_INIT(LDO4 , 100000, 2200 mV, 5350 mV, false,    false,      true, NULL),//VREG_NAME(BUCK1)),
#endif
};

static struct max77696_ldos_platform_data ldos_pdata = {
    .cfg_data        = ldo_cfg_data,
    .num_of_cfg_data = ARRAY_SIZE(ldo_cfg_data),
};

/*******************************************************************************
 * Load Switches
 ******************************************************************************/

VREG_CONSUMERS(LSW1) = {
    VREG_REPRESENTATIVE(LSW1),
};
VREG_CONSUMERS(LSW2) = {
    VREG_REPRESENTATIVE(LSW2),
};
VREG_CONSUMERS(LSW3) = {
    VREG_REPRESENTATIVE(LSW3),
};
VREG_CONSUMERS(LSW4) = {
    VREG_REPRESENTATIVE(LSW4),
};

#define LSW_INIT(_id, _rt, _boot_on, _always_on, _supply)\
        {\
            .lsw_id    = MAX77696_VREG_##_id,\
            .rt        = MAX77696_LSW_RT_##_rt,\
            .init_data = {\
                .constraints = {\
                    .valid_modes_mask = MAX77696_LSW_VALID_MODES,\
                    .valid_ops_mask   = MAX77696_LSW_VALID_OPS,\
                    .min_uV           = 0,\
                    .max_uV           = 0,\
                    .apply_uV         = 0,\
                    .always_on        = _always_on,\
                    .boot_on          = _boot_on,\
                    .name             = VREG_NAME(_id),\
                },\
                .num_consumer_supplies =\
                    ARRAY_SIZE(VREG_CONSUMERS_NAME(_id)),\
                .consumer_supplies     = VREG_CONSUMERS_NAME(_id),\
                .supply_regulator      = _supply,\
            },\
        }

static struct max77696_lsw_cfg_data lsw_cfg_data[] = {
/*           id    rt       boot_on  always_on  supply
 *                 [uV/us]
 *           ----  -------  -------  ---------  ---------------- */
    LSW_INIT(LSW1,   30000,   false,      true, NULL),//VREG_NAME(BUCK1)),
    LSW_INIT(LSW2,   30000,   false,      true, NULL),//VREG_NAME(BUCK1)),
    LSW_INIT(LSW3,   30000,   false,      true, NULL),//VREG_NAME(BUCK1)),
    LSW_INIT(LSW4,   30000,   false,      true, NULL),//VREG_NAME(BUCK1)),
};

static struct max77696_lsws_platform_data lsws_pdata = {
    .cfg_data        = lsw_cfg_data,
    .num_of_cfg_data = ARRAY_SIZE(lsw_cfg_data),
};

/*******************************************************************************
 * EPD Power Supplies
 ******************************************************************************/

VREG_CONSUMERS(DISP) = {
    VREG_REPRESENTATIVE(DISP),
};
VREG_CONSUMERS(VCOM) = {
    VREG_REPRESENTATIVE(VCOM),
};
VREG_CONSUMERS(VEE) = {
    VREG_REPRESENTATIVE(VEE),
};
VREG_CONSUMERS(VNEG) = {
    VREG_REPRESENTATIVE(VNEG),
};
VREG_CONSUMERS(VPOS) = {
    VREG_REPRESENTATIVE(VPOS),
};
VREG_CONSUMERS(VDDH) = {
    VREG_REPRESENTATIVE(VDDH),
};

#define EPD_DISP_VALID_OPS  REGULATOR_CHANGE_STATUS
#define EPD_VCOM_VALID_OPS  REGULATOR_CHANGE_STATUS|REGULATOR_CHANGE_VOLTAGE
#define EPD_VEE_VALID_OPS                           REGULATOR_CHANGE_VOLTAGE
#define EPD_VNEG_VALID_OPS                          REGULATOR_CHANGE_VOLTAGE
#define EPD_VPOS_VALID_OPS                          REGULATOR_CHANGE_VOLTAGE
#define EPD_VDDH_VALID_OPS                          REGULATOR_CHANGE_VOLTAGE

#define EPD_INIT(_id, _min_uV, _max_uV, _apply, _boot_on, _always_on)\
        {\
            .epd_id    = MAX77696_VREG_EPD_##_id,\
            .init_data = {\
                .constraints = {\
                    .valid_modes_mask = MAX77696_EPD_VALID_MODES,\
                    .valid_ops_mask   = EPD_##_id##_VALID_OPS,\
                    .min_uV           = MAX77696_EPD_VID(_min_uV),\
                    .max_uV           = MAX77696_EPD_VID(_max_uV),\
                    .uV_offset        = 0,\
                    .apply_uV         = _apply,\
                    .always_on        = _always_on,\
                    .boot_on          = _boot_on,\
                    .name             = VREG_NAME(_id),\
                },\
                .num_consumer_supplies =\
                    ARRAY_SIZE(VREG_CONSUMERS_NAME(_id)),\
                .consumer_supplies     = VREG_CONSUMERS_NAME(_id),\
                .supply_regulator      = NULL,\
            },\
        }

static struct max77696_epd_cfg_data epd_cfg_data[] = {
    #define NA 0
    #define  V *1000*1000
    #define mV *1000
    #define uV *1

/*           id     min        max        apply  boot_on  always_on
 *                  [uV]       [uV]
 *           -----  ---------  ---------  -----  -------  --------- */
    EPD_INIT(DISP,         NA,        NA, false,   false,     false),
    EPD_INIT(VCOM,  - 5058900,         0, false,   false,     false),
    EPD_INIT(VEE ,  -28020000, -15000000, false,   false,     true ),
    EPD_INIT(VNEG,  - 1620000,   1605750, false,   false,     true ),
    EPD_INIT(VPOS,    8000000,  18000000, false,   false,     true ),
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    EPD_INIT(VDDH,   15000000,  29500000, false,   false,     true ),
#elif defined(CONFIG_MAX77796)
    EPD_INIT(VDDH,   15000000,  30500000, false,   false,     true ),
#endif
};

static struct max77696_epds_platform_data epds_pdata = {
    .cfg_data        = epd_cfg_data,
    .num_of_cfg_data = ARRAY_SIZE(epd_cfg_data),
};

/*******************************************************************************
 * LPDDR2 VDDQ Supply
 ******************************************************************************/

VREG_CONSUMERS(VDDQ) = {
    VREG_REPRESENTATIVE(VDDQ),
};

#define VDDQ_MIN  MAX77696_VDDQ_MIN(MAX77696_VDDQIN_UV)
#define VDDQ_MAX  MAX77696_VDDQ_MAX(MAX77696_VDDQIN_UV)

static struct max77696_vddq_platform_data vddq_pdata = {
    .input_uV  = MAX77696_VDDQIN_UV,
    .init_data = {
        .constraints = {
            .valid_modes_mask = 0,
            .valid_ops_mask   = REGULATOR_CHANGE_VOLTAGE,
            .min_uV           = VDDQ_MIN,
            .max_uV           = VDDQ_MAX,
            .always_on        = true,
            .boot_on          = true,
            .name             = VREG_NAME(VDDQ),
        },
        .num_consumer_supplies = ARRAY_SIZE(VREG_CONSUMERS_NAME(VDDQ)),
        .consumer_supplies     = VREG_CONSUMERS_NAME(VDDQ),
        .supply_regulator      = NULL,
    },
};

/*******************************************************************************
 * Charger
 ******************************************************************************/

static char *max77696_chg_supplicants[] = {
    MAX77696_FG_PSY,
};

static struct max77696_chg_platform_data chg_pdata = {
    .psy_name            = MAX77696_CHG_PSY,
    .supplied_to         = max77696_chg_supplicants,
    .num_supplicants     = ARRAY_SIZE(max77696_chg_supplicants),
    .supplied_from       = NULL,
    .num_supplies        = 0,
};

#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
/*******************************************************************************
 * Energy Harvester
 ******************************************************************************/

static char *max77696_eh_supplicants[] = {
    MAX77696_FG_PSY,
};

static struct max77696_eh_platform_data eh_pdata = {
    .psy_name            = MAX77696_EH_PSY,
    .supplied_to         = max77696_eh_supplicants,
    .num_supplicants     = ARRAY_SIZE(max77696_eh_supplicants),
    .supplied_from       = NULL,
    .num_supplies        = 0,
    .mode                = MAX77696_EH_MODE_CHARGER,
    .acc_ilimit          = 200,
    .acc_det_debounce_ms = 0,
    .acc_det_gpio_assert = 0,
    .acc_det_gpio        = MAX77696_EH_ACC_DET_GPIO,
};
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */

/*******************************************************************************
 * LED Indicators
 ******************************************************************************/

static struct max77696_leds_platform_data leds_pdata = {
    .init_data        = NULL,
    .num_of_init_data = 0,
};

/*******************************************************************************
 * Backlighting: WLED Boost Regulator
 ******************************************************************************/

static struct max77696_bl_platform_data bl_pdata = {
    .boot_brightness = ~0, /* Max brightness */
};

/*******************************************************************************
 * Real-Time Clock
 ******************************************************************************/

struct max77696_rtc_platform_data rtc_pdata = {
    .smpl_msec = 500,
};

/*******************************************************************************
 * UIC
 ******************************************************************************/

static struct max77696_uic_platform_data uic_pdata = {
    .int_type     = MAX77696_UIC_INTTYP_LEVEL,
    .int_delay    = MAX77696_UIC_INTDLY_2TICKS,
    .int_polarity = MAX77696_UIC_INTPOL_ACTIVE_LOW,
    .chg_psy_name = MAX77696_CHG_PSY,
};

/*******************************************************************************
 * FuelGauge
 ******************************************************************************/

static char *max77696_battery_supplies[] = {
    MAX77696_CHG_PSY,
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    MAX77696_EH_PSY,
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
};

static struct max77696_battery_ini battery_ini[] = {
    [0] = {
        .title       = "m3_zc_small",
        .learncfg    = 0x2603,
        .capacity    = 300,
        .misccfg     = 0x0870,
        .ichgterm    = 0x0180,
        .filtercfg   = 0xCEA4,
        .fullsocthr  = 0x5000,
        .relaxcfg    = 0x20C9,
        .dq_acc      = 0x0003,
        .dp_acc      = 0x0140,
        .qrtable00   = 0x3C00,
        .qrtable10   = 0x1B80,
        .qrtable20   = 0x0B04,
        .qrtable30   = 0x0885,
        .rcomp0      = 0x0070,
        .tempco      = 0x263D,
        .v_empty     = 0xA561,
        .iavg_empty  = 0x03C0,
        .modeldata   = {
            /* 0 */
            0x9760, 0xA510, 0xB100, 0xB600, 0xB7A0, 0xB900, 0xBA70, 0xBC70,
            0xBDE0, 0xBFC0, 0xC250, 0xC510, 0xC990, 0xCEA0, 0xD040, 0xD750,
            /* 1 */
            0x0060, 0x0120, 0x0200, 0x0710, 0x0E80, 0x0DF0, 0x1430, 0x1BD0,
            0x1520, 0x0D70, 0x0950, 0x08E0, 0x0800, 0x0780, 0x06B0, 0x01E0,
            /* 3 */
            0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
            0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
        },
    },
};

#define battery_ini_final battery_ini_m3_zc_small

static struct max77696_battery_platform_data battery_pdata = {
    .psy_name        = MAX77696_FG_PSY,
    .supplied_to     = NULL,
    .num_supplicants = 0,
    .supplied_from   = max77696_battery_supplies,
    .num_supplies    = ARRAY_SIZE(max77696_battery_supplies),
    .r_sns           = 10,       /* mOhm */
    .valrt_max       = 5100,     /* mV */
    .valrt_min       = 3600,     /* mV */
    .talrt_max       = 0,        /* C */
    .talrt_min       = 0,        /* C */
    .salrt_max       = 0,        /* % */
    .salrt_min       = 0,        /* % */
    .ini             = battery_ini,
    .num_ini         = ARRAY_SIZE(battery_ini),
};

/*******************************************************************************
 * Key Input
 ******************************************************************************/

static struct max77696_key_platform_data key_pdata = {
    .mrt_sec                     = 12,
    .keycode[MAX77696_KEY_EN0  ] = KEY_POWER,
    .keycode[MAX77696_KEY_1SEC ] = KEY_POWER,
    .keycode[MAX77696_KEY_MRWRN] = KEY_POWER,
};

/******************************************************************************/

static struct max77696_platform_data max77696_pdata = {
    .intb_gpio     = MAX77696_INTB_GPIO,
    .io_busnum     = MAX77696_IO_BUSNUM,

    .topint_pdata  = &topint_pdata ,
    .topsys_pdata  = &topsys_pdata ,
    .gpio_pdata    = &gpio_pdata   ,
    .clk_pdata     = &clk_pdata    ,
    .wdt_pdata     = &wdt_pdata    ,
    .adc_pdata     = &adc_pdata    ,
    .bucks_pdata   = &bucks_pdata  ,
#if defined(CONFIG_MAX77796)
    .bbsts_pdata   = &bbsts_pdata  ,
#endif /* CONFIG_MAX77796 */
    .ldos_pdata    = &ldos_pdata   ,
    .lsws_pdata    = &lsws_pdata   ,
    .epds_pdata    = &epds_pdata   ,
    .vddq_pdata    = &vddq_pdata   ,
    .chg_pdata     = &chg_pdata    ,
#if defined(CONFIG_MAX77696) || defined(CONFIG_MAX77697)
    .eh_pdata      = &eh_pdata     ,
#endif /* CONFIG_MAX77696 || CONFIG_MAX77697 */
    .leds_pdata    = &leds_pdata   ,
    .bl_pdata      = &bl_pdata     ,
    .rtc_pdata     = &rtc_pdata    ,
    .uic_pdata     = &uic_pdata    ,
    .battery_pdata = &battery_pdata,
    .key_pdata     = &key_pdata    ,
};

static struct platform_device max77696_pdev = {
    .name              = MAX77696_NAME,
    .id                = -1,
    .dev.platform_data = &max77696_pdata,
};

static int __init board_max77696_init (void)
{
    int rc;

    rc = platform_device_register(&max77696_pdev);
    if (unlikely(rc)) {
        pr_err(MAX77696_NAME": failed to register [%d]\n", rc);
    }

    return 0;
}
arch_initcall(board_max77696_init);
#endif /* !CONFIG_MAX77696_DT */