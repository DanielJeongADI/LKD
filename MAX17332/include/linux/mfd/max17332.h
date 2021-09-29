
#ifndef __MAX17332_MFD_H__ 
#define __MAX17332_MFD_H__ 

/* MAX17332  Top Devices */
#define MAX17332_NAME			"max17332"

/* MAX17332 Devices */
#define MAX17332_CHARGER_NAME				MAX17332_NAME "-charger"
#define MAX17332_BATTERY_NAME				MAX17332_NAME "-battery"

#define MAX17332_NVM_BASE_ADDR 0x80
#define MAX17332_NVM_HIGH_ADDR 0xEF

/* Function Commands */
#define MAX17332_COMMAND_COPY_NVM (0xE904)
#define MAX17332_COMMAND_FULL_RESET (0x000F)
#define MAX17332_COMMAND_RECALL_HISTORY_REMAINING_WRITES (0xE29B)

#define REG_COMMAND			0x60
#define REG_COMMSTAT 		0x61
#define REG_CONFIG2			0xAB

#define REG_STATUS          0x00
#define REG_STATUS_MASK     0xFFEE

#define REG_PROGALRTS		0xAF
#define REG_PROGALRTS_MASK	0xFF7C

/* Nonvolatile Memory Registers */
#define REG_NROMID0_NVM	 	0xBC
#define REG_NROMID3_NVM	 	0xBF
#define REG_REMAINING_UPDATES_NVM	0xFD

/* ProtStatus register bits for MAX17332 */
#define BIT_SHDN_INT		BIT(0)
#define BIT_TOOCOLDD_INT	BIT(1)
#define BIT_ODCP_INT		BIT(2)
#define BIT_UVP_INT			BIT(3)
#define BIT_TOOHOTD_INT		BIT(4)
#define BIT_DIEHOT_INT		BIT(5)
#define BIT_PERMFAIL_INT	BIT(6)
#define BIT_PREQF_INT		BIT(8)
#define BIT_QOVFLW_INT		BIT(9)
#define BIT_OCCP_INT		BIT(10)
#define BIT_OVP_INT			BIT(11)
#define BIT_TOOCOLDC_INT	BIT(12)
#define BIT_FULL_INT		BIT(13)
#define BIT_TOOHOTC_INT		BIT(14)
#define BIT_CHGWDT_INT		BIT(15)

/* Status register bits for MAX17332 */
#define BIT_STATUS_PA		BIT(15)
#define BIT_STATUS_SMX 	  	BIT(14)
#define BIT_STATUS_TMX 	  	BIT(13)
#define BIT_STATUS_VMX 	  	BIT(12)
#define BIT_STATUS_CA 	  	BIT(11)
#define BIT_STATUS_SMN 	  	BIT(10)
#define BIT_STATUS_TMN 	  	BIT(9)
#define BIT_STATUS_VMN 	  	BIT(8)
#define BIT_STATUS_DSOCI	BIT(7)
#define BIT_STATUS_IMX 	  	BIT(6)
#define BIT_STATUS_ALLOWCHGB 	  	BIT(5)
#define BIT_STATUS_BST 	  	BIT(3)
#define BIT_STATUS_IMN 	  	BIT(2)
#define BIT_STATUS_POR 	  	BIT(1)

/*ChgStat register bits for MAX17332 */
#define BIT_STATUS_DROPOUT 	  	BIT(15)
#define BIT_STATUS_CP 	  	BIT(3)
#define BIT_STATUS_CT 	  	BIT(2)
#define BIT_STATUS_CC 	  	BIT(1)
#define BIT_STATUS_CV 	  	BIT(0)

/* CommStat register bits for MAX17332 */
#define MAX17332_COMMSTAT_NVERROR BIT(2)
#define MAX17332_COMMSTAT_NVBUSY BIT(3)
#define MAX17332_COMMSTAT_CHGOFF_POS 8
#define MAX17332_COMMSTAT_CHGOFF BIT(8)
#define MAX17332_COMMSTAT_DISOFF BIT(9)

/* CONFIG2 register bits for MAX17332*/
#define MAX17332_CONFIG2_POR_CMD BIT(15)

/* Chip Interrupts */
enum {
	MAX17332_FG_POR_INT = 0,
	MAX17332_FG_IMN_INT,
	MAX17332_FG_BST_INT,
	MAX17332_FG_ALLOWCHGB_INT,
	MAX17332_FG_IMX_INT,
	MAX17332_FG_DSOCI_INT,
	MAX17332_FG_VMN_INT,
	MAX17332_FG_TMN_INT,
	MAX17332_FG_SMN_INT,
	MAX17332_FG_CA_INT,
	MAX17332_FG_VMX_INT,
	MAX17332_FG_TMX_INT,
	MAX17332_FG_SMX_INT,
	MAX17332_FG_PROT_INT,

	MAX17332_FG_PROT_INT_START,
	MAX17332_FG_PROT_INT_ODCP = MAX17332_FG_PROT_INT_START,
	MAX17332_FG_PROT_INT_UVP,
	MAX17332_FG_PROT_INT_TOOHOTD,
	MAX17332_FG_PROT_INT_DIEHOTD,
	MAX17332_FG_PROT_INT_PERMFAIL,
	MAX17332_FG_PROT_INT_QOVFLW,
	MAX17332_FG_PROT_INT_OCCP,
	MAX17332_FG_PROT_INT_OVP,
	MAX17332_FG_PROT_INT_TOOCOLDC,
	MAX17332_FG_PROT_INT_FULL,
	MAX17332_FG_PROT_INT_TOOHOTC,
	MAX17332_FG_PROT_INT_CHGWDT,

	MAX17332_NUM_OF_INTS,
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

struct max17332_dev {
	struct mutex				lock;
	struct device				*dev;

	int					irq;
	int					irq_gpio;

	struct regmap_irq_chip_data	*irqc_intsrc;

	struct i2c_client	*pmic;			/* 0x6C, MODEL GAUGE */
	struct i2c_client	*nvm;			/* 0x16, NVM */

	struct regmap		*regmap_pmic;			/* CHARGER */
	struct regmap		*regmap_nvm;			/* NVM */

	struct max17332_pmic_platform_data  *pdata;
};

/*******************************************************************************
 * Platform Data
 ******************************************************************************/

struct max17332_pmic_platform_data {
	int irq; /* system interrupt number for PMIC */
	unsigned int rsense;
};

/*******************************************************************************
 * Chip IO
 ******************************************************************************/
int max17332_read(struct regmap *regmap, u8 addr, u16 *val);
int max17332_write(struct regmap *regmap, u8 addr, u16 val);
int max17332_update_bits(struct regmap *regmap, u8 addr, u16 mask, u16 val);

#endif /* !__MAX17332_MFD_H__ */

