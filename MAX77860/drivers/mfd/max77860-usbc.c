// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2021 Maxim Integrated Products, Inc.
 * Author: Maxim Integrated <ugur.usug@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/max77860.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/interrupt.h>

#define CC_SRC_CUR_CH_BIT_POS		7
#define CC_SRC_CUR_BIT_POS		5
#define CC_SRC_SNK_BIT_POS		4
#define CC_SNK_SRC_BIT_POS		3
#define CC_DBG_EN_BIT_POS		2
#define CC_AUD_EN_BIT_POS		1
#define CC_DET_EN_BIT_POS		0

struct max77860_usbc_platform_data {
	int status;
};

struct max77860_usbc {
	struct device			*dev;
	struct max77860_dev		*max77860;
	struct regmap			*regmap;
	int				usbc_irq;

	u8				bc_status1;
	u8				bc_status2;
	u8				cc_status1;
	u8				cc_status2;

	u8				dcdtmo;
	u8				chg_type_run;
	u8				vbusdet;
	u8				dnvdat_rf;
	u8				dx_ovp;

	u8				cc_vcn;
	u8				cc_vsafe0;
	u8				cc_detabrt;

	enum max77860_chg_type		chg_type;
	enum max77860_pr_chg_type	pr_chg_type;
	enum max77860_cc_stat_type	cc_stat;
	enum max77860_cc_i_stat_type	cc_i_stat;
	enum max77860_cc_pin_stat_type	cc_pin_stat;

	struct max77860_usbc_platform_data *pdata;
};

static void max77860_print_chargertype(struct max77860_usbc *usbc)
{
	switch (usbc->chg_type) {
	case MAX77860_CHG_NOTHING:
		dev_info(usbc->dev, "Charger T: Unknown\n");
		break;
	case MAX77860_CHG_SDP:
		dev_info(usbc->dev, "Charger T: USB Cable\n");
		break;
	case MAX77860_CHG_CDP:
		dev_info(usbc->dev, "Charger T: Downstream Port\n");
		break;
	case MAX77860_CHG_DCP:
		dev_info(usbc->dev, "Charger T: Dedicated\n");
		break;
	default:
		break;
	}
}

static void max77860_print_pr_chargertype(struct max77860_usbc *usbc)
{
	switch (usbc->pr_chg_type) {
	case MAX77860_PR_CHG_UNKNOWN:
		dev_info(usbc->dev, "Chg Pr T: Unknown\n");
		break;
	case MAX77860_PR_CHG_SAMSUNG_2A:
		dev_info(usbc->dev, "Chg Pr T: Samsung 2A\n");
		break;
	case MAX77860_PR_CHG_APPLE_0_5A:
		dev_info(usbc->dev, "Chg Pr T: Apple 0.5A\n");
		break;
	case MAX77860_PR_CHG_APPLE_1A:
		dev_info(usbc->dev, "Chg Pr T: Apple 1A\n");
		break;
	case MAX77860_PR_CHG_APPLE_2A:
		dev_info(usbc->dev, "Chg Pr T: Apple 2A\n");
		break;
	case MAX77860_PR_CHG_APPLE_12W:
		dev_info(usbc->dev, "Chg Pr T: Apple 12W\n");
		break;
	case MAX77860_PR_CHG_3A_DCP:
		dev_info(usbc->dev, "Chg Pr T: 3A DCP\n");
		break;
	case MAX77860_PR_CHG_RFU:
		dev_info(usbc->dev, "Chg Pr T: RFU\n");
		break;
	default:
		break;
	}
}

static void max77860_usbc_bc_irq(struct max77860_usbc *usbc, u8 bc_int)
{
	int ret;

	ret = max77860_read(usbc->regmap, REG_BC_STATUS1, &usbc->bc_status1);
	if (IS_ERR_VALUE(ret)) {
		dev_err(usbc->dev, "REG_BC_STATUS1 read error [%d]\n", ret);
		goto out;
	}
	ret = max77860_read(usbc->regmap, REG_BC_STATUS2, &usbc->bc_status2);
	if (IS_ERR_VALUE(ret)) {
		dev_err(usbc->dev, "REG_BC_STATUS2 read error [%d]\n", ret);
		goto out;
	}

	if ((bc_int & BIT_CHG_TYPEI) != 0) {
		usbc->chg_type = (usbc->bc_status1 & BIT_CHG_TYPE)
			>> FFS(BIT_CHG_TYPE);
		max77860_print_chargertype(usbc);
	}
	if ((bc_int & BIT_DCDTMOI) != 0) {
		usbc->dcdtmo = (usbc->bc_status1 & BIT_DCDTMO)
				>> FFS(BIT_DCDTMO);
		dev_info(usbc->dev, "DCDTmo : %d\n", usbc->dcdtmo);
	}
	if ((bc_int & BIT_PRCHG_TYPI) != 0) {
		usbc->pr_chg_type = (usbc->bc_status1 & BIT_PRCHG_TYP)
				>> FFS(BIT_PRCHG_TYP);
		max77860_print_pr_chargertype(usbc);
	}
	if ((bc_int & (BIT_CHG_TYP_RUNRI | BIT_CHG_TYP_RUNFI)) != 0) {
		usbc->chg_type_run = (usbc->bc_status1 & BIT_CHG_TYP_RUN)
				>> FFS(BIT_CHG_TYP_RUN);
		dev_info(usbc->dev, "Chg Type Run : %d\n", usbc->chg_type_run);
	}
	if ((bc_int & BIT_DNVDATREFI) != 0) {
		usbc->dnvdat_rf = (usbc->bc_status2 & BIT_DNVDATREF)
				>> FFS(BIT_DNVDATREF);
		dev_info(usbc->dev, "DN VData Ref : %d\n", usbc->dnvdat_rf);
	}
	if ((bc_int & BIT_DXOVPI) != 0) {
		usbc->dx_ovp = (usbc->bc_status2 & BIT_DXOVP)
				>> FFS(BIT_DXOVP);
		dev_info(usbc->dev, "DN DX OVP : %d\n", usbc->dx_ovp);
	}
	if ((bc_int & BIT_VBUSDETI) != 0) {
		usbc->vbusdet = (usbc->bc_status1 & BIT_VBUSDET)
				>> FFS(BIT_VBUSDET);
		if (usbc->vbusdet == 1)
			dev_info(usbc->dev, "VBUS > VBUSDET\n");
		else
			dev_info(usbc->dev, "VBUS < VBUSDET\n");
	}

out:
	return;
}

static void max77860_print_cc_stat(struct max77860_usbc *usbc)
{
	switch (usbc->cc_stat) {
	case MAX77860_CC_NO_CONN:
		dev_info(usbc->dev, "CC Stat: Unknown\n");
		break;
	case MAX77860_CC_UFP:
		dev_info(usbc->dev, "CC Stat: UFP\n");
		break;
	case MAX77860_CC_DFP:
		dev_info(usbc->dev, "CC Stat: DFP\n");
		break;
	case MAX77860_CC_AUDIO:
		dev_info(usbc->dev, "CC Stat: Audio\n");
		break;
	case MAX77860_CC_DEBUG:
		dev_info(usbc->dev, "CC Stat: Debug\n");
		break;
	case MAX77860_CC_ERROR:
		dev_info(usbc->dev, "CC Stat: Error\n");
		break;
	case MAX77860_CC_DISABLED:
		dev_info(usbc->dev, "CC Stat: Disabled\n");
		break;
	case MAX77860_CC_RFU:
		dev_info(usbc->dev, "CC Stat: RFU\n");
		break;
	default:
		break;
	}
}

static void max77860_print_cc_i_stat(struct max77860_usbc *usbc)
{
	switch (usbc->cc_i_stat) {
	case MAX77860_CC_I_NOT_UFP:
		dev_info(usbc->dev, "CC I Stat: Not UFP\n");
		break;
	case MAX77860_CC_I_500_MA:
		dev_info(usbc->dev, "CC I Stat: 500mA\n");
		break;
	case MAX77860_CC_I_1_5A:
		dev_info(usbc->dev, "CC I Stat: 1.5A\n");
		break;
	case MAX77860_CC_I_3A:
		dev_info(usbc->dev, "CC I Stat: 3A\n");
		break;
	default:
		break;
	}
}

static void max77860_print_cc_pin_stat(struct max77860_usbc *usbc)
{
	switch (usbc->cc_pin_stat) {
	case MAX77860_CC_PIN_NO_DET:
		dev_info(usbc->dev, "CC Pin Stat: Not Determined\n");
		break;
	case MAX77860_CC_PIN_CC1:
		dev_info(usbc->dev, "CC Pin Stat: CC1\n");
		break;
	case MAX77860_CC_PIN_CC2:
		dev_info(usbc->dev, "CC Pin Stat: CC2\n");
		break;
	case MAX77860_CC_PIN_RFU:
		dev_info(usbc->dev, "CC Pin Stat: RFU\n");
		break;
	default:
		break;
	}
}

static void max77860_usbc_cc_irq(struct max77860_usbc *usbc, u8 cc_int)
{
	int ret;

	ret = max77860_read(usbc->regmap, REG_CC_STATUS1, &usbc->cc_status1);
	if (IS_ERR_VALUE(ret)) {
		dev_err(usbc->dev, "REG_CC_STATUS1 read error [%d]\n", ret);
		goto out;
	}
	ret = max77860_read(usbc->regmap, REG_CC_STATUS2, &usbc->cc_status2);
	if (IS_ERR_VALUE(ret)) {
		dev_err(usbc->dev, "REG_CC_STATUS2 read error [%d]\n", ret);
		goto out;
	}
	if ((cc_int & BIT_CC_STATI) != 0) {
		usbc->cc_stat = (usbc->cc_status1 & BIT_CC_STAT)
			>> FFS(BIT_CC_STAT);
		max77860_print_cc_stat(usbc);
	}
	if ((cc_int & BIT_CC_VCN_STATI) != 0) {
		usbc->cc_vcn = (usbc->cc_status1 & BIT_CC_VCN_STAT)
				>> FFS(BIT_CC_VCN_STAT);
		dev_info(usbc->dev, "CC_VCN : %d\n", usbc->cc_vcn);
	}
	if ((cc_int & BIT_CC_STATI) != 0) {
		usbc->cc_i_stat = (usbc->cc_status1 & BIT_CC_STAT)
				>> FFS(BIT_CC_STAT);
		max77860_print_cc_i_stat(usbc);
	}
	if ((cc_int & BIT_CC_PINSTATI) != 0) {
		usbc->cc_pin_stat = (usbc->cc_status1 & BIT_CC_PINSTAT)
				>> FFS(BIT_CC_PINSTAT);
		max77860_print_cc_pin_stat(usbc);
	}
	if ((cc_int & BIT_DET_ABRTI) != 0) {
		usbc->cc_detabrt = (usbc->cc_status2 & BIT_DET_ABRT)
				>> FFS(BIT_DET_ABRT);
		dev_info(usbc->dev, "CC DET ABRT : %d\n", usbc->cc_detabrt);
	}
	if ((cc_int & BIT_VSAFE0VI) != 0) {
		usbc->cc_vsafe0 = (usbc->cc_status2 & BIT_VSAFE0V_S)
				>> FFS(BIT_VSAFE0V_S);
		dev_info(usbc->dev, "CC VSAFE0 : %d\n", usbc->cc_vsafe0);
	}

out:
	return;
}

static irqreturn_t max77860_usbc_isr(int irq, void *data)
{
	struct max77860_usbc *usbc = data;
	int ret;
	u8 bc_int, cc_int;

	pr_info("%s\n", __func__);

	/* Always read CC_INT before reading BC_INT */
	ret = max77860_read(usbc->regmap, REG_CC_INT, &cc_int);
	if (IS_ERR_VALUE(ret)) {
		dev_err(usbc->dev, "REG_CC_INT read error [%d]\n", ret);
		goto out;
	}

	ret = max77860_read(usbc->regmap, REG_BC_INT, &bc_int);
	if (IS_ERR_VALUE(ret)) {
		dev_err(usbc->dev, "REG_BC_INT read error [%d]\n", ret);
		goto out;
	}

	if (bc_int) {
		dev_info(usbc->dev, "REG_BC_INT = 0x%x\n", bc_int);
		max77860_usbc_bc_irq(usbc, bc_int);
	}
	if (cc_int) {
		dev_info(usbc->dev, "REG_CC_INT = 0x%x\n", cc_int);
		max77860_usbc_cc_irq(usbc, cc_int);
	}

out:
	return IRQ_HANDLED;
}

static int max77860_usbc_initialize(struct max77860_usbc *usbc)
{
	int ret;
	u8 val;

	pr_info("%s\n", __func__);

	/*
	 * interrupt mask - if you want to enable some bits,
	 * you should clear them
	 */
	val  = 0;
	/*
	 * val |= BIT_CHGTypI;
	 * val |= BIT_DCDTmoI;
	 * val |= BIT_PrChgTypI;
	 * val |= BIT_ChgTypRunRI;
	 * val |= BIT_ChgTypRunFI;
	 * val |= BIT_DNVDATREFI;
	 * val |= BIT_DxOVPI;
	 * val |= BIT_VBUSDetI;
	 */

	ret = max77860_write(usbc->regmap, REG_BC_INT_MASK, val);
	if (IS_ERR_VALUE(ret)) {
		pr_err("BC_INT_MASK write error [%d]\n", ret);
		goto out;
	}

	pr_info("%s: BC_INT_MASK is set to 0x%x\n", __func__, val);

	ret = max77860_read(usbc->regmap, REG_BC_INT_MASK, &val);
	if (IS_ERR_VALUE(ret)) {
		pr_err("BC_INT_MASK read error [%d]\n", ret);
		goto out;
	}
	pr_info("%s: BC_INT_MASK is now 0x%x\n", __func__, val);

	/*
	 * interrupt mask - if you want to enable some bits,
	 *	you should clear them
	 */
	val  = 0;
	/*
	 * val |= BIT_CCStatI;
	 * val |= BIT_CCVcnStatI;
	 * val |= BIT_CCIStatI;
	 * val |= BIT_CCPinStatI;
	 * val |= BIT_DetAbrtI;
	 * val |= BIT_VSAFE0VI;
	 */

	ret = max77860_write(usbc->regmap, REG_CC_INT_MASK, val);
	if (IS_ERR_VALUE(ret)) {
		pr_err("CC_INT_MASK write error [%d]\n", ret);
		goto out;
	}

	pr_info("%s: CC_INT_MASK is set to 0x%x\n", __func__, val);

	ret = max77860_read(usbc->regmap, REG_CC_INT_MASK, &val);
	if (IS_ERR_VALUE(ret)) {
		pr_err("CC_INT_MASK read error [%d]\n", ret);
		goto out;
	}
	pr_info("%s: CC_INT_MASK is now 0x%x\n", __func__, val);

	return 0;
out:
	return ret;
}

static ssize_t cc_detect_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf,
				   u8 mask)
{
	struct max77860_usbc *usbc;
	int ret;
	u8 val;

	usbc = dev_get_drvdata(dev);

	ret = max77860_read(usbc->regmap, REG_CC_CONTROL1, &val);
	if (ret < 0) {
		dev_err(dev, "Could not read from CC\n");
		return ret;
	}

	return sprintf(buf, "%d\n", (val & mask) ? 1 : 0);
}

static ssize_t cc_detect_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count,
				    u8 mask, u8 bit_pos)
{
	struct max77860_usbc *usbc;
	int ret, in_value;

	usbc = dev_get_drvdata(dev);

	if (kstrtoint(buf, 0, &in_value))
		return -EINVAL;

	if (in_value > 1 || in_value < 0)
		return -EINVAL;

	in_value <<= bit_pos;

	ret = regmap_update_bits(usbc->regmap, REG_CC_CONTROL1,
				 mask, in_value & 0xFF);
	if (ret < 0) {
		dev_err(dev, "Could not written to CC\n");
		return ret;
	}

	return count;
}

static ssize_t cc_pin_detect_mode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	return cc_detect_mode_show(dev, attr, buf, BIT_CC_DET_EN);
}

static ssize_t cc_pin_detect_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return cc_detect_mode_store(dev, attr, buf, count,
				    BIT_CC_DET_EN, CC_DET_EN_BIT_POS);
}

static ssize_t cc_aud_detect_mode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	return cc_detect_mode_show(dev, attr, buf, BIT_CC_AUD_EN);
}

static ssize_t cc_aud_detect_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return cc_detect_mode_store(dev, attr, buf, count,
				    BIT_CC_AUD_EN, CC_AUD_EN_BIT_POS);
}

static ssize_t cc_dbg_detect_mode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	return cc_detect_mode_show(dev, attr, buf, BIT_CC_DBG_EN);
}

static ssize_t cc_dbg_detect_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return cc_detect_mode_store(dev, attr, buf, count,
				    BIT_CC_DBG_EN, CC_DBG_EN_BIT_POS);
}

static ssize_t cc_dfp_detect_mode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	return cc_detect_mode_show(dev, attr, buf, BIT_CC_SNK_SRC);
}

static ssize_t cc_dfp_detect_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return cc_detect_mode_store(dev, attr, buf, count,
				    BIT_CC_SNK_SRC, CC_SNK_SRC_BIT_POS);
}

static ssize_t cc_ufp_detect_mode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	return cc_detect_mode_show(dev, attr, buf, BIT_CC_SRC_SNK);
}

static ssize_t cc_ufp_detect_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return cc_detect_mode_store(dev, attr, buf, count,
				    BIT_CC_SRC_SNK, CC_SRC_SNK_BIT_POS);
}

static DEVICE_ATTR_RW(cc_pin_detect_mode);
static DEVICE_ATTR_RW(cc_aud_detect_mode);
static DEVICE_ATTR_RW(cc_dbg_detect_mode);
static DEVICE_ATTR_RW(cc_dfp_detect_mode);
static DEVICE_ATTR_RW(cc_ufp_detect_mode);

static struct attribute *max77860_usbc_sysfs_entries[] = {
	&dev_attr_cc_pin_detect_mode.attr,
	&dev_attr_cc_aud_detect_mode.attr,
	&dev_attr_cc_dbg_detect_mode.attr,
	&dev_attr_cc_dfp_detect_mode.attr,
	&dev_attr_cc_ufp_detect_mode.attr,
	NULL,
};

static const struct attribute_group max77860_usbc_attr_group = {
	.attrs = max77860_usbc_sysfs_entries,
};

static int max77860_usbc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77860_dev *max77860 = dev_get_drvdata(dev->parent);
	struct max77860_usbc *usbc;
	int ret;

	pr_info("%s\n", __func__);

	usbc = kzalloc(sizeof(*usbc), GFP_KERNEL);
	if (!usbc)
		return -ENOMEM;

	usbc->dev = &pdev->dev;
	usbc->max77860 = max77860;
	usbc->regmap = max77860->regmap_usbc;
	usbc->pdata = NULL;

	platform_set_drvdata(pdev, usbc);

	ret = max77860_usbc_initialize(usbc);
	if (ret) {
		dev_err(dev, "USBC initialize error\n");
		return ret;
	}

	usbc->usbc_irq = regmap_irq_get_virq(max77860->irqc_intsrc,
					     MAX77860_USBC_INT);

	ret = request_threaded_irq(usbc->usbc_irq, NULL, max77860_usbc_isr,
				   IRQF_TRIGGER_FALLING, "usbc", usbc);
	if (ret) {
		dev_err(dev, "failed to request USBC IRQ\n");
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &max77860_usbc_attr_group);
	if (ret)
		dev_err(dev, "error creating sysfs entries\n");

	return ret;
}

static int max77860_usbc_remove(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	return 0;
}

static void max77860_usbc_shutdown(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
}

#ifdef CONFIG_PM_SLEEP
static int max77860_usbc_suspend(struct device *dev)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int max77860_usbc_resume(struct device *dev)
{
	pr_info("%s\n", __func__);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77860_usbc_pm_ops, max77860_usbc_suspend,
		max77860_usbc_resume);

static const struct platform_device_id max77860_usbc_id[] = {
	{"max77860-usbc", 0},
	{},
};

MODULE_DEVICE_TABLE(platform, max77860_usbc_id);

static struct platform_driver max77860_usbc_driver = {
	.driver = {
		.name = "max77860-usbc",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &max77860_usbc_pm_ops,
#endif
	},
	.shutdown = max77860_usbc_shutdown,
	.probe = max77860_usbc_probe,
	.remove = max77860_usbc_remove,
	.id_table = max77860_usbc_id,
};

static int __init max77860_usbc_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&max77860_usbc_driver);
}
module_init(max77860_usbc_init);

static void __exit max77860_usbc_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&max77860_usbc_driver);
}
module_exit(max77860_usbc_exit);

MODULE_DESCRIPTION("MAX77860 USBC driver");
MODULE_AUTHOR("ugur.usug@maximintegrated.com ");
MODULE_LICENSE("GPL");
