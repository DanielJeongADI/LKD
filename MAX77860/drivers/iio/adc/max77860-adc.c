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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/driver.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/mfd/max77860.h>

#define MAX77860_ADC_MODE_DISABLED	0
#define MAX77860_ADC_MODE_SINGLE	1
#define MAX77860_ADC_MODE_CONT		2

#define MEAS_ADC_COUNT_BIT_POS		0
#define CH1_OFFSET_CAL_EN_BIT_POS	2
#define CH3_OFFSET_CAL_EN_BIT_POS	3
#define CH4_OFFSET_CAL_EN_BIT_POS	4
#define ADC_FILTER_BIT_POS		5
#define VBUS_HV_RANGE_BIT_POS		7

#define MAX77860_ADC_ALL_CH		0xFF

#define MAX77860_ADC_CHANNEL(_channel, _name, _type, _reg) \
	{							\
		.type = _type,					\
		.indexed = 1,					\
		.channel = _channel,				\
		.address = _reg,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
				      BIT(IIO_CHAN_INFO_SCALE) |\
				      BIT(IIO_CHAN_INFO_OFFSET),\
		.datasheet_name = _name,			\
	}

struct max77860_adc_iio {
	struct regmap	*regmap;
};

enum max77860_adc_channel {
	MAX77860_ADC_VBUS_V = 0,
	MAX77860_ADC_VBUS_I,
	MAX77860_ADC_VBAT_V,
	MAX77860_ADC_VBAT_I,
	MAX77860_ADC_VBAT_I_REXT,
	MAX77860_ADC_TEMP,
	MAX77860_ADC_RSVD,
	MAX77860_ADC_TEST,
	MAX77860_ADC_VBUS_I_OFF,
	MAX77860_ADC_VBAT_I_OFF,
	MAX77860_ADC_VBAT_I_REXT_OFF,
};

static const struct iio_chan_spec max77860_adc_channels[] = {
	MAX77860_ADC_CHANNEL(MAX77860_ADC_VBUS_V, "vbus_v", IIO_VOLTAGE,
			     REG_ADC_DATA_CH0),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_VBUS_I, "vbus_i", IIO_CURRENT,
			     REG_ADC_DATA_CH1),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_VBAT_V, "vbat_v", IIO_VOLTAGE,
			     REG_ADC_DATA_CH2),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_VBAT_I, "vbat_i", IIO_CURRENT,
			     REG_ADC_DATA_CH3),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_VBAT_I_REXT, "vbat_i_rext",
			     IIO_CURRENT,
			     REG_ADC_DATA_CH4),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_TEMP, "temp", IIO_TEMP,
			     REG_ADC_DATA_CH5),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_RSVD, "rsvd-vcm", IIO_VOLTAGE,
			     REG_ADC_DATA_CH6),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_TEST, "test", IIO_VOLTAGE,
			     REG_ADC_DATA_CH7),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_VBUS_I_OFF,
			     "vbus_i_off", IIO_CURRENT,
			     REG_ADC_OFFSET_CH1),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_VBAT_I_OFF,
			     "vbus_i_off", IIO_CURRENT,
			     REG_ADC_OFFSET_CH3),
	MAX77860_ADC_CHANNEL(MAX77860_ADC_VBAT_I_REXT_OFF,
			     "vbus_i_rext_off", IIO_CURRENT,
			     REG_ADC_OFFSET_CH4),
};

static int max77860_adc_scale(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2)
{
	u8 byte;
	int ret;

	struct max77860_adc_iio *info = iio_priv(indio_dev);

	switch (chan->channel) {
	case MAX77860_ADC_VBUS_V:
		ret = max77860_read(info->regmap, REG_ADC_CONFIG1, &byte);
		if (ret < 0) {
			dev_err(&indio_dev->dev, "Invalid value: ADC\n");
			return -EINVAL;
		}
		*val = 0;
		if ((byte & BIT_VBUS_HV_RANGE) == BIT_VBUS_HV_RANGE)
			*val2 = 33000;
		else
			*val2 = 14000;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_VBUS_I:
	case MAX77860_ADC_VBUS_I_OFF:
		*val = 0;
		*val2 = 16000;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_VBAT_V:
		*val = 0;
		*val2 = 11000;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_VBAT_I:
	case MAX77860_ADC_VBAT_I_OFF:
		*val = 0;
		*val2 = 12000;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_VBAT_I_REXT:
	case MAX77860_ADC_VBAT_I_REXT_OFF:
		*val = 0;
		*val2 = 78125;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_TEMP:
		*val = 0;
		*val2 = 240000;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_RSVD:
	case MAX77860_ADC_TEST:
		*val = 0;
		*val2 = 3100;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int max77860_adc_offset(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2)
{
	u8 byte;
	int ret;

	struct max77860_adc_iio *info = iio_priv(indio_dev);

	switch (chan->channel) {
	case MAX77860_ADC_VBUS_V:
		ret = max77860_read(info->regmap, REG_ADC_CONFIG1, &byte);
		if (ret < 0) {
			dev_err(&indio_dev->dev, "Invalid value: ADC\n");
			return -EINVAL;
		}
		if ((byte & BIT_VBUS_HV_RANGE) == BIT_VBUS_HV_RANGE) {
			*val = 2;
			*val2 = 700000;
		} else {
			*val = 6;
			*val2 = 300000;
		}
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_VBUS_I:
	case MAX77860_ADC_VBAT_I:
	case MAX77860_ADC_VBAT_I_REXT:
		*val = 0;
		return IIO_VAL_INT;
	case MAX77860_ADC_VBAT_V:
		*val = 2;
		*val2 = 100000;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_TEMP:
		*val = 20;
		return IIO_VAL_INT;
	case MAX77860_ADC_RSVD:
	case MAX77860_ADC_TEST:
		*val = 0;
		*val2 = 600000;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_VBUS_I_OFF:
		*val = -2;
		*val2 = 48000;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_VBAT_I_OFF:
		*val = -1;
		*val2 = 536000;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77860_ADC_VBAT_I_REXT_OFF:
		*val = -10;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int max77860_adc_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val)
{
	int ret;
	u8 byte;
	struct max77860_adc_iio *info = iio_priv(indio_dev);

	ret = max77860_read(info->regmap, chan->address, &byte);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "Invalid value: ADC\n");
		return -EINVAL;
	}

	*val = byte;

	pr_info("%s: reg: %x val: %d\n", __func__,
		(unsigned int)chan->address, *val);

	return IIO_VAL_INT;
}

static int max77860_adc_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		return max77860_adc_offset(indio_dev, chan, val, val2);

	case IIO_CHAN_INFO_SCALE:
		return max77860_adc_scale(indio_dev, chan, val, val2);

	case IIO_CHAN_INFO_RAW:
		return max77860_adc_raw(indio_dev, chan, val);

	default:
		return -EINVAL;
	}
}

static ssize_t show_adc_mode(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct max77860_adc_iio *info = iio_priv(dev_to_iio_dev(dev));
	int ret, len;
	u8 val;

	ret = max77860_read(info->regmap, REG_ADC_CONFIG1, &val);
	if (ret < 0) {
		dev_err(dev, "Could not read from ADC\n");
		return ret;
	}

	val = (val & BITS_MEAS_ADC_COUNT) >> MEAS_ADC_COUNT_BIT_POS;

	if (val == 0)
		len = snprintf(buf, PAGE_SIZE, "Disabled (0)\n");
	else if (val == 1)
		len = snprintf(buf, PAGE_SIZE, "Single Mode (1)\n");
	else
		len = snprintf(buf, PAGE_SIZE, "Continuous Mode (2 or 3)\n");

	return len;
}

static ssize_t store_adc_mode(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct max77860_adc_iio *info = iio_priv(dev_to_iio_dev(dev));
	int ret, in_value;

	if (kstrtoint(buf, 0, &in_value))
		return -EINVAL;

	if (in_value > 3 || in_value < 0)
		return -EINVAL;

	in_value <<= MEAS_ADC_COUNT_BIT_POS;

	ret = regmap_update_bits(info->regmap, REG_ADC_CONFIG1,
				 BITS_MEAS_ADC_COUNT, in_value & 0xFF);
	if (ret < 0) {
		dev_err(dev, "Could not written to ADC\n");
		return ret;
	}

	return count;
}

static IIO_DEVICE_ATTR(adc_mode, 0644,
		       show_adc_mode, store_adc_mode, 0);

static struct attribute *max77860_adc_attributes[] = {
	&iio_dev_attr_adc_mode.dev_attr.attr,
	NULL,
};

static const struct attribute_group max77860_adc_attr_group = {
	.attrs = max77860_adc_attributes,
};

static const struct iio_info max77860_adc_info = {
	.read_raw = max77860_adc_read_raw,
	.attrs	  = &max77860_adc_attr_group,
};

static void max77860_adc_dt_configure(struct platform_device *pdev)
{
	int ret;
	unsigned int val;
	unsigned int adc_config1 = 0;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct max77860_adc_iio *info = iio_priv(indio_dev);
	struct device_node *np;

	np = of_find_node_by_name(pdev->dev.of_node, "adc");

	val = 0;	/* disable */
	of_property_read_u32(np, "channel_enable", &val);
	ret = regmap_write(info->regmap, REG_ADC_CONFIG2, val);
	if (ret < 0)
		dev_err(&pdev->dev, "could not write to device\n");
	dev_info(&pdev->dev, "property channel_enable %d\n", val);

	val = 0;	/* filter disable */
	of_property_read_u32(np, "filter_enable", &val);
	ret = regmap_write(info->regmap, REG_ADC_CONFIG3, val);
	if (ret < 0)
		dev_err(&pdev->dev, "could not write to device\n");
	dev_info(&pdev->dev, "property filter_enable %d\n", val);

	val = 0;	/* 2.7V-6.3V LSB = 14mV */
	of_property_read_u32(np, "vbus_hv_range", &val);
	adc_config1 |= (val & (BIT_VBUS_HV_RANGE >> VBUS_HV_RANGE_BIT_POS))
	       << VBUS_HV_RANGE_BIT_POS;
	dev_info(&pdev->dev, "property vbus_hv_range %d\n", val);

	val = 0;	/* 2-points averaging */
	of_property_read_u32(np, "filter_averaging", &val);
	adc_config1 |= (val & (BITS_ADC_FILTER >> ADC_FILTER_BIT_POS))
		       << ADC_FILTER_BIT_POS;
	dev_info(&pdev->dev, "property filter_averaging %d\n", val);

	ret = regmap_update_bits(info->regmap, REG_ADC_CONFIG1,
				 BIT_VBUS_HV_RANGE | BITS_ADC_FILTER,
				 adc_config1);
	if (ret < 0)
		dev_err(&pdev->dev, "could not write to device\n");
}

static int max77860_adc_probe(struct platform_device *pdev)
{
	struct max77860_adc_iio *info;
	struct iio_dev *indio_dev;
	struct max77860_dev *max77860;
	int ret;

	pr_info("%s: MAX77860 ADC Driver Loading\n", __func__);

	max77860 = dev_get_drvdata(pdev->dev.parent);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);
	platform_set_drvdata(pdev, indio_dev);

	info->regmap = max77860->regmap_chg;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->name = platform_get_device_id(pdev)->name;
	indio_dev->info = &max77860_adc_info;
	indio_dev->channels = max77860_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(max77860_adc_channels);

	max77860_adc_dt_configure(pdev);

	/* Enable ADC */
	ret = regmap_update_bits(info->regmap, REG_ADC_CONFIG1,
				 BITS_MEAS_ADC_COUNT, MAX77860_ADC_MODE_CONT);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not write to device\n");
		return -EINVAL;
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register the device\n");
		goto fail_register;
	}

	return 0;

fail_register:
	regmap_update_bits(info->regmap, REG_ADC_CONFIG1,
			   BITS_MEAS_ADC_COUNT, MAX77860_ADC_MODE_DISABLED);

	regmap_write(info->regmap, REG_ADC_CONFIG2, 0xFF);

	return ret;
}

static int max77860_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct max77860_adc_iio *info = iio_priv(indio_dev);

	pr_info("%s: Driver unloading\n", __func__);

	/* Disable ADC */
	regmap_update_bits(info->regmap, REG_ADC_CONFIG1,
			   BITS_MEAS_ADC_COUNT, MAX77860_ADC_MODE_DISABLED);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct of_device_id max77860_adc_dt_ids[] = {
	{ .compatible = "maxim,max77860-adc"},
	{ /* sentinel*/ },
};
MODULE_DEVICE_TABLE(of, max77860_adc_dt_ids);

static struct platform_driver max77860_adc_driver = {
	.driver = {
		.name = MAX77860_ADC_NAME,
		.owner = THIS_MODULE,
		.of_match_table = max77860_adc_dt_ids,
	},
	.probe = max77860_adc_probe,
	.remove = max77860_adc_remove,
};

static int __init max77860_adc_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&max77860_adc_driver);
}

static void __exit max77860_adc_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&max77860_adc_driver);
}

module_init(max77860_adc_init);
module_exit(max77860_adc_exit);

MODULE_DESCRIPTION("MAX77860 ADC Driver");
MODULE_AUTHOR("ugur.usug@maximintegrated.com ");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
