// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Maxim MAX31730 3-Channel Remote Temperature Sensor
 *
 * Author: Metin Ozkan <metin.ozkan@maximintegrated.com>
 * Copyright (C) 2020 Maxim Integrated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under  the terms of  the GNU General  Public License as published
 * by the Free Software Foundation;  either version 2 of the  License,
 * or (at your option) any later version.
 */

#include <linux/bits.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/workqueue.h>

/* The MAX31730 Register Addresses */
#define MAX31730_REG_TEMP			0x00
#define MAX31730_REG_HIGHEST_TEMP		0x10
#define MAX31730_REG_HIGHEST_TEMP_EN		0x12
#define MAX31730_REG_CONFIG			0x13
#define MAX31730_REG_CUSTOM_IDEALITY_FACT	0x14
#define MAX31730_REG_CUSTOM_IDEALITY_EN		0x15
#define MAX31730_REG_CUSTOM_OFFSET		0x16
#define MAX31730_REG_OFFSET_EN			0x17
#define MAX31730_REG_FILTER_EN			0x18
#define MAX31730_REG_BETA_COMPEN_EN		0x19
#define MAX31730_REG_BETA_VALUE			0x1A
#define MAX31730_REG_TEMP_MAX			0x20
#define MAX31730_REG_TEMP_ALL_MIN		0x30
#define MAX31730_REG_STATUS_HIGH		0x32
#define MAX31730_REG_STATUS_LOW			0x33
#define MAX31730_REG_THERM_MASK			0x34
#define MAX31730_REG_CHANNEL_EN			0x35
#define MAX31730_REG_DIODE_FAULT_STATUS		0x36
#define MAX31730_REG_REF_TEMP			0x40
#define MAX31730_REG_MFR_ID			0x50
#define MAX31730_REG_MFR_REV			0x51
/* MAX31730 Register Bits/Fields */
#define MAX31730_BIT_REMOTE_3			BIT(3)
#define MAX31730_BIT_REMOTE_2			BIT(2)
#define MAX31730_BIT_REMOTE_1			BIT(1)
#define MAX31730_BIT_LOCAL			BIT(0)
 /* Configuration (13h) */
#define MAX31730_BIT_STOP			BIT(7)
#define MAX31730_BIT_POR			BIT(6)
#define MAX31730_BIT_TIMEOUT			BIT(5)
#define MAX31730_BIT_INT_COMP			BIT(4)
#define MAX31730_FIELD_FAULT_QUEUE_11b		(u8)(BIT(3) | BIT(2))
#define MAX31730_FIELD_FAULT_QUEUE_10b		(u8)(BIT(3) | ~BIT(2))
#define MAX31730_FIELD_FAULT_QUEUE_01b		(u8)(~BIT(3) | BIT(2))
#define MAX31730_BIT_EXTRANGE			BIT(1)
#define MAX31730_BIT_ONE_SHOT			BIT(0)
/* MAX31730 Register POR Values and Definitions*/
#define MAX31730_MFR_ID				0x4D
#define MAX31730_MFR_REV			0x01
#define MAX31730_ALERT_RESP_ADDR		0x19
#define MAX31730_TEMP_MIN			(-55000) /* in milliDegreeC */
#define MAX31730_TEMP_MAX			127937	 /* in milliDegreeC */
#define DRIVER_NAME				"max31730"

/* Addresses scanned */
static const unsigned short i2c_address[] = { 0x38, 0x3A, 0x3C, 0x3E,
				0x98, 0x9A, 0x9C, 0x9E, I2C_CLIENT_END };

struct max31730_abs_data {
	int temp_min;		/* in milliDegreeC */
	int l_temp_max;		/* in milliDegreeC */
	int r1_temp_max;	/* in milliDegreeC */
	int r2_temp_max;	/* in milliDegreeC */
	int r3_temp_max;	/* in milliDegreeC */
};

struct max31730_data {
	struct i2c_client		*client;
	struct device			*dev;
	struct attribute_group		*attr_grp;
	struct max31730_abs_data	*pdata;
	struct work_struct		work;
	struct mutex			lock;
	u8	channel;
	u8	orig_config;
	u8	current_config;
	bool	enable;
};

struct smbus_alert_data {
	unsigned short	addr;
	enum i2c_alert_protocol	type;
	unsigned int	data;
};

static const char * const max31730_channel_num[] = {
	"0", /* Local Channel */
	"1", /* Remote 1 Channel */
	"2", /* Remote 2 Channel */
	"3", /* Remote 3 Channel */
};

static int max31730_get_channel_num(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct max31730_data *data = iio_priv(indio_dev);

	//TODO:Remove
	dev_warn(&data->client->dev,
			"Reading channel number: %d\n", data->channel);
	return data->channel;
}

static int max31730_set_channel_num(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int number)
{
	struct max31730_data *data = iio_priv(indio_dev);

	data->channel = number;
	//TODO:Remove
	dev_warn(&data->client->dev,
			"Setting channel number to %d\n", data->channel);
	return 0;
}

static const struct iio_enum max31730_channel_num_enum = {
	.items = max31730_channel_num,
	.num_items = ARRAY_SIZE(max31730_channel_num),
	.get = max31730_get_channel_num,
	.set = max31730_set_channel_num,
};

static const struct iio_chan_spec_ext_info
			max31730_channel_ext_info[] = {
	IIO_ENUM("channel_selection_mode", IIO_SHARED_BY_TYPE,
			 &max31730_channel_num_enum),
	IIO_ENUM_AVAILABLE("channel_selection_mode",
			 &max31730_channel_num_enum),
	{ },
};

#define MAX31730_CHANNEL(num)						\
	{								\
		.type = IIO_TEMP,					\
		.channel = (num),					\
		.indexed = 1,						\
		.scan_index = (num),					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 12,					\
			.storagebits = 16,				\
			.shift = 4,					\
			.endianness = IIO_BE,				\
		},							\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				      BIT(IIO_CHAN_INFO_OFFSET) |	\
				      BIT(IIO_CHAN_INFO_ENABLE),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.ext_info = max31730_channel_ext_info,			\
	}

static const struct iio_chan_spec max31730_channels[] = {
	MAX31730_CHANNEL(0),
	MAX31730_CHANNEL(1),
	MAX31730_CHANNEL(2),
	MAX31730_CHANNEL(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

static inline long max31730_convert_raw_to_deg(int temp)
{
	return DIV_ROUND_CLOSEST((temp >> 4) * 1000, 16);
}

static int max31730_write_config(struct max31730_data *data,
				 u8 set_mask, u8 clr_mask)
{
	u8 value;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	dev_warn(&data->client->dev,
		"Current config byte: %d\n", data->current_config);
	value = data->current_config & ~clr_mask;
	dev_warn(&data->client->dev,
		"After clear_mask, config byte: %d\n", value);
	value |= set_mask;
	dev_warn(&data->client->dev,
		"After set_mask, config byte: %d\n", value);
	if (data->current_config != value) {
		int err;

		err = i2c_smbus_write_byte_data(data->client,
					MAX31730_REG_CONFIG, value);
		if (err) {
			dev_err(&data->client->dev,
				"Write Config Error: %d\n", err);
			return err;
		}

		if (!((set_mask == MAX31730_BIT_POR ||
			   clr_mask == ~MAX31730_BIT_POR) ||
			  (set_mask == MAX31730_BIT_ONE_SHOT ||
			   clr_mask == ~MAX31730_BIT_ONE_SHOT)))
			data->current_config = value;
	}

	//TODO:Remove
	dev_warn(&data->client->dev,
		"...Leaving %s, config byte: %d\n", __func__, value);
	return 0;
}

static ssize_t max31730_show_enable(struct i2c_client *client,
			int reg, int channel, char *buf)
{
	int ret;
	bool enable;

	dev_warn(&client->dev, "Entering %s...\n", __func__);
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return ret;

	/* shows only enable bit of selected channel, not enable byte. */
	enable = !!(ret & BIT(channel));
	dev_warn(&client->dev,
		 "...Leaving %s, enable bit:%d, enable byte:%d\n",
			 __func__, enable, ret);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t max31730_set_enable(struct i2c_client *client,
				int reg, int channel, bool enable)
{
	int err;
	u8 ret;

	dev_warn(&client->dev, "Entering %s...\n", __func__);
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return ret;

	dev_warn(&client->dev, "Old enable byte: %d\n", ret);
	/* sets only enable bit of selected channel, not enable byte. */
	if (enable)
		ret |= BIT(channel);
	else
		ret &= ~BIT(channel);

	err = i2c_smbus_write_byte_data(client, reg, ret);
	if (err) {
		dev_warn(&client->dev, "Set enable error: %d\n", err);
		return err;
	}

	dev_warn(&client->dev,
		"New enable byte: %d, enable bit: %d\n", enable, ret);
	dev_warn(&client->dev, "...Leaving %s\n", __func__);
	return 0;
}

static ssize_t show_fault(struct max31730_data *data, int reg,
			  u8 faultbit, char *buf)
{
	int ret;
	bool fault;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	ret = i2c_smbus_read_byte_data(data->client, reg);
	if (ret < 0)
		return ret;

	fault = !!(ret & faultbit);
	dev_warn(&data->client->dev,
		"...Leaving %s, fault bit: %d\n", __func__, fault);
	return sprintf(buf, "%d\n", fault);
}

static int max31730_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct max31730_data *data = iio_priv(indio_dev);
	int ret, offset;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/* reading reference temperature */
		ret = i2c_smbus_read_word_swapped(data->client,
				MAX31730_REG_REF_TEMP + (data->channel * 2));
		if (ret < 0) {
			dev_err(&data->client->dev,
				"Failed to read raw value, error: %d\n", ret);
			return ret;
		}

		dev_warn(&data->client->dev,
			"channel: %d, temp_raw value: %d\n",
			data->channel, ret);
		*val = max31730_convert_raw_to_deg(ret);
		dev_warn(&data->client->dev,
			"temp_raw value is converted to deg: %d\n", *val);
		break;
	case IIO_CHAN_INFO_OFFSET:
		/* Offset does not apply to Channel 0*/
		if (data->channel == 0) {
			dev_err(&data->client->dev,
				"Offset does not applied to Channel %d\n",
						data->channel);
			return -EINVAL;
		}

		ret = i2c_smbus_read_byte_data(data->client,
						MAX31730_REG_OFFSET_EN);
		/* check if the channel offset is enabled */
		if (!(!!(ret & BIT(data->channel)))) {
			dev_warn(&data->client->dev,
				"Channel ofset not enabled for Channel %d\n",
						data->channel);
			*val = 0;
			return -EINVAL;
		}

		offset = i2c_smbus_read_byte_data(data->client,
					MAX31730_REG_CUSTOM_OFFSET);
		ret = DIV_ROUND_CLOSEST(offset*1000, 8) - 14875;
		dev_warn(&data->client->dev,
			 "temp_offset: %d, calculated temp_offset: %d\n",
				 offset, ret);
		if ((ret < -14875) || (ret > 17000)) {
			dev_err(&data->client->dev,
					"Out of temperature offset range\n");
			return ret;
		}

		*val = ret;
		dev_warn(&data->client->dev,
			"temp_offset converted to %d degree\n", ret);
		break;
	case IIO_CHAN_INFO_ENABLE:
		ret = i2c_smbus_read_byte_data(data->client,
					MAX31730_REG_CHANNEL_EN);
		if (ret < 0)
			return -EINVAL;

		*val = !!(ret & BIT(data->channel));
		dev_warn(&data->client->dev,
			"temp_enable bit: %d, temp_enable byte:%d\n",
			*val, ret);
		break;
	default:
		return -EINVAL;
	}

	dev_warn(&data->client->dev, "...Leaving %s\n", __func__);
	return IIO_VAL_INT;
}

static int max31730_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct max31730_data *data = iio_priv(indio_dev);
	int ret, err;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/* write reference temperature */
		val = clamp_val(val, MAX31730_TEMP_MIN, MAX31730_TEMP_MAX);
		dev_warn(&data->client->dev,
			"temp_raw value clamped:%d\n", val);
		val = DIV_ROUND_CLOSEST(val << 4, 1000) << 4;
		dev_warn(&data->client->dev,
			"temp_raw value converted to %d degree\n", val);
		err = i2c_smbus_write_word_swapped(data->client,
				MAX31730_REG_REF_TEMP + (data->channel * 2),
				(u16)val);
		if (err) {
			dev_err(&data->client->dev,
				"Writing temp_raw error: %d\n", err);
			return err;
		}

		break;
	case IIO_CHAN_INFO_OFFSET:
		val = clamp_val(val, -14875, 17000) + 14875;
		dev_warn(&data->client->dev,
			"temp_raw value clamped:%d\n", val);
		val = DIV_ROUND_CLOSEST(val, 125);
		dev_warn(&data->client->dev,
			"temp_offset value converted to %d degree\n", val);
		err = i2c_smbus_write_byte_data(data->client,
					 MAX31730_REG_CUSTOM_OFFSET, (u8)val);
		if (err) {
			dev_err(&data->client->dev,
				"Writing temp_offset error: %d\n", err);
			return err;
		}

		break;
	case IIO_CHAN_INFO_ENABLE:
		if (val != 0 && val != 1)
			return -EINVAL;
		dev_warn(&data->client->dev,
			"tempenable bit: %d", val);
		ret = max31730_set_enable(data->client,
					  MAX31730_REG_CHANNEL_EN,
					  data->channel, val);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"Writing temp_enable error: %d\n", ret);
			return ret;
		}

		break;
	default:
		return -EINVAL;
	}

	dev_warn(&data->client->dev, "...Leaving %s\n", __func__);
	return 0;
}

static int __maybe_unused max31730_smbalert_protocol(
						struct i2c_client *client)
{
	struct i2c_client *ara;
	struct smbus_alert_data data;
	int status;

	ara = client;
	//ara->addr = MAX31730_ALERT_RESP_ADDR;

	dev_warn(&ara->dev, "Entering %s...\n", __func__);
	dev_warn(&ara->dev, "Ara address is %d\n", ara->addr);

	status = i2c_smbus_read_byte(ara);
	dev_warn(&ara->dev, "Status is %d\n", status);
	if (status < 0)
		return -EINVAL;

	data.data = status & 1;
	data.addr = status >> 1;
	data.type = I2C_PROTOCOL_SMBUS_ALERT;

	dev_warn(&ara->dev, "SMBALERT# from dev 0x%02x, data %d\n",
		data.addr, data.data);
	dev_warn(&ara->dev, "...Leaving %s\n", __func__);
	return 0;
}

static irqreturn_t max31730_interrupt_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct max31730_data *data = iio_priv(indio_dev);

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	disable_irq_nosync(irq);
	schedule_work(&data->work);
	dev_warn(&data->client->dev, "...Leaving %s\n", __func__);
	return IRQ_HANDLED;
}

static void max31730_work(struct work_struct *work)
{
	struct max31730_data *data = container_of(work,
			struct max31730_data, work);
	struct iio_dev *indio_dev = iio_priv_to_dev(data);
	/* int ret_low, ret_high;
	 * bool condition_low, condition_high;
	 */

	mutex_lock(&data->lock);

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	iio_push_to_buffers_with_timestamp(indio_dev, data,
					   iio_get_time_ns(indio_dev));

	/* ret_low = i2c_smbus_read_word_swapped(data->client,
	 *			MAX31730_REG_STATUS_LOW);
	 * ret_high = i2c_smbus_read_word_swapped(data->client,
	 *			MAX31730_REG_STATUS_HIGH);
	 *
	 * condition_low = !!((ret_low < 0) || !(ret_low &
	 *		   (MAX31730_BIT_LOCAL |
	 *			MAX31730_BIT_REMOTE_1 |
	 *			MAX31730_BIT_REMOTE_2 |
	 *			MAX31730_BIT_REMOTE_3)));
	 * condition_high = !!((ret_high < 0) || !(ret_high &
	 *		   (MAX31730_BIT_LOCAL |
	 *			MAX31730_BIT_REMOTE_1 |
	 *			MAX31730_BIT_REMOTE_2 |
	 *			MAX31730_BIT_REMOTE_3)));
	 *
	 * if (!condition_low || !condition_high) {
	 *	 dev_warn(&data->client->dev, "No interrupt to handle()...\n");
	 *	 goto unlock;
	 * }
	 *
	 * dev_warn(&data->client->dev,
	 *	 "int condition low: %d, int condition high: %d\n",
	 *	 condition_low, condition_high);
	 * if ((ret_low & MAX31730_BIT_LOCAL) ||
	 *	(ret_low & MAX31730_BIT_REMOTE_1) ||
	 *	(ret_low & MAX31730_BIT_REMOTE_2) ||
	 *	(ret_low & MAX31730_BIT_REMOTE_3) ||
	 *	(ret_high & MAX31730_BIT_LOCAL) ||
	 *	(ret_high & MAX31730_BIT_REMOTE_1) ||
	 *	(ret_high & MAX31730_BIT_REMOTE_2) ||
	 *	(ret_high & MAX31730_BIT_REMOTE_3))
	 *	iio_push_event(indio_dev,
	 *			IIO_MOD_EVENT_CODE(IIO_TEMP, 0,
	 *				IIO_MOD_TEMP_OBJECT,
	 *				IIO_EV_TYPE_THRESH,
	 *				IIO_EV_DIR_FALLING),
	 *				iio_get_time_ns(indio_dev));
	 *
	 * max31730_smbalert_protocol(data->client);
	 * smbus_do_alert(data->client->dev, data);
	 * i2c_handle_smbus_alert(data->client);
	 */
	enable_irq(data->client->irq);
	dev_warn(&data->client->dev, "...Leaving %s\n", __func__);
	mutex_unlock(&data->lock);
}

static struct max31730_abs_data *max31730_parse_data(
				struct i2c_client *client)
{
	struct max31730_abs_data *pdata;
	struct device_node *dn;
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct max31730_data *data = iio_priv(indio_dev);
	int ret;

	dev_warn(&client->dev, "Entering %s...\n", __func__);
	dn = indio_dev->dev.of_node;
	pdata = data->pdata;
	if (!pdata)
		return NULL;

	ret = of_property_read_u32(dn, "all-temp-min",
				   &pdata->temp_min);
	if (ret)
		pdata->temp_min = (-55000);	/* milliDegreeC */

	ret = of_property_read_u32(dn, "local-temp-max",
				   &pdata->l_temp_max);
	if (ret)
		pdata->l_temp_max = 127000; /* milliDegreeC */

	ret = of_property_read_u32(dn, "remote1-temp-max",
				   &pdata->r1_temp_max);
	if (ret)
		pdata->r1_temp_max = 127000; /* milliDegreeC */

	ret = of_property_read_u32(dn, "remote2-temp-max",
				   &pdata->r2_temp_max);
	if (ret)
		pdata->r2_temp_max = 127000; /* milliDegreeC */

	ret = of_property_read_u32(dn, "remote3-temp-max",
				   &pdata->r3_temp_max);
	if (ret)
		pdata->r3_temp_max = 127000; /* milliDegreeC */

	dev_warn(&client->dev, "...Leaving %s\n", __func__);
	return pdata;
}

static int max31730_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct max31730_data *data = iio_priv(indio_dev);
	int err;

	dev_warn(&client->dev, "Entering %s...\n", __func__);
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	devm_iio_device_unregister(&client->dev, indio_dev);
	err = i2c_smbus_write_byte_data(client, MAX31730_REG_CONFIG,
				  data->orig_config);
	if (err)
		return err;

	dev_warn(&client->dev, "...Leaving %s\n", __func__);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int __maybe_unused max31730_suspend(struct device *dev)
{
	struct max31730_data *data = iio_priv(i2c_get_clientdata
				     (to_i2c_client(dev)));

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	if (data->client->irq) {
		disable_irq(data->client->irq);
		enable_irq_wake(data->client->irq);
	}

	return max31730_write_config(data, (u8)MAX31730_BIT_STOP, 0);
}

static int __maybe_unused max31730_resume(struct device *dev)
{
	struct max31730_data *data = iio_priv(i2c_get_clientdata
				     (to_i2c_client(dev)));

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	if (data->client->irq) {
		disable_irq_wake(data->client->irq);
		enable_irq(data->client->irq);
	}

	return max31730_write_config(data, 0, (u8)MAX31730_BIT_STOP);
}
#endif

static SIMPLE_DEV_PM_OPS(max31730_pm_ops, max31730_suspend,
					max31730_resume);

static ssize_t config_byte_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	dev_warn(&data->client->dev,
		"...Leaving %s, config byte: %d\n", __func__,
		data->current_config);
	*buf = data->current_config;
	return data->current_config;
}

static ssize_t config_standby_mode_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Configuring standby-mode...\n");
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val) /* ADC disabled */
		ret = max31730_write_config(data, (u8)MAX31730_BIT_STOP, 0);
	else /* ADC enabled */
		ret = max31730_write_config(data, 0, (u8)MAX31730_BIT_STOP);

	if (ret < 0)
		return -EINVAL;

	dev_warn(&data->client->dev,
		"...Standby-mode done with %d\n, stop bit: %lu", ret, val);
	return count;
}

static ssize_t config_power_on_reset_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Configuring power-on-reset...\n");
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	ret = max31730_write_config(data, (u8)MAX31730_BIT_POR, 0);
	if (ret < 0)
		return -EINVAL;

	dev_warn(&data->client->dev,
			"...Power-on-reset done with %d\n", ret);
	return count;
}

static ssize_t config_timeout_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Configuring timeout...\n");
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val) {
		/* Timeout disabled */
		ret = max31730_write_config(data, (u8)MAX31730_BIT_TIMEOUT, 0);
	} else {
		/* Timeout enabled */
		ret = max31730_write_config(data, 0, (u8)MAX31730_BIT_TIMEOUT);
	}

	if (ret < 0)
		return -EINVAL;

	dev_warn(&data->client->dev,
		"...Timeout done with %d\n, enable bit: %lu", ret, val);
	return count;
}

static ssize_t config_mode_select_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Configuring mode select...\n");
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val) {	/* Comparator */
		ret = max31730_write_config(data,
				(u8)MAX31730_BIT_INT_COMP, 0);
	} else {	/* Interrupt */
		ret = max31730_write_config(data,
				0, (u8)MAX31730_BIT_INT_COMP);
	}

	if (ret < 0)
		return -EINVAL;

	dev_warn(&data->client->dev,
		"...Mode select done with %d\n, mode: %lu", ret, val);
	return count;
}

static ssize_t config_fault_queue_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Configuring fault queue...\n");
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val == 6)
		ret = max31730_write_config(data,
					    MAX31730_FIELD_FAULT_QUEUE_11b, 0);
	else if (val == 4)
		ret = max31730_write_config(data,
					    MAX31730_FIELD_FAULT_QUEUE_10b, 0);
	else if (val == 2)
		ret = max31730_write_config(data,
					    MAX31730_FIELD_FAULT_QUEUE_01b, 0);
	else /* val == 1 */
		ret = max31730_write_config(data,
					    0, MAX31730_FIELD_FAULT_QUEUE_11b);

	if (ret < 0)
		return -EINVAL;

	dev_warn(&data->client->dev,
		"...Fault queue done with %d\n, queue: %lu", ret, val);
	return count;
}

static ssize_t config_extrange_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Configuring extended range...\n");
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val) {	/* Extrange */
		ret = max31730_write_config(data,
				(u8)MAX31730_BIT_EXTRANGE, 0);
	} else {	/* No extrange */
		ret = max31730_write_config(data,
				0, (u8)MAX31730_BIT_EXTRANGE);
	}

	if (ret < 0)
		return -EINVAL;

	dev_warn(&data->client->dev,
		"...Extended range done with %d\n, extrange: %lu", ret, val);
	return count;
}

static ssize_t config_one_shot_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Configuring one shot...\n");
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	ret = data->current_config;
	if (!(ret & MAX31730_BIT_STOP)) {
		dev_err(&data->client->dev,
			"One shot can only be enabled within stop mode\n");
		return -EINVAL;
	}

	ret = max31730_write_config(data, (u8)MAX31730_BIT_ONE_SHOT, 0);
	if (ret < 0)
		return -EINVAL;

	dev_warn(&data->client->dev, "...One shot done with %d\n", ret);
	return count;
}

static ssize_t show_temp_highest_en(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	return max31730_show_enable(data->client,
					MAX31730_REG_HIGHEST_TEMP_EN,
					data->channel, buf);
}

static ssize_t store_temp_highest_en(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_warn(&data->client->dev, "channel: %d, enable value: %lu\n",
				  data->channel, val);
	ret = max31730_set_enable(data->client,
				  MAX31730_REG_HIGHEST_TEMP_EN,
				  data->channel, !!val);
	if (ret < 0)
		return ret;

	dev_warn(&data->client->dev,
		"...Leaving %s with %d\n", __func__, ret);
	return count;
}

static ssize_t show_custom_ideality_en(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);

	if (data->channel == 0) {
		dev_warn(&data->client->dev,
			"No ideality enable to show for channel 0\n");
		return -EINVAL;
	}
	dev_warn(&data->client->dev, "...%s...\n", __func__);
	return max31730_show_enable(data->client,
					MAX31730_REG_CUSTOM_IDEALITY_EN,
					data->channel, buf);
}

static ssize_t store_custom_ideality_en(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	if (data->channel == 0) {
		dev_warn(&data->client->dev,
			"No ideality enable to store for channel 0\n");
		return -EINVAL;
	}

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_warn(&data->client->dev, "channel: %d, enable value: %lu\n",
					data->channel, val);
	ret = max31730_set_enable(data->client,
				  MAX31730_REG_CUSTOM_IDEALITY_EN,
				  data->channel, !!val);
	if (ret < 0)
		return ret;

	dev_warn(&data->client->dev,
		"...Leaving %s with %d\n", __func__, ret);
	return count;
}

static ssize_t show_custom_offset_en(struct device *dev,
			struct device_attribute *atrr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	if (data->channel == 0) {
		dev_warn(&data->client->dev,
			"No beta offset value for channel 0\n");
		return -EINVAL;
	}

	return max31730_show_enable(data->client,
					MAX31730_REG_OFFSET_EN,
					data->channel, buf);
}

static ssize_t store_custom_offset_en(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	if (data->channel == 0) {
		dev_warn(&data->client->dev,
			"No beta offset value for channel 0\n");
		return -EINVAL;
	}

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_warn(&data->client->dev, "channel: %d, enable value: %lu\n",
					data->channel, val);
	ret = max31730_set_enable(data->client,
				  MAX31730_REG_OFFSET_EN,
				  data->channel, !!val);
	if (ret < 0)
		return ret;

	dev_warn(&data->client->dev,
		"...Leaving %s with %d\n", __func__, ret);
	return count;
}

static ssize_t show_filter_en(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	if (data->channel == 0) {
		dev_warn(&data->client->dev,
			"No filter enable for channel 0\n");
		return -EINVAL;
	}

	return max31730_show_enable(data->client,
					MAX31730_REG_FILTER_EN,
					data->channel, buf);
}

static ssize_t store_filter_en(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	if (data->channel == 0) {
		dev_warn(&data->client->dev,
			"No filter enable for channel 0\n");
		return -EINVAL;
	}

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_warn(&data->client->dev, "channel: %d, enable value: %lu\n",
					data->channel, val);
	ret = max31730_set_enable(data->client,
				  MAX31730_REG_FILTER_EN,
				  data->channel, !!val);
	if (ret < 0)
		return ret;

	dev_warn(&data->client->dev,
		"...Leaving %s with %d\n", __func__, ret);
	return count;
}

static ssize_t show_beta_compen_en(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	return max31730_show_enable(data->client,
					MAX31730_REG_BETA_COMPEN_EN,
					data->channel, buf);
}

static ssize_t store_beta_compen_en(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_warn(&data->client->dev, "channel: %d, enable value: %lu\n",
					data->channel, val);
	ret = max31730_set_enable(data->client,
				  MAX31730_REG_BETA_COMPEN_EN,
				  data->channel, !!val);
	if (ret < 0)
		return ret;

	dev_warn(&data->client->dev,
		"...Leaving %s with %d\n", __func__, ret);
	return count;
}

static ssize_t show_temp_highest(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	int ret;
	long regval;

	ret = i2c_smbus_read_word_swapped(data->client,
					MAX31730_REG_HIGHEST_TEMP);
	if (ret < 0)
		return ret;

	regval = max31730_convert_raw_to_deg(ret);
	dev_warn(&data->client->dev,
		"...%s raw: %d deg: %ld...\n", __func__, ret, regval);
	return sprintf(buf, "%ld\n", regval);
}

static ssize_t show_beta_value(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	int ret;

	if (data->channel == 0) {
		dev_warn(&data->client->dev,
			"No beta value for channel 0\n");
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(data->client,
				     MAX31730_REG_BETA_VALUE + data->channel);
	if (ret < 0)
		return ret;

	dev_warn(&data->client->dev, "...%s: %d...\n", __func__, ret);
	return sprintf(buf, "%d\n", ret);
}

static ssize_t show_ideality_factor(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	int ret;

	ret = i2c_smbus_read_byte_data(data->client,
				       MAX31730_REG_CUSTOM_IDEALITY_FACT);
	if ((ret < 0) & (ret >= 0x40)) {
		dev_warn(&data->client->dev,
				"Invalid ideality factor read: %d\n", ret);
		return ret;
	}

	dev_warn(&data->client->dev, "...%s: %d...\n", __func__, ret);
	return sprintf(buf, "%d\n", ret);
}

static ssize_t store_ideality_factor(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int err;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	val = clamp_val(val, 0, 0x3F);
	dev_warn(&data->client->dev,
			"channel: %d, entered value is: %lu\n",
			data->channel, val);

	err = i2c_smbus_write_byte_data(data->client,
				MAX31730_REG_CUSTOM_IDEALITY_FACT, (u16)val);
	if (err)
		return err;

	dev_warn(&data->client->dev,
		"...Leaving %s with %d\n", __func__, err);
	return count;
}

static ssize_t show_temp_high_limit(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	int ret;
	long regval;

	ret = i2c_smbus_read_word_swapped(data->client,
				MAX31730_REG_TEMP_MAX + (data->channel * 2));
	if (ret < 0)
		return ret;

	regval = max31730_convert_raw_to_deg(ret);
	dev_warn(&data->client->dev,
		"...%s, raw: %d deg: %ld...\n", __func__, ret, regval);
	return sprintf(buf, "%ld\n", regval);
}

static ssize_t store_temp_high_limit(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	long val;
	int err;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_warn(&data->client->dev, "channel: %d, val: %ld\n",
					data->channel, val);
	val = clamp_val(val, MAX31730_TEMP_MIN, MAX31730_TEMP_MAX);
	dev_warn(&data->client->dev, "clamped val: %ld\n", val);
	val = DIV_ROUND_CLOSEST(val << 4, 1000) << 4;
	dev_warn(&data->client->dev, "converted val: %ld\n", val);
	err = i2c_smbus_write_word_swapped(data->client,
				MAX31730_REG_TEMP_MAX + (data->channel * 2),
				(u16)val);
	if (err)
		return err;

	dev_warn(&data->client->dev, "...Leaving %s...\n", __func__);
	return count;
}

static ssize_t show_temp_low_limit(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	int ret;
	long regval;

	ret = i2c_smbus_read_word_swapped(data->client,
					MAX31730_REG_TEMP_ALL_MIN);
	if (ret < 0)
		return ret;

	regval = max31730_convert_raw_to_deg(ret);
	dev_warn(&data->client->dev,
		"...%s, raw: %d deg: %ld...\n", __func__, ret, regval);
	return sprintf(buf, "%ld\n", regval);
}

static ssize_t store_temp_low_limit(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	long val;
	int err;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_warn(&data->client->dev, "channel: %d, new val: %ld\n",
					data->channel, val);
	val = clamp_val(val, MAX31730_TEMP_MIN, MAX31730_TEMP_MAX);
	dev_warn(&data->client->dev, "clamped val: %ld\n", val);
	val = DIV_ROUND_CLOSEST(val << 4, 1000) << 4;
	dev_warn(&data->client->dev, "converted val: %ld\n", val);
	err = i2c_smbus_write_word_swapped(data->client,
					   MAX31730_REG_TEMP_ALL_MIN,
					   (u16)val);
	if (err)
		return err;

	dev_warn(&data->client->dev, "...Leaving %s...\n", __func__);
	return count;
}

static ssize_t show_status_high(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	return show_fault(data, MAX31730_REG_STATUS_HIGH,
				BIT(data->channel), buf);
}

static ssize_t show_status_low(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	return show_fault(data, MAX31730_REG_STATUS_LOW,
				BIT(data->channel), buf);
}

static ssize_t show_diode_fault(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	if (data->channel == 0) {
		dev_warn(&data->client->dev,
			"No diode faults status for channel 0\n");
		return -EINVAL;
	}
	return show_fault(data, MAX31730_REG_DIODE_FAULT_STATUS,
				BIT(data->channel), buf);
}

static ssize_t show_temp_val(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	int ret, regval;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	ret = i2c_smbus_read_byte_data(data->client,
				       MAX31730_REG_CHANNEL_EN);

	if (!(!!(ret & BIT(data->channel))))
		return -ENODATA;

	ret = i2c_smbus_read_word_swapped(data->client,
				MAX31730_REG_TEMP + (data->channel * 2));
	regval = max31730_convert_raw_to_deg(ret);
	dev_warn(&data->client->dev, "raw: %d, deg: %d\n",
				ret, regval);
	dev_warn(&data->client->dev, "...Leaving %s...\n", __func__);
	return sprintf(buf, "%d\n", regval);
}

static ssize_t show_therm_mask(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	int ret, maskbit;

	dev_warn(&data->client->dev, "...%s...\n", __func__);
	ret = i2c_smbus_read_byte_data(data->client,
				       MAX31730_REG_THERM_MASK);
	if (ret < 0)
		return ret;

	maskbit = !!(ret & BIT(data->channel));
	dev_warn(&data->client->dev,
			"maskbit: %d, maskbyte: %d\n", maskbit, ret);
	return sprintf(buf, "%d\n", maskbit);
}

static ssize_t store_therm_mask(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max31730_data *data = iio_priv(indio_dev);
	unsigned long val;
	int err;
	u8 ret;

	dev_warn(&data->client->dev, "Entering %s...\n", __func__);
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(data->client,
				       MAX31730_REG_THERM_MASK);
	if (ret < 0)
		return ret;

	if (!!(val))
		ret |= BIT(data->channel);
	else
		ret &= ~BIT(data->channel);

	dev_warn(&data->client->dev,
			"channel: %d, new val: %lu, new byte val: %d\n",
			data->channel, val, ret);
	err = i2c_smbus_write_byte_data(data->client,
					MAX31730_REG_THERM_MASK, ret);
	if (err)
		return err;

	dev_warn(&data->client->dev, "...Leaving %s...\n", __func__);
	return count;
}

static IIO_DEVICE_ATTR_RO(config_byte, 0);
static IIO_DEVICE_ATTR_WO(config_standby_mode, 0);
static IIO_DEVICE_ATTR_WO(config_power_on_reset, 0);
static IIO_DEVICE_ATTR_WO(config_timeout, 0);
static IIO_DEVICE_ATTR_WO(config_mode_select, 0);
static IIO_DEVICE_ATTR_WO(config_fault_queue, 0);
static IIO_DEVICE_ATTR_WO(config_extrange, 0);
static IIO_DEVICE_ATTR_WO(config_one_shot, 0);

static IIO_DEVICE_ATTR(temp_value,
		       0444,
		       show_temp_val,
		       NULL,
		       MAX31730_REG_TEMP);

static IIO_DEVICE_ATTR(temp_highest,
		       0444,
		       show_temp_highest,
		       NULL,
		       MAX31730_REG_HIGHEST_TEMP);

static IIO_DEVICE_ATTR(temp_highest_en,
		       0644,
		       show_temp_highest_en,
		       store_temp_highest_en,
		       MAX31730_REG_HIGHEST_TEMP_EN);

static IIO_DEVICE_ATTR(custom_offset_en,
		       0644,
		       show_custom_offset_en,
		       store_custom_offset_en,
		       MAX31730_REG_CUSTOM_OFFSET);

static IIO_DEVICE_ATTR(custom_ideality_en,
		       0644,
		       show_custom_ideality_en,
		       store_custom_ideality_en,
		       MAX31730_REG_CUSTOM_IDEALITY_EN);

static IIO_DEVICE_ATTR(custom_ideality_factor,
		       0644,
		       show_ideality_factor,
		       store_ideality_factor,
		       MAX31730_REG_CUSTOM_IDEALITY_FACT);

static IIO_DEVICE_ATTR(filter_en,
		       0644,
		       show_filter_en,
		       store_filter_en,
		       MAX31730_REG_FILTER_EN);

static IIO_DEVICE_ATTR(beta_compen_en,
		       0644,
		       show_beta_compen_en,
		       store_beta_compen_en,
		       MAX31730_REG_BETA_COMPEN_EN);

static IIO_DEVICE_ATTR(beta_value,
		       0444,
		       show_beta_value,
		       NULL,
		       MAX31730_REG_BETA_VALUE);

static IIO_DEVICE_ATTR(temp_high_limit,
		       0644,
		       show_temp_high_limit,
		       store_temp_high_limit,
		       MAX31730_REG_TEMP_MAX);

static IIO_DEVICE_ATTR(temp_low_limit,
		       0644,
		       show_temp_low_limit,
		       store_temp_low_limit,
		       MAX31730_REG_TEMP_ALL_MIN);

static IIO_DEVICE_ATTR(status_temp_high,
		       0444,
		       show_status_high,
		       NULL,
		       MAX31730_REG_STATUS_HIGH);

static IIO_DEVICE_ATTR(status_temp_low,
		       0444,
		       show_status_low,
		       NULL,
		       MAX31730_REG_STATUS_LOW);

static IIO_DEVICE_ATTR(status_diode_fault,
		       0444,
		       show_diode_fault,
		       NULL,
		       MAX31730_REG_DIODE_FAULT_STATUS);

static IIO_DEVICE_ATTR(therm_mask,
		       0644,
		       show_therm_mask,
		       store_therm_mask,
		       MAX31730_REG_THERM_MASK);

static struct attribute *max31730_attributes[] = {
	&iio_dev_attr_temp_value.dev_attr.attr,
	&iio_dev_attr_temp_highest.dev_attr.attr,
	&iio_dev_attr_temp_highest_en.dev_attr.attr,
	&iio_dev_attr_custom_offset_en.dev_attr.attr,
	&iio_dev_attr_custom_ideality_en.dev_attr.attr,
	&iio_dev_attr_custom_ideality_factor.dev_attr.attr,
	&iio_dev_attr_filter_en.dev_attr.attr,
	&iio_dev_attr_beta_compen_en.dev_attr.attr,
	&iio_dev_attr_beta_value.dev_attr.attr,
	&iio_dev_attr_temp_high_limit.dev_attr.attr,
	&iio_dev_attr_temp_low_limit.dev_attr.attr,
	&iio_dev_attr_status_temp_high.dev_attr.attr,
	&iio_dev_attr_status_temp_low.dev_attr.attr,
	&iio_dev_attr_status_diode_fault.dev_attr.attr,
	&iio_dev_attr_therm_mask.dev_attr.attr,
	&iio_dev_attr_config_byte.dev_attr.attr,
	&iio_dev_attr_config_standby_mode.dev_attr.attr,
	&iio_dev_attr_config_power_on_reset.dev_attr.attr,
	&iio_dev_attr_config_timeout.dev_attr.attr,
	&iio_dev_attr_config_mode_select.dev_attr.attr,
	&iio_dev_attr_config_fault_queue.dev_attr.attr,
	&iio_dev_attr_config_extrange.dev_attr.attr,
	&iio_dev_attr_config_one_shot.dev_attr.attr,
	NULL,
};

static const struct attribute_group max31730_attr_group = {
	.attrs = max31730_attributes,
};

static const struct iio_info max31730_info = {
	.read_raw = max31730_read_raw,
	.write_raw = max31730_write_raw,
	.attrs = &max31730_attr_group,
};


static int max31730_probe(struct i2c_client *client,
			const struct i2c_device_id *max31730_id)
{
	struct iio_dev *indio_dev;
	struct max31730_data *data;
	int status, err, ret;
	u8 set_mask, clear_mask;

	dev_warn(&client->dev, "Entering %s...\n", __func__);
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	ret = i2c_smbus_read_byte_data(client, MAX31730_REG_MFR_ID);
	if (ret != MAX31730_MFR_ID) {
		//TODO:remove
		dev_err(&client->dev, "MFD ID mismatch\n");
		return -ENODEV;
	}
	dev_warn(&client->dev, "MFR_ID is %d\n", ret);
	ret = i2c_smbus_read_byte_data(client, MAX31730_REG_MFR_REV);
	if (ret != MAX31730_MFR_REV) {
		//TODO:remove
		dev_err(&client->dev, "MFR Revision mismatch\n");
		return -ENODEV;
	}

	//TODO:Remove
	dev_warn(&client->dev, "MFR_REV is %d\n", ret);
	dev_warn(&client->dev, "MFR ID and Rev are recognized\n");

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	//TODO:Remove
	dev_warn(&client->dev, "I2C operation and alloc are done\n");

	indio_dev->info = &max31730_info;
	indio_dev->name = DRIVER_NAME;
	indio_dev->channels = max31730_channels;
	indio_dev->num_channels = ARRAY_SIZE(max31730_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &client->dev;
	indio_dev->dev.of_node = client->dev.of_node;

	if (client->dev.of_node)
		data->pdata = max31730_parse_data(client);

	//TODO:Remove
	dev_warn(&client->dev, "indio_dev and data are done\n");

	/* Cache original configuration and enable status */
	status = i2c_smbus_read_byte_data(client, MAX31730_REG_CONFIG);
	if (status < 0) {
		dev_err(&client->dev, "Configuration Error: %d, %d\n", status,
		client->addr);
		return status;
	}
	data->orig_config = status;
	data->current_config = status;

	status = i2c_smbus_read_byte_data(client, MAX31730_REG_CHANNEL_EN);
	if (status < 0) {
		dev_err(&client->dev, "Channel Enable Error: %d\n", status);
		return status;
	}

	/* Determine the config for channel enable */
	if (status) {
		set_mask = 0;
		clear_mask = MAX31730_BIT_STOP;
	} else {
		set_mask = MAX31730_BIT_STOP;
		clear_mask = 0;
	}

	err = max31730_write_config(data, set_mask, clear_mask);
	if (err) {
		dev_err(&client->dev, "Write Config Error for STOP\n");
		return err;
	}

	status = i2c_smbus_read_byte_data(client, MAX31730_REG_OFFSET_EN);
	if (status < 0) {
		dev_err(&client->dev, "Offset Enable Error\n");
		return status;
	}

	//TODO:Remove
	dev_warn(&data->client->dev, "Status check is done\n");

	if (client->irq <= 0)
		return -EINVAL;

	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL,
					max31730_interrupt_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					max31730_id->name,
					indio_dev);
	if (ret) {
		dev_err(&client->dev, "irq request error %d\n", ret);
		return ret;
	}

	INIT_WORK(&data->work, max31730_work);

	dev_warn(&data->client->dev, "IRQ setup is done\n");
	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0)
		dev_err(&data->client->dev,
				"IIO device register failed %d\n", ret);

	dev_warn(&data->client->dev,
			"IIO device register success %d\n", ret);
	dev_warn(&data->client->dev, "...Leaving %s\n", __func__);
	return ret;
}

static const struct i2c_device_id max31730_id[] = {
	{ "max31730", 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max31730_id);

static const struct of_device_id max31730_of_match[] = {
	{ .compatible = "maxim,max31730",	},
	{ }
};
MODULE_DEVICE_TABLE(of, max31730_of_match);

static struct i2c_driver max31730_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.of_match_table = of_match_ptr(max31730_of_match),
		.pm		= &max31730_pm_ops,
	},
	.probe			= max31730_probe,
	.remove			= max31730_remove,
	.id_table		= max31730_id,
	.address_list		= i2c_address,
};

module_i2c_driver(max31730_driver);

MODULE_AUTHOR("Metin Ozkan <metin.ozkan@maximintegrated.com>");
MODULE_DESCRIPTION("MAX31730 3-Channel Remote Temperature Sensor");
MODULE_LICENSE("GPL");
