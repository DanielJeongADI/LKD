// SPDX-License-Identifier: GPL-2.0-only

/*******************************************************************************
* Copyright (C) 2021 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/


#include <linux/iio/iio.h>
#include <linux/spi/spi.h>
#include <linux/module.h>

#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/iio/sysfs.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

enum max11410_supported_device_ids {
	ID_MAX11410,
};

struct max11410_state {
	struct spi_device *spi_dev;
	struct iio_trigger *trig;
	struct completion	completion;
	int gpio_interrupt;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	unsigned char data[4] ____cacheline_aligned;
};

static unsigned int max11410_reg_size(unsigned int reg)
{
	// Registers from 0x00 to 0x10 are 1 byte, the rest are 3 bytes long.
	return reg <= 0x10 ? 1 : 3;
}

static int max11410_write_reg(struct max11410_state *st,
		unsigned int reg, unsigned int val)
{
	unsigned int size;
	u8 *data;

	if (reg > 0x6F)
		return -EINVAL;

	size = max11410_reg_size(reg);

	data = st->data;

	struct spi_transfer t = {
		.tx_buf		= data,
		.len		= size + 1,
		.cs_change	= 0,
	};

	struct spi_message m;

	data[0] = (reg & 0x7F);

	switch (size) {
	case 3:
		data[1] = val >> 16;
		data[2] = val >> 8;
		data[3] = val;
		break;
	case 1:
		data[1] = val;
		break;
	default:
		return -EINVAL;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync_locked(st->spi_dev, &m);
}

static int max11410_read_reg(struct max11410_state *st,
		unsigned int reg, int *val)
{
	unsigned int size;
	u8 *data;
	int ret;

	if (reg > 0x6F)
		return -EINVAL;

	size = max11410_reg_size(reg);

	data = st->data;

	struct spi_transfer t[] = {
		{
			.tx_buf = data,
			.len = 1,
		}, {
			.rx_buf = data,
			.len = size,
			.cs_change = 0,
		},
	};
	struct spi_message m;

	data[0] = 0x80 | (reg & 0x7F);

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	ret = spi_sync_locked(st->spi_dev, &m);

	if (ret < 0)
		return ret;

	switch (size) {
	case 3:
		*val = data[0] << 16 | data[1] << 8 | data[2];
		break;
	case 1:
		*val = data[0];
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max11410_write_reg_masked(struct max11410_state *st,
		unsigned int reg, unsigned int val, unsigned int mask)
{
	int ret;
	unsigned int read_val;

	ret = max11410_read_reg(st, reg, &read_val);
	if (ret)
		return ret;

	read_val &= ~mask;
	read_val |= val & mask;
	return max11410_write_reg(st, reg, read_val);
}

#define MAX11410_CHANNEL(idx) {				\
	.type = IIO_VOLTAGE,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.indexed = 1,					\
	.channel = idx,					\
	.scan_index = -1				\
}

static const struct iio_chan_spec adc_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 24,
			.storagebits = 24,
			.shift = 0,
			.endianness = IIO_LE
		}
	},
	MAX11410_CHANNEL(0),
	MAX11410_CHANNEL(1),
	MAX11410_CHANNEL(2),
	MAX11410_CHANNEL(3),
	MAX11410_CHANNEL(4),
	MAX11410_CHANNEL(5),
	MAX11410_CHANNEL(6),
	MAX11410_CHANNEL(7),
	MAX11410_CHANNEL(8),
	MAX11410_CHANNEL(9),
	IIO_CHAN_SOFT_TIMESTAMP(4)
};

static ssize_t config_store_masked(struct device *dev,
					struct device_attribute *devattr,
					size_t count, int config,
					unsigned int addr, unsigned int pos,
					unsigned int width)
{
	int err, reg;

	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct max11410_state *state = iio_priv(indio_dev);

	if (config < 0)
		return -EINVAL;

	reg = config << pos;

	err = max11410_write_reg_masked(state, addr, reg, (BIT(width) - 1) << pos);

	if (err)
		return err;

	return count;
}

static ssize_t config_show_masked(struct device *dev,
					struct device_attribute *devattr,
					char *buf, unsigned int addr,
					unsigned int pos, unsigned int width,
					const char *config[], unsigned int size)
{
	int ret;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct max11410_state *state = iio_priv(indio_dev);

	ret = max11410_read_reg(state, addr, &val);

	if (ret)
		return ret;

	val = (val >> pos) & (BIT(width) - 1);

	if (val >= size)
		return -EINVAL;

	return sprintf(buf, "%s\n", config[val]);
}


#define config_attr(_name, _addr, _pos, _width)				\
static ssize_t store_##_name(struct device *dev,			\
				struct device_attribute *devattr,	\
				const char *buf, size_t count)		\
{									\
	return config_store_masked(dev, devattr, count,			\
		sysfs_match_string(_name, buf),				\
		_addr, _pos, _width);					\
}									\
static ssize_t show_##_name(struct device *dev,				\
				struct device_attribute *devattr,	\
				char *buf)				\
{									\
	return config_show_masked(dev, devattr, buf, _addr, _pos,	\
		_width, _name, sizeof(_name));				\
}									\
static IIO_DEVICE_ATTR(_name, 0644, show_##_name, store_##_name, 0)

static int max11410_sample(struct max11410_state *st, int *sample_raw)
{
	int val, ret;
	int retry = 100;


	if (st->gpio_interrupt > 0)
		reinit_completion(&st->completion);

	// Start Conversion
	ret = max11410_write_reg(st, 0x01, 0x00);

	if (ret)
		return ret;


	if (st->gpio_interrupt > 0) {
		// Wait for interrupt.
		ret = wait_for_completion_timeout(&st->completion,
						msecs_to_jiffies(2000));
		if (!ret)
			return -ETIMEDOUT;

	} else {
		// Wait for status register Conversion Ready flag
		do {
			usleep_range(1000, 20000);

			ret = max11410_read_reg(st, 0x38, &val);

			if (ret)
				return ret;

			if (--retry <= 0)
				return -ETIMEDOUT;

		} while ((val & 1) == 0);
	}

	// Read ADC Data
	return max11410_read_reg(st, 0x30, sample_raw);
}

static ssize_t oneshot(struct device *dev,
				struct device_attribute *devattr,
				char *buf)
{
	int ret;
	unsigned int val = 0;

	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct max11410_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;

	ret = max11410_sample(st, &val);

	if (ret)
		return ret;

	return sprintf(buf, "%d\n", val);
}

static IIO_DEVICE_ATTR(oneshot, 0444, oneshot, NULL, 0);

static const char * const config_calib_start[] = {
	"self", "pga"
};

static ssize_t config_store_calib_start(struct device *dev,
				struct device_attribute *devattr,
				const char *buf, size_t count)
{
	int ret;
	unsigned int val = 0;
	int retry = 100;

	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct max11410_state *st = iio_priv(indio_dev);

	ret = config_store_masked(dev, devattr, count,
	sysfs_match_string(config_calib_start, buf),
	0x03, 0x00, 0x03);

	if (ret != count)
		return ret;
	// Wait for status register Calibration Ready flag
	do {
		msleep(50);

		ret = max11410_read_reg(st, 0x38, &val);

		if (ret)
			return ret;

		if (--retry <= 0)
			return -ETIMEDOUT;

	} while ((val & (1 << 2)) == 0);

	return count;
}

static IIO_DEVICE_ATTR(config_calib_start, 0664, NULL, config_store_calib_start, 0);

static const char * const config_input_path[] = {
	"buffered", "bypass", "pga"
};

config_attr(config_input_path, 0x0E, 4, 2);

static const char * const config_gain[] = {
	"1", "2", "4", "8", "16", "32", "64", "128"
};

config_attr(config_gain, 0x0E, 0, 4);

static const char * const config_ainp[] = {
"ain0", "ain1", "ain2", "ain3", "ain4", "ain5", "ain6", "ain7", "ain8", "ain9",
"avdd", "unconnected", "unconnected", "unconnected", "unconnected", "unconnected"};

config_attr(config_ainp, 0x0B, 4, 4);

static const char * const config_ainn[] = {
"ain0", "ain1", "ain2", "ain3", "ain4", "ain5", "ain6", "ain7", "ain8", "ain9",
"gnd", "unconnected", "unconnected", "unconnected", "unconnected", "unconnected"};

config_attr(config_ainn, 0x0B, 0, 4);

static const char * const config_input_range[] = {"bipolar", "unipolar"};

config_attr(config_input_range, 0x09, 6, 1);

static const char * const config_filter[] = {"fir50/60", "fir50", "fir60", "sinc4"};

config_attr(config_filter, 0x08, 4, 2);

static const char * const config_rate[] = {
"0", "1", "2", "3", "4", "5", "6", "7",
"8", "9", "10", "11", "12", "13", "14", "15"};

config_attr(config_rate, 0x08, 0, 4);

static const char * const config_ref_sel[] = {
"ref0p/ref0n", "ref1p/ref1n", "ref2p/ref2n", "avdd/agnd",
"ref0p/gnd", "ref1p/gnd", "ref2p/gnd", "avdd/agnd"};

config_attr(config_ref_sel, 0x09, 0, 3);


static struct attribute *max11410_attributes[] = {
	&iio_dev_attr_config_gain.dev_attr.attr,
	&iio_dev_attr_config_input_path.dev_attr.attr,
	&iio_dev_attr_config_ainp.dev_attr.attr,
	&iio_dev_attr_config_ainn.dev_attr.attr,
	&iio_dev_attr_config_input_range.dev_attr.attr,
	&iio_dev_attr_config_filter.dev_attr.attr,
	&iio_dev_attr_config_rate.dev_attr.attr,
	&iio_dev_attr_config_ref_sel.dev_attr.attr,
	&iio_dev_attr_config_calib_start.dev_attr.attr,
	&iio_dev_attr_oneshot.dev_attr.attr,
	NULL
};

static const struct attribute_group max11410_attribute_group = {
	.attrs = max11410_attributes,
};

static int max11410_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val, int *val2,
		long info)
{
	int ret, sample_raw;
	struct max11410_state *state = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;

	if (info != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);

	if (chan->indexed) {
		// If we are reading from an index channel,
		// Configure Max11410 input channels accordingly.
		if (chan->channel >= 0 && chan->channel < 10) {
			struct device *dev = &state->spi_dev->dev;
			int ch = chan->channel;

			store_config_ainn(dev, NULL, "gnd", 0);
			store_config_ainp(dev, NULL, config_ainp[ch], 0);
		}
	}

	ret = max11410_sample(state, &sample_raw);

	mutex_unlock(&indio_dev->mlock);

	if (ret)
		return ret;

	*val = sample_raw;

	return IIO_VAL_INT;
}

static const struct iio_info max11410_info = {
	.read_raw = max11410_read_raw,
	.attrs = &max11410_attribute_group
};

static irqreturn_t max11410_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct max11410_state *st = iio_priv(indio_dev);
	int adc_value;
	static char data[8];

	max11410_read_reg(st, 0x30, &adc_value);
	memcpy(data, &adc_value, 4);

	if (iio_buffer_enabled(indio_dev)) {
		iio_push_to_buffers_with_timestamp(indio_dev, data,
						iio_get_time_ns(indio_dev));
	}

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static void max11410_enable_interrupt(struct max11410_state *st)
{
	// Configure GPIO1 of MAX11410 as interrupt output.
	max11410_write_reg(st, 0x05, 0xC1);
}

static int max11410_buffer_postenable(struct iio_dev *indio_dev)
{
	struct max11410_state *st = iio_priv(indio_dev);

	// Enable interrupt pin of max11410
	max11410_enable_interrupt(st);
	// Start continuous conversion.
	return max11410_write_reg(st, 0x01, 0x01);
}

static int max11410_buffer_predisable(struct iio_dev *indio_dev)
{
	struct max11410_state *st = iio_priv(indio_dev);

	// Stop continuous conversion.
	return max11410_write_reg(st, 0x01, 0x00);
}

static const struct iio_buffer_setup_ops max11410_buffer_ops = {
	.postenable = &max11410_buffer_postenable,
	.predisable = &max11410_buffer_predisable,
};

static irqreturn_t max11410_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct max11410_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev))
		iio_trigger_poll_chained(st->trig);
	else
		complete(&st->completion);

	return IRQ_HANDLED;
};

static int max11410_probe(struct spi_device *spi)
{
	struct max11410_state *st;
	struct iio_dev *indio_dev;
	int trigger_gpio = -1;
	int ret = 0;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));

	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi_dev = spi;

	spi_set_drvdata(spi, indio_dev);

	// Read GPIO 1 pin
	ret = device_property_read_u32(&spi->dev, "max11410-gpio1", &trigger_gpio);
	if (ret)
		st->gpio_interrupt = -1;
	else
		st->gpio_interrupt = trigger_gpio;

	init_completion(&st->completion);


	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	indio_dev->channels = adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(adc_channels);
	indio_dev->info = &max11410_info;

	if (st->gpio_interrupt > 0) {

		st->trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d",
						indio_dev->name,
						indio_dev->id);
		if (!st->trig)
			return -ENOMEM;


		iio_trigger_set_drvdata(st->trig, indio_dev);
		ret = devm_iio_trigger_register(&spi->dev, st->trig);
		if (ret)
			return ret;

		indio_dev->trig = iio_trigger_get(st->trig);

		ret = devm_request_threaded_irq(&spi->dev, gpio_to_irq(st->gpio_interrupt),
						NULL,
						&max11410_interrupt,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"max11410", indio_dev);
		if (ret)
			return ret;

		ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
							NULL,
							&max11410_trigger_handler,
							&max11410_buffer_ops);
		if (ret)
			return ret;

		max11410_enable_interrupt(st);
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id max11410_spi_of_id[] = {
	{ .compatible = "maxim,max11410" },
	{ }
};

MODULE_DEVICE_TABLE(of, max11410_spi_of_id);

static const struct spi_device_id max11410_id[] = {
	{"max11410", ID_MAX11410},
	{}
};

MODULE_DEVICE_TABLE(spi, max11410_id);

static struct spi_driver max11410_driver = {
	.driver = {
		.name	= "max11410",
		.owner	= THIS_MODULE,
		.of_match_table = max11410_spi_of_id
	},

	.probe		= max11410_probe,
	.id_table	= max11410_id,
};

module_spi_driver(max11410_driver);



MODULE_AUTHOR("David Jung <david.jung@maximintegrated.com>");
MODULE_AUTHOR("Ibrahim Tilki <ibrahim.tilki@maximintegrated.com>");
MODULE_DESCRIPTION("Maxim Integrated MAX11410 ADC");
MODULE_LICENSE("GPL v2");
