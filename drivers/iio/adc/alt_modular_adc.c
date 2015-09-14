/*
 * Copyright (C) 2015 Altera Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/iio/iio.h>

/* Constant Definitions */
#define MAX_SLOT		64
#define MAX_ADC			2
#define MAX_CHANNEL		18
#define MODE_SINGLE_ADC		1
#define MODE_DUAL_ADC		2
#define ADC_BITS		12
#define ADC_STORE_BITS		16
#define ADC_MAX_STR_SIZE	20

/* Register Definitions */
#define ADC_CMD_REG		0x0
#define ADC_IER_REG		0x100
#define ADC_ISR_REG		0x104

#define ADC_RUN_MSK		0x1
#define ADC_SINGLE_MKS		0x2
#define ADC_STOP_MSK		0x0
#define ADC_LOW_MSK		0xFFF
#define ADC_HIGH_MSK		0xFFF0000

struct altera_adc {
	void __iomem		*seq_regs;
	void __iomem		*sample_regs;

	unsigned int		mode;
	unsigned int		slot_count;
	unsigned int		slot_sequence[MAX_ADC][MAX_SLOT];
	unsigned int		adc_number;
};

static int alt_modular_adc_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val,
				int *val2,
				long mask)
{
	struct altera_adc *adc = iio_priv(indio_dev);
	u32 value;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	value = readl_relaxed(adc->sample_regs + (chan->address * 4));

	*val = (value & ADC_LOW_MSK);

	return IIO_VAL_INT;
}

static const struct iio_info adc_iio_info = {
	.read_raw = &alt_modular_adc_read_raw,
	.driver_module = THIS_MODULE,
};

static void alt_modular_adc_kfree_names(struct iio_dev *indio_dev)
{
	struct altera_adc *adc = iio_priv(indio_dev);
	const struct iio_chan_spec *chan = indio_dev->channels;
	int i;

	for (i = 0; i < adc->slot_count; i++, chan++)
		kfree(chan->extend_name);
}

static int alt_modular_adc_channel_init(struct iio_dev *indio_dev)
{
	struct altera_adc *adc = iio_priv(indio_dev);
	struct iio_chan_spec *chan_array;
	struct iio_chan_spec *chan;
	char channel_name[ADC_MAX_STR_SIZE];
	int i;

	chan_array = devm_kzalloc(&indio_dev->dev, (adc->slot_count *
		sizeof(*chan_array)), GFP_KERNEL);
	if (chan_array == NULL)
		return -ENOMEM;

	chan = chan_array;

	for (i = 0; i < adc->slot_count; i++, chan++) {
		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = i;
		chan->address = i;

		/* Construct iio sysfs name*/
		if (adc->mode == MODE_SINGLE_ADC) {
			snprintf(channel_name, sizeof(channel_name),
				 "adc%d-ch%d",
				 adc->adc_number,
				 adc->slot_sequence[0][i]);
		} else if (adc->mode == MODE_DUAL_ADC) {
			snprintf(channel_name, sizeof(channel_name),
				 "adc1-ch%d_adc2-ch%d",
				 adc->slot_sequence[0][i],
				 adc->slot_sequence[1][i]);
		} else {
			return -EINVAL;
		}

		chan->datasheet_name = kstrndup(channel_name,
				sizeof(channel_name), GFP_KERNEL);
		chan->extend_name = chan->datasheet_name;
		chan->scan_index = i;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = ADC_BITS;
		chan->scan_type.storagebits = ADC_STORE_BITS;
		chan->scan_type.endianness = IIO_CPU;
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	}

	indio_dev->channels = chan_array;

	return 0;
}

static const struct of_device_id alt_modular_adc_match[] = {
	{ .compatible = "altr,modular-adc-1.0" },
	{ .compatible = "altr,modular-dual-adc-1.0" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, alt_modular_adc_match);

static int alt_modular_adc_parse_dt(struct iio_dev *indio_dev,
				struct device *dev)
{
	struct altera_adc *adc = iio_priv(indio_dev);
	struct device_node *np = dev->of_node;
	u32 value;
	int ret, i, j;
	char str[50];

	ret = of_property_read_u32(np, "altr,adc-mode", &value);
	if (ret < 0)
		return ret;
	if (value == 0 || value > MAX_ADC) {
		dev_err(dev, "Invalid ADC mode value");
		return -EINVAL;
	}
	adc->mode = value;

	ret = of_property_read_u32(np, "altr,adc-slot-count", &value);
	if (ret < 0)
		return ret;
	if (value == 0 || value > MAX_SLOT) {
		dev_err(dev, "Invalid ADC slot count value");
		return -EINVAL;
	}
	adc->slot_count = value;

	ret = of_property_read_u32(np, "altr,adc-number", &value);
	if (ret < 0)
		return ret;
	if (value == 0 || value > MAX_ADC) {
		dev_err(dev, "Invalid ADC number value");
		return -EINVAL;
	}
	adc->adc_number = value;

	/* Device tree lookup for channels for each memory slots */
	for (j = 0; j < adc->mode; j++) {
		for (i = 0; i < adc->slot_count; i++) {
			str[0] = '\0';

			snprintf(str, sizeof(str),
				 "altr,adc%d-slot-sequence-%d",
				 (j + 1), (i + 1));

			ret = of_property_read_u32(np, str, &value);
			if (ret < 0)
				return ret;
			if (value > MAX_CHANNEL) {
				dev_err(dev, "Invalid ADC channel value");
				return -EINVAL;
			}
			adc->slot_sequence[j][i] = value;
		}
	}

	return 0;
}

static int alt_modular_adc_probe(struct platform_device *pdev)
{
	struct altera_adc *adc;
	struct device_node *np = pdev->dev.of_node;
	struct iio_dev *indio_dev;
	struct resource *mem;
	int ret;

	if (!np)
		return -ENODEV;

	indio_dev = iio_device_alloc(sizeof(struct altera_adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}

	adc = iio_priv(indio_dev);

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
		"sequencer_csr");
	if (mem == NULL) {
		dev_err(&pdev->dev, "failed to get resource sequencer_csr\n");
		ret = -ENXIO;
		goto err_iio;
	}
	adc->seq_regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(adc->seq_regs)) {
		ret = PTR_ERR(adc->seq_regs);
		goto err_iio;
	}

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
		"sample_store_csr");
	if (mem == NULL) {
		dev_err(&pdev->dev, "failed to get resource sample_store_csr\n");
		ret = -ENXIO;
		goto err_iio;
	}
	adc->sample_regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(adc->sample_regs)) {
		ret = PTR_ERR(adc->sample_regs);
		goto err_iio;
	}

	ret = alt_modular_adc_parse_dt(indio_dev, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to parse device tree\n");
		goto err_iio;
	}

	ret = alt_modular_adc_channel_init(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to initialize ADC channels\n");
		goto err_iio;
	}

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = adc->slot_count;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_iio;

	/* Disable Interrupt */
	writel_relaxed(0, (adc->sample_regs + ADC_IER_REG));

	/* Start Continuous Sampling */
	writel_relaxed((ADC_RUN_MSK), (adc->seq_regs + ADC_CMD_REG));

	return 0;

err_iio:
	/* Kfree sysfs memory */
	alt_modular_adc_kfree_names(indio_dev);

	/* free indio_dev from driver */
	iio_device_free(indio_dev);
	return ret;
}

static int alt_modular_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct altera_adc *adc = iio_priv(indio_dev);

	/* Kfree sysfs memory */
	alt_modular_adc_kfree_names(indio_dev);

	/* Stop ADC */
	writel((ADC_STOP_MSK), (adc->seq_regs + ADC_CMD_REG));

	/* Unregister ADC */
	iio_device_unregister(indio_dev);

	return 0;
}

static struct platform_driver altr_modular_adc_driver = {
	.probe		= alt_modular_adc_probe,
	.remove		= alt_modular_adc_remove,
	.driver		= {
		.name	= "alt-modular-adc",
		.owner	= THIS_MODULE,
		.of_match_table = alt_modular_adc_match,
	},
};

module_platform_driver(altr_modular_adc_driver);

MODULE_DESCRIPTION("Altera Modular ADC Driver");
MODULE_AUTHOR("Chee Nouk Phoon <cnphoon@altera.com>");
MODULE_LICENSE("GPL v2");
