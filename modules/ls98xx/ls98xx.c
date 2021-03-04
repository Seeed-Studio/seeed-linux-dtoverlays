/*
 * ls9800.c - Support for LEVELEK ls9800 ambient light and proximity sensor
 *
 * Copyright 2021 Zhangqun Ming <north_sea@qq.com>
 *
 * 7-bit I2C slave address 0x54
*/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/acpi.h>

#include <linux/iio/iio.h>
#include <linux/iio/events.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>

#include "ls98xx.h"


#define LS98xx_DRV_NAME "ls98xx"


struct ls98xx_gain {
	int scale;
	int uscale;
};

struct ls98xx_chip_info {
	u8 id;
	int als_gain_tbl_size;
	const struct ls98xx_gain *als_gains;
	int ps_gain_tbl_size;
	const struct ls98xx_gain *ps_gains;

/*	u8 als_mode_active;
	u8 als_gain_mask;
	u8 als_gain_shift; 
*/
	const int num_channels;
	struct iio_chan_spec const *channels;
	const struct iio_info *info;
	const struct iio_info *info_no_irq;
};

struct ls98xx_data {
	struct i2c_client *client;
	struct mutex lock_als, lock_ps;
	struct ls98xx_chip_info *chip_info;

	struct regmap *regmap;
};


// iio
static int ls9800_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	return 0;
}

static int ls9800_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	return 0;
}

static ssize_t ls9800_show_proximity_scale_avail(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct ls98xx_data *data = iio_priv(dev_to_iio_dev(dev));
	struct ls98xx_chip_info *info = data->chip_info;
	ssize_t len = 0;
	int i;

	for (i=0; i < info->ps_gain_tbl_size; i++) {
		len += scnprintf(buf+len, PAGE_SIZE - len, "%d.%06d ", 
					info->ps_gains[i].scale, info->ps_gains[i].uscale);
	}
	buf[len-1] = '\n';
	buf[len++] = '\0';
	return len;
}

static ssize_t ls9800_show_intensity_scale_avail(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct ls98xx_data *data = iio_priv(dev_to_iio_dev(dev));
	struct ls98xx_chip_info *info = data->chip_info;
	ssize_t len = 0;
	int i;

	for (i=0; i < info->als_gain_tbl_size; i++) {
		len += scnprintf(buf+len, PAGE_SIZE - len, "%d.%06d ", 
					info->als_gains[i].scale, info->als_gains[i].uscale);
	}

	buf[len-1] = '\n';
	buf[len++] = '\0';
	return len;
}

static IIO_CONST_ATTR_INT_TIME_AVAIL("0.05 0.1 0.2 0.4");
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("20 10 5 2 1 0.5");

static IIO_DEVICE_ATTR(in_proximity_scale_available, S_IRUGO,
		       ls9800_show_proximity_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_intensity_scale_available, S_IRUGO,
		       ls9800_show_intensity_scale_avail, NULL, 0);

static struct attribute *ls9800_attributes[] = {
	&iio_dev_attr_in_proximity_scale_available.dev_attr.attr,
	&iio_dev_attr_in_intensity_scale_available.dev_attr.attr,
	&iio_const_attr_integration_time_available.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ls9800_attribute_group = {
	.attrs = ls9800_attributes,
};

static const struct iio_info ls9800_info = {
	.read_raw = ls9800_read_raw,
	.write_raw = ls9800_write_raw,
	.attrs = &ls9800_attribute_group,
/*	.read_event_value	= &ls9800_read_event,
	.write_event_value	= &ls9800_write_event,
	.read_event_config	= &ls9800_read_event_config,
	.write_event_config	= &ls9800_write_event_config,
*/
};

static const struct iio_info ls9800_info_no_irq = {
	.read_raw = ls9800_read_raw,
	.write_raw = ls9800_write_raw,
	.attrs = &ls9800_attribute_group,
};

static const struct iio_event_spec ls98xx_ps_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE) |
				 BIT(IIO_EV_INFO_PERIOD),
	},
};

static const struct iio_chan_spec ls9800_channels[] = {
/*	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_index = -1,
	},*/

	{
		.type = IIO_PROXIMITY,
		.address = LS98xx_PDATAL_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 2,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
		.num_event_specs = ARRAY_SIZE(ls98xx_ps_event_spec),
		.event_spec = ls98xx_ps_event_spec,
	},
	IIO_CHAN_SOFT_TIMESTAMP(3),
};


// chip info
static const struct ls98xx_gain ls9800_als_gain_tbl[] = {
	{ 1, 0 },		// 1x
	{ 0, 250000 },	// 4x
	{ 0, 62500 },	// 16x
	{ 0, 7812 },	// 128x
	{ 0, 3906 },	// 256x
};
static const struct ls98xx_gain ls9800_ps_gain_tbl[] = {
	{ 1, 0 },		// 1x
	{ 0, 500000 },	// 2x
	{ 0, 250000 },	// 4x
	{ 0, 125000 },	// 8x
	{ 0, 62500 },	// 16x
	{ 0, 31250 },	// 32x
};

static struct ls98xx_chip_info ls98xx_chip_info_tbl[] = {
	{// ls9800
		.id = 0x00,
		.als_gain_tbl_size = ARRAY_SIZE(ls9800_als_gain_tbl),
		.als_gains = ls9800_als_gain_tbl,
		.ps_gain_tbl_size = ARRAY_SIZE(ls9800_ps_gain_tbl),
		.ps_gains = ls9800_ps_gain_tbl,

		.num_channels = ARRAY_SIZE(ls9800_channels),
		.channels = ls9800_channels,

		.info = &ls9800_info,
		.info_no_irq = &ls9800_info_no_irq,
	},
};


// regmap
static bool ls98xx_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case LS98xx_STAT_REG:
	case LS98xx_LDATA1L_REG:
	case LS98xx_LDATA1H_REG:
	case LS98xx_LDATA2L_REG:
	case LS98xx_LDATA2H_REG:
	case LS98xx_LDATA3L_REG:
	case LS98xx_LDATA3H_REG:
	case LS98xx_PDATAL_REG:
	case LS98xx_PDATAH_REG:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config ls98xx_regmap_config = {
	.name = "ls98xx_regmap",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x20,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = ls98xx_is_volatile_reg,
};

static irqreturn_t ls98xx_trigger_handler(int irq, void *p)
{
	return IRQ_HANDLED;
}

static irqreturn_t ls98xx_interrupt_handler(int irq, void *private)
{
	return IRQ_HANDLED;
}

static int ls98xx_powerdown(struct ls98xx_data *data)
{
/*	return ltr501_write_contr(data, data->als_contr &
				  ~data->chip_info->als_mode_active,
				  data->ps_contr & ~LTR501_CONTR_ACTIVE);*/
	return 0;
}

static int ls98xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct regmap *regmap;
	struct ls98xx_data *data;
	struct iio_dev *indio_dev;
	const char *name = "ls98xx";
	int ret;

	DBG_FUNC();

	regmap = devm_regmap_init_i2c(client, &ls98xx_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Regmap initialization failed.\n");
		return PTR_ERR(regmap);
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	data->regmap = regmap;
	mutex_init(&data->lock_als);
	mutex_init(&data->lock_ps);

	if (id) {
		name = id->name;
	}

	data->chip_info = &ls98xx_chip_info_tbl[0];
	indio_dev->info = data->chip_info->info;
	indio_dev->num_channels = data->chip_info->num_channels;
	indio_dev->channels = data->chip_info->channels;
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (client->irq > 0) {

	}
	else {
		indio_dev->info = data->chip_info->info_no_irq;
	}

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
				ls98xx_trigger_handler, NULL);
	if (ret)
		goto powerdown_on_error;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_unreg_buffer;

	return 0;

error_unreg_buffer:
	iio_triggered_buffer_cleanup(indio_dev);

powerdown_on_error:
	ls98xx_powerdown(data);
	return ret;
}

static int ls98xx_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	DBG_FUNC();

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	ls98xx_powerdown(iio_priv(indio_dev));

	return 0;
}

static const struct of_device_id ls98xx_of_ids[] = {
	{
		.compatible = "ls98xx,levelek",
		.data = (const void*)NULL,
	},
	{ } /* sentinel */
};
MODULE_DEVICE_TABLE(of, ls98xx_of_ids);

static struct i2c_driver ls98xx_driver = {
	.driver = {
		.name   = LS98xx_DRV_NAME,
		.of_match_table = ls98xx_of_ids,
		//.pm	= &LS98xx_pm_ops,
	},
	.probe  = ls98xx_probe,
	.remove	= ls98xx_remove,
};

module_i2c_driver(ls98xx_driver);

MODULE_AUTHOR("Zhangqun Ming <north_sea@qq.com>");
MODULE_DESCRIPTION("LEVELEK ls98xx ambient light and proximity sensor");
MODULE_LICENSE("GPL");

