// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456.c - IIO driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 *
 * SPI is not supported by driver
 * BMA456: 7-bit I2C slave address 0x19
 */

#include "bma456.h"


#define GRAVITY_EARTH      9807	//(9.80665f)

struct bma456_data *bma456_data_ptr = NULL;

static const int scale_table[]= {598, 1197, 2394, 4789};
static const int bw_table[] = { // in mHz
	781, 1563, 3125, 6250, 12500, 
	25000, 50000, 100000, 200000, 
	400000, 800000, 1600000
};

/* power_mode */
static uint8_t bma456_chip_enable(struct bma456_data *data, bool enable)
{
	uint16_t rslt = 0;

	return rslt;
}

static int bma456_get_power_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan)
{
	DBG_FUNC("bma456_get_power_mode()");
	return 1;
}

static int bma456_set_power_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, unsigned int mode)
{
	DBG_FUNC("bma456_set_power_mode():%d", mode);
	return 0;
}

static const struct iio_mount_matrix *
	bma456_accel_get_mount_matrix(const struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan)
{
	struct bma456_data *data = iio_priv(indio_dev);
	return &data->orientation;
}

static const char * const bma456_power_modes[] = { 
	"low_noise", "low_power" 
};
static const struct iio_enum bma456_power_mode_enum = {
	.items = bma456_power_modes,
	.num_items = ARRAY_SIZE(bma456_power_modes),
	.get = bma456_get_power_mode,
	.set = bma456_set_power_mode,
};
static const struct iio_chan_spec_ext_info bma456_ext_info[] = {
	IIO_ENUM("power_mode", true, &bma456_power_mode_enum),
	IIO_ENUM_AVAILABLE("power_mode", &bma456_power_mode_enum),
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, bma456_accel_get_mount_matrix),
	{ }
};


/* iio_chan */
static const struct iio_chan_spec bma456_channels[] = {
	BMA456_ACC_CHANNEL(X, 16),
	BMA456_ACC_CHANNEL(Y, 16),
	BMA456_ACC_CHANNEL(Z, 16),
	BMA456_TEMP_CHANNEL,
	IIO_CHAN_SOFT_TIMESTAMP(4),

	{// STEPS
		.type = IIO_STEPS,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				BIT(IIO_CHAN_INFO_ENABLE),
	},
};


/* iio_info */
static ssize_t in_accel_filter_low_pass_3db_frequency_available_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int i = 0;
	size_t len = 0;

	for (i=0; i<ARRAY_SIZE(bw_table); i++) {
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", bw_table[i]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static ssize_t in_accel_scale_available_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i = 0;
	size_t len = 0;

	for (i=0; i<ARRAY_SIZE(scale_table); i++) {
		len += scnprintf(buf+len, PAGE_SIZE-len, "0.%06d ", scale_table[i]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static IIO_DEVICE_ATTR_RO(in_accel_filter_low_pass_3db_frequency_available, 0);
static IIO_DEVICE_ATTR_RO(in_accel_scale_available, 0);
static struct attribute *bma456_attributes[] = {
	&iio_dev_attr_in_accel_filter_low_pass_3db_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	NULL,
};
static const struct attribute_group bma456_attrs_group = {
	.attrs = bma456_attributes,
};

static int bma456_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val, int *val2,
		long mask)
{
	struct bma456_data *data = iio_priv(indio_dev);
	uint16_t rslt;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		DBG_FUNC("IIO_CHAN_INFO_RAW idx=%d", chan->scan_index);
		if (chan->scan_index == TEMP) {
			int32_t temp;
			rslt = bma4_get_temperature(&temp, BMA4_DEG, &data->bma);
			*val = temp;
		}
		else {
			int32_t val32 = 0;
			uint8_t value[2] = { 0 };
			int axis = chan->scan_index;
			if ((AXIS_X <= axis) && (axis <= AXIS_Z)) {
				axis -= AXIS_X;
				rslt = bma4_read_regs(BMA4_DATA_8_ADDR + axis*2, value, 2, &data->bma);
				if (rslt != BMA4_OK) {
					*val = 0;
					return -EINVAL;
				}

				val32 = value[1];
				val32 <<= 8;
				val32 += value[0];
				*val = sign_extend32(val32 >> chan->scan_type.shift, chan->scan_type.realbits - 1);
			}
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		DBG_FUNC("IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY");
		*val = bw_table[data->accel.odr - BMA4_OUTPUT_DATA_RATE_0_78HZ];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		DBG_FUNC("IIO_CHAN_INFO_SCALE type=%d", chan->type);
		switch (chan->type) {
		case IIO_ACCEL:
			*val = 0;
			*val2 = scale_table[data->accel.range - BMA4_ACCEL_RANGE_2G];
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:
			*val = 0;
			*val2 = BMA4_SCALE_TEMP;
			return IIO_VAL_INT_PLUS_MICRO;
		break;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		DBG_FUNC("IIO_CHAN_INFO_OFFSET");
		if (IIO_TEMP == chan->type) {
			*val = BMA4_OFFSET_TEMP; /* 0 LSB @ 23 degree C */
		}
		else {
			int8_t value[1] = { 0 };
			int axis = chan->scan_index;
			if ((AXIS_X <= axis) && (axis <= AXIS_Z)) {
				axis -= AXIS_X;
				rslt = bma4_read_regs(BMA4_OFFSET_0_ADDR + axis, value, 1, &data->bma);
				if (rslt != BMA4_OK) {
					*val = 0;
					return -EINVAL;
				}

				*val = value[0];
			}
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_ENABLE:
		DBG_FUNC("IIO_CHAN_INFO_ENABLE");
		if (IIO_STEPS == chan->type) {
			*val = data->step_enable;
		}
		else {
			// to do:
			*val = 1;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		if (IIO_STEPS == chan->type) {
			*val = data->step_count;
		}
		return IIO_VAL_INT;

	default:
		DBG_FUNC("default mask=%d\n", (int)mask);
		return -EINVAL;
	}

	return -EINVAL;
}

static int bma456_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct bma456_data *data = iio_priv(indio_dev);
	int32_t i = 0;
	uint16_t rslt;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		DBG_FUNC("IIO_CHAN_INFO_SCALE:%d, %d", val, val2);
		for (i=0; i<ARRAY_SIZE(scale_table); i++) {
			if (val2 == scale_table[i])
				break;
		}
		if (i >= ARRAY_SIZE(scale_table))
			return -EINVAL;
		data->accel.range = BMA4_ACCEL_RANGE_2G + i;
		rslt = bma4_set_accel_config(&data->accel, &data->bma);
		if (BMA4_OK != rslt) {
			dev_err(&data->client->dev, "bma4_set_accel_config\n");
			return -EINVAL;
		}
		return rslt;

	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		DBG_FUNC("IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:%d, %d", val, val2);
		for (i=0; i<ARRAY_SIZE(bw_table); i++) {
			if (val == bw_table[i])
				break;
		}
		if (i >= ARRAY_SIZE(bw_table))
			return -EINVAL;
		data->accel.odr = BMA4_OUTPUT_DATA_RATE_0_78HZ + i;
		rslt = bma4_set_accel_config(&data->accel, &data->bma);
		if (BMA4_OK != rslt) {
			dev_err(&data->client->dev, "bma4_set_accel_config\n");
			return -EINVAL;
		}
		return rslt;

	case IIO_CHAN_INFO_OFFSET:
		DBG_FUNC("IIO_CHAN_INFO_OFFSET:%d, %d", chan->type, chan->scan_index);
		if (IIO_ACCEL == chan->type) {
			int8_t value[1] = { 0 };
			int axis = chan->scan_index;
			if ((AXIS_X <= axis) && (axis <= AXIS_Z)) {
				axis -= AXIS_X;
				value[0] = (int8_t)val;
				rslt = bma4_write_regs(BMA4_OFFSET_0_ADDR + axis, value, 1, &data->bma);
				if (rslt != BMA4_OK) {
					return -EINVAL;
				}
				return rslt;
			}
		}
		return -EINVAL;

	case IIO_CHAN_INFO_ENABLE:
		DBG_FUNC("IIO_CHAN_INFO_ENABLE");
		if (IIO_STEPS == chan->type) {
			int enable = !!(val);
			if (enable == data->step_enable)
				return 0;
			data->step_enable = enable;
			if (data->step_enable) {
				data->step_count = 0;
				bma456_reset_step_counter(&data->bma);
				bma456_feature_enable(BMA456_STEP_CNTR, BMA4_ENABLE, &data->bma);
			}
			else {
				bma456_feature_enable(BMA456_STEP_CNTR, BMA4_DISABLE, &data->bma);
			}
		}
		return 0;

	default:
		DBG_FUNC("mask=%d, %d, %d\n", (int)mask, val, val2);
		return -EINVAL;
	}
}

static const struct iio_info bma456_info = {
	.attrs			= &bma456_attrs_group,
	.read_raw		= bma456_read_raw,
	.write_raw		= bma456_write_raw,
};


static irqreturn_t bma456_irq(int irq, void *handle)
{
	struct bma456_data *data = (struct bma456_data *)handle;
	uint16_t int_status;
	uint16_t rslt = bma456_read_int_status(&int_status, &data->bma);
	if (rslt != BMA4_OK) {
		dev_err(&data->client->dev, "read in status.\n");
	}

	if (int_status & BMA456_STEP_CNTR_INT) {
		uint32_t step_count = 0;
		bma456_step_counter_output(&step_count, &data->bma);
		DBG_FUNC("STEP: 0x%04x, %d", int_status, step_count);
		data->step_count = step_count;
	}
	if (int_status & BMA456_ACTIVITY_INT) {
		uint8_t activity = 0;
		bma456_activity_output(&activity, &data->bma);
		if (BMA456_USER_STATIONARY == activity) {
			DBG_FUNC("ACTI: 0x%04x, STATIONARY<%d>", int_status, activity);
		}
		else if (BMA456_USER_WALKING == activity) {
			DBG_FUNC("ACTI: 0x%04x, WALKING<%d>", int_status, activity);
		}
		else if (BMA456_USER_RUNNING == activity) {
			DBG_FUNC("ACTI: 0x%04x, RUNNING<%d>", int_status, activity);
		}
		else if (BMA456_STATE_INVALID == activity) {
			DBG_FUNC("ACTI: 0x%04x, INVALID<%d>", int_status, activity);
		}
	}
	if (int_status & BMA456_WRIST_TILT_INT) {
		DBG_FUNC("WRIST:0x%04x", int_status);
	}
	if (int_status & BMA456_WAKEUP_INT) {
		DBG_FUNC("WAKEUP:0x%04x", int_status);
	}
	if (int_status & BMA456_ANY_NO_MOTION_INT) {
		//DBG_FUNC("MOTION:0x%04x", int_status);
	}
	if (int_status & BMA456_ERROR_INT) {
		DBG_FUNC("ERROR:0x%04x", int_status);
	}

	return IRQ_HANDLED;
}

/* work_cb_irq: scan bma456's interrupt. */
static void work_cb_irq(struct work_struct *work)
{
	struct bma456_data *data = container_of((struct delayed_work*)work, 
		struct bma456_data, work_irq);
	int msec = 1000000 / bw_table[data->accel.odr - BMA4_OUTPUT_DATA_RATE_0_78HZ];

	bma456_irq(data->client->irq, data);
	schedule_delayed_work(&data->work_irq, msecs_to_jiffies(msec));

	return;
}

/* apis */
static uint16_t bma_i2c_read(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t len)
{
	mutex_lock(&bma456_data_ptr->mutex);
	i2c_smbus_read_i2c_block_data(bma456_data_ptr->client, reg, len, data);
	mutex_unlock(&bma456_data_ptr->mutex);
	return 0;
}

static uint16_t bma_i2c_write(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t len)
{
	mutex_lock(&bma456_data_ptr->mutex);
	i2c_smbus_write_i2c_block_data(bma456_data_ptr->client, reg, len, data);
	mutex_unlock(&bma456_data_ptr->mutex);
	return 0;
}

static void bma_delay_ms(uint32_t ms)
{
	msleep(ms);	
}

/* probe */
static int bma456_probe(struct i2c_client *client, 
				const struct i2c_device_id *id)
{
	struct bma456_data *data;
	struct iio_dev *indio_dev;
	int8_t ret = 0;

	DBG_FUNC("\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check I2C_FUNC_I2C\n");
		return -EOPNOTSUPP;
	}

	/* iio_dev: create */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	bma456_data_ptr = data;
	ret = iio_read_mount_matrix(&client->dev, "mount-matrix", &data->orientation);
	if (ret < 0)
		return ret;
	mutex_init(&data->mutex);
//	memset(data->buff, 0, 16);

	/* i2c */
	data->bma.dev_addr        = BMA4_I2C_ADDR_SECONDARY;
	data->bma.interface       = BMA4_I2C_INTERFACE;
	data->bma.bus_read        = bma_i2c_read;
    data->bma.bus_write       = bma_i2c_write;
    data->bma.delay           = bma_delay_ms;
    data->bma.read_write_len  = 8;
    data->bma.resolution      = 16;
    data->bma.feature_len     = BMA456_FEATURE_SIZE;
	bma456_init(&data->bma);

	bma4_set_command_register(0xB6, &data->bma); // reset device
	msleep(500);
	bma456_write_config_file(&data->bma);

	data->accel.odr = BMA4_OUTPUT_DATA_RATE_1600HZ;
    data->accel.range = BMA4_ACCEL_RANGE_4G;
    data->accel.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    data->accel.perf_mode = BMA4_CONTINUOUS_MODE;
    bma4_set_accel_config(&data->accel, &data->bma);
	bma4_set_accel_enable(BMA4_ENABLE, &data->bma);

	/* iio_dev: register */
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = bma456_channels;
	indio_dev->num_channels = ARRAY_SIZE(bma456_channels);
	indio_dev->name = id->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bma456_info;
	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "unable to register iio device:%d\n", ret);
		goto err_chip_disable;
	}

	/* feature */
	data->step_enable = 1;
	data->step_count = 0; 
	if (data->step_enable) {
		bma456_reset_step_counter(&data->bma);
		bma456_step_detector_enable(BMA4_ENABLE, &data->bma);
	}

	/* irq */
	ret = bma456_map_interrupt(BMA4_INTR1_MAP, BMA4_DATA_RDY_INT
				| BMA456_STEP_CNTR_INT | BMA456_ACTIVITY_INT | BMA456_WRIST_TILT_INT 
				| BMA456_WAKEUP_INT | BMA456_ANY_NO_MOTION_INT
				/*| (BMA456_SINGLE_TAP_INT | BMA456_DOUBLE_TAP_INT)*/,
				BMA4_ENABLE, &data->bma);
    if (BMA4_OK != ret) {
		dev_err(&client->dev, "bma456_map_interrupt:%d\n", ret);
		goto err_chip_disable;
	}
	bma456_anymotion_enable_axis(BMA456_ALL_AXIS_EN, &data->bma);
	ret = bma456_feature_enable(BMA456_STEP_CNTR /*| BMA456_WAKEUP*/
			| BMA456_ACTIVITY /*| BMA456_WRIST_TILT | BMA456_ANY_MOTION*/
			/*| (BMA456_SINGLE_TAP | BMA456_DOUBLE_TAP)*/,
			BMA4_ENABLE, &data->bma);
	if (BMA4_OK != ret) {
		dev_err(&client->dev, "feature STEP_CNTR:%d\n", ret);
		goto err_chip_disable;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, 
					NULL,
					bma456_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev_name(&client->dev),
					data);
		if (ret) {
			dev_err(&client->dev, "irq %d busy?\n", client->irq);
			goto err_chip_disable;
		}
	}
	else {
		INIT_DELAYED_WORK(&data->work_irq, work_cb_irq);
		schedule_delayed_work(&data->work_irq, 0);
	}
	DBG_FUNC("Success:0x%x!", data->bma.chip_id);
	return 0;

err_chip_disable:
	bma456_chip_enable(data, false);

	return ret;
}

static int bma456_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bma456_data *data = iio_priv(indio_dev);

	if (!data->client->irq) {
		cancel_delayed_work_sync(&data->work_irq);
	}
	bma456_chip_enable(data, false);
	iio_device_unregister(indio_dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bma456_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bma456_data *data = iio_priv(indio_dev);

	DBG_FUNC();
	return bma456_chip_enable(data, false);
}

static int bma456_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bma456_data *data = iio_priv(indio_dev);

	DBG_FUNC();
	return bma456_chip_enable(data, true);
}

static SIMPLE_DEV_PM_OPS(bma456_pm_ops, bma456_suspend, bma456_resume);
#define BMA456_PM_OPS (&bma456_pm_ops)
#else
#define BMA456_PM_OPS NULL
#endif

static const struct i2c_device_id bma456_ids[] = {
	{ "bma456", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bma456_ids);

static const struct of_device_id bma456_of_match[] = {
	{ .compatible = "bosch,bma456", },
	{ }
};
MODULE_DEVICE_TABLE(of, bma456_of_match);

static struct i2c_driver bma456_driver = {
	.driver = {
		.name	= "bma456",
		.pm	= BMA456_PM_OPS,
		.of_match_table = bma456_of_match,
	},
	.probe		= bma456_probe,
	.remove		= bma456_remove,
	.id_table	= bma456_ids,
};
module_i2c_driver(bma456_driver);

MODULE_AUTHOR("Zhangqun Ming <north_sea@qq.com>");
MODULE_AUTHOR("Seeed, Inc.");
MODULE_DESCRIPTION("Bosch BMA456 triaxial acceleration sensor");
MODULE_LICENSE("GPL v2");
