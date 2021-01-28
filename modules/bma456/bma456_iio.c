// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456_iio.c - Linux kernel driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 */

#include "bma456.h"


/* data */
#define BMA456_ACC_CHANNEL(_axis, _bits) {	\
	.type = IIO_ACCEL,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,			\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)			\
		/*| BIT(IIO_CHAN_INFO_OFFSET)*/,						\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE)	\
		| BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),	\
	.scan_index = AXIS_##_axis,				\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = _bits,					\
		.storagebits = 16,					\
		.shift = 16 - _bits,				\
	},										\
	.ext_info = bma456_ext_info,			\
}

#define BMA456_TEMP_CHANNEL {				\
	.type = IIO_TEMP,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)				\
		| BIT(IIO_CHAN_INFO_SCALE) /*| BIT(IIO_CHAN_INFO_OFFSET)*/,	\
	.scan_index = TEMP,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 8,						\
		.storagebits = 16,					\
	},										\
}

/*
#define BMA456_ACTIVITY_CHANNEL(_chan) {	\
	.type = IIO_ACTIVITY,					\
	.modified = 1,							\
	.channel2 = _chan,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),         \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_ENABLE),      \
}*/


const int scale_table[] = { // in ug
	598, 1197, 2394, 4789
};
const int bw_table[] = { // in us
	1280000, 640000, 320000, 160000, 80000,
    40000,   20000,  10000,  5000,   2500,
    1250,    625,
};

/* power_mode */
static int bma456_get_power_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan)
{
	DBG_FUNC("");
	return 1;
}

static int bma456_set_power_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, unsigned int mode)
{
	DBG_FUNC("%d", mode);
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

/*	BMA456_ACTIVITY_CHANNEL(IIO_MOD_STILL),
	BMA456_ACTIVITY_CHANNEL(IIO_MOD_WALKING),
	BMA456_ACTIVITY_CHANNEL(IIO_MOD_RUNNING),*/
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

/* Activity */
static ssize_t in_activity_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);
	uint8_t activity_output;
	uint8_t rslt = 0;

	rslt = bma456_activity_output(&activity_output, &data->bma);
	bma4_error_codes_print_result("bma456_activity_output status", rslt);
	if (BMA4_OK != rslt) {
		dev_err(dev, "in_activity_show:%d\n", rslt);
		return rslt;
	}
	return sprintf(buf, "%d\n", activity_output);
}

/* SINGLE-TAP */
static ssize_t single_tap_enable_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", !!(data->int_map & BMA456_SINGLE_TAP_INT));
}
static ssize_t single_tap_enable_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);
	uint8_t val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	if (val) {
		data->int_map |= BMA456_SINGLE_TAP_INT;
		bma456_feature_enable(BMA456_SINGLE_TAP, BMA4_ENABLE, &data->bma);
	}
	else {
		data->int_map &= ~BMA456_SINGLE_TAP_INT;
		bma456_feature_enable(BMA456_SINGLE_TAP, BMA4_DISABLE, &data->bma);
	}

	return len;
}
static ssize_t single_tap_sensitivity_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", data->single_tap_sens);
}
static ssize_t single_tap_sensitivity_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);
	uint8_t val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;
	if (val >= 8) {
		dev_err(dev, "single_tap_sensitivity:%d(must<8)\n", val);
		return -EINVAL;
	}
	data->single_tap_sens = val;

	return len;
}

/* DOUBLE-TAP */
static ssize_t double_tap_enable_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", !!(data->int_map & BMA456_DOUBLE_TAP_INT));
}
static ssize_t double_tap_enable_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);
	uint8_t val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	if (val) {
		data->int_map |= BMA456_DOUBLE_TAP_INT;
		bma456_feature_enable(BMA456_DOUBLE_TAP, BMA4_ENABLE, &data->bma);
	}
	else {
		data->int_map &= ~BMA456_DOUBLE_TAP_INT;
		bma456_feature_enable(BMA456_DOUBLE_TAP, BMA4_DISABLE, &data->bma);
	}

	return len;
}
static ssize_t double_tap_sensitivity_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", data->double_tap_sens);
}
static ssize_t double_tap_sensitivity_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);
	uint8_t val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;
	if (val >= 8) {
		dev_err(dev, "double_tap_sensitivity:%d(must<8)\n", val);
		return -EINVAL;
	}
	data->double_tap_sens = val;

	return len;
}

/* ANY_MOT */
static ssize_t any_motion_enable_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", !!(data->int_map & BMA456_ANY_MOT_INT));
}
static ssize_t any_motion_enable_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);
	uint8_t val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	if (val) {
		data->int_map |= BMA456_ANY_MOT_INT;
		bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_ANY_MOT_INT, BMA4_ENABLE, &data->bma);
	}
	else {
		data->int_map &= ~BMA456_ANY_MOT_INT;
		bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_ANY_MOT_INT, BMA4_DISABLE, &data->bma);
	}

	return len;
}
static ssize_t any_motion_sensitivity_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", data->any_mot.threshold);
}
static ssize_t any_motion_sensitivity_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);
	uint16_t val;
	int ret;

	ret = kstrtou16(buf, 0, &val);
	if (ret)
		return ret;
	data->any_mot.threshold = val;

	return len;
}
static ssize_t any_motion_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", !!(data->int_status & BMA456_ANY_MOT_INT));
}

/* NO_MOT */
static ssize_t no_motion_enable_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", !!(data->int_map & BMA456_NO_MOT_INT));
}
static ssize_t no_motion_enable_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);
	uint8_t val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	if (val) {
		data->int_map |= BMA456_NO_MOT_INT;
		bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_NO_MOT_INT, BMA4_ENABLE, &data->bma);
	}
	else {
		data->int_map &= ~BMA456_NO_MOT_INT;
		bma456_map_interrupt(BMA4_INTR1_MAP, BMA456_NO_MOT_INT, BMA4_DISABLE, &data->bma);
	}

	return len;
}
static ssize_t no_motion_sensitivity_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", data->no_mot.threshold);
}
static ssize_t no_motion_sensitivity_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);
	uint16_t val;
	int ret;

	ret = kstrtou16(buf, 0, &val);
	if (ret)
		return ret;
	data->no_mot.threshold = val;

	return len;
}
static ssize_t no_motion_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma456_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", !!(data->int_status & BMA456_NO_MOT_INT));
}

static IIO_DEVICE_ATTR_RO(in_accel_filter_low_pass_3db_frequency_available, 0);
static IIO_DEVICE_ATTR_RO(in_accel_scale_available, 0);
static IIO_DEVICE_ATTR_RO(in_activity, 0);
static IIO_DEVICE_ATTR_RW(single_tap_enable, 0);
static IIO_DEVICE_ATTR_RW(single_tap_sensitivity, 0);
static IIO_DEVICE_ATTR_RW(double_tap_enable, 0);
static IIO_DEVICE_ATTR_RW(double_tap_sensitivity, 0);
static IIO_DEVICE_ATTR_RW(any_motion_enable, 0);
static IIO_DEVICE_ATTR_RW(any_motion_sensitivity, 0);
static IIO_DEVICE_ATTR_RO(any_motion, 0);
static IIO_DEVICE_ATTR_RW(no_motion_enable, 0);
static IIO_DEVICE_ATTR_RW(no_motion_sensitivity, 0);
static IIO_DEVICE_ATTR_RO(no_motion, 0);
static struct attribute *bma456_attributes[] = {
	&iio_dev_attr_in_accel_filter_low_pass_3db_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	/* In addition */
	&iio_dev_attr_in_activity.dev_attr.attr,
	&iio_dev_attr_single_tap_enable.dev_attr.attr,
	&iio_dev_attr_single_tap_sensitivity.dev_attr.attr,
	&iio_dev_attr_double_tap_enable.dev_attr.attr,
	&iio_dev_attr_double_tap_sensitivity.dev_attr.attr,
	&iio_dev_attr_any_motion_enable.dev_attr.attr,
	&iio_dev_attr_any_motion_sensitivity.dev_attr.attr,
	&iio_dev_attr_any_motion.dev_attr.attr,
	&iio_dev_attr_no_motion_enable.dev_attr.attr,
	&iio_dev_attr_no_motion_sensitivity.dev_attr.attr,
	&iio_dev_attr_no_motion.dev_attr.attr,
	NULL,
};
static const struct attribute_group bma456_attrs_group = {
	.attrs = bma456_attributes,
};

static int bma456_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
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
				bma4_error_codes_print_result("bma4_read_regs", rslt);
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
			*val = !!(data->int_map & BMA456_STEP_CNTR_INT);
		}
		else {
			// to do:
			*val = 1;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		if (IIO_STEPS == chan->type) {
			uint32_t step_out = 0;
			rslt = bma456_step_counter_output(&step_out, &data->bma);
			bma4_error_codes_print_result("bma456_step_counter_output status", rslt);
			if (BMA4_OK != rslt)
				return -EINVAL;
			*val = step_out;
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
			dev_err(indio_dev->dev.parent, "bma4_set_accel_config\n");
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
			dev_err(indio_dev->dev.parent, "bma4_set_accel_config\n");
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
			if (val) {
				if (data->int_map & BMA456_STEP_CNTR_INT)
					return 0;
				DBG_FUNC("BMA456_STEP_CNTR enable");
				data->int_map |= BMA456_STEP_CNTR_INT;
				bma456_reset_step_counter(&data->bma);
				bma456_feature_enable(BMA456_STEP_CNTR, BMA4_ENABLE, &data->bma);
			}
			else {
				if (!(data->int_map & BMA456_STEP_CNTR_INT))
					return 0;
				DBG_FUNC("BMA456_STEP_CNTR disable");
				data->int_map &= ~BMA456_STEP_CNTR_INT;
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


int bma456_iio_init(struct device *dev, struct bma456_data *data, const char *name)
{
	int ret = 0;
	struct iio_dev *indio_dev;

	ret = iio_read_mount_matrix(dev, "mount-matrix", &data->orientation);
	if (ret < 0) {
		dev_err(dev, "iio_read_mount_matrix:%d\n", ret);
		return ret;
	}

	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev)
		return -ENOMEM;
	data->indio_dev = indio_dev;
	indio_dev->priv = (void*)data;
	indio_dev->dev.parent = dev;
	indio_dev->channels = bma456_channels;
	indio_dev->num_channels = ARRAY_SIZE(bma456_channels);
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bma456_info;
	ret = devm_iio_device_register(dev, indio_dev);
	if (ret < 0) {
		dev_err(dev, "unable to register iio device:%d\n", ret);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(bma456_iio_init);


int bma456_iio_deinit(struct bma456_data *data)
{
//	iio_device_unregister(data->indio_dev);
	return 0;
}
EXPORT_SYMBOL_GPL(bma456_iio_deinit);
