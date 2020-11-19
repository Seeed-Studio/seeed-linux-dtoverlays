// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456.h - IIO driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 *
 * SPI is not supported by driver
 * BMA456: 7-bit I2C slave address 0x19
 */

#ifndef __BMA456_H__
#define __BMA456_H__

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

//#include "api/bma4_common.h"
#include "api/bma456.h"

//#define DBG_PRINT(format, x...)	printk(KERN_INFO "[TST]%d:%s " format, __LINE__, __func__, ##x)
#define DBG_FUNC(format, x...)		printk(KERN_INFO "[BMA]%s:" format"\n", __func__, ##x)
#define DBG_PRINT(format, x...)		printk(KERN_INFO "[BMA]" format"\n", ##x)

enum bma456_chan {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
	TEMP
};

struct bma456_data {
	struct i2c_client *client;
	struct iio_trigger *trig;
	struct iio_mount_matrix orientation;
	struct mutex mutex;
	//u8 buff[16]; /* 3x 16-bit + 8-bit + padding + timestamp */

	// feature
	uint8_t step_enable;
	uint32_t step_count;

	// workqueue
	struct delayed_work work_irq;

	// api data struct
	struct bma4_dev bma;
	struct bma4_accel_config accel;
};

/* data */
#define BMA456_ACC_CHANNEL(_axis, _bits) {	\
	.type = IIO_ACCEL,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,			\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)			\
		| BIT(IIO_CHAN_INFO_OFFSET),						\
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
		| BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),	\
	.scan_index = TEMP,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 8,						\
		.storagebits = 16,					\
	},										\
}

/*#define BMA456_ACTIVITY_CHANNEL(_chan) {	\
	.type = IIO_ACTIVITY,					\
	.modified = 1,							\
	.channel2 = _chan,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_ENABLE),
}*/

#endif /*End of header guard macro */
