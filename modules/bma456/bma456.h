// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456.h - Linux kernel driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 */

#ifndef __BMA456_H__
#define __BMA456_H__

#include <linux/module.h>
#include <linux/input.h>
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

#include "api/bma456.h"


#define BMA456_DBG      0
#if BMA456_DBG
#define DBG_FUNC(format, x...)		printk(KERN_INFO "[BMA]%d:%s " format"\n", __LINE__, __func__, ##x)
#define DBG_PRINT(format, x...)		printk(KERN_INFO "[BMA]" format"\n", ##x)
#else
#define DBG_FUNC(format, x...)
#define DBG_PRINT(format, x...)
#endif


#define GRAVITY_EARTH      9807	//(9.80665f)


enum bma456_chan {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
	TEMP
};

struct bma456_data {
	struct i2c_client *client;
    struct mutex mutex;
	struct delayed_work work_irq;

    // iio
    struct iio_dev *indio_dev;
	struct iio_mount_matrix orientation;

    // input
    struct input_dev *input;

	// features
    uint16_t int_map;
    uint16_t int_status;

    uint8_t single_tap_sens;
    uint8_t double_tap_sens;

	// api data struct
	struct bma4_dev bma;
	struct bma4_accel_config accel;
    struct bma456_any_no_mot_config any_mot;
    struct bma456_any_no_mot_config no_mot;
};


/* global use */
extern const int scale_table[];
extern const int bw_table[];

/* bma456 */
int8_t bma456_probe(struct device *dev, struct bma456_data *data);
int bma456_remove(struct bma456_data *data);

/* iio */
int bma456_iio_init(struct device *dev, struct bma456_data *data, const char *name);
int bma456_iio_deinit(struct bma456_data *data);

/* input */
void bma456_do_tap(struct bma456_data *data);
int bma456_input_init(struct device *dev, uint16_t bus_type, 
		struct bma456_data *data, const char *name);
int bma456_input_deinit(struct bma456_data *data);

/* irq */
int bma456_irq_request(struct device *dev, int irq, const char *devname, struct bma456_data *data);
int bma456_irq_free(struct bma456_data *data, int irq);

#endif /*End of header guard macro */
