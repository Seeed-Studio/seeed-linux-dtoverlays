// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456_input.c - Linux kernel driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 */

#include "bma456.h"


void bma456_do_tap(struct bma456_data *data)
{
	input_report_key(data->input, BTN_TOUCH, true);
	input_sync(data->input);
	input_report_key(data->input, BTN_TOUCH, false);
}

int bma456_input_init(struct device *dev, uint16_t bus_type, 
		struct bma456_data *data, const char *name)
{
	struct input_dev *input;
	int ret = 0;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	data->input = input;
	
	input->dev.parent = dev;
	input->name = name;
	input->id.bustype = bus_type;
	input->id.vendor = 0x1234;
	input->id.product = 0x8888;
	input->id.version = 0x0100;

	input_set_drvdata(input, data);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	ret = input_register_device(input);
	if (ret) {
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(bma456_input_init);


int bma456_input_deinit(struct bma456_data *data)
{
	input_unregister_device(data->input);
	return 0;
}
EXPORT_SYMBOL_GPL(bma456_input_deinit);
