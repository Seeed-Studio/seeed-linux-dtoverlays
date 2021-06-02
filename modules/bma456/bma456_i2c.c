// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456_i2c.c - Linux kernel driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 *
 * BMA456: 7-bit I2C slave address 0x19
 */

#include "bma456.h"


static BMA4_INTF_RET_TYPE bma_i2c_read(uint8_t reg_addr, 
			uint8_t *read_data, uint32_t len, void *intf_ptr)
{
	struct bma456_data *data = (struct bma456_data *)intf_ptr;

	mutex_lock(&data->mutex);
	i2c_smbus_read_i2c_block_data(data->client, reg_addr, len, read_data);
	mutex_unlock(&data->mutex);
	return 0;
}

static BMA4_INTF_RET_TYPE bma_i2c_write(uint8_t reg_addr, 
			const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
	struct bma456_data *data = (struct bma456_data *)intf_ptr;

	mutex_lock(&data->mutex);
	i2c_smbus_write_i2c_block_data(data->client, reg_addr, len, write_data);
	mutex_unlock(&data->mutex);
	return 0;
}

static void bma_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t ms = period / 1000;
	uint32_t us = period % 1000;

	if (ms)
		msleep(ms);
	if (us)
		udelay(us);
}


/* probe */
static int bma456_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bma456_data *data;
	int ret = 0;

	DBG_FUNC("");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check I2C_FUNC_I2C\n");
		return -EOPNOTSUPP;
	}

	data = (struct bma456_data *)devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	i2c_set_clientdata(client, data);
	data->client = client;
	mutex_init(&data->mutex);

	/* i2c */
	data->bma.intf = BMA4_I2C_INTF;
	data->bma.intf_ptr = (void *)data;
	data->bma.bus_read = bma_i2c_read;
	data->bma.bus_write = bma_i2c_write;
	data->bma.delay_us = bma_delay_us;
	data->bma.read_write_len = 32;

	ret = bma456_probe(&client->dev, data);
	if (BMA4_OK != ret) {
		dev_err(&client->dev, "bma456_probe:%d\n", ret);
		return -ENODEV;
	}

	ret = bma456_iio_init(&client->dev, data, id->name);
	if (ret < 0) {
		dev_err(&client->dev, "bma456_iio_init:%d\n", ret);
		return -ENODEV;
	}

	ret = bma456_input_init(&client->dev, BUS_I2C, data, id->name);
	if (ret < 0) {
		dev_err(&client->dev, "bma456_input_init:%d\n", ret);
		return -ENODEV;
	}

	ret = bma456_irq_request(&client->dev, client->irq, dev_name(&client->dev), data);
	if (ret < 0) {
		dev_err(&client->dev, "bma456_irq_request:%d\n", ret);
		return -ENODEV;
	}

	return 0;
}

static int bma456_i2c_remove(struct i2c_client *client)
{
	struct bma456_data *data = (struct bma456_data *)i2c_get_clientdata(client);

	DBG_FUNC();
	bma456_irq_free(data, client->irq);
	bma456_input_deinit(data);
	bma456_iio_deinit(data);
	bma456_remove(data);

	return 0;
}

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
//		.pm	= BMA456_PM_OPS,
		.of_match_table = bma456_of_match,
	},
	.probe		= bma456_i2c_probe,
	.remove		= bma456_i2c_remove,
	.id_table	= bma456_ids,
};
module_i2c_driver(bma456_driver);


MODULE_AUTHOR("Zhangqun Ming <north_sea@qq.com>");
MODULE_AUTHOR("Seeed, Inc.");
MODULE_DESCRIPTION("Bosch BMA456 triaxial acceleration sensor");
MODULE_LICENSE("GPL v2");
