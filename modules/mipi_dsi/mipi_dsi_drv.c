// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019, Amarula Solutions.
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 */
#include "mipi_dsi.h"


struct i2c_mipi_dsi *i2c_md = NULL;


static int i2c_md_read(struct i2c_client *client, u8 reg)
{
	struct i2c_msg msgs[1];
	u8 addr_buf[1] = { reg };
	u8 data_buf[1] = { 0, };
	int ret;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	usleep_range(100, 300);

	/* Read data from register */
	msgs[0].addr = client->addr;
	msgs[0].flags = I2C_M_RD;
	msgs[0].len = 1;
	msgs[0].buf = data_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	return data_buf[0];
}

static void i2c_md_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret)
		dev_err(&client->dev, "I2C write failed: %d\n", ret);
}


/* drm_panel_funcs */
int i2c_md_enable(void)
{
	i2c_md_write(i2c_md->i2c, REG_PWM, 250);
	msleep(50);
	return  0;
}

int i2c_md_disable(void)
{
	i2c_md_write(i2c_md->i2c, REG_PWM, 0);
	return 0;
}

int i2c_md_prepare(void)
{
	i2c_md_write(i2c_md->i2c, REG_POWERON, 1);
	i2c_md_write(i2c_md->i2c, REG_LCD_RST, 0);
	msleep(100);
	i2c_md_write(i2c_md->i2c, REG_LCD_RST, 1);
	msleep(200);
	return 0;
}

int i2c_md_unprepare(void)
{
	i2c_md_write(i2c_md->i2c, REG_PWM, 0);
	i2c_md_write(i2c_md->i2c, REG_LCD_RST, 0);
	i2c_md_write(i2c_md->i2c, REG_POWERON, 0);
	return 0;
}


/* mipi_dsi_device */
static int i2c_mipi_dsi_device(struct i2c_mipi_dsi *md)
{
	struct device *dev = &md->i2c->dev;
	struct device_node *endpoint, *dsi_host_node;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device_info info = {
		.type = DSI_DRIVER_NAME,
		.channel = 0,
		.node = NULL,
	};

	/* Look up the DSI host.  It needs to probe before we do. */
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint)
		return -ENODEV;
	DBG_FUNC("endpoint[0x%x]%s,%s", (int)endpoint, 
		endpoint->name, endpoint->full_name);

	dsi_host_node = of_graph_get_remote_port_parent(endpoint);
	if (!dsi_host_node)
		goto error;
	DBG_FUNC("host_node[0x%x]%s,%s", (int)dsi_host_node,
		dsi_host_node->name, dsi_host_node->full_name);

	host = of_find_mipi_dsi_host_by_node(dsi_host_node);
	of_node_put(dsi_host_node);
	if (!host) {
		of_node_put(endpoint);
		return -EPROBE_DEFER;
	}
	DBG_FUNC("host[0x%x]%s", (int)host, host->dev->init_name);

	info.node = of_graph_get_remote_port(endpoint);
	if (!info.node)
		goto error;
	DBG_FUNC("info_node[0x%x]%s,%s", (int)info.node, 
		info.node->name, info.node->full_name);
	of_node_put(endpoint);

	md->dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(md->dsi)) {
		dev_err(dev, "DSI device registration failed: %ld\n",
			PTR_ERR(md->dsi));
		return PTR_ERR(md->dsi);
	}
	DBG_FUNC("dsi[0x%x]%s", (int)md->dsi, md->dsi->name);

	return mipi_dsi_attach(md->dsi);

error:
	of_node_put(endpoint);
	return -ENODEV;
}


/* i2c */
static int i2c_md_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct i2c_mipi_dsi *md;
	int ret = 0;

	DBG_FUNC();
	md = devm_kzalloc(dev, sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;
	i2c_md = md;

	i2c_set_clientdata(i2c, md);
	md->i2c = i2c;

	ret = i2c_md_read(i2c, REG_ID);
	if (ret < 0) {
		dev_err(dev, "I2C ID read failed: %d\n", ret);
		return -ENODEV;
	}
	DBG_FUNC("READ_ID=0x%x", ret);
	if (ret != 0xc3) {
		dev_err(dev, "Unknown firmware revision: 0x%02x\n", ret);
		return -ENODEV;
	}

	/* Turn off at boot, so we can cleanly sequence powering on. */
	i2c_md_write(i2c, REG_POWERON, 0);

	ret = i2c_mipi_dsi_device(md);
	if (ret < 0) {
		dev_err(dev, "failed: i2c_mipi_dsi_device\n");
		return ret;
	}

	return 0;
}

static int i2c_md_remove(struct i2c_client *i2c)
{
	struct i2c_mipi_dsi *md = i2c_get_clientdata(i2c);

	DBG_FUNC();
	mipi_dsi_detach(md->dsi);
	mipi_dsi_device_unregister(md->dsi);
	kfree(md->dsi);

	return 0;
}

static const struct of_device_id rpi_touchscreen_of_ids[] = {
	{ .compatible = "seeed,mipi_dsi" },
	{ } /* sentinel */
};
MODULE_DEVICE_TABLE(of, rpi_touchscreen_of_ids);

static struct i2c_driver i2c_md_driver = {
	.driver = {
		.name = "mipi_dsi",
		.of_match_table = rpi_touchscreen_of_ids,
	},
	.probe = i2c_md_probe,
	.remove = i2c_md_remove,
};

extern struct mipi_dsi_driver st7701_dsi_driver;
static int __init i2c_md_init(void)
{
	DBG_FUNC();
	mipi_dsi_driver_register(&st7701_dsi_driver);
	return i2c_add_driver(&i2c_md_driver);
}
module_init(i2c_md_init);

static void __exit i2c_md_exit(void)
{
	DBG_FUNC();
	i2c_del_driver(&i2c_md_driver);
	mipi_dsi_driver_unregister(&st7701_dsi_driver);
}
module_exit(i2c_md_exit);

MODULE_AUTHOR("Zhangqun Ming <north_sea@qq.com>");
MODULE_AUTHOR("Seeed, Inc.");
MODULE_DESCRIPTION("MIPI DSI driver");
MODULE_LICENSE("GPL v2");
