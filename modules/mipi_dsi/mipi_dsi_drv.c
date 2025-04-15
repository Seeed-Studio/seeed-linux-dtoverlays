// SPDX-License-Identifier: GPL-2.0
/*
 * This is a linux kernel driver for MIPI-DSI 
 * panel with touch panel attached to I2C bus.
 *
 * Copyright (c) 2020 Seeed Studio
 * Zhangqun Ming<north_sea@qq.com>
 *
 * I2C slave address: 0x45
 */
#include "mipi_dsi.h"
#include <linux/version.h>

static int dsi_status = DSI_OK; //0:dsi is ok, not 0:something wrong with dsi
static ssize_t dsi_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!dsi_status)
		return scnprintf(buf, PAGE_SIZE, "ok\r\n");
	else
		return scnprintf(buf, PAGE_SIZE, "error : %d\r\n", dsi_status);
}

static DEVICE_ATTR(dsi_state, 0444, dsi_state_show, NULL);

static struct attribute *dsi_state_attrs[] = {
	&dev_attr_dsi_state.attr,
	NULL
};

static const struct attribute_group dsi_attr_group = {
	.attrs = dsi_state_attrs,
};

/*static */int i2c_md_read(struct i2c_mipi_dsi *md, u8 reg, u8 *buf, int len)
{
	struct i2c_client *client = md->i2c;
	struct i2c_msg msgs[1];
	u8 addr_buf[1] = { reg };
	u8 data_buf[1] = { 0, };
	int ret;

	mutex_lock(&md->mutex);
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		mutex_unlock(&md->mutex);
		return -EIO;
	}

	usleep_range(1000, 1500);

	/* Read data from register */
	msgs[0].addr = client->addr;
	msgs[0].flags = I2C_M_RD;
	if (NULL == buf) {
		msgs[0].len = 1;
		msgs[0].buf = data_buf;
	}
	else {
		msgs[0].len = len;
		msgs[0].buf = buf;
	}

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		mutex_unlock(&md->mutex);
		return -EIO;
	}
	mutex_unlock(&md->mutex);

	if (NULL == buf) {
		return data_buf[0];	
	}
	else {
		return ret;
	}
}

/*static */void i2c_md_write(struct i2c_mipi_dsi *md, u8 reg, u8 val)
{
	struct i2c_client *client = md->i2c;
	int ret;

	mutex_lock(&md->mutex);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret)
		dev_err(&client->dev, "I2C write failed: %d\n", ret);

	usleep_range(1000, 1500);
	mutex_unlock(&md->mutex);
}


// MIPI-DSI driver

static int mipi_dsi_probe(struct mipi_dsi_device *dsi)
{
	int ret;

	DBG_PRINT("Probe MIPI-DSI driver");

	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes = 4;

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		dev_err(&dsi->dev, "failed to attach dsi to host: %d\n", ret);
		dsi_status = DSI_ATTACH_ERR;
	}

	return ret;
}

static struct mipi_dsi_driver mipi_dsi_driver = {
	.driver.name = MIPI_DSI_DRIVER_NAME,
	.probe = mipi_dsi_probe,
};


// MIPI-DSI device

static struct mipi_dsi_device *mipi_dsi_device(struct device *dev)
{
	struct mipi_dsi_device *dsi = NULL;
	struct device_node *endpoint, *dsi_host_node;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device_info info = {
		.type = MIPI_DSI_DRIVER_NAME,
		.channel = 0,
		.node = NULL,
	};

	DBG_PRINT("Add MIPI-DSI device to device tree");

	/* Look up the DSI host.  It needs to probe before we do. */
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "No endpoint node!");
		return ERR_PTR(-ENODEV);
	}

	dsi_host_node = of_graph_get_remote_port_parent(endpoint);
	if (!dsi_host_node) {
		dev_err(dev, "No dsi_host node!");
		goto error;
	}

	host = of_find_mipi_dsi_host_by_node(dsi_host_node);
	of_node_put(dsi_host_node);
	if (!host) {
		dev_err(dev, "Can't find mipi_dsi_host!");
		of_node_put(endpoint);
		return ERR_PTR(-EPROBE_DEFER);
	}

	info.node = of_graph_get_remote_port(endpoint);
	if (!info.node) {
		dev_err(dev, "Can't get remote port!");
		goto error;
	}

	of_node_put(endpoint);
	dsi = mipi_dsi_device_register_full(host, &info);
	if(IS_ERR(dsi)) {
		dev_err(dev, "Can't device register_full!");
		dsi_status = DSI_REG_ERR;
		return dsi;
	}

	return dsi;

error:
	of_node_put(endpoint);
	return ERR_PTR(-ENODEV);
}


// Panel

static int panel_prepare(struct drm_panel *panel)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	DBG_PRINT("Prepare panel");

	/* i2c */
	/* reset pin */
	i2c_md_write(md, REG_POWERON, 1);
	msleep(20);
	i2c_md_write(md, REG_LCD_RST, 0);
	msleep(20);
	i2c_md_write(md, REG_LCD_RST, 1);
	msleep(50);

	/* panel */
	if (funcs && funcs->prepare) {
		ret = funcs->prepare(panel);
		if (ret < 0){
			i2c_md_write(md, REG_POWERON, 0);
			i2c_md_write(md, REG_LCD_RST, 0);
			i2c_md_write(md, REG_PWM, 0);
			dsi_status = DSI_PANEL_ERR;
			return ret;
		}
	}
	return ret;
}

static int panel_unprepare(struct drm_panel *panel)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	DBG_PRINT("Unprepare panel");

	if (funcs && funcs->unprepare) {
		ret = funcs->unprepare(panel);
		if (ret < 0)
			return ret;
	}
	i2c_md_write(md, REG_LCD_RST, 0);
	return ret;
}

static int panel_enable(struct drm_panel * panel)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	DBG_PRINT("Enable panel");

	/* panel */
	if (funcs && funcs->enable) {
		ret = funcs->enable(panel);
		if (ret < 0)
			return ret;
	}

	/* i2c */
	i2c_md_write(md, REG_PWM, md->brightness);

	return ret;
}

static int panel_disable(struct drm_panel * panel)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	DBG_PRINT("Disable panel");

	/* i2c */
	i2c_md_write(md, REG_PWM, 0);

	/* panel */
	if (funcs && funcs->disable) {
		ret = funcs->disable(panel);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	if (funcs && funcs->get_modes) {
		ret = funcs->get_modes(panel, connector);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static const struct drm_panel_funcs panel_funcs = {
	.prepare = panel_prepare,
	.unprepare = panel_unprepare,
	.enable = panel_enable,
	.disable = panel_disable,
	.get_modes = panel_get_modes,
};


// Backlight device

static int backlight_update(struct backlight_device *bd)
{
	struct i2c_mipi_dsi *md = bl_get_data(bd);
	int brightness = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK ||
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(6, 12, 0))
		bd->props.fb_blank != FB_BLANK_UNBLANK ||
#endif
		bd->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) {
			brightness = 0;
		}

	DBG_FUNC("brightness=%d", brightness);
	md->brightness = brightness;
	i2c_md_write(md, REG_PWM, brightness);

	return 0;
}

static const struct backlight_ops backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status	= backlight_update,
};

static int backlight_init(struct i2c_mipi_dsi *md)
{
	struct device *dev = &md->i2c->dev;
	struct backlight_properties props;
	struct backlight_device *bd;

	DBG_FUNC("Register backlight device");
	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 255;
	bd = devm_backlight_device_register(dev, dev_name(dev),
					dev, md, &backlight_ops,
					&props);
	if (IS_ERR(bd)) {
		dev_err(dev, "failed to register backlight\n");
		return PTR_ERR(bd);
	}

	bd->props.brightness = 255;
	backlight_update_status(bd);

	return 0;
}


// I2C driver
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static int i2c_md_probe(struct i2c_client *i2c)
#else
static int i2c_md_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
#endif
{
	struct device *dev = &i2c->dev;
	struct i2c_mipi_dsi *md;
	int ret = 0;
	u8 mcu_img_ver[2];

	DBG_PRINT("Probe I2C driver");
	DBG_FUNC("Start");

	md = devm_kzalloc(dev, sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;

	i2c_set_clientdata(i2c, md);
	mutex_init(&md->mutex);
	md->i2c = i2c;
	md->panel_data = (struct panel_data *)of_device_get_match_data(dev);
	if (!md->panel_data) {
		dev_err(dev, "No valid panel data.\n");
		return -ENODEV;
	}

	ret = i2c_md_read(md, REG_ID, NULL, 0);
	if (ret < 0) {
		dev_err(dev, "I2C read id failed: %d\n", ret);
		return -ENODEV;
	}
	if (ret != 0xC3) {
		dev_err(dev, "Unknown chip id: 0x%02x\n", ret);
		return -ENODEV;
	}

	ret = i2c_md_read(md, REG_TP_VERSION, mcu_img_ver, ARRAY_SIZE(mcu_img_ver));
	if (ret < 0) {
		dev_err(dev, "I2C read STM32 firmware version failed: %d\n", ret);
		return -ENODEV;
	}
	DBG_FUNC("STM32 firmware version %u.%u", mcu_img_ver[0], mcu_img_ver[1]);

	i2c_md_write(md, REG_POWERON, 1);

	md->dsi = mipi_dsi_device(dev);
	if (IS_ERR(md->dsi)) {
		dev_err(dev, "DSI device registration failed!\n");
		dsi_status = DSI_FAILURE;
		return PTR_ERR(md->dsi);
	}

	DBG_FUNC("Add panel");
	md->panel_data->set_dsi(md->dsi);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	md->panel.prepare_prev_first = true;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	md->panel.prepare_upstream_first = true;
#endif
	drm_panel_init(&md->panel, dev, &panel_funcs, DRM_MODE_CONNECTOR_DSI);
	drm_panel_add(&md->panel);

	tp_init(md);
	backlight_init(md);

	ret = device_property_read_u32(dev, "mcu_auto_reset_enable", &md->mcu_auto_reset);
	if(ret < 0){	
		dev_err(dev, "Can't get the data of mcu_auto_reset!\n");
	}
	i2c_md_write(md, REG_MCU_AUTO_RESET, (md->mcu_auto_reset&0xff));

	ret = device_property_read_u32(dev, "tp_point_rotate", &md->tp_point_rotate);
	if(ret < 0){	
		dev_err(dev, "Can't get the data of tp_point_rotate!\n");
	}

	ret = sysfs_create_group(&i2c->dev.kobj, &dsi_attr_group);

	DBG_FUNC("Finish");
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int i2c_md_remove(struct i2c_client *i2c)
#else
static void i2c_md_remove(struct i2c_client *i2c)
#endif
{
	struct i2c_mipi_dsi *md = i2c_get_clientdata(i2c);

	DBG_PRINT("Remove I2C driver");

	tp_deinit(md);

	/* Turn off power */
	i2c_md_write(md, REG_POWERON, 0);
	i2c_md_write(md, REG_LCD_RST, 0);
	i2c_md_write(md, REG_PWM, 0);

	// mipi_dsi_detach(md->dsi); // TODO: check if this is needed
	drm_panel_remove(&md->panel);
	mipi_dsi_device_unregister(md->dsi);
	sysfs_remove_group(&i2c->dev.kobj, &dsi_attr_group);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static void i2c_md_shutdown(struct i2c_client *i2c)
{
	struct i2c_mipi_dsi *md = i2c_get_clientdata(i2c);

	DBG_PRINT("Shutdown I2C driver");

	tp_deinit(md);

	/* Turn off power */
	i2c_md_write(md, REG_POWERON, 0);
	i2c_md_write(md, REG_LCD_RST, 0);
	i2c_md_write(md, REG_PWM, 0);

	// mipi_dsi_detach(md->dsi); // TODO: check if this is needed
	drm_panel_remove(&md->panel);
	mipi_dsi_device_unregister(md->dsi);
}

extern const struct panel_data ili9881d_data;
static const struct of_device_id i2c_md_of_ids[] = {
	{
		.compatible = "i2c_dsi,ili9881d",
		.data = (const void*)&ili9881d_data,
	},
	{ } /* sentinel */
};
MODULE_DEVICE_TABLE(of, i2c_md_of_ids);

static struct i2c_driver i2c_md_driver = {
	.driver = {
		.name = "mipi_dsi",
		.of_match_table = i2c_md_of_ids,
	},
	.probe = i2c_md_probe,
	.remove = i2c_md_remove,
	.shutdown = i2c_md_shutdown,
};


// Kernel module

static int __init i2c_md_init(void)
{
	int ret;

	DBG_PRINT("Initialize kernel module");

	DBG_FUNC("Add I2C driver");
	ret = i2c_add_driver(&i2c_md_driver);
	if (ret < 0)
		return ret;

	DBG_FUNC("Register MIPI-DSI driver");
	ret = mipi_dsi_driver_register(&mipi_dsi_driver);
	if (ret < 0)
		dsi_status = DSI_REG_ERR;
		return ret;

	return ret;
}
module_init(i2c_md_init);

static void __exit i2c_md_exit(void)
{
	DBG_PRINT("Exit kernel module");

	DBG_FUNC("Unregister MIPI-DSI driver");
	mipi_dsi_driver_unregister(&mipi_dsi_driver);

	DBG_FUNC("Delete I2C driver");
	i2c_del_driver(&i2c_md_driver);
}
module_exit(i2c_md_exit);

MODULE_AUTHOR("Zhangqun Ming <north_sea@qq.com>");
MODULE_AUTHOR("Seeed, Inc.");
MODULE_DESCRIPTION("MIPI-DSI driver");
MODULE_LICENSE("GPL v2");
