// SPDX-License-Identifier: GPL-2.0-only
/*
 * mipi_dsi.h - MIPI dsi module
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 *
 * IIC: 7-bit I2C slave address 0x45
 */

#ifndef __MIPI_DSI_H__
#define __MIPI_DSI_H__


#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>

#include <video/mipi_display.h>

#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>


#define I2C_MIPI_DSI


#define DBG_FUNC(format, x...)		printk(KERN_INFO "[DSI]%s:" format"\n", __func__, ##x)
#define DBG_PRINT(format, x...)		printk(KERN_INFO "[DSI]" format"\n", ##x)


#define DSI_DRIVER_NAME		        "i2c_mipi_dsi"


/* i2c: commands */
enum REG_ADDR {
	REG_ID = 0x80,
	REG_PORTA, /* BIT(2) for horizontal flip, BIT(3) for vertical flip */
	REG_PORTB,  // --
	REG_PORTC,
	REG_PORTD,
	REG_POWERON,// --
	REG_PWM,    // --
	REG_DDRA,
	REG_DDRB,
	REG_DDRC,
	REG_DDRD,
	REG_TEST,
	REG_WR_ADDRL,
	REG_WR_ADDRH,
	REG_READH,
	REG_READL,
	REG_WRITEH,
	REG_WRITEL,
	REG_ID2,

    REG_LCD_RST,
    REG_TP_RST,
    REG_TP_STATUS,
    REG_TP_POINT,

    REG_MAX
};

struct i2c_mipi_dsi {
	struct i2c_client *i2c;
	struct mutex mutex;
	
	// dsi
	struct mipi_dsi_device *dsi;

	// tp
	struct input_dev *input;
	struct touchscreen_properties prop;
};

extern struct i2c_mipi_dsi *i2c_md;

/* i2c */
int i2c_md_read(struct i2c_client *client, u8 reg, u8 *buf, int len);
void i2c_md_write(struct i2c_client *client, u8 reg, u8 val);

/* drm_panel_funcs */
int i2c_md_enable(void);
int i2c_md_disable(void);
int i2c_md_prepare(void);
int i2c_md_unprepare(void);

// touch panel
int tp_init(struct i2c_mipi_dsi *md);
int tp_deinit(struct i2c_mipi_dsi *md);

#endif /*End of header guard macro */
