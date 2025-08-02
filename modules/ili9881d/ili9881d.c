// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017-2018, Bootlin
 * Copyright (C) 2021, Henson Li <henson@cutiepi.io>
 * Copyright (C) 2021, Penk Chen <penk@cutiepi.io>
 * Copyright (C) 2022, Mark Williams <mark@crystalfontz.com>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>
#include <linux/version.h>
#include <linux/backlight.h>


enum ili9881d_op {
	ILI9881C_SWITCH_PAGE,
	ILI9881C_COMMAND,
};

struct ili9881d_instr {
	enum ili9881d_op	op;

	union arg {
		struct cmd {
			u8	cmd;
			u8	data;
		} cmd;
		u8	page;
	} arg;
};

enum ili9881_desc_flags {
	ILI9881_FLAGS_NO_SHUTDOWN_CMDS = BIT(0),
	ILI9881_FLAGS_PANEL_ON_IN_PREPARE = BIT(1),
	ILI9881_FLAGS_MAX = BIT(31),
};

struct ili9881d_desc {
	const struct ili9881d_instr *init;
	const size_t init_length;
	const struct drm_display_mode *mode;
	const unsigned long mode_flags;
	unsigned int lanes;
	enum ili9881_desc_flags flags;
};

struct ili9881d {
	struct drm_panel	panel;
	struct mipi_dsi_device	*dsi;
	const struct ili9881d_desc	*desc;

	struct regulator	*power;
	struct gpio_desc	*reset;

	enum drm_panel_orientation	orientation;
};

#define ILI9881C_SWITCH_PAGE_INSTR(_page)	\
	{					\
		.op = ILI9881C_SWITCH_PAGE,	\
		.arg = {			\
			.page = (_page),	\
		},				\
	}

#define ILI9881C_COMMAND_INSTR(_cmd, _data)		\
	{						\
		.op = ILI9881C_COMMAND,		\
		.arg = {				\
			.cmd = {			\
				.cmd = (_cmd),		\
				.data = (_data),	\
			},				\
		},					\
	}

static const struct ili9881d_instr gjx101c7_init[] = {
	ILI9881C_SWITCH_PAGE_INSTR(0x03),        
	//GIP_1),
	ILI9881C_COMMAND_INSTR(0x01,0x00),
	ILI9881C_COMMAND_INSTR(0x02,0x00),
	ILI9881C_COMMAND_INSTR(0x03,0x53),
	ILI9881C_COMMAND_INSTR(0x04,0xD3),
	ILI9881C_COMMAND_INSTR(0x05,0x00),
	ILI9881C_COMMAND_INSTR(0x06,0x0D),
	ILI9881C_COMMAND_INSTR(0x07,0x08),
	ILI9881C_COMMAND_INSTR(0x08,0x00),
	ILI9881C_COMMAND_INSTR(0x09,0x00),
	ILI9881C_COMMAND_INSTR(0x0a,0x00),
	ILI9881C_COMMAND_INSTR(0x0b,0x00),
	ILI9881C_COMMAND_INSTR(0x0c,0x00),
	ILI9881C_COMMAND_INSTR(0x0d,0x00),
	ILI9881C_COMMAND_INSTR(0x0e,0x00),
	ILI9881C_COMMAND_INSTR(0x0f,0x28),
	ILI9881C_COMMAND_INSTR(0x10,0x28),
	ILI9881C_COMMAND_INSTR(0x11,0x00),
	ILI9881C_COMMAND_INSTR(0x12,0x00),
	ILI9881C_COMMAND_INSTR(0x13,0x00),
	ILI9881C_COMMAND_INSTR(0x14,0x00),
	ILI9881C_COMMAND_INSTR(0x15,0x00),
	ILI9881C_COMMAND_INSTR(0x16,0x00),
	ILI9881C_COMMAND_INSTR(0x17,0x00),
	ILI9881C_COMMAND_INSTR(0x18,0x00),
	ILI9881C_COMMAND_INSTR(0x19,0x00),
	ILI9881C_COMMAND_INSTR(0x1a,0x00),
	ILI9881C_COMMAND_INSTR(0x1b,0x00),
	ILI9881C_COMMAND_INSTR(0x1c,0x00),
	ILI9881C_COMMAND_INSTR(0x1d,0x00),
	ILI9881C_COMMAND_INSTR(0x1e,0x40),
	ILI9881C_COMMAND_INSTR(0x1f,0x80),
	ILI9881C_COMMAND_INSTR(0x20,0x06),
	ILI9881C_COMMAND_INSTR(0x21,0x01),
	ILI9881C_COMMAND_INSTR(0x22,0x00),
	ILI9881C_COMMAND_INSTR(0x23,0x00),
	ILI9881C_COMMAND_INSTR(0x24,0x00),
	ILI9881C_COMMAND_INSTR(0x25,0x00),
	ILI9881C_COMMAND_INSTR(0x26,0x00),
	ILI9881C_COMMAND_INSTR(0x27,0x00),
	ILI9881C_COMMAND_INSTR(0x28,0x33),
	ILI9881C_COMMAND_INSTR(0x29,0x33),
	ILI9881C_COMMAND_INSTR(0x2a,0x00),
	ILI9881C_COMMAND_INSTR(0x2b,0x00),
	ILI9881C_COMMAND_INSTR(0x2c,0x00),
	ILI9881C_COMMAND_INSTR(0x2d,0x00),
	ILI9881C_COMMAND_INSTR(0x2e,0x00),
	ILI9881C_COMMAND_INSTR(0x2f,0x00),
	ILI9881C_COMMAND_INSTR(0x30,0x00),
	ILI9881C_COMMAND_INSTR(0x31,0x00),
	ILI9881C_COMMAND_INSTR(0x32,0x00),
	ILI9881C_COMMAND_INSTR(0x33,0x00),
	ILI9881C_COMMAND_INSTR(0x34,0x03),
	ILI9881C_COMMAND_INSTR(0x35,0x00),
	ILI9881C_COMMAND_INSTR(0x36,0x00),
	ILI9881C_COMMAND_INSTR(0x37,0x00),
	ILI9881C_COMMAND_INSTR(0x38,0x96),
	ILI9881C_COMMAND_INSTR(0x39,0x00),
	ILI9881C_COMMAND_INSTR(0x3a,0x00),
	ILI9881C_COMMAND_INSTR(0x3b,0x00),
	ILI9881C_COMMAND_INSTR(0x3c,0x00),
	ILI9881C_COMMAND_INSTR(0x3d,0x00),
	ILI9881C_COMMAND_INSTR(0x3e,0x00),
	ILI9881C_COMMAND_INSTR(0x3f,0x00),
	ILI9881C_COMMAND_INSTR(0x40,0x00),
	ILI9881C_COMMAND_INSTR(0x41,0x00),
	ILI9881C_COMMAND_INSTR(0x42,0x00),
	ILI9881C_COMMAND_INSTR(0x43,0x00),
	ILI9881C_COMMAND_INSTR(0x44,0x00),
	//GIP_2),
	ILI9881C_COMMAND_INSTR(0x50,0x00),
	ILI9881C_COMMAND_INSTR(0x51,0x23),
	ILI9881C_COMMAND_INSTR(0x52,0x45),
	ILI9881C_COMMAND_INSTR(0x53,0x67),
	ILI9881C_COMMAND_INSTR(0x54,0x89),
	ILI9881C_COMMAND_INSTR(0x55,0xAB),
	ILI9881C_COMMAND_INSTR(0x56,0x01),
	ILI9881C_COMMAND_INSTR(0x57,0x23),
	ILI9881C_COMMAND_INSTR(0x58,0x45),
	ILI9881C_COMMAND_INSTR(0x59,0x67),
	ILI9881C_COMMAND_INSTR(0x5a,0x89),
	ILI9881C_COMMAND_INSTR(0x5b,0xAB),
	ILI9881C_COMMAND_INSTR(0x5c,0xCD),
	ILI9881C_COMMAND_INSTR(0x5d,0xEF),
	//GIP_3),
	ILI9881C_COMMAND_INSTR(0x5e,0x00),
	ILI9881C_COMMAND_INSTR(0x5f,0x08),
	ILI9881C_COMMAND_INSTR(0x60,0x08),
	ILI9881C_COMMAND_INSTR(0x61,0x06),
	ILI9881C_COMMAND_INSTR(0x62,0x06),
	ILI9881C_COMMAND_INSTR(0x63,0x01),
	ILI9881C_COMMAND_INSTR(0x64,0x01),
	ILI9881C_COMMAND_INSTR(0x65,0x00),
	ILI9881C_COMMAND_INSTR(0x66,0x00),
	ILI9881C_COMMAND_INSTR(0x67,0x02),
	ILI9881C_COMMAND_INSTR(0x68,0x15),
	ILI9881C_COMMAND_INSTR(0x69,0x15),
	ILI9881C_COMMAND_INSTR(0x6a,0x14),
	ILI9881C_COMMAND_INSTR(0x6b,0x14),
	ILI9881C_COMMAND_INSTR(0x6c,0x0D),
	ILI9881C_COMMAND_INSTR(0x6d,0x0D),
	ILI9881C_COMMAND_INSTR(0x6e,0x0C),
	ILI9881C_COMMAND_INSTR(0x6f,0x0C),
	ILI9881C_COMMAND_INSTR(0x70,0x0F),
	ILI9881C_COMMAND_INSTR(0x71,0x0F),
	ILI9881C_COMMAND_INSTR(0x72,0x0E),
	ILI9881C_COMMAND_INSTR(0x73,0x0E),
	ILI9881C_COMMAND_INSTR(0x74,0x02),

	ILI9881C_COMMAND_INSTR(0x75,0x08),
	ILI9881C_COMMAND_INSTR(0x76,0x08),
	ILI9881C_COMMAND_INSTR(0x77,0x06),
	ILI9881C_COMMAND_INSTR(0x78,0x06),
	ILI9881C_COMMAND_INSTR(0x79,0x01),
	ILI9881C_COMMAND_INSTR(0x7a,0x01),
	ILI9881C_COMMAND_INSTR(0x7b,0x00),
	ILI9881C_COMMAND_INSTR(0x7c,0x00),
	ILI9881C_COMMAND_INSTR(0x7d,0x02),
	ILI9881C_COMMAND_INSTR(0x7e,0x15),
	ILI9881C_COMMAND_INSTR(0x7f,0x15),
	ILI9881C_COMMAND_INSTR(0x80,0x14),
	ILI9881C_COMMAND_INSTR(0x81,0x14),
	ILI9881C_COMMAND_INSTR(0x82,0x0D),
	ILI9881C_COMMAND_INSTR(0x83,0x0D),
	ILI9881C_COMMAND_INSTR(0x84,0x0C),
	ILI9881C_COMMAND_INSTR(0x85,0x0C),
	ILI9881C_COMMAND_INSTR(0x86,0x0F),
	ILI9881C_COMMAND_INSTR(0x87,0x0F),
	ILI9881C_COMMAND_INSTR(0x88,0x0E),
	ILI9881C_COMMAND_INSTR(0x89,0x0E),
	ILI9881C_COMMAND_INSTR(0x8A,0x02),
	//page4),
	ILI9881C_SWITCH_PAGE_INSTR(0x04),
	ILI9881C_COMMAND_INSTR(0x6E,0x2B),
	ILI9881C_COMMAND_INSTR(0x6F,0x37),
	ILI9881C_COMMAND_INSTR(0x3A,0xA4),
	ILI9881C_COMMAND_INSTR(0x8D,0x1A),
	ILI9881C_COMMAND_INSTR(0x87,0xBA),
	ILI9881C_COMMAND_INSTR(0xB2,0xD1),
	ILI9881C_COMMAND_INSTR(0x88,0x0B),
	ILI9881C_COMMAND_INSTR(0x38,0x01),
	ILI9881C_COMMAND_INSTR(0x39,0x00),
	ILI9881C_COMMAND_INSTR(0xB5,0x07),
	ILI9881C_COMMAND_INSTR(0x31,0x75),
	ILI9881C_COMMAND_INSTR(0x3B,0x98),
	//CMD_Page
	ILI9881C_SWITCH_PAGE_INSTR(0x01),
	ILI9881C_COMMAND_INSTR(0x22,0x0A),
	ILI9881C_COMMAND_INSTR(0x31,0x00),
	ILI9881C_COMMAND_INSTR(0x53,0x40),
	ILI9881C_COMMAND_INSTR(0x55,0x40),
	ILI9881C_COMMAND_INSTR(0x50,0x99),
	ILI9881C_COMMAND_INSTR(0x51,0x94),
	ILI9881C_COMMAND_INSTR(0x60,0x10),
	ILI9881C_COMMAND_INSTR(0x62,0x20),
	//Pos Register
	ILI9881C_COMMAND_INSTR(0xA0,0x00),
	ILI9881C_COMMAND_INSTR(0xA1,0x00),
	ILI9881C_COMMAND_INSTR(0xA2,0x15),
	ILI9881C_COMMAND_INSTR(0xA3,0x14),
	ILI9881C_COMMAND_INSTR(0xA4,0x1B),
	ILI9881C_COMMAND_INSTR(0xA5,0x2F),
	ILI9881C_COMMAND_INSTR(0xA6,0x25),
	ILI9881C_COMMAND_INSTR(0xA7,0x24),
	ILI9881C_COMMAND_INSTR(0xA8,0x80),
	ILI9881C_COMMAND_INSTR(0xA9,0x1F),
	ILI9881C_COMMAND_INSTR(0xAA,0x2C),
	ILI9881C_COMMAND_INSTR(0xAB,0x6C),
	ILI9881C_COMMAND_INSTR(0xAC,0x16),
	ILI9881C_COMMAND_INSTR(0xAD,0x14),
	ILI9881C_COMMAND_INSTR(0xAE,0x4D),
	ILI9881C_COMMAND_INSTR(0xAF,0x20),
	ILI9881C_COMMAND_INSTR(0xB0,0x29),
	ILI9881C_COMMAND_INSTR(0xB1,0x4F),
	ILI9881C_COMMAND_INSTR(0xB2,0x5F),
	ILI9881C_COMMAND_INSTR(0xB3,0x23),
	//Neg Register
	ILI9881C_COMMAND_INSTR(0xC0,0x00),
	ILI9881C_COMMAND_INSTR(0xC1,0x2E),
	ILI9881C_COMMAND_INSTR(0xC2,0x3B),
	ILI9881C_COMMAND_INSTR(0xC3,0x15),
	ILI9881C_COMMAND_INSTR(0xC4,0x16),
	ILI9881C_COMMAND_INSTR(0xC5,0x28),
	ILI9881C_COMMAND_INSTR(0xC6,0x1A),
	ILI9881C_COMMAND_INSTR(0xC7,0x1C),
	ILI9881C_COMMAND_INSTR(0xC8,0xA7),
	ILI9881C_COMMAND_INSTR(0xC9,0x1B),
	ILI9881C_COMMAND_INSTR(0xCA,0x28),
	ILI9881C_COMMAND_INSTR(0xCB,0x92),
	ILI9881C_COMMAND_INSTR(0xCC,0x1F),
	ILI9881C_COMMAND_INSTR(0xCD,0x1C),
	ILI9881C_COMMAND_INSTR(0xCE,0x4B),
	ILI9881C_COMMAND_INSTR(0xCF,0x1F),
	ILI9881C_COMMAND_INSTR(0xD0,0x28),
	ILI9881C_COMMAND_INSTR(0xD1,0x4E),
	ILI9881C_COMMAND_INSTR(0xD2,0x5C),
	ILI9881C_COMMAND_INSTR(0xD3,0x23),
	// origin parameters
	ILI9881C_SWITCH_PAGE_INSTR(0),
	//PWM
	ILI9881C_COMMAND_INSTR(0x51, 0x0F),
	ILI9881C_COMMAND_INSTR(0x52, 0xFF),
	ILI9881C_COMMAND_INSTR(0x53, 0x2C),

	ILI9881C_COMMAND_INSTR(0x11, 0x00),
	ILI9881C_COMMAND_INSTR(0x29, 0x00),
	ILI9881C_COMMAND_INSTR(0x35, 0x00),
};

static inline struct ili9881d *panel_to_ili9881d(struct drm_panel *panel)
{
	return container_of(panel, struct ili9881d, panel);
}

/*
 * The panel seems to accept some private DCS commands that map
 * directly to registers.
 *
 * It is organised by page, with each page having its own set of
 * registers, and the first page looks like it's holding the standard
 * DCS commands.
 *
 * So before any attempt at sending a command or data, we have to be
 * sure if we're in the right page or not.
 */
static int ili9881d_switch_page(struct ili9881d *ctx, u8 page)
{
	u8 buf[4] = { 0xff, 0x98, 0x81, page };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return 0;
}

static int ili9881d_send_cmd_data(struct ili9881d *ctx, u8 cmd, u8 data)
{
	u8 buf[2] = { cmd, data };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return 0;
}

static int ili9881d_prepare(struct drm_panel *panel)
{
	struct ili9881d *ctx = panel_to_ili9881d(panel);
	unsigned int i;
	int ret;

	/* Power the panel */
	ret = regulator_enable(ctx->power);
	if (ret)
		return ret;
	msleep(5);

	/* And reset it */
	gpiod_set_value_cansleep(ctx->reset, 1);
	msleep(20);

	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(20);

	for (i = 0; i < ctx->desc->init_length; i++) {
		const struct ili9881d_instr *instr = &ctx->desc->init[i];

		if (instr->op == ILI9881C_SWITCH_PAGE)
			ret = ili9881d_switch_page(ctx, instr->arg.page);
		else if (instr->op == ILI9881C_COMMAND)
			ret = ili9881d_send_cmd_data(ctx, instr->arg.cmd.cmd,
						      instr->arg.cmd.data);

		if (ret) {
			return ret;
		}
	}

	ret = ili9881d_switch_page(ctx, 0);
	if (ret) {
		return ret;
	}
	
	ret = mipi_dsi_dcs_set_tear_on(ctx->dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
	if (ret)
		return ret;

	if (ctx->desc->flags & ILI9881_FLAGS_PANEL_ON_IN_PREPARE) {
		msleep(120);

		ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	}

	return 0;
}

static int ili9881d_enable(struct drm_panel *panel)
{
	struct ili9881d *ctx = panel_to_ili9881d(panel);

	if (!(ctx->desc->flags & ILI9881_FLAGS_PANEL_ON_IN_PREPARE)) {
		msleep(120);

		mipi_dsi_dcs_set_display_on(ctx->dsi);
	}

	return 0;
}

static int ili9881d_disable(struct drm_panel *panel)
{
	struct ili9881d *ctx = panel_to_ili9881d(panel);

	if (!(ctx->desc->flags & ILI9881_FLAGS_PANEL_ON_IN_PREPARE)) {
		mipi_dsi_dcs_set_display_off(ctx->dsi);
	}

	return 0;
}

static int ili9881d_unprepare(struct drm_panel *panel)
{
	struct ili9881d *ctx = panel_to_ili9881d(panel);

	if (!(ctx->desc->flags & ILI9881_FLAGS_NO_SHUTDOWN_CMDS)) {
		if (ctx->desc->flags & ILI9881_FLAGS_PANEL_ON_IN_PREPARE)
			mipi_dsi_dcs_set_display_off(ctx->dsi);

		mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	}

	regulator_disable(ctx->power);
	gpiod_set_value_cansleep(ctx->reset, 1);

	return 0;
}

static const struct drm_display_mode nwe080_default_mode = {
	.clock 		= 72394,

	.hdisplay	= 800,
	.hsync_start	= 800 + 46, // hdisplay+HFP
	.hsync_end	= 800 + 46 + 20, // hdisplay+HFP+HSW
	.htotal		= 800 + 46 + 20 + 46, // hdisplay+HFP+HSW+HBP

	.vdisplay	= 1280,
	.vsync_start	= 1280 + 20, // vdisplay+VFP
	.vsync_end	= 1280 + 20 + 8, // vdisplay+VFP+VSW
	.vtotal		= 1280 + 20 + 8 + 15, // vdisplay+VFP+VSW+VBP

	.width_mm 	= 135,
	.height_mm 	= 225,
};

static int ili9881d_get_modes(struct drm_panel *panel,
			      struct drm_connector *connector)
{
	struct ili9881d *ctx = panel_to_ili9881d(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, ctx->desc->mode);
	if (!mode) {
		dev_err(&ctx->dsi->dev, "failed to add mode %ux%ux@%u\n",
			ctx->desc->mode->hdisplay,
			ctx->desc->mode->vdisplay,
			drm_mode_vrefresh(ctx->desc->mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	drm_connector_set_panel_orientation(connector, ctx->orientation);
	return 1;
}

static enum drm_panel_orientation ili9881d_get_orientation(struct drm_panel *panel)
{
	struct ili9881d *ctx = panel_to_ili9881d(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs ili9881d_funcs = {
	.prepare	= ili9881d_prepare,
	.unprepare	= ili9881d_unprepare,
	.enable		= ili9881d_enable,
	.disable	= ili9881d_disable,
	.get_modes	= ili9881d_get_modes,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	.get_orientation = ili9881d_get_orientation,
#endif
};

static int ili9881d_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct ili9881d *ctx;
	int ret;

	ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dsi = dsi;
	ctx->desc = of_device_get_match_data(&dsi->dev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	ctx->panel.prepare_prev_first = true;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	ctx->panel.prepare_upstream_first = true;
#endif
	drm_panel_init(&ctx->panel, &dsi->dev, &ili9881d_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->power = devm_regulator_get(&dsi->dev, "power");
	if (IS_ERR(ctx->power)) {
		dev_err(&dsi->dev, "Couldn't get our power regulator\n");
		return PTR_ERR(ctx->power);
	}

	ctx->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset)) {
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
		return PTR_ERR(ctx->reset);
	}

	ret = of_drm_get_panel_orientation(dsi->dev.of_node, &ctx->orientation);
	if (ret) {
		dev_err(&dsi->dev, "%pOF: failed to get orientation: %d\n",
			dsi->dev.of_node, ret);
		return ret;
	}
#if IS_ENABLED(CONFIG_DRM_PANEL) && (IS_BUILTIN(CONFIG_BACKLIGHT_CLASS_DEVICE) || \
	(IS_MODULE(CONFIG_DRM) && IS_MODULE(CONFIG_BACKLIGHT_CLASS_DEVICE)))
	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;
#else
	ctx->panel.backlight = devm_of_find_backlight(ctx->panel.dev);
#endif
	drm_panel_add(&ctx->panel);

	dsi->mode_flags = ctx->desc->mode_flags;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes = ctx->desc->lanes;

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&ctx->panel);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int ili9881d_dsi_remove(struct mipi_dsi_device *dsi)
#else
static void ili9881d_dsi_remove(struct mipi_dsi_device *dsi)
#endif
{
	struct ili9881d *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	gpiod_set_value_cansleep(ctx->reset, 1);
	// regulator_disable(ctx->power);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static const struct ili9881d_desc gjx101c7_desc = {
	.init = gjx101c7_init,
	.init_length = ARRAY_SIZE(gjx101c7_init),
	.mode = &nwe080_default_mode,
	.mode_flags = MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_MODE_VIDEO,
	.flags = ILI9881_FLAGS_NO_SHUTDOWN_CMDS |
		 ILI9881_FLAGS_PANEL_ON_IN_PREPARE,
	.lanes = 4,
};

static const struct of_device_id ili9881d_of_match[] = {
	{ .compatible = "gjx,gjx101c7", .data = &gjx101c7_desc },
	{}
};
MODULE_DEVICE_TABLE(of, ili9881d_of_match);

static struct mipi_dsi_driver ili9881d_dsi_driver = {
	.probe		= ili9881d_dsi_probe,
	.remove		= ili9881d_dsi_remove,
	.driver = {
		.name		= "ili9881d-dsi",
		.of_match_table	= ili9881d_of_match,
	},
};

static int __init ili9881d_init(void)
{
	int ret = 0;
	ret = mipi_dsi_driver_register(&ili9881d_dsi_driver);
	return ret;
}
late_initcall(ili9881d_init);

static void __exit ili9881d_exit(void)
{
	mipi_dsi_driver_unregister(&ili9881d_dsi_driver);
}
module_exit(ili9881d_exit);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Ilitek ILI9881D Controller Driver");
MODULE_LICENSE("GPL v2");
