// SPDX-License-Identifier: GPL-2.0+
/*
 * ILI9881D panel driver
 * 
 * Copyright (c) 2020 Seeed Studio
 * Zhangqun Ming<north_sea@qq.com>
 */
#include "mipi_dsi.h"


static struct mipi_dsi_device *ili9881d_dsi = NULL;

static const struct drm_display_mode ili9881d_modes = {
	.clock		= 62712 /*73164*/ /*83616*/ /*94068*/ /*104520*/,

	.hdisplay	= 720,
	.hsync_start= 720 + 10,
	.hsync_end	= 720 + 10 + 20,
	.htotal		= 720 + 10 + 20 + 30,

	.vdisplay	= 1280,
	.vsync_start= 1280 + 10,
	.vsync_end	= 1280 + 10 + 20,
	.vtotal		= 1280 + 10 + 20 + 30,

	.width_mm	= 62,
	.height_mm	= 110,
};

static int ili9881d_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	// DBG_PRINT("Get ILI9881D mode");

	mode = drm_mode_duplicate(connector->dev, &ili9881d_modes);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			mode->hdisplay, mode->vdisplay,
			drm_mode_vrefresh(mode));
		return -ENOMEM;
	}
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

#define ILI9881_PAGE(_page)					DSI_DCS_WRITE(dsi,0xff,0x98,0x81,_page)
#define IILI9881_COMMAND(_cmd,_data...)		DSI_DCS_WRITE(dsi,_cmd,_data)
static int ili9881d_prepare(struct drm_panel *panel)
{
	struct mipi_dsi_device *dsi = ili9881d_dsi;
	int ret = 0;
	u16 addr = 0xda;
	u32 val[4] = {0};

	DBG_PRINT("Prepare ILI9881D");

	if (!dsi){
		DBG_FUNC("NO DSI");
		return -1;
	}

	ILI9881_PAGE(0x01);           
	IILI9881_COMMAND(0x91,0x00);
	IILI9881_COMMAND(0x92,0x00);
	IILI9881_COMMAND(0x93,0x72);
	IILI9881_COMMAND(0x94,0x00);
	IILI9881_COMMAND(0x95,0x00);
	IILI9881_COMMAND(0x96,0x09);
	IILI9881_COMMAND(0x97,0x00);
	IILI9881_COMMAND(0x98,0x00);

	IILI9881_COMMAND(0x09,0x01);
	IILI9881_COMMAND(0x0a,0x00);
	IILI9881_COMMAND(0x0b,0x00);
	IILI9881_COMMAND(0x0c,0x01);
	IILI9881_COMMAND(0x0d,0x00);
	IILI9881_COMMAND(0x0e,0x00);
	IILI9881_COMMAND(0x0f,0x1D);
	IILI9881_COMMAND(0x10,0x1D);
	IILI9881_COMMAND(0x11,0x00);
	IILI9881_COMMAND(0x12,0x00);
	IILI9881_COMMAND(0x13,0x00);
	IILI9881_COMMAND(0x14,0x00);
	IILI9881_COMMAND(0x15,0x00);
	IILI9881_COMMAND(0x16,0x00);
	IILI9881_COMMAND(0x17,0x00);
	IILI9881_COMMAND(0x18,0x00);
	IILI9881_COMMAND(0x19,0x00);
	IILI9881_COMMAND(0x1a,0x00);
	IILI9881_COMMAND(0x1b,0x00);
	IILI9881_COMMAND(0x1c,0x00);
	IILI9881_COMMAND(0x1d,0x00);
	IILI9881_COMMAND(0x1e,0xc0);
	IILI9881_COMMAND(0x1f,0x00);
	IILI9881_COMMAND(0x20,0x06);
	IILI9881_COMMAND(0x21,0x02);
	IILI9881_COMMAND(0x22,0x00);
	IILI9881_COMMAND(0x23,0x00);
	IILI9881_COMMAND(0x24,0x00);
	IILI9881_COMMAND(0x25,0x00);
	IILI9881_COMMAND(0x26,0x00);
	IILI9881_COMMAND(0x27,0x00);
	IILI9881_COMMAND(0x28,0x33);
	IILI9881_COMMAND(0x29,0x03);
	IILI9881_COMMAND(0x2a,0x00);
	IILI9881_COMMAND(0x2b,0x00);
	IILI9881_COMMAND(0x2c,0x00);
	IILI9881_COMMAND(0x2d,0x00);
	IILI9881_COMMAND(0x2e,0x00);
	IILI9881_COMMAND(0x2f,0x00);
	IILI9881_COMMAND(0x30,0x00);
	IILI9881_COMMAND(0x31,0x00);
	IILI9881_COMMAND(0x32,0x00);
	IILI9881_COMMAND(0x33,0x00);
	IILI9881_COMMAND(0x34,0x04);
	IILI9881_COMMAND(0x35,0x00);
	IILI9881_COMMAND(0x36,0x00);
	IILI9881_COMMAND(0x37,0x00);
	IILI9881_COMMAND(0x38,0x3C);
	IILI9881_COMMAND(0x39,0x07);
	IILI9881_COMMAND(0x3a,0x00);
	IILI9881_COMMAND(0x3b,0x00);
	IILI9881_COMMAND(0x3c,0x00);

	IILI9881_COMMAND(0x40,0x03);
	IILI9881_COMMAND(0x41,0x20);
	IILI9881_COMMAND(0x42,0x00);
	IILI9881_COMMAND(0x43,0x00);
	IILI9881_COMMAND(0x44,0x03);
	IILI9881_COMMAND(0x45,0x00);
	IILI9881_COMMAND(0x46,0x01);
	IILI9881_COMMAND(0x47,0x08);
	IILI9881_COMMAND(0x48,0x00);
	IILI9881_COMMAND(0x49,0x00);
	IILI9881_COMMAND(0x4a,0x00);
	IILI9881_COMMAND(0x4b,0x00);

	// ==== GL[3OUT=
	IILI9881_COMMAND(0x4c,0x01);
	IILI9881_COMMAND(0x4d,0x54);
	IILI9881_COMMAND(0x4e,0x57);
	IILI9881_COMMAND(0x4f,0x9b);
	IILI9881_COMMAND(0x50,0xf9);
	IILI9881_COMMAND(0x51,0x27);
	IILI9881_COMMAND(0x52,0x2f);
	IILI9881_COMMAND(0x53,0xf2);
	IILI9881_COMMAND(0x54,0xff);
	IILI9881_COMMAND(0x55,0xff);
	IILI9881_COMMAND(0x56,0xff);

	// ==== GR[3OUT==
	IILI9881_COMMAND(0x57,0x01);
	IILI9881_COMMAND(0x58,0x54);
	IILI9881_COMMAND(0x59,0x46);
	IILI9881_COMMAND(0x5a,0x8a);
	IILI9881_COMMAND(0x5b,0xf8);
	IILI9881_COMMAND(0x5c,0x26);
	IILI9881_COMMAND(0x5d,0x2f);
	IILI9881_COMMAND(0x5e,0xf2);
	IILI9881_COMMAND(0x5f,0xff);
	IILI9881_COMMAND(0x60,0xff);
	IILI9881_COMMAND(0x61,0xff);

	IILI9881_COMMAND(0x62,0x06);

	// == GOUT:4]_BWUTL[5:0]==
	IILI9881_COMMAND(0x63,0x01);
	IILI9881_COMMAND(0x64,0x00);
	IILI9881_COMMAND(0x65,0xa4);
	IILI9881_COMMAND(0x66,0xa5);
	IILI9881_COMMAND(0x67,0x58);
	IILI9881_COMMAND(0x68,0x5a);
	IILI9881_COMMAND(0x69,0x54);
	IILI9881_COMMAND(0x6a,0x56);
	IILI9881_COMMAND(0x6b,0x06);
	IILI9881_COMMAND(0x6c,0xff);
	IILI9881_COMMAND(0x6d,0x08);
	IILI9881_COMMAND(0x6e,0x02);
	IILI9881_COMMAND(0x6f,0xff);
	IILI9881_COMMAND(0x70,0x02);
	IILI9881_COMMAND(0x71,0x02);
	IILI9881_COMMAND(0x72,0xff);
	IILI9881_COMMAND(0x73,0xff);
	IILI9881_COMMAND(0x74,0xff);
	IILI9881_COMMAND(0x75,0xff);
	IILI9881_COMMAND(0x76,0xff);
	IILI9881_COMMAND(0x77,0xff);
	IILI9881_COMMAND(0x78,0xff);

	// == GOUT:4]_BWUTR[5:0]==
	IILI9881_COMMAND(0x79,0x01);
	IILI9881_COMMAND(0x7a,0x00);
	IILI9881_COMMAND(0x7b,0xa4);
	IILI9881_COMMAND(0x7c,0xa5);
	IILI9881_COMMAND(0x7d,0x59);
	IILI9881_COMMAND(0x7e,0x5b);
	IILI9881_COMMAND(0x7f,0x55);
	IILI9881_COMMAND(0x80,0x57);
	IILI9881_COMMAND(0x81,0x07);
	IILI9881_COMMAND(0x82,0xff);
	IILI9881_COMMAND(0x83,0x09);
	IILI9881_COMMAND(0x84,0x02);
	IILI9881_COMMAND(0x85,0xff);
	IILI9881_COMMAND(0x86,0x02);
	IILI9881_COMMAND(0x87,0x02);
	IILI9881_COMMAND(0x88,0xff);
	IILI9881_COMMAND(0x89,0xff);
	IILI9881_COMMAND(0x8a,0xff);
	IILI9881_COMMAND(0x8b,0xff);
	IILI9881_COMMAND(0x8c,0xff);
	IILI9881_COMMAND(0x8d,0xff);
	IILI9881_COMMAND(0x8e,0xff);

	IILI9881_COMMAND(0x8f,0x00);
	IILI9881_COMMAND(0x90,0x00);

	IILI9881_COMMAND(0x9d,0x00);
	IILI9881_COMMAND(0x9e,0x00);

	IILI9881_COMMAND(0xa0,0x35);
	IILI9881_COMMAND(0xa1,0x00);
	IILI9881_COMMAND(0xa2,0x00);
	IILI9881_COMMAND(0xa3,0x00);
	IILI9881_COMMAND(0xa4,0x00);
	IILI9881_COMMAND(0xa5,0x00);
	IILI9881_COMMAND(0xa6,0x08);
	IILI9881_COMMAND(0xa7,0x00);
	IILI9881_COMMAND(0xa8,0x00);
	IILI9881_COMMAND(0xa9,0x00);
	IILI9881_COMMAND(0xaa,0x00);
	IILI9881_COMMAND(0xab,0x00);
	IILI9881_COMMAND(0xac,0x00);
	IILI9881_COMMAND(0xad,0x00);
	IILI9881_COMMAND(0xae,0xff);
	IILI9881_COMMAND(0xaf,0x00);
	IILI9881_COMMAND(0xb0,0x00);

	ILI9881_PAGE(0x02);
	IILI9881_COMMAND(0x08,0x11);
	IILI9881_COMMAND(0x0a,0x0c);
	IILI9881_COMMAND(0x0f,0x06);
	IILI9881_COMMAND(0xA0,0x00,0x26,0x35,0x16,0x19,0x2C,0x1F,0x1F,0x96,0x1C,0x28,0x80,0x1A,0x18,0x4C,0x21,0x27,0x55,0x65,0x39);
	IILI9881_COMMAND(0xC0,0x00,0x26,0x35,0x16,0x19,0x2C,0x1F,0x1F,0x96,0x1C,0x28,0x80,0x1A,0x18,0x4C,0x21,0x27,0x55,0x65,0x39);

	//===== GIP code finish =====//
	IILI9881_COMMAND(0x4C,0xA4); // PS_EN on ,0x default :A4
	IILI9881_COMMAND(0x18,0xF4); // SH on ,0x default E4 

	//=========================//
	ILI9881_PAGE(0x04);
	IILI9881_COMMAND(0x5D,0xAF); // VREG1 5.5V 
	IILI9881_COMMAND(0x5E,0xAF); // VREG2 5.5V
	IILI9881_COMMAND(0x60,0x9B); // VCM1 
	IILI9881_COMMAND(0x62,0x9B); // VCM2 
	IILI9881_COMMAND(0x82,0x38); // VREF_VGH_MOD_CLPSEL 16V 
	IILI9881_COMMAND(0x84,0x38); // VREF_VGH_DC 16V     
	IILI9881_COMMAND(0x86,0x18); // VREF_VGL_CLPSEL -10V       
	IILI9881_COMMAND(0x66,0xC4); // VGH_AC x4 ,0xdefault :04
	IILI9881_COMMAND(0xC1,0xF0); // VGH_DC x4 ,0xdefault :70
	IILI9881_COMMAND(0x70,0x60);
	IILI9881_COMMAND(0x71,0x00);

	//=========================//
	IILI9881_COMMAND(0x5B,0x33); // vcore_sel Voltage
	IILI9881_COMMAND(0x6C,0x10); // vcore bias L
	IILI9881_COMMAND(0x77,0x03); // vcore_sel Voltage
	IILI9881_COMMAND(0x7B,0x02); // vcore bias R

	//=========================//
	ILI9881_PAGE(0x01);
	IILI9881_COMMAND(0xF0,0x00); // 1280 Gate NL
	IILI9881_COMMAND(0xF1,0xC8); // 1280 Gate NL

	ILI9881_PAGE(0x05);
	IILI9881_COMMAND(0x22,0x3A); // RGB to BGR

	ILI9881_PAGE(0x00);     
	IILI9881_COMMAND(0x35,0x00);           

	IILI9881_COMMAND(0x11);
	msleep(120);
	IILI9881_COMMAND(0x29);

	return 0;
}

static int ili9881d_unprepare(struct drm_panel *panel)
{
	struct mipi_dsi_device *dsi = ili9881d_dsi;

	mipi_dsi_dcs_enter_sleep_mode(dsi);
	return 0;
}

static const struct drm_panel_funcs ili9881d_funcs = {
	.get_modes = ili9881d_get_modes,
	.prepare = ili9881d_prepare,
	.unprepare	= ili9881d_unprepare,
};

static void ili9881d_set_dsi(struct mipi_dsi_device *dsi)
{
	ili9881d_dsi = dsi;
}

const struct panel_data ili9881d_data = {
	.set_dsi = ili9881d_set_dsi,
	.funcs = &ili9881d_funcs,
};
