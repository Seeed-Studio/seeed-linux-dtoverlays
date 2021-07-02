// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX415 cameras.
 * Copyright (C) 2021, Seeed Studio Co., Ltd
 *
 * Based on Sony imx290 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 * 
 * Based on Sony imx415 camera driver 
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 */

#define DEBUG

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <asm/unaligned.h>

#define IMX415_REG_CHIP_ID 0x311A
#define IMX415_CHIP_ID 0xE0

#define IMX415_REG_STANDBY 0x3000
#define IMX415_MODE_SW_STANDBY 0x1
#define IMX415_MODE_STREAMING 0x0

#define IMX415_REG_XMSTA 0x3002
#define IMX415_XMSTA_START 0x0
#define IMX415_XMSTA_STOP 0x1

#define IMX415_REG_ADBIT 0x3031
#define IMX415_REG_ADBIT_10 0x00
#define IMX415_REG_ADBIT_12 0x01
#define IMX415_REG_MDBIT 0x3032
#define IMX415_REG_MDBIT_10 0x00
#define IMX415_REG_MDBIT_12 0x01

#define IMX415_REG_LANE_MODE 0x4001

#define IMX415_REG_HOLD 0x3001

#define IMX415_LF_GAIN_REG_H 0x3091
#define IMX415_LF_GAIN_REG_L 0x3090

#define IMX415_SF1_GAIN_REG_H 0x3093
#define IMX415_SF1_GAIN_REG_L 0x3092

#define IMX415_LF_EXPO_REG_H 0x3052
#define IMX415_LF_EXPO_REG_M 0x3051
#define IMX415_LF_EXPO_REG_L 0x3050

#define IMX415_SF1_EXPO_REG_H 0x3056
#define IMX415_SF1_EXPO_REG_M 0x3055
#define IMX415_SF1_EXPO_REG_L 0x3054

#define IMX415_RHS1_REG_H 0x3062
#define IMX415_RHS1_REG_M 0x3061
#define IMX415_RHS1_REG_L 0x3060
#define IMX415_RHS1_DEFAULT 0x004D

#define IMX415_EXPOSURE_MIN 8
#define IMX415_EXPOSURE_STEP 1
#define IMX415_VTS_MAX 0x7fff

#define IMX415_GAIN_MIN 0x00
#define IMX415_GAIN_MAX 0xf0
#define IMX415_GAIN_STEP 1
#define IMX415_GAIN_DEFAULT 0x00

#define IMX415_FETCH_GAIN_H(VAL) (((VAL) >> 8) & 0x07)
#define IMX415_FETCH_GAIN_L(VAL) ((VAL)&0xFF)

#define IMX415_FETCH_EXP_H(VAL) (((VAL) >> 16) & 0x0F)
#define IMX415_FETCH_EXP_M(VAL) (((VAL) >> 8) & 0xFF)
#define IMX415_FETCH_EXP_L(VAL) ((VAL)&0xFF)

#define IMX415_FETCH_RHS1_H(VAL) (((VAL) >> 16) & 0x0F)
#define IMX415_FETCH_RHS1_M(VAL) (((VAL) >> 8) & 0xFF)
#define IMX415_FETCH_RHS1_L(VAL) ((VAL)&0xFF)

#define IMX415_FETCH_VTS_H(VAL) (((VAL) >> 16) & 0x0F)
#define IMX415_FETCH_VTS_M(VAL) (((VAL) >> 8) & 0xFF)
#define IMX415_FETCH_VTS_L(VAL) ((VAL)&0xFF)

#define IMX415_VTS_REG_L 0x3024
#define IMX415_VTS_REG_M 0x3025
#define IMX415_VTS_REG_H 0x3026

#define IMX415_HTS_REG_L 0x3028
#define IMX415_HTS_REG_H 0x3029

#define IMX415_MIRROR_BIT_MASK BIT(0)
#define IMX415_FLIP_BIT_MASK BIT(1)
#define IMX415_FLIP_REG 0x3030

#define REG_NULL 0xFFFF

#define IMX415_REG_VALUE_08BIT 1
#define IMX415_REG_VALUE_16BIT 2
#define IMX415_REG_VALUE_24BIT 3

#define IMX415_GROUP_HOLD_REG 0x3001
#define IMX415_GROUP_HOLD_START 0x01
#define IMX415_GROUP_HOLD_END 0x00

#define IMX415_NATIVE_WIDTH 3864U
#define IMX415_NATIVE_HEIGHT 2192U
#define IMX415_PIXEL_ARRAY_LEFT 0U
#define IMX415_PIXEL_ARRAY_TOP 0U
#define IMX415_PIXEL_ARRAY_WIDTH 3864U
#define IMX415_PIXEL_ARRAY_HEIGHT 2192U

#define IMX415_DEFAULT_LINK_FREQ 720000000

static const char *const imx415_supply_name[] = {
	"vdda",
	"vddd",
	"vdddo",
};

#define IMX415_NUM_SUPPLIES ARRAY_SIZE(imx415_supply_name)
#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)

struct imx415_regval
{
	u16 reg;
	u8 val;
};

struct imx415_mode
{
	u32 bus_fmt;
	u32 bpp;

	u32 width;
	u32 height;

	u32 hts_def[2];
	u32 vts_def;
	u32 exp_def;

	const struct imx415_regval *config[2];
	const struct imx415_regval *inck;
};

struct imx415
{
	struct i2c_client *client;
	struct clk *xclk;
	u32 xclk_freq;

	u8 nlanes;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt current_format;
	const struct imx415_mode *cur_mode;

	bool streaming;
	u32 cfg_num;
	u32 cur_vts;
	u32 cur_hts;

	struct regulator_bulk_data supplies[IMX415_NUM_SUPPLIES];
	struct gpio_desc *rst_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *anal_a_gain;

	struct mutex lock;
};

static const struct regmap_config imx415_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static __maybe_unused const struct imx415_regval imx415_global_init_settings[] = {
	{0x30C1, 0x00},
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x3451, 0x02},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3701, 0x00},
	{0x3720, 0x00},
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x39A4, 0x07},
	{0x39A8, 0x32},
	{0x39AA, 0x32},
	{0x39AC, 0x32},
	{0x39AE, 0x32},
	{0x39B0, 0x32},
	{0x39B2, 0x2F},
	{0x39B4, 0x2D},
	{0x39B6, 0x28},
	{0x39B8, 0x30},
	{0x39BA, 0x30},
	{0x39BC, 0x30},
	{0x39BE, 0x30},
	{0x39C0, 0x30},
	{0x39C2, 0x2E},
	{0x39C4, 0x2B},
	{0x39C6, 0x25},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{REG_NULL, 0x00}};

static __maybe_unused const struct imx415_regval imx415_inck_24m_freq_720m_clock[] = {
	{0x3008, 0x54},
	{0x300A, 0x3B},
	{0x3034, 0x09},
	{0x3115, 0x00},
	{0x3116, 0x23},
	{0x3118, 0xB4},
	{0x311A, 0xFC},
	{0x311E, 0x23},
	{0x4004, 0x00},
	{0x4005, 0x06},
	{0x400C, 0x00},
	{0x4074, 0x01},
	{REG_NULL, 0x00}};

static __maybe_unused const struct imx415_regval imx415_inck_72m_freq_720m_clock[] = {
	{0x3008, 0xF8},
	{0x300A, 0xB0},
	{0x3034, 0x09},
	{0x3115, 0x00},
	{0x3116, 0x28},
	{0x3118, 0xA0},
	{0x311A, 0xE0},
	{0x311E, 0x28},
	{0x4004, 0x00},
	{0x4005, 0x12},
	{0x400C, 0x00},
	{0x4074, 0x01},
	{REG_NULL, 0x00}};

static __maybe_unused const struct imx415_regval imx415_inck_27m_freq_891m_clock[] = {
	{0x3008, 0x5D},
	{0x300A, 0x42},
	{0x3034, 0x05},
	{0x3115, 0x00},
	{0x3116, 0x23},
	{0x3118, 0xC6},
	{0x311A, 0xE7},
	{0x311E, 0x23},
	{0x4004, 0xC0},
	{0x4005, 0x06},
	{0x400C, 0x00},
	{0x4074, 0x01},
	{REG_NULL, 0x00}};

static __maybe_unused const struct imx415_regval imx415_inck_37_125m_freq_891m_clock[] = {
	{0x3008, 0x7F},
	{0x300A, 0x5B},
	{0x3034, 0x05},
	{0x3115, 0x00},
	{0x3116, 0x24},
	{0x3118, 0xC0},
	{0x311A, 0xE0},
	{0x311E, 0x24},
	{0x4004, 0x48},
	{0x4005, 0x09},
	{0x400C, 0x00},
	{0x4074, 0x01},
	{REG_NULL, 0x00}};

static __maybe_unused const struct imx415_regval imx415_inck_74_25m_freq_891m_clock[] = {
	{0x3008, 0xFF},
	{0x300A, 0xB6},
	{0x3034, 0x05},
	{0x3115, 0x00},
	{0x3116, 0x28},
	{0x3118, 0xC0},
	{0x311A, 0xE0},
	{0x311E, 0x28},
	{0x4004, 0x90},
	{0x4005, 0x12},
	{0x400C, 0x00},
	{0x4074, 0x01},
	{REG_NULL, 0x00}};

static __maybe_unused const struct imx415_regval imx415_2lane_freq_720m_config[] = {
	{0x3002, 0x00},
	{0x301C, 0x00},
	{0x3022, 0x00},
	{0x3028, 0xF0},
	{0x3029, 0x07},
	{0x3033, 0x09},
	{0x4018, 0x6F},
	{0x401A, 0x2F},
	{0x401C, 0x2F},
	{0x401E, 0xBF},
	{0x401F, 0x00},
	{0x4020, 0x2F},
	{0x4022, 0x57},
	{0x4024, 0x2F},
	{0x4026, 0x4F},
	{0x4028, 0x27},
	{REG_NULL, 0x00}};

static __maybe_unused const struct imx415_regval imx415_2lane_freq_891m_config[] = {
	{0x3002, 0x00},
	{0x301C, 0x00},
	{0x3022, 0x00},
	{0x3028, 0x98},
	{0x3029, 0x09},
	{0x3033, 0x05},
	{0x4018, 0x7F},
	{0x401A, 0x37},
	{0x401C, 0x37},
	{0x401E, 0xF7},
	{0x401F, 0x00},
	{0x4020, 0x3F},
	{0x4022, 0x6F},
	{0x4024, 0x3F},
	{0x4026, 0x5F},
	{0x4028, 0x2F},
	{REG_NULL, 0x00}};

static __maybe_unused const struct imx415_regval imx415_4lane_freq_720m_config[] = {
	{0x3002, 0x00},
	{0x301C, 0x00},
	{0x3022, 0x00},
	{0x3028, 0x2A},
	{0x3029, 0x04},
	{0x3033, 0x09},
	{0x4018, 0x6F},
	{0x401A, 0x2F},
	{0x401C, 0x2F},
	{0x401E, 0xBF},
	{0x401F, 0x00},
	{0x4020, 0x2F},
	{0x4022, 0x57},
	{0x4024, 0x2F},
	{0x4026, 0x4F},
	{0x4028, 0x27},
	{REG_NULL, 0x00}};

static __maybe_unused const struct imx415_regval imx415_4lane_freq_891m_config[] = {
	{0x3002, 0x00},
	{0x301C, 0x00},
	{0x3022, 0x00},
	{0x3028, 0x4C},
	{0x3029, 0x04},
	{0x3033, 0x05},
	{0x4018, 0x7F},
	{0x401A, 0x37},
	{0x401C, 0x37},
	{0x401E, 0xF7},
	{0x401F, 0x00},
	{0x4020, 0x3F},
	{0x4022, 0x6F},
	{0x4024, 0x3F},
	{0x4026, 0x5F},
	{0x4028, 0x2F},
	{REG_NULL, 0x00}};

/* Mode configs */
static const struct imx415_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.bpp = 10,
		.width = IMX415_PIXEL_ARRAY_WIDTH,
		.height = IMX415_PIXEL_ARRAY_HEIGHT,
		.exp_def = 0x00066,
		.hts_def = {
			0x07f0,
			0x010, //fake : somethine wrong with libcamera
		},
		.vts_def = 0x08ca,
		.inck = imx415_inck_24m_freq_720m_clock,
		.config = {
			imx415_2lane_freq_720m_config,
			imx415_4lane_freq_720m_config,
		}
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG12_1X12,
		.bpp = 12,
		.width = IMX415_PIXEL_ARRAY_WIDTH,
		.height = IMX415_PIXEL_ARRAY_HEIGHT,
		.exp_def = 0x00066,
			.hts_def = {
			0x0898,
			0x044c,
		},
		.vts_def = 0x08ca,
		.inck = imx415_inck_24m_freq_720m_clock,
		.config = {
			imx415_2lane_freq_720m_config,
			imx415_4lane_freq_720m_config,
		}
	},
};

static inline struct imx415 *to_imx415(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx415, sd);
}

/* Read registers up to 4 at a time */
static int imx415_read_reg(struct i2c_client *client, u16 reg, unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

/* Write registers up to 4 at a time */
static int imx415_write_reg(struct i2c_client *client, u16 reg, u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int imx415_write_array(struct i2c_client *client, const struct imx415_regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].reg != REG_NULL; i++)
	{
		ret = imx415_write_reg(client, regs[i].reg, IMX415_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

static int imx415_get_reso_dist(const struct imx415_mode *mode, struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) + abs(mode->height - framefmt->height);
}

static const struct imx415_mode *imx415_find_best_fit(struct imx415 *imx415, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < imx415->cfg_num; i++)
	{
		dist = imx415_get_reso_dist(&supported_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) && supported_modes[i].bus_fmt == framefmt->code)
		{
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int imx415_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx415 *imx415 = to_imx415(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = imx415->cur_mode->bus_fmt;

	return 0;
}

static int imx415_enum_frame_sizes(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx415 *imx415 = to_imx415(sd);

	if (fse->index >= imx415->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int imx415_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *fmt)
{
	struct imx415 *imx415 = to_imx415(sd);
	const struct imx415_mode *mode;
	struct v4l2_mbus_framefmt *format;

	mutex_lock(&imx415->lock);

	mode = imx415_find_best_fit(imx415, fmt);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;

	fmt->format.code = imx415->cur_mode->bus_fmt;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
	fmt->format.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->format.colorspace);
	fmt->format.quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->format.colorspace, fmt->format.ycbcr_enc);
	fmt->format.xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->format.colorspace);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		format = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	}
	else
	{
		format = &imx415->current_format;
		imx415->cur_mode = mode;
		imx415->cur_vts = imx415->cur_mode->vts_def;
		if(imx415->nlanes == 2){
			imx415->cur_hts = imx415->cur_mode->hts_def[0];
		}else{
			imx415->cur_hts = imx415->cur_mode->hts_def[1];
		}

	}

	*format = fmt->format;

	mutex_unlock(&imx415->lock);

	return 0;
}

static int imx415_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *fmt)
{
	struct imx415 *imx415 = to_imx415(sd);
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&imx415->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		framefmt = v4l2_subdev_get_try_format(&imx415->sd, cfg, fmt->pad);
	else
		framefmt = &imx415->current_format;

	fmt->format = *framefmt;

	mutex_unlock(&imx415->lock);

	return 0;
}

static u64 imx415_calc_pixel_rate(struct imx415 *imx415)
{
	s64 link_freq = IMX415_DEFAULT_LINK_FREQ;
	u8 nlanes = imx415->nlanes;
	u64 pixel_rate;

	/* pixel rate = link_freq * 2 * nr_of_lanes / bits_per_sample */
	pixel_rate = link_freq * 2 * nlanes;
	do_div(pixel_rate, imx415->cur_mode->bpp);
	dev_dbg(&imx415->client->dev, "pixel_rate: %llu\n\r", pixel_rate);

	return pixel_rate;
}

static void imx415_set_default_format(struct imx415 *imx415)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &imx415->current_format;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->colorspace, fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int imx415_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_selection *sel)
{
	switch (sel->target)
	{

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX415_NATIVE_WIDTH;
		sel->r.height = IMX415_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = IMX415_PIXEL_ARRAY_TOP;
		sel->r.left = IMX415_PIXEL_ARRAY_LEFT;
		sel->r.width = IMX415_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX415_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int imx415_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx415 *imx415 = to_imx415(sd);
	struct v4l2_mbus_framefmt *try_fmt = v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct imx415_mode *def_mode = &supported_modes[0];

	mutex_lock(&imx415->lock);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx415->lock);
	/* No crop or compose */

	return 0;
}

static int imx415_get_regulators(struct device *dev, struct imx415 *imx415)
{
	unsigned int i;

	for (i = 0; i < IMX415_NUM_SUPPLIES; i++)
		imx415->supplies[i].supply = imx415_supply_name[i];

	return devm_regulator_bulk_get(dev, IMX415_NUM_SUPPLIES, imx415->supplies);
}

static int imx415_set_data_lanes(struct imx415 *imx415)
{
	int ret = 0, laneval, relaneval;

	switch (imx415->nlanes)
	{
	case 2:
		laneval = 0x01;
		break;
	case 4:
		laneval = 0x03;
		break;
	default:
		/*
		 * We should never hit this since the data lane count is
		 * validated in probe itself
		 */
		dev_err(&imx415->client->dev, "Lane configuration not supported\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = imx415_write_reg(imx415->client, IMX415_REG_LANE_MODE, IMX415_REG_VALUE_08BIT, laneval);
	if (ret)
	{
		dev_err(&imx415->client->dev, "Error setting Lane mode register\n");
		goto exit;
	}

	ret = imx415_read_reg(imx415->client, IMX415_REG_LANE_MODE, IMX415_REG_VALUE_08BIT, &relaneval);
	if (ret || (relaneval != laneval))
	{
		dev_err(&imx415->client->dev, "Error setting Lane mode register : except %x != %x\n", laneval, relaneval);
	}
	else
	{
		dev_dbg(&imx415->client->dev, "Success setting Lane mode register : except %x = %x\n", laneval, relaneval);
	}

exit:
	return ret;
}

/* Verify chip ID */
static int imx415_identify_module(struct imx415 *imx415)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx415->sd);
	int ret;
	u32 val;

	ret = imx415_read_reg(client, IMX415_REG_CHIP_ID, IMX415_REG_VALUE_08BIT, &val);
	if (ret)
	{
		dev_err(&client->dev, "failed to read chip id %x\n", IMX415_CHIP_ID);
		return ret;
	}

	if (val != IMX415_CHIP_ID)
	{
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n", IMX415_CHIP_ID, val);
		return -EIO;
	}

	dev_dbg(&client->dev, "chip id match: %d\n", IMX415_CHIP_ID);

	return 0;
}

static int __imx415_start_stream(struct imx415 *imx415)
{
	int ret;

	ret = imx415_write_array(imx415->client, imx415_global_init_settings);
	if (ret)
		return ret;

	ret = imx415_write_array(imx415->client, imx415->cur_mode->inck);
	if (ret)
		return ret;

	if(imx415->nlanes == 2){
		ret = imx415_write_array(imx415->client, imx415->cur_mode->config[0]);
	}else{
		ret = imx415_write_array(imx415->client, imx415->cur_mode->config[1]);
	}

	if (ret)
		return ret;

	if (imx415->cur_mode->bpp == 10)
	{
		ret = imx415_write_reg(imx415->client, IMX415_REG_ADBIT, IMX415_REG_VALUE_08BIT, IMX415_REG_ADBIT_10);
		ret = imx415_write_reg(imx415->client, IMX415_REG_MDBIT, IMX415_REG_VALUE_08BIT, IMX415_REG_MDBIT_10);
	}
	else
	{
		ret = imx415_write_reg(imx415->client, IMX415_REG_ADBIT, IMX415_REG_VALUE_08BIT, IMX415_REG_ADBIT_12);
		ret = imx415_write_reg(imx415->client, IMX415_REG_MDBIT, IMX415_REG_VALUE_08BIT, IMX415_REG_MDBIT_12);
	}

	return imx415_write_reg(imx415->client, IMX415_REG_STANDBY, IMX415_REG_VALUE_08BIT, 0);
}

static int __imx415_stop_stream(struct imx415 *imx415)
{
	return imx415_write_reg(imx415->client, IMX415_REG_STANDBY, IMX415_REG_VALUE_08BIT, 1);
}

static int imx415_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx415 *imx415 = to_imx415(sd);
	struct i2c_client *client = imx415->client;
	int ret = 0;

	dev_dbg(&imx415->client->dev, "s_stream: %d. %dx%d, bpp: %d\n", on, imx415->cur_mode->width, imx415->cur_mode->height, imx415->cur_mode->bpp);

	mutex_lock(&imx415->lock);
	on = !!on;
	if (on == imx415->streaming)
		goto unlock_and_return;

	if (on)
	{
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0)
		{
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __imx415_start_stream(imx415);
		if (ret)
		{
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	}
	else
	{
		__imx415_stop_stream(imx415);
		pm_runtime_put(&client->dev);
	}

	imx415->streaming = on;

unlock_and_return:
	mutex_unlock(&imx415->lock);

	return ret;
}

static int imx415_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx415 *imx415 = to_imx415(sd);
	int ret;

	ret = clk_prepare_enable(imx415->xclk);
	if (ret)
	{
		dev_err(&client->dev, "Failed to enable clock\n");
		return ret;
	}

	ret = regulator_bulk_enable(IMX415_NUM_SUPPLIES, imx415->supplies);
	if (ret)
	{
		dev_err(&client->dev, "Failed to enable regulators\n");
		clk_disable_unprepare(imx415->xclk);
		return ret;
	}

	usleep_range(1, 2);
	gpiod_set_value_cansleep(imx415->rst_gpio, 1);
	dev_dbg(&client->dev, "pwoer on");
	usleep_range(30000, 31000);

	/* Set data lane count */
	ret = imx415_set_data_lanes(imx415);
	if (ret)
	{
		return ret;
	}

	return 0;
}

static int imx415_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx415 *imx415 = to_imx415(sd);

	clk_disable_unprepare(imx415->xclk);
	gpiod_set_value_cansleep(imx415->rst_gpio, 0);
	regulator_bulk_disable(IMX415_NUM_SUPPLIES, imx415->supplies);
	dev_dbg(&client->dev, "pwoer off");
	return 0;
}

static int imx415_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx415 *imx415 = container_of(ctrl->handler, struct imx415, ctrls);
	struct i2c_client *client = imx415->client;
	s64 max;
	u32 vts = 0, val;
	int ret = 0;
	u32 shr0 = 0;

	/* Propagate change of current control to all related controls */
    	switch (ctrl->id)
	{
		case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx415->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(imx415->exposure, imx415->exposure->minimum, max, imx415->exposure->step, imx415->exposure->default_value);
  		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id)
	{
	case V4L2_CID_EXPOSURE:
		shr0 = imx415->cur_vts - ctrl->val;
		ret = imx415_write_reg(imx415->client, IMX415_LF_EXPO_REG_L, IMX415_REG_VALUE_08BIT, IMX415_FETCH_EXP_L(shr0));
		ret |= imx415_write_reg(imx415->client, IMX415_LF_EXPO_REG_M, IMX415_REG_VALUE_08BIT, IMX415_FETCH_EXP_M(shr0));
		ret |= imx415_write_reg(imx415->client, IMX415_LF_EXPO_REG_H, IMX415_REG_VALUE_08BIT, IMX415_FETCH_EXP_H(shr0));
		dev_dbg(&imx415->client->dev, "set exposure(shr0)  = %d\n", shr0);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx415_write_reg(imx415->client, IMX415_LF_GAIN_REG_H, IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_H(ctrl->val));
		ret |= imx415_write_reg(imx415->client, IMX415_LF_GAIN_REG_L, IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_L(ctrl->val));
		dev_dbg(&client->dev, "set analog gain 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + imx415->cur_mode->height;
		imx415->cur_vts = vts;

		ret = imx415_write_reg(imx415->client, IMX415_VTS_REG_L, IMX415_REG_VALUE_08BIT, IMX415_FETCH_VTS_L(vts));
		ret |= imx415_write_reg(imx415->client, IMX415_VTS_REG_M, IMX415_REG_VALUE_08BIT, IMX415_FETCH_VTS_M(vts));
		ret |= imx415_write_reg(imx415->client, IMX415_VTS_REG_H, IMX415_REG_VALUE_08BIT, IMX415_FETCH_VTS_H(vts));
		dev_dbg(&imx415->client->dev, "set vblank %d = height(%d) + val(%d)\n", vts, imx415->cur_mode->height, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = imx415_read_reg(imx415->client, IMX415_FLIP_REG, IMX415_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= IMX415_MIRROR_BIT_MASK;
		else
			val &= ~IMX415_MIRROR_BIT_MASK;
		ret = imx415_write_reg(imx415->client, IMX415_FLIP_REG, IMX415_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = imx415_read_reg(imx415->client, IMX415_FLIP_REG, IMX415_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= IMX415_FLIP_BIT_MASK;
		else
			val &= ~IMX415_FLIP_BIT_MASK;
		ret = imx415_write_reg(imx415->client, IMX415_FLIP_REG, IMX415_REG_VALUE_08BIT, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n", __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx415_ctrl_ops = {
	.s_ctrl = imx415_set_ctrl,
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx415_internal_ops = {
	.open = imx415_open,
};
#endif

static const struct dev_pm_ops imx415_pm_ops = {
	SET_RUNTIME_PM_OPS(imx415_power_off, imx415_power_on, NULL)};

static const struct v4l2_subdev_video_ops imx415_video_ops = {
	.s_stream = imx415_s_stream,
};

static const struct v4l2_subdev_pad_ops imx415_pad_ops = {
	.enum_mbus_code = imx415_enum_mbus_code,
	.enum_frame_size = imx415_enum_frame_sizes,
	.get_fmt = imx415_get_fmt,
	.set_fmt = imx415_set_fmt,
	.get_selection = imx415_get_selection,
};

static const struct v4l2_subdev_ops imx415_subdev_ops = {
	.video = &imx415_video_ops,
	.pad = &imx415_pad_ops,
};

static const struct media_entity_operations imx415_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx415_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	/* Only CSI2 is supported for now: */
	struct v4l2_fwnode_endpoint ep = {.bus_type = V4L2_MBUS_CSI2_DPHY};
	const struct imx415_mode *mode;
	struct imx415 *imx415;
	u32 address = 0;
	u32 h_blank, v_blank, exposure_max;
	int ret;

	imx415 = devm_kzalloc(dev, sizeof(*imx415), GFP_KERNEL);
	if (!imx415)
		return -ENOMEM;

	imx415->client = client;
	imx415->cfg_num = ARRAY_SIZE(supported_modes);

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
	{
		dev_err(dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret == -ENXIO)
	{
		dev_err(dev, "Unsupported bus type, should be CSI2\n");
		goto err_free_err;
	}
	else if (ret)
	{
		dev_err(dev, "Parsing endpoint node failed\n");
		goto err_free_err;
	}

	/* Get number of data lanes */
	imx415->nlanes = ep.bus.mipi_csi2.num_data_lanes;
	if (imx415->nlanes != 2 && imx415->nlanes != 4)
	{
		dev_err(dev, "Invalid data lanes: %d\n", imx415->nlanes);
		ret = -EINVAL;
		goto err_free_err;
	}

	dev_dbg(dev, "Using %u data lanes\n", imx415->nlanes);

	if (!ep.nr_of_link_frequencies)
	{
		dev_err(dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto err_free_err;
	}

	/* get system clock (xclk) */
	imx415->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(imx415->xclk))
	{
		dev_err(dev, "Could not get xclk");
		ret = PTR_ERR(imx415->xclk);
		goto err_free_err;
	}

	fwnode_property_read_u32(dev_fwnode(dev), "reg", &address);

	dev_dbg(dev, "imx415 address锛?%d\n", address);

	ret = fwnode_property_read_u32(dev_fwnode(dev), "clock-frequency", &imx415->xclk_freq);
	if (ret)
	{
		dev_err(dev, "Could not get xclk frequency\n");
		goto err_free_err;
	}

	/* external clock must be 24 MHz */
	if (imx415->xclk_freq != 24000000)
	{
		dev_err(dev, "External clock frequency %u is not supported\n", imx415->xclk_freq);
		ret = -EINVAL;
		goto err_free_err;
	}

	ret = clk_set_rate(imx415->xclk, imx415->xclk_freq);
	if (ret)
	{
		dev_err(dev, "Could not set xclk frequency\n");
		goto err_free_err;
	}

	ret = imx415_get_regulators(dev, imx415);
	if (ret < 0)
	{
		dev_err(dev, "Cannot get regulators\n");
		goto err_free_err;
	}

	imx415->rst_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(imx415->rst_gpio))
	{
		dev_err(dev, "Cannot get reset gpio\n");
		ret = PTR_ERR(imx415->rst_gpio);
		goto err_free_err;
	}

	mutex_init(&imx415->lock);

	imx415->ctrls.lock = &imx415->lock;

	/*
	 * Initialize the frame format. In particular, imx415->cur_mode 
	 * and imx415->bpp are set to defaults: imx415_calc_pixel_rate() call
	 * below relies on these fields.
	 */
	imx415->cur_mode = &supported_modes[0];
	mode = imx415->cur_mode;
	dev_dbg(dev, "curent mode: %d\n", imx415->cur_mode->width), v4l2_ctrl_handler_init(&imx415->ctrls, 7);

	if (imx415->link_freq)
		imx415->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx415->anal_a_gain = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops, V4L2_CID_ANALOGUE_GAIN, IMX415_GAIN_MIN, IMX415_GAIN_MAX, IMX415_GAIN_STEP, IMX415_GAIN_DEFAULT);
	if(imx415->nlanes == 2){
		imx415->cur_hts = imx415->cur_mode->hts_def[0];
	}else{
		imx415->cur_hts = imx415->cur_mode->hts_def[1];
	}
	h_blank = imx415->cur_hts - mode->width;
	dev_dbg(dev, "probe mode->hts_def: %u, mode->width: %u h_blank: %u\n\r", imx415->cur_hts, mode->width, h_blank);
	imx415->hblank = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops, V4L2_CID_HBLANK, h_blank, h_blank, 1, h_blank);
	if (imx415->hblank)
		imx415->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v_blank = mode->vts_def - mode->height;

	imx415->vblank = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops, V4L2_CID_VBLANK, v_blank, IMX415_VTS_MAX - mode->height, 1, v_blank);

	exposure_max = mode->vts_def - 4;
	imx415->exposure = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops, V4L2_CID_EXPOSURE, IMX415_EXPOSURE_MIN, exposure_max, IMX415_EXPOSURE_STEP, mode->exp_def);

	imx415->hflip = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx415->vflip = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	imx415->pixel_rate = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops, V4L2_CID_PIXEL_RATE, 0, imx415_calc_pixel_rate(imx415), 1, imx415_calc_pixel_rate(imx415));

	imx415->sd.ctrl_handler = &imx415->ctrls;

	if (imx415->ctrls.error)
	{
		dev_err(dev, "Control initialization error %d\n", imx415->ctrls.error);
		ret = imx415->ctrls.error;
		goto err_free_ctrl;
	}

	v4l2_i2c_subdev_init(&imx415->sd, client, &imx415_subdev_ops);

	/* Power on the device to match runtime PM state below */
	ret = imx415_power_on(dev);
	if (ret < 0)
	{
		dev_err(dev, "Could not power on the device\n");
		goto err_free_err;
	}

	ret = imx415_identify_module(imx415);
	if (ret)
		goto error_power_off;

	imx415->sd.dev = &imx415->client->dev;
	imx415->sd.entity.ops = &imx415_subdev_entity_ops;
	imx415->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx415->sd.internal_ops = &imx415_internal_ops;
	imx415->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	imx415->pad.flags = MEDIA_PAD_FL_SOURCE;

	imx415_set_default_format(imx415);

	ret = media_entity_pads_init(&imx415->sd.entity, 1, &imx415->pad);
	if (ret < 0)
	{
		dev_err(dev, "Could not register media entity\n");
		goto err_free_ctrl;
	}

	ret = v4l2_async_register_subdev(&imx415->sd);
	if (ret < 0)
	{
		dev_err(dev, "Could not register v4l2 device\n");
		goto err_free_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	v4l2_fwnode_endpoint_free(&ep);

	return 0;

err_free_entity:
	media_entity_cleanup(&imx415->sd.entity);
error_power_off:
	imx415_power_off(dev);
err_free_ctrl:
	v4l2_ctrl_handler_free(&imx415->ctrls);
	mutex_destroy(&imx415->lock);
err_free_err:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

static int imx415_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx415 *imx415 = to_imx415(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	mutex_destroy(&imx415->lock);

	pm_runtime_disable(&imx415->client->dev);
	if (!pm_runtime_status_suspended(&imx415->client->dev))
		imx415_power_off(&imx415->client->dev);
	pm_runtime_set_suspended(&imx415->client->dev);

	return 0;
}

static const struct of_device_id imx415_of_match[] = {
	{.compatible = "sony,imx415"},
	{/* sentinel */}};

MODULE_DEVICE_TABLE(of, imx415_of_match);

static struct i2c_driver imx415_i2c_driver = {
	.probe_new = imx415_probe,
	.remove = imx415_remove,
	.driver = {
		.name = "imx415",
		.pm = &imx415_pm_ops,
		.of_match_table = of_match_ptr(imx415_of_match),
	},
};

module_i2c_driver(imx415_i2c_driver);

MODULE_DESCRIPTION("Sony IMX415 CMOS Image Sensor Driver");
MODULE_AUTHOR("Hongtai.Liu lht856@foxmail.com");
MODULE_LICENSE("GPL v2");
