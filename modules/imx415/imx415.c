// SPDX-License-Identifier: GPL-2.0
/*
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

enum imx415_clk_index
{
	CLK_37_125,
	CLK_74_25,
};

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN V4L2_CID_GAIN
#endif

#define IMX415_REG_CHIP_ID 0x311A
#define IMX415_CHIP_ID 0xE0

#define IMX415_REG_STANDBY 0x3000
#define IMX415_MODE_SW_STANDBY 0x1
#define IMX415_MODE_STREAMING 0x0

#define IMX415_REG_XMSTA 0x3002
#define IMX415_XMSTA_START 0x0
#define IMX415_XMSTA_STOP 0x1

#define IMX415_REG_LANE_MODE 0x4011

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
#define IMX415_PIXEL_ARRAY_LEFT 12U
#define IMX415_PIXEL_ARRAY_TOP 16U
#define IMX415_PIXEL_ARRAY_WIDTH 3840U
#define IMX415_PIXEL_ARRAY_HEIGHT 2160U

#define IMX415_HMAX_MIN_2LANE 8800
#define IMX415_HMAX_MIN_4LANE 4400
#define IMX415_HMAX_MAX 0xffff
#define IMX415_VMAX_MAX 0x7fff

static const char *const imx415_supply_name[] = {
	"vdda",
	"vddd",
	"vdddo",
};

#define IMX415_NUM_SUPPLIES ARRAY_SIZE(imx415_supply_name)

struct imx415_regval
{
	u16 reg;
	u8 val;
};

struct imx415_mode
{
	u32 width;
	u32 height;
	u32 hmax;
	u32 vmax;
	u8 link_freq_index;
	struct v4l2_rect crop;

	const struct imx415_regval *data;
	u32 data_size;

	/* Clock setup can vary. Index as enum imx415_clk_index */
	const struct imx415_regval *clk_data[2];
	u32 clk_size;
};

struct imx415
{
	struct device *dev;
	struct clk *xclk;
	u32 xclk_freq;
	struct regmap *regmap;
	u8 nlanes;
	u8 bpp;
	u16 hmax_min;

	const struct imx415_pixfmt *formats;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt current_format;
	const struct imx415_mode *current_mode;

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

struct imx415_pixfmt
{
	u32 code;
	u8 bpp;
};

#define IMX415_NUM_FORMATS 2

static const struct imx415_pixfmt imx415_colour_formats[IMX415_NUM_FORMATS] = {
	{MEDIA_BUS_FMT_SRGGB10_1X10, 10},
	{MEDIA_BUS_FMT_SRGGB12_1X12, 12},
};

static const struct regmap_config imx415_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static const char *const imx415_test_pattern_menu[] = {
	"Disabled",
	"Sequence Pattern 1",
	"Horizontal Color-bar Chart",
	"Vertical Color-bar Chart",
	"Sequence Pattern 2",
	"Gradation Pattern 1",
	"Gradation Pattern 2",
	"000/555h Toggle Pattern",
};

static const struct imx415_regval imx415_global_init_settings[] = {
	{0x3007, 0x00},
	{0x301a, 0x00},
	{0x303a, 0x0c},
	{0x3040, 0x00},
	{0x3041, 0x00},
	{0x303c, 0x00},
	{0x303d, 0x00},
	{0x3042, 0x9c},
	{0x3043, 0x07},
	{0x303e, 0x49},
	{0x303f, 0x04},
	{0x304b, 0x0a},
	{0x300f, 0x00},
	{0x3010, 0x21},
	{0x3012, 0x64},
	{0x3016, 0x09},
	{0x3070, 0x02},
	{0x3071, 0x11},
	{0x309b, 0x10},
	{0x309c, 0x22},
	{0x30a2, 0x02},
	{0x30a6, 0x20},
	{0x30a8, 0x20},
	{0x30aa, 0x20},
	{0x30ac, 0x20},
	{0x30b0, 0x43},
	{0x3119, 0x9e},
	{0x311c, 0x1e},
	{0x311e, 0x08},
	{0x3128, 0x05},
	{0x313d, 0x83},
	{0x3150, 0x03},
	{0x317e, 0x00},
	{0x32b8, 0x50},
	{0x32b9, 0x10},
	{0x32ba, 0x00},
	{0x32bb, 0x04},
	{0x32c8, 0x50},
	{0x32c9, 0x10},
	{0x32ca, 0x00},
	{0x32cb, 0x04},
	{0x332c, 0xd3},
	{0x332d, 0x10},
	{0x332e, 0x0d},
	{0x3358, 0x06},
	{0x3359, 0xe1},
	{0x335a, 0x11},
	{0x3360, 0x1e},
	{0x3361, 0x61},
	{0x3362, 0x10},
	{0x33b0, 0x50},
	{0x33b2, 0x1a},
	{0x33b3, 0x04},
};

static const struct imx415_regval imx415_37_125mhz_clock_1080p[] = {
	{0x305c, 0x18},
	{0x305d, 0x03},
	{0x305e, 0x20},
	{0x305f, 0x01},
	{0x315e, 0x1a},
	{0x3164, 0x1a},
	{0x3444, 0x20},
	{0x3445, 0x25},
	{0x3480, 0x49},
};

static const struct imx415_regval imx415_74_250mhz_clock_1080p[] = {
	{0x305c, 0x0c},
	{0x305d, 0x03},
	{0x305e, 0x10},
	{0x305f, 0x01},
	{0x315e, 0x1b},
	{0x3164, 0x1b},
	{0x3444, 0x40},
	{0x3445, 0x4a},
	{0x3480, 0x92},
};

static const struct imx415_regval imx415_1080p_settings[] = {
	/* mode settings */
	{0x3007, 0x00},
	{0x303a, 0x0c},
	{0x3414, 0x0a},
	{0x3472, 0x80},
	{0x3473, 0x07},
	{0x3418, 0x38},
	{0x3419, 0x04},
	{0x3012, 0x64},
	{0x3013, 0x00},
	/* data rate settings */
	{0x3405, 0x10},
	{0x3446, 0x57},
	{0x3447, 0x00},
	{0x3448, 0x37},
	{0x3449, 0x00},
	{0x344a, 0x1f},
	{0x344b, 0x00},
	{0x344c, 0x1f},
	{0x344d, 0x00},
	{0x344e, 0x1f},
	{0x344f, 0x00},
	{0x3450, 0x77},
	{0x3451, 0x00},
	{0x3452, 0x1f},
	{0x3453, 0x00},
	{0x3454, 0x17},
	{0x3455, 0x00},
};

static const struct imx415_regval imx415_37_125mhz_clock_720p[] = {
	{0x305c, 0x20},
	{0x305d, 0x00},
	{0x305e, 0x20},
	{0x305f, 0x01},
	{0x315e, 0x1a},
	{0x3164, 0x1a},
	{0x3444, 0x20},
	{0x3445, 0x25},
	{0x3480, 0x49},
};

static const struct imx415_regval imx415_74_250mhz_clock_720p[] = {
	{0x305c, 0x10},
	{0x305d, 0x00},
	{0x305e, 0x10},
	{0x305f, 0x01},
	{0x315e, 0x1b},
	{0x3164, 0x1b},
	{0x3444, 0x40},
	{0x3445, 0x4a},
	{0x3480, 0x92},
};

static const struct imx415_regval imx415_720p_settings[] = {
	/* mode settings */
	{0x3007, 0x10},
	{0x303a, 0x06},
	{0x3414, 0x04},
	{0x3472, 0x00},
	{0x3473, 0x05},
	{0x3418, 0xd0},
	{0x3419, 0x02},
	{0x3012, 0x64},
	{0x3013, 0x00},
	/* data rate settings */
	{0x3405, 0x10},
	{0x3446, 0x4f},
	{0x3447, 0x00},
	{0x3448, 0x2f},
	{0x3449, 0x00},
	{0x344a, 0x17},
	{0x344b, 0x00},
	{0x344c, 0x17},
	{0x344d, 0x00},
	{0x344e, 0x17},
	{0x344f, 0x00},
	{0x3450, 0x57},
	{0x3451, 0x00},
	{0x3452, 0x17},
	{0x3453, 0x00},
	{0x3454, 0x17},
	{0x3455, 0x00},
};

static const struct imx415_regval imx415_10bit_settings[] = {
	{0x3005, 0x00},
	{0x3046, 0x00},
	{0x3129, 0x1d},
	{0x317c, 0x12},
	{0x31ec, 0x37},
	{0x3441, 0x0a},
	{0x3442, 0x0a},
	{0x300a, 0x3c},
	{0x300b, 0x00},
};

static const struct imx415_regval imx415_12bit_settings[] = {
	{0x3005, 0x01},
	{0x3046, 0x01},
	{0x3129, 0x00},
	{0x317c, 0x00},
	{0x31ec, 0x0e},
	{0x3441, 0x0c},
	{0x3442, 0x0c},
	{0x300a, 0xf0},
	{0x300b, 0x00},
};

/* supported link frequencies */
#define FREQ_INDEX_1080P 0
#define FREQ_INDEX_720P 1
static const s64 imx415_link_freq_2lanes[] = {
	[FREQ_INDEX_1080P] = 445500000,
	[FREQ_INDEX_720P] = 297000000,
};
static const s64 imx415_link_freq_4lanes[] = {
	[FREQ_INDEX_1080P] = 222750000,
	[FREQ_INDEX_720P] = 148500000,
};

/*
 * In this function and in the similar ones below We rely on imx415_probe()
 * to ensure that nlanes is either 2 or 4.
 */
static inline const s64 *imx415_link_freqs_ptr(const struct imx415 *imx415)
{
	if (imx415->nlanes == 2)
		return imx415_link_freq_2lanes;
	else
		return imx415_link_freq_4lanes;
}

static inline int imx415_link_freqs_num(const struct imx415 *imx415)
{
	if (imx415->nlanes == 2)
		return ARRAY_SIZE(imx415_link_freq_2lanes);
	else
		return ARRAY_SIZE(imx415_link_freq_4lanes);
}

/* Mode configs */
static const struct imx415_mode imx415_modes_2lanes[] = {
	{
		.width = IMX415_PIXEL_ARRAY_WIDTH,
		.height = IMX415_PIXEL_ARRAY_HEIGHT,
		.hmax = 0x0226 * 4,
		.vmax = 0x08fc * 2,
		.link_freq_index = FREQ_INDEX_1080P,
		.crop = {
			.left = IMX415_PIXEL_ARRAY_LEFT,
			.top = IMX415_PIXEL_ARRAY_TOP,
			.width = IMX415_PIXEL_ARRAY_WIDTH,
			.height = IMX415_PIXEL_ARRAY_HEIGHT,
		},
		.data = imx415_1080p_settings,
		.data_size = ARRAY_SIZE(imx415_1080p_settings),
		.clk_data = {
			[CLK_37_125] = imx415_37_125mhz_clock_1080p,
			[CLK_74_25] = imx415_74_250mhz_clock_1080p,
		},
		.clk_size = ARRAY_SIZE(imx415_37_125mhz_clock_1080p),
	},
};

static const struct imx415_mode imx415_modes_4lanes[] = {
	{
		.width = IMX415_PIXEL_ARRAY_WIDTH,
		.height = IMX415_PIXEL_ARRAY_HEIGHT,
		.hmax = 0x0226 * 4,
		.vmax = 0x08fc * 2,
		.link_freq_index = FREQ_INDEX_1080P,
		.crop = {
			.left = IMX415_PIXEL_ARRAY_LEFT,
			.top = IMX415_PIXEL_ARRAY_TOP,
			.width = IMX415_PIXEL_ARRAY_WIDTH,
			.height = IMX415_PIXEL_ARRAY_HEIGHT,
		},
		.data = imx415_1080p_settings,
		.data_size = ARRAY_SIZE(imx415_1080p_settings),
		.clk_data = {
			[CLK_37_125] = imx415_37_125mhz_clock_720p,
			[CLK_74_25] = imx415_74_250mhz_clock_720p,
		},
		.clk_size = ARRAY_SIZE(imx415_37_125mhz_clock_720p),
	},
};

static inline const struct imx415_mode *imx415_modes_ptr(const struct imx415 *imx415)
{
	if (imx415->nlanes == 2)
		return imx415_modes_2lanes;
	else
		return imx415_modes_4lanes;
}

static inline int imx415_modes_num(const struct imx415 *imx415)
{
	if (imx415->nlanes == 2)
		return ARRAY_SIZE(imx415_modes_2lanes);
	else
		return ARRAY_SIZE(imx415_modes_4lanes);
}

static inline struct imx415 *to_imx415(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx415, sd);
}

static inline int imx415_read_reg(struct imx415 *imx415, u16 addr, u8 *value)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(imx415->regmap, addr, &regval);
	if (ret)
	{
		dev_err(imx415->dev, "I2C read failed for addr: %x\n", addr);
		return ret;
	}

	*value = regval & 0xff;

	return 0;
}

static int imx415_write_reg(struct imx415 *imx415, u16 addr, u8 value)
{
	int ret;

	ret = regmap_write(imx415->regmap, addr, value);
	if (ret)
	{
		dev_err(imx415->dev, "I2C write failed for addr: %x\n", addr);
		return ret;
	}

	return ret;
}

static int imx415_set_register_array(struct imx415 *imx415,
									 const struct imx415_regval *settings,
									 unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings)
	{
		ret = imx415_write_reg(imx415, settings->reg, settings->val);
		if (ret < 0)
			return ret;
	}

	/* Provide 10ms settle time */
	usleep_range(10000, 11000);

	return 0;
}

static int imx415_write_buffered_reg(struct imx415 *imx415, u16 address_low,
									 u8 nr_regs, u32 value)
{
	unsigned int i;
	int ret;

	ret = imx415_write_reg(imx415, IMX415_REG_HOLD, 0x01);
	if (ret)
	{
		dev_err(imx415->dev, "Error setting hold register\n");
		return ret;
	}

	for (i = 0; i < nr_regs; i++)
	{
		ret = imx415_write_reg(imx415, address_low + i,
							   (u8)(value >> (i * 8)));
		if (ret)
		{
			dev_err(imx415->dev, "Error writing buffered registers\n");
			return ret;
		}
	}

	ret = imx415_write_reg(imx415, IMX415_REG_HOLD, 0x00);
	if (ret)
	{
		dev_err(imx415->dev, "Error setting hold register\n");
		return ret;
	}

	return ret;
}

static int imx415_set_gain(struct imx415 *imx415, u32 value)
{
	int ret;

	ret = imx415_write_buffered_reg(imx415, IMX415_LF_GAIN_REG_L, 2, value);

	if (ret)
		dev_err(imx415->dev, "Unable to write gain\n");

	return ret;
}

static int imx415_set_exposure(struct imx415 *imx415, u32 value)
{
	u32 exposure = (imx415->current_mode->height + imx415->vblank->val) - value - 1;
	int ret;

	ret = imx415_write_buffered_reg(imx415, IMX415_LF_EXPO_REG_L, 3, exposure);

	if (ret)
		dev_err(imx415->dev, "Unable to write exposure\n");

	return ret;
}

static int imx415_set_hmax(struct imx415 *imx415, u32 val)
{
	u32 hmax = val + imx415->current_mode->width;
	int ret;

	ret = imx415_write_buffered_reg(imx415, IMX415_HTS_REG_L, 2,
									hmax);
	if (ret)
		dev_err(imx415->dev, "Error setting HMAX register\n");

	return ret;
}

static int imx415_set_vmax(struct imx415 *imx415, u32 val)
{
	u32 vmax = val + imx415->current_mode->height;
	int ret;

	ret = imx415_write_buffered_reg(imx415, IMX415_VTS_REG_L, 3,
									vmax);
	if (ret)
		dev_err(imx415->dev, "Unable to write vmax\n");

	/*
	 * Changing vblank changes the allowed range for exposure.
	 * We don't supply the current exposure as default here as it
	 * may lie outside the new range. We will reset it just below.
	 */
	__v4l2_ctrl_modify_range(imx415->exposure,
							 IMX415_EXPOSURE_MIN,
							 vmax - 2,
							 IMX415_EXPOSURE_STEP,
							 vmax - 2);

	/*
	 * Becuse of the way exposure works for this sensor, updating
	 * vblank causes the effective exposure to change, so we must
	 * set it back to the "new" correct value.
	 */
	imx415_set_exposure(imx415, imx415->exposure->val);

	return ret;
}

/* Stop streaming */
static int imx415_stop_streaming(struct imx415 *imx415)
{
	int ret;

	ret = imx415_write_reg(imx415, IMX415_REG_STANDBY, 0x01);
	if (ret < 0)
		return ret;

	msleep(30);

	return imx415_write_reg(imx415, IMX415_REG_XMSTA, 0x01);
}

static int imx415_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx415 *imx415 = container_of(ctrl->handler,
										 struct imx415, ctrls);
	int ret = 0;
	u8 val;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(imx415->dev))
		return 0;

	switch (ctrl->id)
	{
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx415_set_gain(imx415, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx415_set_exposure(imx415, ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		ret = imx415_set_hmax(imx415, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx415_set_vmax(imx415, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = imx415_read_reg(imx415, IMX415_FLIP_REG, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= IMX415_MIRROR_BIT_MASK;
		else
			val &= ~IMX415_MIRROR_BIT_MASK;
		ret = imx415_write_reg(imx415, IMX415_FLIP_REG, val);
		break;

	case V4L2_CID_VFLIP:
		ret = imx415_read_reg(imx415, IMX415_FLIP_REG, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= IMX415_FLIP_BIT_MASK;
		else
			val &= ~IMX415_FLIP_BIT_MASK;
		ret = imx415_write_reg(imx415, IMX415_FLIP_REG, val);
		break;

	// case V4L2_CID_TEST_PATTERN:
	// 	if (ctrl->val) {
	// 		imx415_write_reg(imx415, IMX415_BLKLEVEL_LOW, 0x00);
	// 		imx415_write_reg(imx415, IMX415_BLKLEVEL_HIGH, 0x00);
	// 		usleep_range(10000, 11000);
	// 		imx415_write_reg(imx415, IMX415_PGCTRL,
	// 				 (u8)(IMX415_PGCTRL_REGEN |
	// 				 IMX415_PGCTRL_THRU |
	// 				 IMX415_PGCTRL_MODE(ctrl->val)));
	// 	} else {
	// 		imx415_write_reg(imx415, IMX415_PGCTRL, 0x00);
	// 		usleep_range(10000, 11000);
	// 		if (imx415->bpp == 10)
	// 			imx415_write_reg(imx415, IMX415_BLKLEVEL_LOW,
	// 					 0x3c);
	// 		else /* 12 bits per pixel */
	// 			imx415_write_reg(imx415, IMX415_BLKLEVEL_LOW,
	// 					 0xf0);
	// 		imx415_write_reg(imx415, IMX415_BLKLEVEL_HIGH, 0x00);
	// 	}
	// 	break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx415->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx415_ctrl_ops = {
	.s_ctrl = imx415_set_ctrl,
};

static int imx415_enum_mbus_code(struct v4l2_subdev *sd,
								 struct v4l2_subdev_pad_config *cfg,
								 struct v4l2_subdev_mbus_code_enum *code)
{
	const struct imx415 *imx415 = to_imx415(sd);

	if (code->index >= IMX415_NUM_FORMATS)
		return -EINVAL;

	code->code = imx415->formats[code->index].code;

	return 0;
}

static int imx415_enum_frame_size(struct v4l2_subdev *sd,
								  struct v4l2_subdev_pad_config *cfg,
								  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct imx415 *imx415 = to_imx415(sd);
	const struct imx415_mode *imx415_modes = imx415_modes_ptr(imx415);

	if (fse->code != imx415->formats[0].code &&
		fse->code != imx415->formats[1].code)
		return -EINVAL;

	if (fse->index >= imx415_modes_num(imx415))
		return -EINVAL;

	fse->min_width = imx415_modes[fse->index].width;
	fse->max_width = imx415_modes[fse->index].width;
	fse->min_height = imx415_modes[fse->index].height;
	fse->max_height = imx415_modes[fse->index].height;

	return 0;
}

static int imx415_get_fmt(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_format *fmt)
{
	struct imx415 *imx415 = to_imx415(sd);
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&imx415->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		framefmt = v4l2_subdev_get_try_format(&imx415->sd, cfg,
											  fmt->pad);
	else
		framefmt = &imx415->current_format;

	fmt->format = *framefmt;

	mutex_unlock(&imx415->lock);

	return 0;
}

static inline u8 imx415_get_link_freq_index(struct imx415 *imx415)
{
	return imx415->current_mode->link_freq_index;
}

static s64 imx415_get_link_freq(struct imx415 *imx415)
{
	u8 index = imx415_get_link_freq_index(imx415);

	return *(imx415_link_freqs_ptr(imx415) + index);
}

static u64 imx415_calc_pixel_rate(struct imx415 *imx415)
{
	s64 link_freq = imx415_get_link_freq(imx415);
	u8 nlanes = imx415->nlanes;
	u64 pixel_rate;

	/* pixel rate = link_freq * 2 * nr_of_lanes / bits_per_sample */
	pixel_rate = link_freq * 2 * nlanes;
	do_div(pixel_rate, imx415->bpp);
	return pixel_rate;
}

static int imx415_set_fmt(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_format *fmt)
{
	struct imx415 *imx415 = to_imx415(sd);
	const struct imx415_mode *mode;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	mutex_lock(&imx415->lock);

	mode = v4l2_find_nearest_size(imx415_modes_ptr(imx415),
								  imx415_modes_num(imx415), width, height,
								  fmt->format.width, fmt->format.height);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;

	for (i = 0; i < IMX415_NUM_FORMATS; i++)
		if (imx415->formats[i].code == fmt->format.code)
			break;

	if (i >= IMX415_NUM_FORMATS)
		i = 0;

	fmt->format.code = imx415->formats[i].code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
	fmt->format.ycbcr_enc =
		V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->format.colorspace);
	fmt->format.quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->format.colorspace,
									  fmt->format.ycbcr_enc);
	fmt->format.xfer_func =
		V4L2_MAP_XFER_FUNC_DEFAULT(fmt->format.colorspace);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		format = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	}
	else
	{
		format = &imx415->current_format;
		imx415->current_mode = mode;
		imx415->bpp = imx415->formats[i].bpp;

		if (imx415->link_freq)
			__v4l2_ctrl_s_ctrl(imx415->link_freq,
							   imx415_get_link_freq_index(imx415));
		if (imx415->pixel_rate)
			__v4l2_ctrl_s_ctrl_int64(imx415->pixel_rate,
									 imx415_calc_pixel_rate(imx415));

		if (imx415->hblank)
		{
			__v4l2_ctrl_modify_range(imx415->hblank,
									 imx415->hmax_min - mode->width,
									 IMX415_HMAX_MAX - mode->width,
									 1, mode->hmax - mode->width);
			__v4l2_ctrl_s_ctrl(imx415->hblank,
							   mode->hmax - mode->width);
		}
		if (imx415->vblank)
		{
			__v4l2_ctrl_modify_range(imx415->vblank,
									 mode->vmax - mode->height,
									 IMX415_VMAX_MAX - mode->height,
									 1,
									 mode->vmax - mode->height);
			__v4l2_ctrl_s_ctrl(imx415->vblank,
							   mode->vmax - mode->height);
		}
		if (imx415->exposure)
			__v4l2_ctrl_modify_range(imx415->exposure,
									 IMX415_EXPOSURE_MIN,
									 mode->vmax - 2,
									 IMX415_EXPOSURE_STEP,
									 mode->vmax - 2);
	}

	*format = fmt->format;

	mutex_unlock(&imx415->lock);

	return 0;
}

static int imx415_entity_init_cfg(struct v4l2_subdev *subdev,
								  struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = {0};

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 3840;
	fmt.format.height = 2160;

	imx415_set_fmt(subdev, cfg, &fmt);

	return 0;
}

static int imx415_write_current_format(struct imx415 *imx415)
{
	int ret;

	switch (imx415->current_format.code)
	{
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_Y10_1X10:
		ret = imx415_set_register_array(imx415, imx415_10bit_settings,
										ARRAY_SIZE(
											imx415_10bit_settings));
		if (ret < 0)
		{
			dev_err(imx415->dev, "Could not set format registers\n");
			return ret;
		}
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_Y12_1X12:
		ret = imx415_set_register_array(imx415, imx415_12bit_settings,
										ARRAY_SIZE(
											imx415_12bit_settings));
		if (ret < 0)
		{
			dev_err(imx415->dev, "Could not set format registers\n");
			return ret;
		}
		break;
	default:
		dev_err(imx415->dev, "Unknown pixel format\n");
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_rect *
__imx415_get_pad_crop(struct imx415 *imx415, struct v4l2_subdev_pad_config *cfg,
					  unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which)
	{
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx415->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx415->current_mode->crop;
	}

	return NULL;
}

static int imx415_get_selection(struct v4l2_subdev *sd,
								struct v4l2_subdev_pad_config *cfg,
								struct v4l2_subdev_selection *sel)
{
	switch (sel->target)
	{
	case V4L2_SEL_TGT_CROP:
	{
		struct imx415 *imx415 = to_imx415(sd);

		mutex_lock(&imx415->lock);
		sel->r = *__imx415_get_pad_crop(imx415, cfg, sel->pad,
										sel->which);
		mutex_unlock(&imx415->lock);

		return 0;
	}

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

/* Start streaming */
static int imx415_start_streaming(struct imx415 *imx415)
{
	enum imx415_clk_index clk_idx = imx415->xclk_freq == 37125000 ? CLK_37_125 : CLK_74_25;
	int ret;

	/* Set init register settings */
	ret = imx415_set_register_array(imx415, imx415_global_init_settings,
									ARRAY_SIZE(
										imx415_global_init_settings));
	if (ret < 0)
	{
		dev_err(imx415->dev, "Could not set init registers\n");
		return ret;
	}

	ret = imx415_set_register_array(imx415,
									imx415->current_mode->clk_data[clk_idx],
									imx415->current_mode->clk_size);
	if (ret < 0)
	{
		dev_err(imx415->dev, "Could not set clock registers\n");
		return ret;
	}

	/* Apply the register values related to current frame format */
	ret = imx415_write_current_format(imx415);
	if (ret < 0)
	{
		dev_err(imx415->dev, "Could not set frame format\n");
		return ret;
	}

	/* Apply default values of current mode */
	ret = imx415_set_register_array(imx415, imx415->current_mode->data,
									imx415->current_mode->data_size);
	if (ret < 0)
	{
		dev_err(imx415->dev, "Could not set current mode\n");
		return ret;
	}

	/* Apply customized values from user */
	ret = v4l2_ctrl_handler_setup(imx415->sd.ctrl_handler);
	if (ret)
	{
		dev_err(imx415->dev, "Could not sync v4l2 controls\n");
		return ret;
	}

	ret = imx415_write_reg(imx415, IMX415_REG_STANDBY, 0x00);
	if (ret < 0)
		return ret;

	msleep(30);

	/* Start streaming */
	return imx415_write_reg(imx415, IMX415_REG_XMSTA, 0x00);
}

static int imx415_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx415 *imx415 = to_imx415(sd);
	int ret = 0;

	if (enable)
	{
		ret = pm_runtime_get_sync(imx415->dev);
		if (ret < 0)
		{
			pm_runtime_put_noidle(imx415->dev);
			goto unlock_and_return;
		}

		ret = imx415_start_streaming(imx415);
		if (ret)
		{
			dev_err(imx415->dev, "Start stream failed\n");
			pm_runtime_put(imx415->dev);
			goto unlock_and_return;
		}
	}
	else
	{
		imx415_stop_streaming(imx415);
		pm_runtime_put(imx415->dev);
	}
	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx415->vflip, enable);
	__v4l2_ctrl_grab(imx415->hflip, enable);

unlock_and_return:

	return ret;
}

static int imx415_get_regulators(struct device *dev, struct imx415 *imx415)
{
	unsigned int i;

	for (i = 0; i < IMX415_NUM_SUPPLIES; i++)
		imx415->supplies[i].supply = imx415_supply_name[i];

	return devm_regulator_bulk_get(dev, IMX415_NUM_SUPPLIES,
								   imx415->supplies);
}

static int imx415_set_data_lanes(struct imx415 *imx415)
{
	int ret = 0, laneval;

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
		dev_err(imx415->dev, "Lane configuration not supported\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = imx415_write_reg(imx415, IMX415_REG_LANE_MODE, laneval);
	if (ret)
	{
		dev_err(imx415->dev, "Error setting Lane mode register\n");
		goto exit;
	}

exit:
	return ret;
}

/* Verify chip ID */
static int imx415_identify_module(struct imx415 *imx415)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx415->sd);
	int ret;
	u8 val;

	ret = imx415_read_reg(imx415, IMX415_REG_CHIP_ID, &val);
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

	dev_info(&client->dev, "chip id match: %d\n", IMX415_CHIP_ID);

	return 0;
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
		dev_err(imx415->dev, "Failed to enable clock\n");
		return ret;
	}

	ret = regulator_bulk_enable(IMX415_NUM_SUPPLIES, imx415->supplies);
	if (ret)
	{
		dev_err(imx415->dev, "Failed to enable regulators\n");
		clk_disable_unprepare(imx415->xclk);
		return ret;
	}

	usleep_range(1, 2);
	gpiod_set_value_cansleep(imx415->rst_gpio, 1);
	dev_info(imx415->dev, "pwoer on");
	usleep_range(30000, 31000);

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

	return 0;
}

static const struct dev_pm_ops imx415_pm_ops = {
	SET_RUNTIME_PM_OPS(imx415_power_off, imx415_power_on, NULL)};

static const struct v4l2_subdev_video_ops imx415_video_ops = {
	.s_stream = imx415_set_stream,
};

static const struct v4l2_subdev_pad_ops imx415_pad_ops = {
	.init_cfg = imx415_entity_init_cfg,
	.enum_mbus_code = imx415_enum_mbus_code,
	.enum_frame_size = imx415_enum_frame_size,
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

/*
 * Returns 0 if all link frequencies used by the driver for the given number
 * of MIPI data lanes are mentioned in the device tree, or the value of the
 * first missing frequency otherwise.
 */
static s64 imx415_check_link_freqs(const struct imx415 *imx415,
								   const struct v4l2_fwnode_endpoint *ep)
{
	int i, j;
	const s64 *freqs = imx415_link_freqs_ptr(imx415);
	int freqs_count = imx415_link_freqs_num(imx415);

	for (i = 0; i < freqs_count; i++)
	{
		for (j = 0; j < ep->nr_of_link_frequencies; j++)
			if (freqs[i] == ep->link_frequencies[j])
				break;
		if (j == ep->nr_of_link_frequencies)
			return freqs[i];
	}
	return 0;
}

static const struct of_device_id imx415_of_match[] = {
	{.compatible = "sony,imx415", .data = imx415_colour_formats},
	{/* sentinel */}};

static int imx415_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	/* Only CSI2 is supported for now: */
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY};
	const struct of_device_id *match;
	const struct imx415_mode *mode;
	struct imx415 *imx415;
	u32 address = 0;
	s64 fq;
	u32 h_blank;
	u32 v_blank;
	int ret;

	imx415 = devm_kzalloc(dev, sizeof(*imx415), GFP_KERNEL);
	if (!imx415)
		return -ENOMEM;

	imx415->dev = dev;
	imx415->regmap = devm_regmap_init_i2c(client, &imx415_regmap_config);
	if (IS_ERR(imx415->regmap))
	{
		dev_err(dev, "Unable to initialize I2C\n");
		return -ENODEV;
	}

	match = of_match_device(imx415_of_match, dev);
	if (!match)
		return -ENODEV;
	imx415->formats = (const struct imx415_pixfmt *)match->data;

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
	imx415->hmax_min = (imx415->nlanes == 2) ? IMX415_HMAX_MIN_2LANE : IMX415_HMAX_MIN_4LANE;

	dev_dbg(dev, "Using %u data lanes\n", imx415->nlanes);

	if (!ep.nr_of_link_frequencies)
	{
		dev_err(dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto err_free_err;
	}

	/* Check that link frequences for all the modes are in device tree */
	fq = imx415_check_link_freqs(imx415, &ep);
	if (fq)
	{
		dev_err(dev, "Link frequency of %lld is not supported\n", fq);
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

	dev_dbg(dev, "imx415 address： %d\n", address);

	ret = fwnode_property_read_u32(dev_fwnode(dev), "clock-frequency",
								   &imx415->xclk_freq);
	if (ret)
	{
		dev_err(dev, "Could not get xclk frequency\n");
		goto err_free_err;
	}

	/* external clock must be 37.125 MHz */
	if (imx415->xclk_freq != 37125000 && imx415->xclk_freq != 74250000)
	{
		dev_err(dev, "External clock frequency %u is not supported\n",
				imx415->xclk_freq);
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

	imx415->rst_gpio = devm_gpiod_get_optional(dev, "reset",
											   GPIOD_OUT_HIGH);
	if (IS_ERR(imx415->rst_gpio))
	{
		dev_err(dev, "Cannot get reset gpio\n");
		ret = PTR_ERR(imx415->rst_gpio);
		goto err_free_err;
	}

	mutex_init(&imx415->lock);

	/*
	 * Initialize the frame format. In particular, imx415->current_mode
	 * and imx415->bpp are set to defaults: imx415_calc_pixel_rate() call
	 * below relies on these fields.
	 */
	imx415_entity_init_cfg(&imx415->sd, NULL);

	dev_err(dev, "curent mode: %d\n", imx415->current_mode->width),

		v4l2_ctrl_handler_init(&imx415->ctrls, 9);

	imx415->anal_a_gain = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops,
											V4L2_CID_ANALOGUE_GAIN, IMX415_GAIN_MIN,
											IMX415_GAIN_MAX, IMX415_GAIN_STEP,
											IMX415_GAIN_DEFAULT);

	mode = imx415->current_mode;

	h_blank = mode->hmax - mode->width;

	imx415->hblank = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops,
									   V4L2_CID_HBLANK,
									   h_blank,
									   h_blank,
									   1,
									   h_blank);

	v_blank = mode->vmax - mode->height;

	imx415->vblank = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops,
									   V4L2_CID_VBLANK,
									   v_blank,
									   v_blank,
									   1,
									   v_blank);

	imx415->exposure = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops,
										 V4L2_CID_EXPOSURE,
										 IMX415_EXPOSURE_MIN,
										 mode->vmax - 2,
										 IMX415_EXPOSURE_STEP,
										 mode->vmax - 2);

	imx415->hflip = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops,
									  V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx415->vflip = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops,
									  V4L2_CID_VFLIP, 0, 1, 1, 0);

	imx415->link_freq = v4l2_ctrl_new_int_menu(&imx415->ctrls, &imx415_ctrl_ops,
											   V4L2_CID_LINK_FREQ,
											   imx415_link_freqs_num(imx415) - 1, 0,
											   imx415_link_freqs_ptr(imx415));
	if (imx415->link_freq)
		imx415->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx415->pixel_rate = v4l2_ctrl_new_std(&imx415->ctrls, &imx415_ctrl_ops,
										   V4L2_CID_PIXEL_RATE,
										   1, INT_MAX, 1,
										   imx415_calc_pixel_rate(imx415));

	v4l2_ctrl_new_std_menu_items(&imx415->ctrls, &imx415_ctrl_ops,
								 V4L2_CID_TEST_PATTERN,
								 ARRAY_SIZE(imx415_test_pattern_menu) - 1,
								 0, 0, imx415_test_pattern_menu);

	imx415->sd.ctrl_handler = &imx415->ctrls;

	if (imx415->ctrls.error)
	{
		dev_err(dev, "Control initialization error %d\n",
				imx415->ctrls.error);
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

	/* Set data lane count */
	ret = imx415_set_data_lanes(imx415);
	if (ret)
		goto error_power_off;

	imx415->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx415->sd.dev = &client->dev;
	imx415->sd.entity.ops = &imx415_subdev_entity_ops;
	imx415->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	imx415->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx415->sd.entity, 1, &imx415->pad);
	if (ret < 0)
	{
		dev_err(dev, "Could not register media entity\n");
		goto err_free_ctrl;
	}

	/* Initialize the frame format (this also sets imx415->current_mode) */
	imx415_entity_init_cfg(&imx415->sd, NULL);

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

	pm_runtime_disable(imx415->dev);
	if (!pm_runtime_status_suspended(imx415->dev))
		imx415_power_off(imx415->dev);
	pm_runtime_set_suspended(imx415->dev);

	return 0;
}

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
MODULE_AUTHOR("Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>");
MODULE_LICENSE("GPL v2");