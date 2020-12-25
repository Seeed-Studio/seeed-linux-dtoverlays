#include "mipi_dsi.h"


#define GOODIX_CONTACT_SIZE		        8
#define GOODIX_BUFFER_STATUS_READY	    (((uint32_t)0x01)<<7)//BIT(7)
#define GOODIX_HAVE_KEY			        (((uint32_t)0x01)<<4)//BIT(4)


#define TP_DEFAULT_WIDTH	480
#define TP_DEFAULT_HEIGHT	854
#define TP_MAX_POINTS       5
#define TP_POLL_INTERVAL    17


static u32 touch_rec = 0;
static void tp_poll_func(struct input_dev *input)
{
	struct i2c_mipi_dsi *md = (struct i2c_mipi_dsi *)input_get_drvdata(input);
	int num = 0;
	int ret;
	u8 data[40];

	ret = i2c_md_read(md, REG_TP_STATUS, data, 2);
	//DBG_FUNC("0x%x,0x%x", data[0], data[1]);
	if (data[0] & GOODIX_BUFFER_STATUS_READY) {
		num = data[0] & 0x0F;
		//DBG_FUNC("touched:%d", num);
		ret = i2c_md_read(md, REG_TP_POINT, data, GOODIX_CONTACT_SIZE);
		/*DBG_FUNC("0x%x,0x%x,0x%x,0x%x, 0x%x,0x%x,0x%x,0x%x,", 
		data[0],data[1],data[2],data[3],
		data[4],data[5],data[6],data[7]);*/
		if (num) {
			int x, y;

			touch_rec = 1;
			x = data[1];
			x <<= 8;
			x += data[0];
			y = data[3];
			y <<= 8;
			y += data[2];

			input_mt_slot(input, 0);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, 1);
			touchscreen_report_pos(input, &md->prop, x, y, true);
			//DBG_FUNC("%d,%d", x, y);
		}
	}
	else {
		if (touch_rec) {
			touch_rec = 0;
			input_mt_slot(input, 0);
			input_mt_report_slot_inactive(input);
		}
	}

	input_mt_sync_frame(input);
	input_sync(input);
}

int tp_init(struct i2c_mipi_dsi *md)
{
	struct i2c_client *i2c = md->i2c;
	struct device *dev = &i2c->dev;
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}
	md->input = input;
	input_set_drvdata(input, md);

	input->dev.parent = dev;
	input->name = "seeed-tp";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x1234;
	input->id.product = 0x1001;
	input->id.version = 0x0100;

	i2c_md_write(md, REG_TP_RST, 1);
	msleep(10);
	i2c_md_write(md, REG_TP_RST, 0);
	msleep(10);
	i2c_md_write(md, REG_TP_RST, 1);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, TP_DEFAULT_WIDTH, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, TP_DEFAULT_HEIGHT, 0, 0);
	//touchscreen_parse_properties(input, true, &md->prop);
 #if 0
	md->prop.max_x = 1024;//TP_DEFAULT_WIDTH;
	md->prop.max_y = 768;//TP_DEFAULT_HEIGHT;
	md->prop.invert_x = 0;
	md->prop.invert_y = 1;
	md->prop.swap_x_y = 1;
#endif
	ret = input_mt_init_slots(input, TP_MAX_POINTS, INPUT_MT_DIRECT);
	if (ret) {
		dev_err(dev, "could not init mt slots, %d\n", ret);
		return ret;
	}

	ret = input_setup_polling(input, tp_poll_func);
	if (ret) {
		dev_err(dev, "could not set up polling mode, %d\n", ret);
		return ret;
	}
	input_set_poll_interval(input, TP_POLL_INTERVAL);

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "could not register input device, %d\n", ret);
		return ret;
	}

	return 0;
}

int tp_deinit(struct i2c_mipi_dsi *md)
{
	input_unregister_device(md->input);
	return 0;
}
