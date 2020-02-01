
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/printk.h>
#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/input.h> /* BUS_I2C */

#define DEFUALT_MPR121_ADDR 0X5B

#define TOUCH_THRESHOLD_MAX 0X50

/****************************************************Sensor register address!!***********************************************/
/****************************************************************************************************************************/

#define CHANNEL_NUM 12

/*These two registers indicate the detected touch/release status of all of the 12 sensing input channels*/
#define TOUCH_STATUS_REG_ADDR_L 0X00
#define TOUCH_STATUS_REG_ADDR_H 0X01

/*The MPR121 provides filtered electrode output data for all 12 channels*/
/*Total 26 registers,for 12 channel dlectrode out put data.Each channel corresponding two registers:high byte and low byte*/
/*0x04~0x1d*/
#define FILTERED_DATA_REG_START_ADDR_L 0X04
#define FILTERED_DATA_REG_START_ADDR_H 0X05

/*Along with the 10-bit electrode filtered data output, each channel also has a 10-bit baseline value*/
/*0X1E~0X2A*/
#define BASELINE_VALUE_REG_START_ADDR 0X1E

/*All12 of the electrode baseline values are controlled by the same set of filtering control registers, 0x2B ~ 0x35*/
/*The 13th channel ELEPROX is controlled by registers 0x36 ~ 0x40*/
#define BASELINE_FILTERING_CONTROL_REG_START_ADDR 0X2B

/*Each of the 12 channels can be set with its own set of touch and release thresholds. Touch and release are detected by
comparing the electrode filtered data to the baseline value. typically in the range 0x04~0x10*/
/*Touch condition: Baseline - Electrode filtered data > Touch threshold
  Release condition: Baseline - Electrode filtered data < Release threshold*/
#define THRESHOLD_REG_START_ADDR 0X41

/*All 12 channels use the same set of touch and release debounce numbers.*/
#define DEBOUNCE_REG_ADDR 0X5B

/*These two registers set the global AFE settings. This includes global electrode charge/discharge current CDC, global charge/
discharge time CDT, as well as a common filtering setting (FFI, SFI, ESI) for all 12 channels, including the 13th Eleprox channel*/
#define FILTER_AND_GLOBAL_CDC_CFG_ADDR 0X5C
#define FILTER_AND_GLOBAL_CDT_CFG_ADDR 0X5D

/*0X5F-0X6B*/
#define ELEC_CHARGE_CURRENT_REG_START_ADDR 0X5F

/*0X6C-0X72*/
#define ELEC_CHARGE_TIME_REG_START_ADDR 0X6C

/*The Electrode Configuration Register (ECR) determines if the MPR121 is in Run Mode or Stop Mode*/
/*Default is 0 to stop mode*/
#define ELEC_CFG_REG_ADDR 0X5E

struct mpr121_platform_data
{
    u8 key;
};
struct mpr121
{
    struct device *dev;
    struct mpr121_platform_data pdata;
};

static const struct mpr121_platform_data mpr121_default_init = {
    .key = 0,
};

struct mpr121_bus_ops
{
    u16 bustype;
    int (*read)(struct device *, unsigned char);
    int (*read_block)(struct device *, unsigned char, int, void *);
    int (*write)(struct device *, unsigned char, unsigned char);
};

static int mpr121_smbus_read(struct device *dev, unsigned char reg)
{
    struct i2c_client *client = to_i2c_client(dev);

    return i2c_smbus_read_byte_data(client, reg);
}

static int mpr121_smbus_write(struct device *dev,
                              unsigned char reg, unsigned char val)
{
    struct i2c_client *client = to_i2c_client(dev);

    return i2c_smbus_write_byte_data(client, reg, val);
}

static int mpr121_smbus_read_block(struct device *dev,
                                   unsigned char reg, int count,
                                   void *buf)
{
    struct i2c_client *client = to_i2c_client(dev);

    return i2c_smbus_read_i2c_block_data(client, reg, count, buf);
}

static int mpr121_i2c_read_block(struct device *dev,
                                 unsigned char reg, int count,
                                 void *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int ret;

    ret = i2c_master_send(client, &reg, 1);
    if (ret < 0)
        return ret;

    ret = i2c_master_recv(client, buf, count);
    if (ret < 0)
        return ret;

    if (ret != count)
        return -EIO;

    return 0;
}

static const struct mpr121_bus_ops mpr121_smbus_bops = {
    .bustype = BUS_I2C,
    .write = mpr121_smbus_write,
    .read = mpr121_smbus_read,
    .read_block = mpr121_smbus_read_block,
};

static const struct mpr121_bus_ops mpr121_i2c_bops = {
    .bustype = BUS_I2C,
    .write = mpr121_smbus_write,
    .read = mpr121_smbus_read,
    .read_block = mpr121_i2c_read_block,
};

static ssize_t mpr121_data_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
    u16 value = 10;
    u8 val_l, val_h;
    int i;
    u16 filtered_data_buf[CHANNEL_NUM]={0};

    val_l = mpr121_smbus_read(dev, TOUCH_STATUS_REG_ADDR_L);
    val_h = mpr121_smbus_read(dev, TOUCH_STATUS_REG_ADDR_H);

    value = val_h << 8 | val_l;

    for(i=0;i<CHANNEL_NUM;i++)
    {
        if(value&(1<<i))
        {
            val_l = mpr121_smbus_read(dev,FILTERED_DATA_REG_START_ADDR_L+2*i);
            val_h = mpr121_smbus_read(dev,FILTERED_DATA_REG_START_ADDR_L+2*i+1);
            filtered_data_buf[i]=(u16)val_h<<8|val_l;
            if(TOUCH_THRESHOLD_MAX<filtered_data_buf[i])
            {
                value &=~(1<<i);
            }
        }
    }


    return sprintf(buf, "%x\n", value);
}

void mpr121_init(struct device *dev)
{
    /*Set stop mode by set ELEC_CFG_REG_ADDR to 0x0*/
    mpr121_smbus_write(dev, ELEC_CFG_REG_ADDR, 0);
    mpr121_smbus_write(dev, ELEC_CFG_REG_ADDR, 0);
    /**    FFI-6,CDC-16UA,CDT-0.5US,SFI-4,ESI-4MS 0x2310
    */
    mpr121_smbus_write(dev, FILTER_AND_GLOBAL_CDC_CFG_ADDR, 0x10);
    mpr121_smbus_write(dev, FILTER_AND_GLOBAL_CDT_CFG_ADDR, 0x23);

    /*Touch debounce =2*SFI*ESI,Realese debounce=2*SFI*ESI */
    mpr121_smbus_write(dev, DEBOUNCE_REG_ADDR, 0X22);

    /*Set start mode with proximity disable by set ELEC_CFG_REG_ADDR */
    mpr121_smbus_write(dev, ELEC_CFG_REG_ADDR, 0x3c);
}

static ssize_t mpr121_init_store(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t count)
{
    mpr121_init(dev);
    return count;
}

static DEVICE_ATTR(mpr121_data, 0664, mpr121_data_show, NULL);
static DEVICE_ATTR(mpr121_init, 0664, NULL, mpr121_init_store);

static struct attribute *mpr121_attributes[] = {
    &dev_attr_mpr121_data.attr,
    &dev_attr_mpr121_init.attr,
    NULL};

static const struct attribute_group mpr121_attr_group = {
    .attrs = mpr121_attributes,
};

struct mpr121 *mpr121_probe(struct device *dev, int irq,
                            bool fifo_delay_default,
                            const struct mpr121_bus_ops *bops)
{
    struct mpr121 *ac;
    u32 err;
    const struct mpr121_platform_data *pdata;

    ac = kzalloc(sizeof(*ac), GFP_KERNEL);
    if (!ac)
    {
        err = -ENOMEM;
        goto err_free_mem;
    }

    pdata = dev_get_platdata(dev);
    if (!pdata)
    {
        printk(KERN_ALERT
               "No platform data: Using default initialization\n");
        pdata = &mpr121_default_init;
    }

    ac->pdata = *pdata;
    pdata = &ac->pdata;

    err = sysfs_create_group(&dev->kobj, &mpr121_attr_group);
    if (err)
        goto err_remove_attr;

    printk(KERN_INFO "mpr121 Registered\n");

    mpr121_init(dev);
    return ac;

err_remove_attr:
    printk(KERN_ALERT "create sysfs failed\n");
    sysfs_remove_group(&dev->kobj, &mpr121_attr_group);

err_free_mem:
    kfree(ac);
    return ERR_PTR(err);
}

int mpr121_remove(struct mpr121 *ac)
{

    //sysfs_remove_group(&ac->dev->kobj, &mpr121_attr_group);

    printk(KERN_INFO "unregistered accelerometer\n");
    kfree(ac);

    return 0;
}

static int mpr121_i2c_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
    struct mpr121 *ac;
    int error;

    error = i2c_check_functionality(client->adapter,
                                    I2C_FUNC_SMBUS_BYTE_DATA);
    if (!error)
    {
        printk(KERN_ALERT "SMBUS Byte Data not Supported\n");
        return -EIO;
    }

    ac = mpr121_probe(&client->dev, client->irq, false,
                      i2c_check_functionality(client->adapter,
                                              I2C_FUNC_SMBUS_READ_I2C_BLOCK)
                          ? &mpr121_smbus_bops
                          : &mpr121_i2c_bops);
    if (IS_ERR(ac))
        return PTR_ERR(ac);

    i2c_set_clientdata(client, ac);

    return 0;
}

static int mpr121_i2c_remove(struct i2c_client *client)
{

    struct mpr121 *ac = i2c_get_clientdata(client);

    return mpr121_remove(ac);
}

static const struct i2c_device_id mpr121_id[] = {
    {"mpr121", 0},
    {}};

MODULE_DEVICE_TABLE(i2c, mpr121_id);

#ifdef CONFIG_OF
static const struct of_device_id mpr121_of_id[] = {
    /*
	 * The ADXL346 is backward-compatible with the ADXL345. Differences are
	 * handled by runtime detection of the device model, there's thus no
	 * need for listing the "adi,adxl346" compatible value explicitly.
	 */
    /*
	 * Deprecated, DT nodes should use one or more of the device-specific
	 * compatible values "adi,adxl345" and "adi,adxl346".
	 */
    {
        .compatible = "mpr121",
    },
    {}};

MODULE_DEVICE_TABLE(of, mpr121_of_id);
#endif

static struct i2c_driver mpr121_driver = {
    .driver = {
        .name = "mpr121",
        .of_match_table = of_match_ptr(mpr121_of_id),
    },
    .probe = mpr121_i2c_probe,
    .remove = mpr121_i2c_remove,
    .id_table = mpr121_id,
};

module_i2c_driver(mpr121_driver);

MODULE_AUTHOR("Baozhu Zuo <baozhu.zuo@gmail.com>");
MODULE_DESCRIPTION("mpr121 I2C Bus Driver");
MODULE_LICENSE("GPL");
