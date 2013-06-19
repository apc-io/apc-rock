/*++
        drivers/mfd/fan5365.c

        Copyright (c) 2012 WonderMedia Technologies, Inc.

        This program is free software: you can redistribute it and/or modify it under the
        terms of the GNU General Public License as published by the Free Software Foundation,
        either version 2 of the License, or (at your option) any later version.

        This program is distributed in the hope that it will be useful, but WITHOUT
        ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
        PARTICULAR PURPOSE.  See the GNU General Public License for more details.
        You should have received a copy of the GNU General Public License along with
        this program.  If not, see <http://www.gnu.org/licenses/>.

        WonderMedia Technologies, Inc.
        10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>

#include <linux/timer.h>


#define FAN5365_I2C_NAME "FAN5365"
#define FAN5365_NR_DEVS  1 

struct pmic_data {
	struct i2c_client *client;
	struct delayed_work work;
};
struct dev_data {
	dev_t devno;
	struct cdev cdev;
  	struct class *class;
	struct i2c_client *client;
};

static struct pmic_data *gl_pmic;
static struct dev_data dev;

unsigned int FAN5365_read_pmic_volt(void);
void FAN5365_set_pmic_volt(unsigned int volt);

static int i2cReadFromFAN5365(struct i2c_client *client, unsigned char bufferIndex,
		unsigned char dataBuffer[], unsigned short dataLength) {
	int ret;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = I2C_M_NOSTART,
			.len = 1,
			.buf = &bufferIndex
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = dataLength,
			.buf = dataBuffer
		}
	};

	memset(dataBuffer, 0x00, dataLength);
	ret = i2c_transfer(client->adapter, msgs, 2);
	/*
	printk("dataBuffer = %x\n", dataBuffer[0]);
	*/
	return ret;
}

static int i2cWriteToFAN5365(struct i2c_client *client, unsigned char bufferIndex,
	unsigned char const dataBuffer[], unsigned short dataLength) {

	unsigned char buffer4Write[256];
	struct i2c_msg msgs[1] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = dataLength + 1,
			.buf = buffer4Write
		}
	};
	buffer4Write[0] = bufferIndex;
	memcpy(&(buffer4Write[1]), dataBuffer, dataLength);

	return i2c_transfer(client->adapter, msgs, 1);
}

static int wmt_read_pmic_ready(void)
{
	unsigned char buf[1];
	buf[0] = 0;
	i2cReadFromFAN5365(gl_pmic->client, 3, buf, 1);
	if (buf[0] & BIT5)
		return 0;
	return -1;
}

unsigned int FAN5365_read_pmic_volt(void)
{
	unsigned char buf[1];
	i2cReadFromFAN5365(gl_pmic->client, 1, buf, 1);
	buf[0] &= ~(BIT7 | BIT6);
	return 7500 + 125 * buf[0];
}
EXPORT_SYMBOL(FAN5365_read_pmic_volt);

void FAN5365_set_pmic_volt(unsigned int volt)
{
	unsigned char reg_value;
	if(volt >= 14375)
		volt = 14375;
	reg_value = (volt - 7500)/125;
	reg_value |= (BIT7 | BIT6);
	i2cWriteToFAN5365(gl_pmic->client, 1, &reg_value, 1);
	while (wmt_read_pmic_ready() == 1) 
		schedule();
}
EXPORT_SYMBOL(FAN5365_set_pmic_volt);

static void FAN5365_pmic_work_func(struct work_struct *work)
{
	FAN5365_set_pmic_volt(13000);
	schedule_delayed_work(&gl_pmic->work, msecs_to_jiffies(60000));
}

static int FAN5365_dev_open(struct inode *inode, struct file *filp)
{
	return 0;
}
static int FAN5365_dev_close(struct inode *inode, struct file *filp)
{
	return 0;
}
static struct file_operations FAN5365_dev_fops = {
	.owner          = THIS_MODULE,
	.open           = FAN5365_dev_open,
	.release        = FAN5365_dev_close,
};

static ssize_t FAN5365_show_vol( struct device *dev, struct device_attribute *attr, char *buf)
{
	int volt;
	int count;
	volt = FAN5365_read_pmic_volt();
	count = sprintf(buf, "%d\n", volt);
	return count;
}

static ssize_t FAN5365_store_vol( struct device *dev, struct device_attribute *attr, char const *buf, size_t count)
{
	int volt;
	sscanf(buf, "%d", &volt);
	FAN5365_set_pmic_volt(volt);
	return count;
}

static struct device_attribute FAN5365_attributes[] = {
	__ATTR(voltage, 0644, FAN5365_show_vol, FAN5365_store_vol),
	__ATTR_NULL,
};

static int create_device_attributes(struct device *dev,	struct device_attribute *attrs)
{
	int i;
	int err = 0;
	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (0 != err)
			break;
	}

	if (0 != err) {
		for (; i >= 0 ; --i)
			device_remove_file(dev, &attrs[i]);
	}
	return err;
}

static int FAN5365_pmic_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {
	int ret = 0;
	struct device *device;
	gl_pmic = kzalloc(sizeof(struct pmic_data), GFP_KERNEL);

	ret = alloc_chrdev_region(&dev.devno, 0, 1, FAN5365_I2C_NAME);
  	if (ret) {
    		printk("%s, can't allocate chrdev\n", __func__);
		return ret;
	}
	cdev_init(&dev.cdev, &FAN5365_dev_fops);  
  	ret = cdev_add(&dev.cdev, dev.devno, 1);
  	if (ret < 0) {
    		printk("%s, add character device error, ret %d\n", __func__, ret);
		return ret;
	}
	dev.class = class_create(THIS_MODULE, "FAN5365_PMIC");
	if(IS_ERR(dev.class)) {
		printk("%s, create class, error\n", __func__);
		return ret;
  	}

	device = device_create(dev.class, NULL, dev.devno, NULL, FAN5365_I2C_NAME);
	ret = create_device_attributes(device, FAN5365_attributes);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "FAN5365_pmic_probe : need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		return ret;
	}
	gl_pmic->client = client;

	INIT_DELAYED_WORK(&gl_pmic->work, FAN5365_pmic_work_func);
	/*
	schedule_delayed_work(&gl_pmic->work, msecs_to_jiffies(2000));
	*/
	i2c_set_clientdata(client, gl_pmic);

	return 0;

}

static int FAN5365_pmic_remove(struct i2c_client *client) {
	return 0;
}

static const struct i2c_device_id FAN5365_pmic_id[] = {
	{FAN5365_I2C_NAME, 0},
	{ }
};


static struct i2c_driver FAN5365_pmic_driver = {
	.probe = FAN5365_pmic_probe,
	.remove = FAN5365_pmic_remove,
	.id_table = FAN5365_pmic_id,
	.driver = {
		.name = "FAN5365-pmic",
	},
};

static struct i2c_board_info fan5365_pmic_i2c_board_info = {
	.type          = "FAN5365",
	.flags         = 0x00,
	.addr          = 0x4A,
	.platform_data = NULL,
	.archdata      = NULL,
	.irq           = -1,
};

static int __devinit FAN5365_init(void) {
	struct i2c_board_info *fan5365_i2c_bi;
	struct i2c_adapter *adapter = NULL;
	struct i2c_client *client   = NULL;
	int ret = 0;

	fan5365_i2c_bi = &fan5365_pmic_i2c_board_info;
	adapter = i2c_get_adapter(3);/*in bus 3*/
	if (NULL == adapter) {
		printk("can not get i2c adapter, client address error\n");
		ret = -ENODEV;
		return -1;
	}
	client = i2c_new_device(adapter, fan5365_i2c_bi);
	if (client == NULL) {
		printk("allocate i2c client failed\n");
		ret = -ENOMEM;
		return -1;
	}
	i2c_put_adapter(adapter);

	return i2c_add_driver(&FAN5365_pmic_driver);
}

static void __exit FAN5365_exit(void)
{
	i2c_del_driver(&FAN5365_pmic_driver);
}

module_init(FAN5365_init);
module_exit(FAN5365_exit);

MODULE_AUTHOR("WonderMedia Technologies, Inc.");
MODULE_DESCRIPTION("FAN5365 PMIC Driver");
MODULE_LICENSE("GPL");
