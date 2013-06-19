/* drivers/input/touchscreen/IT7260_ts_i2c.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

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
#include "IT7260_ts.h"
#include <mach/hardware.h>

#include <linux/timer.h>
#include <linux/gpio.h>

static struct timer_list tp_timer;
static void tp_irq_handler_reg(unsigned long arg);

static int g_ModuleInstalled;
static int g_is_it7260;
#define READ_POINTE_INTERVAL 10//10ms

#define IT7260_I2C_NAME "IT7260"
#include <linux/gpio.h>
static int ite7260_major = 0; // dynamic major by default
static int ite7260_minor = 0;
static struct cdev ite7260_cdev;
static struct class *ite7260_class = NULL;
static dev_t ite7260_dev;
static struct input_dev *input_dev;
static int ts_suspend = 0;
extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);
#define SET_GPIO_TS_INT() {\
	REG32_VAL(wmt_ts_gpt.ctraddr) |= wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.ocaddr) &= ~wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.pcaddr) &= ~wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.peaddr) |= wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.isaddr) = wmt_ts_gpt.isbmp; \
}
struct wmt_gpt_s {
        char name[20];
        unsigned int bitmap;
        unsigned int ctraddr;
        unsigned int ocaddr;
        unsigned int idaddr;
        unsigned int peaddr;
        unsigned int pcaddr;
        unsigned int itbmp;
        unsigned int itaddr;
        unsigned int isbmp;
        unsigned int isaddr;
        unsigned int irq;
};

static struct wmt_gpt_s wmt_ts_gpt ={
	.name = "it7260",
	.bitmap = 0x200,/*GPIO 9*/
	.ctraddr = 0xFE110040,
	.ocaddr = 0xFE10080,
	.idaddr = 0xFE110000,
	.peaddr = 0xFE110480,
	.pcaddr = 0xFE1104c0,
	.itbmp = 0x00000100, /* low level */
	.itaddr = 0xFE110308,
	.isbmp = 0x200,
	.isaddr = 0xFE110360,
	.irq = 5,
};

static void parse_gpt_arg(void)
{
	char *varname = "wmt.gpt.ts";
	unsigned int varlen = 160;
	unsigned char buf[160];
	int retval;
	int i = 0;
	retval = wmt_getsyspara(varname, buf, &varlen);
	if (retval != 0)
		return;
	if (buf[0] != 'i')
		return;
	if (buf[1] != 't')
		return;
	if (buf[2] != '7')
		return;
	if (buf[3] != '2')
		return;
	if (buf[4] != '6')
		return;
	if (buf[5] != '0')
		return;
	i = 7;
	sscanf((buf + i), "%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x",
			&wmt_ts_gpt.bitmap,
			&wmt_ts_gpt.ctraddr,
			&wmt_ts_gpt.ocaddr,
			&wmt_ts_gpt.idaddr,
			&wmt_ts_gpt.peaddr,
			&wmt_ts_gpt.pcaddr,
			&wmt_ts_gpt.itbmp,
			&wmt_ts_gpt.itaddr,
			&wmt_ts_gpt.isbmp,
			&wmt_ts_gpt.isaddr,
			&wmt_ts_gpt.irq);
		
	wmt_ts_gpt.ctraddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.ocaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.idaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.peaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.pcaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.itaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.isaddr += WMT_MMAP_OFFSET;
	printk("wmt.gpt.ts = %x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x\n",
		wmt_ts_gpt.bitmap,
		wmt_ts_gpt.ctraddr,
		wmt_ts_gpt.ocaddr,
		wmt_ts_gpt.idaddr,
		wmt_ts_gpt.peaddr,
		wmt_ts_gpt.pcaddr,
		wmt_ts_gpt.itbmp,
		wmt_ts_gpt.itaddr,
		wmt_ts_gpt.isbmp,
		wmt_ts_gpt.isaddr,
		wmt_ts_gpt.irq);
}

static int pulldown_intsrc_commended(void)
{
	// set the interrupt gpio as out
	REG32_VAL(wmt_ts_gpt.ctraddr) |= wmt_ts_gpt.bitmap; /* Enable gpio*/
	REG32_VAL(wmt_ts_gpt.ctraddr + 0x80) &= ~(wmt_ts_gpt.bitmap); /*pull down*/
	REG32_VAL(wmt_ts_gpt.ocaddr) |= wmt_ts_gpt.bitmap; /* Set output*/
	mdelay(30);
	REG32_VAL(wmt_ts_gpt.ctraddr + 0x80) |= wmt_ts_gpt.bitmap;/*pull up*/
	return 0;
}

static void ts_set_gpio_int(void)
{
	if (wmt_ts_gpt.itbmp & 0xff) {
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff;
		REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & 0x7F);
	} else if (wmt_ts_gpt.itbmp & 0xff00) {
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff00;
		REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & 0x7F00);
	} else if (wmt_ts_gpt.itbmp & 0xff0000) {
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff0000;
		REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & 0x7F0000);
	} else if (wmt_ts_gpt.itbmp & 0xff000000) {
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff000000;
		REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & 0x7F000000);
	}
	SET_GPIO_TS_INT();
}

static int ts_read_irqstatus(void)
{
	return (REG32_VAL(wmt_ts_gpt.isaddr) & wmt_ts_gpt.isbmp) == (wmt_ts_gpt.isbmp) ? 1 : 0;
}

static void ts_clear_irq(void)
{
        REG32_VAL(wmt_ts_gpt.isaddr) = wmt_ts_gpt.isbmp;
}
static void ts_enable_irq(void)
{
	if (ts_suspend == 0) {
		if (wmt_ts_gpt.itbmp & 0xff)
			REG32_VAL(wmt_ts_gpt.itaddr) |= 0x80;
		else if (wmt_ts_gpt.itbmp & 0xff00)
			REG32_VAL(wmt_ts_gpt.itaddr) |= 0x8000;
		else if (wmt_ts_gpt.itbmp & 0xff0000)
			REG32_VAL(wmt_ts_gpt.itaddr) |= 0x800000;
		else if (wmt_ts_gpt.itbmp & 0xff000000)
			REG32_VAL(wmt_ts_gpt.itaddr) |= 0x80000000;
	}
}
static void ts_disable_irq(void)
{
	if (wmt_ts_gpt.itbmp & 0xff)
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x80;
	else if (wmt_ts_gpt.itbmp & 0xff00)
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x8000;
	else if (wmt_ts_gpt.itbmp & 0xff0000)
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x800000;
	else if (wmt_ts_gpt.itbmp & 0xff000000)
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x80000000;
}
#ifdef DEBUG
#define TS_DEBUG(fmt,args...)  printk( KERN_DEBUG "[it7260_i2c]: " fmt, ## args)
#define DBG() printk("[%s]:%d => \n",__FUNCTION__,__LINE__)
#else
#define TS_DEBUG(fmt,args...)
#define DBG()
#endif

static struct workqueue_struct *IT7260_wq;

struct IT7260_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct work_struct work;
	struct early_suspend early_suspend;
	uint8_t debug_log_level;
};
#ifdef CONFIG_HAS_EARLYSUSPEND
static void IT7260_ts_early_suspend(struct early_suspend *h);
static void IT7260_ts_late_resume(struct early_suspend *h);
#endif

static struct IT7260_ts_data *gl_ts;

int i2cReadFromIt7260(struct i2c_client *client, unsigned char bufferIndex,
		unsigned char dataBuffer[], unsigned short dataLength) {
	int ret;
	struct i2c_msg msgs[2] = { { .addr = client->addr, .flags = I2C_M_NOSTART,
			.len = 1, .buf = &bufferIndex }, { .addr = client->addr, .flags =
			I2C_M_RD, .len = dataLength, .buf = dataBuffer } };

	memset(dataBuffer, 0xFF, dataLength);
	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret;
}

int i2cWriteToIt7260(struct i2c_client *client, unsigned char bufferIndex,
		unsigned char const dataBuffer[], unsigned short dataLength) {
	unsigned char buffer4Write[256];
	struct i2c_msg msgs[1] = { { .addr = client->addr, .flags = 0, .len =
			dataLength + 1, .buf = buffer4Write } };

	buffer4Write[0] = bufferIndex;
	memcpy(&(buffer4Write[1]), dataBuffer, dataLength);
	return i2c_transfer(client->adapter, msgs, 1);
}

static int IdentifyCapSensor(struct IT7260_ts_data *ts);

static void Read_Point(struct IT7260_ts_data *ts) {
	unsigned char ucQuery = 0;
	unsigned char pucPoint[14];
#ifdef HAS_8_BYTES_LIMIT
	unsigned char cPoint[8];
	unsigned char ePoint[6];
#endif //HAS_8_BYTES_LIMIT
	int ret = 0;
	int finger2_pressed = 0;
	int xraw, yraw, xtmp, ytmp;
	int i = 0;
	static int x[2] = { (int) -1, (int) -1 };
	static int y[2] = { (int) -1, (int) -1 };
	static bool finger[2] = { 0, 0 };
	static bool flag = 0;

	i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	if (ucQuery < 0) {
		//pr_info("=error Read_Point=\n");
		if (ts->use_irq)
			ts_enable_irq();
		return;
	} else {
		if (ucQuery & 0x80) {
#ifdef HAS_8_BYTES_LIMIT
			i2cReadFromIt7260(ts->client, 0xC0, cPoint, 8);
			ret = i2cReadFromIt7260(ts->client, 0xE0, ePoint, 6);
			for(i=0; i<6; i++) {
				pucPoint[i] = ePoint[i];
			}
			for(i=0; i<8; i++) {
				pucPoint[i+6] = cPoint[i];
			}
#else //HAS_8_BYTES_LIMIT
			ret = i2cReadFromIt7260(ts->client, 0xE0, pucPoint, 14);
#endif //HAS_8_BYTES_LIMIT
			/*
			pr_info("=Read_Point read ret[%d]--point[%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x]=\n",
				ret,pucPoint[0],pucPoint[1],pucPoint[2],
				pucPoint[3],pucPoint[4],pucPoint[5],pucPoint[6],pucPoint[7],pucPoint[8],
				pucPoint[9],pucPoint[10],pucPoint[11],pucPoint[12],pucPoint[13]);
			*/
			if (ret) {
				// gesture
				if (pucPoint[0] & 0xF0) {
					if (ts->use_irq)
						ts_enable_irq();
					//pr_info("(pucPoint[0] & 0xF0) is true, it's a gesture\n") ;
					//pr_info("pucPoint[0]=%x\n", pucPoint[0]);
					return;
				}
				// palm
				if (pucPoint[1] & 0x01) {
					if (ts->use_irq)
						//enable_irq(ts->client->irq);
						//ts_clear_irq();
						ts_enable_irq();
					//pr_info("pucPoint 1 is 0x01, it's a palm\n") ;
					return;
				}
				// no more data
				if (!(pucPoint[0] & 0x08)) {
					if (finger[0]) {
						//input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 1);
						/*
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						*/
						//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
						//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
						/*
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x[0]);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y[0]);
						input_mt_sync(ts->input_dev);
						*/
						finger[0] = 0;
						flag = 1;
					}
					if (finger[1]) {
						//input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 2);
						/*
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						*/
						//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
						//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
						/*
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x[1]);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y[1]);
						input_mt_sync(ts->input_dev);
						*/
						finger[1] = 0;
						flag = 1;
					}
					if (flag) {
						//input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_mt_sync(ts->input_dev);
						input_report_key(ts->input_dev, BTN_TOUCH, 0);
						input_sync(ts->input_dev);
						flag = 0;
					}
					if (ts->use_irq) {
						msleep(READ_POINTE_INTERVAL);
						//ts_clear_irq();
						ts_enable_irq();
					}
					//pr_info("(pucPoint[0] & 0x08) is false, means no more data\n") ;
					return;
				}
				// 3 fingers
				if (pucPoint[0] & 0x04) {
					if (ts->use_irq) {
						msleep(READ_POINTE_INTERVAL);
						ts_enable_irq();
					}
					//pr_info("(pucPoint[0] & 0x04) is true, we don't support three fingers\n") ;
					return;
				}

				if (pucPoint[0] & 0x01) {
					char pressure_point, z, w;

					xraw = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];
					yraw = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];

					pressure_point = pucPoint[5] & 0x0f;
					//pr_info("=Read_Point1 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);

					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
					//input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 1);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, xraw);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, yraw);
					input_mt_sync(ts->input_dev);
					x[0] = xraw;
					y[0] = yraw;
					finger[0] = 1;
					//pr_info("=input Read_Point1 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
				}

				if (pucPoint[0] & 0x02) {
					char pressure_point, z, w;
					xraw = ((pucPoint[7] & 0x0F) << 8) + pucPoint[6];
					yraw = ((pucPoint[7] & 0xF0) << 4) + pucPoint[8];

					pressure_point = pucPoint[9] & 0x0f;

					//pr_info("=Read_Point2 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 1);
					//input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 1);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, xraw);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, yraw);
					input_mt_sync(ts->input_dev);
					x[1] = xraw;
					y[1] = yraw;
					finger[1] = 1;
					//pr_info("input Read_Point2 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
				}
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
				input_sync(ts->input_dev);
			}
		}
	}
	if (ts->use_irq) {
		msleep(READ_POINTE_INTERVAL);
		ts_enable_irq();
	}
	//pr_info("=end Read_Point=\n");

	//IdentifyCapSensor(gl_ts);
}

///////////////////////////////////////////////////////////////////////////////////////

static void IT7260_ts_work_func(struct work_struct *work) {
	int i;
	int ret;
	int bad_data = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[15];
	struct IT7260_ts_data *ts = container_of(work, struct IT7260_ts_data, work);
	//gl_ts = ts;
	if (ts_suspend == 0)
		Read_Point(ts);
}

int delayCount = 0;
static irqreturn_t IT7260_ts_irq_handler(int irq, void *dev_id) {
	struct IT7260_ts_data *ts = dev_id;

	if (!ts_read_irqstatus())
		return IRQ_NONE;
	if (delayCount == 1) {
		pr_info("=IT7260_ts_irq_handler= %x\n", REG32_VAL(wmt_ts_gpt.isaddr));
	}
	ts_disable_irq();
	ts_clear_irq();
	if (ts_suspend == 0)
		queue_work(IT7260_wq, &ts->work);
	//pr_info("1=IT7260_ts_irq_handler= %x\n", REG32_VAL(wmt_ts_gpt.isaddr));
	return IRQ_HANDLED;
}

/////////////////////////////////////////////////////////
void sendCalibrationCmd(void) {
	int ret = 0;
	struct IT7260_ts_data *ts = gl_ts;
	unsigned char data[] = { 0x13, 0x00, 0x00, 0x00, 0x00 };
	unsigned char resp[2];
	unsigned char ucQuery;

	ret = i2cWriteToIt7260(ts->client, 0x20, data, 5);
	printk(KERN_INFO "IT7260 sent calibration command [%d]!!!\n", ret);

	do {
		ucQuery = 0x00;
		mdelay(1000);
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while ((ucQuery & 0x01) != 0);

	//Read out response to clear interrupt.
	i2cReadFromIt7260(ts->client, 0xA0, resp, 2);

	//Send SW-Reset command
	data[0] = 0x6F;
	i2cWriteToIt7260(ts->client, 0x20, data, 1);
}

EXPORT_SYMBOL(sendCalibrationCmd);


void enableAutoTune(void) {
	int ret = 0;
	struct IT7260_ts_data *ts = gl_ts;
	unsigned char data[] = { 0x13, 0x00, 0x01, 0x00, 0x00 };
	unsigned char resp[2];
	unsigned char ucQuery;

	ret = i2cWriteToIt7260(ts->client, 0x20, data, 5);
	printk(KERN_INFO "IT7260 sent calibration command [%d]!!!\n", ret);

	do {
		ucQuery = 0x00;
		mdelay(1000);
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while ((ucQuery & 0x01) != 0);

	//Read out response to clear interrupt.
	i2cReadFromIt7260(ts->client, 0xA0, resp, 2);

	//Send SW-Reset command
	data[0] = 0x6F;
	i2cWriteToIt7260(ts->client, 0x20, data, 1);
}

EXPORT_SYMBOL(enableAutoTune);

static void tp_irq_handler_reg(unsigned long arg)
{
	delayCount = 0;
}

static int IdentifyCapSensor(struct IT7260_ts_data *ts) {
	unsigned char ucQuery;
	unsigned char pucCmd[80];
	int ret = 0;
	int test_read_count = 0;
	//pr_info("=entry IdentifyCapSensor=\n");
	//pr_info("=entry IdentifyCapSensor---name[%s]---addr[%x]-flags[%d]=\n",ts->client->name,ts->client->addr,ts->client->flags);
	i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	//pr_info("=IdentifyCapSensor read 0x80 =%d=\n",ucQuery);
	if (ucQuery < 0) {
		msleep(250);
		ucQuery = 0xFF;
	}
	while (ucQuery & 0x01) {
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
		if (ucQuery < 0) {
			ucQuery = 0xFF;
		}
	}
	//pr_info("=IdentifyCapSensor write cmd=\n");
	pucCmd[0] = 0x00;
	ret = i2cWriteToIt7260(ts->client, 0x20, pucCmd, 1);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
		return ret;
	}

	i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	if (ucQuery < 0) {
		ucQuery = 0xFF;
	}
	test_read_count = 0;
	while ((ucQuery & 0x01) && (test_read_count < 0x2000)) {
		test_read_count++;
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
		if (ucQuery < 0) {
			ucQuery = 0xFF;
		}
	}
	//pr_info("=IdentifyCapSensor write read id=\n");
	ret = i2cReadFromIt7260(ts->client, 0xA0, pucCmd, 8);
	//pr_info(
	//		"=IdentifyCapSensor read id--[%d][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x]=\n",
	//		ret, pucCmd[0], pucCmd[1], pucCmd[2], pucCmd[3], pucCmd[4],
	//		pucCmd[5], pucCmd[6], pucCmd[7], pucCmd[8], pucCmd[9]);

}

static int IT7260_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {
	struct IT7260_ts_data *ts;
	int ret = 0;
	struct IT7260_i2c_platform_data *pdata;
	unsigned long irqflags;
	unsigned char ucQuery = 0;
	u8 cmdbuf[2] = { 0x07, 0 };

	irqflags = IRQF_TRIGGER_HIGH;
	pr_info("=entry IT7260_ts_probe=\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "IT7260_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_check_functionality_failed;
	}
	ts->client = client;

	ts->debug_log_level = 0x3;
	ts->input_dev = input_dev;

	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	//ret=IdentifyCapSensor(ts);
	//if(ret<0)
	//	goto err_power_failed;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = IT7260_ts_early_suspend;
	ts->early_suspend.resume = IT7260_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	//IT7260_wq = create_singlethread_workqueue("IT7260_wq");
	IT7260_wq = create_workqueue("IT7260_wq");
	if (!IT7260_wq)
		goto err_check_functionality_failed;
	INIT_WORK(&ts->work, IT7260_ts_work_func);

	init_timer(&tp_timer) ;

	tp_timer.expires = jiffies + 30 * HZ;
	tp_timer.function = &tp_irq_handler_reg;

	add_timer(&tp_timer);

	client->irq = wmt_ts_gpt.irq;
	
	ts_set_gpio_int();

	if (client->irq) {
//		ret = request_irq(client->irq, IT7260_ts_irq_handler, IRQF_TRIGGER_LOW,
//		ret = request_irq(client->irq, IT7260_ts_irq_handler, IRQF_DISABLED | IRQF_TRIGGER_LOW,
//		ret = request_irq(client->irq, IT7260_ts_irq_handler, IRQF_SHARED | IRQF_TRIGGER_LOW,
//				client->name, ts);
		ret = request_irq(client->irq, IT7260_ts_irq_handler, IRQF_SHARED, client->name, ts);
		pr_info("IT7260_ts_probe-request_irq[%d]=\n", ret);
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	gl_ts = ts;
	pr_info("=end IT7260_ts_probe=\n");

	//To reset point queue.
	i2cWriteToIt7260(ts->client, 0x20, cmdbuf, 1);
	mdelay(10);
	i2cReadFromIt7260(ts->client, 0xA0, cmdbuf, 2);
	mdelay(10);

	ts_clear_irq();
	ts_enable_irq();
	return 0;
	err_power_failed: kfree(ts);

	err_check_functionality_failed: return ret;

}

static int IT7260_ts_remove(struct i2c_client *client) {
	return 0;
}

static int IT7260_ts_suspend(struct i2c_client *client, pm_message_t mesg) {
	int ret = 0;
	ts_suspend = 1;
/*
	u8 cmdbuf[] = { 0x04, 0x00, 0x02 };
*/

	printk(KERN_DEBUG "IT7260_ts_i2c call suspend\n");
	ts_disable_irq();
	cancel_work_sync(&gl_ts->work);
/*
	if (i2cWriteToIt7260(client, 0x20, cmdbuf, 3) >= 0)
		ret = 0;
	else
		ret = -1;
*/

	return ret;
}

static int IT7260_ts_resume(struct i2c_client *client) {
	unsigned char ucQuery;

	printk(KERN_DEBUG "IT7260_ts_i2c call resume\n");
	pulldown_intsrc_commended();
	i2cReadFromIt7260(client, 0x80, &ucQuery, 1);
	ts_set_gpio_int();
	ts_suspend = 0;
	ts_enable_irq();
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void IT7260_ts_early_suspend(struct early_suspend *h)
{
	struct IT7260_ts_data *ts;
	ts = container_of(h, struct IT7260_ts_data, early_suspend);
	IT7260_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void IT7260_ts_late_resume(struct early_suspend *h)
{
	struct IT7260_ts_data *ts;
	ts = container_of(h, struct IT7260_ts_data, early_suspend);
	IT7260_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id IT7260_ts_id[] = { { IT7260_I2C_NAME, 0 },
		{ } };

bool IT7260_Init(void) {
	int i;
	int tmp;
	unsigned char ucQuery = 0;
	unsigned char buffer[128];
	struct IT7260_ts_data *ts = gl_ts;

	// Identify Cap Sensor
	do {
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);
	buffer[0] = 0x00;
	i2cWriteToIt7260(ts->client, 0x20, buffer, 1);
	do {
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);

	memset(&buffer, 0, sizeof(buffer));
	i2cReadFromIt7260(ts->client, 0xA0, buffer, 8);
	pr_info("=IT7260_Init --[%x][%x][%x][%x][%x][%x]=\n", buffer[0], buffer[1],
			buffer[2], buffer[3], buffer[4], buffer[5]);
	if (buffer[1] != 'I' || buffer[2] != 'T' || buffer[3] != 'E') {
		//	return false;
	}

	// Get firmware information
	do {
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);
	buffer[0] = 0x01;
	buffer[1] = 0x00;
	i2cWriteToIt7260(ts->client, 0x20, buffer, 2);
	do {
		i2cReadFromIt7260(ts->client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);
	memset(&buffer, 0, sizeof(buffer));
	i2cReadFromIt7260(ts->client, 0xA0, buffer, 8);
	tmp = 0;
	//for (i = 5; i < 9; i++) {
	for (i = 5; i < 8; i++) {
		tmp += buffer[i];
	}
	if (tmp == 0) {
		//	return false;
	}

	//// Reinitialize Firmware
	//set_ite_i2c_nostop(1);
	//do {
	//	ucQuery = i2c_smbus_read_byte_data(ts->client, 0x80);
	//} while (ucQuery & 0x01);
	//buffer[0] = 0x6F;
	//set_ite_i2c_nostop(0);
	//i2c_smbus_write_byte_data(ts->client, 0x20, buffer[0]);

	return true;
}

static struct i2c_driver IT7260_ts_driver = { .probe = IT7260_ts_probe,
	.remove = IT7260_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = IT7260_ts_suspend,
	.resume = IT7260_ts_resume,
#endif
	.id_table = IT7260_ts_id,
	.driver = { .name = "IT7260-ts", },
};

struct ite7260_data {
	rwlock_t lock;
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

int ite7260_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg) {
	struct ite7260_data *dev = filp->private_data;
	int retval = 0;
	int i;
	unsigned char ucQuery;
	unsigned char buffer[MAX_BUFFER_SIZE];
	struct ioctl_cmd168 data;
	unsigned char datalen;
	unsigned char ent[] = {0x60, 0x00, 0x49, 0x54, 0x37, 0x32};
	unsigned char ext[] = {0x60, 0x80, 0x49, 0x54, 0x37, 0x32};

	//pr_info("=ite7260_ioctl=\n");
	memset(&data, 0, sizeof(struct ioctl_cmd168));

	switch (cmd) {
	case IOCTL_SET:
		//pr_info("=IOCTL_SET=\n");
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}
		buffer[0] = (unsigned char) data.bufferIndex;
		//pr_info("%.2X ", buffer[0]);
		for (i = 1; i < data.length + 1; i++) {
			buffer[i] = (unsigned char) data.buffer[i - 1];
			//pr_info("%.2X ", buffer[i]);
		}
		if (!memcmp(&(buffer[1]), ent, sizeof(ent))) {

			pr_info("Disabling IRQ.\n");

			//disable_irq(gl_ts->client->irq);
			ts_disable_irq();

		}

		if (!memcmp(&(buffer[1]), ext, sizeof(ext))) {

			pr_info("Enabling IRQ.\n");

			//enable_irq(gl_ts->client->irq);
			ts_enable_irq();

		}

		//pr_info("=================================================\n");
		//pr_info("name[%s]---addr[%x]-flags[%d]=\n",gl_ts->client->name,gl_ts->client->addr,gl_ts->client->flags);
		datalen = (unsigned char) (data.length + 1);
		//pr_info("datalen=%d\n", datalen);
		//write_lock(&dev->lock);
		retval = i2cWriteToIt7260(gl_ts->client,
				(unsigned char) data.bufferIndex, &(buffer[1]), datalen - 1);
		//write_unlock(&dev->lock);
		//pr_info("SET:retval=%x\n", retval);
		retval = 0;
		break;

	case IOCTL_GET:
		//pr_info("=IOCTL_GET=\n");
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		//pr_info("sizeof(struct ioctl_cmd168)=%d\n", sizeof(struct ioctl_cmd168));
		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}

		//pr_info("=================================================\n");
		//pr_info("name[%s]---addr[%x]-flags[%d]=\n",gl_ts->client->name,gl_ts->client->addr,gl_ts->client->flags);
		//read_lock(&dev->lock);
		retval = i2cReadFromIt7260(gl_ts->client,
				(unsigned char) data.bufferIndex, (unsigned char*) buffer,
				(unsigned char) data.length);
		//read_unlock(&dev->lock);
		//pr_info("GET:retval=%x\n", retval);
		retval = 0;
		for (i = 0; i < data.length; i++) {
			data.buffer[i] = (unsigned short) buffer[i];
		}
		//pr_info("GET:bufferIndex=%x, dataLength=%d, buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", data.bufferIndex, data.length, buffer[0], buffer[1], buffer[2], buffer[3]);
		//pr_info("GET:bufferIndex=%x, dataLength=%d, buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", data.bufferIndex, data.length, data.buffer[0], data.buffer[1], data.buffer[2], data.buffer[3]);
		//if (data.bufferIndex == 0x80)
		//	data.buffer[0] = 0x00;
		if ( copy_to_user((int __user *)arg, &data, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}
		break;

	default:
		retval = -ENOTTY;
		break;
	}

	done:
	//pr_info("DONE! retval=%d\n", retval);
	return (retval);
}

int ite7260_open(struct inode *inode, struct file *filp) {
	int i;
	struct ite7260_data *dev;

	pr_info("=ite7260_open=\n");
	dev = kmalloc(sizeof(struct ite7260_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	/* initialize members */
	rwlock_init(&dev->lock);
	for (i = 0; i < MAX_BUFFER_SIZE; i++) {
		dev->buffer[i] = 0xFF;
	}

	filp->private_data = dev;

	return 0; /* success */
}

int ite7260_close(struct inode *inode, struct file *filp) {
	struct ite7260_data *dev = filp->private_data;

	if (dev) {
		kfree(dev);
	}

	return 0; /* success */
}

struct file_operations ite7260_fops = { .owner = THIS_MODULE, .open =
		ite7260_open, .release = ite7260_close, .unlocked_ioctl = ite7260_ioctl, };

static void parse_arg(void)
{
	int retval;
	unsigned char buf[80];
	unsigned char tmp_buf[80];
	int i = 0;
	int j = 0;
	int varlen = 80;
	char *varname = "wmt.io.ts";
	retval = wmt_getsyspara(varname, buf, &varlen);
	if (retval == 0) {
		for (i = 0; i < 80; ++i) {
			if (buf[i] == ':')
				break;
			g_ModuleInstalled = (buf[i] - '0' == 1)?1:0;
		}
		++i;
		for (; i < 80; ++i) {
			if (buf[i] == ':')
				break;
			tmp_buf[j] = buf[i];
			++j;
		}
		if (!strncmp(tmp_buf,"it7260",6))
			g_is_it7260 = 1;
		else
			g_ModuleInstalled = 0;

	} else
		g_ModuleInstalled = 0;
	if (g_is_it7260 == 0)
		g_ModuleInstalled = 0;

	/*
	printk("g_ModuleInstalled = %d\n", g_ModuleInstalled);
	printk("is_ite7260 = %d\n", g_is_ite7260);
	printk("res_X = %d, res_Y = %d\n", res_x, res_y);
	*/
	
}

static struct i2c_board_info it7260_ts_i2c_board_info = {
	.type          = "IT7260",
	.flags         = 0x00,
	.addr          = 0x46,
	.platform_data = NULL,
	.archdata      = NULL,
	.irq           = -1,
};

static int __devinit IT7260_ts_init(void) {
	dev_t dev = MKDEV(ite7260_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;
	int input_err = 0;
	struct device *class_dev = NULL;
	struct i2c_board_info *it7260_i2c_bi;
    	struct i2c_adapter *adapter = NULL;
	struct i2c_client *client   = NULL;
	int ret = 0;

	parse_arg();
	parse_gpt_arg();
	if (g_ModuleInstalled == 0) {
		printk("ITE7260:it is not ITE touch\n");
		return -EIO;
	}
	DBG();

	it7260_i2c_bi = &it7260_ts_i2c_board_info;
	adapter = i2c_get_adapter(1);/*in bus 1*/
	if (NULL == adapter) {
		printk("can not get i2c adapter, client address error\n");
		ret = -ENODEV;
		return -1;
	}
	client = i2c_new_device(adapter, it7260_i2c_bi);
	if (client == NULL) {
		printk("allocate i2c client failed\n");
		ret = -ENOMEM;
		return -1;
	}
	i2c_put_adapter(adapter);
	//	if(!IT7260_Init()) {
	//		TS_DEBUG("IT7260 cannot be connected or is in firmware upgrade mode.\n");
	//		goto error;
	//	}

	alloc_ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (alloc_ret) {
		TS_DEBUG("IT7260 cdev can't get major number\n");
		goto error;
	}
	ite7260_major = MAJOR(dev);

	// allocate the character device
	cdev_init(&ite7260_cdev, &ite7260_fops);
	ite7260_cdev.owner = THIS_MODULE;
	ite7260_cdev.ops = &ite7260_fops;
	cdev_err = cdev_add(&ite7260_cdev, MKDEV(ite7260_major, ite7260_minor), 1);
	if(cdev_err) {
		goto error;
	}

	// register class
	ite7260_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(ite7260_class)) {
		TS_DEBUG("Err: failed in creating class.\n");
		goto error;
	}

	ite7260_dev = MKDEV(ite7260_major, ite7260_minor);
	class_dev = device_create(ite7260_class, NULL, ite7260_dev, NULL, DEVICE_NAME);
	if(class_dev == NULL)
	{
		TS_DEBUG("Err: failed in creating device.\n");
		goto error;
	}
	TS_DEBUG("=========================================\n");
	TS_DEBUG("register IT7260 cdev, major: %d, minor: %d \n", ite7260_major, ite7260_minor);
	TS_DEBUG("=========================================\n");

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		input_err = -ENOMEM;
		printk(KERN_ERR "IT7260_ts_probe: Failed to allocate input device\n");
		goto error;
	}
	input_dev->name = "IT7260";
	input_dev->phys = "I2C";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x7260;
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	//set_bit(BTN_2, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, SCREEN_X_RESOLUTION, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, SCREEN_Y_RESOLUTION, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 2, 0, 0);
	//input_set_abs_params(input_dev, ABS_X, 0, SCREEN_X_RESOLUTION, 0, 0);
	//input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_Y_RESOLUTION, 0, 0);
	//input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);
	input_err = input_register_device(input_dev);
	if (input_err) goto error;
	pr_info("it7260 driver is on###############################################################\n");

	return i2c_add_driver(&IT7260_ts_driver);

	error:
	if(cdev_err == 0) {
		cdev_del(&ite7260_cdev);
	}
	if(alloc_ret == 0) {
		unregister_chrdev_region(dev, 1);
	}
	if(input_dev) {
		input_free_device(input_dev);
	}
	if (IT7260_wq)
	destroy_workqueue(IT7260_wq);

	return -1;
}

static void __exit IT7260_ts_exit(void) {
	dev_t dev = MKDEV(ite7260_major, ite7260_minor);

	// unregister class
	device_destroy(ite7260_class, ite7260_dev);
	class_destroy(ite7260_class);

	// unregister driver handle
	cdev_del(&ite7260_cdev);
	unregister_chrdev_region(dev, 1);

	i2c_del_driver(&IT7260_ts_driver);
	if (IT7260_wq)
	destroy_workqueue(IT7260_wq);
}

module_init( IT7260_ts_init);
module_exit( IT7260_ts_exit);

MODULE_DESCRIPTION("IT7260 Touchscreen Driver");
MODULE_LICENSE("GPL");
