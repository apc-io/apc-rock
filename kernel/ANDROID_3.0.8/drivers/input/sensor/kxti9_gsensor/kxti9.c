/*
 * Copyright (C) 2009 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#ifdef KXTI9_INT_MODE
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#endif
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include "kxti9.h"
#include "../gsensor.h"

#define NAME			"g-sensor"
#define G_MAX			2048 //8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define INT_SRC_REG1		0x15
#define INT_STATUS_REG		0x16
#define TILT_POS_CUR		0x10
#define INT_REL			0x1A
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define DATA_CTRL		0x21
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define CTRL_REG3		0x1D
#define TILT_TIMER		0x28
#define WUF_TIMER		0x29
#define WUF_THRESH		0x5A
#define TDT_TIMER		0x2B
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x00
#define PC1_ON			0x80
/* INTERRUPT SOURCE 2 BITS */
#define TPS			0x01
#define TDTS0			0x04
#define TDTS1			0x08
/* INPUT_ABS CONSTANTS */
#define FUZZ			0 //32
#define FLAT			0 //32
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RES_TILT_TIMER		3
#define RES_CTRL_REG3		4
#define RES_WUF_TIMER		5
#define RES_WUF_THRESH		6
#define RES_TDT_TIMER		7
#define RES_TDT_H_THRESH	8
#define RES_TDT_L_THRESH	9
#define RES_TAP_TIMER		10
#define RES_TOTAL_TIMER		11
#define RES_LAT_TIMER		12
#define RES_WIN_TIMER		13
#define RESUME_ENTRIES		14

static struct gsensor_conf gs_conf;
static struct kxti9_data *gs_ti9;

extern int gsensor_i2c_register_device (void);

struct kxti9_platform_data kxti9_pdata = {
	.min_interval	= 1,
	.poll_interval	= 200,//1000,

	.g_range	= KXTI9_G_2G,
	.shift_adj	= SHIFT_ADJ_2G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,

	.data_odr_init		= ODR12_5F,
#ifdef KXTI9_INT_MODE
	.ctrl_reg1_init		= KXTI9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init		= KXTI9_IEN | KXTI9_IEA | KXTI9_IEL,
#else
	.ctrl_reg1_init		= KXTI9_G_2G | RES_12BIT,
	.int_ctrl_init		= 0,
#endif
	.tilt_timer_init	= 0x03,
	.engine_odr_init	= OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init		= 0x16,
	.wuf_thresh_init	= 0x28,
	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xFF,
	.tdt_l_thresh_init	= 0x14,
	.tdt_tap_timer_init	= 0x53,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x10,
	.tdt_window_timer_init	= 0xA0,
};

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
struct {
	unsigned int cutoff;
	u8 mask;
} kxti9_odr_table[] = {
	{
	3,	ODR800F}, {
	5,	ODR400F}, {
	10,	ODR200F}, {
	20,	ODR100F}, {
	40,	ODR50F}, {
	80,	ODR25F}, {
	0,	ODR12_5F},
};

struct kxti9_data {
	struct i2c_client *client;
	struct kxti9_platform_data *pdata;
	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;
#ifdef KXTI9_INT_MODE
	struct work_struct irq_work;
#endif

	int hw_initialized;
	atomic_t enabled;
	u8 resume[RESUME_ENTRIES];
	int res_interval;
#ifdef KXTI9_INT_MODE
	int irq;
#endif
};

static int kxti9_i2c_read(struct kxti9_data *ti9, u8 addr, u8 *data, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = ti9->client->addr,
		 .flags = ti9->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = &addr,
		 },
		{
		 .addr = ti9->client->addr,
		 .flags = (ti9->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};
	err = i2c_transfer(ti9->client->adapter, msgs, 2);

	if (err != 2)
		dev_err(&ti9->client->dev, "read transfer error\n");
	else
		err = 0;

	return err;
}

static int kxti9_i2c_write(struct kxti9_data *ti9, u8 addr, u8 *data, int len)
{
	int err;
	int i;
	u8 buf[len + 1];

	struct i2c_msg msgs[] = {
		{
		 .addr = ti9->client->addr,
		 .flags = ti9->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	buf[0] = addr;
	for (i = 0; i < len; i++) {
		buf[i + 1] = data[i];
	}

	err = i2c_transfer(ti9->client->adapter, msgs, 1);

	if (err != 1)
		dev_err(&ti9->client->dev, "write transfer error\n");
	else
		err = 0;

	return err;
}

static int kxti9_verify(struct kxti9_data *ti9)
{
	int err;
	u8 buf;

	err = kxti9_i2c_read(ti9, WHO_AM_I, &buf, 1);
	/*** DEBUG OUTPUT - REMOVE ***/
	dev_info(&ti9->client->dev, "WHO_AM_I = 0x%02x\n", buf);
	/*** <end> DEBUG OUTPUT - REMOVE ***/
	if (err < 0)
		dev_err(&ti9->client->dev, "read err int source\n");
	if (buf != 0x04 && buf != 0x08) // jakie add 0x8 for kxtj9
		err = -1;
	return err;
}

int kxti9_update_g_range(struct kxti9_data *ti9, u8 new_g_range)
{
	int err;
	u8 shift;
	u8 buf;

	switch (new_g_range) {
	case KXTI9_G_2G:
		shift = SHIFT_ADJ_2G;
		break;
	case KXTI9_G_4G:
		shift = SHIFT_ADJ_4G;
		break;
	case KXTI9_G_8G:
		shift = SHIFT_ADJ_8G;
		break;
	default:
		dev_err(&ti9->client->dev, "invalid g range request\n");
		return -EINVAL;
	}
	if (shift != ti9->pdata->shift_adj) {
		if (ti9->pdata->shift_adj > shift)
			ti9->resume[RES_WUF_THRESH] >>=
						(ti9->pdata->shift_adj - shift);
		if (ti9->pdata->shift_adj < shift)
			ti9->resume[RES_WUF_THRESH] <<=
						(shift - ti9->pdata->shift_adj);

		if (atomic_read(&ti9->enabled)) {
			buf = PC1_OFF;
			err = kxti9_i2c_write(ti9, CTRL_REG1, &buf, 1);
			if (err < 0)
				return err;
			buf = ti9->resume[RES_WUF_THRESH];
			err = kxti9_i2c_write(ti9, WUF_THRESH, &buf, 1);
			if (err < 0)
				return err;
			buf = (ti9->resume[RES_CTRL_REG1] & 0xE7) | new_g_range;
			err = kxti9_i2c_write(ti9, CTRL_REG1, &buf, 1);
			if (err < 0)
				return err;
			ti9->resume[RES_CTRL_REG1] = buf;
			ti9->pdata->shift_adj = shift;
		}
	}
	return 0;
}

int kxti9_update_odr(struct kxti9_data *ti9, int poll_interval)
{
	int err = -1;
	int i;
	u8 config;

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next slower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(kxti9_odr_table); i++) {
		config = kxti9_odr_table[i].mask;
		if (poll_interval < kxti9_odr_table[i].cutoff)
			break;
	}

	if (atomic_read(&ti9->enabled)) {
		err = kxti9_i2c_write(ti9, DATA_CTRL, &config, 1);
		if (err < 0)
			return err;
		/*
		 *  Latch on input_dev - indicates that kxti9_input_init passed
		 *  and this workqueue is available
		 */
		if (ti9->input_dev) {
			cancel_delayed_work_sync(&ti9->input_work);
			schedule_delayed_work(&ti9->input_work,
				      msecs_to_jiffies(poll_interval));
		}
	}
	ti9->resume[RES_DATA_CTRL] = config;

	return 0;
}

static int kxti9_hw_init(struct kxti9_data *ti9)
{
	int err = -1;
	u8 buf[7];

	buf[0] = PC1_OFF;
	err = kxti9_i2c_write(ti9, CTRL_REG1, buf, 1);
	if (err < 0)
		return err;
	err = kxti9_i2c_write(ti9, DATA_CTRL, &ti9->resume[RES_DATA_CTRL], 1);
	if (err < 0)
		return err;
	err = kxti9_i2c_write(ti9, CTRL_REG3, &ti9->resume[RES_CTRL_REG3], 1);
	if (err < 0)
		return err;
	err = kxti9_i2c_write(ti9, TILT_TIMER, &ti9->resume[RES_TILT_TIMER], 1);
	if (err < 0)
		return err;
	err = kxti9_i2c_write(ti9, WUF_TIMER, &ti9->resume[RES_WUF_TIMER], 1);
	if (err < 0)
		return err;
	err = kxti9_i2c_write(ti9, WUF_THRESH, &ti9->resume[RES_WUF_THRESH], 1);
	if (err < 0)
		return err;
	buf[0] = ti9->resume[RES_TDT_TIMER];
	buf[1] = ti9->resume[RES_TDT_H_THRESH];
	buf[2] = ti9->resume[RES_TDT_L_THRESH];
	buf[3] = ti9->resume[RES_TAP_TIMER];
	buf[4] = ti9->resume[RES_TOTAL_TIMER];
	buf[5] = ti9->resume[RES_LAT_TIMER];
	buf[6] = ti9->resume[RES_WIN_TIMER];
	err = kxti9_i2c_write(ti9, TDT_TIMER, buf, 7);
	if (err < 0)
		return err;
	err = kxti9_i2c_write(ti9, INT_CTRL1, &ti9->resume[RES_INT_CTRL1], 1);
	if (err < 0)
		return err;
	buf[0] = (ti9->resume[RES_CTRL_REG1] | PC1_ON);
	err = kxti9_i2c_write(ti9, CTRL_REG1, buf, 1);
	if (err < 0)
		return err;
	ti9->resume[RES_CTRL_REG1] = buf[0];
	ti9->hw_initialized = 1;

	return 0;
}

static void kxti9_device_power_off(struct kxti9_data *ti9)
{
	int err;
	u8 buf = PC1_OFF;

	err = kxti9_i2c_write(ti9, CTRL_REG1, &buf, 1);
	if (err < 0)
		dev_err(&ti9->client->dev, "soft power off failed\n");
#ifdef KXTI9_INT_MODE
	disable_irq(ti9->irq);
#endif
	if (ti9->pdata->power_off)
		ti9->pdata->power_off();
	ti9->hw_initialized = 0;
}

static int kxti9_device_power_on(struct kxti9_data *ti9)
{
	int err;

	if (ti9->pdata->power_on) {
		err = ti9->pdata->power_on();
		if (err < 0)
			return err;
	}
#ifdef KXTI9_INT_MODE
	enable_irq(ti9->irq);
#endif
	if (!ti9->hw_initialized) {
		msleep(100);
		err = kxti9_hw_init(ti9);
		if (err < 0) {
			kxti9_device_power_off(ti9);
			return err;
		}
	}

	return 0;
}

#ifdef KXTI9_INT_MODE
static irqreturn_t kxti9_isr(int irq, void *dev)
{
	struct kxti9_data *ti9 = dev;

	disable_irq_nosync(irq);
	schedule_work(&ti9->irq_work);

	return IRQ_HANDLED;
}
#endif

static u8 kxti9_resolve_dir(struct kxti9_data *ti9, u8 dir)
{
	switch (dir) {
	case 0x20:	/* -X */
		if (ti9->pdata->negate_x)
			dir = 0x10;
		if (ti9->pdata->axis_map_y == 0)
			dir >>= 2;
		if (ti9->pdata->axis_map_z == 0)
			dir >>= 4;
		break;
	case 0x10:	/* +X */
		if (ti9->pdata->negate_x)
			dir = 0x20;
		if (ti9->pdata->axis_map_y == 0)
			dir >>= 2;
		if (ti9->pdata->axis_map_z == 0)
			dir >>= 4;
		break;
	case 0x08:	/* -Y */
		if (ti9->pdata->negate_y)
			dir = 0x04;
		if (ti9->pdata->axis_map_x == 1)
			dir <<= 2;
		if (ti9->pdata->axis_map_z == 1)
			dir >>= 2;
		break;
	case 0x04:	/* +Y */
		if (ti9->pdata->negate_y)
			dir = 0x08;
		if (ti9->pdata->axis_map_x == 1)
			dir <<= 2;
		if (ti9->pdata->axis_map_z == 1)
			dir >>= 2;
		break;
	case 0x02:	/* -Z */
		if (ti9->pdata->negate_z)
			dir = 0x01;
		if (ti9->pdata->axis_map_x == 2)
			dir <<= 4;
		if (ti9->pdata->axis_map_y == 2)
			dir <<= 2;
		break;
	case 0x01:	/* +Z */
		if (ti9->pdata->negate_z)
			dir = 0x02;
		if (ti9->pdata->axis_map_x == 2)
			dir <<= 4;
		if (ti9->pdata->axis_map_y == 2)
			dir <<= 2;
		break;
	default:
		return -EINVAL;
	}

	return dir;
}

static int kxti9_get_acceleration_data(struct kxti9_data *ti9, int *xyz)
{
	int err;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware values */
	int hw_d[3];

	err = kxti9_i2c_read(ti9, XOUT_L, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (int) (((acc_data[1]) << 8) | acc_data[0]);
	hw_d[1] = (int) (((acc_data[3]) << 8) | acc_data[2]);
	hw_d[2] = (int) (((acc_data[5]) << 8) | acc_data[4]);

	hw_d[0] = (hw_d[0] & 0x8000) ? ((hw_d[0] | 0xFFFF0000) + 1) : (hw_d[0]);
	hw_d[1] = (hw_d[1] & 0x8000) ? ((hw_d[1] | 0xFFFF0000) + 1) : (hw_d[1]);
	hw_d[2] = (hw_d[2] & 0x8000) ? ((hw_d[2] | 0xFFFF0000) + 1) : (hw_d[2]);

	hw_d[0] >>= 4;
	hw_d[1] >>= 4;
	hw_d[2] >>= 4;

	xyz[0] = ((ti9->pdata->negate_x) ? (-hw_d[ti9->pdata->axis_map_x])
		  : (hw_d[ti9->pdata->axis_map_x]));
	xyz[1] = ((ti9->pdata->negate_y) ? (-hw_d[ti9->pdata->axis_map_y])
		  : (hw_d[ti9->pdata->axis_map_y]));
	xyz[2] = ((ti9->pdata->negate_z) ? (-hw_d[ti9->pdata->axis_map_z])
		  : (hw_d[ti9->pdata->axis_map_z]));

	/*** DEBUG OUTPUT - REMOVE ***/
	//dev_info(&ti9->client->dev, "x:%d y:%d z:%d\n", xyz[0], xyz[1], xyz[2]);
	/*** <end> DEBUG OUTPUT - REMOVE ***/

	return err;
}

static void kxti9_report_values(struct kxti9_data *ti9, int *xyz)
{
	input_report_abs(ti9->input_dev, ABS_X, xyz[gs_conf.xyz_axis[ABS_X][0]]*gs_conf.xyz_axis[ABS_X][1]);
	input_report_abs(ti9->input_dev, ABS_Y, xyz[gs_conf.xyz_axis[ABS_Y][0]]*gs_conf.xyz_axis[ABS_Y][1]);
	input_report_abs(ti9->input_dev, ABS_Z, xyz[gs_conf.xyz_axis[ABS_Z][0]]*gs_conf.xyz_axis[ABS_Z][1]);
	input_sync(ti9->input_dev);
}

#ifdef KXTI9_INT_MODE
static void kxti9_irq_work_func(struct work_struct *work)
{
/*
 *	int_status output:
 *	[INT_SRC_REG2][INT_SRC_REG1][TILT_POS_PRE][TILT_POS_CUR]
 *	INT_SRC_REG1, TILT_POS_PRE, and TILT_POS_CUR directions are translated
 *	based on platform data variables.
 */

	int err;
	int int_status = 0;
	u8 status;
	u8 buf[2];

	struct kxti9_data *ti9
			= container_of(work, struct kxti9_data, irq_work);

	err = kxti9_i2c_read(ti9, INT_STATUS_REG, &status, 1);
	if (err < 0)
		dev_err(&ti9->client->dev, "read err int source\n");
	int_status = status << 24;
	if ((status & TPS) > 0) {
		err = kxti9_i2c_read(ti9, TILT_POS_CUR, buf, 2);
		if (err < 0)
			dev_err(&ti9->client->dev, "read err tilt dir\n");
		int_status |= kxti9_resolve_dir(ti9, buf[0]);
		int_status |= kxti9_resolve_dir(ti9, buf[1]) << 8;
		/*** DEBUG OUTPUT - REMOVE ***/
		dev_info(&ti9->client->dev, "IRQ TILT [%x]\n",
						kxti9_resolve_dir(ti9, buf[0]));
		/*** <end> DEBUG OUTPUT - REMOVE ***/
	}
	if (((status & TDTS0) | (status & TDTS1)) > 0) {
		err = kxti9_i2c_read(ti9, INT_SRC_REG1, buf, 1);
		if (err < 0)
			dev_err(&ti9->client->dev, "read err tap dir\n");
		int_status |= (kxti9_resolve_dir(ti9, buf[0])) << 16;
		/*** DEBUG OUTPUT - REMOVE ***/
		dev_info(&ti9->client->dev, "IRQ TAP%d [%x]\n",
		((status & TDTS1) ? (2) : (1)), kxti9_resolve_dir(ti9, buf[0]));
		/*** <end> DEBUG OUTPUT - REMOVE ***/
	}
	/*** DEBUG OUTPUT - REMOVE ***/
	if ((status & 0x02) > 0) {
		if (((status & TDTS0) | (status & TDTS1)) > 0)
			dev_info(&ti9->client->dev, "IRQ WUF + TAP\n");
		else
			dev_info(&ti9->client->dev, "IRQ WUF\n");
	}
	/*** <end> DEBUG OUTPUT - REMOVE ***/
	if (int_status & 0x2FFF) {
		input_report_abs(ti9->input_dev, ABS_MISC, int_status);
		input_sync(ti9->input_dev);
	}
	err = kxti9_i2c_read(ti9, INT_REL, buf, 1);
	if (err < 0)
		dev_err(&ti9->client->dev,
				"error clearing interrupt status: %d\n", err);

	enable_irq(ti9->irq);
}
#endif

static int kxti9_enable(struct kxti9_data *ti9)
{
	int err;
	int int_status = 0;
	u8 buf;

	if (!atomic_cmpxchg(&ti9->enabled, 0, 1)) {
		err = kxti9_device_power_on(ti9);
		err = kxti9_i2c_read(ti9, INT_REL, &buf, 1);
		if (err < 0) {
			dev_err(&ti9->client->dev,
					"error clearing interrupt: %d\n", err);
			atomic_set(&ti9->enabled, 0);
			return err;
		}
		if ((ti9->resume[RES_CTRL_REG1] & TPE) > 0) {
			err = kxti9_i2c_read(ti9, TILT_POS_CUR, &buf, 1);
			if (err < 0) {
				dev_err(&ti9->client->dev,
					"read err current tilt\n");
			int_status |= kxti9_resolve_dir(ti9, buf);
			input_report_abs(ti9->input_dev, ABS_MISC, int_status);
			input_sync(ti9->input_dev);
			}
		}
		schedule_delayed_work(&ti9->input_work,
			msecs_to_jiffies(ti9->res_interval));
	}

	return 0;
}

static int kxti9_disable(struct kxti9_data *ti9)
{
	if (atomic_cmpxchg(&ti9->enabled, 1, 0)) {
		cancel_delayed_work_sync(&ti9->input_work);
		kxti9_device_power_off(ti9);
	}

	return 0;
}

static void kxti9_input_work_func(struct work_struct *work)
{
	struct kxti9_data *ti9 = container_of((struct delayed_work *)work,
						struct kxti9_data, input_work);
	int xyz[3] = { 0 };

	mutex_lock(&ti9->lock);

	if (kxti9_get_acceleration_data(ti9, xyz) == 0)
		kxti9_report_values(ti9, xyz);

	schedule_delayed_work(&ti9->input_work,
			msecs_to_jiffies(ti9->res_interval));

	mutex_unlock(&ti9->lock);
}

int kxti9_input_open(struct input_dev *input)
{
	struct kxti9_data *ti9 = input_get_drvdata(input);

	return kxti9_enable(ti9);
}

void kxti9_input_close(struct input_dev *dev)
{
	struct kxti9_data *ti9 = input_get_drvdata(dev);

	kxti9_disable(ti9);
}

static int kxti9_input_init(struct kxti9_data *ti9)
{
	int err;

	INIT_DELAYED_WORK(&ti9->input_work, kxti9_input_work_func);
	ti9->input_dev = input_allocate_device();
	if (!ti9->input_dev) {
		err = -ENOMEM;
		dev_err(&ti9->client->dev, "input device allocate failed\n");
		goto err0;
	}
	//ti9->input_dev->open = kxti9_input_open;
	//ti9->input_dev->close = kxti9_input_close;

	input_set_drvdata(ti9->input_dev, ti9);

	set_bit(EV_ABS, ti9->input_dev->evbit);
	set_bit(ABS_MISC, ti9->input_dev->absbit);

	input_set_abs_params(ti9->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(ti9->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(ti9->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	ti9->input_dev->name = "g-sensor";

	err = input_register_device(ti9->input_dev);
	if (err) {
		dev_err(&ti9->client->dev,
			"unable to register input polled device %s: %d\n",
			ti9->input_dev->name, err);
		goto err1;
	}

	return 0;
err1:
	input_free_device(ti9->input_dev);
err0:
	return err;
}

static void kxti9_input_cleanup(struct kxti9_data *ti9)
{
	input_unregister_device(ti9->input_dev);
}

/* sysfs */
static ssize_t kxti9_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", ti9->res_interval);
}

static ssize_t kxti9_delay_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);

	ti9->res_interval = max(val, ti9->pdata->min_interval);
	kxti9_update_odr(ti9, ti9->res_interval);

	return count;
}

static ssize_t kxti9_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", atomic_read(&ti9->enabled));
}

static ssize_t kxti9_enable_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		kxti9_enable(ti9);
	else
		kxti9_disable(ti9);
	return count;
}

static ssize_t kxti9_tilt_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	u8 tilt;

	if (ti9->resume[RES_CTRL_REG1] & TPE) {
		kxti9_i2c_read(ti9, TILT_POS_CUR, &tilt, 1);
		return sprintf(buf, "%d\n", kxti9_resolve_dir(ti9, tilt));
	} else {
		return sprintf(buf, "%d\n", 0);
	}
}

static ssize_t kxti9_tilt_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		ti9->resume[RES_CTRL_REG1] |= TPE;
	else
		ti9->resume[RES_CTRL_REG1] &= (~TPE);
	kxti9_i2c_write(ti9, CTRL_REG1, &ti9->resume[RES_CTRL_REG1], 1);
	return count;
}

static ssize_t kxti9_wake_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	u8 val = ti9->resume[RES_CTRL_REG1] & WUFE;
	if (val)
		return sprintf(buf, "%d\n", 1);
	else
		return sprintf(buf, "%d\n", 0);
}

static ssize_t kxti9_wake_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		ti9->resume[RES_CTRL_REG1] |= WUFE;
	else
		ti9->resume[RES_CTRL_REG1] &= (~WUFE);
	kxti9_i2c_write(ti9, CTRL_REG1, &ti9->resume[RES_CTRL_REG1], 1);
	return count;
}

static ssize_t kxti9_tap_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	u8 val = ti9->resume[RES_CTRL_REG1] & TDTE;
	if (val)
		return sprintf(buf, "%d\n", 1);
	else
		return sprintf(buf, "%d\n", 0);
}

static ssize_t kxti9_tap_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		ti9->resume[RES_CTRL_REG1] |= TDTE;
	else
		ti9->resume[RES_CTRL_REG1] &= (~TDTE);
	kxti9_i2c_write(ti9, CTRL_REG1, &ti9->resume[RES_CTRL_REG1], 1);
	return count;
}

static ssize_t kxti9_selftest_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	u8 ctrl = 0x00;
	if (val)
		ctrl = 0xCA;
	kxti9_i2c_write(ti9, 0x3A, &ctrl, 1);
	return count;
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, kxti9_delay_show, kxti9_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, kxti9_enable_show,
						kxti9_enable_store);
static DEVICE_ATTR(tilt, S_IRUGO|S_IWUSR, kxti9_tilt_show, kxti9_tilt_store);
static DEVICE_ATTR(wake, S_IRUGO|S_IWUSR, kxti9_wake_show, kxti9_wake_store);
static DEVICE_ATTR(tap, S_IRUGO|S_IWUSR, kxti9_tap_show, kxti9_tap_store);
static DEVICE_ATTR(selftest, S_IWUSR, NULL, kxti9_selftest_store);

static struct attribute *kxti9_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_tilt.attr,
	&dev_attr_wake.attr,
	&dev_attr_tap.attr,
	&dev_attr_selftest.attr,
	NULL
};

static struct attribute_group kxti9_attribute_group = {
	.attrs = kxti9_attributes
};
/* /sysfs */
static int __devinit kxti9_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int err = -1;
	struct kxti9_data *ti9 = kzalloc(sizeof(*ti9), GFP_KERNEL);
	gs_ti9 = ti9;
	if (ti9 == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}
	/*
	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		err = -ENODEV;
		goto err0;
	}
	*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}
	mutex_init(&ti9->lock);
	mutex_lock(&ti9->lock);
	ti9->client = client;
	i2c_set_clientdata(client, ti9);

#ifdef KXTI9_INT_MODE
	INIT_WORK(&ti9->irq_work, kxti9_irq_work_func);
#endif
	ti9->pdata = kmalloc(sizeof(*ti9->pdata), GFP_KERNEL);
	if (ti9->pdata == NULL)
		goto err1;

	err = sysfs_create_group(&client->dev.kobj, &kxti9_attribute_group);
	if (err)
		goto err1;

	//memcpy(ti9->pdata, client->dev.platform_data, sizeof(*ti9->pdata));
	memcpy(ti9->pdata, &kxti9_pdata, sizeof(*ti9->pdata));

	if (ti9->pdata->init) {
		err = ti9->pdata->init();
		if (err < 0)
			goto err2;
	}

#ifdef KXTI9_INT_MODE
	ti9->irq = gpio_to_irq(ti9->pdata->gpio);
#endif

	memset(ti9->resume, 0, ARRAY_SIZE(ti9->resume));
	ti9->resume[RES_DATA_CTRL] = ti9->pdata->data_odr_init;
	ti9->resume[RES_CTRL_REG1] = ti9->pdata->ctrl_reg1_init;
	ti9->resume[RES_INT_CTRL1] = ti9->pdata->int_ctrl_init;
	ti9->resume[RES_TILT_TIMER] = ti9->pdata->tilt_timer_init;
	ti9->resume[RES_CTRL_REG3] = ti9->pdata->engine_odr_init;
	ti9->resume[RES_WUF_TIMER] = ti9->pdata->wuf_timer_init;
	ti9->resume[RES_WUF_THRESH] = ti9->pdata->wuf_thresh_init;
	ti9->resume[RES_TDT_TIMER] = ti9->pdata->tdt_timer_init;
	ti9->resume[RES_TDT_H_THRESH] = ti9->pdata->tdt_h_thresh_init;
	ti9->resume[RES_TDT_L_THRESH] = ti9->pdata->tdt_l_thresh_init;
	ti9->resume[RES_TAP_TIMER] = ti9->pdata->tdt_tap_timer_init;
	ti9->resume[RES_TOTAL_TIMER] = ti9->pdata->tdt_total_timer_init;
	ti9->resume[RES_LAT_TIMER] = ti9->pdata->tdt_latency_timer_init;
	ti9->resume[RES_WIN_TIMER]    = ti9->pdata->tdt_window_timer_init;
	ti9->res_interval = ti9->pdata->poll_interval;

	err = kxti9_device_power_on(ti9);
	if (err < 0)
		goto err3;
	atomic_set(&ti9->enabled, 1);

	err = kxti9_verify(ti9);
	if (err < 0) {
		dev_err(&client->dev, "unresolved i2c client\n");
		goto err4;
	}

	err = kxti9_update_g_range(ti9, ti9->pdata->g_range);
	if (err < 0)
		goto err4;

	err = kxti9_update_odr(ti9, ti9->res_interval);
	if (err < 0)
		goto err4;

	err = kxti9_input_init(ti9);
	if (err < 0)
		goto err4;

	kxti9_device_power_off(ti9);
	atomic_set(&ti9->enabled, 0);

#ifdef KXTI9_INT_MODE
	err = request_irq(ti9->irq, kxti9_isr,
			IRQF_TRIGGER_RISING | IRQF_DISABLED, "kxti9-irq", ti9);
	if (err < 0) {
		pr_err("%s: request irq failed: %d\n", __func__, err);
		goto err5;
	}
	disable_irq_nosync(ti9->irq);
#endif

	mutex_unlock(&ti9->lock);

	return 0;

#ifdef KXTI9_INT_MODE
err5:
#endif
	kxti9_input_cleanup(ti9);
err4:
	kxti9_device_power_off(ti9);
err3:
	if (ti9->pdata->exit)
		ti9->pdata->exit();
err2:
	kfree(ti9->pdata);
	sysfs_remove_group(&client->dev.kobj, &kxti9_attribute_group);
err1:
	mutex_unlock(&ti9->lock);
	kfree(ti9);
err0:
	return err;
}

static int __devexit kxti9_remove(struct i2c_client *client)
{
	struct kxti9_data *ti9 = i2c_get_clientdata(client);

#ifdef KXTI9_INT_MODE
	free_irq(ti9->irq, ti9);
	gpio_free(ti9->pdata->gpio);
#endif
	kxti9_input_cleanup(ti9);
	kxti9_device_power_off(ti9);
	if (ti9->pdata->exit)
		ti9->pdata->exit();
	kfree(ti9->pdata);
	sysfs_remove_group(&client->dev.kobj, &kxti9_attribute_group);
	kfree(ti9);

	return 0;
}

#ifdef CONFIG_PM
static int kxti9_resume(struct i2c_client *client)
{
	//struct kxti9_data *ti9 = i2c_get_clientdata(client);
	//return kxti9_enable(ti9);
	return 0;
}

static int kxti9_suspend(struct i2c_client *client, pm_message_t mesg)
{
	//struct kxti9_data *ti9 = i2c_get_clientdata(client);
	//return kxti9_disable(ti9);
	return 0;
}

static void kxti9_shutdown(struct i2c_client *client)
{
	struct kxti9_data *ti9 = i2c_get_clientdata(client);
	if (atomic_read(&ti9->enabled)) {
		flush_delayed_work_sync(&ti9->input_work);
		cancel_delayed_work_sync(&ti9->input_work);
	}
}
#endif

static const struct i2c_device_id kxti9_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, kxti9_id);

static struct i2c_driver kxti9_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = kxti9_probe,
	.remove = __devexit_p(kxti9_remove),
	.resume = kxti9_resume,
	.suspend = kxti9_suspend,
	.shutdown = kxti9_shutdown,
	.id_table = kxti9_id,
};

int kxti9_enable_accel(int en)
{
	printk(KERN_DEBUG "%s: enable = %d\n", __func__, en);
    if (en)
        kxti9_enable(gs_ti9);
    else
        kxti9_disable(gs_ti9);
	return 0;
}

int kxti9_setDelay(int mdelay)
{
	printk(KERN_DEBUG "%s: delay = %d\n", __func__, mdelay);
	gs_ti9->res_interval = mdelay;
	return kxti9_update_odr(gs_ti9, gs_ti9->res_interval);
}

int kxti9_getLSG(int *lsg)
{
    int max_count;
	if (gs_ti9->resume[RES_CTRL_REG1] & RES_12BIT)
	    max_count = 2048;
	else
	    max_count = 128;

	if ((gs_ti9->resume[RES_CTRL_REG1] & 0x18) == KXTI9_G_2G)
	    *lsg = max_count >> 1;
	else if ((gs_ti9->resume[RES_CTRL_REG1] & 0x18) == KXTI9_G_4G)
	    *lsg = max_count >> 2;
	else if ((gs_ti9->resume[RES_CTRL_REG1] & 0x18) == KXTI9_G_8G)
	    *lsg = max_count >> 3;

	printk(KERN_DEBUG "%s: LSG = %d\n", __func__, *lsg);
	return 0;
}

struct gsensor_data kxti9_gs_data = {
	.i2c_addr = KXTI9_I2C_ADDR,
	.enable = kxti9_enable_accel,
	.setDelay = kxti9_setDelay,
	.getLSG = kxti9_getLSG,
};

static int __init kxti9_init(void)
{
	if (get_gsensor_conf(&gs_conf))
		return -1;

	if (gs_conf.op != 2)
		return -1;

	printk("G-Sensor kxti9 init\n");

	if (gsensor_register(&kxti9_gs_data))
		return -1;

	if (gsensor_i2c_register_device() < 0)
		return -1;

	return i2c_add_driver(&kxti9_driver);
}

static void __exit kxti9_exit(void)
{
	i2c_del_driver(&kxti9_driver);
}

module_init(kxti9_init);
module_exit(kxti9_exit);

MODULE_DESCRIPTION("KXTI9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
