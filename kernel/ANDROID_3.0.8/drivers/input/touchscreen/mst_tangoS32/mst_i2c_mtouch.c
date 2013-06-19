/* MST multitouch controller driver.
 *
 * Copyright(c) 2010 Master Touch Inc.
 *
 * Author: David <matt@0xlab.org>
 *
 */

#define DEBUG
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>

#include "mst_i2c_mtouch.h"

#define MST_DRIVER_NAME 	"mst_touch"

#define COORD_INTERPRET(MSB_BYTE, LSB_BYTE) \
		(MSB_BYTE << 8 | LSB_BYTE)

//#define MST_DEBUG

#undef dbg
#ifdef MST_DEBUG
	#define dbg(fmt,args...)  printk("DBG:%s_%d:"fmt,__FUNCTION__,__LINE__,##args)
#else
	#define dbg(fmt,args...)  
#endif

#undef dbg_err
#define dbg_err(fmt,args...)  printk("ERR:%s_%d:"fmt,__FUNCTION__,__LINE__,##args)

#ifdef CONFIG_MTD_WMT_SF
static int g_ModuleInstalled;
static int g_is_mst_tangos32;
static int res_x, res_y;
#endif
static int suspend_stage;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend mst_early_suspend;

static void mst_ts_early_suspend(struct early_suspend *h);
static void mst_ts_late_resume(struct early_suspend *h);
#endif
struct mst_data{
	__u16 	x, y, w, p;
	__u16 	x2, y2, w2, p2;
	struct input_dev *dev;
	struct mutex lock;
	int gpio;
	int irq;
	int x_resl;
	int y_resl;
	int nt;
	struct delayed_work work;
};

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

static struct wmt_gpt_s wmt_ts_gpt = {
	.name = "mst_tangos32",
	.bitmap = 0x200,/*GPIO 9*/
	.ctraddr = 0xd8110040 + WMT_MMAP_OFFSET,
	.ocaddr = 0xd8110080 + WMT_MMAP_OFFSET,
	.idaddr = 0xd8110000 + WMT_MMAP_OFFSET,
	.peaddr = 0xd8110480 + WMT_MMAP_OFFSET,
	.pcaddr = 0xd81104c0 + WMT_MMAP_OFFSET,
	.itbmp = 0x00008200, /* low level */
	.itaddr = 0xd8110308 + WMT_MMAP_OFFSET,
	.isbmp = 0x200,
	.isaddr = 0xd8110360 + WMT_MMAP_OFFSET,
	.irq = 5,
};

struct mst_data mst_context;

#define SET_GPIO_TS_INT() {\
	REG32_VAL(wmt_ts_gpt.ctraddr) |= wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.ocaddr) &= ~wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.peaddr) |= wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.pcaddr) |= wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.isaddr) = wmt_ts_gpt.isbmp; \
}
///////////////////////////////////////////////
//        GPIO SETUP REF      
///////////////////////////////////////////////
static int mst_setup_irq(unsigned int num) 
{
	if (wmt_ts_gpt.itbmp & 0xff) {
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff;
		REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & ~BIT7);
	} else if (wmt_ts_gpt.itbmp & 0xff00) {
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff00;
		REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & ~BIT15);
	} else if (wmt_ts_gpt.itbmp & 0xff0000) {
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff0000;
		REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & ~BIT23);
	} else if (wmt_ts_gpt.itbmp & 0xff000000) {
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff000000;
		REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & ~BIT31);
	}
	SET_GPIO_TS_INT();

	return 0;
}



static inline int mst_check_irq_status(void)
{
	return (REG32_VAL(wmt_ts_gpt.isaddr) & (wmt_ts_gpt.isbmp)); /*interrupt status*/
}

static int mst_clean_irq_status(unsigned int num)
{

	REG32_VAL(wmt_ts_gpt.isaddr) = wmt_ts_gpt.isbmp; /*clear interrupt status*/
	return 0;
}

static int mst_enable_irq(unsigned int num)
{
	REG32_VAL(wmt_ts_gpt.isaddr) = wmt_ts_gpt.isbmp; /*clear interrupt status*/
	if (suspend_stage == 0) {
		if (wmt_ts_gpt.itbmp & 0xff)
			REG32_VAL(wmt_ts_gpt.itaddr) |= 0x80;
		else if (wmt_ts_gpt.itbmp & 0xff00)
			REG32_VAL(wmt_ts_gpt.itaddr) |= 0x8000;
		else if (wmt_ts_gpt.itbmp & 0xff0000)
			REG32_VAL(wmt_ts_gpt.itaddr) |= 0x800000;
		else if (wmt_ts_gpt.itbmp & 0xff000000)
			REG32_VAL(wmt_ts_gpt.itaddr) |= 0x80000000;
	}

	return 0;
}

static int mst_disable_irq(unsigned int num)
{
	if (wmt_ts_gpt.itbmp & 0xff)
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x80;
	else if (wmt_ts_gpt.itbmp & 0xff00)
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x8000;
	else if (wmt_ts_gpt.itbmp & 0xff0000)
		REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x800000;
	else if (wmt_ts_gpt.itbmp & 0xff000000)
        	REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x80000000;
	return 0;
}


static inline int mst_check_irq_enable(void)
{
    if (wmt_ts_gpt.itbmp & 0xff)
            return REG32_VAL(wmt_ts_gpt.itaddr) & 0x80;
    else if (wmt_ts_gpt.itbmp & 0xff00)
            return REG32_VAL(wmt_ts_gpt.itaddr) & 0x8000;
    else if (wmt_ts_gpt.itbmp & 0xff0000)
            return REG32_VAL(wmt_ts_gpt.itaddr) & 0x800000;
    else if (wmt_ts_gpt.itbmp & 0xff000000)
            return REG32_VAL(wmt_ts_gpt.itaddr) & 0x80000000;
    else
            return 0;
}


static inline int mst_pen_isup(unsigned int gpio)
{
	return (REG32_VAL(wmt_ts_gpt.idaddr) & ((wmt_ts_gpt.bitmap))) ;
}

static int mst_gpio2irq(unsigned int gpio)
{
	return wmt_ts_gpt.irq; 
}

///////////////////////////////////////////////

extern int wmt_i2c_xfer_continue_if_4(struct i2c_msg *msg, unsigned int num, int bus_id);

static int __mst_reg_write(struct mst_data *mst, u8 reg, u8 value)
{
	int ret = 0;
	struct i2c_msg msg[1];
	unsigned char buf[2];

	buf[0] = reg;
	buf[1] = value;

	msg[0].addr = MST_I2C_ADDR;
	msg[0].flags = 0 ;
	msg[0].flags &= ~(I2C_M_RD);
	msg[0].len = 2;
	msg[0].buf = buf;

	ret = wmt_i2c_xfer_continue_if_4(msg, 1,MST_I2C_BUS_ID);
	if (ret <= 0)
		dbg("IIC xfer failed,errno=%d\n",ret);

    	return ret;
}

static int reg_write(struct mst_data *mst, u8 reg, u8 val)
{
	int timeout = 20;

	mutex_lock(&mst->lock);
	while (--timeout && __mst_reg_write(mst, reg, val) <= 0)
		udelay(100);
	mutex_unlock(&mst->lock);

	if (!timeout)
		dbg("IIC reg write error!\n");

	return timeout;
}

static int __mst_reg_read(struct mst_data *mst, u8 reg, u8* value)
{
	int ret = 0;
	struct i2c_msg msg[2];
	unsigned char buf[2];

	buf[0] = reg;
	buf[1] = 0x0;

	msg[0].addr = MST_I2C_ADDR;
	msg[0].flags = 0 ;
	msg[0].flags &= ~(I2C_M_RD);
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = MST_I2C_ADDR;
	msg[1].flags = 0 ;
	msg[1].flags |= (I2C_M_RD);
	msg[1].len = 1;
	msg[1].buf = value;

	ret = wmt_i2c_xfer_continue_if_4(msg, 2,MST_I2C_BUS_ID);
	if (ret <= 0)
		dbg("IIC xfer failed,errno=%d\n",ret);

    	return ret;

}

static u8 reg_read(struct mst_data *mst, u8 reg)
{
	int32_t  timeout = 20;
	u8 value = 0;
	
	mutex_lock(&mst->lock);
	while (--timeout && __mst_reg_read(mst, reg, &value)<=0);
		udelay(100);
	mutex_unlock(&mst->lock);
	
	if (!timeout)
		dbg("IIC reg read error!\n");

	return (value & 0xff);
}

static int reg_set_bit_mask(struct mst_data *mst,
			    u8 reg, u8 mask, u8 val)
{
	int ret = 0;
	u8 tmp;

	val &= mask;

	mutex_lock(&mst->lock);

	ret = __mst_reg_read(mst, reg,&tmp);
	if (ret <= 0)
		goto exit;
	
	tmp &= ~mask;
	tmp |= val;
	ret = __mst_reg_write(mst, reg, tmp);
exit:
	if (ret <= 0)
		dbg_err("Master IIC read/write Failed!\n");
	mutex_unlock(&mst->lock);

	return ret;
}


static const char *pwr_mod_str[] = {

	[MST_PWR_ACT] = "act",
	[MST_PWR_SLEEP] = "sleep",
	[MST_PWR_DSLEEP] = "dsleep",
	[MST_PWR_FREEZE] = "freeze",
};

static ssize_t cat_pwr_mod(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct mst_data *mst = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", pwr_mod_str[(reg_read(mst, MST_PWR_MODE) & 0x3)]);
}
static ssize_t echo_pwr_mod(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count)
{
	struct mst_data *mst = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(pwr_mod_str); i++) {
		if (!strncmp(buf, pwr_mod_str[i], strlen(pwr_mod_str[i]))) {
			reg_set_bit_mask(mst, MST_PWR_MODE, 0x3, i);
		}
	}
	return count;
}

static DEVICE_ATTR(pwr_mod, S_IRUGO | S_IWUSR, cat_pwr_mod, echo_pwr_mod);

static const char *int_mod_str[] = {

	[MST_INT_PERIOD] = "period",
	[MST_INT_FMOV] = "fmov",
	[MST_INT_FTOUCH] = "ftouch",
};

static ssize_t cat_int_mod(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct mst_data *mst = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", int_mod_str[(reg_read(mst, MST_INT_MODE) & 0x3)]);
}
static ssize_t echo_int_mod(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count)
{
	struct mst_data *mst = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(int_mod_str); i++) {
		if (!strncmp(buf, int_mod_str[i], strlen(int_mod_str[i])))
			reg_set_bit_mask(mst, MST_INT_MODE, 0x3, i);
	}
	return count;
}

static DEVICE_ATTR(int_mod, S_IRUGO | S_IWUSR, cat_int_mod, echo_int_mod);

static const char *int_trig_str[] = {

	[MST_INT_TRIG_LOW] = "low-act",
	[MST_INT_TRIG_HI] = "hi-act",
};

static ssize_t cat_int_trig(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct mst_data *mst = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n",
			int_trig_str[((reg_read(mst, MST_INT_MODE) >> 2)& 0x1)]);
}
static ssize_t echo_int_trig(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count)
{
	struct mst_data *mst = dev_get_drvdata(dev);
	u8 tmp;

	if (!strncmp(buf, int_trig_str[MST_INT_TRIG_LOW],
			strlen(int_trig_str[MST_INT_TRIG_LOW]))) {
		tmp = reg_read(mst, MST_INT_MODE);
		tmp &= ~(MST_INT_TRIG_LOW_EN);
		reg_write(mst, MST_INT_MODE, tmp);
	} else if (!strncmp(buf, int_trig_str[MST_INT_TRIG_HI],
			strlen(int_trig_str[MST_INT_TRIG_HI]))){
		tmp = reg_read(mst, MST_INT_MODE);
		tmp |= MST_INT_TRIG_LOW_EN;
		reg_write(mst, MST_INT_MODE, tmp);
	} else {
		dbg_err("trigger mode not support.\n");
	}
	return count;
}

static DEVICE_ATTR(int_trig, S_IRUGO | S_IWUSR, cat_int_trig, echo_int_trig);

static ssize_t cat_pwr_asleep(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct mst_data *mst = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",((reg_read(mst, MST_PWR_MODE) >> 2)& 0x1));
}
static ssize_t echo_pwr_asleep(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count)
{
	struct mst_data *mst = dev_get_drvdata(dev);
	unsigned int asleep_mod = simple_strtoul(buf, NULL, 10);
	u8 tmp;

	asleep_mod = !!asleep_mod;

	if (asleep_mod) {
		tmp = reg_read(mst, MST_PWR_MODE);
		tmp |= MST_PWR_ASLEEP_EN;
		reg_write(mst, MST_PWR_MODE, tmp);
	} else {
		tmp = reg_read(mst, MST_PWR_MODE);
		tmp &= ~(MST_PWR_ASLEEP_EN);
		reg_write(mst, MST_PWR_MODE, tmp);
	}
	return count;
}

static DEVICE_ATTR(pwr_asleep, S_IRUGO | S_IWUSR, cat_pwr_asleep, echo_pwr_asleep);

static ssize_t cat_int_width(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct mst_data *mst = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",reg_read(mst, MST_INT_WIDTH));
}

static ssize_t echo_int_width(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count)
{
	struct mst_data *mst = dev_get_drvdata(dev);
	unsigned int int_width = simple_strtoul(buf, NULL, 10);

	int_width &= 0xff;

	reg_write(mst, MST_INT_WIDTH, int_width);
	return count;
}

static DEVICE_ATTR(int_width, S_IRUGO | S_IWUSR, cat_int_width, echo_int_width);

static struct attribute *mst_attributes[] = {
	&dev_attr_pwr_mod.attr,
	&dev_attr_pwr_asleep.attr,
	&dev_attr_int_mod.attr,
	&dev_attr_int_trig.attr,
	&dev_attr_int_width.attr,
	NULL
};
static const struct attribute_group mst_group = {
	.attrs = mst_attributes,
};

static int mst_calibrate(void)
{
	int ret = 0;
#if 0
	struct mst_data *mst = &mst_context;
	
	/* auto calibrate */
	ret = reg_write(mst,MST_SPECOP,0x03);
	msleep(1000);
#endif

	return ret?0:-1;
}

static inline void mst_report(struct mst_data *mst)
{
	//if (mst->x||mst->y||mst->x2||mst->y2) {
		//input_report_abs(mst->dev, ABS_MT_TOUCH_MAJOR, mst->w);
		input_report_abs(mst->dev, ABS_MT_POSITION_X, mst->x);
		input_report_abs(mst->dev, ABS_MT_POSITION_Y, mst->y);
		input_report_abs(mst->dev, ABS_MT_TRACKING_ID, 0);
		input_mt_sync(mst->dev);

		if (mst->nt==2) {
			//input_report_abs(mst->dev, ABS_MT_TOUCH_MAJOR, mst->w2);
			input_report_abs(mst->dev, ABS_MT_POSITION_X, mst->x2);
			input_report_abs(mst->dev, ABS_MT_POSITION_Y, mst->y2);
			input_report_abs(mst->dev, ABS_MT_TRACKING_ID, 1);
			input_mt_sync(mst->dev);
		}
	
		input_report_key(mst->dev, BTN_TOUCH, 1);
		input_sync(mst->dev);

	//}
	
	return;
}

static void mst_i2c_work(struct work_struct *work)
{
	struct mst_data *mst =
			container_of((struct delayed_work*)work, struct mst_data, work);

	mst->nt = reg_read(mst, MST_TOUCH);
	
	if (mst_pen_isup(mst->gpio)||mst->nt==0) {
		//input_report_abs(mst->dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_key(mst->dev, BTN_TOUCH, 0);
		input_mt_sync(mst->dev);
		input_sync(mst->dev);
		mst_enable_irq(mst->gpio);
		return;
	}

	if (mst->nt==2) {
		
		mst->x = COORD_INTERPRET(reg_read(mst, MST_POS_X_HI),
				reg_read(mst, MST_POS_X_LOW));
		mst->y = COORD_INTERPRET(reg_read(mst, MST_POS_Y_HI),
				reg_read(mst, MST_POS_Y_LOW));
		/*
		mst->p = COORD_INTERPRET(reg_read(mst, MST_STRNGTH_HI),
				reg_read(mst, MST_STRNGTH_LOW));
		*/
		mst->w = reg_read(mst, MST_AREA_X);

		mst->x2 = COORD_INTERPRET(reg_read(mst, MST_POS_X2_HI),
				reg_read(mst, MST_POS_X2_LOW));
		mst->y2 = COORD_INTERPRET(reg_read(mst, MST_POS_Y2_HI),
				reg_read(mst, MST_POS_Y2_LOW));
		/*
		mst->p2 = COORD_INTERPRET(reg_read(mst, MST_STRNGTH_HI),
				reg_read(mst, MST_STRNGTH_LOW));
		*/

		mst->w2 = reg_read(mst, MST_AREA_X2);

		dbg("  x=%-4d,y=%-4d,w=%-4d,   x2=%-4d,y2=%-4d,w2=%-4d, ntouch=%d\n",
			mst->x, mst->y, mst->w,  mst->x2, mst->y2, mst->w2,mst->nt);

		if (!(mst->w == 0 || mst->w2 == 0))
			mst_report(mst);

	} else if (mst->nt == 1) {
		
		mst->x = COORD_INTERPRET(reg_read(mst, MST_POS_X_HI),
				reg_read(mst, MST_POS_X_LOW));
		mst->y = COORD_INTERPRET(reg_read(mst, MST_POS_Y_HI),
				reg_read(mst, MST_POS_Y_LOW));
		/*
		mst->p = COORD_INTERPRET(reg_read(mst, MST_STRNGTH_HI),
				reg_read(mst, MST_STRNGTH_LOW));
		*/
		mst->w = reg_read(mst, MST_AREA_X);
		dbg("  x=%-4d,y=%-4d,w=%-4d, ntouch=%d\n",mst->x, mst->y, mst->w,mst->nt);
		if (mst->w != 0)
			mst_report(mst);
	} else {
		dbg("Mst loop error!\n");

	}

	if (suspend_stage == 0)
		schedule_delayed_work(&mst->work, msecs_to_jiffies(20));
	return;
	
}

static irqreturn_t mst_irq_handler(int irq, void *_mst)
{
	struct mst_data *mst = _mst;

	if(!mst_check_irq_enable())
		return IRQ_NONE;
	if(mst_check_irq_status() == 0)
		return IRQ_NONE;
	mst_disable_irq(mst->gpio);
	mst_clean_irq_status(mst->gpio);

	udelay(20);
	if (mst_pen_isup(mst->gpio)) {
		dbg("get touch  interrupt ,but check pen is up ?!\n");
		mst_enable_irq(mst->gpio);
		return IRQ_HANDLED;
	}

	dbg("======== pen is down ========\n");
	schedule_delayed_work(&mst->work,0);
	
	return IRQ_HANDLED;

}


static int mst_probe(struct platform_device *pdev)
{
	struct mst_data *mst;
	struct input_dev *input_dev;
	int err = 0;
	char tmp=0;
	
	mst = &mst_context;
	dev_set_drvdata(&pdev->dev, mst);
	mutex_init(&mst->lock);

	/* allocate input device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dbg_err( "Failed to allocate input device \n");
		return -ENODEV;
	}
	
	
	
	mst->dev = input_dev;
	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) |
					BIT_MASK(EV_ABS);

	__set_bit(BTN_TOUCH, input_dev->keybit);	
	//input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 256, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
						0, mst->x_resl, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
						0, mst->y_resl, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 2, 0, 0);
	input_dev->name = pdev->name;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &pdev->dev;

	err = input_register_device(input_dev);
	if (err){
		dbg_err("register input device failed!\n");
		goto exit_input;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	mst_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	mst_early_suspend.suspend = mst_ts_early_suspend;
	mst_early_suspend.resume = mst_ts_late_resume;
	register_early_suspend(&mst_early_suspend);
#endif
	INIT_DELAYED_WORK(&mst->work, mst_i2c_work);

#if 0
	/* auto calibrate */
	reg_write(mst,MST_SPECOP,0x03);
	msleep(1000);
#endif

	/* setup INT mode */
	reg_write(mst, MST_INT_MODE, 0x0a);

	tmp = reg_read(mst, MST_INT_MODE);
	tmp &= ~(MST_INT_TRIG_LOW_EN);
	reg_write(mst, MST_INT_MODE, tmp);

	mst_setup_irq(mst->gpio);
	err = request_irq(mst->irq, mst_irq_handler,IRQF_SHARED, MST_DRIVER_NAME, mst);
	if(err){
		dbg_err("Master request irq failed!");
		goto exit_input;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &mst_group);
	if (err < 0){
		dbg_err("Create sysfs failed!\n");
		goto exit_input;
	}
	mst_enable_irq(mst->gpio);

	return 0;

exit_input:
	input_unregister_device(mst->dev);
	
	return err;
}

static int __devexit mst_remove(struct platform_device *pdev)
{
	struct mst_data *mst = dev_get_drvdata(&pdev->dev);

	mst_disable_irq(mst->gpio);
	free_irq(mst->irq, mst);

	cancel_delayed_work_sync(&mst->work);
	
	input_unregister_device(mst->dev);
	sysfs_remove_group(&pdev->dev.kobj, &mst_group);

	mutex_destroy(&mst->lock);
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void mst_ts_early_suspend(struct early_suspend *h)
{
	struct mst_data *mst = &mst_context;
	suspend_stage = 1;

	mst_disable_irq(mst->gpio);
	cancel_delayed_work_sync(&mst->work);
	/*
	reg_set_bit_mask(mst, MST_PWR_MODE, MST_PWR_DSLEEP, 0x03);
	*/
	/*
	mst_disable_irq(mst->gpio);
	*/
}
void mst_ts_late_resume(struct early_suspend *h)
{
	int tmp = 0;
	struct mst_data *mst = &mst_context;
	/*
	reg_set_bit_mask(mst, MST_PWR_MODE, MST_PWR_ACT, 0x03);
	*/

	/* setup INT mode */
	reg_write(mst, MST_INT_MODE, 0x0a);

	tmp = reg_read(mst, MST_INT_MODE);
	tmp &= ~(MST_INT_TRIG_LOW_EN);
	reg_write(mst, MST_INT_MODE, tmp);

	mst_setup_irq(mst->gpio);
	suspend_stage = 0;
	mst_enable_irq(mst->gpio);
}
#endif
#ifdef CONFIG_PM
static int mst_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int mst_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define mst_suspend NULL
#define mst_resume NULL
#endif

static void mst_release(struct device *device)
{
    return;
}

static struct platform_device mst_device = {
	.name  	= MST_DRIVER_NAME,
	.id       	= 0,
	.dev    	= {.release = mst_release},
};

static struct platform_driver mst_driver = {
	.driver = {
	             .name    	= MST_DRIVER_NAME,
		       .owner	= THIS_MODULE,
	 },
	.probe     = mst_probe,
	.remove   = mst_remove,
	.suspend  = mst_suspend,
	.resume   = mst_resume,
};


/////////////////////////////////////////////
//          char device register reference
/////////////////////////////////////////////
static struct class* l_dev_class = NULL;
static struct device *l_clsdevice = NULL;

#define MST_DEV_NAME 				 "wmtts"
#define MST_MAJOR					11

#define TS_IOC_MAGIC  				't'
#define TS_IOCTL_CAL_START    		_IO(TS_IOC_MAGIC,   1)
#define TS_IOCTL_CAL_DONE     		_IOW(TS_IOC_MAGIC,  2, int*)
#define TS_IOCTL_GET_RAWDATA  		_IOR(TS_IOC_MAGIC,  3, int*)
#define TS_IOCTL_CAL_QUIT			_IOW(TS_IOC_MAGIC,  4, int*)
#define TS_IOCTL_CAPACITANCE_CAL	_IOW(TS_IOC_MAGIC,  5, int*)
#define TS_IOC_MAXNR          			5

static int mst_open(struct inode *inode, struct file *filp)
{
	dbg("wmt ts driver opening...\n");

	return 0;
}

static int mst_close(struct inode *inode, struct file *filp)
{
	dbg("wmt ts driver closing...\n");

	return 0;
}

static long mst_ioctl(struct file *dev, unsigned int cmd, unsigned long arg)
{

	if (_IOC_TYPE(cmd) != TS_IOC_MAGIC) {
		dbg("CMD ERROR!");
		return -ENOTTY;
	}
	
	if (_IOC_NR(cmd) > TS_IOC_MAXNR) {
		dbg("NO SUCH IO CMD!\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case TS_IOCTL_CAL_START:
		printk("MST: TS_IOCTL_CAL_START\n");
		
		return 0;
		
	case TS_IOCTL_CAL_DONE:
		printk("MST: TS_IOCTL_CAL_DONE\n");

		return 0;
		
	case TS_IOCTL_CAL_QUIT:
		printk("MST: TS_IOCTL_CAL_QUIT\n");
		
		return 0;
		
	case TS_IOCTL_GET_RAWDATA:
		printk("MST: TS_IOCTL_GET_RAWDATA\n");

		return 0;
	case TS_IOCTL_CAPACITANCE_CAL:
		printk("MST: TS_IOCTL_CAPACITANCE_CAL\n");
		if(mst_calibrate()){
			dbg("MST: calibrate failed!\n");
			return -EINVAL;
		}

		return 0;
			
	}
	
	return -EINVAL;
}

static ssize_t mst_read(struct file *filp, char *buf, size_t count, loff_t *l)
{

	return 0;
}


static struct file_operations mst_fops = {
	.read           	= mst_read,
	.compat_ioctl        	= mst_ioctl,
	.open       	= mst_open,
	.release      	= mst_close,
};

static int mst_register_chdev(void)
{
	int ret =0;
	
	/* Create device node*/
	if (register_chrdev (MST_MAJOR, MST_DEV_NAME, &mst_fops)) {
		printk (KERN_ERR "wmt touch: unable to get major %d\n", MST_MAJOR);
		return -EIO;
	}	
	
	l_dev_class = class_create(THIS_MODULE, MST_DEV_NAME);
	if (IS_ERR(l_dev_class)) {
		unregister_chrdev(MST_MAJOR, MST_DEV_NAME);
		ret = PTR_ERR(l_dev_class);
		printk(KERN_ERR "Can't class_create touch device !!\n");
		return ret;
	}
	
	l_clsdevice = device_create(l_dev_class, NULL, MKDEV(MST_MAJOR, 0), NULL, MST_DEV_NAME);
	if (IS_ERR(l_clsdevice)) {
		unregister_chrdev(MST_MAJOR, MST_DEV_NAME);
		class_destroy(l_dev_class);
		ret = PTR_ERR(l_clsdevice);
		printk(KERN_ERR "Failed to create device %s !!!",MST_DEV_NAME);
		return ret;
	}

	return 0;
}


static int mst_unregister_chdev(void)
{	
	device_destroy(l_dev_class, MKDEV(MST_MAJOR, 0));
	unregister_chrdev(MST_MAJOR, MST_DEV_NAME);
	class_destroy(l_dev_class);

	return 0;
}
#ifdef CONFIG_MTD_WMT_SF
extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);
#endif

#ifdef CONFIG_MTD_WMT_SF
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
		if (!strncmp(tmp_buf,"mst_tangos32",12))
			g_is_mst_tangos32 = 1;
		else
			g_ModuleInstalled = 0;

		/*
		tmp_buf[j] = '\0';
		printk("id = %s\n", tmp_buf);
		*/
		++i;
		j = 0;
		for (; i < 80; ++i) {
			if (buf[i] == ':')
				break;
			tmp_buf[j] = buf[i];
			++j;
		}
		/*
		tmp_buf[j] = '\0';
		printk("bus is %s\n", tmp_buf);
		*/
		++i;
		sscanf((buf+i), "%d:%d",
			&res_x,
			&res_y);

	} else
		g_ModuleInstalled = 0;
	if (g_is_mst_tangos32 == 0)
		g_ModuleInstalled = 0;

	/*
	printk("g_ModuleInstalled = %d\n", g_ModuleInstalled);
	printk("is_mst_tangos32 = %d\n", g_is_mst_tangos32);
	printk("res_X = %d, res_Y = %d\n", res_x, res_y);
	*/
	
}
#endif

static int check_touch_env(struct mst_data *mst)
{
	/*
    	 Get u-boot parameter
	*/
#ifdef CONFIG_MTD_WMT_SF
	parse_arg();

	if (!g_ModuleInstalled){
		printk("MST:it is not MST touch\n");
		return -EIO;
	}
	mst->x_resl = res_x;
	mst->y_resl = res_y;
	mst->gpio = 9;/*use gpio 9*/
	
	mst->irq = mst_gpio2irq(mst->gpio);
#endif
#ifndef CONFIG_MTD_WMT_SF
        mst->x_resl = 1024;
        mst->y_resl = 600;
        mst->gpio = 9;/*use gpio 9*/

        mst->irq = mst_gpio2irq(mst->gpio);

#endif
	printk("p.x = %d, p.y = %d, gpio=%d, irq=%d\n", mst->x_resl, mst->y_resl, mst->gpio,mst->irq);

	if (mst->irq < 0) {
		printk("MST: map gpio irq error ?!\n");
		return -ENODEV;
	}

	return 0;
}

#ifdef CONFIG_MTD_WMT_SF
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
	if (strncmp(buf,"mst_tangos32",12))
		return ;
	i += 13;
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
	/*
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
	*/
}
#endif
static int __init mst_init(void)
{
	int ret = 0;

	memset(&mst_context,0,sizeof(struct mst_data));
	
#ifdef CONFIG_MTD_WMT_SF
	parse_gpt_arg();
#endif

	if (check_touch_env(&mst_context))
		return -EIO;

	ret = platform_device_register(&mst_device);
	if (ret) {
		dbg_err("register platform drivver failed!\n");

		return ret;
	}
	
	ret = platform_driver_register(&mst_driver);
	if (ret) {
		dbg_err("register platform device failed!\n");
		goto exit_unregister_pdev;
	}

	ret = mst_register_chdev();
	if (ret) {
		dbg_err("register char devcie failed!\n");
		goto exit_unregister_pdrv;
	}
	
	return ret;
	
exit_unregister_pdrv:
	platform_driver_unregister(&mst_driver);
exit_unregister_pdev:
	platform_device_unregister(&mst_device);

	return ret;
	
}

static void mst_exit(void)
{
	platform_driver_unregister(&mst_driver);
	platform_device_unregister(&mst_device);
	mst_unregister_chdev();

	return;
}
module_init(mst_init);
module_exit(mst_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MasterTouch");
