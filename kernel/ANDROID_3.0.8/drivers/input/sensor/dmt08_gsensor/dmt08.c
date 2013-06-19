/*
 * @file drivers/misc/dmt0308.c
 * @brief DMT g-sensor Linux device driver
 * @author Domintech Technology Co., Ltd (http://www.domintech.com.tw)
 * @version 1.31
 * @date 2012/3/27
 *
 * @section LICENSE
 *
 *  Copyright 2011 Domintech Technology Co., Ltd
 *
 * 	This software is licensed under the terms of the GNU General Public
 * 	License version 2, as published by the Free Software Foundation, and
 * 	may be copied, distributed, and modified under those terms.
 *
 * 	This program is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/serio.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/syscalls.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include "dmt08.h"
#include "../gsensor.h"

wait_queue_head_t	open_wq;
atomic_t		active;
static unsigned int interval;
struct mutex DMT_mutex;
struct delayed_work work;
struct work_struct irq_work;
atomic_t delay;

static struct gsensor_conf gs_conf;

void gsensor_write_offset_to_file(void);
void gsensor_read_offset_from_file(void);
char OffsetFileName[] = "/data/misc/dmt/offset.txt";
static int Device_First_Time_Opened_flag=1;
//*************************************************
static char const *const ACCELEMETER_CLASS_NAME = "accelemeter";
#define CHIP_ENABLE    137
#if (defined(CONFIG_SENSORS_DMARD03) || defined(CONFIG_SENSORS_DMARD03_MODULE))
static char const *const GSENSOR_DEVICE_NAME = "dmard03";
#elif (defined(CONFIG_SENSORS_DMARD08) || defined(CONFIG_SENSORS_DMARD08_MODULE) || defined(CONFIG_WMT_SENSOR_DMT08))
static char const *const GSENSOR_DEVICE_NAME = "dmard08";
#endif

static int device_init(void);
static void device_exit(void);

static int device_open(struct inode*, struct file*);
static ssize_t device_write(struct file*, const char*, size_t, loff_t*);
static ssize_t device_read(struct file*, char*, size_t, loff_t*);
static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int device_close(struct inode*, struct file*);

//static int device_i2c_suspend(struct i2c_client *client, pm_message_t mesg);
//static int device_i2c_resume(struct i2c_client *client);
static int __devinit device_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit device_i2c_remove(struct i2c_client *client);
static inline void device_i2c_correct_accel_sign(s16 *val);
void device_i2c_read_xyz(struct i2c_client *client, s16 *xyz);
void device_i2c_merge_register_values(struct i2c_client *client, s16 *val, u8 msb, u8 lsb);

struct input_dev 	*input;

static int DMT_GetOpenStatus(void)
{
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
	printk("%s:start active=%d\n",__func__,active.counter);
#endif	
	wait_event_interruptible(open_wq, (atomic_read(&active) != 0));
	return 0;
}

static int DMT_GetCloseStatus(void)
{
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
	printk("%s:start active=%d\n",__func__,active.counter);
#endif
	wait_event_interruptible(open_wq, (atomic_read(&active) <= 0));
	return 0;
}

static void DMT_sysfs_update_active_status(int en)
{
	unsigned long dmt_delay;
	if(en)
	{
		dmt_delay = msecs_to_jiffies(atomic_read(&delay));
		if(dmt_delay < 1)
			dmt_delay = 1;
		//printk("schedule_delayed_work start with delay time=%lu\n",dmt_delay);
		schedule_delayed_work(&work,dmt_delay);
	}
	else 
		cancel_delayed_work_sync(&work);
}

static ssize_t DMT_enable_acc_show( struct device *dev, struct device_attribute *attr, char *buf)
{
	buf="show";
	return 1;
}

static ssize_t DMT_enable_acc_store( struct device *dev, struct device_attribute *attr, char const *buf, size_t count)
{
	int en;
#if DMT_DEBUG_DATA
	printk("%s:buf=%x %x\n",__func__,buf[0],buf[1]);
#endif	
	if(buf[0]!= 0x30 && buf[0]!= 0x31)
	{
	 printk("%s:illegle data !!\n",__func__);
	 return 0;
	} 
	en= (buf[0]-0x30 > 0) ? 1:0;
	DMT_sysfs_update_active_status(en);
	return 1;
}

static ssize_t DMT_delay_acc_show( struct device *dev, struct device_attribute *attr, char *buf){
	return 1;
}

static ssize_t DMT_delay_acc_store( struct device *dev, struct device_attribute *attr,char const *buf, size_t count)
{
	int error;
	unsigned long data;
	error = strict_strtoul(buf, 10, &data);
	if(error) {
		pr_err("%s strict_strtoul error\n", __FUNCTION__);
		return -1;
	}
	mutex_lock(&DMT_mutex);
	interval=(unsigned int)data;       
	mutex_unlock(&DMT_mutex);
	atomic_set(&delay, (unsigned int) data);
#if DMT_DEBUG_DATA
	printk("Driver attribute set delay =%lu\n",data);
#endif
	return 1;
}

static struct device_attribute DMT_attributes[] = {
	__ATTR(enable_acc, 0755, DMT_enable_acc_show, DMT_enable_acc_store),
	__ATTR(delay_acc,  0755, DMT_delay_acc_show,  DMT_delay_acc_store),
	__ATTR_NULL,
};

static int create_device_attributes(struct device *dev,	struct device_attribute *attrs)
{
	int i;
	int err = 0;
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
#endif	
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

int input_init(void)
{
	int err=0;
	input=input_allocate_device();
	if (!input)
		return -ENOMEM;
	else
		printk("input device allocate Success !!\n");
	/* Setup input device */
	set_bit(EV_ABS, input->evbit);
	/* Accelerometer [-78.5, 78.5]m/s2 in Q16*/
	input_set_abs_params(input, ABS_X, -1024, 1024, 0, 0);
	input_set_abs_params(input, ABS_Y, -1024, 1024, 0, 0);
	input_set_abs_params(input, ABS_Z, -1024, 1024, 0, 0);

	/* Set name */
	input->name = "g-sensor";

	/* Register */
	err = input_register_device(input);
	if (err) {
		input_free_device(input);
		return err;
	}
	atomic_set(&active, 0);
#if DMT_DEBUG_DATA
	printk("in driver ,active=%d\n",active.counter);
#endif
	init_waitqueue_head(&open_wq);

	return err;
}

typedef union {
	struct {
		s16	x;
		s16	y;
		s16	z;
	} u;
	s16	v[SENSOR_DATA_SIZE];
} raw_data;
static raw_data offset;

struct dev_data {
	dev_t devno;
	struct cdev cdev;
  	struct class *class;
	struct i2c_client *client;
};
static struct dev_data dev;

s16 sensorlayout[3][3] = {
#if defined(CONFIG_GSEN_LAYOUT_PAT_1)
    { 1, 0, 0},	{ 0, 1,	0}, { 0, 0, 1},
#elif defined(CONFIG_GSEN_LAYOUT_PAT_2)
    { 0, 1, 0}, {-1, 0,	0}, { 0, 0, 1},
#elif defined(CONFIG_GSEN_LAYOUT_PAT_3)
    {-1, 0, 0},	{ 0,-1,	0}, { 0, 0, 1},
#elif defined(CONFIG_GSEN_LAYOUT_PAT_4)
    { 0,-1, 0},	{ 1, 0,	0}, { 0, 0, 1},
#elif defined(CONFIG_GSEN_LAYOUT_PAT_5)
    {-1, 0, 0},	{ 0, 1,	0}, { 0, 0,-1},
#elif defined(CONFIG_GSEN_LAYOUT_PAT_6)
    { 0,-1, 0}, {-1, 0,	0}, { 0, 0,-1},
#elif defined(CONFIG_GSEN_LAYOUT_PAT_7)
    { 1, 0, 0},	{ 0,-1,	0}, { 0, 0,-1},
#elif defined(CONFIG_GSEN_LAYOUT_PAT_8)
    { 0, 1, 0},	{ 1, 0,	0}, { 0, 0,-1},
#endif
};

void gsensor_read_accel_avg(int num_avg, raw_data *avg_p )
{
   	long xyz_acc[SENSOR_DATA_SIZE];   
  	s16 xyz[SENSOR_DATA_SIZE];
  	int i, j;
	
	//initialize the accumulation buffer
  	for(i = 0; i < SENSOR_DATA_SIZE; ++i) 
		xyz_acc[i] = 0;

	for(i = 0; i < num_avg; i++) 
	{      
		device_i2c_read_xyz(dev.client, (s16 *)&xyz);
		for(j = 0; j < SENSOR_DATA_SIZE; j++) 
			xyz_acc[j] += xyz[j];
  	}

	// calculate averages
  	for(i = 0; i < SENSOR_DATA_SIZE; i++) 
		avg_p->v[i] = (s16) (xyz_acc[i] / num_avg);
}
/* calc delta offset */
int gsensor_calculate_offset(int gAxis,raw_data avg)
{
	switch(gAxis)
	{
    case CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Z_NEGATIVE:  
	   	offset.u.x =  avg.u.x ;    
	   	offset.u.y =  avg.u.y ;
	   	offset.u.z =  avg.u.z + DEFAULT_SENSITIVITY;
    	break;
		case CONFIG_GSEN_CALIBRATION_GRAVITY_ON_X_POSITIVE:  
			offset.u.x =  avg.u.x + DEFAULT_SENSITIVITY;    
			offset.u.y =  avg.u.y ;
			offset.u.z =  avg.u.z ;
		 	break;
		case CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Z_POSITIVE:  
			offset.u.x =  avg.u.x ;    
			offset.u.y =  avg.u.y ;
			offset.u.z =  avg.u.z - DEFAULT_SENSITIVITY;
		 	break;
		case CONFIG_GSEN_CALIBRATION_GRAVITY_ON_X_NEGATIVE:  
			offset.u.x =  avg.u.x - DEFAULT_SENSITIVITY;    
			offset.u.y =  avg.u.y ;
			offset.u.z =  avg.u.z ;
		 	break;
		case CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Y_NEGATIVE:
			offset.u.x =  avg.u.x ;    
			offset.u.y =  avg.u.y + DEFAULT_SENSITIVITY;
			offset.u.z =  avg.u.z ;
		 	break;
		case CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Y_POSITIVE: 
			offset.u.x =  avg.u.x ;    
			offset.u.y =  avg.u.y - DEFAULT_SENSITIVITY;
			offset.u.z =  avg.u.z ;
		 	break;
		default:  
			return -ENOTTY;
	}
	return 0;
}

void gsensor_calibrate(int side){	
	raw_data avg;
	int avg_num = 16;
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
#endif
	// get acceleration average reading
	gsensor_read_accel_avg(avg_num, &avg);
	// calculate and set the offset
	gsensor_calculate_offset(side, avg); 
}

void ce_on(void){
	//omap_mux_set_gpio(OMAP_PIN_INPUT_PULLUP, CHIP_ENABLE);
}

void ce_off(void){
	//omap_mux_set_gpio(OMAP_PIN_INPUT_PULLDOWN, CHIP_ENABLE);
}

void gsensor_reset(void){
	ce_off();
	msleep(300); 
	ce_on();
}

void gsensor_set_offset(int val[3])
{
	int i;
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
#endif
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		offset.v[i] = (s16) val[i];
}

static int device_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int device_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static void device_i2c_shutdown(struct i2c_client *client)
{
	flush_delayed_work_sync(&work);
	cancel_delayed_work_sync(&work);
}

struct file_operations dmt_g_sensor_fops = {
	.owner = THIS_MODULE,
	.read = device_read,
	.write = device_write,
	.unlocked_ioctl = device_ioctl,
	.open = device_open,
	.release = device_close,
};

static const struct i2c_device_id device_i2c_ids[] = {
	{DEVICE_I2C_NAME, 0},
	{}   
};

MODULE_DEVICE_TABLE(i2c, device_i2c_ids);

static struct i2c_driver device_i2c_driver = {
	.driver	= {
		.owner = THIS_MODULE,
		.name = DEVICE_I2C_NAME,
		},
	.class = I2C_CLASS_HWMON,
	.probe = device_i2c_probe,
	.remove	= __devexit_p(device_i2c_remove),
	.suspend = device_i2c_suspend,
	.resume	= device_i2c_resume,
	.shutdown = device_i2c_shutdown,
	.id_table = device_i2c_ids,
};

static int device_open(struct inode *inode, struct file *filp){
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
#endif
	//Device_First_Time_Opened_flag
	if(Device_First_Time_Opened_flag)
	{
		Device_First_Time_Opened_flag=0;
		//gsensor_read_offset_from_file();
	}
	return 0; 
}

static ssize_t device_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static ssize_t device_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	s16 xyz[SENSOR_DATA_SIZE];
	int i;

	device_i2c_read_xyz(dev.client, (s16 *)&xyz);
	//offset compensation 
	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
			xyz[i] -= offset.v[i];
	
	if(copy_to_user(buf, &xyz, count)) 
		return -EFAULT;
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
	PRINT_X_Y_Z(xyz[0], xyz[1], xyz[2]);
#endif
	
	return count;
}

static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0, ret = 0, i;
	int intBuf[SENSOR_DATA_SIZE];
	s16 xyz[SENSOR_DATA_SIZE];
	//check type and number
	if (_IOC_TYPE(cmd) != IOCTL_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > SENSOR_MAXNR) return -ENOTTY;

	//check user space pointer is valid
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;
	
	switch(cmd) 
	{
		case SENSOR_RESET:
			//gsensor_reset();
			printk("RUN RESET");
			return ret;

		case SENSOR_CALIBRATION:
			// get orientation info
			if(copy_from_user(&intBuf, (int*)arg, sizeof(int))) return -EFAULT;
			gsensor_calibrate(intBuf[0]);
			// write in to file	
			gsensor_write_offset_to_file();
			
			// return the offset
			for(i = 0; i < SENSOR_DATA_SIZE; ++i)
				intBuf[i] = offset.v[i];
			
			ret = copy_to_user((int *)arg, &intBuf, sizeof(intBuf));
			return ret;
		
		case SENSOR_GET_OFFSET:
			// get offset from file
			gsensor_read_offset_from_file();
			
			for(i = 0; i < SENSOR_DATA_SIZE; ++i)
				intBuf[i] = offset.v[i];

			ret = copy_to_user((int *)arg, &intBuf, sizeof(intBuf));
			return ret;

		case SENSOR_SET_OFFSET:
			ret = copy_from_user(&intBuf, (int *)arg, sizeof(intBuf));
			gsensor_set_offset(intBuf);
			// write in to file
			gsensor_write_offset_to_file();
			return ret;
		
		case SENSOR_READ_ACCEL_XYZ:
			device_i2c_read_xyz(dev.client, (s16 *)&xyz);
			for(i = 0; i < SENSOR_DATA_SIZE; ++i)
				intBuf[i] = xyz[i] - offset.v[i];
			
		  	ret = copy_to_user((int*)arg, &intBuf, sizeof(intBuf));
			return ret;
		case SENSOR_SETYPR:
			if(copy_from_user(&intBuf, (int*)arg, sizeof(intBuf))) 
			{
				printk("%s:copy_from_user(&intBuf, (int*)arg, sizeof(intBuf)) ERROR, -EFAULT\n",__func__);			
				return -EFAULT;
			}
			input_report_abs(input, ABS_X, intBuf[0]);
			input_report_abs(input, ABS_Y, intBuf[1]);
			input_report_abs(input, ABS_Z, intBuf[2]);
			input_sync(input);
			//printk("%s:SENSOR_SETYPR OK! x=%d,y=%d,z=%d\n",__func__,intBuf[0],intBuf[1],intBuf[2]);

			return 1;
		case SENSOR_GET_OPEN_STATUS:
			//printk("%s:Going into DMT_GetOpenStatus()\n",__func__);
			DMT_GetOpenStatus();
			//printk("%s:DMT_GetOpenStatus() finished\n",__func__);
			return 1;
			break;
		case SENSOR_GET_CLOSE_STATUS:
			//printk("%s:Going into DMT_GetCloseStatus()\n",__func__);
			DMT_GetCloseStatus();	
			//printk("%s:DMT_GetCloseStatus() finished\n",__func__);
			return 1;
			break;		
		case SENSOR_GET_DELAY:
			
		  	ret = copy_to_user((int*)arg, &interval, sizeof(interval));
			return 1;
			break;
		default:  /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}
	
	return 0;
}
	
static int device_close(struct inode *inode, struct file *filp)
{
	printk("Close device\n");
	return 0;
}

static int device_i2c_xyz_read_reg(struct i2c_client *client,u8 *buffer, int length)
{
	struct i2c_msg msg[] = 
	{
		{.addr = client->addr, .flags = 0, .len = 1, .buf = buffer,}, 
		{.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = buffer,},
	};
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
#endif
	return i2c_transfer(client->adapter, msg, 2);
}

void device_i2c_read_xyz(struct i2c_client *client, s16 *xyz_p)
{	
	u8 buffer[6];
	s16 xyzTmp[SENSOR_DATA_SIZE];
	int i, j;

	//get xyz high/low bytes, 0x02~0x07
	buffer[0] = 2;
	device_i2c_xyz_read_reg(client, buffer, 6);

	//merge to 11-bits value
	for(i = 0; i < SENSOR_DATA_SIZE; ++i){
		xyz_p[i] = 0;
		device_i2c_merge_register_values(client, (xyzTmp + i), buffer[2*i], buffer[2*i + 1]);
	//transfer to the default layout
		for(j = 0; j < 3; j++)
			xyz_p[i] += sensorlayout[i][j] * xyzTmp[j];
	}
}

void device_i2c_merge_register_values(struct i2c_client *client, s16 *val, u8 msb, u8 lsb)
{
	*val = (((u16)msb) << 3) | (u16)lsb;
	device_i2c_correct_accel_sign(val);
}

static inline void device_i2c_correct_accel_sign(s16 *val)
{
	*val<<= (sizeof(s16) * BITS_PER_BYTE - 11);  
	*val>>= (sizeof(s16) * BITS_PER_BYTE - 11);  
}

static void DMT_work_func(struct work_struct *fakework)
{
	int i;
	static int firsttime=0;
	s16 xyz[SENSOR_DATA_SIZE];
	unsigned long t=atomic_read(&delay);
  	unsigned long dmt_delay = msecs_to_jiffies(t);
	if(!firsttime)
	{
		//gsensor_read_offset_from_file();	
	 	firsttime=1;
	}

	//dmt_delay/=1000;

#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
	printk("t=%lu ,dmt_delay=%lu\n",t,dmt_delay);
#endif

  	device_i2c_read_xyz(dev.client, (s16 *)&xyz);
  	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
     		xyz[i] -= offset.v[i];

#if DMT_DEBUG_DATA
	printk("x: %d, y: %d, z: %d\n", xyz[0], xyz[1], xyz[2]);
#endif	
	input_report_abs(input, ABS_X, xyz[gs_conf.xyz_axis[ABS_X][0]]*gs_conf.xyz_axis[ABS_X][1]);
	input_report_abs(input, ABS_Y, xyz[gs_conf.xyz_axis[ABS_Y][0]]*gs_conf.xyz_axis[ABS_Y][1]);
	input_report_abs(input, ABS_Z, xyz[gs_conf.xyz_axis[ABS_Z][0]]*gs_conf.xyz_axis[ABS_Z][1]);
	input_sync(input);
		
	if(dmt_delay<1)
		dmt_delay=1;

	schedule_delayed_work(&work, dmt_delay);
}


static int __devinit device_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	u8 buffer[4];
	int i;

	for(i = 0; i < SENSOR_DATA_SIZE; ++i)
		offset.v[i] = 0;

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
  		printk("%s, functionality check failed\n", __func__);
    		return -1;
  	}
	
	gsensor_reset();

	buffer[0] = CONTROL_REGISTERS;
	device_i2c_xyz_read_reg(client, buffer, 4);
	if( buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x88 && buffer[3] == 0x08)
	{
		printk(KERN_INFO "i2c Read 0x08 = %d!\n",buffer[0]);
		printk(KERN_INFO "i2c Read 0x09 = %d!\n",buffer[1]);
		printk(KERN_INFO "i2c Read 0x0a = %d!\n",buffer[2]);
		printk(KERN_INFO "i2c Read 0x0b = %d!\n",buffer[3]);
		printk(KERN_INFO "@@@ %s DMT_DEVICE_NAME registered I2C driver!\n",__FUNCTION__);
		dev.client = client;
	}
	else
	{
		printk(KERN_INFO "err : i2c Read 0x08 = %d!\n",buffer[0]);
		printk(KERN_INFO "err : i2c Read 0x09 = %d!\n",buffer[1]);
		printk(KERN_INFO "err : i2c Read 0x0a = %d!\n",buffer[2]);
		printk(KERN_INFO "err : i2c Read 0x0b = %d!\n",buffer[3]);
		dev.client = NULL;
		return -1;
	}
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
	//check sensorlayout[i][j]
	for(i = 0; i < 3; ++i)
	{
		for(j = 0; j < 3; j++)
			printk("%d",sensorlayout[i][j]);
		printk("\n");
	}
#endif
	return 0;
}

static int __devexit device_i2c_remove(struct i2c_client *client)
{
#if DMT_DEBUG_DATA
	IN_FUNC_MSG;
#endif
	return 0;
}

int dmt08_enable(int en)
{
	printk(KERN_DEBUG "%s: enable = %d\n", __func__, en);
	DMT_sysfs_update_active_status(en);
	return 0;
}

int dmt08_setDelay(int mdelay)
{
	printk(KERN_DEBUG "%s: delay = %d\n", __func__, mdelay);
	atomic_set(&delay, mdelay);
	return 0;
}

int dmt08_getLSG(int *lsg)
{
    *lsg = 256;
	return 0;
}

struct gsensor_data dmt08_gs_data = {
	.i2c_addr = DMT08_I2C_ADDR,
	.enable = dmt08_enable,
	.setDelay = dmt08_setDelay,
	.getLSG = dmt08_getLSG,
};

static int __init device_init(void)
{
	int err=-1;	
	struct device *device;
	int ret = 0;
	IN_FUNC_MSG;

	if (get_gsensor_conf(&gs_conf))
		return -1;

	if (gs_conf.op != 1)
		return -1;

	printk("G-Sensor dmt08 init\n");

	if (gsensor_register(&dmt08_gs_data))
		return -1;

	if (gsensor_i2c_register_device() < 0)
		return -1;

	atomic_set(&delay, 200);

	ret = alloc_chrdev_region(&dev.devno, 0, 1, GSENSOR_DEVICE_NAME);
  	if(ret)
	{
    		printk("%s, can't allocate chrdev\n", __func__);
		return ret;
	}
	printk("%s, register chrdev(%d, %d)\n", __func__, MAJOR(dev.devno), MINOR(dev.devno));
	
	cdev_init(&dev.cdev, &dmt_g_sensor_fops);  
	dev.cdev.owner = THIS_MODULE;
  	ret = cdev_add(&dev.cdev, dev.devno, 1);
  	if(ret < 0)
	{
    		printk("%s, add character device error, ret %d\n", __func__, ret);
		return ret;
	}
	dev.class = class_create(THIS_MODULE, ACCELEMETER_CLASS_NAME);
	if(IS_ERR(dev.class))
	{
   		printk("%s, create class, error\n", __func__);
		return ret;
  	}
	device=device_create(dev.class, NULL, dev.devno, NULL, GSENSOR_DEVICE_NAME);

	mutex_init(&DMT_mutex);

	INIT_DELAYED_WORK(&work, DMT_work_func);
	printk("DMT: INIT_DELAYED_WORK\n");

	err=input_init();
	if(err)
	{
		printk("%s:input_init fail, error code= %d\n", __func__, err);
	}

	err = create_device_attributes(device,DMT_attributes);

	return i2c_add_driver(&device_i2c_driver);
}


static void __exit device_exit(void)
{
	IN_FUNC_MSG;
	input_unregister_device(input);
	input_free_device(input);
	cdev_del(&dev.cdev);
	unregister_chrdev_region(dev.devno, 1);
	device_destroy(dev.class, dev.devno);
	class_destroy(dev.class);
	i2c_del_driver(&device_i2c_driver);
}


void gsensor_write_offset_to_file(void)
{
	char data[18];
	unsigned int orgfs;
	long lfile=-1;
	spinlock_t lock; //add by yang
	spin_lock_init(&lock); //add by yang
	sprintf(data,"%5d %5d %5d",offset.u.x,offset.u.y,offset.u.z);

	orgfs = get_fs();
// Set segment descriptor associated to kernel space
	set_fs(KERNEL_DS);
	
    	lfile=sys_open(OffsetFileName,O_WRONLY|O_CREAT, 0777);
	if (lfile < 0)
	{
		printk("sys_open %s error!!. %ld\n",OffsetFileName,lfile);
	}
	else
	{
		spin_lock(&lock); //add by yang
		sys_write(lfile, data,18);
		sys_close(lfile);
		spin_unlock(&lock); //add by yang
	}
	set_fs(orgfs);
}

void gsensor_read_offset_from_file(void)
{
	unsigned int orgfs;
	char data[18];
	long lfile=-1;
	orgfs = get_fs();
// Set segment descriptor associated to kernel space
	set_fs(KERNEL_DS);

	lfile = sys_open(OffsetFileName, O_RDONLY, 0);
	if (lfile < 0)
	{
		printk("sys_open %s error!!. %ld\n",OffsetFileName,lfile);
		strcpy(data,"00000 00000 00000");

	}
	else
	{
		sys_read(lfile, data, 18);
		sys_close(lfile);
	}
	sscanf(data,"%hd %hd %hd",&offset.u.x,&offset.u.y,&offset.u.z);
	printk("%5d %5d %5d",offset.u.x,offset.u.y,offset.u.z);
	set_fs(orgfs);
}

MODULE_AUTHOR("DMT_RD");
MODULE_DESCRIPTION("DMT Gsensor Driver");
MODULE_LICENSE("GPL");

module_init(device_init);
module_exit(device_exit);
