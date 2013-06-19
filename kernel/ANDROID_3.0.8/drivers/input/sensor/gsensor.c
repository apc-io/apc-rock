/*++
Copyright (c) 2012 WonderMedia Technologies, Inc. All Rights Reserved.
This PROPRIETARY SOFTWARE is the property of WonderMedia Technologies, Inc. 
and may contain trade secrets and/or other confidential information of 
WonderMedia Technologies, Inc. This file shall not be disclosed to any 
third party, in whole or in part, without prior written consent of 
WonderMedia.  

THIS PROPRIETARY SOFTWARE AND ANY RELATED DOCUMENTATION ARE PROVIDED 
AS IS, WITH ALL FAULTS, AND WITHOUT WARRANTY OF ANY KIND EITHER EXPRESS 
OR IMPLIED, AND WonderMedia TECHNOLOGIES, INC. DISCLAIMS ALL EXPRESS 
OR IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
QUIET ENJOYMENT OR NON-INFRINGEMENT.
--*/

#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include "gsensor.h"

#define GSENSOR_I2C_NAME	"g-sensor"

#ifdef CONFIG_WMT_SENSOR_DMT08
#define GSENSOR_I2C_ADDR	0x1c
#elif defined CONFIG_WMT_SENSOR_KXTI9
#define GSENSOR_I2C_ADDR	0x0f
#endif

struct gsensor_data *gs_data;

extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);

struct i2c_board_info gsensor_i2c_board_info = {
	.type          = GSENSOR_I2C_NAME,
	.flags         = 0x00,
	.platform_data = NULL,
	.archdata      = NULL,
	.irq           = -1,
};

int gsensor_i2c_register_device (void)
{
	struct i2c_board_info *gsensor_i2c_bi;
	struct i2c_adapter *adapter = NULL;
	struct i2c_client *client   = NULL;
	gsensor_i2c_bi = &gsensor_i2c_board_info;
	adapter = i2c_get_adapter(0);/*in bus 0*/

	if (NULL == adapter) {
		printk("can not get i2c adapter, client address error\n");
		return -1;
	}
	gsensor_i2c_bi->addr = gs_data->i2c_addr;
	client = i2c_new_device(adapter, gsensor_i2c_bi);
	if (client == NULL) {
		printk("allocate i2c client failed\n");
		return -1;
	}
	i2c_put_adapter(adapter);
	return 0;
}

/*
 * Get the configure of sensor from u-boot.
 * Return: 0--success, other--error.
 */
int get_gsensor_conf(struct gsensor_conf *gs_conf)
{
	char varbuf[64];
	int n;
	int varlen;

	memset(varbuf, 0, sizeof(varbuf));
	varlen = sizeof(varbuf);
	if (wmt_getsyspara("wmt.io.gsensor", varbuf, &varlen)) {
		printk("wmt.io.gsensor not defined!\n");
		return -1;
	} else {
		n = sscanf(varbuf, "%d:%d:%d:%d:%d:%d:%d:%d",
				&gs_conf->op,
				&gs_conf->samp,
				&(gs_conf->xyz_axis[0][0]),
				&(gs_conf->xyz_axis[0][1]),
				&(gs_conf->xyz_axis[1][0]),
				&(gs_conf->xyz_axis[1][1]),
				&(gs_conf->xyz_axis[2][0]),
				&(gs_conf->xyz_axis[2][1]));
				printk(KERN_INFO "wmt.io.gsensor = %d:%d:%d:%d:%d:%d:%d:%d\n",
				gs_conf->op,
				gs_conf->samp,
				gs_conf->xyz_axis[0][0],
				gs_conf->xyz_axis[0][1],
				gs_conf->xyz_axis[1][0],
				gs_conf->xyz_axis[1][1],
				gs_conf->xyz_axis[2][0],
				gs_conf->xyz_axis[2][1]);
		if (n != 8) {
			printk("wmt.io.gsensor format is incorrect!\n");
			return -1;
		}

		if (gs_conf->op <= 0) {
			printk(KERN_INFO "wmt.io.gsensor is disabled\n");
			return -1;
		}
	}
	return 0;
}

static long gsensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int flag;

	if (!gs_data)
		return -1;

	switch (cmd) {
	case WMT_IOCTL_APP_SET_AFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		else {
			if (flag < 0 || flag > 1)
				return -EINVAL;
			gs_data->enable(flag);
		}
		break;
	case WMT_IOCTL_APP_GET_AFLAG:
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	case WMT_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		else
			gs_data->setDelay(flag);

		break;
	case WMT_IOCTL_APP_GET_DELAY:
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	case WMT_IOCTL_APP_GET_LSG:
	    gs_data->getLSG(&flag);
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

struct file_operations gsensor_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gsensor_ioctl,
};

struct miscdevice gsensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "g-sensor",
	.fops = &gsensor_fops,
};

int gsensor_register(struct gsensor_data *data)
{
	int err=-1;

	gs_data = data;
	err = misc_register(&gsensor_device);
	if (err)
		printk(KERN_ERR "%s: gsensor misc register failed\n", __func__);
	return err;
}

EXPORT_SYMBOL_GPL(gsensor_i2c_register_device);
EXPORT_SYMBOL_GPL(get_gsensor_conf);
EXPORT_SYMBOL_GPL(gsensor_register);
