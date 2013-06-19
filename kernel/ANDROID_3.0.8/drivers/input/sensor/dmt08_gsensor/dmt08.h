/*
 * @file include/linux/dmt.h
 * @brief DMARD05 & DMARD06 & DMARD07 g-sensor Linux device driver
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
#ifndef DMT_H
#define DMT_H

#if (defined(CONFIG_SENSORS_DMARD05) || defined(CONFIG_SENSORS_DMARD05_MODULE))
#define DEVICE_I2C_NAME "dmard05"
#define DEFAULT_SENSITIVITY 64
#define WHO_AM_I_VALUE 	0x05
#define X_OUT 		0x41
#define SW_RESET 	0x53
#define WHO_AM_I 	0x0f
#elif (defined(CONFIG_SENSORS_DMARD06) || defined(CONFIG_SENSORS_DMARD06_MODULE))
#define DEVICE_I2C_NAME "dmard06"
#define DEFAULT_SENSITIVITY 32
#define WHO_AM_I_VALUE 	0x06
#define X_OUT 		0x41
#define SW_RESET 	0x53
#define WHO_AM_I 	0x0f
#elif (defined(CONFIG_SENSORS_DMARD07) || defined(CONFIG_SENSORS_DMARD07_MODULE))
#define DEVICE_I2C_NAME "dmard07"
#define DEFAULT_SENSITIVITY 64
#define WHO_AM_I_VALUE 	0x07
#define X_OUT 		0x41
#define SW_RESET 	0x53
#define WHO_AM_I 	0x0f
#elif (defined(CONFIG_SENSORS_DMARD03) || defined(CONFIG_SENSORS_DMARD03_MODULE))
#define DEVICE_I2C_NAME "dmard03"
#define DEFAULT_SENSITIVITY 256
#define CONTROL_REGISTERS  0x08
#elif (defined(CONFIG_SENSORS_DMARD08) || defined(CONFIG_SENSORS_DMARD08_MODULE) || defined(CONFIG_WMT_SENSOR_DMT08))
#define DEVICE_I2C_NAME "g-sensor"
#define DEFAULT_SENSITIVITY 256
#define CONTROL_REGISTERS  0x08
#define DMT08_I2C_ADDR	0x1c
#endif

//#define DMT_DEBUG_DATA	1
#define DMT_DEBUG_DATA 		0

#if DMT_DEBUG_DATA
#define IN_FUNC_MSG printk(KERN_INFO "@DMT@ In %s\n", __func__)
#define PRINT_X_Y_Z(x, y, z) printk(KERN_INFO "@DMT@ X/Y/Z axis: %04d , %04d , %04d\n", (x), (y), (z))
#define PRINT_OFFSET(x, y, z) printk(KERN_INFO "@offset@  X/Y/Z axis: %04d , %04d , %04d\n",offset.x,offset.y,offset.z);
#else
#define IN_FUNC_MSG
#define PRINT_X_Y_Z(x, y, z)
#define PRINT_OFFSET(x, y, z)
#endif

//g-senor layout configuration, choose one of the following configuration
#define CONFIG_GSEN_LAYOUT_PAT_1
//#define CONFIG_GSEN_LAYOUT_PAT_2
//#define CONFIG_GSEN_LAYOUT_PAT_3
//#define CONFIG_GSEN_LAYOUT_PAT_4
//#define CONFIG_GSEN_LAYOUT_PAT_5
//#define CONFIG_GSEN_LAYOUT_PAT_6
//#define CONFIG_GSEN_LAYOUT_PAT_7
//#define CONFIG_GSEN_LAYOUT_PAT_8

#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Z_NEGATIVE 1
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Z_POSITIVE 2
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Y_NEGATIVE 3
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Y_POSITIVE 4
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_X_NEGATIVE 5
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_X_POSITIVE 6

#define AVG_NUM 16

#define IOCTL_MAGIC  0x09
#define SENSOR_DATA_SIZE 3                           

#define SENSOR_RESET    		_IO(IOCTL_MAGIC, 0)
#define SENSOR_CALIBRATION   		_IOWR(IOCTL_MAGIC,  1, int[SENSOR_DATA_SIZE])
#define SENSOR_GET_OFFSET  		_IOR(IOCTL_MAGIC,  2, int[SENSOR_DATA_SIZE])
#define SENSOR_SET_OFFSET  		_IOWR(IOCTL_MAGIC,  3, int[SENSOR_DATA_SIZE])
#define SENSOR_READ_ACCEL_XYZ  		_IOR(IOCTL_MAGIC,  4, int[SENSOR_DATA_SIZE])
#define SENSOR_SETYPR  			_IOW(IOCTL_MAGIC,  5, int[SENSOR_DATA_SIZE])
#define SENSOR_GET_OPEN_STATUS		_IO(IOCTL_MAGIC,  6)
#define SENSOR_GET_CLOSE_STATUS		_IO(IOCTL_MAGIC,  7)
#define SENSOR_GET_DELAY		_IOR(IOCTL_MAGIC,  8, unsigned int*)

#define SENSOR_MAXNR 8

#endif               
