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

#ifndef __GSENSOR_H__
#define __GSENSOR_H__

#define IOCTL_WMT  0x01
#define WMT_IOCTL_APP_SET_AFLAG		_IOW(IOCTL_WMT, 0x01, int)
#define WMT_IOCTL_APP_SET_DELAY		_IOW(IOCTL_WMT, 0x02, int)
#define WMT_IOCTL_APP_GET_AFLAG		_IOW(IOCTL_WMT, 0x03, int)
#define WMT_IOCTL_APP_GET_DELAY		_IOW(IOCTL_WMT, 0x04, int)
#define WMT_IOCTL_APP_GET_LSG		_IOW(IOCTL_WMT, 0x05, int)

struct gsensor_conf {
	int op;
	int samp;
	int xyz_axis[3][2];
};

struct gsensor_data {
	int i2c_addr;
	int (*enable) (int en);
	int (*setDelay) (int mdelay);
	int (*getLSG) (int *lsg);
};

extern int gsensor_i2c_register_device (void);
extern int get_gsensor_conf(struct gsensor_conf *gs_conf);
extern int gsensor_register(struct gsensor_data *gs_data);

#endif