/*++ 
 * linux/drivers/media/video/wmt_v4l2/cmos/cmos-dev-gc.c
 * WonderMedia v4l cmos device driver
*
 * Copyright c 2010  WonderMedia  Technologies, Inc.
 *
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 2 of the License, or 
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * WonderMedia Technologies, Inc.
 * 4F, 533, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C
--*/
#define CMOS_DEV_GC_C
#include "cmos-dev-gc.h"
#include "../wmt-vid.h"
#include <linux/module.h>

unsigned char gc0308[]={
0xfe, 0x80, 0xfe, 0x00, 0xd2, 0x10, 0x22, 0x55, 0x03, 0x01, 0x04, 0x2c, 0x5a, 0x56, 0x5b, 0x40,
 0x5c, 0x4a, 0x22, 0x57, 0x01, 0xfa, 0x02, 0x70, 0x0f, 0x01, 0xe2, 0x00, 0xe3, 0x64, 0xe4, 0x02,
 0xe5, 0x58, 0xe6, 0x03, 0xe7, 0x20, 0xe8, 0x04, 0xe9, 0xb0, 0xea, 0x09, 0xeb, 0xc4, 0x05, 0x00,
 0x06, 0x00, 0x07, 0x00, 0x08, 0x00, 0x09, 0x01, 0x0a, 0xe8, 0x0b, 0x02, 0x0c, 0x88, 0x0d, 0x02,
 0x0e, 0x02, 0x10, 0x22, 0x11, 0xfd, 0x12, 0x2a, 0x13, 0x00, 0x15, 0x0a, 0x16, 0x05, 0x17, 0x01,
 0x18, 0x44, 0x19, 0x44, 0x1a, 0x1e, 0x1b, 0x00, 0x1c, 0xc1, 0x1d, 0x08, 0x1e, 0x60, 0x1f, 0x02,/*1).0x01,2)0x16,*/
 0x20, 0xff, 0x21, 0xf8, 0x22, 0x57, 0x24, 0xa2, 0x25, 0x0f, 0x26, 0x02, 0x2f, 0x01, 0x30, 0xf7,
 0x31, 0x50, 0x32, 0x00, 0x39, 0x04, 0x3a, 0x18, 0x3b, 0x20, 0x3c, 0x00, 0x3d, 0x00, 0x3e, 0x00,
 0x3f, 0x00, 0x50, 0x10, 0x53, 0x82, 0x54, 0x80, 0x55, 0x80, 0x56, 0x82, 0x8b, 0x40, 0x8c, 0x40,
 0x8d, 0x40, 0x8e, 0x2e, 0x8f, 0x2e, 0x90, 0x2e, 0x91, 0x3c, 0x92, 0x50, 0x5d, 0x12, 0x5e, 0x1a,
 0x5f, 0x24, 0x60, 0x07, 0x61, 0x15, 0x62, 0x08, 0x64, 0x03, 0x66, 0xe8, 0x67, 0x86, 0x68, 0xa2,
 0x69, 0x18, 0x6a, 0x0f, 0x6b, 0x00, 0x6c, 0x5f, 0x6d, 0x8f, 0x6e, 0x55, 0x6f, 0x38, 0x70, 0x15,
 0x71, 0x33, 0x72, 0xdc, 0x73, 0x80, 0x74, 0x02, 0x75, 0x3f, 0x76, 0x02, 0x77, 0x36, 0x78, 0x88,
 0x79, 0x81, 0x7a, 0x81, 0x7b, 0x22, 0x7c, 0xff, 0x93, 0x48, 0x94, 0x00, 0x95, 0x05, 0x96, 0xe8,
 0x97, 0x40, 0x98, 0xf0, 0xb1, 0x38, 0xb2, 0x38, 0xbd, 0x38, 0xbe, 0x36, 0xd0, 0xc9, 0xd1, 0x10,
 0xd3, 0x80, 0xd5, 0xf2, 0xd6, 0x16, 0xdb, 0x92, 0xdc, 0xa5, 0xdf, 0x23, 0xd9, 0x00, 0xda, 0x00,
 0xe0, 0x09, 0xed, 0x04, 0xee, 0xa0, 0xef, 0x40, 0x80, 0x03, 0x80, 0x03, 0x9f, 0x10, 0xa0, 0x20,
 0xa1, 0x38, 0xa2, 0x4e, 0xa3, 0x63, 0xa4, 0x76, 0xa5, 0x87, 0xa6, 0xa2, 0xa7, 0xb8, 0xa8, 0xca,
 0xa9, 0xd8, 0xaa, 0xe3, 0xab, 0xeb, 0xac, 0xf0, 0xad, 0xf8, 0xae, 0xfd, 0xaf, 0xff, 0xc0, 0x00,
 0xc1, 0x10, 0xc2, 0x1c, 0xc3, 0x30, 0xc4, 0x43, 0xc5, 0x54, 0xc6, 0x65, 0xc7, 0x75, 0xc8, 0x93,
 0xc9, 0xb0, 0xca, 0xcb, 0xcb, 0xe6, 0xcc, 0xff, 0xf0, 0x02, 0xf1, 0x01, 0xf2, 0x01, 0xf3, 0x30,
 0xf9, 0x9f, 0xfa, 0x78, 0xfe, 0x01, 0x00, 0xf5, 0x02, 0x1a, 0x0a, 0xa0, 0x0b, 0x60, 0x0c, 0x08,
 0x0e, 0x4c, 0x0f, 0x39, 0x11, 0x3f, 0x12, 0x72, 0x13, 0x13, 0x14, 0x42, 0x15, 0x43, 0x16, 0xc2,
 0x17, 0xa8, 0x18, 0x18, 0x19, 0x40, 0x1a, 0xd0, 0x1b, 0xf5, 0x70, 0x40, 0x71, 0x58, 0x72, 0x30,
 0x73, 0x48, 0x74, 0x20, 0x75, 0x60, 0x77, 0x20, 0x78, 0x32, 0x30, 0x03, 0x31, 0x40, 0x32, 0xe0,
 0x33, 0xe0, 0x34, 0xe0, 0x35, 0xb0, 0x36, 0xc0, 0x37, 0xc0, 0x38, 0x04, 0x39, 0x09, 0x3a, 0x12,
 0x3b, 0x1c, 0x3c, 0x28, 0x3d, 0x31, 0x3e, 0x44, 0x3f, 0x57, 0x40, 0x6c, 0x41, 0x81, 0x42, 0x94,
 0x43, 0xa7, 0x44, 0xb8, 0x45, 0xd6, 0x46, 0xee, 0x47, 0x0d, 0xfe, 0x00, 0xfe, 0x00, 0x10, 0x26,
 0x11, 0x0d, 0x1a, 0x2a, 0x1c, 0x49, 0x1d, 0x9a, 0x1e, 0x61, 0x3a, 0x20, 0x50, 0x14, 0x53, 0x80,
 0x56, 0x80, 0x8b, 0x20, 0x8c, 0x20, 0x8d, 0x20, 0x8e, 0x14, 0x8f, 0x10, 0x90, 0x14, 0x94, 0x02,
 0x95, 0x07, 0x96, 0xe0, 0xb1, 0x40, 0xb2, 0x40, 0xb3, 0x40, 0xb6, 0xe0, 0xd0, 0xcb, 0xd3, 0x48,
 0xf2, 0x02, 0xf7, 0x12, 0xf8, 0x0a, 0xfe, 0x01, 0x02, 0x20, 0x04, 0x10, 0x05, 0x08, 0x06, 0x20,
 0x08, 0x0a, 0x0e, 0x44, 0x0f, 0x32, 0x10, 0x41, 0x11, 0x37, 0x12, 0x22, 0x13, 0x19, 0x14, 0x44,
 0x15, 0x44, 0x19, 0x50, 0x1a, 0xd8, 0x32, 0x10, 0x35, 0x00, 0x36, 0x80, 0x37, 0x00, 0xfe, 0x00,
 0xd2, 0x90, 0x9f, 0x10, 0xa0, 0x20, 0xa1, 0x38, 0xa2, 0x4e, 0xa3, 0x63, 0xa4, 0x76, 0xa5, 0x87,
 0xa6, 0xa2, 0xa7, 0xb8, 0xa8, 0xca, 0xa9, 0xd8, 0xaa, 0xe3, 0xab, 0xeb, 0xac, 0xf0, 0xad, 0xf8,
 0xae, 0xfd, 0xaf, 0xff, 0x14, 0x13, 0xff, 0xff,
};
unsigned char gc0308_320_240[]={
0xfe,  0x01, 0x54, 0x22, 0x55, 0x03, 0x56, 0x00, 0x57, 0x00, 0x58, 0x00, 0x59, 0x00, 0xfe, 0x00
};

unsigned int gc0307[]={
0x43, 0x00, 0x44, 0xa2, 0x40, 0x10, 0x41, 0x00, 0x42, 0x10, 0x47, 0x00, 0x48, 0xc3, 0x49, 0x00,
 0x4a, 0x00, 0x4b, 0x00, 0x4e, 0x22, 0x4f, 0x01, 0x1c, 0x00, 0x1d, 0x00, 0x11, 0x05, 0x01, 0xfa,
 0x02, 0x70, 0x1c, 0x00, 0x1d, 0x00, 0x10, 0x01, 0x11, 0x05, 0x05, 0x00, 0x06, 0x00, 0x07, 0x00,
 0x08, 0x00, 0x09, 0x01, 0x0a, 0xe8, 0x0b, 0x02, 0x0c, 0x80, 0x0d, 0x22, 0x0e, 0x02, 0x12, 0x70,
 0x13, 0x00, 0x14, 0x00, 0x15, 0xba, 0x16, 0x13, 0x17, 0x52, 0x1e, 0x41, 0x1f, 0x32, 0x47, 0x00,
 0x19, 0x06, 0x1a, 0x06, 0x31, 0x00, 0x3b, 0x00, 0x59, 0x0f, 0x58, 0xc6, 0x57, 0x08, 0x56, 0x77,
 0x35, 0xd8, 0x36, 0x40, 0x3c, 0x00, 0x3d, 0x00, 0x3e, 0x00, 0x3f, 0x00, 0xb5, 0x70, 0xb6, 0x40,
 0xb7, 0x00, 0xb8, 0x38, 0xb9, 0xc3, 0xba, 0x0f, 0x7e, 0x35, 0x7f, 0x86, 0x5c, 0x68, 0x5d, 0x78,
 0x61, 0x80, 0x63, 0x80, 0x65, 0x98, 0x67, 0x80, 0x68, 0x18, 0x69, 0x58, 0x6a, 0xf6, 0x6b, 0xfb,
 0x6c, 0xf4, 0x6d, 0x5a, 0x6e, 0xe6, 0x6f, 0x00, 0x70, 0x14, 0x71, 0x1c, 0x72, 0x20, 0x73, 0x10,
 0x74, 0x3c, 0x75, 0x52, 0x7d, 0x2f, 0x80, 0x0c, 0x81, 0x0c, 0x82, 0x44, 0x83, 0x18, 0x84, 0x18,
 0x85, 0x04, 0x87, 0x34, 0x88, 0x04, 0x89, 0x01, 0x8a, 0x50, 0x8b, 0x50, 0x8c, 0x07, 0x50, 0x0c,
 0x5f, 0x3c, 0x8e, 0x02, 0x86, 0x02, 0x51, 0x20, 0x52, 0x08, 0x53, 0x00, 0x77, 0x80, 0x78, 0x00,
 0x79, 0x00, 0x7a, 0x00, 0x7b, 0x40, 0x7c, 0x00, 0xa0, 0x40, 0xa1, 0x40, 0xa2, 0x34, 0xa3, 0x34,
 0xa4, 0xc8, 0xa5, 0x02, 0xa6, 0x28, 0xa7, 0x02, 0xa8, 0xee, 0xa9, 0x12, 0xaa, 0x01, 0xab, 0x20,
 0xac, 0xf0, 0xad, 0x10, 0xae, 0x18, 0xaf, 0x74, 0xb0, 0xe0, 0xb1, 0x20, 0xb2, 0x6c, 0xb3, 0x40,
 0xb4, 0x04, 0xbb, 0x42, 0xbc, 0x60, 0xbd, 0x50, 0xbe, 0x50, 0xbf, 0x0c, 0xc0, 0x06, 0xc1, 0x60,
 0xc2, 0xf1, 0xc3, 0x40, 0xc4, 0x1c, 0xc5, 0x56, 0xc6, 0x1d, 0xca, 0x70, 0xcb, 0x70, 0xcc, 0x78,
 0xcd, 0x80, 0xce, 0x80, 0xcf, 0x80, 0x20, 0x06, 0x21, 0xc0, 0x22, 0x60, 0x23, 0x88, 0x24, 0x96,
 0x25, 0x30, 0x26, 0xd0, 0x27, 0x00, 0x28, 0x01, 0x29, 0x90, 0x2a, 0x02, 0x2b, 0x58, 0x2c, 0x02,
 0x2d, 0x58, 0x2e, 0x05, 0x2f, 0x78, 0x30, 0x20, 0x31, 0x00, 0x32, 0x1c, 0x33, 0x90, 0x34, 0x10,
 0xd0, 0x34, 0xd1, 0x50, 0xd2, 0x61, 0xd4, 0x64, 0xd5, 0x01, 0xd6, 0x64, 0xd7, 0x03, 0xd8, 0x02,
 0xdd, 0x22, 0xe0, 0x03, 0xe1, 0x02, 0xe2, 0x27, 0xe3, 0x1e, 0xe8, 0x3b, 0xe9, 0x6e, 0xea, 0x2c,
 0xeb, 0x50, 0xec, 0x73, 0xed, 0x00, 0xee, 0x00, 0xef, 0x00, 0xf0, 0x01, 0x00, 0x20, 0x01, 0x20,
 0x02, 0x20, 0x03, 0x20, 0x04, 0x78, 0x05, 0x78, 0x06, 0x78, 0x07, 0x78, 0x10, 0x04, 0x11, 0x04,
 0x12, 0x04, 0x13, 0x04, 0x14, 0x01, 0x15, 0x01, 0x16, 0x01, 0x17, 0x01, 0x20, 0x00, 0x21, 0x00,
 0x22, 0x00, 0x23, 0x00, 0x24, 0x00, 0x25, 0x00, 0x26, 0x00, 0x27, 0x00, 0x40, 0x11, 0x45, 0x06,
 0x46, 0x06, 0x47, 0x05, 0x48, 0x04, 0x49, 0x03, 0x4a, 0x03, 0x62, 0xd8, 0x63, 0x24, 0x64, 0x24,
 0x65, 0x24, 0x66, 0xd8, 0x67, 0x24, 0x5a, 0x00, 0x5b, 0x00, 0x5c, 0x00, 0x5d, 0x00, 0x5e, 0x00,
 0x5f, 0x00, 0x69, 0x03, 0x70, 0x5d, 0x71, 0xed, 0x72, 0xff, 0x73, 0xe5, 0x74, 0x5f, 0x75, 0xe6,
 0x76, 0x41, 0x77, 0xef, 0x78, 0xff, 0x79, 0xff, 0x7a, 0x5f, 0x7b, 0xfa, 0x7e, 0x00, 0x7f, 0x00,
 0x80, 0xc8, 0x81, 0x06, 0x82, 0x08, 0x83, 0x13, 0x84, 0x23, 0x85, 0x35, 0x86, 0x44, 0x87, 0x53,
 0x88, 0x60, 0x89, 0x6d, 0x8a, 0x84, 0x8b, 0x98, 0x8c, 0xaa, 0x8d, 0xb8, 0x8e, 0xc6, 0x8f, 0xd1,
 0x90, 0xdb, 0x91, 0xea, 0x92, 0xf5, 0x93, 0xfb, 0x94, 0x04, 0x95, 0x0e, 0x96, 0x1b, 0x97, 0x28,
 0x98, 0x35, 0x99, 0x41, 0x9a, 0x4e, 0x9b, 0x67, 0x9c, 0x7e, 0x9d, 0x94, 0x9e, 0xa7, 0x9f, 0xba,
 0xa0, 0xc8, 0xa1, 0xd4, 0xa2, 0xe7, 0xa3, 0xf4, 0xa4, 0xfa, 0xf0, 0x00, 0x40, 0x7e, 0x41, 0x2f,
 0x0f, 0x22/*0xb2*/, 0x45, 0x27, 0x47, 0x2c, 0x43, 0x40, 0x44, 0xe2, 0xff, 0xff, 0x01, 0xfa, 0x02, 0x70,
 0x10, 0x01, 0xd6, 0x64, 0x28, 0x02, 0x29, 0x58, 0x2a, 0x03, 0x2b, 0x20, 0x2c, 0x03, 0x2d, 0xe8,
 0x2e, 0x09, 0x2f, 0xc4, 0x7e, 0x65, 0x7f, 0x56,
};


int cmos_gc0307_identify(void)
{
	char data=0x0;
	data = wmt_vid_i2c_read(CMOS_GC0307_I2C_ADDR,0) ;
	if(data==0x99)
	{
		printk("find cmos device gc0307\n");
		return 0;
	}
	
	return -1;
}


int cmos_init_gc0307(cmos_init_arg_t  *init_arg)
{
	unsigned int * array_addr;
	unsigned int addr,data,array_size,i;
	array_size=sizeof(gc0307)/sizeof(unsigned int);
	array_addr=gc0307;

	for(i=0;i<array_size;i+=2)
	{
        addr = array_addr[i];
        data = array_addr[i+1];
		wmt_vid_i2c_write(CMOS_GC0307_I2C_ADDR,addr,data);
	}
  
	if ((init_arg->cmos_v_flip == 0) &&(init_arg->cmos_h_flip == 0))
	{
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x0f, 0x32);
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x45, 0x27);
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x47, 0x2c);
	}else if ((init_arg->cmos_v_flip == 0) &&(init_arg->cmos_h_flip == 1)){
	
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x0f, 0x22);
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x45, 0x26);
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x47, 0x28);
	}else if ((init_arg->cmos_v_flip == 1) &&(init_arg->cmos_h_flip == 0)){
	
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x0f, 0x12);
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x45, 0x25);
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x47, 0x24);
	}else if ((init_arg->cmos_v_flip == 1) &&(init_arg->cmos_h_flip == 1)){
	
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x0f, 0x02);
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x45, 0x24);
		wmt_vid_i2c_write( CMOS_GC0307_I2C_ADDR, 0x47, 0x20);
	}
      return 0;

}
int cmos_exit_gc0307(void)
{
 	return 0;
}
int cmos_gc0308_identify(void)
{
	char data=0x0;
	data = wmt_vid_i2c_read(CMOS_GC0308_I2C_ADDR,0) ;
	if(data ==0x9b)
	{
		printk("find cmos device gc0308\n");
		return 0;
	}

	return -1;
}

int cmos_init_gc0308(cmos_init_arg_t  *init_arg)
{
	unsigned char *array_addr,addr,data;
	unsigned int array_size,i;
	array_size=sizeof(gc0308);
	array_addr=gc0308;
       printk("cmos_init_gc0308 %d %d \n", init_arg->width, init_arg->height);
	for(i=0;i<array_size;i+=2)
	{
        addr = array_addr[i];
        data = array_addr[i+1];
		wmt_vid_i2c_write(CMOS_GC0308_I2C_ADDR,addr,data);
	}

	if((init_arg->width == 320) && ( init_arg->height == 240))
	{
		printk("init gc0308_320_240 \n");
		array_size=sizeof(gc0308_320_240);
		array_addr=gc0308_320_240;

		for(i=0;i<array_size;i+=2)
		{
	          addr = array_addr[i];
	          data = array_addr[i+1];
		    wmt_vid_i2c_write(CMOS_GC0308_I2C_ADDR,addr,data);
		}
	}

	//set blanking  , fps  25 
      	wmt_vid_i2c_write(CMOS_GC0308_I2C_ADDR, 0x1, 0x6a);
      	wmt_vid_i2c_write(CMOS_GC0308_I2C_ADDR, 0x2, 0x70);
      	wmt_vid_i2c_write(CMOS_GC0308_I2C_ADDR, 0xf, 0x0);
  
	data = wmt_vid_i2c_read(CMOS_GC0308_I2C_ADDR,0x14) ;
	data &= 0xfc;
	
	data |= 0x10;
	if (init_arg->cmos_v_flip)
		data |= 0x2;
	
	if (init_arg->cmos_h_flip)
		data |= 0x1;
	
      wmt_vid_i2c_write( CMOS_GC0308_I2C_ADDR, 0x14, data);
      return 0;
}

int cmos_exit_gc0308(void)
{
 	return 0;
}

#undef CMOS_DEV_GC_C

