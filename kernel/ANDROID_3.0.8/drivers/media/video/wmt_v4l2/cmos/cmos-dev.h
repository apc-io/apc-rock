/*++ 
 * linux/drivers/media/video/wmt_v4l2/cmos/cmos-dev.h
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

#ifndef CMOS_DEV_H
/* To assert that only one occurrence is included */
#define CMOS_DEV_H
/*-------------------- MODULE DEPENDENCY -------------------------------------*/

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/


/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  viaapi_xxx_t;  *//*Example*/

/*------------------------------------------------------------------------------
    Definitions of structures
------------------------------------------------------------------------------*/

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef CMOS_DEV_C 
    #define EXTERN
#else
    #define EXTERN   extern
#endif 

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/

/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  viaapi_xxxx(vdp_Void); *//*Example*/
#include <linux/videodev2.h> //for pix_color_format

typedef struct {
	int width;
	int height;
	int cmos_v_flip;
	int cmos_h_flip;
	unsigned int pix_color_format;
	int is_rawdata;
} cmos_init_arg_t;

int cmos_init_ext_device(cmos_init_arg_t  *init_arg);
int cmos_exit_device(int width, int height);
short cmos_get_dev_id(char *dev_name);
int cmos_get_dev_clk_invert(void);
unsigned char cmos_get_dev_init_frame_delay_count(void);

    
#endif /* ifndef CMOS_DEV_H */

/*=== END cmos-ov.h ==========================================================*/
