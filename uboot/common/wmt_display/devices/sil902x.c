/*++ 
 * linux/drivers/video/wmt/sil902x.c
 * WonderMedia video post processor (VPP) driver
 *
 * Copyright c 2012  WonderMedia  Technologies, Inc.
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

#define SIL902X_C
/*--------------------------------------------------------------------* 
SiI902x Software customization for PC platform.
Based on :
	Video input configuration=
		24 bit RGB in + Hsync + Vsync + DE + PCLK 
		rising edge is the 1st clock after DE=High
	Audio input configuration=
		SPDIF, word size, sample frequency all refer to stream header

	SiI902x i2c device address=0x72. ( CI2CA pin=LOW) 
//================== i2c routine =========================
SiI902x i2c max speed is 100KHz.
A version max speed is 400KHz

Data = I2C_ReadByte(TX_SLAVE_ADDR, RegOffset);
I2C_WriteByte(TPI_BASE_ADDR, RegOffset, Data);
I2C_WriteBlock(TPI_BASE_ADDR, TPI_Offset, pData, NBytes);
I2C_ReadBlock(TPI_BASE_ADDR, TPI_Offset, pData, NBytes);

------------------------------------------------------------------------------*/
// #define DEBUG
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "../vout.h"
#include "sil902x.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  SIL902X_XXXX  xxxx    *//*Example*/

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define SIL902X_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx sil902x_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in sil902x.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  sil902x_xxx;        *//*Example*/

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void sil902x_xxx(void); *//*Example*/
int sil902x_module_init(void)
{	
//	vout_device_register(&sil902x_vout_dev_ops);
	return 0;
} /* End of sil902x_module_init */
module_init(sil902x_module_init);
/*--------------------End of Function Body -----------------------------------*/
#undef SIL902X_C

