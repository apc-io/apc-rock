/*++ 
 * linux/drivers/video/wmt/devices/sil902x.h
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

#ifndef SIL902X_H
/* To assert that only one occurrence is included */
#define SIL902X_H
/*-------------------- MODULE DEPENDENCY -------------------------------------*/

/*	following is the C++ header	*/
#ifdef	__cplusplus
extern	"C" {
#endif

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/
/* #define  SIL902X_XXXX  1    *//*Example*/
#define SIL902X_STATUS_ARG_NUM	3

/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  sil902x_xxx_t;  *//*Example*/
typedef struct {
	int (*init)(void);
	int (*poll)(void);
	void (*cp_enable)(int enable);
	void (*get_status)(int *arg);
	void (*config)(int vic);
} sil902x_ko_ops_t;

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef SIL902X_C /* allocate memory for variables only in xxx.c */
#define EXTERN
#else
#define EXTERN   extern
#endif /* ifdef SIL902X_C */

/* EXTERN int      sil902x_xxx; *//*Example*/
EXTERN void sil902x_init_module(int ko);
EXTERN int sil902x_enable_access(void);
EXTERN int sil902x_get_cp_support(void);

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/
/* #define SIL902X_XXX_YYY   xxxx *//*Example*/
/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  sil902x_xxx(void); *//*Example*/

#ifdef	__cplusplus
}
#endif	
#endif /* ifndef SIL902X_H */

