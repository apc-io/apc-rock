/*++ 
 * Common interface for WonderMedia SoC hardware encoder and decoder drivers
 *
 * Copyright (c) 2012  WonderMedia  Technologies, Inc.
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
 * 10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C
--*/
#ifndef CODEC_H
#define CODEC_H

/*-------------------- MODULE DEPENDENCY -------------------------------------*/

typedef enum {
    CODEC_VD_JPEG,  
    CODEC_VD_MSVD,
    CODEC_VE_H264,
    CODEC_MAX,
    CODEC_VE_JPEG, // reserved
} codec_type;

/*------------------------------------------------------------------------------

------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
    Definitions of structures
------------------------------------------------------------------------------*/


/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef CODEC_C 
    #define EXTERN
#else
    #define EXTERN   extern
#endif /* ifdef CODEC_C */

/* EXTERN int      msvd_xxxx; */

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/

/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  msvd_xxxx(vdp_Void); */

int wmt_codec_clock_en(codec_type type, int enable);

/*=== END wmt-codec.h ==========================================================*/
#endif /* #ifndef CODEC_H */

