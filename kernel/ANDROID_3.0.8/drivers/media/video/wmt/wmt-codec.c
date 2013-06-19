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
#ifndef CODEC_C
#define CODEC_C

#include <linux/device.h>       /* For EXPORT_SYMBOL() */ 
#include <linux/spinlock.h>
#include <mach/hardware.h>

#include "wmt-codec.h"


//#define CODEC_DEBUG

#undef DBG_MSG
#ifdef CODEC_DEBUG
  #define DBG_MSG(fmt, args...)    printk("{%s} " fmt, __FUNCTION__ , ## args)
#else
  #define DBG_MSG(fmt, args...)
#endif
#define DBG_ERR(fmt, args...)      printk("*E* {%s} " fmt, __FUNCTION__ , ## args)


#define REG_READ(addr)            REG32_VAL(addr)
#define REG_WRITE(addr, val)      REG32_VAL(addr) = (val)


/*------------------------------------------------------------------------------
    MSVD_BASE_ADDR(0xD80F3000) were defined in  
    \arch\arm\mach-wmt\include\mach\wmt_mmap.h
------------------------------------------------------------------------------*/
#define REG_MSVD_CLOCK_ENABLE     (MSVD_BASE_ADDR + 0x000)


/*!*************************************************************************
* wmt_codec_clock_en
* 
* Public Function
*/
/*!
* \brief
*	Set source PRD table
*
* \retval  0 if success
*/ 
int wmt_codec_clock_en(codec_type type, int enable)
{
    static DEFINE_SPINLOCK(clk_lock);
    unsigned int clock;
    unsigned long flags;

    if( (type < 0 ) || (type >= CODEC_MAX) ){
        DBG_ERR("Unsupported codec ID %d\n", type);
        return -1;
    }     
    spin_lock_irqsave(&clk_lock, flags);   

    if( type == CODEC_VD_JPEG ) {
        if(enable){
            auto_pll_divisor(DEV_JDEC, CLK_ENABLE, 0, 0);
            auto_pll_divisor(DEV_MSVD, CLK_ENABLE, 0, 0);
            auto_pll_divisor(DEV_VDU,  CLK_ENABLE, 0, 0);
            auto_pll_divisor(DEV_NA0,  CLK_ENABLE, 0, 0);

            clock = REG_READ(REG_MSVD_CLOCK_ENABLE);
            REG_WRITE(REG_MSVD_CLOCK_ENABLE, (clock | 0x000000F0));
        }
        else{
            clock = REG_READ(REG_MSVD_CLOCK_ENABLE);
            REG_WRITE(REG_MSVD_CLOCK_ENABLE, (clock & (~0x000000F0)));                
            auto_pll_divisor(DEV_NA0,  CLK_DISABLE, 0, 0);
            auto_pll_divisor(DEV_VDU,  CLK_DISABLE, 0, 0);
            auto_pll_divisor(DEV_MSVD, CLK_DISABLE, 0, 0);
            auto_pll_divisor(DEV_JDEC, CLK_DISABLE, 0, 0);
        } 
    }
    else if( type == CODEC_VD_MSVD ) {
        if(enable) {
            auto_pll_divisor(DEV_MSVD, CLK_ENABLE, 0, 0);
            auto_pll_divisor(DEV_VDU,  CLK_ENABLE, 0, 0);
            auto_pll_divisor(DEV_NA0,  CLK_ENABLE, 0, 0);
            clock = REG_READ(REG_MSVD_CLOCK_ENABLE);
            REG_WRITE(REG_MSVD_CLOCK_ENABLE, (clock | 0x0000000F)); //msvd
        }
        else {
            clock = REG_READ(REG_MSVD_CLOCK_ENABLE);
            REG_WRITE(REG_MSVD_CLOCK_ENABLE, (clock & (~0x0000000F)));
            auto_pll_divisor(DEV_NA0,  CLK_DISABLE, 0, 0);
            auto_pll_divisor(DEV_VDU,  CLK_DISABLE, 0, 0);
            auto_pll_divisor(DEV_MSVD, CLK_DISABLE, 0, 0);
        }        
    } 
    else if( type == CODEC_VE_H264 ) {
        if(enable) {
            auto_pll_divisor(DEV_NA0, CLK_ENABLE, 0, 0);
            auto_pll_divisor(DEV_MSVD, CLK_ENABLE, 0, 0);
            auto_pll_divisor(DEV_H264, CLK_ENABLE, 0, 0);
            clock = REG_READ(REG_MSVD_CLOCK_ENABLE);
            REG_WRITE(REG_MSVD_CLOCK_ENABLE, (clock | 0x00000F00)); //h264enc
        }
        else {
            clock = REG_READ(REG_MSVD_CLOCK_ENABLE);
            REG_WRITE(REG_MSVD_CLOCK_ENABLE, (clock & (~0x00000F00)));
            auto_pll_divisor(DEV_H264, CLK_DISABLE, 0, 0);
            auto_pll_divisor(DEV_MSVD, CLK_DISABLE, 0, 0);
            auto_pll_divisor(DEV_NA0,  CLK_DISABLE, 0, 0);
        }                
    }
    spin_unlock_irqrestore(&clk_lock, flags);
    
    return 0;
} /* End of wmt_codec_clock_en() */


EXPORT_SYMBOL(wmt_codec_clock_en);

/*--------------------End of Function Body -----------------------------------*/
#endif /* ifndef CODEC_C */
