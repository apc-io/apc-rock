/*++ 
Copyright (c) 2010 WonderMedia Technologies, Inc.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details. You
should have received a copy of the GNU General Public License along with this
program. If not, see http://www.gnu.org/licenses/>.

WonderMedia Technologies, Inc.
10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/


#ifndef __CARD_H__
#define __CARD_H__

#if !defined(__TTYPE_H__)
#include "ttype.h"
#endif
#if !defined(__MAC_H__)
#include "mac.h"
#endif
#if !defined(__MII_H__)
#include "mii.h"
#endif


/*---------------------  Export Definitions -------------------------*/

// media type
#define MEDIA_AUTO              0x00        //
#define MEDIA_100M_HALF         0x01        //
#define MEDIA_100M_FULL         0x02        //
#define MEDIA_10M_HALF          0x03        //
#define MEDIA_10M_FULL          0x04        //
#define MEDIA_1M                0x05        // Home PHY



/*
 * Loopback mode
 */
/* LOBYTE is MAC LB mode, HIBYTE is MII LB mode */
#define CARD_LB_NONE            MAKEWORD(MAC_LB_NONE, MII_LB_NONE)
/* PHY must ISO, avoid MAC loopback packet go out */
#define CARD_LB_MAC             MAKEWORD(MAC_LB_INTERNAL, MII_LB_ISO)   // PHY must ISO, avoid MAC loopback packet go out
#define CARD_LB_MII             MAKEWORD(MAC_LB_PHY, MII_LB_INTERNAL)

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/
#ifdef __cplusplus
extern "C" {                            /* Assume C declarations for C++ */
#endif /* __cplusplus */




#ifdef __cplusplus
}                                       /* End of extern "C" { */
#endif /* __cplusplus */

#endif /* __CARD_H__ */



