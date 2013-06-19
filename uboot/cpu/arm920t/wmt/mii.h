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


#ifndef __MII_H__
#define __MII_H__

#if !defined(__TTYPE_H__)
#include "ttype.h"
#endif


/*---------------------  Export Definitions -------------------------*/
#define CB_DELAY_MII_LB_STABLE    (1000)
#define MAX_PHY_DEVICE            32      // max. # of PHY in a MII bus

//
// Registers in the MII (offset unit is WORD)
//
#define MII_REG_BMCR        0x00        // physical address
#define MII_REG_BMSR        0x01        //
#define MII_REG_PHYID1      0x02        // OUI
#define MII_REG_PHYID2      0x03        // OUI + Module ID + REV ID
#define MII_REG_ANAR        0x04        //
#define MII_REG_ANLPAR      0x05        //
#define MII_REG_MODCFG      0x10
// NS, MYSON only
#define MII_REG_PCR         0x17        //
// ESI only
#define MII_REG_PCSR        0x17        //

//
// Bits in the BMCR register
//
#define BMCR_RESET          0x8000      //
#define BMCR_LBK            0x4000      //
#define BMCR_SPEED          0x2000      //
#define BMCR_AUTO           0x1000      //
#define BMCR_PD             0x0800      //
#define BMCR_ISO            0x0400      //
#define BMCR_REAUTO         0x0200      //
#define BMCR_FDX            0x0100      //

//
// Bits in the BMSR register
//
#define BMSR_AUTOCM         0x0020      //
#define BMSR_LNK            0x0004      //

//
// Bits in the ANAR register
//
#define ANAR_ASMDIR         0x0800      // Asymmetric PAUSE support
#define ANAR_PAUSE          0x0400      // Symmetric PAUSE Support
#define ANAR_T4             0x0200      //
#define ANAR_TXFD           0x0100      //
#define ANAR_TX             0x0080      //
#define ANAR_10FD           0x0040      //
#define ANAR_10             0x0020      //

//
// Bits in the ANLPAR register
//
#define ANLPAR_ASMDIR       0x0800      // Asymmetric PAUSE support
#define ANLPAR_PAUSE        0x0400      // Symmetric PAUSE Support
#define ANLPAR_T4           0x0200      //
#define ANLPAR_TXFD         0x0100      //
#define ANLPAR_TX           0x0080      //
#define ANLPAR_10FD         0x0040      //
#define ANLPAR_10           0x0020      //


// NS, MYSON only

//
// Bits in the PCR register
//
#define PCR_LED4MODE        0x0002      //


// ESI only

//
// Bits in the PCR register
//
#define PCSR_LEDON4ACT      0x0080      //


// Loopback mode
#define MII_LB_NONE         0x00        //
#define MII_LB_INTERNAL     0x01        //
#define MII_LB_ISO          0x02        // isolate endec/twister


//
// Company ID
//
#define CID_REV_ID_MASK_OFF 0xFFFFFFF0UL    // the last 4-bit is revision id,
                                            // we don't care it

#define CID_NS              0x20005C00UL    // OUI = 08-00-17 , 0x2000 5C01
#define CID_ESI             0x00437400UL    //                , 0x0043 7412
#define CID_DAVICOM         0x0181B800UL    // OUI = 00-60-6E , 0x0181 B800
#define CID_DAVICOM_B       0x0181B802UL    //
#define CID_MYSON           0x0302D000UL    //                , 0x0302 D000
#define CID_SEEQ            0x0016F880UL    // OUI = 00-A0-7D , 0x0016 F880
#define CID_BROADCOM        0x00406000UL    //                , 0x0040 61B1
#define CID_KENDIN          0x00221720UL    // OUI = 00-10-A1 , 0x0022 1720
#define CID_ADHOC           0x00225610UL    // OUI = 00-10-A9 , 0x0022 5610
#define CID_AMD_AM79C901_T  0x00006B70UL    //                , 0x0000 6B70
#define CID_VIA_VT3072      0x01018F20UL
//#define CID_VIA_VT3082      0x01018F20UL

#define CB_MAX_COUNT_AUTO_COMPLETE      (0x1244)
                                        // AUTO-NEGO complete, polling time out count
                                        // about 6 sec.

/*---------------------  Export Types  ------------------------------*/

/*---------------------  Export Macros ------------------------------*/

#define MIIvRegBitsOn(dwIoBase, byRevId, byMiiAddr, wBits)      \
{                                                               \
    WORD wOrgData;                                              \
    MIIbReadEmbedded(dwIoBase, byRevId, byMiiAddr, &wOrgData);  \
    MIIbWriteEmbedded(dwIoBase, byRevId, byMiiAddr,             \
                    (WORD)(wOrgData | wBits));                  \
}


#define MIIvRegBitsOff(dwIoBase, byRevId, byMiiAddr, wBits)     \
{                                                               \
    WORD wOrgData;                                              \
    MIIbReadEmbedded(dwIoBase, byRevId, byMiiAddr, &wOrgData);  \
    MIIbWriteEmbedded(dwIoBase, byRevId, byMiiAddr,             \
                    (WORD)(wOrgData & (~wBits)));               \
}


#define MIIvReadPhyCmrId(dwIoBase, byRevId, pdwPhyCmrId)        \
{                                                               \
    MIIbReadEmbedded(dwIoBase, byRevId, MII_REG_PHYID2,         \
                (PWORD)pdwPhyCmrId);                            \
    MIIbReadEmbedded(dwIoBase, byRevId, MII_REG_PHYID1,         \
                ((PWORD)pdwPhyCmrId) + 1);                      \
}


#define MIIvSetAutoNegotiationOn(dwIoBase, byRevId)             \
{                                                               \
    MIIvRegBitsOn(dwIoBase, byRevId, MII_REG_BMCR, (BMCR_AUTO|BMCR_REAUTO)); \
}


#define MIIvSetAutoNegotiationOff(dwIoBase, byRevId)            \
{                                                               \
    MIIvRegBitsOff(dwIoBase, byRevId, MII_REG_BMCR, BMCR_AUTO); \
}


#define MIIvSetResetOn(dwIoBase, byRevId)                       \
{                                                               \
    MIIvRegBitsOn(dwIoBase, byRevId, MII_REG_BMCR, BMCR_RESET); \
}


#define MIIvSetReAuto(dwIoBase, byRevId)                        \
{                                                               \
    MIIvRegBitsOn(dwIoBase, byRevId, MII_REG_BMCR, BMCR_REAUTO);\
}

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/
#ifdef __cplusplus
extern "C" {                            /* Assume C declarations for C++ */
#endif /* __cplusplus */

BOOL
MIIbIsAutoNegotiationOn(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

BOOL
MIIbIsIn100MMode(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

BOOL
MIIbIsInFullDuplexMode(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

BOOL
MIIbIsRegBitsOff(
    DWORD   dwIoBase,
    BYTE    byRevId,
    BYTE    byMiiAddr,
    WORD    wTestBits
    );

BOOL
MIIbIsRegBitsOn(
    DWORD   dwIoBase,
    BYTE    byRevId,
    BYTE    byMiiAddr,
    WORD    wTestBits
    );

BOOL
MIIbReadEmbedded(
    DWORD   dwIoBase,
    BYTE    byRevId,
    BYTE    byMiiAddr,
    PWORD   pwData
    );

BOOL
MIIbWriteEmbedded(
    DWORD   dwIoBase,
    BYTE    byRevId,
    BYTE    byMiiAddr,
    WORD    wData
    );

VOID
MIIvInitialize(
    DWORD   dwIoBase,
    BYTE    byRevId,
    DWORD   dwPhyCmrId
    );

VOID
MIIvSetLoopbackMode(
    DWORD   dwIoBase,
    BYTE    byRevId,
    BYTE    byLoopbackMode
    );

VOID
MIIvWaitForNwayCompleted(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

#ifdef __cplusplus
}                                       /* End of extern "C" { */
#endif /* __cplusplus */

#endif // __MII_H__

