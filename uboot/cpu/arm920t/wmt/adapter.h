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


#ifndef __ADAPTER_H__
#define __ADAPTER_H__

#if !defined(__TTYPE_H__)
#include "ttype.h"
#endif
#if !defined(__TETHER_H__)
#include "tether.h"
#endif
#if !defined(__MIB_H__)
#include "mib.h"
#endif

#include <malloc.h>

#if !defined(__DESC_H__)
#include "desc.h"
#endif
#if !defined(__MEM_H__)
#include "mem.h"
#endif
#include <common.h>
#include "tbit.h"
/*---------------------  Export Definitions -------------------------*/
//
// constants
//
#define W_DEVICE_ID_3106A   0x3106      // device ID for 3106A
#define W_DEVICE_ID_3053A   0x3053

//
// Adapter Information
//
typedef struct tagSAdapterInfo {

    UINT    cbTotalAdapterNum;
    UINT    uAdapterIndex;              // device index

    // got from PCI regs
    WORD    wDevId;                     // The Device ID of this adapter
    BYTE    byRevId;                    // The Revision Number of this adapter
    DWORD   dwIoBase;
    BYTE    byPMRegOffset;              // the content of Cap_Ptr

    // got from MAC regs (loaded from EEPROM)
    BYTE    byPhyId;
    BYTE    abyEtherAddr[U_ETHER_ADDR_LEN];

    // got from MII regs (updated by Link-Change)
    DWORD   dwPhyCmrId;                 // Company/Module/Revision ID of the PHY
    BOOL    bLinkPass;                  // current link status
    BOOL    bSpeed100M;                 // current speed status
    BOOL    bFullDuplex;                // current duplex status
    UINT    uConnectionType;            // for media connect type parameter

    // for RDR
    UINT         cbRD;               // number of total RD
    PSRxDesc     *apRD;              // RD pointer array
    SAllocMap    amRxDescRing;
    SAllocMap    *aamRxDescBuf;
    int          idxRxCurDesc;
	int          idxRxdirtyDesc;
    int          idxRxPktStartDesc;  // if multi-RD, this is the starting RD

    // for TDR
    UINT         cbTD;               // number of total TD
    PSTxDesc     *apTD;              // TD pointer array
    SAllocMap    amTxDescRing;
    SAllocMap    *aamTxDescBuf;
    int          idxTxCurDesc;
	int          idxTxdirtyDesc;
    int          idxTxPktStartDesc;  // if multi-TD, this is the starting TD
    UINT         cbTxAbortRetry;

    DWORD   dwCacheLineSize;

    // for IRQ
    BYTE    byIrqLevel;                 // IRQ number
    PVOID   pvAdapterIsr;               // adapter's ISR

    INT     iRxPktHandled;              // rx packets has been handled
    INT     iTxPktPosted;               // tx packets posted and wait to be sent

    // for statistic
    SStatCounter    scStatistic;

    // for default setting
    BOOL    bTxOn;
    BOOL    bTxContinuous;              // for origional performance test (Tx Continuous - performance test)
    BOOL    bTxContFunTest;             // for functional test (Tx Continuous - functional test)
    BOOL    bTxSinglePacket;
    BOOL    bRandomPktLength;
    BOOL    bRandomPktData;
    LONG    lTxPacketNum;
    LONG    lTxPacketNumShadow;

    // DEBUG....
    BOOL    bWhenRxCrcErrNoBuf;
    BOOL    bWhenRxDataErrNoBuf;
    BOOL    bIncPktLength;
    UINT    cbPktSize;

    PBYTE   pbyTmpBuff;

    // for 3065, multiple descriptors in one packet
    UINT    uTxDescNumPerPacket;
    UINT    uTxPktSize;

    // for 3065, Early Receive handling
    BOOL    bERNow;

    BOOL    bPktFw;
    PVOID   pvFwAdapter;

} SAdapterInfo, *PSAdapterInfo;


typedef struct _adapter_option {
    int     iTdRingNum;
    int     iRDescNum;
    int     iTDescNum;
    UINT    uiBuffsize;
    BYTE    byRevId;
    BOOL    bNoInit;
    ULONG   ulInitCmds;
} SAdapterOpts, *PSAdapterOpts;

#define INIT_CMD_NONE               0
#define INIT_CMD_CLEAR_STICKHW      0x00000001UL

#define W_MAX_TIMEOUT               0x0FFFU     //

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/
extern PSAdapterInfo sg_aAdapter;
extern SAdapterOpts  g_sOptions;

/*---------------------  Export Functions  --------------------------*/
#ifdef __cplusplus
extern "C" {                            /* Assume C declarations for C++ */
#endif /* __cplusplus */

BOOL
ADPbBind(
    PSAdapterInfo pAdapter
    );

BOOL
ADPbInitializeEx(
    PSAdapterInfo pAdapter
    );

BOOL
ADPbShutdown(
    PSAdapterInfo pAdapter
    );

UINT
ADPuInitAll(
    void
    );

VOID
CARDvSetLoopbackMode(
    DWORD   dwIoBase,
    BYTE    byRevId,
    WORD    wLoopbackMode
    );

VOID
CARDvSetMediaLinkMode(
    DWORD   dwIoBase,
    BYTE    byRevId,
    UINT    uConnectionType
    );

#ifdef __cplusplus
}                                       /* End of extern "C" { */
#endif /* __cplusplus */

#endif // __ADAPTER_H__

