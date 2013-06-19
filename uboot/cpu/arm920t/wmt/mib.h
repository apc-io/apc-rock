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


#ifndef __MIB_H__
#define __MIB_H__


#if !defined(__TTYPE_H__)
#include "ttype.h"
#endif
#if !defined(__DESC_H__)
#include "desc.h"
#endif


/*---------------------  Export Definitions -------------------------*/
//
// statistic counter
//
typedef struct tagSStatCounter {
    // ISR0,1 status count
    //
    DWORD   dwIsrRxOK;
    DWORD   dwIsrTxOK;
    DWORD   dwIsrRxErr;                 // FOV | BUFF | CRC | FAE
    DWORD   dwIsrTxErr;                 // FUD | BUFF | ABT | BE
    DWORD   dwIsrRxDescUdfl;
    DWORD   dwIsrTxDescUdfl;
    DWORD   dwIsrBusErr;
    DWORD   dwIsrTlyCntOvfl;

    DWORD   dwIsrRxErly;

    union {
        DWORD   dwIsrTxErly;
      DWORD   dwIsrTxFifoUdfl;
    } u1;

    DWORD   dwIsrRxFifoOvfl;
    DWORD   dwIsrRxPktRace;
    DWORD   dwIsrRxNoBuf;
    DWORD   dwIsrTxAbort;
    DWORD   dwIsrLinkStatusChg;

    // IsrKey for RevId < 3065 , GenInt for RevId >= 3065
    union {
        DWORD   dwIsrKey;
        DWORD   dwGenInt;
    } u2;

    DWORD   dwIsrUnknown;               // unknown interrupt count
    DWORD   dwIsrTllyMPA;               // tally counter for Miss Packet Abort
    DWORD   dwIsrTllyCRC;               // tally counter for CRC err

    // MISR for 3065
    DWORD   dwMisrSoftTimer0;
    DWORD   dwMisrSoftTimer1;
    DWORD   dwMisrPhyStatChg;
    DWORD   dwMisrTDWBRace;
    DWORD   dwMisrSSRC;
    DWORD   dwMisrUserDef;
    DWORD   dwMisrPWE;

    DWORD   dwIntLinkUp;
    DWORD   dwIntLinkDown;

    // RSR0,1 status count
    //
    DWORD   dwRsrCRCErr;
    DWORD   dwRsrFrmAlgnErr;
    DWORD   dwRsrFifoOvfl;
    DWORD   dwRsrLong;
    DWORD   dwRsrRunt;
    DWORD   dwRsrBusErr;
    DWORD   dwRsrBufUdfl;
    DWORD   dwRsrErr;                   // FOV | BUFF | CRC | FAE | SERR
    DWORD   dwRsrOK;

    DWORD   dwRsrRxPacket;
    DWORD   dwRsrRxOctet;
    DWORD   dwRsrBroadcast;
    DWORD   dwRsrMulticast;
    DWORD   dwRsrDirected;

    DWORD   dwRsrRxFragment;
    DWORD   dwRsrRxFrmLen64;
    DWORD   dwRsrRxFrmLen65_127;
    DWORD   dwRsrRxFrmLen128_255;
    DWORD   dwRsrRxFrmLen256_511;
    DWORD   dwRsrRxFrmLen512_1023;
    DWORD   dwRsrRxFrmLen1024_1518;
    // for VT3106
    DWORD dwRsrRxVIDHit;
    DWORD dwRsrRxTagFrame;

    // TSR0,1 status count
    //
    DWORD   dwTsrTotalColRetry;         // total collision retry count
    DWORD   dwTsrOnceCollision;         // this packet only occur one collision
    DWORD   dwTsrMoreThanOnceCollision; // this packet occur more than one collision

    DWORD   dwTsrCollision;             // this packet has ever occur collision,
                                        // that is (dwTsrOnceCollision + dwTsrMoreThanOnceCollision)
    DWORD   dwTsrHeartBeat;

    DWORD   dwTsrAbort;
    DWORD   dwTsrLateCollision;
    DWORD   dwTsrCarrierLost;
    DWORD   dwTsrFifoUdfl;
    DWORD   dwTsrBufUdfl;
    DWORD   dwTsrBusErr;
    DWORD   dwTsrJab;
    DWORD   dwTsrErr;                   // FUD | BUFF | ABT
    DWORD   dwTsrOK;

    DWORD   dwTsrTxPacket;
    DWORD   dwTsrTxOctet;
    DWORD   dwTsrBroadcast;
    DWORD   dwTsrMulticast;
    DWORD   dwTsrDirected;

    // RD/TD count
    DWORD   dwCntRxFrmLength;
    DWORD   dwCntTxBufLength;

    BYTE    abyCntRxPattern[16];
    BYTE    abyCntTxPattern[16];

    // for pingpong
    DWORD   dwCntNoResponse;
    DWORD   dwCntSerialNumErr;
    DWORD   dwCntPPTDataErr;

    // PATCH....
    DWORD   dwCntRxDataErr;             // rx buffer data software compare CRC err count
    DWORD   dwIsrContinuePktRace;       // if continueous packet-race happen, should reset
    DWORD   dwIsrContinueNoBuf;         // if continueous no-buffer

    // for multi Tx descriptor
    DWORD   dwTDSizePerPacket[CB_MAX_DESC_PER_PKT];

} SStatCounter, *PSStatCounter;

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/
#ifdef __cplusplus
extern "C" {                            /* Assume C declarations for C++ */
#endif /* __cplusplus */

VOID
STAvClearAllCounter(
    PSStatCounter pStatistic
    );

VOID
STAvUpdateIsrStatCounter(
    PSStatCounter pStatistic,
    DWORD   dwIsr,
    BYTE    byMISR
    );

VOID
STAvUpdateRDStatCounter(
    PVOID   pvAdapter,
    PSRxDesc    prdCurr,
    PBYTE   pbyBuffer,
    UINT    cbFrameLength
    );

VOID
STAvUpdateRDStatCounterEx(
    PVOID   pvAdapter,
    PSRxDesc    prdCurr,
    PBYTE   pbyBuffer,
    UINT    cbFrameLength
    );

VOID
STAvUpdateTDStatCounter(
    PVOID   pvAdapter,
    PSStatCounter pStatistic,
    BYTE    byTSR0,
    BYTE    byTSR1,
    PBYTE   pbyBuffer,
    UINT    cbFrameLength
    );

VOID
STAvUpdateTDStatCounterEx(
    PVOID   pvAdapter,
    PSStatCounter pStatistic,
    BYTE    byTSR0,
    BYTE    byTSR1,
    PBYTE   pbyBuffer,
    DWORD   cbFrameLength
    );

VOID
STAvUpdateTallyCounter(
    PSStatCounter pStatistic,
    WORD    wMPACount,
    WORD    wCRCCount
    );

#ifdef __cplusplus
}                                       /* End of extern "C" { */
#endif /* __cplusplus */

#endif // __MIB_H__

