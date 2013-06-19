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


#ifndef __DESC_H__
#define __DESC_H__


#if !defined(__TTYPE_H__)
#include "ttype.h"
#endif


/*---------------------  Export Definitions -------------------------*/

#define B_OWNED_BY_CHIP     1           //
#define B_OWNED_BY_HOST     0           //

//
// Bits in the RSR0 register
//
#define RSR0_BUFF           0x80        //
#define RSR0_SERR           0x40        //
#define RSR0_RUNT           0x20        //
#define RSR0_LONG           0x10        //
#define RSR0_FOV            0x08        //
#define RSR0_FAE            0x04        //
#define RSR0_CRC            0x02        //
#define RSR0_RERR           0x01        //

//
// Bits in the RSR1 register
//
#define RSR1_RXOK           0x80        // rx OK
#define RSR1_VIDHIT         0x40        // VID Hit
#define RSR1_MAR            0x20        // MAC accept multicast address packet
#define RSR1_BAR            0x10        // MAC accept broadcast address packet
#define RSR1_PHY            0x08        // MAC accept physical address packet
#define RSR1_CHN            0x04        // chain buffer, always = 1
#define RSR1_STP            0x02        // start of packet
#define RSR1_EDP            0x01        // end of packet

#define PQSTS_IPOK          0x20        //IP Checkusm validatiaon ok
#define PQSTS_TUOK          0x10        //TCP/UDP Checkusm validatiaon ok
#define PQSTS_IPKT          0x08        //Received an IP packet
#define PQSTS_TCPKT         0x04        //Received a TCP packet
#define PQSTS_UDPKT         0x02        //Received a UDP packet
#define PQSTS_TAG           0x01        //Received a tagged packet

//
// Bits in the TSR0 register
//
#define TSR0_CDH            0x80        // AQE test fail (CD heartbeat)
#define TSR0_COLS           0x10        // experience collision in this transmit event
#define TSR0_NCR3           0x08        // collision retry counter[3]
#define TSR0_NCR2           0x04        // collision retry counter[2]
#define TSR0_NCR1           0x02        // collision retry counter[1]
#define TSR0_NCR0           0x01        // collision retry counter[0]

//
// Bits in the TSR1 register
//
#define TSR1_TERR           0x80        //
#define TSR1_JAB            0x40        // jabber condition occured
#define TSR1_SERR           0x20        //
#define TSR1_TBUFF          0x10        //
#define TSR1_UDF            0x08        //
#define TSR1_CRS            0x04        //
#define TSR1_OWC            0x02        // late collision
#define TSR1_ABT            0x01        //

//
// Bits in the TCR register
//
#define TCR_IC              0x80        // assert interrupt immediately
          // while descriptor has been send complete
#define TCR_EDP             0x40        // end of packet
#define TCR_STP             0x20        // start of packet
#define TCR_TCPCK           0x10        // request TCP checksum calculation.
#define TCR_UDPCK           0x08        // request UDP checksum calculation.
#define TCR_IPCK            0x04        // request TCP checksum calculation.
#define TCR_CRC             0x01        // disable CRC generation


// max transmit or receive buffer size
//#define CB_MAX_BUF_SIZE     1536U     // max buffer size
#define CB_MAX_BUF_SIZE     2048U       // max buffer size
#define CB_BUF_SIZE_65      2048U       // default Rx buffer size for 3065
                          // NOTE: must be multiple of 4
#define CB_MAX_RD_NUM       64          // MAX # of RD
#define CB_MAX_TD_NUM       32          // MAX # of TD

#define CB_INIT_RD_NUM       10         // init # of RD, for setup default
#define CB_INIT_TD_NUM       10         // init # of TD, for setup default

// for 3106S
#define CB_TD_RING_NUM      7           // # of TD rings.

#define CB_MAX_DESC_PER_PKT  4          // max descriptors per packet (Tx)

// max number of physical segments
// in a single NDIS packet. Above this threshold, the packet
// is copied into a single physically contiguous buffer
#define CB_MAX_SEGMENT      (1 * CB_MIN_TX_DESC)

#define CB_MAP_REGISTER_NUM (1 * CB_MAX_SEGMENT)

// if collisions excess 15 times , tx will abort, and
// if tx fifo underflow, tx will fail
// we should try to resend it
#define CB_MAX_TX_ABORT_RETRY   3


/*---------------------  Export Types  ------------------------------*/

typedef struct tagRDES0 {
    BYTE    byRSR0;
    BYTE    byRSR1;
    WORD    f15FrameLen : 15;
    WORD    f1Owner : 1;
} SRDES0;

typedef struct tagRDES1 {
    WORD    f15RxBufLen : 15;
    WORD    f1Reserve : 1;
    BYTE    byPQSTS;                 // VT3106
    BYTE    byIPKT;                  // VT3106
} SRDES1;

//
// receive descriptor
//
typedef struct tagSRxDesc {
    SRDES0  m_rd0RD0;
    SRDES1  m_rd1RD1;
    DWORD   m_dwRxBufferAddr;       // pointer to logical buffer
    DWORD   m_dwRxNextDescAddr;     // pointer to next logical descriptor
} SRxDesc, *PSRxDesc;
typedef const SRxDesc*      PCSRxDesc;

typedef struct tagTDES0 {
    BYTE    byTSR0;
    BYTE    byTSR1;
    WORD    f12VID:12;
    WORD    f4Prioity:3;
    WORD    f1Owner : 1;
} STDES0;

typedef struct tagTDES1 {
    WORD    f15TxBufLen : 15;
    WORD    f1Chain : 1;
    BYTE    byTCR;
    BYTE    byReserve;
} STDES1;

//
// transmit descriptor
//
typedef struct tagSTxDesc {
    STDES0  m_td0TD0;
    STDES1  m_td1TD1;
    DWORD   m_dwTxBufferAddr;       // pointer to logical buffer
    DWORD   m_dwTxNextDescAddr;     // pointer to next logical descriptor
} STxDesc, *PSTxDesc;
typedef const STxDesc*      PCSTxDesc;

/*---------------------  Export Macros ------------------------------*/

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/


#endif // __DESC_H__

