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


#ifndef __MAC_H__
#define __MAC_H__

#if !defined(__TTYPE_H__)
#include "ttype.h"
#endif
#if !defined(__TMACRO_H__)
#include "tmacro.h"
#endif
#if !defined(__UPC_H__)
#include "upc.h"
#endif


/*---------------------  Export Definitions -------------------------*/
//
// Registers in the MAC
//
#define MAC_REG_PAR         0x00        // physical address
#define MAC_REG_RCR         0x06        //
#define MAC_REG_TCR         0x07        //
#define MAC_REG_CR0         0x08        //
#define MAC_REG_CR1         0x09        //
#define MAC_REG_TQWK        0x0A
#define MAC_REG_ISR         0x0C        //
#define MAC_REG_IMR         0x0E        //
#define MAC_REG_MAR         0x10        //
#define MAC_REG_MCAM        0x10        //
#define MAC_REG_VCAM        0x16        //
#define MAC_REG_CUR_RD_ADDR 0x18        //
#define MAC_REG_CUR_TD_ADDR 0x1C        //
#define MAC_REG_RX_DMA_PTR  0x60        //
#define MAC_REG_MPHY        0x6C        //
#define MAC_REG_MIISR       0x6D        //
#define MAC_REG_BCR0        0x6E        //
#define MAC_REG_BCR1        0x6F        //
#define MAC_REG_MIICR       0x70        //
#define MAC_REG_MIIAD       0x71        //
#define MAC_REG_MIIDATA     0x72        //
#define MAC_REG_EECSR       0x74        //
#define MAC_REG_TEST        0x75        //
#define MAC_REG_CFGA        0x78        //
#define MAC_REG_CFGB        0x79        //
#define MAC_REG_CFGC        0x7A        //
#define MAC_REG_CFGD        0x7B        //
#define MAC_REG_CNTR_MPA    0x7C        //
#define MAC_REG_CNTR_CRC    0x7E        //

// Registers only in integrated LAN
#define MAC_REG_BISTCSR0    0x50        //
#define MAC_REG_BISTCSR1    0x51        //
#define MAC_REG_BISTCSR2    0x52        //
#define MAC_REG_BISTCSR3    0x53        //

// Registers for 3065
#define MAC_REG_GFTEST      0x54        //
#define MAC_REG_RFTCMD      0x55        //
#define MAC_REG_TFTCMD      0x56        //
#define MAC_REG_GFSTATUS    0x57        //
#define MAC_REG_BNRY        0x58        //
#define MAC_REG_CURR        0x5A        //
#define MAC_REG_FIFO_DATA   0x5C        //
#define MAC_REG_CUR_RXDMA   0x60        //
#define MAC_REG_CUR_TXDMA   0x64        //

// Registers for 3359 Ethernet MAC module
#define MAC_REG_EEADR       0x7C        //
#define MAC_REG_EECMD       0x7D        //
#define MAC_REG_EESD_CKSUM  0x7E        //
#define MAC_REG_DPMSR       0x7F        //

#define MAC_REG_MISC_CR0    0x80        //
#define MAC_REG_MISC_CR1    0x81        //
#define MAC_REG_PMCPORT     0x82        //
#define MAC_REG_STICKHW     0x83        //
#define MAC_REG_MISR        0x84        //
#define MAC_REG_MIMR        0x86        //
#define MAC_REG_CAMMSK      0x88        //
#define MAC_REG_BPMA        0x8C        //
#define MAC_REG_BPMD        0x8F        //
#define MAC_REG_BPCMD       0x90        //
#define MAC_REG_BPIN_DATA   0x91        //
#define MAC_REG_CAMC        0x92        //
#define MAC_REG_CAMADD      0x93        //
#define MAC_REG_EECHKSUM    0x93        //
#define MAC_REG_SUS_MII_AD  0x94        //
#define MAC_REG_MIBCR       0x94
#define MAC_REG_MIBDATA     0x96        //
#define MAC_REG_SUS_PHY_ID  0x96        //
#define MAC_REG_MIBPORT     0x96        //
#define MAC_REG_MIBDAT      0x97        //
#define MAC_REG_PAUSE_TIMER 0x98        //
#define MAC_REG_FLOWCR0     0x98
#define MAC_REG_SOFT_TIMER0 0x9C        //
#define MAC_REG_SOFT_TIMER1 0x9E        //
#define MAC_REG_WOLCR_SET   0xA0        //
#define MAC_REG_PWCFG_SET   0xA1        //
#define MAC_REG_TSTREG_SET  0xA2        //
#define MAC_REG_WOLCR1_SET  0xA2        //
#define MAC_REG_WOLCG_SET   0xA3        //
#define MAC_REG_WOLCR_CLR   0xA4        //
#define MAC_REG_PWCFG_CLR   0xA5        //
#define MAC_REG_TSTREG_CLR  0xA6        //
#define MAC_REG_WOLCR1_CLR  0xA6        //
#define MAC_REG_WOLCG_CLR   0xA7        //
#define MAC_REG_PWRCSR_SET  0xA8        //
#define MAC_REG_PWRCSR1_SET 0xA9        //
#define MAC_REG_PWRCSR_CLR  0xAC        //
#define MAC_REG_PWRCSR1_CLR 0xAD        //
#define MAC_REG_PATRN_CRC0  0xB0        //
#define MAC_REG_PATRN_CRC1  0xB4        //
#define MAC_REG_PATRN_CRC2  0xB8        //
#define MAC_REG_PATRN_CRC3  0xBC        //
#define MAC_REG_BYTEMSK0_0  0xC0        //
#define MAC_REG_BYTEMSK0_1  0xC4        //
#define MAC_REG_BYTEMSK0_2  0xC8        //
#define MAC_REG_BYTEMSK0_3  0xCC        //
#define MAC_REG_BYTEMSK1_0  0xD0        //
#define MAC_REG_BYTEMSK1_1  0xD4        //
#define MAC_REG_BYTEMSK1_2  0xD8        //
#define MAC_REG_BYTEMSK1_3  0xDC        //
#define MAC_REG_BYTEMSK2_0  0xE0        //
#define MAC_REG_BYTEMSK2_1  0xE4        //
#define MAC_REG_BYTEMSK2_2  0xE8        //
#define MAC_REG_BYTEMSK2_3  0xEC        //
#define MAC_REG_BYTEMSK3_0  0xF0        //
#define MAC_REG_BYTEMSK3_1  0xF4        //
#define MAC_REG_BYTEMSK3_2  0xF8        //
#define MAC_REG_BYTEMSK3_3  0xFC        //


//
// Bits in the RCR register
//
#define RCR_RRFT2           0x80        //
#define RCR_RRFT1           0x40        //
#define RCR_RRFT0           0x20        //
#define RCR_PROM            0x10        //
#define RCR_AB              0x08        //
#define RCR_AM              0x04        //
#define RCR_AR              0x02        //
#define RCR_SEP             0x01        //

//
// Bits in the TCR register
//
#define TCR_RTSF            0x80        //
#define TCR_RTFT1           0x40        //
#define TCR_RTFT0           0x20        //
#define TCR_OFSET           0x08        //
#define TCR_LB1             0x04        // loopback[1]
#define TCR_LB0             0x02        // loopback[0]
#define TCR_PQEN            0x01

//
// Bits in the CR0 register
//
#define CR0_RDMD            0x40        // rx descriptor polling demand
#define CR0_TDMD            0x20        // tx descriptor polling demand
#define CR0_TXON            0x10        //
#define CR0_RXON            0x08        //
#define CR0_STOP            0x04        // stop MAC, default = 1
#define CR0_STRT            0x02        // start MAC
#define CR0_INIT            0x01        // start init process

//
// Bits in the CR1 register
//
#define CR1_SFRST           0x80        // software reset
#define CR1_RDMD1           0x40        //
#define CR1_TDMD1           0x20        //
#define CR1_KEYPAG          0x10        //
#define CR1_DPOLL           0x08        // disable rx/tx auto polling
#define CR1_FDX             0x04        // full duplex mode
#define CR1_ETEN            0x02        // early tx mode
#define CR1_EREN            0x01        // early rx mode

//
// Bits in the IMR register
//
#define IMR_KEYM            0x8000      //
#define IMR_SRCM            0x4000      //
#define IMR_ABTM            0x2000      //
#define IMR_NORBFM          0x1000      //
#define IMR_PKTRAM          0x0800      //
#define IMR_OVFM            0x0400      //
#define IMR_ETM             0x0200      //
#define IMR_ERM             0x0100      //
#define IMR_CNTM            0x0080      //
#define IMR_BEM             0x0040      //
#define IMR_RUM             0x0020      //
#define IMR_TUM             0x0010      //
#define IMR_TXEM            0x0008      //
#define IMR_RXEM            0x0004      //
#define IMR_PTXM            0x0002      //
#define IMR_PRXM            0x0001      //

//
// Bits in the ISR (MISR) register
//
#define ISR_GENI            0x00008000UL    //  for 3065
#define ISR_SRCI            0x00004000UL    //
#define ISR_ABTI            0x00002000UL    //
#define ISR_NORBF           0x00001000UL    //
#define ISR_PKTRA           0x00000800UL    //
#define ISR_OVFI            0x00000400UL    //
#define ISR_UDFI            0x00000200UL    //  for 3065
#define ISR_ERI             0x00000100UL    //
#define ISR_CNT             0x00000080UL    //
#define ISR_BE              0x00000040UL    //
#define ISR_RU              0x00000020UL    //
#define ISR_TU              0x00000010UL    //
#define ISR_TXE             0x00000008UL    //
#define ISR_RXE             0x00000004UL    //
#define ISR_PTX             0x00000002UL    //
#define ISR_PRX             0x00000001UL    //
// Bits in MISR
#define ISR_PWEINT          0x00800000UL    // power event report in test mode
#define ISR_UDPINT_CLR      0x00400000UL    // userdefined, host driven interrupt.clear
#define ISR_UDPINT_SET      0x00200000UL    // userdefined, host driven interrupt.Set
#define ISR_SSRCI           0x00100000UL    // suspend well mii polling status change interrupt
#define ISR_TDWBRAI         0x00080000UL    // TD WB queue race
#define ISR_PHYINT          0x00040000UL    // PHY state change interrupt, active by
                                            // PHYINTEN (misc.cr[9]) in normal mode
#define ISR_TM1_INT         0x00020000UL    //
#define ISR_TM0_INT         0x00010000UL    //


//
// Bits in the MIISR register
//
#define MIISR_N_FDX         0x04
#define MIISR_LNKFL         0x02        //
#define MIISR_SPEED         0x01        //

//
// Bits in the MIICR register
//
#define MIICR_MAUTO         0x80        //
#define MIICR_RCMD          0x40        //
#define MIICR_WCMD          0x20        //
#define MIICR_MDPM          0x10        //
#define MIICR_MOUT          0x08        //
#define MIICR_MDO           0x04        //
#define MIICR_MDI           0x02        //
#define MIICR_MDC           0x01        //

//
// Bits in the MIIAD register
//
#define MIIAD_MIDLE         0x80        //
#define MIIAD_MSRCEN        0x40        //
#define MIIAD_MDONE         0x20        //
//
// Bits in the MIBCR register
//
#define MIBCR_MIBEN         0x10
#define MIBCR_MIBHALF       0x20
#define MIBCR_MIBINC        0x40
#define MIBCR_MIBRTN        0x80
//
// Bits in the EECSR register
//
#define EECSR_EEPR          0x80        // eeprom programed status, 73h means programed
#define EECSR_EMBP          0x40        // eeprom embeded programming
#define EECSR_AUTOLD        0x20        // eeprom content reload
#define EECSR_DPM           0x10        // eeprom direct programming
#define EECSR_CS            0x08        // eeprom CS pin
#define EECSR_SK            0x04        // eeprom SK pin
#define EECSR_DI            0x02        // eeprom DI pin
#define EECSR_DO            0x01        // eeprom DO pin

//
// Bits in the BCR0 register
//
#define BCR0_BOOT_MASK      ((BYTE) 0xC0)
#define BCR0_BOOT_INT19     ((BYTE) 0x00)
#define BCR0_BOOT_INT18     ((BYTE) 0x40)
#define BCR0_BOOT_LOCAL     ((BYTE) 0x80)
#define BCR0_BOOT_BEV       ((BYTE) 0xC0)

#define BCR0_MED2           0x80        //
#define BCR0_LED100M        0x40        //
#define BCR0_CRFT2          0x20        //
#define BCR0_CRFT1          0x10        //
#define BCR0_CRFT0          0x08        //
#define BCR0_DMAL2          0x04        //
#define BCR0_DMAL1          0x02        //
#define BCR0_DMAL0          0x01        //

//
// Bits in the BCR1 register
//
#define BCR1_MED1           0x80        //
#define BCR1_MED0           0x40        //
#define BCR1_CTSF           0x20        //
#define BCR1_CTFT1          0x10        //
#define BCR1_CTFT0          0x08        //
#define BCR1_POT2           0x04        //
#define BCR1_POT1           0x02        //
#define BCR1_POT0           0x01        //

//
// Bits in the CFGA register
//
#define CFGA_EELOAD         0x80        // enable eeprom embeded and direct programming
#define CFGA_MMIOEN         0x20        // memory mapped I/O enable (3043)
#define CFGA_LED0S0         0x01        //

// VT3106
#define CFGA_PMHCTG         0x20        // pattern match crc calculation TAG info inclusion

//
// Bits in the CFGB register
//
#define CFGB_QPKTDIS        0x80        //
#define CFGB_MRLDIS         0x20        //

//
// Bits in the CFGC register
//
#define CFGC_BOOT_RPL       ((BYTE) 0x80) //Boot method selection for VT3106S
#define CFGC_MEDEN          0x80        //
#define CFGC_BROPT          0x40        //
#define CFGC_DLYEN          0x20        //
#define CFGC_DTSEL          0x10        //
#define CFGC_BTSEL          0x08        //
#define CFGC_BPS2           0x04        // bootrom select[2]
#define CFGC_BPS1           0x02        // bootrom select[1]
#define CFGC_BPS0           0x01        // bootrom select[0]
//
// Bits in the CAMCR register
//
#define CAMC_CAMRD          0x08
#define CAMC_CAMWR          0x04
#define CAMC_VCAMSL         0x02
#define CAMC_CAMEN          0x01

//
// Bits in the CFGD register
//
#define CFGD_GPIOEN         0x80        //
#define CFGD_DIAG           0x40        //
#define CFGD_BOOT_RPL       ((BYTE) 0x20) //Boot method selection for VT3106J, VT3206
#define CFGD_MAGIC          0x10        //
#define CFGD_CRADOM         0x08        //
#define CFGD_CAP            0x04        //
#define CFGD_MBA            0x02        //
#define CFGD_BAKOPT         0x01        //

#define CFGD_MMIOEN         0x80        // memory mapped I/O enable (3065/3106)

// for VT3065

// Bits in STICKHW
#define STICKHW_LEGWOLEN        0x0080  // status for software reference
#define STICKHW_LEGACY_WOLSR    0x0008
#define STICKHW_LEGACY_WOLEN    0x0004
#define STICKHW_DS1_SHADOW      0x0002  // R/W by software/cfg cycle
#define STICKHW_DS0_SHADOW      0x0001  // suspend well DS write port

// Bits in MISC.CR0
#define MISC_CR0_TM0US          0x20
#define MISC_CR0_FDXTFEN        0x10    // Full-duplex flow control TX enable
#define MISC_CR0_FDXRFEN        0x08    // Full-duplex flow control RX enable
#define MISC_CR0_HDXFEN         0x04    // Half-duplex flow control enable
#define MISC_CR0_TIMER0_SUSPEND 0x02
#define MISC_CR0_TIMER0_EN      0x01

// Bits in MISC.CR1
#define MISC_CR1_FORSRST        0x40
#define MISC_CR1_VAUXJMP        0x20
#define MISC_CR1_PHYINT         0x02
#define MISC_CR1_TIMER1_EN      0x01

// Bits in BPCMD
#define BPCMD_BPDNE             0x80
#define BPCMD_EBPWR             0x02
#define BPCMD_EBPRD             0x01

// Bits in MISR
#define MISR_PWEINT             0x80    // power event report in test mode
#define MISR_UDPINT_CLR         0x40    // userdefined, host driven interrupt.clear
#define MISR_UDPINT_SET         0x20    // userdefined, host driven interrupt.Set
#define MISR_SSRCI              0x10    // suspend well mii polling status change interrupt
#define MISR_TDWBRAI            0x08    // TD WB queue race
#define MISR_PHYINT             0x04    // PHY state change interrupt, active by
                                        // PHYINTEN (misc.cr[9]) in normal mode
#define MISR_TM1_INT            0x02
#define MISR_TM0_INT            0x01

// Bits in WOLCR
#define WOLCR_LNKOFF_EN         0x80    // link off detected enable
#define WOLCR_LNKON_EN          0x40    // link on detected enable
#define WOLCR_MAGPKT_EN         0x20    // magic packet filter enable
#define WOLCR_UNICAST_EN        0x10    // unicast filter enable
#define WOLCR_MSWOLEN3          0x08    // enable pattern match filtering
#define WOLCR_MSWOLEN2          0x04
#define WOLCR_MSWOLEN1          0x02
#define WOLCR_MSWOLEN0          0x01


// Bits in PWCFG
#define PWCFG_SMIITIME          0x80    // internal MII I/F timing
#define PWCFG_PCISTICK          0x40    // PCI sticky R/W enable
#define PWCFG_WOLTYPE           0x20    // pulse(1) or button (0)
#define PWCFG_LEGCY_WOL         0x10
#define PWCFG_PMCSR_PME_SR      0x08
#define PWCFG_PMCSR_PME_EN      0x04    // control by PCISTICK
#define PWCFG_LEGACY_WOLSR      0x02    // Legacy WOL_SR shadow
#define PWCFG_LEGACY_WOLEN      0x01    // Legacy WOL_EN shadow

// Bits in TestReg
#define TSTREG_SGENWATCH        0x08
#define TSTREG_SMCSNOOP         0x04
#define TSTREG_SMACTEST         0x02
#define TSTREG_SNORMAL          0x01

// Bits in WOLCFG
#define WOLCFG_PME_OVR          0x80    // for legacy use, force PMEEN always
#define WOLCFG_SFDX             0x40    // full duplex while in WOL mode
#define WOLCFG_SAM              0x20    // accept multicast case reset, default=0
#define WOLCFG_SAB              0x10    // accept broadcast case reset, default=0
#define WOLCFG_SMIIACC          0x08    // ??
#define WOLCFG_SMIIOPT          0x04    // MIIOPT to extend clock in suspendwell
#define WOLCFG_SSRCEN           0x02    // suspend well mii status change enable
#define WOLCFG_PHYINTEN         0x01    // 0:PHYINT trigger enable, 1:use internal MII
                                        // to report status change

// Bits in DPMSR
#define DPMSR_PDPM              0x04    // Direct Programming Request

// Bits in RAMBIST CSRs
#define BISTCSR0_TSTPAT0        0x01    //
#define BISTCSR0_TSTPAT1        0x02    //
#define BISTCSR0_TSTMODE        0x04    //
#define BISTCSR0_TX0RX1         0x08    //
#define BISTCSR1_BISTGO         0x01    //
#define BISTCSR2_TSTSR          0x01    //
#define BISTCSR3_CMDPRTEN       0x02    //
#define BISTCSR3_RAMTSTEN       0x01    //


// Ethernet address filter type
#define PKT_TYPE_NONE               0x0000  // turn off receiver
#define PKT_TYPE_DIRECTED           0x0001  // obselete, directed address is always accepted
#define PKT_TYPE_MULTICAST          0x0002
#define PKT_TYPE_ALL_MULTICAST      0x0004
#define PKT_TYPE_BROADCAST          0x0008
#define PKT_TYPE_PROMISCUOUS        0x0020
#define PKT_TYPE_LONG               0x2000  // NOTE.... the definition of LONG is >2048 bytes in our chip
#define PKT_TYPE_RUNT               0x4000
#define PKT_TYPE_ERROR              0x8000  // accept error packets, e.g. CRC error


// Loopback mode
#define MAC_LB_NONE         0x00        //
#define MAC_LB_INTERNAL     0x01        //
#define MAC_LB_PHY          0x02        // MII or Internal-10BaseT loopback


// Forced media type
#define FORCED_MEDIA_NONE       0x00    //
#define FORCED_MEDIA_AUTO       0x01    //
#define FORCED_MEDIA_100M_HALF  0x02    // hub card
#define FORCED_MEDIA_100M_FULL  0x03    // fiber channel
#define FORCED_MEDIA_10M_HALF   0x04    //
#define FORCED_MEDIA_10M_FULL   0x05    //


// enabled mask value of irq
#if defined(_SIM)
#define IMR_MASK_VALUE      0x000BFFFFUL    // initial value of IMR
#else
#define IMR_MASK_VALUE      0x000BD77FUL    // initial value of IMR
                                            // ignore ABTI,PKTRACE,CNT to
                                            // reduce intr. frequency
                                            // NOTE.... do not enable NoBuf int mask at NDIS driver
                                            // when (1) NoBuf -> RxThreshold = SF
                                            //      (2) OK    -> RxThreshold = original value
#endif


// wait time within loop
#define CB_DELAY_LOOP_WAIT  10          // 10ms
#define CB_DELAY_MII_STABLE 660         //

// max time out delay time
#define W_MAX_TIMEOUT       0x0FFFU     //

/*---------------------  Export Types  ------------------------------*/

/*---------------------  Export Macros ------------------------------*/
#define MACvRegBitsOn(dwIoBase, byRegOfs, byBits)           \
{                                                           \
    BYTE byData;                                            \
    VNSvInPortB(dwIoBase + byRegOfs, &byData);              \
    VNSvOutPortB(dwIoBase + byRegOfs, byData | (BYTE)(byBits));   \
}


#define MACvRegBitsOff(dwIoBase, byRegOfs, byBits)          \
{                                                           \
    BYTE byData;                                            \
    VNSvInPortB(dwIoBase + byRegOfs, &byData);              \
    VNSvOutPortB(dwIoBase + byRegOfs, byData & (BYTE)~(byBits));  \
}


// set the chip with current rx descriptor address
#define MACvSetCurrRxDescAddr(dwIoBase, dwCurrDescAddr)     \
{                                                           \
    VNSvOutPortD(dwIoBase + MAC_REG_CUR_RD_ADDR,            \
        dwCurrDescAddr);                                    \
}

#define MACvGetCurrRxDescAddr(dwIoBase, pdwCurrDescAddr)    \
{                                                           \
    VNSvInPortD(dwIoBase + MAC_REG_CUR_RD_ADDR,             \
        (PDWORD)pdwCurrDescAddr);                           \
}


// set the chip with current tx descriptor address
#define MACvSetCurrTxDescAddr(dwIoBase, dwCurrDescAddr)     \
{                                                           \
    VNSvOutPortD(dwIoBase + MAC_REG_CUR_TD_ADDR,            \
        dwCurrDescAddr);                                    \
}

// set the chip with current tx descriptor address for 3106

#define MACvGetCurrTxDescAddr(dwIoBase, pdwCurrDescAddr)    \
{                                                           \
    VNSvInPortD(dwIoBase + MAC_REG_CUR_TD_ADDR,             \
        (PDWORD)pdwCurrDescAddr);                           \
}

#define MACvReadTallyCounter(dwIoBase, pwRegMPA,            \
                pwRegCRC)                                   \
{                                                           \
    VNSvInPortW(dwIoBase + MAC_REG_CNTR_MPA, pwRegMPA);     \
    VNSvInPortW(dwIoBase + MAC_REG_CNTR_CRC, pwRegCRC);     \
}


#define MACvStart(dwIoBase)                 \
{                                           \
    VNSvOutPortB(dwIoBase + MAC_REG_CR0,    \
        CR0_TXON | CR0_RXON | CR0_STRT);    \
}


#define MACvRxOn(dwIoBase)                              \
{                                                       \
    BYTE byOrgValue;                                    \
    VNSvInPortB(dwIoBase + MAC_REG_CR0, &byOrgValue);   \
    byOrgValue = (BYTE)((byOrgValue & ~CR0_STOP) | CR0_RXON | CR0_STRT);  \
    VNSvOutPortB(dwIoBase + MAC_REG_CR0, byOrgValue);   \
}


#define MACvTxOn(dwIoBase)                              \
{                                                       \
    BYTE byOrgValue;                                    \
    VNSvInPortB(dwIoBase + MAC_REG_CR0, &byOrgValue);   \
    byOrgValue = (BYTE)((byOrgValue & ~CR0_STOP) | CR0_TXON | CR0_STRT); \
    VNSvOutPortB(dwIoBase + MAC_REG_CR0, byOrgValue);   \
}


#define MACvTransmit(dwIoBase)                          \
{                                                       \
    BYTE byOrgValue;                                    \
    VNSvInPortB(dwIoBase + MAC_REG_CR1, &byOrgValue);   \
    byOrgValue = (BYTE)(byOrgValue | CR1_TDMD1);        \
    VNSvOutPortB(dwIoBase + MAC_REG_CR1, byOrgValue);   \
}


#define MACvClearStckDS(dwIoBase)                           \
{                                                           \
    BYTE byOrgValue;                                        \
    VNSvInPortB(dwIoBase + MAC_REG_STICKHW, &byOrgValue);   \
    byOrgValue = byOrgValue & (BYTE)0xFC;                   \
    VNSvOutPortB(dwIoBase + MAC_REG_STICKHW, byOrgValue);   \
}


#define MACvPwrEvntDisable(dwIoBase)                    \
{                                                       \
    VNSvOutPortB(dwIoBase + MAC_REG_WOLCR_CLR, 0xFF);   \
    VNSvOutPortB(dwIoBase + MAC_REG_WOLCR1_CLR, 0x03);  \
}

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/
#ifdef __cplusplus
extern "C" {                            /* Assume C declarations for C++ */
#endif /* __cplusplus */

BOOL
MACbIsCableLinkOk(
    DWORD   dwIoBase
    );

BOOL
MACbIsIn100MMode(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

VOID
MACvIsInFullDuplexMode(
    PBOOL   pbFullDup,
    DWORD   dwIoBase
    );

BOOL
MACbIsRegBitsOff(
    DWORD   dwIoBase,
    BYTE    byRegOfs,
    BYTE    byTestBits
    );

BOOL
MACbIsRegBitsOn(
    DWORD   dwIoBase,
    BYTE    byRegOfs,
    BYTE    byTestBits
    );

BOOL
MACbSafeSoftwareReset(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

BOOL
MACbStop(
    DWORD   dwIoBase
    );

BOOL
MACbSafeStop(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

BOOL
MACbSafeRxOff(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

BOOL
MACbSafeTxOff(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

BOOL
MACbSoftwareReset(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

BYTE
MACbyReadEECSR(
    DWORD   dwIoBase
    );

BYTE
MACbyReadMIICR(
    DWORD   dwIoBase
    );

BYTE
MACbyReadMultiAddr(
    DWORD   dwIoBase,
    UINT    uByteIdx
    );

VOID
MACvWriteMultiAddr(
    DWORD   dwIoBase,
    UINT    uByteIdx,
    BYTE    byData
    );

VOID
MACvEnableMiiAutoPoll(
    DWORD   dwIoBase
    );

VOID
MACvInitialize(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

VOID
MACvIntDisable(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

VOID
MACvIntEnable(
    DWORD   dwIoBase,
    BYTE    byRevId,
    DWORD   dwMask
    );

VOID
MACvSafeDisableMiiAutoPoll(
    DWORD   dwIoBase,
    BYTE    byRevId
    );

VOID
MACvRestoreContext(
    DWORD   dwIoBase,
    BYTE    byRevId,
    PBYTE   pbyCxtBuf
    );

VOID
MACvReadISR(
    DWORD   dwIoBase,
    BYTE    byRevId,
    PDWORD  pdwValue
    );

VOID
MACvWriteISR(
    DWORD   dwIoBase,
    BYTE    byRevId,
    DWORD   dwValue
    );

VOID
MACvSaveContext(
    DWORD   dwIoBase,
    BYTE    byRevId,
    PBYTE   pbyCxtBuf
    );

VOID
MACvSetDmaLength(
    DWORD   dwIoBase,
    BYTE    byDmaLength
    );

VOID
MACvSetDuplexMode(
    DWORD   dwIoBase,
    BYTE    byRevId,
    BOOL    bFullDuplexOn
    );

VOID
MACvSetLoopbackMode(
    DWORD   dwIoBase,
    BYTE    byLoopbackMode
    );

VOID
MACvSetPacketFilter(
    DWORD   dwIoBase,
    WORD    wFilterType
    );

VOID
MACvSetRxThreshold(
    DWORD   dwIoBase,
    BYTE    byThreshold
    );

VOID
MACvGetTxThreshold(
    DWORD   dwIoBase,
    PBYTE   pbyThreshold
    );

VOID
MACvSetTxThreshold(
    DWORD   dwIoBase,
    BYTE    byThreshold
    );

VOID
MACvTimer0MicroSDelay(
    DWORD   dwIoBase,
    BYTE    byRevId,
    UINT    udelay
    );

VOID
MACvTimer0MiniSDelay(
    DWORD   dwIoBase,
    BYTE    byRevId,
    UINT    udelay
    );

VOID
MACvWriteEECSR(
    DWORD   dwIoBase,
    BYTE    byValue,
    BYTE    byRevId
    );

BYTE
MACbyGetPhyId(
    DWORD   dwIoBase
    );

#ifdef __cplusplus
}                                       /* End of extern "C" { */
#endif /* __cplusplus */

#endif // __MAC_H__

