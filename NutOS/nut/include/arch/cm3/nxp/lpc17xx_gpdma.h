#ifndef _LPC17XX_GPDMA_H_
#define _LPC17XX_GPDMA_H_

/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 *
 * Parts taken from lpc177x_8x_gpdma.h          2011-06-02
 *
 * file     lpc177x_8x_gpdma.h
 * brief    Contains all macro definitions and function prototypes
 *          support for GPDMA firmware library on LPC177x_8x
 * version  1.0
 * date     02. June. 2011
 * author   NXP MCU SW Application Team
 *
 * Copyright(C) 2011, NXP Semiconductor
 * All rights reserved.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors'
 * relevant copyright in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 **********************************************************************/

/*!
 * \verbatim
 * $Id: $
 * \endverbatim
 */

/*============================================================================*
  Public definitions
 *============================================================================*/

#define GPDMA_NUM_CHANNELS               8  /* Number of DMA channels */

/*----------------------------------------------------------------------------*
  DMA Connection number definitions
 *----------------------------------------------------------------------------*/

#if defined(MCU_LPC176x)
#  define GPDMA_CONN_SSP0_Tx               0  /* SSP0 Tx */
#  define GPDMA_CONN_SSP0_Rx               1  /* SSP0 Rx */
#  define GPDMA_CONN_SSP1_Tx               2  /* SSP1 Tx */
#  define GPDMA_CONN_SSP1_Rx               3  /* SSP1 Rx */
#  define GPDMA_CONN_ADC                   4  /* ADC */
#  define GPDMA_CONN_I2S_Channel_0         5  /* I2S channel 0 */
#  define GPDMA_CONN_I2S_Channel_1         6  /* I2S channel 1 */
#  define GPDMA_CONN_DAC                   7  /* DAC */
#  define GPDMA_CONN_UART0_Tx              8  /* UART0 Tx */
#  define GPDMA_CONN_UART0_Rx              9  /* UART0 Rx */
#  define GPDMA_CONN_UART1_Tx             10  /* UART1 Tx */
#  define GPDMA_CONN_UART1_Rx             11  /* UART1 Rx */
#  define GPDMA_CONN_UART2_Tx             12  /* UART2 Tx */
#  define GPDMA_CONN_UART2_Rx             13  /* UART2 Rx */
#  define GPDMA_CONN_UART3_Tx             14  /* UART3 Tx */
#  define GPDMA_CONN_UART3_Rx             15  /* UART3 Rx */
#  define GPDMA_CONN_MAT0_0               16  /* MAT0.0 */
#  define GPDMA_CONN_MAT0_1               17  /* MAT0.1 */
#  define GPDMA_CONN_MAT1_0               18  /* MAT1.0 */
#  define GPDMA_CONN_MAT1_1               19  /* MAT1.1 */
#  define GPDMA_CONN_MAT2_0               20  /* MAT2.0 */
#  define GPDMA_CONN_MAT2_1               21  /* MAT2.1 */
#  define GPDMA_CONN_MAT3_0               22  /* MAT3.0 */
#  define GPDMA_CONN_MAT3_1               23  /* MAT3.1 */
#elif defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
                                       /*  0  reserved */
#  define GPDMA_CONN_MCI                   1  /* SD card */
#  define GPDMA_CONN_SSP0_Tx               2  /* SSP0 Tx */
#  define GPDMA_CONN_SSP0_Rx               3  /* SSP0 Rx */
#  define GPDMA_CONN_SSP1_Tx               4  /* SSP1 Tx */
#  define GPDMA_CONN_SSP1_Rx               5  /* SSP1 Rx */
#  define GPDMA_CONN_SSP2_Tx               6  /* SSP2 Tx */
#  define GPDMA_CONN_SSP2_Rx               7  /* SSP2 Rx */
#  define GPDMA_CONN_ADC                   8  /* ADC */
#  define GPDMA_CONN_DAC                   9  /* DAC */
#  define GPDMA_CONN_UART0_Tx             10  /* UART0 Tx */
#  define GPDMA_CONN_UART0_Rx             11  /* UART0 Rx */
#  define GPDMA_CONN_UART1_Tx             12  /* UART1 Tx */
#  define GPDMA_CONN_UART1_Rx             13  /* UART1 Rx */
#  define GPDMA_CONN_UART2_Tx             14  /* UART2 Tx */
#  define GPDMA_CONN_UART2_Rx             15  /* UART2 Rx */
#  define GPDMA_CONN_MAT0_0               16  /* MAT0.0 */
#  define GPDMA_CONN_MAT0_1               17  /* MAT0.1 */
#  define GPDMA_CONN_MAT1_0               18  /* MAT1.0 */
#  define GPDMA_CONN_MAT1_1               19  /* MAT1.1 */
#  define GPDMA_CONN_MAT2_0               20  /* MAT2.0 */
#  define GPDMA_CONN_MAT2_1               21  /* MAT2.1 */
#  define GPDMA_CONN_I2S_Channel_0        22  /* I2S channel 0 */
#  define GPDMA_CONN_I2S_Channel_1        23  /* I2S channel 1 */
                                       /* 24  reserved */
                                       /* 25  reserved */
#  define GPDMA_CONN_UART3_Tx             26  /* UART3 Tx */
#  define GPDMA_CONN_UART3_Rx             27  /* UART3 Rx */
#  define GPDMA_CONN_UART4_Tx             28  /* UART3 Tx */
#  define GPDMA_CONN_UART4_Rx             29  /* UART3 Rx */
#  define GPDMA_CONN_MAT3_0               30  /* MAT3.0 */
#  define GPDMA_CONN_MAT3_1               31  /* MAT3.1 */
#endif


/*----------------------------------------------------------------------------*
  GPDMA Transfer type definitions
 *----------------------------------------------------------------------------*/

#define GPDMA_TRANSFERTYPE_M2M           0  /* Memory to memory - DMA control */
#define GPDMA_TRANSFERTYPE_M2P           1  /* Memory to peripheral - DMA control */
#define GPDMA_TRANSFERTYPE_P2M           2  /* Peripheral to memory - DMA control */
#define GPDMA_TRANSFERTYPE_P2P           3  /* Source peripheral to destination peripheral - DMA control */
#define GPDMA_TRANSFERTYPE_M2P_DEST_CTRL 5  /* Memory to peripheral - Destination peripheral control */
#define GPDMA_TRANSFERTYPE_P2M_SRC_CTRL  6  /* Peripheral to memory - Source peripheral control */


/*----------------------------------------------------------------------------*
  Burst size in Source and Destination definitions
 *----------------------------------------------------------------------------*/

#define GPDMA_BSIZE_1                    0  /* Burst size = 1 */
#define GPDMA_BSIZE_4                    1  /* Burst size = 4 */
#define GPDMA_BSIZE_8                    2  /* Burst size = 8 */
#define GPDMA_BSIZE_16                   3  /* Burst size = 16 */
#define GPDMA_BSIZE_32                   4  /* Burst size = 32 */
#define GPDMA_BSIZE_64                   5  /* Burst size = 64 */
#define GPDMA_BSIZE_128                  6  /* Burst size = 128 */
#define GPDMA_BSIZE_256                  7  /* Burst size = 256 */


/*----------------------------------------------------------------------------*
  Width in Source transfer width and Destination transfer width definitions
 *----------------------------------------------------------------------------*/

#define GPDMA_WIDTH_BYTE                 0  /* Width = 1 byte */
#define GPDMA_WIDTH_HALFWORD             1  /* Width = 2 bytes */
#define GPDMA_WIDTH_WORD                 2  /* Width = 4 bytes */


/*----------------------------------------------------------------------------*
  DMA Request Select Mode definitions
 *----------------------------------------------------------------------------*/

#define GPDMA_REQSEL_UART                0  /* UART TX/RX is selected */
#define GPDMA_REQSEL_TIMER               1  /* Timer match is selected */




/*============================================================================*
  Bit definitions
 *============================================================================*/

/*----------------------------------------------------------------------------*
  Macro defines for DMA Interrupt Status register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACIntStat_Ch(n)         ((1UL << n) & 0xFF)
#define GPDMA_DMACIntStat_BITMASK       0xFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Interrupt Terminal Count Request Status register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACIntTCStat_Ch(n)       ((1UL << n) & 0xFF)
#define GPDMA_DMACIntTCStat_BITMASK     0xFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Interrupt Terminal Count Request Clear register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACIntTCClear_Ch(n)      ((1UL << n) & 0xFF)
#define GPDMA_DMACIntTCClear_BITMASK    0xFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Interrupt Error Status register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACIntErrStat_Ch(n)      ((1UL << n) & 0xFF)
#define GPDMA_DMACIntErrStat_BITMASK    0xFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Interrupt Error Clear register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACIntErrClr_Ch(n)       ((1UL << n) & 0xFF)
#define GPDMA_DMACIntErrClr_BITMASK     0xFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Raw Interrupt Terminal Count Status register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACRawIntTCStat_Ch(n)    ((1UL << n) & 0xFF)
#define GPDMA_DMACRawIntTCStat_BITMASK  0xFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Raw Error Interrupt Status register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACRawIntErrStat_Ch(n)   ((1UL << n) & 0xFF)
#define GPDMA_DMACRawIntErrStat_BITMASK 0xFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Enabled Channel register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACEnbldChns_Ch(n)       ((1UL << n) & 0xFF)
#define GPDMA_DMACEnbldChns_BITMASK     0xFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Software Burst Request register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACSoftBReq_Src(n)       ((1UL << n) & 0xFFFF)
#define GPDMA_DMACSoftBReq_BITMASK      0xFFFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Software Single Request register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACSoftSReq_Src(n)       ((1UL << n) & 0xFFFF)
#define GPDMA_DMACSoftSReq_BITMASK      0xFFFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Software Last Burst Request register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACSoftLBReq_Src(n)      ((1UL << n) & 0xFFFF)
#define GPDMA_DMACSoftLBReq_BITMASK     0xFFFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Software Last Single Request register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACSoftLSReq_Src(n)      ((1UL << n) & 0xFFFF)
#define GPDMA_DMACSoftLSReq_BITMASK     0xFFFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Configuration register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACConfig_E              0x01    /* DMA Controller enable*/
#define GPDMA_DMACConfig_M              0x02    /* AHB Master endianness configuration*/
#define GPDMA_DMACConfig_BITMASK        0x03


/*----------------------------------------------------------------------------*
  Macro defines for DMA Synchronization register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACSync_Src(n)           ((1UL << n) & 0xFFFF)
#define GPDMA_DMACSync_BITMASK          0xFFFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Request Select register
 *----------------------------------------------------------------------------*/

#define GPDMA_DMAReqSel_Input(n)        ((1UL << (n - 8)) & 0xFF)
#define GPDMA_DMAReqSel_BITMASK         0xFF


/*----------------------------------------------------------------------------*
  Macro defines for DMA Channel Linker List Item registers
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACCxLLI_BITMASK         0xFFFFFFFC  /* DMA Channel Linker List Item registers bit mask */


/*----------------------------------------------------------------------------*
  Macro defines for DMA channel control registers
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACCxControl_TransferSize(n) ((n & 0xFFF) << 0)  /* Transfer size*/
#define GPDMA_DMACCxControl_SBSize(n)       ((n & 0x07) << 12)  /* Source burst size */
#define GPDMA_DMACCxControl_DBSize(n)       ((n & 0x07) << 15)  /* Destination burst size */
#define GPDMA_DMACCxControl_SWidth(n)       ((n & 0x07) << 18)  /* Source transfer width */
#define GPDMA_DMACCxControl_DWidth(n)       ((n & 0x07) << 21)  /* Destination transfer width */
#define GPDMA_DMACCxControl_SI              (1UL << 26)         /* Source increment */
#define GPDMA_DMACCxControl_DI              (1UL << 27)         /* Destination increment */
#define GPDMA_DMACCxControl_Prot1           (1UL << 28)         /* Indicates that the access is in user mode or privileged mode */
#define GPDMA_DMACCxControl_Prot2           (1UL << 29)         /* Indicates that the access is bufferable or not bufferable */
#define GPDMA_DMACCxControl_Prot3           (1UL << 30)         /* Indicates that the access is cacheable or not cacheable */
#define GPDMA_DMACCxControl_I               (1UL << 31)         /* Terminal count interrupt enable bit */
#define GPDMA_DMACCxControl_BITMASK         0xFCFFFFFF          /* DMA channel control registers bit mask */


/*----------------------------------------------------------------------------*
  Macro defines for DMA Channel Configuration registers
 *----------------------------------------------------------------------------*/

#define GPDMA_DMACCxConfig_E                    (1UL << 0)          /* DMA control enable */
#define GPDMA_DMACCxConfig_SrcPeripheral(n)     ((n & 0x1F) << 1)   /* Source peripheral */
#define GPDMA_DMACCxConfig_DestPeripheral(n)    ((n & 0x1F) << 6)   /* Destination peripheral */
#define GPDMA_DMACCxConfig_TransferType(n)      ((n & 0x7) << 11)   /* This value indicates the type of transfer */
#define GPDMA_DMACCxConfig_IE                   (1UL << 14)         /* Interrupt error mask */
#define GPDMA_DMACCxConfig_ITC                  (1UL << 15)         /* Terminal count interrupt mask */
#define GPDMA_DMACCxConfig_L                    (1UL << 16)         /* Lock */
#define GPDMA_DMACCxConfig_A                    (1UL << 17)         /* Active */
#define GPDMA_DMACCxConfig_H                    (1UL << 18)         /* Halt */
#define GPDMA_DMACCxConfig_BITMASK              0x7FFFF             /* DMA Channel Configuration registers bit mask */


/*============================================================================*
  Type definitions
 *============================================================================*/

/*----------------------------------------------------------------------------*
  GPDMA Status enumeration
 *----------------------------------------------------------------------------*/

typedef enum {
    GPDMA_STAT_INT             = 0x01, /* GPDMA Interrupt Status */
    GPDMA_STAT_INTTC           = 0x02, /* GPDMA Interrupt Terminal Count Request Status */
    GPDMA_STAT_INTERR          = 0x04, /* GPDMA Interrupt Error Status */
    GPDMA_STAT_RAWINTTC        = 0x08, /* GPDMA Raw Interrupt Terminal Count Status */
    GPDMA_STAT_RAWINTERR       = 0x10, /* GPDMA Raw Error Interrupt Status */
    GPDMA_STAT_ENABLED_CH      = 0x20  /* GPDMA Enabled Channel Status */
} gpdma_status_t;


/*----------------------------------------------------------------------------*
  GPDMA Interrupt clear status enumeration
 *----------------------------------------------------------------------------*/

typedef enum{
    GPDMA_STATCLR_INTTC,    /* GPDMA Interrupt Terminal Count Request Clear */
    GPDMA_STATCLR_INTERR    /* GPDMA Interrupt Error Clear */
} gpdma_state_clear_t;


/*----------------------------------------------------------------------------*
  GPDMA Channel info struct
 *----------------------------------------------------------------------------*/

typedef struct {
    void (*handler) (int ch, uint32_t status, void *);
    void  *arg;
} gpdma_vector_t;

/*----------------------------------------------------------------------------*
  GPDMA Channel configuration structure type definition
 *----------------------------------------------------------------------------*/

typedef struct {
    uint32_t ch;            /* DMA channel number, should be in
                               range from 0 to 7.
                               Note: DMA channel 0 has the highest priority
                               and DMA channel 7 the lowest priority.
                             */

    uint32_t transfer_size; /* Length/Size of transfer */

    uint32_t transfer_width;/* Transfer width - used for TransferType is GPDMA_TRANSFERTYPE_M2M only */

    uint32_t src_addr;      /* Physical Source Address, used in case TransferType is chosen as
                               GPDMA_TRANSFERTYPE_M2M or GPDMA_TRANSFERTYPE_M2P */

    uint32_t dst_addr;      /* Physical Destination Address, used in case TransferType is chosen as
                               GPDMA_TRANSFERTYPE_M2M or GPDMA_TRANSFERTYPE_P2M */

    uint32_t transfer_type; /* Transfer Type, should be one of the following:
                               - GPDMA_TRANSFERTYPE_M2M: Memory to memory - DMA control
                               - GPDMA_TRANSFERTYPE_M2P: Memory to peripheral - DMA control
                               - GPDMA_TRANSFERTYPE_P2M: Peripheral to memory - DMA control
                               - GPDMA_TRANSFERTYPE_P2P: Source peripheral to destination peripheral - DMA control
                             */

    uint32_t src_conn;      /* Peripheral Source Connection type, used in case TransferType is chosen as
                               GPDMA_TRANSFERTYPE_P2M or GPDMA_TRANSFERTYPE_P2P, should be one of
                               following:
                               - GPDMA_CONN_SSP0_Tx: SSP0, Tx
                               - GPDMA_CONN_SSP0_Rx: SSP0, Rx
                               - GPDMA_CONN_SSP1_Tx: SSP1, Tx
                               - GPDMA_CONN_SSP1_Rx: SSP1, Rx
                               - GPDMA_CONN_ADC: ADC
                               - GPDMA_CONN_I2S_Channel_0: I2S Channel 0
                               - GPDMA_CONN_I2S_Channel_1: I2S Channel 1
                               - GPDMA_CONN_DAC: DAC
                               - GPDMA_CONN_UART0_Tx_MAT0_0: UART0 Tx / MAT0.0
                               - GPDMA_CONN_UART0_Rx_MAT0_1: UART0 Rx / MAT0.1
                               - GPDMA_CONN_UART1_Tx_MAT1_0: UART1 Tx / MAT1.0
                               - GPDMA_CONN_UART1_Rx_MAT1_1: UART1 Rx / MAT1.1
                               - GPDMA_CONN_UART2_Tx_MAT2_0: UART2 Tx / MAT2.0
                               - GPDMA_CONN_UART2_Rx_MAT2_1: UART2 Rx / MAT2.1
                               - GPDMA_CONN_UART3_Tx_MAT3_0: UART3 Tx / MAT3.0
                               - GPDMA_CONN_UART3_Rx_MAT3_1: UART3 Rx / MAT3.1
                             */

    uint32_t dst_conn;      /* Peripheral Destination Connection type, used in case TransferType is chosen as
                               GPDMA_TRANSFERTYPE_M2P or GPDMA_TRANSFERTYPE_P2P, should be one of
                               following:
                               - GPDMA_CONN_SSP0_Tx: SSP0, Tx
                               - GPDMA_CONN_SSP0_Rx: SSP0, Rx
                               - GPDMA_CONN_SSP1_Tx: SSP1, Tx
                               - GPDMA_CONN_SSP1_Rx: SSP1, Rx
                               - GPDMA_CONN_ADC: ADC
                               - GPDMA_CONN_I2S_Channel_0: I2S Channel 0
                               - GPDMA_CONN_I2S_Channel_1: I2S Channel 1
                               - GPDMA_CONN_DAC: DAC
                               - GPDMA_CONN_UART0_Tx_MAT0_0: UART0 Tx / MAT0.0
                               - GPDMA_CONN_UART0_Rx_MAT0_1: UART0 Rx / MAT0.1
                               - GPDMA_CONN_UART1_Tx_MAT1_0: UART1 Tx / MAT1.0
                               - GPDMA_CONN_UART1_Rx_MAT1_1: UART1 Rx / MAT1.1
                               - GPDMA_CONN_UART2_Tx_MAT2_0: UART2 Tx / MAT2.0
                               - GPDMA_CONN_UART2_Rx_MAT2_1: UART2 Rx / MAT2.1
                               - GPDMA_CONN_UART3_Tx_MAT3_0: UART3 Tx / MAT3.0
                               - GPDMA_CONN_UART3_Rx_MAT3_1: UART3 Rx / MAT3.1
                             */

    uint32_t dma_lli;       /* Linker list item structure data address
                               if there's no linker list, set as '0'
                             */

} gpdma_channel_cfg_t;


/*----------------------------------------------------------------------------*
  GPDMA Linker List Item structure type definition
 *----------------------------------------------------------------------------*/

typedef struct {
    uint32_t SrcAddr;     /* Source Address */
    uint32_t DstAddr;     /* Destination address */
    uint32_t NextLLI;     /* Next LLI address, otherwise set to '0' */
    uint32_t Control;     /* GPDMA Control of this LLI */
} gpdma_lli_t;


int  Lpc17xxGPDMA_Init(void);
int  Lpc17xxGPDMA_Setup(gpdma_channel_cfg_t *ch_config, void (*handler) (int ch, uint32_t status, void *), void* arg);
int  Lpc17xxGPDMA_IntGetStatus(gpdma_status_t type, uint8_t ch);
void Lpc17xxGPDMA_ClearIntPending(gpdma_state_clear_t type, uint8_t ch);
void Lpc17xxGPDMA_ChannelCmd(uint8_t ch, int enabled);

#endif /* _LPC17XX_GPDMA_H_ */
