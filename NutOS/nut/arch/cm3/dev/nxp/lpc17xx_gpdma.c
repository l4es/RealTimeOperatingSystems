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
 * Parts taken from lpc177x_8x_gpdma.c         2011-06-02
 *
 * file     lpc177x_8x_gpdma.c
 * brief    Contains all functions support for GPDMA firmware library
 *          on LPC177x_8x
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

#include <inttypes.h>

#include <cfg/arch.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>

#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#include <arch/cm3/nxp/lpc176x_clk.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#else
#warning "Unknown LPC familiy"
#endif

#include <arch/cm3/nxp/lpc17xx_gpdma.h>

static int initialised = 0;

/*!
 * \brief Lookup Table of Connection Type matched with
 *        Peripheral Data (FIFO) register base address
 */

volatile const void *GPDMA_LUTPerAddr[] = {
        [GPDMA_CONN_SSP0_Tx]       = (&LPC_SSP0->DR),      // SSP0 Tx
        [GPDMA_CONN_SSP0_Rx]       = (&LPC_SSP0->DR),      // SSP0 Rx
        [GPDMA_CONN_SSP1_Tx]       = (&LPC_SSP1->DR),      // SSP1 Tx
        [GPDMA_CONN_SSP1_Rx]       = (&LPC_SSP1->DR),      // SSP1 Rx
        [GPDMA_CONN_UART0_Tx]      = (&LPC_UART0->THR),    // UART0 Tx
        [GPDMA_CONN_UART0_Rx]      = (&LPC_UART0->RBR),    // UART0 Rx
        [GPDMA_CONN_UART1_Tx]      = (&LPC_UART1->THR),    // UART1 Tx
        [GPDMA_CONN_UART1_Rx]      = (&LPC_UART1->RBR),    // UART1 Rx
        [GPDMA_CONN_UART2_Tx]      = (&LPC_UART2->THR),    // UART2 Tx
        [GPDMA_CONN_UART2_Rx]      = (&LPC_UART2->RBR),    // UART2 Rx
        [GPDMA_CONN_UART3_Tx]      = (&LPC_UART3->THR),    // UART3 Tx
        [GPDMA_CONN_UART3_Rx]      = (&LPC_UART3->RBR),    // UART3 Rx
        [GPDMA_CONN_ADC]           = (&LPC_ADC->GDR),      // ADC
        [GPDMA_CONN_DAC]           = (&LPC_DAC->CR),       // DAC
        [GPDMA_CONN_I2S_Channel_0] = (&LPC_I2S->TXFIFO),   // I2S Channel 0
        [GPDMA_CONN_I2S_Channel_1] = (&LPC_I2S->RXFIFO),   // I2S Channel 1
        [GPDMA_CONN_MAT0_0]        = (&LPC_TIM0->MR0),     // MAT0.0
        [GPDMA_CONN_MAT0_1]        = (&LPC_TIM0->MR1),     // MAT0.1
        [GPDMA_CONN_MAT1_0]        = (&LPC_TIM1->MR0),     // MAT1.0
        [GPDMA_CONN_MAT1_1]        = (&LPC_TIM1->MR1),     // MAT1.1
        [GPDMA_CONN_MAT2_0]        = (&LPC_TIM2->MR0),     // MAT2.0
        [GPDMA_CONN_MAT2_1]        = (&LPC_TIM2->MR1),     // MAT2.1
        [GPDMA_CONN_MAT3_0]        = (&LPC_TIM3->MR0),     // MAT3.0
        [GPDMA_CONN_MAT3_1]        = (&LPC_TIM3->MR1),     // MAT3.1

#if defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
        [GPDMA_CONN_MCI]           = (&LPC_MCI->FIFO),     // SD Card
        [GPDMA_CONN_SSP2_Tx]       = (&LPC_SSP2->DR),      // SSP2 Tx
        [GPDMA_CONN_SSP2_Rx]       = (&LPC_SSP2->DR),      // SSP2 Rx
        [GPDMA_CONN_UART4_Tx]      = (&LPC_UART4->THR),    // UART4 Tx
        [GPDMA_CONN_UART4_Rx]      = (&LPC_UART4->RBR),    // UART4 Rx
#endif
};

/*!
 * \brief Lookup Table of GPDMA Channel Number matched with
 *        GPDMA channel pointer
 */
const LPC_GPDMACH_TypeDef *pGPDMACh[GPDMA_NUM_CHANNELS] = {
        LPC_GPDMACH0,                   // GPDMA Channel 0
        LPC_GPDMACH1,                   // GPDMA Channel 1
        LPC_GPDMACH2,                   // GPDMA Channel 2
        LPC_GPDMACH3,                   // GPDMA Channel 3
        LPC_GPDMACH4,                   // GPDMA Channel 4
        LPC_GPDMACH5,                   // GPDMA Channel 5
        LPC_GPDMACH6,                   // GPDMA Channel 6
        LPC_GPDMACH7,                   // GPDMA Channel 7
};

/*!
 * \brief Optimized Peripheral Source and Destination burst size
 */
const uint8_t GPDMA_LUTPerBurst[] = {
        [GPDMA_CONN_SSP0_Tx]       = GPDMA_BSIZE_4,    // SSP0 Tx
        [GPDMA_CONN_SSP0_Rx]       = GPDMA_BSIZE_4,    // SSP0 Rx
        [GPDMA_CONN_SSP1_Tx]       = GPDMA_BSIZE_4,    // SSP1 Tx
        [GPDMA_CONN_SSP1_Rx]       = GPDMA_BSIZE_4,    // SSP1 Rx
        [GPDMA_CONN_UART0_Tx]      = GPDMA_BSIZE_1,    // UART0 Tx
        [GPDMA_CONN_UART0_Rx]      = GPDMA_BSIZE_1,    // UART0 Rx
        [GPDMA_CONN_UART1_Tx]      = GPDMA_BSIZE_1,    // UART1 Tx
        [GPDMA_CONN_UART1_Rx]      = GPDMA_BSIZE_1,    // UART1 Rx
        [GPDMA_CONN_UART2_Tx]      = GPDMA_BSIZE_1,    // UART2 Tx
        [GPDMA_CONN_UART2_Rx]      = GPDMA_BSIZE_1,    // UART2 Rx
        [GPDMA_CONN_UART3_Tx]      = GPDMA_BSIZE_1,    // UART3 Tx
        [GPDMA_CONN_UART3_Rx]      = GPDMA_BSIZE_1,    // UART3 Rx
        [GPDMA_CONN_ADC]           = GPDMA_BSIZE_1,    // ADC
        [GPDMA_CONN_DAC]           = GPDMA_BSIZE_1,    // DAC
        [GPDMA_CONN_I2S_Channel_0] = GPDMA_BSIZE_32,   // I2S channel 0
        [GPDMA_CONN_I2S_Channel_1] = GPDMA_BSIZE_32,   // I2S channel 1
        [GPDMA_CONN_MAT0_0]        = GPDMA_BSIZE_1,    // MAT0.0
        [GPDMA_CONN_MAT0_1]        = GPDMA_BSIZE_1,    // MAT0.1
        [GPDMA_CONN_MAT1_0]        = GPDMA_BSIZE_1,    // MAT1.0
        [GPDMA_CONN_MAT1_1]        = GPDMA_BSIZE_1,    // MAT1.1
        [GPDMA_CONN_MAT2_0]        = GPDMA_BSIZE_1,    // MAT2.0
        [GPDMA_CONN_MAT2_1]        = GPDMA_BSIZE_1,    // MAT2.1
        [GPDMA_CONN_MAT3_0]        = GPDMA_BSIZE_1,    // MAT3.0
        [GPDMA_CONN_MAT3_1]        = GPDMA_BSIZE_1,    // MAT3.1

#if defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
        [GPDMA_CONN_MCI]           = GPDMA_BSIZE_8,    // SD Card
        [GPDMA_CONN_SSP2_Tx]       = GPDMA_BSIZE_4,    // SSP2 Tx
        [GPDMA_CONN_SSP2_Rx]       = GPDMA_BSIZE_4,    // SSP2 Rx
        [GPDMA_CONN_UART4_Tx]      = GPDMA_BSIZE_1,    // UART4 Tx
        [GPDMA_CONN_UART4_Rx]      = GPDMA_BSIZE_1,    // UART4 Rx
#endif
};


/*!
 * \brief Optimized Peripheral Source and Destination transfer width
 */
const uint8_t GPDMA_LUTPerWid[] = {
        [GPDMA_CONN_SSP0_Tx]       = GPDMA_WIDTH_BYTE,   // SSP0 Tx
        [GPDMA_CONN_SSP0_Rx]       = GPDMA_WIDTH_BYTE,   // SSP0 Rx
        [GPDMA_CONN_SSP1_Tx]       = GPDMA_WIDTH_BYTE,   // SSP1 Tx
        [GPDMA_CONN_SSP1_Rx]       = GPDMA_WIDTH_BYTE,   // SSP1 Rx
        [GPDMA_CONN_UART0_Tx]      = GPDMA_WIDTH_BYTE,   // UART0 Tx
        [GPDMA_CONN_UART0_Rx]      = GPDMA_WIDTH_BYTE,   // UART0 Rx
        [GPDMA_CONN_UART1_Tx]      = GPDMA_WIDTH_BYTE,   // UART1 Tx
        [GPDMA_CONN_UART1_Rx]      = GPDMA_WIDTH_BYTE,   // UART1 Rx
        [GPDMA_CONN_UART2_Tx]      = GPDMA_WIDTH_BYTE,   // UART2 Tx
        [GPDMA_CONN_UART2_Rx]      = GPDMA_WIDTH_BYTE,   // UART2 Rx
        [GPDMA_CONN_UART3_Tx]      = GPDMA_WIDTH_BYTE,   // UART3 Tx
        [GPDMA_CONN_UART3_Rx]      = GPDMA_WIDTH_BYTE,   // UART3 Rx
        [GPDMA_CONN_ADC]           = GPDMA_WIDTH_WORD,   // ADC
        [GPDMA_CONN_DAC]           = GPDMA_WIDTH_BYTE,   // DAC
        [GPDMA_CONN_I2S_Channel_0] = GPDMA_WIDTH_WORD,   // I2S channel 0
        [GPDMA_CONN_I2S_Channel_1] = GPDMA_WIDTH_WORD,   // I2S channel 1
        [GPDMA_CONN_MAT0_0]        = GPDMA_WIDTH_WORD,   // MAT0.0
        [GPDMA_CONN_MAT0_1]        = GPDMA_WIDTH_WORD,   // MAT0.1
        [GPDMA_CONN_MAT1_0]        = GPDMA_WIDTH_WORD,   // MAT1.0
        [GPDMA_CONN_MAT1_1]        = GPDMA_WIDTH_WORD,   // MAT1.1
        [GPDMA_CONN_MAT2_0]        = GPDMA_WIDTH_WORD,   // MAT2.0
        [GPDMA_CONN_MAT2_1]        = GPDMA_WIDTH_WORD,   // MAT2.1
        [GPDMA_CONN_MAT3_0]        = GPDMA_WIDTH_WORD,   // MAT3.0
        [GPDMA_CONN_MAT3_1]        = GPDMA_WIDTH_WORD,   // MAT3.1

#if defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
        [GPDMA_CONN_MCI]           = GPDMA_WIDTH_WORD,   // SD Card
        [GPDMA_CONN_SSP2_Tx]       = GPDMA_WIDTH_BYTE,   // SSP2 Tx
        [GPDMA_CONN_SSP2_Rx]       = GPDMA_WIDTH_BYTE,   // SSP2 Rx
        [GPDMA_CONN_UART4_Tx]      = GPDMA_WIDTH_BYTE,   // UART4 Tx
        [GPDMA_CONN_UART4_Rx]      = GPDMA_WIDTH_BYTE,   // UART4 Rx
#endif
};


/* Channel info struct */
static gpdma_vector_t GPDMA_Vector[GPDMA_NUM_CHANNELS];


/*!
 * \brief Generic interrupt handler
 *
 * Generic interrupt handler will call any application specific handler
 * if one is registered. Application handlers are registered using Lpc17xxGPDMA_Setup
 *
 */

void Lpc17xxGPDMA_DMA_IRQHandler(void *arg)
{
    int ch;
    uint32_t int_stat = LPC_GPDMA->IntStat;
    uint32_t int_tc_stat = LPC_GPDMA->IntTCStat;
    uint32_t int_err_stat = LPC_GPDMA->IntErrStat;
    uint32_t status;

    ch = 0;
    while (int_stat) {
        status = 0;
        if (int_stat & 0x01) {
            if (int_tc_stat & 0x01) {
                status |= GPDMA_STAT_INTTC;
                LPC_GPDMA->IntTCClear = 1 << ch;
            }
            if (int_err_stat & 0x01) {
                status |= GPDMA_STAT_INTERR;
                LPC_GPDMA->IntErrClr = 1 << ch;
            }

            if (GPDMA_Vector[ch].handler != NULL) {
                GPDMA_Vector[ch].handler(ch, status, GPDMA_Vector[ch].arg);
            }

            int_stat >>= 1;
            int_tc_stat >>= 1;
            int_err_stat >>= 1;
            ch++;
        }
    }
}


/*!
 * \brief Initialize GPDMA controller
 *
 * \param none
 *
 * \return 0 on success, -1 in case of an error (DMA IRQ could not be registered)
 */
int Lpc17xxGPDMA_Init(void)
{
    int ch;

    if (!initialised) {
        /* Enable GPDMA clock */
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCGPDMA);

        /* Reset all channel configuration register */
        LPC_GPDMACH0->CConfig = 0;
        LPC_GPDMACH1->CConfig = 0;
        LPC_GPDMACH2->CConfig = 0;
        LPC_GPDMACH3->CConfig = 0;
        LPC_GPDMACH4->CConfig = 0;
        LPC_GPDMACH5->CConfig = 0;
        LPC_GPDMACH6->CConfig = 0;
        LPC_GPDMACH7->CConfig = 0;

        /* Clear all DMA interrupt and error flag */
        LPC_GPDMA->IntTCClear = 0xFF;
        LPC_GPDMA->IntErrClr  = 0xFF;

        for (ch = 0; ch < GPDMA_NUM_CHANNELS; ch++) {
            GPDMA_Vector[ch].handler = NULL;
            GPDMA_Vector[ch].arg = NULL;
        }

        /* Register the generic DMA IRQ handler */
        if (NutRegisterIrqHandler(&sig_DMA, Lpc17xxGPDMA_DMA_IRQHandler, NULL) != 0) {
            return -1;
        }

        NutIrqEnable(&sig_DMA);
        initialised = 1;
    }
    return 0;
}


/*!
 * \brief Setup GPDMA channel
 *
 * Setup GPDMA channel peripheral according to the specified
 * parameters in the ch_config.
 *
 * \param  ch_config    Pointer to a gpdma_channel_cfg_t structure that
 *                      contains the configuration information for the specified
 *                      GPDMA channel peripheral.
 *
 * \return 0 on success, -1 in case of an error
 */

int Lpc17xxGPDMA_Setup(gpdma_channel_cfg_t *ch_config, void (*handler) (int ch, uint32_t status, void *), void* arg)
{
    LPC_GPDMACH_TypeDef *pDMAch;
    uint32_t tmp1, tmp2;

    if ((ch_config == NULL) || (ch_config->ch >= GPDMA_NUM_CHANNELS)) {
        return -1;
    }

    if (LPC_GPDMA->EnbldChns & (GPDMA_DMACEnbldChns_Ch(ch_config->ch))) {
        /* Channel was enabled before. Need to release the channel first. Return an error. */
        return -1;
    }

    /* Get channel pointer */
    pDMAch = (LPC_GPDMACH_TypeDef *) pGPDMACh[ch_config->ch];

    /* Reset the interrupt status */
    LPC_GPDMA->IntTCClear = GPDMA_DMACIntTCClear_Ch(ch_config->ch);
    LPC_GPDMA->IntErrClr  = GPDMA_DMACIntErrClr_Ch(ch_config->ch);

    /* Clear DMA config */
    pDMAch->CControl = 0x00;
    pDMAch->CConfig  = 0x00;

    /* Assign Linker List Item value */
    pDMAch->CLLI = ch_config->dma_lli;

    /* Set value to Channel Control Registers */
    switch (ch_config->transfer_type) {
        /* Memory to memory */
        case GPDMA_TRANSFERTYPE_M2M:
            /* Assign physical source and destination address */
            pDMAch->CSrcAddr = ch_config->src_addr;
            pDMAch->CDestAddr = ch_config->dst_addr;
            pDMAch->CControl  = GPDMA_DMACCxControl_TransferSize(ch_config->transfer_size) |
                                GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_32) |
                                GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_32) |
                                GPDMA_DMACCxControl_SWidth(ch_config->transfer_width) |
                                GPDMA_DMACCxControl_DWidth(ch_config->transfer_width) |
                                GPDMA_DMACCxControl_SI |
                                GPDMA_DMACCxControl_DI |
                                GPDMA_DMACCxControl_I;
            break;

        /* Memory to peripheral */
        case GPDMA_TRANSFERTYPE_M2P:
        case GPDMA_TRANSFERTYPE_M2P_DEST_CTRL:
            /* Assign physical source */
            pDMAch->CSrcAddr = ch_config->src_addr;
            /* Assign peripheral destination address */
            pDMAch->CDestAddr = (uint32_t)GPDMA_LUTPerAddr[ch_config->dst_conn];
            pDMAch->CControl  = GPDMA_DMACCxControl_TransferSize((uint32_t)ch_config->transfer_size) |
                                GPDMA_DMACCxControl_SBSize((uint32_t)GPDMA_LUTPerBurst[ch_config->dst_conn]) |
                                GPDMA_DMACCxControl_DBSize((uint32_t)GPDMA_LUTPerBurst[ch_config->dst_conn]) |
                                GPDMA_DMACCxControl_SWidth((uint32_t)GPDMA_LUTPerWid[ch_config->dst_conn]) |
                                GPDMA_DMACCxControl_DWidth((uint32_t)GPDMA_LUTPerWid[ch_config->dst_conn]) |
                                GPDMA_DMACCxControl_SI |
                                GPDMA_DMACCxControl_I;
            break;

        /* Peripheral to memory */
        case GPDMA_TRANSFERTYPE_P2M:
        case GPDMA_TRANSFERTYPE_P2M_SRC_CTRL:
            /* Assign peripheral source address */
            pDMAch->CSrcAddr = (uint32_t)GPDMA_LUTPerAddr[ch_config->src_conn];
            /* Assign memory destination address */
            pDMAch->CDestAddr = ch_config->dst_addr;
            pDMAch->CControl  = GPDMA_DMACCxControl_TransferSize((uint32_t)ch_config->transfer_size) |
                                GPDMA_DMACCxControl_SBSize((uint32_t)GPDMA_LUTPerBurst[ch_config->src_conn]) |
                                GPDMA_DMACCxControl_DBSize((uint32_t)GPDMA_LUTPerBurst[ch_config->src_conn]) |
                                GPDMA_DMACCxControl_SWidth((uint32_t)GPDMA_LUTPerWid[ch_config->src_conn]) |
                                GPDMA_DMACCxControl_DWidth((uint32_t)GPDMA_LUTPerWid[ch_config->src_conn]) |
                                GPDMA_DMACCxControl_DI |
                                GPDMA_DMACCxControl_I;
            break;

        /* Peripheral to peripheral */
        case GPDMA_TRANSFERTYPE_P2P:
            /* Assign peripheral source address */
            pDMAch->CSrcAddr = (uint32_t)GPDMA_LUTPerAddr[ch_config->src_conn];
            /* Assign peripheral destination address */
            pDMAch->CDestAddr = (uint32_t)GPDMA_LUTPerAddr[ch_config->dst_conn];
            pDMAch->CControl  = GPDMA_DMACCxControl_TransferSize((uint32_t)ch_config->transfer_size) |
                                GPDMA_DMACCxControl_SBSize((uint32_t)GPDMA_LUTPerBurst[ch_config->src_conn]) |
                                GPDMA_DMACCxControl_DBSize((uint32_t)GPDMA_LUTPerBurst[ch_config->dst_conn]) |
                                GPDMA_DMACCxControl_SWidth((uint32_t)GPDMA_LUTPerWid[ch_config->src_conn]) |
                                GPDMA_DMACCxControl_DWidth((uint32_t)GPDMA_LUTPerWid[ch_config->dst_conn]) |
                                GPDMA_DMACCxControl_I;
            break;

        /* Unsupported transfer type, return an error */
        default:
            return -1;
    }

    /* Re-Configure DMA Request Select for source peripheral */
    if((ch_config->src_conn != 8)&&(ch_config->src_conn != 9)) {
        if (ch_config->src_conn > 15) {
            LPC_SC->DMAREQSEL |=  (1 << (ch_config->src_conn - 16));
        } else {
            LPC_SC->DMAREQSEL &= ~(1 << (ch_config->src_conn));
        }
    }

    /* Re-Configure DMA Request Select for Destination peripheral */
    if((ch_config->dst_conn != 8) && (ch_config->dst_conn != 9)) {
        if (ch_config->dst_conn > 15) {
            LPC_SC->DMAREQSEL |=  (1 << (ch_config->dst_conn - 16));
        } else {
            LPC_SC->DMAREQSEL &= ~(1 << (ch_config->dst_conn));
        }
    }

    /* Enable DMA channels, little endian */
    LPC_GPDMA->Config = GPDMA_DMACConfig_E;
    while (!(LPC_GPDMA->Config & GPDMA_DMACConfig_E));

    /* Calculate absolute value for Connection number */
    tmp1 = ch_config->src_conn;
    tmp1 = ((tmp1 > 15) ? (tmp1 - 16) : tmp1);
    tmp2 = ch_config->dst_conn;
    tmp2 = ((tmp2 > 15) ? (tmp2 - 16) : tmp2);

    /* Configure DMA Channel, enable Error Counter and Terminate counter */
    pDMAch->CConfig = GPDMA_DMACCxConfig_IE | GPDMA_DMACCxConfig_ITC | /* GPDMA_DMACCxConfig_E |*/
                      GPDMA_DMACCxConfig_TransferType((uint32_t)ch_config->transfer_type) |
                      GPDMA_DMACCxConfig_SrcPeripheral(tmp1) |
                      GPDMA_DMACCxConfig_DestPeripheral(tmp2);

    /* Configure the channel info struct */
    GPDMA_Vector[ch_config->ch].handler = handler;
    GPDMA_Vector[ch_config->ch].arg = arg;

    return 0;
}


/*!
 * \brief Enable/Disable DMA channel
 *
 * Setup GPDMA channel peripheral according to the specified
 * parameters in the ch_config.
 *
 * \param  ch        GPDMA channel, should be in range from 0 to 7
 * \param  enabled   New state of this channel: 1: enabled, 0: disabled
 *
 * \return none
 */
void Lpc17xxGPDMA_ChannelCmd(uint8_t ch, int enabled)
{
    LPC_GPDMACH_TypeDef *pDMAch;

    /* Get Channel pointer */
    pDMAch = (LPC_GPDMACH_TypeDef *) pGPDMACh[ch];

    if (enabled) {
        pDMAch->CConfig |= GPDMA_DMACCxConfig_E;
    } else {
        pDMAch->CConfig &= ~GPDMA_DMACCxConfig_E;
    }
}


/*!
 * \brief Check interrupt status
 *
 * Check if corresponding channel does have an active interrupt request or not
 *
 * \param  type     type of status, should be:
 *                      - GPDMA_STAT_INT:       GPDMA Interrupt Status
 *                      - GPDMA_STAT_INTTC:     GPDMA Interrupt Terminal Count Request Status
 *                      - GPDMA_STAT_INTERR:    GPDMA Interrupt Error Status
 *                      - GPDMA_STAT_RAWINTTC:  GPDMA Raw Interrupt Terminal Count Status
 *                      - GPDMA_STAT_RAWINTERR: GPDMA Raw Error Interrupt Status
 *                      - GPDMA_STAT_ENABLED_CH:GPDMA Enabled Channel Status
 *
 * \param  ch       GPDMA channel, should be in range from 0 to 7
 *
 * \return          status of DMA channel interrupt after masking
 *                      1: the corresponding channel has no active interrupt request
 *                      0: the corresponding channel does have an active interrupt request
 */

int  Lpc17xxGPDMA_IntGetStatus(gpdma_status_t type, uint8_t ch)
{
    switch (type) {
        /* check status of DMA channel interrupts */
        case GPDMA_STAT_INT:
            if (LPC_GPDMA->IntStat & (GPDMA_DMACIntStat_Ch(ch))) {
                return 1;
            }
            return 0;

        /* check terminal count interrupt request status for DMA */
        case GPDMA_STAT_INTTC:
            if (LPC_GPDMA->IntTCStat & GPDMA_DMACIntTCStat_Ch(ch)) {
                return 1;
            }
            return 0;

        /* check interrupt status for DMA channels */
        case GPDMA_STAT_INTERR:
            if (LPC_GPDMA->IntErrStat & GPDMA_DMACIntTCClear_Ch(ch)) {
                return 1;
            }
            return 0;

        /* check status of the terminal count interrupt for DMA channels */
        case GPDMA_STAT_RAWINTTC:
            if (LPC_GPDMA->RawIntErrStat & GPDMA_DMACRawIntTCStat_Ch(ch)) {
                return 1;
            }
            return 0;

        /* check status of the error interrupt for DMA channels */
        case GPDMA_STAT_RAWINTERR:
            if (LPC_GPDMA->RawIntTCStat & GPDMA_DMACRawIntErrStat_Ch(ch)) {
                return 1;
            }
            return 0;

        /* check enable status for DMA channels */
        default:
            if (LPC_GPDMA->EnbldChns & GPDMA_DMACEnbldChns_Ch(ch)) {
                return 1;
            }
            return 0;
    }
}


/*!
 * \brief Clear one or more interrupt requests on DMA channels
 *
 * \param  type     type of status, should be:
 *                      - GPDMA_STATCLR_INTTC:  GPDMA Interrupt Terminal Count Request Clear
 *                      - GPDMA_STATCLR_INTERR: GPDMA Interrupt Error Clear
 *
 * \param  ch       GPDMA channel, should be in range from 0 to 7
 *
 * \return          none
 */
void Lpc17xxGPDMA_ClearIntPending(gpdma_state_clear_t type, uint8_t ch)
{
    if (type == GPDMA_STATCLR_INTTC) {
        /* clears the terminal count interrupt request on DMA channel */
        LPC_GPDMA->IntTCClear = GPDMA_DMACIntTCClear_Ch(ch);
    } else {
        /* clear the error interrupt request */
        LPC_GPDMA->IntErrClr = GPDMA_DMACIntErrClr_Ch(ch);
    }
}

