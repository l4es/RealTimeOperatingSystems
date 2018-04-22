/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Nikolaj Zamotaev. All rights reserved.
 * Copyright (C) 2013-2016 by Uwe Bonnes
 *                          (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 */

/*
 * \verbatim
 * $Id: stm32f1_dma.c 6513 2016-06-30 12:08:05Z u_bonnes $
 * \endverbatim
 */

#include <stdlib.h>

#include <cfg/arch.h>
#include <cfg/devices.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_dma.h>

/*!
 * \brief Table to align channels and interrupts for simpler access
 */
static const DMATAB DmaTab[] = {
    { DMA1,  0, DMA1_Channel1 },
    { DMA1,  4, DMA1_Channel2 },
    { DMA1,  8, DMA1_Channel3 },
    { DMA1, 12, DMA1_Channel4 },
    { DMA1, 16, DMA1_Channel5 },
#if !defined(HW_DMA1_5CH_STM32)
    { DMA1, 20, DMA1_Channel6 },
    { DMA1, 24, DMA1_Channel7 },
#endif
#if defined(HW_DMA2_STM32F1)
    { DMA2,  0, DMA2_Channel1 },
    { DMA2,  4, DMA2_Channel2 },
    { DMA2,  8, DMA2_Channel3 },
    { DMA2, 12, DMA2_Channel4 },
    { DMA2, 16, DMA2_Channel5 },
#endif
#if defined(HW_DMA2_7CH_STM32)
    { DMA2, 20, DMA2_Channel6 },
    { DMA2, 24, DMA2_Channel7 },
#endif
};

/*
 * \brief DMA Transfer Setup Function.
 *
 * This function sets up a DMA transfer. This function automatically
 * detects the transfer direction and if sources or targets are memory
 * or peripherals and sets up the right flags.
 *
 * \param channel One of the available channels of the STM32. If used
 *        with peripherals, check to use the correct channel for the
 *        desired peripheral unit.
 * \param dst Destination where to send the data to. Can be memory or peripheral.
 * \param src Source to take the data from. Can be memory or peripheral.
 * \param flags Option flags for priority and autoincrement of memory or peripheral.
 *
 */
void DMA_Setup( uint8_t ch, void* dst, void* src, uint16_t length, uint32_t flags)
{
    DMA_Channel_TypeDef* channel = DmaTab[ch].dma_ch;
    uint32_t cc = flags & ~(DMA_CCR_MEM2MEM | DMA_CCR_DIR | DMA_CCR_EN);
    uint32_t cp;
    uint32_t cm;

    /* Stop DMA channel */
    channel->CCR = cc;

    /* Detect transfer type and set Registers accordingly */
    if ((uint32_t)src & PERIPH_BASE) {
        /* Peripheral to Memory */
        cp = (uint32_t)src;
        cm = (uint32_t)dst;
    } else if ((uint32_t)dst & PERIPH_BASE) {
        /* Memory to Peripheral */
        cc |= DMA_CCR_DIR;
        cp = (uint32_t)dst;
        cm = (uint32_t)src;
    } else {
        /* Memory to Memory Transfer */
        cc |= DMA_CCR_MEM2MEM | DMA_CCR_DIR;
        cp = (uint32_t)src;
        cm = (uint32_t)dst;
    }

    channel->CCR   = cc;
    channel->CNDTR = length;
    channel->CPAR  = cp;
    channel->CMAR  = cm;

};

/*
 * \brief Enable DMA Transfer.
 */
void DMA_Enable(uint8_t ch)
{
    DMA_Channel_TypeDef *channel = DmaTab[ch].dma_ch;
    channel->CCR |= DMA_CCR_EN;
}

/*
 * \brief Query if DMA Channel is enable.
 *
 * \param ch Channel to query
 * \return 0 if disabled
 */
int DmaIsEnabled(uint8_t ch)
{
    DMA_Channel_TypeDef *channel = DmaTab[ch].dma_ch;
    return (channel->CCR & DMA_CCR_EN);
}

/*
 * \brief Disable DMA Transfer.
 */
void DMA_Disable(uint8_t ch)
{
    DMA_Channel_TypeDef *channel = DmaTab[ch].dma_ch;
    do {
        channel->CCR &= ~DMA_CCR_EN;
    } while(channel->CCR & DMA_CCR_EN);
}


/*!
 * \brief DMA System Initialization
 *
 * Register all DMA interrupt handlers.
 */
void DMA_Init(void)
{
    uint8_t i;
    DMA_Channel_TypeDef *channel;

    if ((RCC->AHBENR & RCC_AHBENR_DMA1EN ) == 0) {
        /* Enable DMA clocks */
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;

        /* Clear pending interrupts in DMA 1 ISR */
        DMA1->IFCR = 0xFFFFFFFF;
#if defined(STM_HAS_DMA2) && defined(RCC_AHBENR_DMA2EN)
        if ((RCC->AHBENR & RCC_AHBENR_DMA2EN ) == 0) {
            RCC->AHBENR |= RCC_AHBENR_DMA2EN;
            /* Clear pending interrupts in DMA 2 ISR */
            DMA2->IFCR = 0xFFFFFFFF;
        }
#endif
        /* Clear interrupt related flags in channels */
        for (i = 0; i < DMA_COUNT; i++) {
            channel = DmaTab[i].dma_ch;
            channel->CCR = 0;
            DMA_ClearFlag(i,0xf);
        }
    }
}

/*!
 * \brief Control DMA channel interrupt masks
 *
 * Setup interrupt mask on given channel. Channel numbers are
 * from 0..n while documentation numbers them from 1..n.
 * For that please use the defines DMAx_Cn.
 *
 * \param ch   Channel number to set interrupt mask.
 * \param mask Mask to set on the designated channel interrupts.
 * \param ena  Enable (1) or Disable (0) the bits in the mask.
 */
void DMA_IrqMask( uint8_t ch, uint32_t mask, uint8_t ena)
{
    DMA_Channel_TypeDef *channel = DmaTab[ch].dma_ch;
    uint32_t ccr = channel->CCR;
    DMA_TypeDef *dma = DmaTab[ch].dma;
    uint32_t msk = mask & DMA_FLAGMASK;

    if (ena) {
        /* Enable some/all of the DMA channels interrupts */
        ccr |= msk;
    } else {
        /* Disable some/all of the DMA channels interrupts */
        ccr &= ~msk;
    }

    /* Set channel configuration */
    channel->CCR = ccr;

    /* Clear already pending flags */
    dma->IFCR = (mask << (DmaTab[ch].fofs));
};

/*!
 * \brief Clear DMA channel flags.
 *
 * Setup interrupt mask on given channel. Channel numbers are
 * from 0..n while documentation numbers them from 1..n.
 * For that please use the defines DMAx_Cn.
 *
 * \param ch    Channel number.
 * \param flags Mask of flags to clear.
 */
void DMA_ClearFlag( uint8_t ch, uint32_t flags)
{
    volatile DMA_TypeDef *dma = DmaTab[ch].dma;
    uint32_t msk = (flags & DMA_FLAGMASK) << DmaTab[ch].fofs;

    dma->IFCR = msk;
}

/*!
 * \brief Get DMA channel flags.
 *
 * Get interrupt / status flags of given channel.
 *
 * \param ch    Channel number to set interrupt mask.
 *
 * \return flags Mask of flags to clear.
 */
uint32_t DMA_GetFlag( uint8_t ch)
{
    volatile DMA_TypeDef *dma = DmaTab[ch].dma;
    uint32_t flags = (dma->ISR>>DmaTab[ch].fofs) & DMA_FLAGMASK;
    return flags;
}


/*!
 * \brief     Get number of remaining transfers.
 **
 * \param ch  Channel number to query.
 *
 * \return    Number of remaining transfers.
 */
uint16_t  DMA_GetRemainingTransfers( uint8_t ch)
{
    DMA_Channel_TypeDef *channel = DmaTab[ch].dma_ch;
    return channel->CNDTR;
}

/*!
 * \brief     Get memory base address of transfer.
 *
 * For memory-to-memory transfer, we return the destination address.
 *
 * \param ch  Channel number to query.
 *
 * \return    Base address of transfer memory
 */
void*  DmaGetMemoryBase( uint8_t ch)
{
    DMA_Channel_TypeDef *channel = DmaTab[ch].dma_ch;
    return (void *)channel->CMAR;
}

/*!
 * \brief     Set DMA channel selection
 *
 * F090, L0 and L4 need channel selection set for each type of device.

 *
 * \param ch   Channel number to set.
 * \param csel Channel to set.
 *
 * \return    None
 */
void DmaChannelSelection(uint8_t ch, uint8_t csel)
{
#if defined(HW_DMA_CSELR_STM32)
    uint32_t cselr;
    volatile uint32_t *cselr_p;
    uint8_t shift;
#if defined(MCU_STM32F09) || defined(MCU_STM32L4)
    if (ch > 6 ) {
        cselr_p  = DMA2_CSELR_REG;
        ch -= 6;
    } else {
        cselr_p = DMA1_CSELR_REG;
    }
#else
    cselr_p  = DMA1_CSELR_REG;
#endif
    cselr = *cselr_p;
    shift = ch << 2;
    cselr &= ~( 0xf << shift);
    cselr |=  (csel << shift);
    *cselr_p = cselr;
#endif
}
