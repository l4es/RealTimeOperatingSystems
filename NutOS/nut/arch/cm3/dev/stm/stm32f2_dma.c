/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Nikolaj Zamotaev. All rights reserved.
 * Copyright (C) 2012 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
 * $Id$
 * \endverbatim
 */

/* Both F2 and F4 have 8 DMA streams with 8 requests per stream
   Only DMA2 can do Memory to memory transfers

   DMA/Controller(D), Stream(S) and Request(R) are encoded in one byte

   Byte 7  6  5  4  3  2  1  0
        D  S2 S1 S0 0  R2 R1 R0

*/
#include <cfg/arch.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>
#include <dev/gpio.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_dma.h>

#define CH2NR(ch) ((ch & 0x70)>>4)
#define CH2STREAM(base, ch) (base + 0x10 + ( CH2NR(ch) * 0x18))

/*
 * \brief DMA Transfer Setup Function.
 *
 * This function sets up a DMA transfer. This function automatically
 * detects the transfer direction and if sources or targets are memory
 * or peripherals and sets up the right flags.
 *
 * \param channel One of the available channes of the STM32. If used
 *        with peripherals, check to use the correct channel for the
 *        desired peripheral unit.
 * \param dst Destination where to send the data to. Can be memory or peripheral.
 * \param src Source to take the data from. Can be memory or peripheral.
 * \param flags Option flags for priority and autoincrement of memory or peripheral.
 *
 */
void DMA_Setup( uint8_t ch, void* dst, void* src, uint16_t length, uint32_t flags)
{
    uint32_t dma_base = (ch& 0x80)?DMA2_BASE:DMA1_BASE;
    uint32_t stream_base = CH2STREAM(dma_base, ch);
    uint32_t cc = flags & ~(DMA_SxCR_CHSEL||DMA_SxCR_DIR|DMA_SxCR_EN);
    uint32_t cp;
    uint32_t cm;

    /* Clear all stream related flags */
    DMA_ClearFlag( ch, DMA_TEIF| DMA_HTIF| DMA_TCIF);
    cc |= (ch & 0x7)<<_BI32(DMA_SxCR_CHSEL_0);
    /* if DMA stream running, Stop DMA stream */
    if (CM3BBGET(stream_base, DMA_Stream_TypeDef,
                 CR, _BI32(DMA_SxCR_EN)))
    {
        /* Disable stream */
        CM3BBCLR(stream_base, DMA_Stream_TypeDef, CR, _BI32(DMA_SxCR_EN));
        while(CM3BBGET(stream_base, DMA_Stream_TypeDef,
                       CR, _BI32(DMA_SxCR_EN)));
    }

    /* Detect transfer type and set Registers accordingly */
    if ((uint32_t)src & PERIPH_BASE) {
        /* Peripheral to Memory */
        cp=(uint32_t)src;
        cm=(uint32_t)dst;
    }
    else if ((uint32_t)dst & PERIPH_BASE) {
        /* Memory to Peripheral */
        cc |= DMA_SxCR_DIR_0;
        cp=(uint32_t)dst;
        cm=(uint32_t)src;
    }
    else {
        /* Memory to Memory Transfer */
        if (dma_base == DMA1_BASE)
            /* FIXME: Provide return value */
            return;
        cc |= DMA_SxCR_DIR_1;
        cp =(uint32_t)src;
        cm =(uint32_t)dst;
    }

    CM3REG(stream_base, DMA_Stream_TypeDef, CR)=cc;
    CM3REG(stream_base, DMA_Stream_TypeDef, NDTR)=length;
    CM3REG(stream_base, DMA_Stream_TypeDef, PAR)=cp;
    CM3REG(stream_base, DMA_Stream_TypeDef, M0AR)=cm;

};

/*
 * \brief Enable DMA Transfer.
 */
void DMA_Enable(uint8_t ch)
{
    uint32_t dma_base = (ch& 0x80)?DMA2_BASE:DMA1_BASE;
    uint32_t stream_base = CH2STREAM(dma_base, ch);
    CM3BBSET(stream_base, DMA_Stream_TypeDef, CR, _BI32(DMA_SxCR_EN));
}

/*
 * \brief Query if DMA Channel is enable.
 *
 * \param ch Channel to query
 * \return 0 if disabled
 */
int DmaIsEnabled(uint8_t ch)
{
    uint32_t dma_base = (ch& 0x80)?DMA2_BASE:DMA1_BASE;
    uint32_t stream_base = CH2STREAM(dma_base, ch);
    return CM3BBGET(stream_base, DMA_Stream_TypeDef, CR, _BI32(DMA_SxCR_EN));
}

/*
 * \brief Disable DMA Transfer.
 */
void DMA_Disable(uint8_t ch)
{
    uint32_t dma_base = (ch& 0x80)?DMA2_BASE:DMA1_BASE;
    uint32_t stream_base = CH2STREAM(dma_base, ch);
    do{
        CM3BBCLR(stream_base, DMA_Stream_TypeDef, CR, _BI32(DMA_SxCR_EN));
    } while (CM3BBGET(stream_base, DMA_Stream_TypeDef, CR, _BI32(DMA_SxCR_EN)));
}


/*!
 * \brief DMA System Initialization
 *
 * Register all DMA interrupt handlers.
 * Both DMA controllers are initialized separate
 */
#if 0
void DMA_Init(uint8_t ch)
{
    uint8_t stream_nr = CH2NR(ch);
    uint32_t dma_base = (ch& 0x80)?DMA2_BASE:DMA1_BASE;
    if (ch < 0x80)
    {
        if (!(CM3BBGET(RCC_BASE, RCC_TypeDef, AHB1ENR,
                       _BI32(RCC_AHB1ENR_DMA1EN ))))
        {
            /* Reset DMA1 */
            CM3BBSET(RCC_BASE, RCC_TypeDef, AHB1RSTR, _BI32(RCC_AHB1RSTR_DMA1RST));
            /* Enable DMA1 Clock */
            CM3BBSET(RCC_BASE, RCC_TypeDef, AHB1ENR, _BI32(RCC_AHB1ENR_DMA1EN ));
            CM3BBCLR(RCC_BASE, RCC_TypeDef, AHB1RSTR, _BI32(RCC_AHB1RSTR_DMA1RST));
            return;
        }
    }
    else if (!(CM3BBGET(RCC_BASE, RCC_TypeDef, AHB1ENR, _BI32(RCC_AHB1ENR_DMA2EN ))))
    {
        /* Reset DMA2 */
        CM3BBSET(RCC_BASE, RCC_TypeDef, AHB1RSTR, _BI32(RCC_AHB1RSTR_DMA2RST));
        /* Enable DMA2 Clock */
        CM3BBSET(RCC_BASE, RCC_TypeDef, AHB1ENR, _BI32(RCC_AHB1ENR_DMA2EN ));
        CM3BBCLR(RCC_BASE, RCC_TypeDef, AHB1RSTR, _BI32(RCC_AHB1RSTR_DMA2RST));
        return;
    }
    switch (stream_nr)
    {
    case 0: CM3REG(dma_base, DMA_TypeDef, LIFCR) &= ~(DMA_LIFCR_CFEIF0|DMA_LIFCR_CDMEIF0|DMA_LIFCR_CTEIF0 |DMA_LIFCR_CHTIF0 |DMA_LIFCR_CTCIF0);
        break;
    case 1: CM3REG(dma_base, DMA_TypeDef, LIFCR) &= ~(DMA_LIFCR_CFEIF1|DMA_LIFCR_CDMEIF1|DMA_LIFCR_CTEIF1 |DMA_LIFCR_CHTIF1 |DMA_LIFCR_CTCIF1);
        break;
    case 2: CM3REG(dma_base, DMA_TypeDef, LIFCR) &= ~(DMA_LIFCR_CFEIF2|DMA_LIFCR_CDMEIF2|DMA_LIFCR_CTEIF2 |DMA_LIFCR_CHTIF2 |DMA_LIFCR_CTCIF2);
        break;
    case 3: CM3REG(dma_base, DMA_TypeDef, LIFCR) &= ~(DMA_LIFCR_CFEIF3|DMA_LIFCR_CDMEIF3|DMA_LIFCR_CTEIF3 |DMA_LIFCR_CHTIF3 |DMA_LIFCR_CTCIF3);
        break;
    case 4: CM3REG(dma_base, DMA_TypeDef, HIFCR) &= ~(DMA_HIFCR_CFEIF4|DMA_HIFCR_CDMEIF4|DMA_HIFCR_CTEIF4 |DMA_HIFCR_CHTIF4 |DMA_HIFCR_CTCIF4);
        break;
    case 5: CM3REG(dma_base, DMA_TypeDef, HIFCR) &= ~(DMA_HIFCR_CFEIF4|DMA_HIFCR_CDMEIF4|DMA_HIFCR_CTEIF4 |DMA_HIFCR_CHTIF4 |DMA_HIFCR_CTCIF4);
        break;
    case 6: CM3REG(dma_base, DMA_TypeDef, HIFCR) &= ~(DMA_HIFCR_CFEIF4|DMA_HIFCR_CDMEIF4|DMA_HIFCR_CTEIF4 |DMA_HIFCR_CHTIF4 |DMA_HIFCR_CTCIF4);
        break;
    case 7: CM3REG(dma_base, DMA_TypeDef, HIFCR) &= ~(DMA_HIFCR_CFEIF4|DMA_HIFCR_CDMEIF4|DMA_HIFCR_CTEIF4 |DMA_HIFCR_CHTIF4 |DMA_HIFCR_CTCIF4);
        break;
    }
#else
void DMA_Init(void)
{
    if (!CM3BBGET(RCC_BASE, RCC_TypeDef, AHB1ENR, _BI32(RCC_AHB1ENR_DMA1EN ))) {
        /* Reset DMA1 */
        CM3BBSET(RCC_BASE, RCC_TypeDef, AHB1RSTR, _BI32(RCC_AHB1RSTR_DMA1RST));
        /* Enable DMA1 Clock */
        CM3BBSET(RCC_BASE, RCC_TypeDef, AHB1ENR, _BI32(RCC_AHB1ENR_DMA1EN ));
        CM3BBCLR(RCC_BASE, RCC_TypeDef, AHB1RSTR, _BI32(RCC_AHB1RSTR_DMA1RST));
    }
    if (!CM3BBGET(RCC_BASE, RCC_TypeDef, AHB1ENR, _BI32(RCC_AHB1ENR_DMA2EN ))) {
        /* Reset DMA2 */
        CM3BBSET(RCC_BASE, RCC_TypeDef, AHB1RSTR, _BI32(RCC_AHB1RSTR_DMA2RST));
        /* Enable DMA2 Clock */
        CM3BBSET(RCC_BASE, RCC_TypeDef, AHB1ENR, _BI32(RCC_AHB1ENR_DMA2EN ));
        CM3BBCLR(RCC_BASE, RCC_TypeDef, AHB1RSTR, _BI32(RCC_AHB1RSTR_DMA2RST));
    }
    return;
}
#endif

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
 *
 * On F2/F4, Interrupt enable bits and interrupt clear flags don't correspond!
 * The Clear flags are arranged with sffereing spacing!
 */
void DMA_IrqMask( uint8_t ch, uint32_t mask, uint8_t ena)
{
    uint32_t dma_base = (ch& 0x80)?DMA2_BASE:DMA1_BASE;
    uint32_t stream_base = CH2STREAM(dma_base, ch);
    uint32_t en_dis_msk = 0, clr_msk = DMA_LIFCR_CFEIF0|DMA_LIFCR_CDMEIF0;

    if (mask & DMA_TCIF)
    {
        en_dis_msk |= DMA_SxCR_TCIE;
        clr_msk    |= DMA_LIFCR_CTCIF0;
    }
    if (mask & DMA_HTIF)
    {
        en_dis_msk |= DMA_SxCR_HTIE;
        clr_msk    |= DMA_LIFCR_CHTIF0;
    }
    if (mask & DMA_TEIF)
    {
        en_dis_msk |= DMA_SxCR_TEIE;
        clr_msk    |= DMA_LIFCR_CTEIF0;
    }
    if (ena)
        CM3REG(stream_base, DMA_Stream_TypeDef, CR) |= en_dis_msk;
    else
        CM3REG(stream_base, DMA_Stream_TypeDef, CR) &= ~en_dis_msk;
    switch (CH2NR(ch))
    {
    case 0: CM3REG(dma_base, DMA_TypeDef, LIFCR) |= clr_msk; break;
    case 1: CM3REG(dma_base, DMA_TypeDef, LIFCR) |= clr_msk<<6; break;
    case 2: CM3REG(dma_base, DMA_TypeDef, LIFCR) |= clr_msk<<16; break;
    case 3: CM3REG(dma_base, DMA_TypeDef, LIFCR) |= clr_msk<<22; break;
    case 4: CM3REG(dma_base, DMA_TypeDef, HIFCR) |= clr_msk; break;
    case 5: CM3REG(dma_base, DMA_TypeDef, HIFCR) |= clr_msk<<6; break;
    case 6: CM3REG(dma_base, DMA_TypeDef, HIFCR) |= clr_msk<<16; break;
    case 7: CM3REG(dma_base, DMA_TypeDef, HIFCR) |= clr_msk<<22; break;
    }
};

/*!
 * \brief Clear DMA channel flags.
 *
 * \param ch    Channel number.
 * \param flags Mask of flags to clear.
 */
void DMA_ClearFlag( uint8_t ch, uint32_t flags)
{
    uint32_t dma_base = (ch& 0x80)?DMA2_BASE:DMA1_BASE;
    uint32_t clr_msk = DMA_LIFCR_CFEIF0|DMA_LIFCR_CDMEIF0|DMA_LIFCR_CTEIF0;
    if (flags & DMA_TCIF) clr_msk    |= DMA_LIFCR_CTCIF0;
    if (flags & DMA_HTIF) clr_msk    |= DMA_LIFCR_CHTIF0;
    if (flags & DMA_TEIF) clr_msk    |= DMA_LIFCR_CTEIF0;
    switch (CH2NR(ch))
    {
    case 0: CM3REG(dma_base, DMA_TypeDef, LIFCR) |= clr_msk; break;
    case 1: CM3REG(dma_base, DMA_TypeDef, LIFCR) |= clr_msk<<6; break;
    case 2: CM3REG(dma_base, DMA_TypeDef, LIFCR) |= clr_msk<<16; break;
    case 3: CM3REG(dma_base, DMA_TypeDef, LIFCR) |= clr_msk<<22; break;
    case 4: CM3REG(dma_base, DMA_TypeDef, HIFCR) |= clr_msk; break;
    case 5: CM3REG(dma_base, DMA_TypeDef, HIFCR) |= clr_msk<<6; break;
    case 6: CM3REG(dma_base, DMA_TypeDef, HIFCR) |= clr_msk<<16; break;
    case 7: CM3REG(dma_base, DMA_TypeDef, HIFCR) |= clr_msk<<22; break;
    }
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
    uint32_t dma_base = (ch& 0x80)?DMA2_BASE:DMA1_BASE;
    uint32_t flags = 0, ret = 0;
    switch (CH2NR(ch))
    {
    case 0: flags = (CM3REG(dma_base, DMA_TypeDef, LISR)    )   & 0x3d; break;
    case 1: flags = (CM3REG(dma_base, DMA_TypeDef, LISR)>> 6) & 0x3d; break;
    case 2: flags = (CM3REG(dma_base, DMA_TypeDef, LISR)>>16) & 0x3d; break;
    case 3: flags = (CM3REG(dma_base, DMA_TypeDef, LISR)>>22) & 0x3d; break;
    case 4: flags = (CM3REG(dma_base, DMA_TypeDef, HISR)    ) & 0x3d; break;
    case 5: flags = (CM3REG(dma_base, DMA_TypeDef, HISR)>> 6) & 0x3d; break;
    case 6: flags = (CM3REG(dma_base, DMA_TypeDef, HISR)>>16) & 0x3d; break;
    case 7: flags = (CM3REG(dma_base, DMA_TypeDef, HISR)>>22) & 0x3d; break;
    }
    if (flags & DMA_LISR_TCIF0) ret |= DMA_TCIF;
    if (flags & DMA_LISR_HTIF0) ret |= DMA_HTIF;
    if (flags & DMA_LISR_TEIF0) ret |= DMA_TEIF;
    return ret;
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
    uint32_t dma_base = (ch & 0x80)? DMA2_BASE : DMA1_BASE;
    uint32_t stream_base = CH2STREAM(dma_base, ch);
    return CM3REG(stream_base, DMA_Stream_TypeDef, NDTR);
}

/*!
 * \brief     Get memory base address of transfer.
 *
 * For memory-to-memory transfer, we return the destination address.
 *
 * \param ch  Channel number to query.
 *
 * \return    Base address of transfer memory.
 */
void*  DmaGetMemoryBase( uint8_t ch)
{
    uint32_t dma_base = (ch & 0x80)? DMA2_BASE : DMA1_BASE;
    uint32_t stream_base = CH2STREAM(dma_base, ch);
    return (void*) CM3REG(stream_base, DMA_Stream_TypeDef, M0AR);
}
