/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2012-2014 by Uwe Bonnes
 *                          (bon@elektron.ikp.physik.tu-darmstadt.de)
 *
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

/*!
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \file dev/can_dev.h
 * \brief Headers for canbus interface
 */

#include <sys/event.h>
#include <sys/heap.h>
#include <sys/atom.h>

#include <string.h>

#include <dev/canbus.h>

/* Time out for INAK bit */
#define INAK_TimeOut        ((uint32_t)0x0000FFFF)

/* Time out for SLAK bit */
#define SLAK_TimeOut        ((uint32_t)0x0000FFFF)

static int CanSetState(NUTCANBUS *bus, int enable)
{
    int rc;
    uint32_t wait_ack = 0;
    __IO uint32_t *CANBBx = bus->bb_base;

    if (!bus->sig_tx_irq)
        /* we may not set the companion */
        return CAN_IS_COMPANION;
    if(enable)
    {
        CM3BB_OFFSETCLR(CANBBx, CAN_TypeDef, MCR, CAN_MCR_INRQ);

        /* Wait the acknowledge */
        for(wait_ack = 0, rc = 1; (rc) && (wait_ack < INAK_TimeOut); wait_ack++)
            rc  = CM3BB_OFFSETGET(CANBBx, CAN_TypeDef, MSR, CAN_MSR_INAK);
        return (rc == 0)?0:CAN_BUS_OFF;
    }
    else
    {
        CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, MCR, CAN_MCR_INRQ);

        /* Wait the acknowledge */
        for(wait_ack = 0, rc = 0; (!rc) && (wait_ack < INAK_TimeOut); wait_ack++)
            rc  = CM3BB_OFFSETGET(CANBBx, CAN_TypeDef, MSR, CAN_MSR_INAK);
        return (rc==1)?0:CAN_ERROR;
    }
}

static void STMCanTXInterrupt( void *arg)
{
    NUTCANBUS *bus = arg;
    CANBUSINFO *ci = bus->bus_ci;
    __IO uint32_t *CANBBx = bus->bb_base;

    CM3BB_OFFSETCLR(CANBBx, CAN_TypeDef, IER, CAN_IER_TMEIE);
    ci->can_tx_interrupts++;
    NutEventPostFromIrq(&(ci->can_tx_rdy));
}

static void STMCanRX0Interrupt( void *arg)
{
    NUTCANBUS *bus = arg;
    CAN_TypeDef *CANx = (CAN_TypeDef*)bus->bus_base;
    CANBUSINFO *ci = bus->bus_ci;
    CANBUFFER *rxbuf = &(ci->can_RxBuf);
    __IO uint32_t *CANBBx = bus->bb_base;

    if (CM3BB_OFFSETGET(CANBBx, CAN_TypeDef, RF0R, CAN_RF0R_FOVR0))
    {
        CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, RF0R,_BI32(CAN_RF0R_FOVR0));
        ci->can_overruns++;
    }
    ci->can_rx_interrupts++;
    while(CANx->RF0R & 3)
    {
        // Space in buffer ?
        if (rxbuf->datalength < rxbuf->size)
        {
            int index = (rxbuf->dataindex + rxbuf->datalength) % rxbuf->size;
            CAN_FIFOMailBox_TypeDef*bufPtr = &rxbuf->dataptr[index];
            memcpy(bufPtr, &CANx->sFIFOMailBox[0], sizeof(CAN_FIFOMailBox_TypeDef));

            /*Increment buffer length */
            rxbuf->datalength++;
            NutEventPostFromIrq(&ci->can_rx_rdy);
            /* Statistic housekeeping */
            ci->can_rx_frames++;
            CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, RF0R, CAN_RF0R_RFOM0);
            while(CM3BB_OFFSETGET(CANBBx, CAN_TypeDef, RF0R, CAN_RF0R_RFOM0));
        }
        else
        {
            /* No overrun yet, but we can't clean the FIFO yet */
            /* Disable the FIFO message pending interrupt*/
            CM3BB_OFFSETCLR(CANBBx, CAN_TypeDef, IER, CAN_IER_FMPIE0);
            return;
        }
    }
}

static void STMCanRX1Interrupt( void *arg)
{
    NUTCANBUS *bus = arg;
    CAN_TypeDef *CANx = (CAN_TypeDef*)bus->bus_base;
    CANBUSINFO *ci = bus->bus_ci;
    CANBUFFER *rxbuf = &(ci->can_RxBuf);
    __IO uint32_t *CANBBx = bus->bb_base;

    if (CM3BB_OFFSETGET(CANBBx, CAN_TypeDef, RF1R, CAN_RF1R_FOVR1))
    {
        CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, RF1R,_BI32(CAN_RF1R_FOVR1));
        ci->can_overruns++;
    }
    ci->can_rx_interrupts++;
    while(CANx->RF1R & 3)
    {
        // Space in buffer ?
        if (rxbuf->datalength < rxbuf->size)
        {
            int index = (rxbuf->dataindex + rxbuf->datalength) % rxbuf->size;
            CAN_FIFOMailBox_TypeDef*bufPtr = &rxbuf->dataptr[index];
            memcpy(bufPtr, &CANx->sFIFOMailBox[1], sizeof(CAN_FIFOMailBox_TypeDef));

            // Increment buffer length
            rxbuf->datalength++;
            NutEventPostFromIrq(&ci->can_rx_rdy);
            // Stat houskeeping
            ci->can_rx_frames++;
            CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, RF1R, CAN_RF1R_RFOM1);
            while(CM3BB_OFFSETGET(CANBBx, CAN_TypeDef, RF1R, CAN_RF1R_RFOM1));
        }
        else
        {
            /* No overrun yet, but we can't clean the FIFO yet */
            /* Disable the FIFO message pending interrupt*/
            CM3BB_OFFSETCLR(CANBBx, CAN_TypeDef, IER, CAN_IER_FMPIE1);
            return;
        }

    }
}

/* Simple acknowledge the error and count it.
 * TODO: Better error handling
 */
static void STMCanErrorInterrupt( void *arg)
{
    NUTCANBUS *bus = arg;
    CANBUSINFO *ci = bus->bus_ci;
    __IO uint32_t *CANBBx = bus->bb_base;

    CM3BB_OFFSETCLR(CANBBx, CAN_TypeDef, MSR, CAN_MSR_ERRI);
    ci->can_sce_interrupts++;
}

/*
 * \brief Set a CAN identifier filter
 *
 * \param bus
 * \param filter
 * @return 0 if filter could be inserted in the list, CANRESULT else
 *
 * The Filter registers belong to CAN1
 *
 * Filter may be exact 29 bit match(29 E), 29 bit with mask (29M), exact 16 bit (16E) and
 * 11 bit with mask (16M)
 *
 * A STM32 filter bank entry may hold one 29M, two 29E, two 16M and four 16E, with all entries
 * in a bank of same type. if we e.g. add the first 29E entry, we mark the second 29E entry with
 * FILTER_EXPLICIT and replace this placeholder on the write of the second 29E entry. A 16(M|E) entry must
 * have the high 16 id/ask bits NULL
 *
 * We don't explicit stop reception, so when adding 29E/16M/16E to a partial populated
 * filter bank, the other entries of that filter bank are temporary inactive
 *
 */
int CanAddFilter( NUTCANBUS *bus, CANFILTER *filter)
{
    int i, index_start, max_index;
    uint32_t mode = CAN1->FM1R;
    uint32_t scale = CAN1->FS1R;
    uint32_t assignment = CAN1->FFA1R;
    uint32_t activation = CAN1->FA1R;
    int fifo = (bus->sig_tx_irq)?0:1; /* FIFO1 is reveive only*/

    if (bus->bus_base == CAN1_BASE)
    {
        index_start = 0;
        max_index = (CAN1->FMR)>>8 & 0x3f;
    }
    else
    {
        index_start = (CAN1->FMR)>>8 & 0x3f;
        max_index = sizeof(CAN1->sFilterRegister)/sizeof(CAN1->sFilterRegister[0]);
    }

    if(filter->mask == FILTER_EXPLICIT) /* Explicit*/
    {
        if ((filter-> id) > 0xffff) /* 29 E*/
        {
            uint32_t id_29 =   (filter->id   << 3) | ((filter->id_ext)?4:0) | ((filter->id_rtr)?  2:0);
            for(i=index_start; i<max_index; i++)
            {
                if (!(activation & (1<<i)))
                {
                    CAN_FilterRegister_TypeDef *free_entry = &(CAN1->sFilterRegister[i]);
                    free_entry->FR1 = id_29;
                    free_entry->FR2 = 0xffffffff;
                    activation |= (1<<i);
                    if (fifo)
                        assignment |= (1<<i);
                    else
                        assignment &= ~(1<<i);
                    mode  |=  (1<<i);
                    scale |=  (1<<i);
                    goto flt_success;
                }
                else if(((assignment & (1<<i)) == (fifo <<i)) &&
                        (       mode & (1<<i)) &&
                        (      scale & (1<<i)))
                    /* Entry matches type and may have space */
                {
                    CAN_FilterRegister_TypeDef *partial_entry = &(CAN1->sFilterRegister[i]);
                    if (partial_entry->FR2 == 0xffffffff)
                    {
                        CAN1->FA1R = (activation & ~(1<<i)); /* Mark entry for writing */
                        partial_entry->FR2 = id_29;
                        goto flt_success;
                    }
                }
            }
        }
        else
        {
            uint16_t id_16 = ((filter->id >>16) & 0x7) |((filter->id  & 0x7ff)  << 5) | ((filter->id_ext  )?8:0) | ((filter->id_rtr  )?  0x10:0);
            for(i=index_start; i<max_index; i++)
            {
                if (!(activation & (1<<i)))
                {
                    CAN_FilterRegister_TypeDef *free_entry = &(CAN1->sFilterRegister[i]);
                    free_entry->FR1 = 0xffff0000 | id_16;
                    free_entry->FR2 = 0xffffffff;
                    activation |= (1<<i);
                    if (fifo)
                        assignment |=  (1<<i);
                    else
                        assignment &= ~(1<<i);
                    mode  |=  (1<<i);
                    scale &= ~(1<<i);
                    goto flt_success;
                }
                else if(((assignment & (1<<i)) == (fifo <<i)) &&
                        (       mode & (1<<i)) &&
                        !(     scale & (1<<i)))
                    /* Entry matches type and may have space */
                {
                    /* Only (32 bit) word access to CAN Registers allowed! */
                    CAN_FilterRegister_TypeDef *partial_entry = &(CAN1->sFilterRegister[i]);
                    if ((partial_entry->FR1 & 0xffff0000) == 0xffff0000)
                    {
                        CAN1->FA1R = (activation & ~(1<<i)); /* Mark entry for writing */
                        partial_entry->FR1 &= 0x0000ffff;
                        partial_entry->FR1 |= (id_16 <<16);
                        goto flt_success;
                    }
                    else if (partial_entry->FR2 == 0xffffffff)
                    {
                        CAN1->FA1R = (activation & ~(1<<i)); /* Mark entry for writing */
                        partial_entry->FR2 = 0xffff0000 | id_16;
                        goto flt_success;
                    }
                    else if ((partial_entry->FR2 & 0xffff0000) == 0xffff0000)
                    {
                        CAN1->FA1R = (activation & ~(1<<i)); /* Mark entry for writing */
                        partial_entry->FR2 &= 0x0000ffff;
                        partial_entry->FR2 |= (id_16 <<16);
                        goto flt_success;
                    }
                }
            }
        }
    }
    else if (filter->mask > 0xffff) /* 29 M*/
    {
        for(i=index_start; i<max_index; i++)
        {
            /* find first free entry */
            if (!(activation & (1<<i)))
            {
                CAN_FilterRegister_TypeDef *free_entry = &(CAN1->sFilterRegister[i]);
                uint32_t id_29   = (filter->id   << 3) | ((filter->id_ext  )?4:0) | ((filter->id_rtr  )?2:0);
                uint32_t mask_29 = (filter->mask << 3) | ((filter->mask_ext)?4:0) | ((filter->mask_rtr)?2:0);
                free_entry->FR1 = id_29;
                free_entry->FR2 = mask_29;
                activation |= (1<<i);
                if (fifo)
                    assignment |= (1<<i);
                else
                    assignment &= ~(1<<i);
                mode &= ~(1<<i);
                scale |= (1<<i);
                goto flt_success;
            }
        }
    }
    else
    {
        uint16_t id_16   =  ((filter->id  >>16) & 0x7) |((filter->id  & 0x7ff)  << 5) | ((filter->id_ext  )?8:0) | ((filter->id_rtr  )?  0x10:0);
        uint16_t mask_16 =  ((filter->mask>>16) & 0x7) |((filter->mask& 0x7ff)  << 5) | ((filter->mask_ext)?8:0) | ((filter->mask_rtr)?  0x10:0);
        for(i=index_start; i<max_index; i++)
        {
            if (!(activation & (1<<i)))
            {
                CAN_FilterRegister_TypeDef *free_entry = &(CAN1->sFilterRegister[i]);
                free_entry->FR1 = (mask_16 << 16) | id_16;
                free_entry->FR2 = 0xffffffff;
                activation |= (1<<i);
                if (fifo)
                    assignment |= (1<<i);
                else
                    assignment &= ~(1<<i);
                mode  &= ~(1<<i);
                scale &= ~(1<<i);
                goto flt_success;
            }

            else if(((assignment & (1<<i)) == (fifo <<i)) &&
                    !(      mode & (1<<i)) &&
                    !(      scale & (1<<i)))
                /* Entry matches type and may have space */
            {
                CAN_FilterRegister_TypeDef *partial_entry = &(CAN1->sFilterRegister[i]);
                if (partial_entry->FR2 == 0xffffffff)
                {
                    CAN1->FA1R = (activation & ~(1<<i)); /* Mark entry for writing */
                    partial_entry->FR2 =  (mask_16 << 16) |id_16;
                    goto flt_success;
                 }
            }
        }
    }
    return CAN_ILLEGAL_MOB;
flt_success:
    CAN1->FM1R = mode;
    CAN1->FS1R = scale;
    CAN1->FFA1R = assignment;
    CAN1->FA1R = activation;
    return 0;
}

int CanRxAvail(NUTCANBUS *bus)
{
    CANBUSINFO *ci = bus->bus_ci;
    CANBUFFER *rxbuf = &(ci->can_RxBuf);
    return rxbuf->datalength;
}

/*!
 * \brief Set CAN Bus features during initialization.
 *
 *
 */
int CanSetFeatures( NUTCANBUS *bus, uint32_t flags, uint8_t ena)
{
    int rc = 0;
    CAN_TypeDef *CANx = (CAN_TypeDef*)bus->bus_base;
    uint32_t mcr = CANx->MCR;

    if (!bus->sig_tx_irq)
        /* we may not set the companion */
        return CAN_IS_COMPANION;
    if (CanSetState(bus, 0))
        return CAN_ERROR;

    /* Set the time triggered communication mode */
    if (flags & CAN_TTCM) {
        if( ena)
            mcr |= CAN_MCR_TTCM;
        else
            mcr &= ~CAN_MCR_TTCM;
    }

    /* Set the automatic bus-off management */
    if (flags&CAN_ABOM) {
        if( ena)
            mcr |= CAN_MCR_ABOM;
        else
            mcr &= ~CAN_MCR_ABOM;
    }

    /* Set the automatic wake-up mode */
    if (flags&CAN_AWUM) {
        if (ena)
            mcr |= CAN_MCR_AWUM;
        else
            mcr &= ~CAN_MCR_AWUM;
    }

    /* Set the no automatic retransmission */
    if (flags&CAN_NART) {
        if (ena)
            mcr |= CAN_MCR_NART;
        else
            mcr &= ~CAN_MCR_NART;
    }

    /* Set the receive FIFO locked mode */
    if (flags&CAN_RFLM) {
        if (ena)
            mcr |= CAN_MCR_RFLM;
        else
            mcr &= ~CAN_MCR_RFLM;
    }

    /* Set the transmit FIFO priority */
    if (flags&CAN_TXFP) {
        if (ena)
            mcr |= CAN_MCR_TXFP;
        else
            mcr &= ~CAN_MCR_TXFP;
    }
    CANx->MCR = mcr;

    if (CanSetState(bus, 1))
        return CAN_BUS_OFF;
    return rc;
}

int CanGetFeatures( NUTCANBUS *bus, uint32_t flags)
{
    int rc = CAN_ERROR;
    CAN_TypeDef *CANx = (CAN_TypeDef*)bus->bus_base;
    uint32_t mcr = CANx->MCR;

    if (flags&CAN_TTCM) {
        /* Get time triggered communication mode */
        rc = (mcr&CAN_MCR_TTCM)?1:0;
    }
    else if (flags&CAN_ABOM) {
        /* Get automatic bus-off management */
        rc = (mcr&CAN_MCR_ABOM)?1:0;
    }
    else if (flags&CAN_AWUM) {
        /* Get automatic wake-up mode */
        rc = (mcr&CAN_MCR_AWUM)?1:0;
    }
    else if (flags&CAN_NART) {
        /* Get the no automatic retransmission config */
        rc = (mcr&CAN_MCR_NART)?1:0;
    }
    else if (flags&CAN_RFLM) {
        /* Get receive FIFO locked mode */
        rc = (mcr&CAN_MCR_RFLM)?1:0;
    }
    else if (flags&CAN_TXFP) {
        /* Set the transmit FIFO priority */
        rc = (mcr&CAN_MCR_TXFP)?1:0;
    }

    return rc;
}

#define CAN_APB1_DIV(x) ((x-1) & 0x3ff)
#define CAN_BS1(x) (((x-1) & 0xf) << 16)
#define CAN_BS2(x) (((x-1) & 0x7) << 20)
#define CAN_SJW(x) (((x-1) & 0x3) << 24)
#if defined(MCU_STM32F1) || defined(MCU_STM32F3)
#define STM_CAN_ABP1_CHECK 36000000
#define STM_CAN_BTR_1M   (CAN_APB1_DIV(2)  |CAN_BS1(15)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_800k (CAN_APB1_DIV(3)  |CAN_BS1(12)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_500k (CAN_APB1_DIV(4)  |CAN_BS1(14)|CAN_BS2(3)|CAN_SJW(2))
#define STM_CAN_BTR_250k (CAN_APB1_DIV(9)  |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_125k (CAN_APB1_DIV(18) |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_100k (CAN_APB1_DIV(20) |CAN_BS1(14)|CAN_BS2(3)|CAN_SJW(2))
#define STM_CAN_BTR_50k  (CAN_APB1_DIV(45) |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_20k  (CAN_APB1_DIV(100)|CAN_BS1(15)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_10k  (CAN_APB1_DIV(225)|CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#elif defined(MCU_STM32L1)
#define STM_CAN_ABP1_CHECK 32000000
#define STM_CAN_BTR_1M   (CAN_APB1_DIV(2)  |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_800k (CAN_APB1_DIV(2)  |CAN_BS1(16)|CAN_BS2(3)|CAN_SJW(2))
#define STM_CAN_BTR_500k (CAN_APB1_DIV(4)  |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_250k (CAN_APB1_DIV(8)  |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_125k (CAN_APB1_DIV(16) |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_100k (CAN_APB1_DIV(20) |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_50k  (CAN_APB1_DIV(40) |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_20k  (CAN_APB1_DIV(100)|CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_10k  (CAN_APB1_DIV(200)|CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#elif defined(MCU_STM32F2)
#define STM_CAN_ABP1_CHECK 30000000
#define STM_CAN_BTR_1M   (CAN_APB1_DIV(2)  |CAN_BS1(12)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_800k 0
#define STM_CAN_BTR_500k (CAN_APB1_DIV(3)  |CAN_BS1(16)|CAN_BS2(3)|CAN_SJW(2))
#define STM_CAN_BTR_250k (CAN_APB1_DIV(6)  |CAN_BS1(16)|CAN_BS2(3)|CAN_SJW(2))
#define STM_CAN_BTR_125k (CAN_APB1_DIV(15) |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_100k (CAN_APB1_DIV(20) |CAN_BS1(12)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_50k  (CAN_APB1_DIV(30) |CAN_BS1(16)|CAN_BS2(3)|CAN_SJW(2))
#define STM_CAN_BTR_20k  (CAN_APB1_DIV(75) |CAN_BS1(16)|CAN_BS2(3)|CAN_SJW(2))
#define STM_CAN_BTR_10k  (CAN_APB1_DIV(150)|CAN_BS1(16)|CAN_BS2(3)|CAN_SJW(2))
#elif defined(MCU_STM32F4) || defined(MCU_STM32F7)
#define STM_CAN_ABP1_CHECK 42000000
#define STM_CAN_BTR_1M   (CAN_APB1_DIV(2)  |CAN_BS1(16)|CAN_BS2(4)|CAN_SJW(2))
#define STM_CAN_BTR_800k 0
#define STM_CAN_BTR_500k (CAN_APB1_DIV(4)  |CAN_BS1(16)|CAN_BS2(4)|CAN_SJW(2))
#define STM_CAN_BTR_250k (CAN_APB1_DIV(8)  |CAN_BS1(16)|CAN_BS2(4)|CAN_SJW(2))
#define STM_CAN_BTR_125k (CAN_APB1_DIV(21) |CAN_BS1(13)|CAN_BS2(2)|CAN_SJW(2))
#define STM_CAN_BTR_100k (CAN_APB1_DIV(20) |CAN_BS1(16)|CAN_BS2(4)|CAN_SJW(2))
#define STM_CAN_BTR_50k  (CAN_APB1_DIV(40) |CAN_BS1(16)|CAN_BS2(4)|CAN_SJW(2))
#define STM_CAN_BTR_20k  (CAN_APB1_DIV(100)|CAN_BS1(16)|CAN_BS2(4)|CAN_SJW(2))
#define STM_CAN_BTR_10k  (CAN_APB1_DIV(200)|CAN_BS1(16)|CAN_BS2(4)|CAN_SJW(2))
#else
#warning "Unknown STM32 family"
#endif
#define BTR2FREQ(x) (((x & 0x3ff) + 1) * (((x >> 16) & 0xf) + ((x >> 20) & 0x7) + 3))
/* Check the defines */
#if (STM_CAN_BTR_10k != 0) && ((10000 * BTR2FREQ(STM_CAN_BTR_10k)) != STM_CAN_ABP1_CHECK)
#warning "Bad BTR setting for STM_CAN_BTR_10k"
#endif
#if (STM_CAN_BTR_20k != 0) && ((20000 * BTR2FREQ(STM_CAN_BTR_20k)) != STM_CAN_ABP1_CHECK)
#warning "Bad BTR setting for STM_CAN_BTR_20k"
#endif
#if (STM_CAN_BTR_50k != 0) && ((50000 * BTR2FREQ(STM_CAN_BTR_50k)) != STM_CAN_ABP1_CHECK)
#warning "Bad BTR setting for STM_CAN_BTR_50k"
#endif
#if (STM_CAN_BTR_100k != 0) && ((100000 * BTR2FREQ(STM_CAN_BTR_100k)) != STM_CAN_ABP1_CHECK)
#warning "Bad BTR setting for STM_CAN_BTR_100k"
#endif
#if (STM_CAN_BTR_125k != 0) && ((125000 * BTR2FREQ(STM_CAN_BTR_125k)) != STM_CAN_ABP1_CHECK)
#warning "Bad BTR setting for STM_CAN_BTR_125k"
#endif
#if (STM_CAN_BTR_250k != 0) && ((250000 * BTR2FREQ(STM_CAN_BTR_250k)) != STM_CAN_ABP1_CHECK)
#warning "Bad BTR setting for STM_CAN_BTR_250k"
#endif
#if (STM_CAN_BTR_500k != 0) && ((500000 * BTR2FREQ(STM_CAN_BTR_500k)) != STM_CAN_ABP1_CHECK)
#warning "Bad BTR setting for STM_CAN_BTR_500k"
#endif
#if (STM_CAN_BTR_800k != 0) && ((800000 * BTR2FREQ(STM_CAN_BTR_800k)) != STM_CAN_ABP1_CHECK)
#warning "Bad BTR setting for STM_CAN_BTR_800k"
#endif
#if (STM_CAN_BTR_1M != 0) && ((1000000 * BTR2FREQ(STM_CAN_BTR_1M)) != STM_CAN_ABP1_CHECK)
#warning "Bad BTR setting for STM_CAN_BTR_1M"
#endif

/*!
 * Set the baudrate
 * \param bus  Identifies the CANBUS
 * \param baud Symbolic value for the Baudrate
 * \param alt_btr Use given value with CAN_SPEED_CUSTOM
 *
 * For STM32, alt_btr may be used for setting silent/loopback silent+loopback mode
 *
 * \return 0 if baudrate can be delivered or -1 else
 */
int CanSetBaud( NUTCANBUS *bus, int baud, uint32_t alt_btr)
{
    /* CAN is connected to APB1 bus. Max frequency varies with family:
       F1: 36 MHz
       L1: 32 MHz
       F2: 30 MHz
       F3: 30 MHz
       F4: 42 MHz
       Assume this frequency for defining the BTR register for now.
       If we need to cope with other APB1 Clocks, this gets much more
       complicated.
       If my understanding is right, BS1 == 13 and BS2 == 2 gives the
       CiA proposed 87.5 % sampling point. Sync is always 1.
       (Sync + BS1) /(Sync + BS1 + BS2) = 14/16 = 0.875
       For high rates we keep and adjust BS1
    */
    uint32_t btr= 0;
    CAN_TypeDef *CANx = (CAN_TypeDef*)bus->bus_base;
    if (!bus->sig_tx_irq)
        /* we may not set the companion */
        return CAN_IS_COMPANION;
    if (CanSetState(bus, 0))
        return CAN_ERROR;
    switch (baud)
    {
       case CAN_SPEED_10K:
       btr = STM_CAN_BTR_10k;
       break;
       case CAN_SPEED_20K:
       btr = STM_CAN_BTR_20k;
       break;
       case CAN_SPEED_50K:
       btr = STM_CAN_BTR_50k;
       break;
       case CAN_SPEED_100K:
       btr = STM_CAN_BTR_100k;
       break;
       case CAN_SPEED_125K:
       btr = STM_CAN_BTR_125k;
       break;
       case CAN_SPEED_250K:
       btr = STM_CAN_BTR_250k;
       break;
       case CAN_SPEED_500K:
       btr = STM_CAN_BTR_500k;
       break;
       case CAN_SPEED_800K:
       btr = STM_CAN_BTR_800k;
       break;
       case CAN_SPEED_1M:
       btr = STM_CAN_BTR_1M;
       break;
       case CAN_SPEED_CUSTOM:
       btr = alt_btr;
       break;
       default:
       return CAN_INVALID_SPEED;
    }
    if (btr == 0)
    return CAN_ERROR;
    CANx->BTR &= ~0x03ff03ff ;
    CANx->BTR |= btr;
    if (CanSetState(bus, 1))
        return CAN_BUS_OFF;
    return 0;
}

void CANSetRxTimeout(NUTCANBUS *bus, uint32_t timeout)
{
    CANBUSINFO *ci = bus->bus_ci;
    ci->can_rx_timeout = timeout;
}

void CANSetTxTimeout(NUTCANBUS *bus, uint32_t timeout)
{
    CANBUSINFO *ci = bus->bus_ci;
    ci->can_tx_timeout = timeout;
}

static int Stm32CanBusInit( NUTCANBUS *bus)
{
    int rc = 0;
    __IO uint32_t *CANBBx = bus->bb_base;
    uint32_t wait_ack = 0;

    if (!(bus->sig_tx_irq)) { /* for the companion, we only check that CAN1/2 is clock */
        if (bus->bus_base == CAN1_BASE) {
            rc = CM3BBGET(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_CAN1EN));
            CM3BBSET(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN1RST));
            CM3BBCLR(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN1RST));
        } else {
#ifdef RCC_APB1ENR_CAN2EN
            rc = CM3BBGET(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_CAN2EN));
            CM3BBSET(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN2RST));
            CM3BBCLR(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN2RST));
#endif
        }
        if (rc) {
            return 0;
        } else {
            return CAN_NO_COMPANION;
        }
    }

    /* If bus has hardware init function, call it. */
    if( bus->bus_inithw) {
        rc = (bus->bus_inithw)();
    }
    if (rc)
        return rc;

    /* Software Master reset*/
    CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, MCR, CAN_MCR_RESET);

    /* exit from sleep mode */
    CM3BB_OFFSETCLR(CANBBx, CAN_TypeDef, MCR, CAN_MCR_SLEEP);
    for(wait_ack = 0, rc = 1; (rc) && (wait_ack < SLAK_TimeOut); wait_ack++)
        rc  = CM3BB_OFFSETGET(CANBBx, CAN_TypeDef, MSR, CAN_MSR_SLAK);
    if (rc) {
        return CAN_ERROR;
    }

    /* We send tx mailboxes in chronological order and
       FOR NOW we don't retransmit on errer */
    CanSetFeatures( bus, CAN_TXFP | CAN_NART | CAN_ABOM , 1);

    return rc;
}

#define CAN_DEF_RX_ENTRIES 16
/*!
 * \brief Initialize CAN interface.
 *
 * \param bus
 * \param entries Number of buffer Mailboxes, CAN_DEF_RX_ENTRIES if <1.
 *
 * Here we set up the Hardware, baudrate and special setting
 *
 */
    int NutRegisterCanBus( NUTCANBUS *bus, int entries )
{
    int rc = CAN_ERROR;

    CANBUSINFO *ci = NULL;
    CANBUFFER *rxbuf;

    if (entries < 1)
        entries = CAN_DEF_RX_ENTRIES;

    void *dataptr  =  NutHeapAlloc(entries * sizeof(CAN_FIFOMailBox_TypeDef));
    if (dataptr == 0)
        return rc;
    memset( dataptr, 0, entries * sizeof(CAN_FIFOMailBox_TypeDef));

    ci = NutHeapAlloc(sizeof(CANBUSINFO));
    if( ci == NULL) {
        NutHeapFree(dataptr);
        return rc;
    }
    memset( ci, 0, sizeof(CANBUSINFO));
    rxbuf = &(ci->can_RxBuf);
    rxbuf->dataptr = dataptr;
    rxbuf->size = entries;
    rxbuf->dataindex = 0;
    rxbuf->datalength = 0;
    ci->can_rx_timeout = NUT_WAIT_INFINITE;

    /* Link bus and ci */
    bus->bus_ci = ci;
    if (bus->sig_tx_irq)
    {
        if( NutRegisterIrqHandler( bus->sig_tx_irq, STMCanTXInterrupt, bus ) ) {
            NutHeapFree(dataptr);
            NutHeapFree( ci);
            return rc;
        }
    }
    if (bus->sig_rx_irq)
    {
        if (bus->sig_tx_irq)
        {
            if( NutRegisterIrqHandler( bus->sig_rx_irq, STMCanRX0Interrupt, bus ) ) {
                NutHeapFree(dataptr);
                NutHeapFree( ci);
                return rc;
            }
        }
        else
        {
            if( NutRegisterIrqHandler( bus->sig_rx_irq, STMCanRX1Interrupt, bus ) ) {
                NutHeapFree(dataptr);
                NutHeapFree( ci);
                return rc;
            }
        }
    }
    if (bus->sig_sce_irq)
    {
        if( NutRegisterIrqHandler( bus->sig_sce_irq, STMCanErrorInterrupt, bus ) ) {
            NutHeapFree(dataptr);
            NutHeapFree( ci);
            return rc;
        }
    }

    rc= Stm32CanBusInit(bus);
    if (rc)
    {
        NutHeapFree(dataptr);
        NutHeapFree( ci);
        return rc;
    }
     if (bus->sig_tx_irq)
     {
         NutIrqSetPriority(bus->sig_tx_irq, 0);
         rc = NutIrqEnable(bus->sig_tx_irq);
         if( rc) {
             NutHeapFree(dataptr);
             NutHeapFree( ci);
             return rc;
         }
     }
    if (bus->sig_rx_irq)
    {
        NutIrqSetPriority(bus->sig_rx_irq, 1);
        rc = NutIrqEnable(bus->sig_rx_irq);
        if( rc) {
            NutHeapFree(dataptr);
            NutHeapFree( ci);
            return rc;
        }
    }
    if (bus->sig_sce_irq)
    {
        NutIrqSetPriority(bus->sig_sce_irq, 2);
        rc = NutIrqEnable(bus->sig_sce_irq);
        if( rc) {
            NutHeapFree(dataptr);
            NutHeapFree( ci);
            return rc;
        }
    }
    return rc;
}


/**
 * @internal
 * Search for a free mailbox
 * @return Index of first free mailbox or -1 if none is available
 */
static int CANGetFreeMailbox(NUTCANBUS *bus)
{
    CAN_TypeDef *CANx = (CAN_TypeDef*)bus->bus_base;
    uint32_t tsr = CANx->TSR;
    if (tsr & CAN_TSR_TME0)
    return 0;
    if (tsr & CAN_TSR_TME1)
    return 1;
    if (tsr & CAN_TSR_TME2)
    return 2;
    return CAN_TXBUF_FULL;
}

/* Fixme: This definition has to go to a common header to be agreed on*/
#if defined(GCC)
#define __MAY_ALIAS (__attribute__((__may_alias__)))
#else
#define __MAY_ALIAS
#endif

/**
 * Send a CAN message
 *
 * For buffers, we only use the internal tx mailboxes
 *
 * @param frame Container for CAN message to be sent
 * @return Result code. See @ref CAN_RESULT
 */
static int StmCanSendMsg(NUTCANBUS  *bus, CANFRAME *frame)
{
    CAN_TypeDef *CANx = (CAN_TypeDef*)bus->bus_base;
    __IO uint32_t *CANBBx = bus->bb_base;
    uint32_t __MAY_ALIAS *tdlr = (uint32_t*) &(frame->byte[0]);
    uint32_t __MAY_ALIAS *tdhr = (uint32_t*) &(frame->byte[4]);
    uint32_t tir;
    CAN_TxMailBox_TypeDef *tx_mailbox;
    int index = CANGetFreeMailbox(bus);

    if (index < 0)
    {
        CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, IER, CAN_IER_TMEIE);
        return CAN_TXBUF_FULL;
    }

    tx_mailbox = &(CANx->sTxMailBox[index]);
    tx_mailbox->TDLR = *tdlr;
    tx_mailbox->TDHR = *tdhr;
    tx_mailbox->TDTR = frame->len;
    if (frame->ext)
    {
    tir = frame->id << 3;
    tir |= CAN_TI0R_IDE;
    }
    else
    {
    tir = frame->id << 21;
    }
    tir |= CAN_TI0R_TXRQ | ((frame->rtr)?CAN_TI0R_RTR : 0);
    tx_mailbox->TIR = tir;
    return CAN_SUCCESS;
}

/*!
 * Checks if there's still space in output buffer
 *
 * \param dev Pointer to the device structure
 * \return 1 if space is available
 */
int CanTxFree(NUTCANBUS *bus)
{
    if (!bus->sig_tx_irq)
        /* companion can not transmitt */
        return 0;
    return (CANGetFreeMailbox(bus) >= 0)?1:0;
}

/*!
 * Reads a frame from input buffer
 *
 * This function reads a frame from the input buffer. If the input buffer
 * is empty the function will block unitl new frames are received,
 * or the timeout is reached.
 *
 * \param dev Pointer to the device structure
 * \param frame Pointer to the receive frame
 * \return 1 if timeout, 0 otherwise
 */
int CanInput(NUTCANBUS *bus, CANFRAME * frame)
{
    CANBUSINFO *ci = bus->bus_ci;
    __IO uint32_t *CANBBx = bus->bb_base;
    uint32_t __MAY_ALIAS *rdlr = (uint32_t*) &(frame->byte[0]);
    uint32_t __MAY_ALIAS *rdhr = (uint32_t*) &(frame->byte[4]);
    CANBUFFER *rxbuf = &(ci->can_RxBuf);
    CAN_FIFOMailBox_TypeDef *dataPtr ;

    while (rxbuf->datalength == 0)
    {
        if (NutEventWait(&ci->can_rx_rdy, ci->can_rx_timeout))
            return 1;
    }

    NutEnterCritical();
    dataPtr = &rxbuf->dataptr[rxbuf->dataindex];
    rxbuf->dataindex++;
    if (rxbuf->dataindex >= rxbuf->size)
        rxbuf->dataindex -= rxbuf->size;
    rxbuf->datalength--;
    NutExitCritical();
    /* Reenable interrupt*/
    if (bus->sig_tx_irq)
        CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, IER, CAN_IER_FMPIE0);
    else
        CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, IER, CAN_IER_FMPIE1);

    frame->rtr = (dataPtr->RIR & CAN_RI0R_RTR)?1:0;
    frame->id = dataPtr->RIR >>((dataPtr->RIR & CAN_RI0R_IDE)?3:21);
    if(frame->rtr)
        frame->id &= 0x3ff;
    frame->ext = (dataPtr->RIR & CAN_RI0R_IDE)?1:0;
    frame->rtr = (dataPtr->RIR & CAN_RI0R_RTR)?1:0;
    *rdlr = dataPtr->RDLR;
    *rdhr = dataPtr->RDHR;
    frame->len = dataPtr->RDTR & 0xf;
    return 0;
}

/*
 * Enable FIFO message pending interrupt and activate filters
 */
void CanEnableRx(NUTCANBUS *bus)
{
    __IO uint32_t *CANBBx = bus->bb_base;
    if(bus->sig_tx_irq)
        CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, IER, CAN_IER_FMPIE0);
    else
        CM3BB_OFFSETSET(CANBBx, CAN_TypeDef, IER, CAN_IER_FMPIE1);
    CM3BB_OFFSETCLR(CANBBx, CAN_TypeDef, FMR, CAN_FMR_FINIT);
}


/*!
 * Write a frame from to output buffer
 *
 * This function writes a frame to the output buffer. If the output buffer
 * is full the function will block until frames are send.
 *
 * \param bus Pointer to the device structure
 * \param frame Pointer to the receive frame
 */
int CanOutput(NUTCANBUS *bus, CANFRAME * frame)
{
    int rc;
    CANBUSINFO *ci = bus->bus_ci;
    if (!bus->sig_tx_irq)
        /* companion can not send */
        return CAN_IS_COMPANION;
    while ((rc = StmCanSendMsg(bus, frame)) == CAN_TXBUF_FULL)
    {
        if (NutEventWait(&ci->can_rx_rdy, ci->can_tx_timeout))
            return 1;
    };
    ci->can_tx_frames++;
    return rc;
}

/*!
 * Report the counter values
 *
 * This function writes a frame to the output buffer. If the output buffer
 * is full the function will block until frames are send.
 *
 * \param bus Pointer to the device structure
 * \param index Number of pointer to receive
 *
 * \return -1 if counter not available, counter value otherwise
 */
int CanGetCounter(NUTCANBUS *bus, enum CAN_COUNTERS index)
{
    CANBUSINFO *ci = bus->bus_ci;
    switch (index)
    {
    case CAN_RX_FRAMES: return ci->can_rx_frames;
    case CAN_TX_FRAMES: return ci->can_tx_frames;
    case CAN_INTERRUPTS:
        return (ci->can_rx_interrupts +ci->can_tx_interrupts +ci->can_sce_interrupts);
    case CAN_RX_INTERRUPTS:
        return ci->can_rx_interrupts;
    case CAN_TX_INTERRUPTS:
        return ci->can_tx_interrupts;
    case CAN_SCE_INTERRUPTS:
        return ci->can_sce_interrupts;
    case CAN_OVERRUNS:
        return ci->can_overruns;
    case CAN_ERRORS:
        return ci->can_errors;
    default:
        return -1;
    }
    return -1;
}
