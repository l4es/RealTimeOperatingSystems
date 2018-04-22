#ifndef _STM32_CAN_H_
#define _STM32_CAN_H_
/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2012 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de).
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
 */

/*!
 * \struct _CANFRAME canbus.h
 * \brief CAN frame structure
 */

struct _CANBUFFER {
    CAN_FIFOMailBox_TypeDef *dataptr;  //<physical memory address where the buffer is stored, may be device specific
    uint8_t size;         // the allocated size of the buffer
    volatile uint8_t datalength;   // the length of the data currently in the buffer
    uint8_t dataindex;    // the index into the buffer where the data starts
};
/*!
 * \struct _CANINFO canbus.h
 * \brief CAN controller information structure.
 * This is installed in heap at initializaton
 * of a bus.
 *
 * We need a counter for every interrupt, or otherwise
 * we need to use atomic increment
 */
struct _CANBUSINFO {
    HANDLE volatile can_rx_rdy;     /*!< Receiver event queue. */
    HANDLE volatile can_tx_rdy;     /*!< Transmitter event queue. */
    uint32_t can_rx_frames;         /*!< Number of packets received. */
    uint32_t can_tx_frames;         /*!< Number of packets sent. */
    uint32_t can_rx_interrupts;     /*!< Number of interrupts. */
    uint32_t can_tx_interrupts;     /*!< Number of interrupts. */
    uint32_t can_sce_interrupts;    /*!< Number of interrupts. */
    uint32_t can_overruns;          /*!< Number of packet overruns. */
    uint32_t can_errors;            /*!< Number of frame errors. */
    uint32_t can_rx_timeout;        /*!< Receive timeout to wait when no frame available */
    uint32_t can_tx_timeout;        /*!< Transmit timeout to wait when slot gets available */
    CANBUFFER can_RxBuf;            /*!< Buffer for RX queue. */
};

struct _NUTCANBUS {
    uptr_t      bus_base;       /*< Periphery Base Register Address */
    __IO uint32_t* bb_base;     /*< Periphery BIT BAND Base Register Address */
    IRQ_HANDLER *sig_rx_irq;    /*< IRQ Handler RX Interrupt */
    IRQ_HANDLER *sig_tx_irq;    /*< IRQ Handler TX Interrupt */
    IRQ_HANDLER *sig_sce_irq;   /*< IRQ Handler SCE Interrupt */
    struct _CANBUSINFO  *bus_ci;        /*< CANINFO for the BUS */
    int (*bus_inithw)(void);    /*< Function for low level hardware initialization */
};

extern NUTCANBUS Stm32CanBus1;
extern NUTCANBUS Stm32CanBus1C;
extern NUTCANBUS Stm32CanBus2;
extern NUTCANBUS Stm32CanBus2C;

#endif
