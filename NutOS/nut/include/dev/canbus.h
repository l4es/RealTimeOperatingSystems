#ifndef _CANBUS_H_
#define _CANBUS_H_
/*
 * Copyright (C) 2012 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file dev/canbus.h
 * \brief Headers for canbus interface
 */

#include <cfg/arch.h>
#include <cfg/can_dev.h>
#include <sys/types.h>
#include <sys/device.h>

#include <dev/irqreg.h>

/*!
 * \addtogroup xgCanDev
 */
/*@{*/

/*!
 * \brief CAN Baud rate constants
 */
#define CAN_SPEED_10K      0   ///< 10 kbit/s, max. cable length 5000 m
#define CAN_SPEED_20K      1   ///< 20 kbit/s, max. cable length 2500 m
#define CAN_SPEED_50K      2   ///< 50 kbit/s, max. cable length 1000 m
#define CAN_SPEED_100K     3   ///< 100 kbit/s, max. cable length 600 m
#define CAN_SPEED_125K     4   ///< 125 kbit/s, max. cable length 500 m
#define CAN_SPEED_250K     5   ///< 250 kbit/s, max. cable length 250 m
#define CAN_SPEED_500K     6   ///< 500 kbit/s, max. cable length 100 m
#define CAN_SPEED_800K     7   ///< 800 kbit/s, max. cable length 50 m
#define CAN_SPEED_1M       8   ///< 1 Mbit/s, max. cable length 25 m
#define CAN_SPEED_CUSTOM  -1

/*!
 * \brief CAN error codes
 */
enum CAN_RESULT
{
   CAN_SUCCESS       =  0,   ///< Successful operation
   CAN_ERROR         = -1,   ///< Unspecified error
   CAN_TXBUF_FULL    = -2,   ///< All TX message objects busy
   CAN_RXBUF_EMPTY   = -3,   ///< All RX message objects busy
   CAN_ILLEGAL_MOB   = -4,   ///< Message object index out of range
   CAN_INVALID_SPEED = -5,   ///< Invalid baud rate parameter
   CAN_PASSIVE       = -6,   ///< Bus is in passive state
   CAN_BUS_OFF       = -7,   ///< Bus is bus-off
   CAN_NO_COMPANION  = -8,   ///< FIFO2 needs FIFO1 active
   CAN_IS_COMPANION  = -9,   ///< FIFO2 can only set Filters
};

/*!
 * \brief CAN event counters
 */
enum CAN_COUNTERS
{
    CAN_RX_FRAMES      =  0,   /*!< Number of packets received. */
    CAN_TX_FRAMES      =  1,   /*!< Number of packets sent. */
    CAN_INTERRUPTS     =  2,   /*!< Number of interrupts. */
    CAN_RX_INTERRUPTS  =  3,   /*!< Number of RX interrupts. */
    CAN_TX_INTERRUPTS  =  4,   /*!< Number of TX interrupts. */
    CAN_SCE_INTERRUPTS =  5,   /*!< Number of ERROR interrupts. */
    CAN_OVERRUNS       =  6,   /*!< Number of packet overruns. */
    CAN_ERRORS         =  7,   /*!< Number of frame errors. */
    CAN_NO_COUNTERS    =  8,
};

/* Enable / disable time triggered communication mode */
#define CAN_TTCM    0x0001

/* Enable / disable automatic bus-off management */
#define CAN_ABOM    0x0002

/* Set the automatic wake-up mode */
#define CAN_AWUM    0x0004

/* Enable / disable no automatic retransmission */
#define CAN_NART    0x0008

/* Enable / disable receive FIFO locked mode */
#define CAN_RFLM    0x0010

/* Enable / disable transmit FIFO priority */
#define CAN_TXFP    0x0020

/*!
 * \brief CAN bus structure, defined by device
 */
typedef struct _NUTCANBUS NUTCANBUS;
/*!
 * \brief CAN bus info structure, defined by device
 */
typedef struct _CANBUSINFO CANBUSINFO;

/*!
 * \brief CAN BUFFER structure, defined by device
 */
typedef struct _CANBUFFER CANBUFFER;


/* Include architecture specific CAN implementation */
#if defined(DEV_CAN)
#if defined(MCU_STM32)
#include <arch/cm3/stm/stm32_can.h>
#else
#warning DEV_CANBUS defined, but no device specific implementation givem
#endif
#endif

/*!
 * \struct _CANFRAME canbus.h
 * \brief CAN frame structure
 */

struct _CANFRAME {              // todo: Implement flags
    uint32_t id;                  // Identifier
    uint8_t byte[8];
    uint8_t len;                 // Length of frame, max = 8
    uint8_t ext;                 // Boolean, extendet frame
    uint8_t rtr;                 // Boolean, remote transmition bit
};

/*!
 * \brief CAN frame type
 */

typedef struct _CANFRAME CANFRAME;

/*!
 * \struct _CANFILTER canbus.h
 * \brief CAN message filter structure
 */

struct _CANFILTER{           //
    uint32_t id;             ///< Identifier
    uint32_t mask;           ///< Mask, use 0xffffffff for exactt match
    uint8_t id_ext;          ///< Boolean, extended frame
    uint8_t id_rtr;          ///< Boolean, remote transmition bit
    uint8_t mask_ext;        ///< Boolean, match id_ext
    uint8_t mask_rtr;        ///< Boolean, match id_rtr
};

/*!
 * \brief CAN message filter type
 */

typedef struct _CANFILTER CANFILTER;

#define FILTER_EXPLICIT 0xffffffff

/*!
 * \brief _NUTCANBUS
 */

extern int NutRegisterCanBus( NUTCANBUS *bus, int entries );
extern int CanAddFilter( NUTCANBUS *bus, CANFILTER *filter );
extern int CanSetBaud( NUTCANBUS *bus, int baud, uint32_t alt_btr);
extern void CANSetRxTimeout(NUTCANBUS *bus, uint32_t timeout);
extern void CANSetTxTimeout(NUTCANBUS *bus, uint32_t timeout);
extern void CanEnableRx( NUTCANBUS *bus);
extern int CanRxAvail( NUTCANBUS *bus);
extern int CanInput(NUTCANBUS *bus, CANFRAME *output);
extern int CanTxFree(NUTCANBUS *can);
extern int CanOutput( NUTCANBUS *bus, CANFRAME *output);
extern int CanGetCounter( NUTCANBUS *bus, enum CAN_COUNTERS index);

#endif
