#ifndef _DEV_TWIF_H_
#define _DEV_TWIF_H_

/*
 * Copyright (C) 2001-2005 by egnite Software GmbH. All rights reserved.
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

/*
 * $Log$
 * Revision 1.3  2008/08/11 06:59:59  haraldkipp
 * BSD types replaced by stdint types (feature request #1282721).
 *
 * Revision 1.2  2005/10/24 10:56:30  haraldkipp
 * Added const modifier to transmit data pointer in TwMasterTransact().
 *
 * Revision 1.1.1.1  2003/05/09 14:41:09  haraldkipp
 * Initial using 3.2.1
 *
 * Revision 1.2  2003/03/31 14:53:23  harald
 * Prepare release 3.1
 *
 */

#include <sys/types.h>
#include <cfg/arch.h>

#include <cfg/twi.h>
#include <dev/irqreg.h>
#include <stdint.h>

/*!
 * \brief TWI ioctl() control codes
 */
#define TWI_SETSPEED        0x0401  /*!< \brief Set transfer speed. */
#define TWI_GETSPEED        0x0402  /*!< \brief Query transfer speed. */
#define TWI_SETSLAVEADDRESS 0x0403  /*!< \brief Set local slave address. */
#define TWI_GETSLAVEADDRESS 0x0404  /*!< \brief Query local slave address. */
#define TWI_SETSTATUS       0x0409  /*!< \brief Set status. */
#define TWI_GETSTATUS       0x040a  /*!< \brief Query status. */

/*!
 * \brief TWI error codes
 */
#define TWERR_OK          0  /*!< \brief No error occured. */
#define TWERR_TIMEOUT    -1  /*!< \brief Interface timeout. */
#define TWERR_BUS        -2  /*!< \brief Bus error. */
#define TWERR_IF_LOCKED  -3  /*!< \brief Interface locked. */
#define TWERR_SLA_NACK   -4  /*!< \brief No slave response. */
#define TWERR_DATA_NACK  -5  /*!< \brief Data not acknowledged. */
#define TWERR_OVRE       -6  /*!< \brief Overrun Error. */
#define TWERR_ARBLOST    -7  /*!< \brief Arbitration lost. */
#define TWERR_SPCTOUT    -8  /*!< \brief Start/Stop Condition timeout */
#define TWERR_BUSY       -9  /*!< \brief Bus is busy timeout */

#define TWSLA_MIN        17  /*!< \brief Lowest slave address.
                              * Addresses below are reserved
                              * for special purposes.
                              */
#define TWSLA_MAX        79  /*!< \brief Highest slave address.
                              * Addresses above are reserved
                              * for special purposes.
                              */
#define TWSLA_BCAST       0  /*!< \brief Broadcast slave address. */
#define TWSLA_HOST       16  /*!< \brief Host slave address. */
#define TWSLA_DEFAULT   193  /*!< \brief Default slave address. */

typedef struct _NUTTWIBUS NUTTWIBUS;


/* Include architecture specific TWI implementation */
#if defined(__AVR__)

#include "dev/twibus_avr.h"

#elif defined(__arm__) && defined(__CORTEX__)

#if defined(MCU_STM32)
#include <arch/cm3/stm/stm32_twi.h>
#endif

#elif defined(__arm__) && !defined(__CORTEX__)

#if defined(MCU_AT91R40008)
#include "dev/twibus_bbif.h"
#else
#include "dev/twibus_at91.h"
#endif

#elif defined(__m68k__)
#include <arch/m68k/twi.h>
#endif


/*!
 * \brief TWI/I2C bus structure.
 */
struct _NUTTWIBUS {
    /*! \brief Bus base address.
     */
    uptr_t bus_base;

    /*! \brief Bus data and event interrupt handler.
     */
    IRQ_HANDLER *bus_sig_ev;

    /*! \brief Bus error interrupt handler.
     * If not supported by your device, leave it empty.
     */
    IRQ_HANDLER *bus_sig_er;

    /*! \brief Bus lock queue.
     */
    HANDLE bus_mutex;

    /*! \brief Interface Control Block.
     */
    NUTTWIICB *bus_icb;

    /*! \brief DMA channel for TX direction.
     */
    uint_fast8_t bus_dma_tx;

    /*! \brief DMA channel for RX direction.
     */
    uint_fast8_t bus_dma_rx;

    /*! \brief Initialize bus controller.
     *
     * This routine is called during device registration.
     */
    int (*bus_initbus) (void);

    /*! \brief Recover bus controller.
     *
     * This routine is called for recovering where a slave hangs with SCL low.
     */
    int (*bus_recover) (void);

};


extern int NutTwiMasterTranceive( NUTTWIBUS  *bus,
                                  uint8_t     sla,
                                  const void *txdata, uint16_t txlen,
                                  void       *rxdata, uint16_t rxsiz,
                                  uint32_t    tmo );

extern int NutTwiMasterRegRead( NUTTWIBUS *bus,
                                uint8_t    sla,
                                uint32_t   iadr, uint8_t iadrlen,
                                void      *rxdata, uint16_t rxsiz,
                                uint32_t   tmo );

extern int NutTwiMasterRegWrite( NUTTWIBUS  *bus,
                                 uint8_t     sla,
                                 uint32_t    iadr, uint8_t iadrlen,
                                 const void *txdata, uint16_t txsiz,
                                 uint32_t    tmo );

extern int NutTwiMasterError(NUTTWIBUS *bus);

extern int NutTwiSlaveListen(NUTTWIBUS *bus, uint8_t *sla, void *rxdata, uint16_t rxsiz, uint32_t tmo);

extern int NutTwiSlaveRespond(NUTTWIBUS *bus, void *txdata, uint16_t txlen, uint32_t tmo);

extern int NutTwiSlaveError(NUTTWIBUS *bus);

extern uint16_t NutTwiIndexes( NUTTWIBUS *bus, uint8_t idx );

extern int NutTwiIOCtl( NUTTWIBUS *bus, int req, void *conf );

extern int NutRegisterTwiBus( NUTTWIBUS *bus, uint8_t sla );

extern int NutDestroyTwiBus( NUTTWIBUS *bus);

/*
 * Nut/OS Adaption to old TWI implementation
 */
#if defined(DEF_TWIBUS)
#define TwInit(slv) NutRegisterTwiBus(&DEF_TWIBUS, slv)
#define TwIOCtl(req, conf) NutTwiIOCtl(&DEF_TWIBUS, req, conf)

#define TwMasterTransact( sla, txd, txl, rxd, rxs, tmo) NutTwiMasterTranceive(&DEF_TWIBUS, sla, txd, txl, rxd, rxs, tmo)
#define TwMasterRegRead( sla, iadr, ial, rxd, rxs, tmo) NutTwiMasterRegRead(&DEF_TWIBUS, sla, iadr, ial, rxd, rxs, tmo)
#define TwMasterRegWrite( sla, iadr, ial, txd, txs, tmo) NutTwiMasterRegWrite(&DEF_TWIBUS, sla, iadr, ial, txd, txs, tmo)
#define TwMasterError(void) NutTwiMasterError(&DEF_TWIBUS)
#define TwMasterIndexes( idx) NutTwiIndexes(&DEF_TWIBUS, idx)

#define TwSlaveListen(sla, rxdata, rxsiz, tmo) NutTwiSlaveListen(&DEF_TWIBUS, rxdata, rxsiz, tmo)
#define TwSlaveRespond(txdata, txlen, tmo) NutTwiSlaveRespond(&DEF_TWIBUS, txlen, tmo)
#define TwSlaveError(void) NutTwiSlaveError(&DEF_TWIBUS)
#else
#define TwInit(slv) -1
#define TwIOCtl(req, conf)

#define TwMasterTransact( sla, txd, txl, rxd, rxs, tmo) -1
static inline int TwMasterRegRead(uint8_t sla, uint32_t iadr, uint8_t ial, const void* rxd, uint16_t rxs, uint32_t tmo)
{
    (void)sla;
    (void)iadr;
    (void)ial;
    (void)rxd;
    (void)rxs;
    (void)tmo;
    return -1;
}


#define TwMasterRegWrite( sla, iadr, ial, txd, txs, tmo) -1
#define TwMasterError(void)
#define TwMasterIndexes( idx)

#define TwSlaveListen(sla, rxdata, rxsiz, tmo)
#define TwSlaveRespond(txdata, txlen, tmo)
#define TwSlaveError(void)


#endif

#endif
