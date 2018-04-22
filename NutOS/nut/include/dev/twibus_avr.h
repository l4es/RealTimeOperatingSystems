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
 */

#ifndef _DEV_TWIBUS_AVR_H_
#define _DEV_TWIBUS_AVR_H_

#include <sys/types.h>
#include <cfg/arch.h>

typedef struct _NUTTWIICB NUTTWIICB;
/*
 * Runtime Data container.
 * This is installed in heap at initializaton
 * of a bus.
 */
struct _NUTTWIICB {
    /********** Master mode *********/

    /*! \brief Flag that interface is busy
     */
    volatile uint_fast8_t tw_if_busy;

    /*! \brief Bus slave address.
     */
    volatile uint_fast16_t tw_mm_sla;

    /*! \brief Bus current error condition.
     */
    volatile int_fast8_t tw_mm_err;

    /*! \brief Bus last error condition.
     */
    volatile int_fast8_t tw_mm_error;

    /*! \brief Bus transmission data buffer pointer.
     */
    const uint8_t *tw_mm_txbuf;

    /*! \brief Bus transmission data block length.
     */
    volatile uint_fast16_t tw_mm_txlen;

    /*! \brief Bus transmissinn position.
     */
    volatile uint_fast16_t tw_mm_txidx;

    /*! \brief Bus reception data buffer pointer.
     */
    uint8_t *tw_mm_rxbuf;

    /*! \brief Bus reception data block length.
     */
    volatile uint_fast16_t tw_mm_rxlen;

    /*! \brief Bus reception position.
     */
    volatile uint_fast16_t tw_mm_rxidx;


    /*! \brief Bus data direction.
     */
    volatile uint_fast8_t tw_mm_dir;

    /*! \brief Transmission Ongoing Mutex.
     */
    HANDLE tw_mm_mtx;

    /********** Slave mode *********/


    /*! \brief Slave address received.
     */
    uint_fast8_t tw_sm_sla;

    /*! \brief Current slave mode error.
     */
    volatile uint_fast8_t tw_sm_err;

    /*! \brief Last slave mode error.
     */
    volatile uint_fast8_t tw_sm_error;


    /*! \brief Pointer to the slave transmit buffer.
     */
    uint8_t *tw_sm_txbuf;

    /*! \brief Number of bytes to transmit in slave mode.
     */
    uint_fast16_t tw_sm_txlen;

    /*! \brief Current slave transmit buffer index.
     */
    volatile uint_fast16_t tw_sm_txidx;

    /*! \brief Pointer to the slave receive buffer.
     */
    uint8_t *tw_sm_rxbuf;

    /*! \brief Size of the master receive buffer.
     */
    volatile uint_fast16_t tw_sm_rxlen;

    /*! \brief Current slave receive buffer index.
     */
    volatile uint_fast16_t tw_sm_rxidx;

    /*! \brief Threads waiting for slave receive.
     */
    HANDLE tw_sm_rxmtx;

    /*! \brief Threads waiting for slave transmit done.
     */
    HANDLE tw_sm_txmtx;
};

extern NUTTWIBUS AVRTwiBus;

#ifndef DEF_TWIBUS
#define DEF_TWIBUS AVRTwiBus
#endif

#endif /* _DEV_TWIBUS_AVR_H_ */
