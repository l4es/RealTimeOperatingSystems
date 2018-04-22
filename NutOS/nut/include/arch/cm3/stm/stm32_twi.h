#ifndef _STM32_TWI_H_
#define _STM32_TWI_H_
/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
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

/*
 * \verbatim
 * $Id: stm32_twi.h 5070 2013-03-20 14:50:47Z u_bonnes $
 * \endverbatim
 */

/*!
 * \file arch/cm3/dev/stm/stm32_twi.h
 * \brief Header file for STM32F I2C bus
 */

typedef struct _NUTTWIICB NUTTWIICB;
/*
 * Runtime Data container.
 * This is installed in heap at initializaton
 * of a bus.
 */
struct _NUTTWIICB {
    /********** Master mode *********/

    /*! \brief Bus slave address.
     */
    volatile uint_fast16_t tw_mm_sla;

    /*! \brief Bus current error condition.
     */
    volatile int_fast8_t tw_mm_err;

    /*! \brief Bus last error condition.
     */
    volatile int_fast8_t tw_mm_error;

    /*! \brief Bus nodes internal address register length.
     */
    uint8_t *tw_mm_iadr;

    /*! \brief Bus nodes internal address register.
     */
    volatile uint_fast8_t tw_mm_iadrlen;

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
};

extern NUTTWIBUS Stm32TwiBus_1;
extern NUTTWIBUS Stm32TwiBus_2;

#define I2C1_DMA_CHANNEL_TX           DMA1_C6
#define I2C1_DMA_CHANNEL_RX           DMA1_C7

#define I2C2_DMA_CHANNEL_TX           DMA1_C4
#define I2C2_DMA_CHANNEL_RX           DMA1_C5

#endif /* _STM32_TWI_H_ */
