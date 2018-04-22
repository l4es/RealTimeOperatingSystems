#ifndef _LPC17XX_SPI_H_
#define _LPC17XX_SPI_H_

/*
 * Copyright (C) 2013 by Simon Budig <simon@budig.de>
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
 * $Id: $
 * \endverbatim
 */

/*----------------------------------------------------------------------------*
  SPI Control Register (S0SPCR)
 *----------------------------------------------------------------------------*/

#define SPI_CR_BITENABLE      (1 << 2)    /* if set send 8..16 bits - see SPI_BITS */
#define SPI_CR_CPHA           (1 << 3)    /* clock phase */
#define SPI_CR_CPOL           (1 << 4)    /* clock polarity */
#define SPI_CR_MSTR           (1 << 5)    /* spi master / slave */
#define SPI_CR_LSBF           (1 << 6)    /* lsb first */
#define SPI_CR_SPIE           (1 << 7)    /* spi interrupt enable */
#define SPI_CR_BITS(x)        (((x) & 0xf) << 8)  /* requires SPI_BITENABLE, valid range 8..16 */
#define SPI_CR_BITS_MASK      (0xf << 8)

/*----------------------------------------------------------------------------*
  SPI Status Register (S0SPSR)
 *----------------------------------------------------------------------------*/

#define SPI_SR_ABRT           (1 << 3)    /* Slave abort */
#define SPI_SR_MODF           (1 << 4)    /* Mode fault */
#define SPI_SR_ROVR           (1 << 5)    /* Read overrun */
#define SPI_SR_WCOL           (1 << 6)    /* Write collision */
#define SPI_SR_SPIF           (1 << 7)    /* SPI transfer complete flag */

/*----------------------------------------------------------------------------*
  SPI Data Register (S0SPDR)
 *----------------------------------------------------------------------------*/

#define SPI_DR_DATALOW_MASK   (0x00ff)
#define SPI_DR_DATAHIGH_MASK  (0xff00)

/*----------------------------------------------------------------------------*
  SPI Clock Counter Register (S0SPCCR)
 *----------------------------------------------------------------------------*/

#define SPI_CCR_COUNTER_MASK   (0x00ff)       /* must be even and >= 8. */
                         /* Also depends on the peripheral clock chosen */

/*----------------------------------------------------------------------------*
  SPI Interrupt Register (S0SPINT)
 *----------------------------------------------------------------------------*/

#define SPI_INT_SPIF         (1 << 0)   /* SPI interrupt flag */

#endif
