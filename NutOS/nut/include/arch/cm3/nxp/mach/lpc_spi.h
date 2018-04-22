#ifndef _ARCH_CM3_NXP_MACH_LPC_SPI_H_
#define _ARCH_CM3_NXP_MACH_LPC_SPI_H_

/*
 * Copyright 2011 by egnite GmbH
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
 * \file arch/cm3/nxp/mach/lpc_spi.h
 * \brief LPC SPI definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmv7mLpcSpi
 */
/*@{*/


/*! \name SPI Control Register */
/*@{*/
#define SPCR_OFF        0x00000000
#define SPCR            (LPC_SPI_BASE + SPCR_OFF)
#define SPCR_EN         (1 << 2)
#define SPCR_CPHA       (1 << 3)
#define SPCR_CPOL       (1 << 4)
#define SPCR_MSTR       (1 << 5)
#define SPCR_LSBF       (1 << 6)
#define SPCR_SPIE       (1 << 7)
#define SPCR_BITS_LSB   8
#define SPCR_BITS       0x00000F00
#define SPCR_8BITS      0x00000800
#define SPCR_9BITS      0x00000900
#define SPCR_10BITS     0x00000A00
#define SPCR_11BITS     0x00000B00
#define SPCR_12BITS     0x00000C00
#define SPCR_13BITS     0x00000D00
#define SPCR_14BITS     0x00000E00
#define SPCR_15BITS     0x00000F00
#define SPCR_16BITS     0x00000000
/*@}*/

/*! \name SPI Status Register */
/*@{*/
#define SPSR_OFF        0x00000004
#define SPSR            (LPC_SPI_BASE + SPSR_OFF)
#define SPSR_ABRT       (1 << 3)
#define SPSR_MODF       (1 << 4)
#define SPSR_ROVR       (1 << 5)
#define SPSR_WCOL       (1 << 6)
#define SPSR_SPIF       (1 << 7)
/*@}*/

/*! \name SPI Data Register */
/*@{*/
#define SPDR_OFF        0x00000008
#define SPDR            (LPC_SPI_BASE + SPDR_OFF)
/*@}*/

/*! \name SPI Clock Counter Register */
/*@{*/
#define SPCCR_OFF       0x0000000C
#define SPCCR           (LPC_SPI_BASE + SPCCR_OFF)
/*@}*/

/*! \name SPI Interrupt Flag Register */
/*@{*/
#define SPINT_OFF       0x0000001C
#define SPINT           (LPC_SPI_BASE + SPINT_OFF)
#define SPINT_SPIF      (1 << 0)
/*@}*/


/*@}*/
#endif

