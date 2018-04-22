#ifndef _ARCH_CM3_NXP_MACH_LPC_GPIO_H_
#define _ARCH_CM3_NXP_MACH_LPC_GPIO_H_

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
 * \file arch/cm3/nxp/mach/lpc_gpio.h
 * \brief LPC GPIO definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmv7mLpcGpio
 */
/*@{*/


/*! \name Fast GPIO Port Register Offsets */
/*@{*/
#define FIO_DIR_OFF         0x00000000  /*!< Direction control register offset. */
#define FIO_MASK_OFF        0x00000010  /*!< Mask register offset. */
#define FIO_PIN_OFF         0x00000014  /*!< Pin value register offset. */
#define FIO_SET_OFF         0x00000018  /*!< Set register offset. */
#define FIO_CLR_OFF         0x0000001C  /*!< Clear register offset. */
/*@{*/

/*! \name GPIO0 Register Addresses */
/*@{*/
#ifdef LPC_GPIO0_BASE
#define FIO0DIR             (LPC_GPIO0_BASE + FIO_DIR_OFF)
#define FIO0MASK            (LPC_GPIO0_BASE + FIO_MASK_OFF)
#define FIO0PIN             (LPC_GPIO0_BASE + FIO_PIN_OFF)
#define FIO0SET             (LPC_GPIO0_BASE + FIO_SET_OFF)
#define FIO0CLR             (LPC_GPIO0_BASE + FIO_CLR_OFF)
#endif
/*@}*/

/*! \name GPIO1 Register Addresses */
/*@{*/
#ifdef LPC_GPIO1_BASE
#define FIO1DIR             (LPC_GPIO1_BASE + FIO_DIR_OFF)
#define FIO1MASK            (LPC_GPIO1_BASE + FIO_MASK_OFF)
#define FIO1PIN             (LPC_GPIO1_BASE + FIO_PIN_OFF)
#define FIO1SET             (LPC_GPIO1_BASE + FIO_SET_OFF)
#define FIO1CLR             (LPC_GPIO1_BASE + FIO_CLR_OFF)
#endif
/*@}*/

/*! \name GPIO2 Register Addresses */
/*@{*/
#ifdef LPC_GPIO2_BASE
#define FIO2DIR             (LPC_GPIO2_BASE + FIO_DIR_OFF)
#define FIO2MASK            (LPC_GPIO2_BASE + FIO_MASK_OFF)
#define FIO2PIN             (LPC_GPIO2_BASE + FIO_PIN_OFF)
#define FIO2SET             (LPC_GPIO2_BASE + FIO_SET_OFF)
#define FIO2CLR             (LPC_GPIO2_BASE + FIO_CLR_OFF)
#endif
/*@}*/

/*! \name GPIO3 Register Addresses */
/*@{*/
#ifdef LPC_GPIO3_BASE
#define FIO3DIR             (LPC_GPIO3_BASE + FIO_DIR_OFF)
#define FIO3MASK            (LPC_GPIO3_BASE + FIO_MASK_OFF)
#define FIO3PIN             (LPC_GPIO3_BASE + FIO_PIN_OFF)
#define FIO3SET             (LPC_GPIO3_BASE + FIO_SET_OFF)
#define FIO3CLR             (LPC_GPIO3_BASE + FIO_CLR_OFF)
#endif
/*@}*/

/*! \name GPIO4 Register Addresses */
/*@{*/
#ifdef LPC_GPIO4_BASE
#define FIO4DIR             (LPC_GPIO4_BASE + FIO_DIR_OFF)
#define FIO4MASK            (LPC_GPIO4_BASE + FIO_MASK_OFF)
#define FIO4PIN             (LPC_GPIO4_BASE + FIO_PIN_OFF)
#define FIO4SET             (LPC_GPIO4_BASE + FIO_SET_OFF)
#define FIO4CLR             (LPC_GPIO4_BASE + FIO_CLR_OFF)
#endif
/*@}*/

#ifdef LPC_GPIOINT_BASE
#define GPIO_IER_OFF        0x00000010
#define GPIO_IFR_OFF        0x00000014
#define GPIO_ISR_OFF        0x00000004
#define GPIO_ISF_OFF        0x00000008
#define GPIO_IC_OFF         0x0000000C
#define GPIO_IS_OFF         0x00000000
#endif

/*@}*/
#endif

