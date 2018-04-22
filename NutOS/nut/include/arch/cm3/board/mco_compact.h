#ifndef _BOARD_MCO_COMPACT_H_
#define _BOARD_MCO_COMPACT_H_

/*
 * Copyright (C) 2014 by Ole Reinhardt (ole.reinhardt@kernelconcepts.de)
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
 * \file arch/cm3/board/mco_compact.h
 * \brief mco_compact board specific settings.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */


/* define RTC */

#ifndef RTC_CHIP
#include <dev/lpc17xx_rtc.h>
#define RTC_CHIP rtcLpc17xx
#endif

/* USARTs */

#if !defined(DEV_UART)
#include <dev/usart_lpc17xx.h>
#define DEV_UART        DEV_UART0
#define DEV_UART_NAME   DEV_UART0_NAME
#endif

#ifndef DEV_DEBUG
#define DEV_DEBUG       devDebug3
#endif

#ifndef DEV_DEBUG_NAME
#define DEV_DEBUG_NAME  "uart3"
#endif

/* define MMC interface */

#ifndef DEV_MMCARD0
#include <dev/lpc177x_8x_mci.h>
#define DEV_MMCARD0         devLpcMci0
#define DEV_MMCARD0_NAME    "MMC0"
#endif

/* Ethernet interface */

#include <dev/lpc17xx_emac.h>
#ifndef DEV_ETHER_NAME
#define DEV_ETHER_NAME  "eth0"
#endif

/* SPI Bus interface */

#include <dev/spibus_lpc17xx_ssp.h>
#ifndef DEV_SPIBUS0
#define DEV_SPIBUS0     spiBus0Lpc17xxSsp
#endif

#endif /* _BOARD_MCO_COMPACT_H_ */
