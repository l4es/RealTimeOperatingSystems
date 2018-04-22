#ifndef ARCH_AVR_USART_CB_H_
#define ARCH_AVR_USART_CB_H_

/*
 * Copyright (C) 2012-2013 by egnite GmbH
 * Copyright (C) 2001-2003 by egnite Software GmbH
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

#include <sys/device.h>

#include <dev/usart_cb.h>

/*!
 * \defgroup xgUsartCbAvr AVR Low Level USART Driver
 * \ingroup xgUsartCb
 */
/*@{*/

/*!
 * \brief UART interrupt handler for AVR.
 */
extern void AvrUsartCbInterrupt(void *arg);

/*!
 * \brief UART hardware handshake interrupt handler for AVR.
 */
extern void AvrCtsInterrupt(void *arg);

/*!
 * \brief Start AVR UART receiver.
 */
extern void AvrUsartCbRxStart(USARTCB_DCB *dcb);

/*!
 * \brief Start AVR UART transmitter.
 */
extern void AvrUsartCbTxStart(USARTCB_DCB *dcb);

/*!
 * \brief UART0 device structure for AVR.
 */
extern NUTDEVICE devUsart0CbAvr;

/*!
 * \brief UART1 device structure for AVR.
 */
extern NUTDEVICE devUsart1CbAvr;

/*@}*/

#endif

