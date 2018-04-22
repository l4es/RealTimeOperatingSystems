#ifndef ARCH_AVR_USART_AVRCTS_H_
#define ARCH_AVR_USART_AVRCTS_H_

/*
 * Copyright (C) 2013 by egnite GmbH
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

/*!
 * \defgroup xgUsartCtsAvr USART Driver CTS Resolver for AVR
 * \ingroup xgUsartCb
 */
/*@{*/

#undef UART_CTS_SIGNAL
#undef UART_CTS_BIT
#undef UART_CTS_PORT
#undef UART_CTS_PIN
#undef UART_CTS_DDR

#include <dev/irqreg.h>

#ifndef INT0
#define INT0    0
#endif
#ifndef INT1
#define INT1    1
#endif
#ifndef INT2
#define INT2    2
#endif
#ifndef INT3
#define INT3    3
#endif
#ifndef INT4
#define INT4    4
#endif
#ifndef INT5
#define INT5    5
#endif
#ifndef INT6
#define INT6    6
#endif
#ifndef INT7
#define INT7    7
#endif

#if (UART_CTS_IRQ == INT0)
#define UART_CTS_SIGNAL sig_INTERRUPT0
#define UART_CTS_BIT    0
#define UART_CTS_PORT   PORTD
#define UART_CTS_PIN    PIND
#define UART_CTS_DDR    DDRD

#elif (UART_CTS_IRQ == INT1)
#define UART_CTS_SIGNAL sig_INTERRUPT1
#define UART_CTS_BIT    1
#define UART_CTS_PORT   PORTD
#define UART_CTS_PIN    PIND
#define UART_CTS_DDR    DDRD

#elif (UART_CTS_IRQ == INT2)
#define UART_CTS_SIGNAL sig_INTERRUPT2
#define UART_CTS_BIT    2
#define UART_CTS_PORT   PORTD
#define UART_CTS_PIN    PIND
#define UART_CTS_DDR    DDRD

#elif (UART_CTS_IRQ == INT3)
#define UART_CTS_SIGNAL sig_INTERRUPT3
#define UART_CTS_BIT    3
#define UART_CTS_PORT   PORTD
#define UART_CTS_PIN    PIND
#define UART_CTS_DDR    DDRD

#elif (UART_CTS_IRQ == INT4)
#define UART_CTS_SIGNAL sig_INTERRUPT4
#define UART_CTS_BIT    4
#define UART_CTS_PORT   PORTE
#define UART_CTS_PIN    PINE
#define UART_CTS_DDR    DDRE

#elif (UART_CTS_IRQ == INT5)
#define UART_CTS_SIGNAL sig_INTERRUPT5
#define UART_CTS_BIT    5
#define UART_CTS_PORT   PORTE
#define UART_CTS_PIN    PINE
#define UART_CTS_DDR    DDRE

#elif (UART_CTS_IRQ == INT6)
#define UART_CTS_SIGNAL sig_INTERRUPT6
#define UART_CTS_BIT    6
#define UART_CTS_PORT   PORTE
#define UART_CTS_PIN    PINE
#define UART_CTS_DDR    DDRE

#elif (UART_CTS_IRQ == INT7)
#define UART_CTS_SIGNAL sig_INTERRUPT7
#define UART_CTS_BIT    7
#define UART_CTS_PORT   PORTE
#define UART_CTS_PIN    PINE
#define UART_CTS_DDR    DDRE

#endif

/*@}*/

#endif
