/*
 * Copyright (C) 2013 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 * $Id: $
 * \endverbatim
 */

#include <cfg/devices.h>
#include <cfg/arch.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>

#ifndef NUT_IRQPRI_DEF
#define NUT_IRQPRI_DEF  4
#endif

CREATE_HANDLER(WDT,           WDT,        NUT_IRQPRI_DEF);    /* Watchdog Timer */

CREATE_HANDLER(USART0,        UART0,      NUT_IRQPRI_DEF);    /* USART 0 */
CREATE_HANDLER(USART1,        UART1,      NUT_IRQPRI_DEF);    /* USART 1 */
CREATE_HANDLER(USART2,        UART2,      NUT_IRQPRI_DEF);    /* USART 2 */
CREATE_HANDLER(USART3,        UART3,      NUT_IRQPRI_DEF);    /* USART 3 */

#if defined (HW_UART4_LPC17xx)
CREATE_HANDLER(USART4,        UART4,      NUT_IRQPRI_DEF);    /* USART 4 */
#endif 

CREATE_HANDLER(I2C0,          I2C0,       NUT_IRQPRI_DEF);    /* I2C Channel 0 */
CREATE_HANDLER(I2C1,          I2C1,       NUT_IRQPRI_DEF);    /* I2C Channel 1 */
CREATE_HANDLER(I2C2,          I2C2,       NUT_IRQPRI_DEF);    /* I2C Channel 2 */

CREATE_HANDLER(RTC,           RTC,        NUT_IRQPRI_DEF);    /* Real Time Clock */

#if defined (HW_MCI_LPC177x_8x)
CREATE_HANDLER(MCI,           MCI,        NUT_IRQPRI_DEF);    /* Multimedia Card Interface / SDIO Interface */
#endif

#if defined (HW_EMAC_LPC17xx)
CREATE_HANDLER(EMAC,          ENET,       NUT_IRQPRI_DEF);    /* Ethernet MAC */
#endif

CREATE_HANDLER(GPIO,          GPIO,       NUT_IRQPRI_DEF);    /* GPIO */
CREATE_HANDLER(DMA,           DMA,        NUT_IRQPRI_DEF);    /* General Purpose DMA Controller */

CREATE_HANDLER(EINT0,         EINT0,      NUT_IRQPRI_DEF);    /* General Purpose DMA Controller */
CREATE_HANDLER(EINT1,         EINT1,      NUT_IRQPRI_DEF);    /* General Purpose DMA Controller */
CREATE_HANDLER(EINT2,         EINT2,      NUT_IRQPRI_DEF);    /* General Purpose DMA Controller */
CREATE_HANDLER(EINT3,         EINT3,      NUT_IRQPRI_DEF);    /* General Purpose DMA Controller */


