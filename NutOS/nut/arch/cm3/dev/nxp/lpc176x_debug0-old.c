/*
 * Copyright (C) 2011-2012 by egnite GmbH
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

#include <cfg/uart.h>
#include <dev/debug.h>
#include <sys/timer.h>

#include <arch/cm3/nxp/mach/lpc1700.h>
#include <arch/cm3/nxp/lpc_debug.h>

/*!
 * \addtogroup xgDevDebugLpc
 */
/*@{*/

static int Debug0Init(NUTDEVICE * dev)
{
    /* Set UART PCLK divider to 2 and determine the clock rate. */
    mem_wr32(SC_PCLKSEL0, (mem_rd32(SC_PCLKSEL0) & ~SC_PCLK_UART0) | (SC_PCLK_DIV4 << SC_PCLK_UART0_LSB));

    mem_wr32(UART0_LCR, mem_rd32(UART0_LCR) | UART_DLAB);
    mem_wr32_mb(UART0_DLM, 0);
    mem_wr32(UART0_DLL, 7);
    mem_wr32_mb(UART0_LCR, (mem_rd32(UART0_LCR) & ~UART_DLAB) | UART_WLEN_8);

    mem_wr32(UART0_FCR, UART_FIFO_TXRST | UART_FIFO_RXRST | UART_FIFO_EN);

    mem_wr32(UART0_FDR, 237);
    /* Enable peripheral pins. */
    mem_wr32(PINSEL(0), mem_rd32(PINSEL(0)) | PS0_P0_2_TXD0 | PS0_P0_3_RXD0);

    return 0;
}

static NUTFILE dbg0file;

NUTDEVICE devDebug0 = {
    NULL,               /*!< Pointer to next device, dev_next. */
    {'u', 'a', 'r', 't', '0', 0, 0, 0, 0}
    ,                   /*!< Unique device name, dev_name. */
    0,                  /*!< Type of device, dev_type. */
    LPC_UART0_BASE,     /*!< Base address, dev_base. */
    0,                  /*!< First interrupt number, dev_irq. */
    NULL,               /*!< Interface control block, dev_icb. */
    &dbg0file,          /*!< Driver control block, dev_dcb. */
    Debug0Init,         /*!< Driver initialization routine, dev_init. */
    LpcDevDebugIOCtl,   /*!< Driver specific control function, dev_ioctl. */
#ifdef NUT_DEV_DEBUG_READ
    LpcDevDebugRead,   /*!< dev_read. */
#else
    NULL,               /*!< dev_read. */
#endif
    LpcDevDebugWrite,   /*!< dev_write. */
    LpcDevDebugOpen,    /*!< dev_open. */
    LpcDevDebugClose,   /*!< dev_close. */
#ifdef NUT_DEV_DEBUG_READ
    LpcDevDebugSize,    /*!< dev_size. */
#else
    NULL,               /*!< dev_size. */
#endif
    NULL,               /*!< dev_select, optional, not yet implemented. */
};

/*@}*/
