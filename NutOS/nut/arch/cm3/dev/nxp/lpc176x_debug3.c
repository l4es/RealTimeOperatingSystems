/*
 * Copyright (C) 2013 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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

#include <cfg/arch.h>
#include <cfg/uart.h>
#include <dev/debug.h>
#include <sys/timer.h>
#include <sys/file.h>
#include <sys/device.h>

#include <arch/cm3/nxp/lpc176x.h>
#include <arch/cm3/nxp/lpc176x_clk.h>
#include <arch/cm3/nxp/lpc17xx_usart.h>

/*!
 * \addtogroup xgNutArchCm3Lpc177x_8xDebug
 */
/*@{*/

static NUTFILE *Lpc17xxDevDebugOpen(NUTDEVICE * dev, const char *name, int mode, int acc);
static int Lpc17xxDevDebugClose(NUTFILE * fp);
static int Lpc17xxDevDebugWrite(NUTFILE * fp, const void *buffer, int len);
#ifdef NUT_DEV_DEBUG_READ
static int Lpc17xxDevDebugRead(NUTFILE * fp, void *buffer, int size);
static long Lpc17xxDevDebugSize(NUTFILE *fp);
#endif
static int Lpc17xxDevDebugIOCtl(NUTDEVICE * dev, int req, void *conf);
static int Lpc17xxDevDebugInit(NUTDEVICE * dev);

/*!
 * \name LPC17xx USART3 Device
 */
/*@{*/

static NUTFILE dbg3file;

/*!
 * \brief Debug UART 3 device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the LPC17xx's on-chip USART3.
 *
 * The device is named usart3.
 *
 * \showinitializer
 */

NUTDEVICE devDebug3 = {
    NULL,                       /*!< Pointer to next device, dev_next. */
    {'u', 'a', 'r', 't', '3', 0, 0, 0, 0}
    ,                           /*!< Unique device name, dev_name. */
    0,                          /*!< Type of device, dev_type. */
    LPC_UART3_BASE,             /*!< Base address, dev_base. */
    0,                          /*!< First interrupt number, dev_irq. */
    NULL,                       /*!< Interface control block, dev_icb. */
    &dbg3file,                  /*!< Driver control block, dev_dcb. */
    Lpc17xxDevDebugInit,        /*!< Driver initialization routine, dev_init. */
    Lpc17xxDevDebugIOCtl,       /*!< Driver specific control function, dev_ioctl. */
#ifdef NUT_DEV_DEBUG_READ
    Lpc17xxDevDebugRead,        /*!< dev_read. */
#else
    NULL,                       /*!< dev_read. */
#endif
    Lpc17xxDevDebugWrite,       /*!< dev_write. */
    Lpc17xxDevDebugOpen,        /*!< dev_open. */
    Lpc17xxDevDebugClose,       /*!< dev_close. */
#ifdef NUT_DEV_DEBUG_READ
    Lpc17xxDevDebugSize,        /*!< dev_size. */
#else
    NULL,                       /*!< dev_size. */
#endif
    NULL,                       /*!< dev_select, optional, not yet implemented */
};

/*@}*/

/*!
 * \brief Debug UART0 GPIO configuartion and assignment.
 */

#define TX_GPIO_PORT    NUTGPIO_PORT0
#define TX_GPIO_PIN     0
#define TX_GPIO_PIN_CFG GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHERAL2
#define RX_GPIO_PORT    NUTGPIO_PORT0
#define RX_GPIO_PIN     1
#define RX_GPIO_PIN_CFG GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL2

/* Alternatively:
#define TX_GPIO_PORT    NUTGPIO_PORT0
#define TX_GPIO_PIN     25
#define TX_GPIO_PIN_CFG GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHERAL3
#define RX_GPIO_PORT    NUTGPIO_PORT0
#define RX_GPIO_PIN     26
#define RX_GPIO_PIN_CFG GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL3

or:

#define TX_GPIO_PORT    NUTGPIO_PORT4
#define TX_GPIO_PIN     28
#define TX_GPIO_PIN_CFG GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHERAL3
#define RX_GPIO_PORT    NUTGPIO_PORT4
#define RX_GPIO_PIN     29
#define RX_GPIO_PIN_CFG GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL3

*/

/*!
 * \brief Debug UART0 base configuration.
 */

#define USARTn      LPC_UART3
#define USARTnBase  LPC_UART3_BASE

/*@}*/
#include "lpc17xx_debug.c"
