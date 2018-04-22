/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/usart.h>

#if defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#else
#warning "Unknown LPC familiy"
#endif

#include <arch/cm3/nxp/lpc17xx_usart.h>

/*!
 * \addtogroup xgNutArchCm3Lpc177x_8xUsart
 */
/*@{*/

/*----------------------------------------------------------------------------*
  Public functions
 *----------------------------------------------------------------------------*/

static uint32_t Lpc17xxUsartGetSpeed(void);
static int Lpc17xxUsartSetSpeed(uint32_t baudrate);
static uint8_t Lpc17xxUsartGetDataBits(void);
static int Lpc17xxUsartSetDataBits(uint8_t bits);
static uint8_t Lpc17xxUsartGetParity(void);
static int Lpc17xxUsartSetParity(uint8_t mode);
static uint8_t Lpc17xxUsartGetStopBits(void);
static int Lpc17xxUsartSetStopBits(uint8_t bits);
static uint32_t Lpc17xxUsartGetFlowControl(void);
static int Lpc17xxUsartSetFlowControl(uint32_t flags);
static uint32_t Lpc17xxUsartGetStatus(void);
static int Lpc17xxUsartSetStatus(uint32_t flags);
static uint8_t Lpc17xxUsartGetClockMode(void);
static int Lpc17xxUsartSetClockMode(uint8_t mode);
static void Lpc17xxUsartTxStart(void);
static void Lpc17xxUsartRxStart(void);
static int Lpc17xxUsartInit(void);
static int Lpc17xxUsartDeinit(void);

/*
 * USART2 device control block structure.
 */
static USARTDCB dcb_usart2 = {
    0,                          /* dcb_modeflags */
    0,                          /* dcb_statusflags */
    0,                          /* dcb_rtimeout */
    0,                          /* dcb_wtimeout */
    { 0,0,0,0,0,0,0,0,0},       /* dcb_tx_rbf */
    { 0,0,0,0,0,0,0,0,0},       /* dcb_rx_rbf */
    0,                          /* dbc_last_eol */
    Lpc17xxUsartInit,           /* dcb_init */
    Lpc17xxUsartDeinit,         /* dcb_deinit */
    Lpc17xxUsartTxStart,        /* dcb_tx_start */
    Lpc17xxUsartRxStart,        /* dcb_rx_start */
    Lpc17xxUsartSetFlowControl, /* dcb_set_flow_control */
    Lpc17xxUsartGetFlowControl, /* dcb_get_flow_control */
    Lpc17xxUsartSetSpeed,       /* dcb_set_speed */
    Lpc17xxUsartGetSpeed,       /* dcb_get_speed */
    Lpc17xxUsartSetDataBits,    /* dcb_set_data_bits */
    Lpc17xxUsartGetDataBits,    /* dcb_get_data_bits */
    Lpc17xxUsartSetParity,      /* dcb_set_parity */
    Lpc17xxUsartGetParity,      /* dcb_get_parity */
    Lpc17xxUsartSetStopBits,    /* dcb_set_stop_bits */
    Lpc17xxUsartGetStopBits,    /* dcb_get_stop_bits */
    Lpc17xxUsartSetStatus,      /* dcb_set_status */
    Lpc17xxUsartGetStatus,      /* dcb_get_status */
    Lpc17xxUsartSetClockMode,   /* dcb_set_clock_mode */
    Lpc17xxUsartGetClockMode,   /* dcb_get_clock_mode */
};

/*!
 * \name LPC17xx USART2 Device
 */
/*@{*/

/*!
 * \brief USART2 device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the LPC17xx's on-chip USART2.
 *
 * The device is named usart2.
 *
 * \showinitializer
 */
NUTDEVICE devUsartLpc17xx_2 = {
    0,                          /* Pointer to next device, dev_next. */
    {'u', 's', 'a', 'r', 't', '2', 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    LPC_UART2_BASE,             /* Base address, dev_base. */
    UART2_IRQn,                 /* First interrupt number, dev_irq. */
    NULL,                       /* Interface control block, dev_icb. */
    &dcb_usart2,                /* Driver control block, dev_dcb. */
    UsartInit,                  /* Driver initialization routine, dev_init. */
    UsartIOCtl,                 /* Driver specific control function, dev_ioctl. */
    UsartRead,                  /* Read from device, dev_read. */
    UsartWrite,                 /* Write to device, dev_write. */
    UsartOpen,                  /* Open a device or file, dev_open. */
    UsartClose,                 /* Close a device or file, dev_close. */
    UsartSize,                  /* Request file size, dev_size. */
    UsartSelect,                /* Select function, dev_select. */
};

/*@}*/

/*!
 * \brief USART2 GPIO configuartion and assignment.
 */

#define TX_GPIO_PORT    NUTGPIO_PORT0
#define TX_GPIO_PIN     10
#define TX_GPIO_PIN_CFG GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHERAL1
#define RX_GPIO_PORT    NUTGPIO_PORT0
#define RX_GPIO_PIN     11
#define RX_GPIO_PIN_CFG GPIO_CFG_INPUT | GPIO_CFG_PERIPHERAL1

/*!
 * \brief USART2 base configuration.
 */

#ifdef USART2_INIT_BAUDRATE
#define USART_INIT_BAUTRATE USART2_INIT_BAUDRATE
#endif


/* USART2 does not support hardware handshake */
#define USART_HWFLOWCTRL USART_HardwareFlowControl_None
#undef  US_MODE_HWHANDSHAKE

#ifdef USART2_XONXOFF_CONTROL
#define USART_XONXOFF_CONTROL
#else
#undef  USART_XONXOFF_CONTROL
#endif

#ifdef USART2_MODE_IRDA
#define USART_MODE_IRDA
#else
#undef  USART_MODE_IRDA
#endif

/* TODO: DMA Support */
/*
#ifdef USART2_SUPPORT_DMA
#define UART_DMA_TXCHANNEL  DMA1_C4
#define UART_DMA_RXCHANNEL  DMA1_C5
#define UART_DMA_TXIRQ      sig_DMA1_CH4
#define UART_DMA_RXIRQ      sig_DMA1_CH5
#else
#undef UART_DMA_TXCHANNEL
#undef UART_DMA_RXCHANNEL
#endif
*/

#define USARTn      LPC_UART2
#define USARTnBase  LPC_UART2_BASE
#define USARTirqn   UART2_IRQn

#define SigUSART    sig_USART2
#define DcbUSART    dcb_usart2

/*@}*/
#include "lpc17xx_usart.c"
