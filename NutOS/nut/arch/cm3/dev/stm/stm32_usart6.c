/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2012 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: stm32_usart6.c 6642 2017-05-16 13:01:24Z u_bonnes $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <cfg/uart.h>
#include <cfg/arch/gpio.h>

#include <string.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/usart.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_usart_pinmux.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_uart.h>
#include <arch/cm3/stm/stm32_usart.h>

/*!
 * \addtogroup xgNutArchArmStm32Us
 */
/*@{*/

/*!
 * \brief USART6 device control block structure.
 */
static USARTDCB dcb_usart6 = {
    0,                          /* dcb_modeflags */
    0,                          /* dcb_statusflags */
    0,                          /* dcb_rtimeout */
    0,                          /* dcb_wtimeout */
    { 0,0,0,0,0,0,0,0,0},       /* dcb_tx_rbf */
    { 0,0,0,0,0,0,0,0,0},       /* dcb_rx_rbf */
    0,                          /* dbc_last_eol */
    Stm32UsartInit,             /* dcb_init */
    Stm32UsartDeinit,           /* dcb_deinit */
    Stm32UsartTxStart,          /* dcb_tx_start */
    Stm32UsartRxStart,          /* dcb_rx_start */
    Stm32UsartSetFlowControl,   /* dcb_set_flow_control */
    Stm32UsartGetFlowControl,   /* dcb_get_flow_control */
    Stm32UsartSetSpeed,         /* dcb_set_speed */
    Stm32UsartGetSpeed,         /* dcb_get_speed */
    Stm32UsartSetDataBits,      /* dcb_set_data_bits */
    Stm32UsartGetDataBits,      /* dcb_get_data_bits */
    Stm32UsartSetParity,        /* dcb_set_parity */
    Stm32UsartGetParity,        /* dcb_get_parity */
    Stm32UsartSetStopBits,      /* dcb_set_stop_bits */
    Stm32UsartGetStopBits,      /* dcb_get_stop_bits */
    Stm32UsartSetStatus,        /* dcb_set_status */
    Stm32UsartGetStatus,        /* dcb_get_status */
    Stm32UsartSetClockMode,     /* dcb_set_clock_mode */
    Stm32UsartGetClockMode,     /* dcb_get_clock_mode */
};

/*!
 * \name STM32 USART6 Device
 */
/*@{*/
/*!
 * \brief USART6 device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the AT91's on-chip USART6.
 *
 * The device is named \b uart6.
 *
 * \showinitializer
 */
NUTDEVICE devUsartStm32_6 = {
    0,                          /* Pointer to next device, dev_next. */
    {'u', 's', 'a', 'r', 't', '6', 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    USART6_BASE,                /* Base address, dev_base. */
    0,                         /* First interrupt number, dev_irq. */
    NULL,                       /* Interface control block, dev_icb. */
    &dcb_usart6,                /* Driver control block, dev_dcb. */
    UsartInit,                  /* Driver initialization routine, dev_init. */
    UsartIOCtl,                 /* Driver specific control function, dev_ioctl. */
    UsartRead,                  /* Read from device, dev_read. */
    UsartWrite,                 /* Write to device, dev_write. */
    UsartOpen,                  /* Open a device or file, dev_open. */
    UsartClose,                 /* Close a device or file, dev_close. */
    UsartSize,                  /* Request file size, dev_size. */
    UsartSelect,                /* Select function, dev_select. */
};

# define  Stm32F1UsartRemap()

#define USART_TX  USART6_TX
#define USART_RX  USART6_RX
#define USART_CTS USART6_CTS
#define USART_RTS USART6_RTS
#define USART_CK  USART6_CK

#define USART_TX_AF  USART6_TX_AF
#define USART_RX_AF  USART6_RX_AF
#define USART_CTS_AF USART6_CTS_AF
#define USART_RTS_AF USART6_RTS_AF
#define USART_CK_AF  USART6_CK_AF

#ifdef USART6_HARDWARE_HANDSHAKE
 #define CTS_GPIO_PORT    NUTGPIO_PORTG
 #if !defined(USART6_CTS_PIN) || USART6_CTS_PIN  == 13
  #define CTS_GPIO_PIN      13
 #elif USART6_RX_PIN == 15
  #define CTS_GPIO_PIN     15
 #else
  #warning "Illegal USART6 CTS pin assignement"
 #endif
 #define RTS_GPIO_PORT    NUTGPIO_PORTG
 #if !defined(USART6_RTS_PIN) || USART6_RTS_PIN == 8
 #define RTS_GPIO_PIN      8
 #elif USART6_RTS_PIN == 12
  #define RTS_GPIO_PIN     12
 #else
  #warning "Illegal USART6 RTS pin assignement"
 #endif
#endif /* USART6_HARDWARE_HANDSHAKE */
#if !defined(USART6_CK_PIN) || USART6_CK_PIN == 8
 #define CK_GPIO_PORT    NUTGPIO_PORTC
 #define CK_GPIO_PIN      8
#elif USART6_CK_PIN == 7
 #define CK_GPIO_PORT    NUTGPIO_PORTG
 #define CK_GPIO_PIN     9
#else
 #warning "Illegal USART6 CK pin assignement"
#endif

#ifdef USART6_RS485_CTRL
#define USART_485_CTRL
#ifdef USART6_485DE_INV
#define USART_485DE_INV
#endif
#ifdef USART6_485RE_INV
#define USART_485RE_INV
#endif
#if defined(USART6_485DE_PORT) && defined(USART6_485DE_PIN)
  #define DE_GPIO_BASE  GPIO_ID2GPIO(USART6_485DE_PORT)
  #define DE_GPIO_PORT  USART6_485DE_PORT
  #define DE_GPIO_PIN   USART6_485DE_PIN
#endif
#if defined(USART6_485RE_PORT) && defined(USART6_485RE_PIN)
#define NRE_GPIO_BASE   GPIO_ID2GPIO(USART6_485RE_PORT)
#define NRE_GPIO_PORT   USART6_485RE_PORT
#define NRE_GPIO_PIN    USART6_485RE_PIN
#endif
#endif /* USART6_RS485_CTRL */

/*!
 * \brief USART6 base configuration.
 */
static void  StmUsartClkEnable(int enable)
{
    if (enable)
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;
    if (!enable)
        RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
}

#ifdef USART6_INIT_BAUDRATE
#define USART_INIT_BAUTRATE USART6_INIT_BAUDRATE
#endif

#ifdef USART6_HARDWARE_HANDSHAKE
  #define USART_HWFLOWCTRL USART_HardwareFlowControl_RTS_CTS
  #define US_MODE_HWHANDSHAKE
#else
  #define USART_HWFLOWCTRL USART_HardwareFlowControl_None
  #undef  US_MODE_HWHANDSHAKE
#endif

#ifdef USART6_XONXOFF_CONTROL
#define USART_XONXOFF_CONTROL
#else
#undef  USART_XONXOFF_CONTROL
#endif

#ifdef USART6_MODE_IRDA
#define USART_MODE_IRDA
#else
#undef  USART_MODE_IRDA
#endif

#ifdef USART6_HARDWARE_HDX
#define USART_HARDWARE_HDX
#else
#undef USART_HARDWARE_HDX
#endif

#ifdef USART6_SUPPORT_DMA
 #if defined(MCU_STM32F1)||defined(MCU_STM32L1)
  #define UART_DMA_TXCHANNEL  DMA1_C4
  #define UART_DMA_RXCHANNEL  DMA1_C5
 #elif  defined(MCU_STM32F2)||defined(MCU_STM32F4)
  #define UART_DMA_TXCHANNEL  USART6_TX_DMA
  #define UART_DMA_RXCHANNEL  USART6_RX_DMA
 #else
  #warning "STM32 family has no implemented DMA"
 #endif
#else
 #undef UART_DMA_TXCHANNEL
 #undef UART_DMA_RXCHANNEL
#endif

#define USARTidx    5
#define USARTn      USART6
#define USARTnBase  USART6_BASE
#define USARTclk    BASE2CLKSRC(USART6_BASE)
#define USARTnSIG   &sig_USART6
#define UART_DR_PTR (uint32_t*)(USARTnBase+4)

#define DcbUSART    dcb_usart6

/*@}*/
#include "stm32_usart.c"
