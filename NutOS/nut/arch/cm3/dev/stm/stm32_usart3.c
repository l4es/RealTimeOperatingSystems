/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2012 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * $Id: stm32_usart3.c 6642 2017-05-16 13:01:24Z u_bonnes $
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
 * \addtogroup xgNutArchCm3Stm32
 */
/*@{*/

/*!
 * \brief USART3 device control block structure.
 */
static USARTDCB dcb_usart3 = {
    0,                          /* dcb_modeflags */
    0,                          /* dcb_statusflags */
    0,                          /* dcb_rtimeout */
    0,                          /* dcb_wtimeout */
    { 0,0,0,0,0,0,0},           /* dcb_tx_rbf */
    { 0,0,0,0,0,0,0},           /* dcb_rx_rbf */
    0,                          /* dcb_last_eol */
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
 * \name STM32 USART3 Device
 */
/*@{*/
/*!
 * \brief USART3 device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the STM32s on-chip USART3.
 *
 * The device is named \b uart3.
 *
 * \showinitializer
 */
NUTDEVICE devUsartStm32_3 = {
    0,                          /* Pointer to next device, dev_next. */
    {'u', 's', 'a', 'r', 't', '3', 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    USART3_BASE,                /* Base address, dev_base. */
    0,                          /* First interrupt number, dev_irq. */
    NULL,                       /* Interface control block, dev_icb. */
    &dcb_usart3,                /* Driver control block, dev_dcb. */
    UsartInit,                  /* Driver initialization routine, dev_init. */
    UsartIOCtl,                 /* Driver specific control function, dev_ioctl. */
    UsartRead,                  /* Read from device, dev_read. */
    UsartWrite,                 /* Write to device, dev_write. */
    UsartOpen,                  /* Open a device or file, dev_open. */
    UsartClose,                 /* Close a device or file, dev_close. */
    UsartSize,                  /* Request file size, dev_size. */
    UsartSelect,                /* Select function, dev_select. */
};

/*!
 * \brief USART3 GPIO configuartion and assignment.
 */
#if defined(MCU_STM32F1)
static void Stm32F1UsartRemap(void)
{
    AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP;
    AFIO->MAPR |= USART3_REMAP * AFIO_MAPR_USART3_REMAP;
}
#else
# define  Stm32F1UsartRemap()
#endif

#define USART_TX  USART3_TX
#define USART_RX  USART3_RX
#define USART_CTS USART3_CTS
#define USART_RTS USART3_RTS
#define USART_CK  USART3_CK

#define USART_TX_AF  USART3_TX_AF
#define USART_RX_AF  USART3_RX_AF
#define USART_CTS_AF USART3_CTS_AF
#define USART_RTS_AF USART3_RTS_AF
#define USART_CK_AF  USART3_CK_AF

#ifdef USART3_RS485_CTRL
#define USART_485_CTRL
#ifdef USART3_485DE_INV
#define USART_485DE_INV
#endif
#ifdef USART3_485RE_INV
#define USART_485RE_INV
#endif
#if defined(USART3_485DE_PORT) && defined(USART3_485DE_PIN)
#define DE_GPIO_BASE  GPIO_ID2GPIO(USART3_485DE_PORT)
#define DE_GPIO_PORT  USART3_485DE_PORT
#define DE_GPIO_PIN   USART3_485DE_PIN
#endif
#if defined(USART3_485RE_PORT) && defined(USART3_485RE_PIN)
#define NRE_GPIO_BASE  GPIO_ID2GPIO(USART3_485RE_PORT)
#define NRE_GPIO_PORT  USART3_485RE_PORT
#define NRE_GPIO_PIN   USART3_485RE_PIN
#endif
#endif /* USART3_RS485_CTRL */

/*!
 * \brief USART3 base configuration.
 */
static void  StmUsartClkEnable(int enable)
{
    if (enable)
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
    if (!enable)
        RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
}

#ifdef USART3_INIT_BAUDRATE
#define USART_INIT_BAUTRATE USART3_INIT_BAUDRATE
#endif

#ifdef USART3_HARDWARE_HANDSHAKE
  #define USART_HWFLOWCTRL USART_HardwareFlowControl_RTS_CTS
  #define US_MODE_HWHANDSHAKE
#else
  #define USART_HWFLOWCTRL USART_HardwareFlowControl_None
  #undef  US_MODE_HWHANDSHAKE
#endif

#ifdef USART3_XONXOFF_CONTROL
#define USART_XONXOFF_CONTROL
#else
#undef  USART_XONXOFF_CONTROL
#endif

#ifdef USART3_MODE_IRDA
#define USART_MODE_IRDA
#else
#undef  USART_MODE_IRDA
#endif

#ifdef USART3_HARDWARE_HDX
#define USART_HARDWARE_HDX
#else
#undef USART_HARDWARE_HDX
#endif

#ifdef USART3_SUPPORT_DMA
 #if defined(MCU_STM32F1)||defined(MCU_STM32L1)
  #define UART_DMA_TXCHANNEL  DMA1_C2
  #define UART_DMA_RXCHANNEL  DMA1_C3
 #elif  defined(MCU_STM32F2)||defined(MCU_STM32F4)
  #define UART_DMA_TXCHANNEL  USART3_TX_DMA
  #define UART_DMA_RXCHANNEL  USART3_TRX_DMA
 #else
  #warning "STM32 family has no implemented DMA"
 #endif
#else
#undef UART_DMA_TXCHANNEL
#undef UART_DMA_RXCHANNEL
#endif

#if defined(USART3_TX_PIN_INV)
#define USART_TX_PIN_INV
#else
#undef USART_TX_PIN_INV
#endif
#if defined(USART3_RX_PIN_INV)
#define USART_RX_PIN_INV
#else
#undef USART_RX_PIN_INV
#endif

#if defined(USART3_SWAP)
#define USART_SWAP
#else
#undef USART_SWAP
#endif

#define USARTidx    2
#define USARTn      USART3
#define USARTnBase  USART3_BASE
#define USARTclk    BASE2CLKSRC(USART3_BASE)
#define USARTnSIG   &sig_USART3
#define UART_DR_PTR (uint32_t*)(USARTnBase+4)

#define DcbUSART dcb_usart3

/*@}*/
#include "stm32_usart.c"
