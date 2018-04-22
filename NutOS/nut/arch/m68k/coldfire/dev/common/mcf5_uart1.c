/*
 * Copyright 2012 by Embedded Technologies s.r.o
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

#include <cfg/arch.h>
#include <cfg/uart.h>
#include <arch/m68k.h>
#include <dev/usart.h>

#define BASE    1

/*
 * Function prototypes
 */
static uint32_t Mcf5UsartGetSpeed(void);
static int Mcf5UsartSetSpeed(uint32_t rate);
static uint8_t Mcf5UsartGetDataBits(void);
static int Mcf5UsartSetDataBits(uint8_t bits);
static uint8_t Mcf5UsartGetParity(void);
static int Mcf5UsartSetParity(uint8_t mode);
static uint8_t Mcf5UsartGetStopBits(void);
static int Mcf5UsartSetStopBits(uint8_t bits);
static uint32_t Mcf5UsartGetFlowControl(void);
static int Mcf5UsartSetFlowControl(uint32_t flags);
static uint32_t Mcf5UsartGetStatus(void);
static int Mcf5UsartSetStatus(uint32_t flags);
static void Mcf5UsartTxStart(void);
static void Mcf5UsartRxStart(void);
static int Mcf5UsartInit(void);
static int Mcf5UsartDeinit(void);

/*
 * \brief UART1 control structure used for write only registers.
 */
static UARTWREGS reg_uart1;

/*!
 * \brief UART1 device control block structure.
 */
static USARTDCB dcb_uart1 = {
    0,                          /* dcb_modeflags */
    0,                          /* dcb_statusflags */
    0,                          /* dcb_rtimeout */
    0,                          /* dcb_wtimeout */
    {0, 0, 0, 0, 0, 0, 0, 0},   /* dcb_tx_rbf */
    {0, 0, 0, 0, 0, 0, 0, 0},   /* dcb_rx_rbf */
    0,                          /* dbc_last_eol */
    Mcf5UsartInit,              /* dcb_init */
    Mcf5UsartDeinit,            /* dcb_deinit */
    Mcf5UsartTxStart,           /* dcb_tx_start */
    Mcf5UsartRxStart,           /* dcb_rx_start */
    Mcf5UsartSetFlowControl,    /* dcb_set_flow_control */
    Mcf5UsartGetFlowControl,    /* dcb_get_flow_control */
    Mcf5UsartSetSpeed,          /* dcb_set_speed */
    Mcf5UsartGetSpeed,          /* dcb_get_speed */
    Mcf5UsartSetDataBits,       /* dcb_set_data_bits */
    Mcf5UsartGetDataBits,       /* dcb_get_data_bits */
    Mcf5UsartSetParity,         /* dcb_set_parity */
    Mcf5UsartGetParity,         /* dcb_get_parity */
    Mcf5UsartSetStopBits,       /* dcb_set_stop_bits */
    Mcf5UsartGetStopBits,       /* dcb_get_stop_bits */
    Mcf5UsartSetStatus,         /* dcb_set_status */
    Mcf5UsartGetStatus,         /* dcb_get_status */
    0,                          /* dcb_set_clock_mode */
    0,                          /* dcb_get_clock_mode */
};

/*!
 * \name Coldfire UART1 Device
 */
/*@{*/
/*!
 * \brief Coldfire device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the Coldfire's on-chip UART1.
 *
 * The device is named \b uart1.
 *
 * \showinitializer
 */
NUTDEVICE devUartMcf5_1 = {
    0,                          /* Pointer to next device, dev_next. */
    {'u', 'a', 'r', 't', '1', 0, 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    BASE,                       /* Base address, dev_base. */
    0,                          /* First interrupt number, dev_irq (not used). */
    0,                          /* Interface control block, dev_icb (not used). */
    &dcb_uart1,                 /* Driver control block, dev_dcb. */
    UsartInit,                  /* Driver initialization routine, dev_init. */
    UsartIOCtl,                 /* Driver specific control function, dev_ioctl. */
    UsartRead,                  /* Read from device, dev_read. */
    UsartWrite,                 /* Write to device, dev_write. */
    UsartOpen,                  /* Open a device or file, dev_open. */
    UsartClose,                 /* Close a device or file, dev_close. */
    UsartSize,                  /* Request file size, dev_size. */
    UsartSelect,                /* Select function, dev_select */
};

/*@}*/

/*
 * Peripheral GPIO Configuration
 */
#define TXD_PORT        UART1_TXD_PORT
#define TXD_PIN         UART1_TXD_PIN
#define TXD_PERIPHERAL  UART1_TXD_PERIPHERAL

#define RXD_PORT        UART1_RXD_PORT
#define RXD_PIN         UART1_RXD_PIN
#define RXD_PERIPHERAL  UART1_RXD_PERIPHERAL

#ifdef UART1_RTS_PIN
#define RTS_PORT        UART1_RTS_PORT
#define RTS_PIN         UART1_RTS_PIN
#define RTS_PERIPHERAL  UART1_RTS_PERIPHERAL
#endif

#ifdef UART1_CTS_PIN
#define CTS_PORT        UART1_CTS_PORT
#define CTS_PIN         UART1_CTS_PIN
#define CTS_PERIPHERAL  UART1_CTS_PERIPHERAL
#endif

/*
 * Half/Full Duplex Configuration
 */
#ifdef UART1_HARDWARE_HDX
#define HDX_ENABLED
#endif

/*
 * Software Flow Control
 */
#ifdef UART1_XONXOFF
#define XONXOFF
#endif

/*
 * Half Duplex Control
 */
#ifdef UART1_HDX_CTRL
#define HDX_CTRL
#define HDX_CTRL_BOARD_SPEC     UART1_HDX_CTRL_BOARD_SPEC
#ifdef UART1_HDX_CTRL_INV
#define HDX_CTRL_INV
#endif
#if (defined UART1_HDX_CTRL_PIN) || (defined UART1_HDX_CTRL_PORT)
#define HDX_CTRL_PORT           UART1_HDX_CTRL_PORT
#define HDX_CTRL_PIN            UART1_HDX_CTRL_PIN
#endif
#endif

/*
 * RS485 DE/RE Control
 */
#ifdef UART1_RS485_CTRL
#define RS485_CTRL
#define RS485_CTRL_BOARD_SPEC   UART1_RS485_CTRL_BOARD_SPEC
#ifdef UART1_RS485_CTRL_DE_INV
#define RS485_CTRL_DE_INV
#endif
#if (defined UART1_RS485_CTRL_DE_PIN) || (defined UART1_RS485_CTRL_DE_PORT)
#define RS485_CTRL_DE_PORT      UART1_RS485_CTRL_DE_PORT
#define RS485_CTRL_DE_PIN       UART1_RS485_CTRL_DE_PIN
#endif
#ifdef UART1_RS485_CTRL_RE_INV
#define RS485_CTRL_RE_INV
#endif
#if (defined UART1_RS485_CTRL_RE_PIN) || (defined UART1_RS485_CTRL_RE_PORT)
#define RS485_CTRL_RE_PORT      UART1_RS485_CTRL_RE_PORT
#define RS485_CTRL_RE_PIN       UART1_RS485_CTRL_RE_PIN
#endif
#endif

/*
 * Global Variables
 */
#define sig_uart    sig_UART1
#define dcb_uart    dcb_uart1
#define reg_uart    reg_uart1

#include "mcf5_uart.c"
