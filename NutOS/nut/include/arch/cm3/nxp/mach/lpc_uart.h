#ifndef _ARCH_CM3_NXP_MACH_LPC_UART_H_
#define _ARCH_CM3_NXP_MACH_LPC_UART_H_

/*
 * Copyright 2011 by egnite GmbH
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

/*!
 * \file arch/cm3/nxp/mach/lpc_uart.h
 * \brief LPC UART definitions
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmv7mLpcUart
 */
/*@{*/


/*! \name UART Receiver Buffer Registers */
/*@{*/
#define UART_RBR_OFF        0x00000000
/*@}*/

/*! \name UART Transmit Holding Registers */
/*@{*/
#define UART_THR_OFF        0x00000000
/*@}*/

/*! \name UART Divisor Latch LSB Registers */
/*@{*/
#define UART_DLL_OFF        0x00000000
/*@}*/

/*! \name UART Divisor Latch MSB Registers */
/*@{*/
#define UART_DLM_OFF        0x00000004
/*@}*/

/*! \name UART Interrupt Enable Registers */
/*@{*/
#define UART_IER_OFF        0x00000004
#define UART_INT_RDA        (1 << 0)
#define UART_INT_THRE       (1 << 1)
#define UART_INT_RLS        (1 << 2)
#define UART_INT_ABEO       (1 << 8)
#define UART_INT_ABTO       (1 << 9)
/*@}*/

/*! \name UART Interrupt Identification Registers */
/*@{*/
#define UART_IIR_OFF        0x00000008
#define UART_INT_STATUS     (1 << 0)
#define UART_INT_ID         0x0000000E
#define UART_INT_ID_RLS     0x00000006
#define UART_INT_ID_RDA     0x00000004
#define UART_INT_ID_CTI     0x0000000C
#define UART_INT_ID_THRE    0x00000002
#define UART_INT_FIFO       0x000000C0
/*@}*/

/*! \name UART FIFO Control Registers */
/*@{*/
#define UART_FCR_OFF        0x00000008
#define UART_FIFO_EN        (1 << 0)
#define UART_FIFO_RXRST     (1 << 1)
#define UART_FIFO_TXRST     (1 << 2)
#define UART_FIFO_DMA       (1 << 3)
#define UART_FIFO_RXTRIG    0x000000C0
#define UART_FIFO_RXTRIG_LSB 6
/*@}*/

/*! \name UART Line Control Registers */
/*@{*/
#define UART_LCR_OFF        0x0000000C
#define UART_WLEN           0x00000003
#define UART_WLEN_5         0x00000000
#define UART_WLEN_6         0x00000001
#define UART_WLEN_7         0x00000002
#define UART_WLEN_8         0x00000003
#define UART_STOP_2         (1 << 2)
#define UART_PAR            0x00000038
#define UART_PAR_NONE       0x00000000
#define UART_PAR_ODD        0x00000008      /*!< \brief Odd parity */
#define UART_PAR_EVEN       0x00000018      /*!< \brief Even parity */
#define UART_PAR_MARK       0x00000028      /*!< \brief Marked parity. */
#define UART_PAR_SPACE      0x00000038      /*!< \brief Space parity. */
#define UART_BREAK          (1 << 6)
#define UART_DLAB           (1 << 7)
/*@}*/

/*! \name UART Line Status Registers */
/*@{*/
#define UART_LSR_OFF        0x00000014
#define UART_RDR            (1 << 0)
#define UART_OE             (1 << 1)
#define UART_PE             (1 << 2)
#define UART_FE             (1 << 3)
#define UART_BI             (1 << 4)
#define UART_THRE           (1 << 5)
#define UART_TEMT           (1 << 6)
#define UART_RXFE           (1 << 7)
/*@}*/

/*! \name UART Scratch Pad Registers */
/*@{*/
#define UART_SCR_OFF        0x0000001C
/*@}*/

/*! \name UART Auto-Baud Control Registers */
/*@{*/
#define UART_ACR_OFF        0x00000020
#define UART_AB_START       (1 << 0)
#define UART_AB_MODE        (1 << 1)
#define UART_AB_AUTORESTART (1 << 2)
/*@}*/

/*! \name UART IrDA Control Registers */
/*@{*/
#define UART_ICR_OFF        0x00000024
#define UART_IRDA_EN        (1 << 0)
#define UART_IRDA_INV       (1 << 1)
#define UART_PULSE          0x0000003C
#define UART_PULSE_VAR      0x00000000
#define UART_PULSE_2PCLK    0x00000004
#define UART_PULSE_4PCLK    0x0000000C
#define UART_PULSE_8PCLK    0x00000014
#define UART_PULSE_16PCLK   0x0000001C
#define UART_PULSE_32PCLK   0x00000024
#define UART_PULSE_64PCLK   0x0000002C
#define UART_PULSE_128PCLK  0x00000034
#define UART_PULSE_256PCLK  0x0000003C
/*@}*/

/*! \name UART Fractional Divider Registers */
/*@{*/
#define UART_FDR_OFF        0x00000028
#define UART_DIVADDVAL      0x0000000F
#define UART_DIVADDVAL_LSB  0
#define UART_MULVAL         0x000000F0
#define UART_MULVAL_LSB     4
/*@}*/

/*! \name UART Transmit Enable Registers */
/*@{*/
#define UART_TER_OFF        0x00000030
#define UART_TXEN           (1 << 7)
/*@}*/

/*! \name UART0 Register Addresses */
/*@{*/
#ifdef LPC_UART0_BASE
#define UART0_RBR           (LPC_UART0_BASE + UART_RBR_OFF)
#define UART0_THR           (LPC_UART0_BASE + UART_THR_OFF)
#define UART0_DLL           (LPC_UART0_BASE + UART_DLL_OFF)
#define UART0_DLM           (LPC_UART0_BASE + UART_DLM_OFF)
#define UART0_IER           (LPC_UART0_BASE + UART_IER_OFF)
#define UART0_IIR           (LPC_UART0_BASE + UART_IIR_OFF)
#define UART0_FCR           (LPC_UART0_BASE + UART_FCR_OFF)
#define UART0_LCR           (LPC_UART0_BASE + UART_LCR_OFF)
#define UART0_LSR           (LPC_UART0_BASE + UART_LSR_OFF)
#define UART0_SCR           (LPC_UART0_BASE + UART_SCR_OFF)
#define UART0_ACR           (LPC_UART0_BASE + UART_ACR_OFF)
#define UART0_ICR           (LPC_UART0_BASE + UART_ICR_OFF)
#define UART0_FDR           (LPC_UART0_BASE + UART_FDR_OFF)
#define UART0_TER           (LPC_UART0_BASE + UART_TER_OFF)
#ifdef LPC_UART0_EXTENDED
#define UART0_MCR           (LPC_UART0_BASE + UART_MCR_OFF)
#define UART0_MSR           (LPC_UART0_BASE + UART_MSR_OFF)
#define UART0_RS485CTRL     (LPC_UART0_BASE + UART_RS485CTRL_OFF)
#define UART0_ADRMATCH      (LPC_UART0_BASE + UART_ADRMATCH_OFF)
#define UART0_RS485DLY      (LPC_UART0_BASE + UART_RS485DLY_OFF)
#endif
#endif

/*! \name UART1 Register Addresses */
/*@{*/
#ifdef LPC_UART1_BASE
#define UART1_RBR           (LPC_UART1_BASE + UART_RBR_OFF)
#define UART1_THR           (LPC_UART1_BASE + UART_THR_OFF)
#define UART1_DLL           (LPC_UART1_BASE + UART_DLL_OFF)
#define UART1_DLM           (LPC_UART1_BASE + UART_DLM_OFF)
#define UART1_IER           (LPC_UART1_BASE + UART_IER_OFF)
#define UART1_IIR           (LPC_UART1_BASE + UART_IIR_OFF)
#define UART1_FCR           (LPC_UART1_BASE + UART_FCR_OFF)
#define UART1_LCR           (LPC_UART1_BASE + UART_LCR_OFF)
#define UART1_LSR           (LPC_UART1_BASE + UART_LSR_OFF)
#define UART1_SCR           (LPC_UART1_BASE + UART_SCR_OFF)
#define UART1_ACR           (LPC_UART1_BASE + UART_ACR_OFF)
#define UART1_ICR           (LPC_UART1_BASE + UART_ICR_OFF)
#define UART1_FDR           (LPC_UART1_BASE + UART_FDR_OFF)
#define UART1_TER           (LPC_UART1_BASE + UART_TER_OFF)
#ifdef LPC_UART1_EXTENDED
#define UART1_MCR           (LPC_UART1_BASE + UART_MCR_OFF)
#define UART1_MSR           (LPC_UART1_BASE + UART_MSR_OFF)
#define UART1_RS485CTRL     (LPC_UART1_BASE + UART_RS485CTRL_OFF)
#define UART1_ADRMATCH      (LPC_UART1_BASE + UART_ADRMATCH_OFF)
#define UART1_RS485DLY      (LPC_UART1_BASE + UART_RS485DLY_OFF)
#endif
#endif
/*@}*/

/*! \name UART2 Register Addresses */
/*@{*/
#ifdef LPC_UART2_BASE
#define UART2_RBR           (LPC_UART2_BASE + UART_RBR_OFF)
#define UART2_THR           (LPC_UART2_BASE + UART_THR_OFF)
#define UART2_DLL           (LPC_UART2_BASE + UART_DLL_OFF)
#define UART2_DLM           (LPC_UART2_BASE + UART_DLM_OFF)
#define UART2_IER           (LPC_UART2_BASE + UART_IER_OFF)
#define UART2_IIR           (LPC_UART2_BASE + UART_IIR_OFF)
#define UART2_FCR           (LPC_UART2_BASE + UART_FCR_OFF)
#define UART2_LCR           (LPC_UART2_BASE + UART_LCR_OFF)
#define UART2_LSR           (LPC_UART2_BASE + UART_LSR_OFF)
#define UART2_SCR           (LPC_UART2_BASE + UART_SCR_OFF)
#define UART2_ACR           (LPC_UART2_BASE + UART_ACR_OFF)
#define UART2_ICR           (LPC_UART2_BASE + UART_ICR_OFF)
#define UART2_FDR           (LPC_UART2_BASE + UART_FDR_OFF)
#define UART2_TER           (LPC_UART2_BASE + UART_TER_OFF)
#ifdef LPC_UART2_EXTENDED
#define UART2_MCR           (LPC_UART2_BASE + UART_MCR_OFF)
#define UART2_MSR           (LPC_UART2_BASE + UART_MSR_OFF)
#define UART2_RS485CTRL     (LPC_UART2_BASE + UART_RS485CTRL_OFF)
#define UART2_ADRMATCH      (LPC_UART2_BASE + UART_ADRMATCH_OFF)
#define UART2_RS485DLY      (LPC_UART2_BASE + UART_RS485DLY_OFF)
#endif
#endif
/*@}*/

/*! \name UART3 Register Addresses */
/*@{*/
#ifdef LPC_UART3_BASE
#define UART3_RBR           (LPC_UART3_BASE + UART_RBR_OFF)
#define UART3_THR           (LPC_UART3_BASE + UART_THR_OFF)
#define UART3_DLL           (LPC_UART3_BASE + UART_DLL_OFF)
#define UART3_DLM           (LPC_UART3_BASE + UART_DLM_OFF)
#define UART3_IER           (LPC_UART3_BASE + UART_IER_OFF)
#define UART3_IIR           (LPC_UART3_BASE + UART_IIR_OFF)
#define UART3_FCR           (LPC_UART3_BASE + UART_FCR_OFF)
#define UART3_LCR           (LPC_UART3_BASE + UART_LCR_OFF)
#define UART3_LSR           (LPC_UART3_BASE + UART_LSR_OFF)
#define UART3_SCR           (LPC_UART3_BASE + UART_SCR_OFF)
#define UART3_ACR           (LPC_UART3_BASE + UART_ACR_OFF)
#define UART3_ICR           (LPC_UART3_BASE + UART_ICR_OFF)
#define UART3_FDR           (LPC_UART3_BASE + UART_FDR_OFF)
#define UART3_TER           (LPC_UART3_BASE + UART_TER_OFF)
#ifdef LPC_UART3_EXTENDED
#define UART3_MCR           (LPC_UART3_BASE + UART_MCR_OFF)
#define UART3_MSR           (LPC_UART3_BASE + UART_MSR_OFF)
#define UART3_RS485CTRL     (LPC_UART3_BASE + UART_RS485CTRL_OFF)
#define UART3_ADRMATCH      (LPC_UART3_BASE + UART_ADRMATCH_OFF)
#define UART3_RS485DLY      (LPC_UART3_BASE + UART_RS485DLY_OFF)
#endif
#endif
/*@}*/


/*@}*/

#endif
