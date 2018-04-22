#ifndef _LPC17XX_USART_H_
#define _LPC17XX_USART_H_

/*
 * Copyright (C) 2012 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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

/*----------------------------------------------------------------------------*
  General defines
 *----------------------------------------------------------------------------*/

/* Accepted Error baud rate value (in percent unit) */
#define UART_ACCEPTED_BAUDRATE_ERROR    3


/*----------------------------------------------------------------------------*
  Receive and transmit register masks
 *----------------------------------------------------------------------------*/

/* UART Received Buffer mask bit (8 bits) */
#define UART_RBR_MASK            0xFF

/* UART Transmit Holding mask bit (8 bits) */
#define UART_THR_MASK            0xFF


/*----------------------------------------------------------------------------*
  Devisior latch defines
 *----------------------------------------------------------------------------*/

/* Macro for loading least significant halfs of divisors into LSB register */
#define UART_LOAD_DLL(div)       ((div) & 0xFF)
/* Divisor latch LSB bit mask */
#define UART_DLL_MASKBIT         0xFF

/* Divisor latch MSB bit mask */
#define UART_DLM_MASKBIT         0xFF
/* Macro for loading most significant halfs of divisors */
#define UART_LOAD_DLM(div)  (((div) >> 8) & 0xFF)



/*----------------------------------------------------------------------------*
  Interrupt defines
 *----------------------------------------------------------------------------*/

/* RBR Interrupt enable */
#define UART_IER_RBRINT_EN_POS  0
#define UART_IER_RBRINT_EN      0x00000001

/* THR Interrupt enable */
#define UART_IER_THREINT_EN_POS 1
#define UART_IER_THREINT_EN     0x00000002

/* RX line status interrupt enable */
#define UART_IER_RLSINT_EN_POS  2
#define UART_IER_RLSINT_EN      0x00000004

/* Modem status interrupt enable */
#define UART1_IER_MSINT_EN_POS  3
#define UART1_IER_MSINT_EN      0x00000008

/* CTS1 signal transition interrupt enable */
#define UART1_IER_CTSINT_EN_POS 7
#define UART1_IER_CTSINT_EN     0x00000080

/* Enables the end of auto-baud interrupt */
#define UART_IER_ABEOINT_EN_POS 8
#define UART_IER_ABEOINT_EN     0x00000100

/* Enables the auto-baud time-out interrupt */
#define UART_IER_ABTOINT_EN_POS 9
#define UART_IER_ABTOINT_EN     0x00000200

/* UART interrupt enable register bit mask */
#define UART_IER_BITMASK        0x00000307

/* UART1 interrupt enable register bit mask */
#define UART1_IER_BITMASK       0x0000038F


/*----------------------------------------------------------------------------*
  UART interrupt identification register defines
 *----------------------------------------------------------------------------*/

/* Interrupt Status - Active low */
#define UART_IIR_INTSTAT_PEND   0x00000001
/* Interrupt identification: Receive line status*/
#define UART_IIR_INTID_RLS      0x00000006
/* Interrupt identification: Receive data available*/
#define UART_IIR_INTID_RDA      0x00000004
/* Interrupt identification: Character time-out indicator*/
#define UART_IIR_INTID_CTI      0x0000000C
/* Interrupt identification: THRE interrupt*/
#define UART_IIR_INTID_THRE     0x00000002
/* Interrupt identification: Modem interrupt*/
#define UART1_IIR_INTID_MODEM   0x00000000
/* Interrupt identification: Interrupt ID mask */
#define UART_IIR_INTID_MASK     0x0000000E
/* These bits are equivalent to UnFCR[0] */
#define UART_IIR_FIFO_EN        0x000000C0
/* End of auto-baud interrupt */
#define UART_IIR_ABEO_INT       0x00000100
/* Auto-baud time-out interrupt */
#define UART_IIR_ABTO_INT       0x00000200
/* UART interrupt identification register bit mask */
#define UART_IIR_BITMASK        0x000003CF


/*----------------------------------------------------------------------------*
  UART FIFO control register defines
 *----------------------------------------------------------------------------*/

/* UART FIFO enable */
#define UART_FCR_FIFO_EN        0x00000001
/* UART FIFO RX reset */
#define UART_FCR_RX_RS          0x00000002
/* UART FIFO TX reset */
#define UART_FCR_TX_RS          0x00000004
/* UART DMA mode selection */
#define UART_FCR_DMAMODE_SEL    0x00000008
/* UART FIFO trigger level 0: 1 character */
#define UART_FCR_TRG_LEV0       0x00000000
/* UART FIFO trigger level 1: 4 character */
#define UART_FCR_TRG_LEV1       0x00000040
/* UART FIFO trigger level 2: 8 character */
#define UART_FCR_TRG_LEV2       0x00000080
/* UART FIFO trigger level 3: 14 character */
#define UART_FCR_TRG_LEV3       0x000000C0
/* UART FIFO control bit mask */
#define UART_FCR_BITMASK        0x000000CF

#define UART_TX_FIFO_SIZE       16


/*----------------------------------------------------------------------------*
  UART line control register defines
 *----------------------------------------------------------------------------*/

#define UART_LCR_WLEN_POS       0
/* UART 5 bit data mode */
#define UART_LCR_WLEN5          0x00000000
/* UART 6 bit data mode */
#define UART_LCR_WLEN6          0x00000001
/* UART 7 bit data mode */
#define UART_LCR_WLEN7          0x00000002
/* UART 8 bit data mode */
#define UART_LCR_WLEN8          0x00000003
/* UART WLEN Bitmask */
#define UART_LCR_WLEN_BITMASK   0x00000003

/* UART 1.5 ot 2 Stop Bits Select */
#define UART_LCR_STOPBIT_SEL_POS 2
#define UART_LCR_STOPBIT_SEL    0x00000004

/* UART Parity Enable */
#define UART_LCR_PARITY_EN_POS  3
#define UART_LCR_PARITY_EN      0x00000008

#define UART_LCR_PARITY_POS     4
/* UART Odd Parity Select */
#define UART_LCR_PARITY_ODD     0x00000000
/* UART Even Parity Select */
#define UART_LCR_PARITY_EVEN    0x00000010
/* UART force 1 stick parity */
#define UART_LCR_PARITY_F_1     0x00000020
/* UART force 0 stick parity */
#define UART_LCR_PARITY_F_0     0x00000030
/* UART Parity Bitmask */
#define UART_LCR_PARITY_BITMASK 0x00000038


/* UART Transmission Break enable */
#define UART_LCR_BREAK_EN_POS   6
#define UART_LCR_BREAK_EN       0x00000040

/* UART Divisor Latches Access bit enable */
#define UART_LCR_DLAB_EN_POS    7
#define UART_LCR_DLAB_EN        0x00000080

/* UART line control bit mask */
#define UART_LCR_BITMASK        0x000000FF


/*----------------------------------------------------------------------------*
  UART1 modem control register defines
 *----------------------------------------------------------------------------*/

/* Source for modem output pin DTR */
#define UART1_MCR_DTR_CTRL              0x00000001
/* Source for modem output pin RTS */
#define UART1_MCR_RTS_CTRL              0x00000002
/* Loop back mode select */
#define UART1_MCR_LOOPB_EN              0x00000010
/* Enable Auto RTS flow-control */
#define UART1_MCR_AUTO_RTS_EN           0x00000040
/* Enable Auto CTS flow-control */
#define UART1_MCR_AUTO_CTS_EN           0x00000080
/* UART1 bit mask value */
#define UART1_MCR_BITMASK               0x000000D3


/*----------------------------------------------------------------------------*
  UART line status register defines
 *----------------------------------------------------------------------------*/

/* Line status register: Receive data ready*/
#define UART_LSR_RDR            0x00000001
/* Line status register: Overrun error*/
#define UART_LSR_OE             0x00000002
/* Line status register: Parity error*/
#define UART_LSR_PE             0x00000004
/* Line status register: Framing error*/
#define UART_LSR_FE             0x00000008
/* Line status register: Break interrupt*/
#define UART_LSR_BI             0x00000010
/* Line status register: Transmit holding register empty*/
#define UART_LSR_THRE           0x00000020
/* Line status register: Transmitter empty*/
#define UART_LSR_TEMT           0x00000040
/* Error in RX FIFO*/
#define UART_LSR_RXFE           0x00000080
/* UART Line status bit mask */
#define UART_LSR_BITMASK        0x000000FF


/*----------------------------------------------------------------------------*
  UART Modem (UART1 only) status register defines
 *----------------------------------------------------------------------------*/

/* Set upon state change of input CTS */
#define UART1_MSR_DELTA_CTS     0x00000001
/* Set upon state change of input DSR */
#define UART1_MSR_DELTA_DSR     0x00000002
/* Set upon low to high transition of input RI */
#define UART1_MSR_LO2HI_RI      0x00000004
/* Set upon state change of input DCD */
#define UART1_MSR_DELTA_DCD     0x00000008
/* Clear To Send State */
#define UART1_MSR_CTS           0x00000010
/* Data Set Ready State */
#define UART1_MSR_DSR           0x00000020
/* Ring Indicator State */
#define UART1_MSR_RI            0x00000040
/* Data Carrier Detect State */
#define UART1_MSR_DCD           0x00000080
/* MSR register bit-mask value */
#define UART1_MSR_BITMASK       0x000000FF


/*----------------------------------------------------------------------------*
  UART Scratch Pad Register defines
 *----------------------------------------------------------------------------*/

/* UART Scratch Pad bit mask */
#define UART_SCR_BIMASK         0x000000FF


/*----------------------------------------------------------------------------*
  UART Auto baudrate control register defines
 *----------------------------------------------------------------------------*/

/* UART Auto-baud start */
#define UART_ACR_START          0x00000001
/* UART Auto baudrate Mode 1 */
#define UART_ACR_MODE           0x00000002
/* UART Auto baudrate restart */
#define UART_ACR_AUTO_RESTART   0x00000004
/* UART End of auto-baud interrupt clear */
#define UART_ACR_ABEOINT_CLR    0x00000100
/* UART Auto-baud time-out interrupt clear */
#define UART_ACR_ABTOINT_CLR    0x00000200
/* UART Auto Baudrate register bit mask */
#define UART_ACR_BITMASK        0x00000307


/*----------------------------------------------------------------------------*
  UART IrDA control register defines
 *----------------------------------------------------------------------------*/

/* IrDA mode enable */
#define UART_ICR_IRDAEN                 0x00000001
/* IrDA serial input inverted */
#define UART_ICR_IRDAINV                0x00000002
/* IrDA fixed pulse width mode */
#define UART_ICR_FIXPULSE_EN            0x00000004
/* PulseDiv - Configures the pulse when FixPulseEn = 1 */
#define UART_ICR_PULSEDIV(n)            ((uint32_t)(((n) & 0x07) <<3))
/* UART IRDA bit mask */
#define UART_ICR_BITMASK                0x0000003F


/*----------------------------------------------------------------------------*
  UART Fractional divider register defines
 *----------------------------------------------------------------------------*/

/* Baud-rate generation pre-scaler divisor */
#define UART_FDR_DIVADDVAL(n)   ((uint32_t)((n) & 0x0F))
/* Baud-rate pre-scaler multiplier value */
#define UART_FDR_MULVAL(n)      ((uint32_t)(((n) << 4) & 0xF0))
/* UART Fractional Divider register bit mask */
#define UART_FDR_BITMASK        0x000000FF


/*----------------------------------------------------------------------------*
  UART Tx Enable register defines
 *----------------------------------------------------------------------------*/

/* Transmit enable bit */
#define UART_TER_TXEN_POS       7
#define UART_TER_TXEN           0x00000080
/* UART Transmit Enable Register bit mask */
#define UART_TER_BITMASK        0x00000080


/*----------------------------------------------------------------------------*
  UART1 RS485 Control register defines
 *----------------------------------------------------------------------------*/

/* RS-485/EIA-485 Normal Multi-drop Mode (NMM) is disabled */
#define UART1_RS485CTRL_NMM_EN          0x00000001
/* The receiver is disabled */
#define UART1_RS485CTRL_RX_DIS          0x00000002
/* Auto Address Detect (AAD) is enabled */
#define UART1_RS485CTRL_AADEN           0x00000004
/* If direction control is enabled (bit DCTRL = 1), pin DTR is used for direction control */
#define UART1_RS485CTRL_SEL_DTR         0x00000008
/* Enable Auto Direction Control */
#define UART1_RS485CTRL_DCTRL_EN        0x00000010
/* This bit reverses the polarity of the direction control signal on the RTS (or DTR) pin.
The direction control pin will be driven to logic "1" when the transmitter has data to be sent */
#define UART1_RS485CTRL_OINV_1          0x00000020

/* RS485 control bit-mask value */
#define UART1_RS485CTRL_BITMASK         0x0000003F


/*----------------------------------------------------------------------------*
  UART1 RS-485 Address Match register defines
 *----------------------------------------------------------------------------*/
/* Address match register bitmask */
#define UART1_RS485ADRMATCH_BITMASK     0x000000FF


/*----------------------------------------------------------------------------*
  UART1 RS-485 Delay value register defines
 *----------------------------------------------------------------------------*/
/* RS-485 Delay value register bitmask */
#define UART1_RS485DLY_BITMASK          0x000000FF


/*----------------------------------------------------------------------------*
  UART FIFO Level register defines
 *----------------------------------------------------------------------------*/

/* Reflects the current level of the UART receiver FIFO */
#define UART_FIFOLVL_RXFIFOLVL(n)       ((uint32_t)((n) & 0x0F))
/* Reflects the current level of the UART transmitter FIFO */
#define UART_FIFOLVL_TXFIFOLVL(n)       ((uint32_t)(((n) >> 8) & 0x0F))
/* UART FIFO Level Register bit mask */
#define UART_FIFOLVL_BITMASK            0x00000F0F

#endif /* _LPC17XX_USART_H_ */
