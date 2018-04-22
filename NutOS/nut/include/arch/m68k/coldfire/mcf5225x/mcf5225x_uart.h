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

#ifndef MCF5225X_UART_H_
#define MCF5225X_UART_H_

/* Duplex modes */
#define HALF    0
#define FULL    1

/* UART control structure used for write only registers */
typedef struct _UARTWREGS UARTWREGS;
struct _UARTWREGS {
    uint8_t uimr;   /* UISR/UIMR register, write from one, red from other => write only */
    uint8_t ubg1;   /* Write only */
    uint8_t ubg2;   /* Write only */
    uint8_t umr1;   /* Write/ Read after RESET MODE REGISTER POINTER */
    uint8_t umr2;   /* Write/ Read after Write/ Read from UMR1 register */
};

/* UART Registers */
#define MCF_UART_UMR(x)                      (*(volatile uint8_t *)(0x40000200 + ((x) * 0x40)))
#define MCF_UART_USR(x)                      (*(volatile uint8_t *)(0x40000204 + ((x) * 0x40)))
#define MCF_UART_UCSR(x)                     (*(volatile uint8_t *)(0x40000204 + ((x) * 0x40)))
#define MCF_UART_UCR(x)                      (*(volatile uint8_t *)(0x40000208 + ((x) * 0x40)))
#define MCF_UART_URB(x)                      (*(volatile uint8_t *)(0x4000020C + ((x) * 0x40)))
#define MCF_UART_UTB(x)                      (*(volatile uint8_t *)(0x4000020C + ((x) * 0x40)))
#define MCF_UART_UIPCR(x)                    (*(volatile uint8_t *)(0x40000210 + ((x) * 0x40)))
#define MCF_UART_UACR(x)                     (*(volatile uint8_t *)(0x40000210 + ((x) * 0x40)))
#define MCF_UART_UIMR(x)                     (*(volatile uint8_t *)(0x40000214 + ((x) * 0x40)))
#define MCF_UART_UISR(x)                     (*(volatile uint8_t *)(0x40000214 + ((x) * 0x40)))
#define MCF_UART_UBG1(x)                     (*(volatile uint8_t *)(0x40000218 + ((x) * 0x40)))
#define MCF_UART_UBG2(x)                     (*(volatile uint8_t *)(0x4000021C + ((x) * 0x40)))
#define MCF_UART_UIP(x)                      (*(volatile uint8_t *)(0x40000234 + ((x) * 0x40)))
#define MCF_UART_UOP1(x)                     (*(volatile uint8_t *)(0x40000238 + ((x) * 0x40)))
#define MCF_UART_UOP0(x)                     (*(volatile uint8_t *)(0x4000023C + ((x) * 0x40)))

/* MCF_UART_UMR */
#define MCF_UART_UMR_BC(x)                   (((x) & 0x3) << 0)
#define MCF_UART_UMR_BC_5                    0
#define MCF_UART_UMR_BC_6                    0x1
#define MCF_UART_UMR_BC_7                    0x2
#define MCF_UART_UMR_BC_8                    0x3
#define MCF_UART_UMR_PT                      0x4
#define MCF_UART_UMR_PM(x)                   (((x) & 0x3) << 0x3)
#define MCF_UART_UMR_ERR                     0x20
#define MCF_UART_UMR_RXIRQ                   0x40
#define MCF_UART_UMR_RXRTS                   0x80
#define MCF_UART_UMR_PM_MULTI_ADDR           0x1C
#define MCF_UART_UMR_PM_MULTI_DATA           0x18
#define MCF_UART_UMR_PM_NONE                 0x10
#define MCF_UART_UMR_PM_FORCE_HI             0xC
#define MCF_UART_UMR_PM_FORCE_LO             0x8
#define MCF_UART_UMR_PM_ODD                  0x4
#define MCF_UART_UMR_PM_EVEN                 0
#define MCF_UART_UMR_SB(x)                   (((x) & 0xF) << 0)
#define MCF_UART_UMR_SB_STOP_BITS_1          0x7
#define MCF_UART_UMR_SB_STOP_BITS_15         0x8
#define MCF_UART_UMR_SB_STOP_BITS_2          0xF
#define MCF_UART_UMR_TXCTS                   0x10
#define MCF_UART_UMR_TXRTS                   0x20
#define MCF_UART_UMR_CM(x)                   (((x) & 0x3) << 0x6)
#define MCF_UART_UMR_CM_NORMAL               0
#define MCF_UART_UMR_CM_ECHO                 0x40
#define MCF_UART_UMR_CM_LOCAL_LOOP           0x80
#define MCF_UART_UMR_CM_REMOTE_LOOP          0xC0

/* MCF_UART_USR */
#define MCF_UART_USR_RXRDY                   0x1
#define MCF_UART_USR_FFULL                   0x2
#define MCF_UART_USR_TXRDY                   0x4
#define MCF_UART_USR_TXEMP                   0x8
#define MCF_UART_USR_OE                      0x10
#define MCF_UART_USR_PE                      0x20
#define MCF_UART_USR_FE                      0x40
#define MCF_UART_USR_RB                      0x80

/* MCF_UART_UCSR */
#define MCF_UART_UCSR_TCS(x)                 (((x) & 0xF) << 0)
#define MCF_UART_UCSR_TCS_SYS_CLK            0xD
#define MCF_UART_UCSR_TCS_CTM16              0xE
#define MCF_UART_UCSR_TCS_CTM                0xF
#define MCF_UART_UCSR_RCS(x)                 (((x) & 0xF) << 0x4)
#define MCF_UART_UCSR_RCS_SYS_CLK            0xD0
#define MCF_UART_UCSR_RCS_CTM16              0xE0
#define MCF_UART_UCSR_RCS_CTM                0xF0

/* MCF_UART_UCR */
#define MCF_UART_UCR_RC(x)                   (((x) & 0x3) << 0)
#define MCF_UART_UCR_RX_ENABLED              0x1
#define MCF_UART_UCR_RX_DISABLED             0x2
#define MCF_UART_UCR_TC(x)                   (((x) & 0x3) << 0x2)
#define MCF_UART_UCR_TX_ENABLED              0x4
#define MCF_UART_UCR_TX_DISABLED             0x8
#define MCF_UART_UCR_MISC(x)                 (((x) & 0x7) << 0x4)
#define MCF_UART_UCR_NONE                    0
#define MCF_UART_UCR_RESET_MR                0x10
#define MCF_UART_UCR_RESET_RX                0x20
#define MCF_UART_UCR_RESET_TX                0x30
#define MCF_UART_UCR_RESET_ERROR             0x40
#define MCF_UART_UCR_RESET_BKCHGINT          0x50
#define MCF_UART_UCR_START_BREAK             0x60
#define MCF_UART_UCR_STOP_BREAK              0x70

/* MCF_UART_URB */
#define MCF_UART_URB_RB(x)                   (((x) & 0xFF) << 0)

/* MCF_UART_UTB */
#define MCF_UART_UTB_TB(x)                   (((x) & 0xFF) << 0)

/* MCF_UART_UIPCR */
#define MCF_UART_UIPCR_CTS                   0x1
#define MCF_UART_UIPCR_COS                   0x10

/* MCF_UART_UACR */
#define MCF_UART_UACR_IEC                    0x1

/* MCF_UART_UIMR */
#define MCF_UART_UIMR_TXRDY                  0x1
#define MCF_UART_UIMR_FFULL_RXRDY            0x2
#define MCF_UART_UIMR_DB                     0x4
#define MCF_UART_UIMR_COS                    0x80

/* MCF_UART_UISR */
#define MCF_UART_UISR_TXRDY                  0x1
#define MCF_UART_UISR_FFULL_RXRDY            0x2
#define MCF_UART_UISR_DB                     0x4
#define MCF_UART_UISR_COS                    0x80

/* MCF_UART_UBG1 */
#define MCF_UART_UBG1_Divider_MSB(x)         (((x) & 0xFF) << 0)

/* MCF_UART_UBG2 */
#define MCF_UART_UBG2_Divider_LSB(x)         (((x) & 0xFF) << 0)

/* MCF_UART_UIP */
#define MCF_UART_UIP_CTS                     0x1

/* MCF_UART_UOP1 */
#define MCF_UART_UOP1_RTS                    0x1

/* MCF_UART_UOP0 */
#define MCF_UART_UOP0_RTS                    0x1

#endif /* MCF5225X_UART_H_ */
