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

#include <inttypes.h>
#include <cfg/clock.h>

#include <dev/debug.h>
#include <dev/gpio.h>

#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#include <arch/cm3/nxp/lpc176x_clk.h>
#include <arch/cm3/nxp/lpc176x_gpio.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_gpio.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_gpio.h>
#else
#warning "Unknown LPC familiy"
#endif

#include <arch/cm3/nxp/lpc17xx_usart.h>


/*!
 * \addtogroup xgNutArchArmLpc17xxDebug
 */
/*@{*/


/* USART default speed if not preset by nutconfig */
#ifndef DEBUG_INIT_BAUTRATE
#define DEBUG_INIT_BAUTRATE 115200
#endif

/*!
 * \brief Carefully enable USART hardware functions.
 *
 * Always enabale transmitter and receiver, even on read-only or
 * write-only mode.
 */
static void Lpc17xxDevDebugEnable(void)
{

    /* Enable UART transmitter. The receiver can not be enabled seperately on
       the LPC architecture. We just could disable the RX FIFOs, is this a
       good idea?
     */

    USARTn->TER = UART_TER_TXEN;
}

/*!
 * \brief Carefully disable USART hardware functions.
 *
 * TODO: Beschreibung anpassen
 * This routine is called before changing cruical settings like
 * baudrate, frame format etc.
 *
 * The previous version uses a 10ms delay to make sure, that any
 * incoming or outgoing character is processed. However, this time
 * depends on the baudrate.
 *
 * In fact we do not need to take care of incoming characters,
 * when changing such settings.
 *
 * For outgoing characters however, settings may be changed on
 * the fly and we should wait, until the last character transmitted
 * with the old settings has left the shift register. While TXRDY
 * is set when the holding register is empty, TXEMPTY is set when the
 * shift register is empty. The bad news is, that both are zero, if
 * the transmitter is disabled. We are not able to determine this
 * state. So we check TXRDY first and, if set, wait for any character
 * currently in the shift register.
 */
static void Lpc17xxDevDebugDisable(void)
{
    /* If the transmitter is enabled, wait until all bits had been shifted out. */
    if (USARTn->TER & UART_TER_TXEN) {
        while ((USARTn->LSR & UART_LSR_TEMT) == 0);
    }

    /* Disable Transmitter, receiver can not be disabled on the LPC
       architecture. We just could disable the RX FIFOs, is this a good idea?
     */
    USARTn->TER = 0;
}

/*!
 * \brief Set the USART hardware bit rate.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param rate Number of bits per second.
 *
 * \return 0 on success, -1 otherwise.
 */

static int Lpc17xxDevDebugSetSpeed(uint32_t baudrate)
{
    uint32_t uart_clock;
    uint32_t calcBaudrate = 0;
    uint32_t temp = 0;

    uint32_t mulFracDiv, dividerAddFracDiv;
    uint32_t diviser = 0 ;
    uint32_t bestm = 1;
    uint32_t bestd = 0;
    uint32_t best_divisor = 0;

    uint32_t relativeError = 0;
    uint32_t best_error = 100000;

    /* get UART block clock */
    uart_clock = NutArchClockGet(NUT_HWCLK_PCLK);

#if defined(MCU_LPC176x)

    if ((LPC_UART_TypeDef*)USARTn == LPC_UART0) {
        uart_clock /= Lpc176x_PclkDivGet(CLKPWR_PCLKSEL_UART0);
    } else
    if ((LPC_UART1_TypeDef*)USARTn == LPC_UART1) {
        uart_clock /= Lpc176x_PclkDivGet(CLKPWR_PCLKSEL_UART1);
    } else
    if ((LPC_UART_TypeDef*)USARTn == LPC_UART2) {
        uart_clock /= Lpc176x_PclkDivGet(CLKPWR_PCLKSEL_UART2);
    } else
    if ((LPC_UART_TypeDef*)USARTn == LPC_UART3) {
        uart_clock /= Lpc176x_PclkDivGet(CLKPWR_PCLKSEL_UART3);
    }

#endif

    uart_clock = uart_clock >> 4; /* div by 16 */

    /* Baudrate calculation is done according the following formula:
       BaudRate= uart_clock * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)

       To avoid floating point calculation the formulae is adjusted with the
       multiply and divide method.

       The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
       0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15
    */

    for (mulFracDiv = 1 ; mulFracDiv <= 15 ;mulFracDiv++)
    {
        for (dividerAddFracDiv = 0 ; dividerAddFracDiv <= 15 ;dividerAddFracDiv++)
        {
            temp = (mulFracDiv * uart_clock) / ((mulFracDiv + dividerAddFracDiv));

            diviser = temp / baudrate;
            if ((temp % baudrate) > (baudrate / 2))
                diviser++;

            if (diviser > 2 && diviser < 65536)
            {
                calcBaudrate = temp / diviser;

                if (calcBaudrate <= baudrate) {
                    relativeError = baudrate - calcBaudrate;
                } else {
                    relativeError = calcBaudrate - baudrate;
                }

                if ((relativeError < best_error))
                {
                    bestm = mulFracDiv ;
                    bestd = dividerAddFracDiv;
                    best_divisor = diviser;
                    best_error = relativeError;
                    if (relativeError == 0) {
                        break;
                    }
                }
            }
        }
        if (relativeError == 0) {
            break;
        }
    }

    Lpc17xxDevDebugDisable();

    if (best_error < ((baudrate * UART_ACCEPTED_BAUDRATE_ERROR) / 100)) {
        /* Set DLAB bit */
        CM3BBSET(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_DLAB_EN_POS);

        USARTn->DLM  = UART_LOAD_DLM(best_divisor);
        USARTn->DLL  = UART_LOAD_DLL(best_divisor);

        /* Reset DLAB bit */
        CM3BBCLR(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_DLAB_EN_POS);
        USARTn->FDR  = (UART_FDR_MULVAL(bestm) | UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;
    } else {
        return -1;
    }

    Lpc17xxDevDebugEnable();

    return 0;
}

/*!
 * \brief Set the USART hardware to the number of data bits.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxDevDebugSetDataBits(uint8_t bits)
{
    int rc = 0;
    uint32_t lcr;

    Lpc17xxDevDebugDisable();

    lcr = USARTn->LCR & ~UART_LCR_WLEN_BITMASK;
    switch (bits)
    {
        case 5:
            lcr |= UART_LCR_WLEN5;
            break;

        case 6:
            lcr |= UART_LCR_WLEN6;
            break;

        case 7:
            lcr |= UART_LCR_WLEN7;
            break;

        case 8:
            lcr |= UART_LCR_WLEN8;
            break;

        default:
            Lpc17xxDevDebugEnable();
            return -1;
    }
    USARTn->LCR = lcr & UART_LCR_BITMASK;

    Lpc17xxDevDebugEnable();

    return rc;
}

/*!
 * \brief Set the USART hardware to the specified parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param mode 0 (disabled), 1 (odd), 2 (even) 3 (mark) or 4(space)
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxDevDebugSetParity(uint8_t mode)
{
    uint32_t lcr;

    Lpc17xxDevDebugDisable();

    lcr = USARTn->LCR & ~UART_LCR_PARITY_BITMASK;

    switch (mode) {
        case 0:
            /* Parity disabled, do nothing */
            break;
        case 1:
            lcr |= UART_LCR_PARITY_EN | UART_LCR_PARITY_ODD;
            break;
        case 2:
            lcr |= UART_LCR_PARITY_EN | UART_LCR_PARITY_EVEN;
            break;
        case 3:
            lcr |= UART_LCR_PARITY_EN | UART_LCR_PARITY_F_1;
            break;
        case 4:
            lcr |= UART_LCR_PARITY_EN | UART_LCR_PARITY_F_0;
            break;
        default:
            Lpc17xxDevDebugEnable();
            return -1;
    }

    USARTn->LCR = lcr & UART_LCR_BITMASK;

    Lpc17xxDevDebugEnable();

    return 0;
}

/*!
 * \brief Set the USART hardware to the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param bits The number of stop bits set, either 1, 2 or 3 (1.5 bits).
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxDevDebugSetStopBits(uint8_t bits)
{
    Lpc17xxDevDebugDisable();

    switch (bits) {
        case 1:
            CM3BBCLR(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_STOPBIT_SEL_POS);
            break;
        case 2:
            CM3BBSET(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_STOPBIT_SEL_POS);
            break;
        case 3:
            if ((USARTn->LCR & UART_LCR_WLEN_BITMASK) == UART_LCR_WLEN5) {
                CM3BBSET(USARTnBase, LPC_UART_TypeDef, LCR, UART_LCR_STOPBIT_SEL_POS);
            } else {
                Lpc17xxDevDebugEnable();
                return -1;
            }
            break;
        default:
            Lpc17xxDevDebugEnable();
            return -1;
    }

    Lpc17xxDevDebugEnable();

    return 0;
}

/*!
 * \brief   Open debug device.
 *
 * \param   dev  Pointer to a previously registered \ref NUTDEVICE structure.
 * \param   name Ignored, typically points to an empty string.
 * \param   mode Ignored, operation mode.
 * \param   acc  Ignored, should be zero.
 *
 * \return  Pointer to a static NUTFILE structure.
 */
NUTFILE *Lpc17xxDevDebugOpen(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    NUTFILE *fp = (NUTFILE *) (dev->dev_dcb);

    fp->nf_dev = dev;
    fp->nf_fcb = NULL;

    return fp;
}

/*!
 * \brief   Close debug device.
 *
 * \param   fp Pointer to a \ref _NUTFILE structure, obtained by a previous
 *             call to LpcDevDebugOpen().
 *
 * \return  Always 0.
 */
int Lpc17xxDevDebugClose(NUTFILE * fp)
{
    return 0;
}

/*!
 * \brief Send a single character to debug device.
 *
 * A newline character will be automatically prepended by a carriage
 * return.
 */
static void DebugPut(const NUTDEVICE * dev, char ch)
{
    if (ch == '\n') {
        DebugPut(dev, '\r');
    }
    while ((USARTn->LSR & UART_LSR_THRE) == 0);
    USARTn->THR = ch;
}

/*!
 * \brief Send characters to debug device 0.
 *
 * This function is called by the low level input routines of the
 * \ref xrCrtLowio "C runtime library", using the _NUTDEVICE::dev_write
 * entry.
 *
 * A newline character will be automatically prepended by a carriage
 * return.
 *
 * \param fp Pointer to a \ref _NUTFILE structure, obtained by a previous
 *           call to LpcDevDebugOpen().
 *
 * \return Number of characters sent.
 */
static int Lpc17xxDevDebugWrite(NUTFILE * fp, const void *buffer, int len)
{
    int c = len;
    const char *cp = buffer;

    while (c--) {
        DebugPut(fp->nf_dev, *cp++);
    }
    return len;
}

#ifdef NUT_DEV_DEBUG_READ

/*!
 * \brief Read characters from debug device.
 *
 * This function is called by the low level input routines of the
 * \ref xrCrtLowio "C runtime library", using the _NUTDEVICE::dev_read
 * entry.
 *
 * The function will block the calling thread until at least one
 * character has been received.
 *
 * \param fp     Pointer to a \ref _NUTFILE structure, obtained by a
 *               previous call to LpcDevDebugOpen().
 * \param buffer Pointer to the buffer that receives the data. If zero,
 *               then all characters in the input buffer will be
 *               removed.
 * \param size   Maximum number of bytes to read.
 *
 * \return The number of bytes read, which may be less than the number
 *         of bytes specified. A return value of -1 indicates an error,
 *         while zero is returned in case of a timeout.
 */

static int Lpc17xxDevDebugRead(NUTFILE * fp, void *buffer, int size)
{
    int rc;
    unsigned int ch;
    char *bp = (char *) buffer;

    /* Wait for the first character, forever. */
    for (rc = 0; rc < size; rc++) {
        while ((USARTn->LSR & UART_LSR_RDR) == 0) {
            NutSleep(1);
            if ((rc || bp == NULL) &&
                (USARTn->LSR & UART_LSR_RDR) == 0) {
                return rc;
            }
        }
        ch = USARTn->RBR;
        if (bp) {
            if (ch == '\r') {
                *bp++ = '\n';
            } else {
                *bp++ = (char) ch;
            }
        }
    }
    return rc;
}

/*!
 * \brief Retrieves the number of characters in input buffer.
 *
 * This function is called by the low level size routine of the C runtime
 * library, using the _NUTDEVICE::dev_size entry.
 *
 * \param fp Pointer to a \ref _NUTFILE structure, obtained by a previous
 *           call to LpcDevDebugOpen().
 *
 * \return The number of bytes currently stored in input buffer.
 */
static long Lpc17xxDevDebugSize(NUTFILE *fp)
{
    if (USARTn->LSR & UART_LSR_RDR) {
        return 1;
    }
    return 0;
}

#endif /* NUT_DEV_DEBUG_READ */

/*!
 * \brief Handle I/O controls for debug device 2.
 *
 * The debug device supports UART_SETSPEED, UART_SETDATABITS,
 *                           UART_SETPARITY and UART_SETSTOPBITS.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxDevDebugIOCtl(NUTDEVICE * dev, int req, void *conf)
{
    int rc = -1;

    switch(req) {
    case UART_SETSPEED:
        rc = Lpc17xxDevDebugSetSpeed(*((uint32_t *)conf));
        break;

    case UART_SETDATABITS:
        rc = Lpc17xxDevDebugSetDataBits(*((uint32_t *)conf));
        break;

    case UART_SETPARITY:
        rc = Lpc17xxDevDebugSetParity(*((uint32_t *)conf));
        break;

    case UART_SETSTOPBITS:
        rc = Lpc17xxDevDebugSetStopBits(*((uint32_t *)conf));
        break;

    default:
        break;
    }

    return rc;
}


/*
 * \brief Initialize the USART hardware driver.
 *
 * This function is called during device registration by the upper level
 * USART driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxDevDebugInit(NUTDEVICE * dev)
{
    /* Enable UART clock and power */
#if defined(MCU_LPC176x)
    if((LPC_UART_TypeDef*)USARTn == LPC_UART0) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART0);
    } else
    if((LPC_UART1_TypeDef*)USARTn == LPC_UART1) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART1);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART2) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART2);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART3) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART3);
    }
#elif defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
    if((LPC_UART_TypeDef*)USARTn == LPC_UART0) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART0);

        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART0);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART0);
    } else
    if((LPC_UART1_TypeDef*)USARTn == LPC_UART1) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART1);

        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART1);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART1);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART2) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART2);

        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART2);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART2);
    } else
    if((LPC_UART_TypeDef*)USARTn == LPC_UART3) {
        SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCUART3);

        SysCtlPeripheralResetEnable(CLKPWR_RSTCON0_UART3);
        SysCtlPeripheralResetDisable(CLKPWR_RSTCON0_UART3);
    }
#endif
    /* Disable IRQs */
    USARTn->IER = 0;

    /* Clear FIFOs */
    USARTn->FCR |= UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS;

    /* Dummy reading */
    while (USARTn->LSR & UART_LSR_RDR) {
        (volatile uint32_t)USARTn->RBR;
    }

    /* Enable transmitter */
    USARTn->TER = UART_TER_TXEN;

    /* Wait for current transmit complete */
    while (!(USARTn->LSR & UART_LSR_THRE));

    /* Disable transmitter */
    USARTn->TER = 0;

    /* Set LCR to default state */
    USARTn->LCR = 0;

    /* Set ACR to default state */
    USARTn->ACR = 0;

#if defined(MCU_LPC176x)
    if((LPC_UART1_TypeDef*)USARTn == LPC_UART1) {
        /* Set RS485 control to default state */
        ((LPC_UART1_TypeDef*)USARTn)->RS485CTRL = 0;

        /* Set RS485 delay timer to default state */
        ((LPC_UART1_TypeDef*)USARTn)->RS485DLY = 0;

        /* Set RS485 addr match to default state */
        ((LPC_UART1_TypeDef*)USARTn)->ADRMATCH = 0;
    }
#elif defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
    /* Set RS485 control to default state */
    USARTn->RS485CTRL = 0;

    /* Set RS485 delay timer to default state */
    USARTn->RS485DLY = 0;

    /* Set RS485 addr match to default state */
    USARTn->ADRMATCH = 0;
#endif

    /* Dummy reading to clear bits */
    (volatile uint32_t)USARTn->LSR;

    if(((LPC_UART1_TypeDef *)USARTn) == LPC_UART1) {
        /* Set Modem Control to default state */
        ((LPC_UART1_TypeDef *)USARTn)->MCR = 0;

        /* Dummy Reading to Clear Status */
        (volatile uint32_t)((LPC_UART1_TypeDef *)USARTn)->MSR;
    }
#if defined(MCU_LPC177x_8x) || defined(MCU_LPC407x_8x)
    if(((LPC_UART4_TypeDef *)USARTn) == LPC_UART4) {
        /* Set IrDA to default state for all UART other than UART1 */
        ((LPC_UART4_TypeDef *)USARTn)->ICR = 0;
    }
#endif
    /* Configure USART Tx as alternate function push-pull */
    GpioPinConfigSet( TX_GPIO_PORT, TX_GPIO_PIN, TX_GPIO_PIN_CFG);
    /* Configure USART Rx as input floating */
    GpioPinConfigSet( RX_GPIO_PORT, RX_GPIO_PIN, RX_GPIO_PIN_CFG);

#if defined(RTS_GPIO_PORT) && defined(RTS_GPIO_PIN)
    /* Configure USART RTS as alternate function push-pull */
    GpioPinConfigSet( RTS_GPIO_PORT, RTS_GPIO_PIN, RTS_GPIO_PIN_CFG);
#endif
#if defined(CTS_GPIO_PORT) && defined(CTS_GPIO_PIN)
    /* Configure USART CTS as input floating */
    GpioPinConfigSet( CTS_GPIO_PORT, CTS_GPIO_PIN, CTS_GPIO_PIN_CFG);
#endif


    /* Configure UART communication parameters */

    Lpc17xxDevDebugSetSpeed(DEBUG_INIT_BAUTRATE);
    Lpc17xxDevDebugSetDataBits(8);
    Lpc17xxDevDebugSetStopBits(1);
    Lpc17xxDevDebugSetParity(0);

    return 0;
}

/*@}*/
