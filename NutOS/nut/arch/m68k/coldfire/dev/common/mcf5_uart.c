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

#include <string.h>
#include <arch/m68k.h>
#include <dev/gpio.h>
#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

/*!
 * \addtogroup xgNutArchM68kColdfireUart
 */
/*@{*/

/* Enable Transmit Ready Interrupt. */
#define SET_TXRDY_INTERRUPT() \
    {   \
        reg_uart.uimr |= MCF_UART_UIMR_TXRDY;\
        PREVENT_SPURIOUS_INTERRUPT(MCF_UART_UIMR(BASE) = reg_uart.uimr;)\
    }

/* Disable Transmit Ready Interrupt. */
#define CLR_TXRDY_INTERRUPT() \
    {   \
        reg_uart.uimr &= ~MCF_UART_UIMR_TXRDY;\
        PREVENT_SPURIOUS_INTERRUPT(MCF_UART_UIMR(BASE) = reg_uart.uimr;)\
    }


/* Enable Receive Ready Interrupt. */
#define SET_RXRDY_INTERRUPT() \
    {   \
        reg_uart.uimr |= MCF_UART_UIMR_FFULL_RXRDY;\
        PREVENT_SPURIOUS_INTERRUPT(MCF_UART_UIMR(BASE) = reg_uart.uimr;)\
    }

/* Disable Receive Ready Interrupt. */
#define CLR_RXRDY_INTERRUPT() \
    {   \
        reg_uart.uimr &= ~MCF_UART_UIMR_FFULL_RXRDY;\
        PREVENT_SPURIOUS_INTERRUPT(MCF_UART_UIMR(BASE) = reg_uart.uimr;)\
    }

/* Disable All Interrupt. */
#define CLR_ALL_INTERRUPT() \
    {   \
        reg_uart.uimr = 0;\
        PREVENT_SPURIOUS_INTERRUPT(MCF_UART_UIMR(BASE) = 0;)\
    }

/*
 * Software Flow Control
 */
#ifdef XONXOFF

/*!
 * \brief Enables software flow control if not equal zero.
 */
static uint_fast8_t flow_control;

/* \brief ASCII code for software flow control, starts transmitter. */
#define ASCII_XON   0x11
/* \brief ASCII code for software flow control, stops transmitter. */
#define ASCII_XOFF  0x13

/* \brief XON transmit pending flag. */
#define XON_PENDING     0x10
/* \brief XOFF transmit pending flag. */
#define XOFF_PENDING    0x20
/* \brief XOFF sent flag. */
#define XOFF_SENT       0x40
/* \brief XOFF received flag. */
#define XOFF_RCVD       0x80
#endif

/*
 * Half Duplex Control
 */
#ifdef HDX_CTRL_PIN
    #ifdef HDX_CTRL_INV
        #define HDX_CTRL_FULL()     GpioPinSetHigh(HDX_CTRL_PORT, HDX_CTRL_PIN)
        #define HDX_CTRL_HALF()     GpioPinSetLow(HDX_CTRL_PORT, HDX_CTRL_PIN)
    #else
        #define HDX_CTRL_FULL()     GpioPinSetLow(HDX_CTRL_PORT, HDX_CTRL_PIN)
        #define HDX_CTRL_HALF()     GpioPinSetHigh(HDX_CTRL_PORT, HDX_CTRL_PIN)
    #endif
#else
    #define HDX_CTRL_FULL()
    #define HDX_CTRL_HALF()
#endif

#ifdef HDX_CTRL_BOARD_SPEC
        // TODO
#endif

/*
 * RS485 DE/RE Control
 */
#ifdef RS485_CTRL_DE_PIN
    #ifdef RS485_CTRL_DE_INV
        #define RS485_CTRL_TX_ENA() GpioPinSetLow(RS485_CTRL_DE_PORT, RS485_CTRL_DE_PIN)
        #define RS485_CTRL_TX_DIS() GpioPinSetHigh(RS485_CTRL_DE_PORT, RS485_CTRL_DE_PIN)
    #else
        #define RS485_CTRL_TX_ENA() GpioPinSetHigh(RS485_CTRL_DE_PORT, RS485_CTRL_DE_PIN)
        #define RS485_CTRL_TX_DIS() GpioPinSetLow(RS485_CTRL_DE_PORT, RS485_CTRL_DE_PIN)
    #endif
#else
    #define RS485_CTRL_TX_DIS()
    #define RS485_CTRL_TX_ENA()
#endif

#ifdef RS485_CTRL_RE_PIN
    #ifdef RS485_CTRL_RE_INV
        #define RS485_CTRL_RX_ENA() GpioPinSetHigh(RS485_CTRL_RE_PORT, RS485_CTRL_RE_PIN)
        #define RS485_CTRL_RX_DIS() GpioPinSetLow(RS485_CTRL_RE_PORT, RS485_CTRL_RE_PIN)
    #else
        #define RS485_CTRL_RX_ENA() GpioPinSetLow(RS485_CTRL_RE_PORT, RS485_CTRL_RE_PIN)
        #define RS485_CTRL_RX_DIS() GpioPinSetHigh(RS485_CTRL_RE_PORT, RS485_CTRL_RE_PIN)
    #endif
#else
    #define RS485_CTRL_RX_ENA()
    #define RS485_CTRL_RX_DIS()
#endif

#ifdef RS485_CTRL_BOARD_SPEC
    // TODO
#endif

/*!
 * \brief Receiver error flags.
 */
static uint8_t rx_errors;

/*!
 * \brief Enables half duplex control if not equal zero.
 */
static uint_fast8_t hdx_control;

#ifdef RTS_PIN
/*!
 * \brief Enables RTS control if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to control the RTS signal.
 */
static uint_fast8_t rts_control;
#endif

#ifdef CTS_PIN
/*!
 * \brief Enables CTS sense if not equal zero.
 *
 * This variable exists only if the hardware configuration defines a
 * port bit to sense the CTS signal.
 */
static uint_fast8_t cts_sense;
#endif
/*
 * \brief USARTn transmit complete interrupt handler.
 *
 * Used with half duplex communication to switch from tranmit to receive
 * mode after the last character has been transmitted.
 *
 * \param arg Pointer to the device specific control block.
 */
static void Mcf5UsartTxEmpty(void *arg)
{
    /*
     * Switch RS485 bus driver to receive mode
     */
    RS485_CTRL_TX_DIS();
    RS485_CTRL_RX_ENA();

    /*
     * Enable receiver
     */
    MCF_UART_UCR(BASE) = MCF_UART_UCR_RX_ENABLED;

    /*
     * Disable TX interrupt.
     */
    CLR_TXRDY_INTERRUPT();
}

/*
 * \brief USARTn transmit data register empty interrupt handler.
 *
 * \param arg Pointer to the device specific control block.
 */
static void Mcf5UsartTxReady(void *arg)
{
    register RINGBUF *rbf = &((USARTDCB *)arg)->dcb_tx_rbf;;
    register uint8_t *cp;

#ifdef XONXOFF
    /*
     * Process pending software flow controls first.
     */
    if (flow_control & (XON_PENDING | XOFF_PENDING)) {
        if (flow_control & XOFF_PENDING) {
            MCF_UART_UTB(BASE) = ASCII_XOFF;
            flow_control |= XOFF_SENT;
        } else {
            MCF_UART_UTB(BASE) = ASCII_XON;
            flow_control &= ~XOFF_SENT;
        }
        flow_control &= ~(XON_PENDING | XOFF_PENDING);
        return;
    }

    if (flow_control & XOFF_RCVD) {
        /*
         * If XOFF has been received, we disable the transmit interrupts
         * and return without sending anything.
         */
        CLR_TXRDY_INTERRUPT();
        return;
    }
#endif

    /*
     * Prepare next character for transmitting
     */
    if (rbf->rbf_cnt) {

        /*
         * Previous character transmitted successfully
         */
        rbf->rbf_cnt--;

        /*
         * Start transmission of the next character and clear TXRDY bit
         * in USR register.
         */
        cp = rbf->rbf_tail;
        MCF_UART_UTB(BASE) = *cp;

        /*
         * Wrap around the buffer pointer if we reached its end.
         */
        if (++cp == rbf->rbf_last) {
            cp = rbf->rbf_start;
        }
        rbf->rbf_tail = cp;

        /*
         * Wakeup waiting thread when tx buffer low watermark is reached
         */
        if (rbf->rbf_cnt == rbf->rbf_lwm) {
            NutEventPostFromIrq(&rbf->rbf_que);
            NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_WRITE);
        }
    }

    if (rbf->rbf_cnt == 0){
        /*
         * Nothing left to transmit, wakeup waiting thread
         */
        NutEventPostFromIrq(&rbf->rbf_que);
        NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_WRITE);

        /*
         * Disable TX interrupt in full-duplex mode.
         * In half-duplex mode, TX interrupt will be disabled after last byte
         * will be completely shifted out (Mcf5UsartTxEmpty)
         */
        if (!hdx_control){
            CLR_TXRDY_INTERRUPT();
        }
    }
}

/*
 * \brief USARTn receive complete interrupt handler.
 *
 * \param arg Pointer to the device specific control block.
 */
static void Mcf5UsartRxComplete(void *arg) {
    register RINGBUF *rbf = &((USARTDCB *)arg)->dcb_rx_rbf;
    register size_t cnt;
    register uint8_t ch;
    uint_fast8_t usr;
    uint_fast8_t postEvent = 0;

    /*
     * Receive all bytes from RxFIFO
     */
    do {
        /*
         * Reading the error flags must come first, because reading
         * the data register clears the status.
         */
        usr = MCF_UART_USR(BASE);

       /* Record UART Errors  */
        rx_errors |= usr;

#ifdef RTS_PIN
        /*
         * Stop receiving if RX buffer is full.
         * After UART's RxFIFO went full, RTS signal will be automatically asserted.
         */
        if (rbf->rbf_cnt >= rbf->rbf_siz) {
            if (rts_control) {
                CLR_RXRDY_INTERRUPT();
                return;
            }
        }
#endif

       /* Receive char from Rx FIFO */
        ch = MCF_UART_URB(BASE);

       /* Reset TX errors. */
       if (usr & MCF_UART_USR_OE) {
           MCF_UART_UCR(BASE) = MCF_UART_UCR_RESET_ERROR;
       }

#ifdef XONXOFF
        /*
         * Handle software handshake. We have to do this before checking the
         * buffer, because flow control must work in write-only mode, where
         * there is no receive buffer.
         */
        if (flow_control) {
            /* XOFF character disables transmit interrupts. */
            if (ch == ASCII_XOFF) {
                CLR_TXRDY_INTERRUPT();
                flow_control |= XOFF_RCVD;
                return;
            }
            /* XON enables transmit interrupts. */
            else if (ch == ASCII_XON) {
                SET_TXRDY_INTERRUPT();
                flow_control &= ~XOFF_RCVD;
                return;
            }
        }
#endif
        /*
         * Check buffer overflow.
         */
        cnt = rbf->rbf_cnt;
        if (cnt >= rbf->rbf_siz) {
            rx_errors |= MCF_UART_USR_OE;
            return;
        }

        /*
         * Wake up waiting threads if this is the first byte in the buffer.
         */
        if (cnt++ == 0){
            postEvent = 1;
        }

#ifdef XONXOFF
        /*
         * Check the high watermark for software handshake. If the number of
         * buffered bytes is above this mark, then send XOFF.
         */
        else if (flow_control) {
            if(cnt >= rbf->rbf_hwm) {
                if((flow_control & XOFF_SENT) == 0) {
                    if (MCF_UART_USR(BASE) & MCF_UART_USR_TXRDY) {
                        MCF_UART_UTB(BASE) = ASCII_XOFF;
                        flow_control |= XOFF_SENT;
                        flow_control &= ~XOFF_PENDING;
                    } else {
                        flow_control |= XOFF_PENDING;
                    }
                }
            }
        }
#endif
        /*
         * Store the character and increment and the ring buffer pointer.
         */
        *rbf->rbf_head++ = ch;
        if (rbf->rbf_head == rbf->rbf_last) {
            rbf->rbf_head = rbf->rbf_start;
        }

        /*
         * Update the ring buffer counter.
         */
        rbf->rbf_cnt = cnt;

    } while ( MCF_UART_USR(BASE) & MCF_UART_USR_RXRDY );

    /*
     *  Wakeup waiting threads
     */
    if (postEvent) {
        NutEventPostFromIrq(&rbf->rbf_que);
        NutSelectWakeupFromIrq(rbf->wq_list, WQ_FLAG_READ);
    }
}

/*!
 * \brief USART interrupt handler.
 *
 * \param arg Pointer to the device specific control block.
 */
static void Mcf5UsartInterrupts(void *arg)
{
    if (MCF_UART_USR(BASE) & MCF_UART_USR_RXRDY) {
        Mcf5UsartRxComplete(arg);
    }

    if (MCF_UART_USR(BASE) & MCF_UART_USR_TXRDY) {
        Mcf5UsartTxReady(arg);
    }

    if ((MCF_UART_USR(BASE) & MCF_UART_USR_TXEMP) && (hdx_control) && (((USARTDCB *) arg)->dcb_tx_rbf.rbf_cnt == 0)) {
        Mcf5UsartTxEmpty(arg);
    }
}

/*!
 * \brief Carefully enable USART hardware functions.
 *
 * Always enable transmitter and receiver, even on read-only or
 * write-only mode. So we can support software flow control.
 */
static void Mcf5UsartEnable(void)
{
    NutEnterCritical();

    MCF_UART_UCR(BASE) = MCF_UART_UCR_TX_ENABLED;
    MCF_UART_UCR(BASE) = MCF_UART_UCR_RX_ENABLED;

    SET_TXRDY_INTERRUPT();
    SET_RXRDY_INTERRUPT();

    NutExitCritical();
}

/*!
 * \brief Carefully disable USART hardware functions.
 */
static void Mcf5UsartDisable(void)
{
    CLR_ALL_INTERRUPT();

    /*
     * Allow incoming or outgoing character to finish.
     */
    NutDelay(10);

    /*
     * Disable USART transmit and receive.
     */
    MCF_UART_UCR(BASE) = MCF_UART_UCR_TX_DISABLED;
    MCF_UART_UCR(BASE) = MCF_UART_UCR_RX_DISABLED;
}

/*!
 * \brief Query the USART hardware for the selected speed.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The currently selected baudrate.
 */
static uint32_t Mcf5UsartGetSpeed(void)
{
    uint16_t sv;

    sv = reg_uart.ubg1 << 8 | reg_uart.ubg2;
    return NutGetCpuClock() / (32 * sv);
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
static int Mcf5UsartSetSpeed(uint32_t rate)
{
    uint16_t sv;

    sv = (uint16_t) (NutGetCpuClock() / (rate * 32));
    reg_uart.ubg1 = (uint8_t) ((sv & 0xFF00) >> 8);
    reg_uart.ubg2 = (uint8_t) (sv & 0x00FF);

    Mcf5UsartDisable();
    MCF_UART_UBG1(BASE) = reg_uart.ubg1;
    MCF_UART_UBG2(BASE) = reg_uart.ubg2;
    Mcf5UsartEnable();

    return 0;
}

/*!
 * \brief Query the USART hardware for the number of data bits.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The number of data bits set.
 */
static uint8_t Mcf5UsartGetDataBits(void)
{
    return (reg_uart.umr1 & 0x3) + 5;
}

/*!
 * \brief Set the USART hardware to the number of data bits.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5UsartSetDataBits(uint8_t bits)
{
    reg_uart.umr1 &= ~MCF_UART_UMR_BC(0x3);

    switch (bits) {
    case 5:
        reg_uart.umr1 |= MCF_UART_UMR_BC_5;
        break;
    case 6:
        reg_uart.umr1 |= MCF_UART_UMR_BC_6;
        break;
    case 7:
        reg_uart.umr1 |= MCF_UART_UMR_BC_7;
        break;
    case 8:
        reg_uart.umr1 |= MCF_UART_UMR_BC_8;
        break;
    }

    Mcf5UsartDisable();
    MCF_UART_UCR(BASE) = MCF_UART_UCR_RESET_MR;
    MCF_UART_UMR(BASE) = reg_uart.umr1;
    Mcf5UsartEnable();

    /*
     * Verify the result.
     */
    if (Mcf5UsartGetDataBits() != bits) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Query the USART hardware for the parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return Parity mode, either 0 (disabled), 1 (odd) or 2 (even).
 */
static uint8_t Mcf5UsartGetParity(void)
{
    switch (reg_uart.umr1 & 0x1C) {
    case MCF_UART_UMR_PM_NONE:
        return 0;
    case MCF_UART_UMR_PM_ODD:
        return 1;
    case MCF_UART_UMR_PM_EVEN:
        return 2;
    }

    return -1;
}

/*!
 * \brief Set the USART hardware to the specified parity mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param mode 0 (none), 1 (odd) or 2 (even).
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5UsartSetParity(uint8_t mode)
{
    reg_uart.umr1 &= ~MCF_UART_UMR_PM(0x3);

    switch (mode) {
    case 0:
        reg_uart.umr1 |= MCF_UART_UMR_PM_NONE;
        break;
    case 1:
        reg_uart.umr1 |= MCF_UART_UMR_PM_ODD;
        break;
    case 2:
        reg_uart.umr1 |= MCF_UART_UMR_PM_EVEN;
        break;
    }
    Mcf5UsartDisable();
    MCF_UART_UCR(BASE) = MCF_UART_UCR_RESET_MR;
    MCF_UART_UMR(BASE) = reg_uart.umr1;
    Mcf5UsartEnable();

    /*
     * Verify the result.
     */
    if (Mcf5UsartGetParity() != mode) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Query the USART hardware for the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return The number of stop bits set, either 1 or 2.
 */
static uint8_t Mcf5UsartGetStopBits(void)
{
    switch (reg_uart.umr2 & 0xF) {
    case MCF_UART_UMR_SB_STOP_BITS_1:
        return 1;
    case MCF_UART_UMR_SB_STOP_BITS_2:
        return 2;
    case MCF_UART_UMR_SB_STOP_BITS_15:
        return 15;
    }
    return -1;
}

/*!
 * \brief Set the USART hardware to the number of stop bits.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \param stop bits 1, 2 or 15 (1,5).
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5UsartSetStopBits(uint8_t bits)
{
    reg_uart.umr2 &= ~0xF;

    switch (bits) {
    case 1:
        reg_uart.umr2 |= MCF_UART_UMR_SB_STOP_BITS_1;
        break;
    case 2:
        reg_uart.umr2 |= MCF_UART_UMR_SB_STOP_BITS_2;
        break;
    case 15:
        reg_uart.umr2 |= MCF_UART_UMR_SB_STOP_BITS_15;
        break;
    }

    Mcf5UsartDisable();
    MCF_UART_UMR(BASE) = reg_uart.umr2;
    Mcf5UsartEnable();

    /*
     * Verify the result.
     */
    if (Mcf5UsartGetStopBits() != bits) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Query the USART hardware status.
 *
 * \return Status flags.
 */
static uint32_t Mcf5UsartGetStatus(void)
{
    uint32_t rc = 0;

    /*
     * Set receiver error flags.
     */
    if (rx_errors & MCF_UART_USR_FE) {
        rc |= UART_FRAMINGERROR;
    }
    if (rx_errors & MCF_UART_USR_OE) {
        rc |= UART_OVERRUNERROR;
    }
    if (rx_errors & MCF_UART_USR_PE) {
        rc |= UART_PARITYERROR;
    }

#ifdef XONXOFF
    /*
     * Determine software handshake status. The flow control status may
     * change during interrupt, but this doesn't really hurt us.
     */
    if (flow_control) {
        if (flow_control & XOFF_SENT) {
            rc |= UART_RXDISABLED;
        }
        if (flow_control & XOFF_RCVD) {
            rc |= UART_TXDISABLED;
        }
    }
#endif

#ifdef RTS_PIN
    /*
     * Determine hardware handshake control status.
     */
    if (GpioPinGet(RTS_PORT, RTS_PIN)) {
        rc |= UART_RTSDISABLED;
        if (rts_control) {
            rc |= UART_RXDISABLED;
        }
    } else {
        rc |= UART_RTSENABLED;
    }
#endif

#ifdef CTS_PIN
    /*
     * Determine hardware handshake sense status.
     */
    if (MCF_UART_UIP(BASE) & MCF_UART_UIP_CTS) {
        rc |= UART_CTSDISABLED;
        if (cts_sense) {
            rc |= UART_RXDISABLED;
        }
    } else {
        rc |= UART_CTSENABLED;
    }
#endif

    /*
     * If transmitter and receiver haven't been detected disabled by any
     * of the checks above, then they are probably enabled.
     */
    if ((rc & UART_RXDISABLED) == 0) {
        rc |= UART_RXENABLED;
    }
    if ((rc & UART_TXDISABLED) == 0) {
        rc |= UART_TXENABLED;
    }

    return rc;
}

/*!
 * \brief Set the USART hardware status.
 *
 * \param flags Status flags.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5UsartSetStatus(uint32_t flags)
{
    /*
     * Clear UART receive errors.
     */
    if (flags & UART_FRAMINGERROR) {
        rx_errors &= ~MCF_UART_USR_FE;
    }
    if (flags & UART_OVERRUNERROR) {
        MCF_UART_UCR(BASE) = MCF_UART_UCR_RESET_ERROR;
        rx_errors &= ~MCF_UART_USR_OE;
    }
    if (flags & UART_PARITYERROR) {
        rx_errors &= ~MCF_UART_USR_PE;
    }

#ifdef XONXOFF
    /*
     * Process software handshake control.
     */
    if (flow_control) {

        /* Access to the flow control status must be atomic. */
        NutEnterCritical();

        /*
         * Enabling or disabling the receiver means to behave like
         * having sent a XON or XOFF character resp.
         */
        if (flags & UART_RXENABLED) {
            flow_control &= ~XOFF_SENT;
        } else if (flags & UART_RXDISABLED) {
            flow_control |= XOFF_SENT;
        }

        /*
         * Enabling or disabling the transmitter means to behave like
         * having received a XON or XOFF character resp.
         */
        if (flags & UART_TXENABLED) {
            flow_control &= ~XOFF_RCVD;
        } else if (flags & UART_TXDISABLED) {
            flow_control |= XOFF_RCVD;
        }
        NutExitCritical();
    }
#endif

    /*
     *  Read only or not supported flags
     */
    if (flags & (
#ifndef XONXOFF
            UART_RXENABLED | UART_RXDISABLED | UART_TXENABLED | UART_TXDISABLED |
#endif
            UART_RTSENABLED | UART_RTSDISABLED | UART_CTSENABLED | UART_CTSDISABLED |
            UART_RXADDRFRAME | UART_RXNORMFRAME | UART_TXADDRFRAME | UART_TXNORMFRAME |
            UART_DTRENABLED | UART_DTRDISABLED ))
        return -1;

    return 0;
}

/*!
 * \brief Query flow control mode.
 *
 * This routine is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 *
 * \return See UsartIOCtl().
 */
static uint32_t Mcf5UsartGetFlowControl(void)
{
    uint32_t rc = 0;

#ifdef XONXOFF
    if (flow_control) {
        rc |= USART_MF_XONXOFF;
    } else {
        rc &= ~USART_MF_XONXOFF;
    }
#endif

#ifdef RTS_PIN
    if (rts_control) {
        rc |= USART_MF_RTSCONTROL;
    } else {
        rc &= ~USART_MF_RTSCONTROL;
    }
#endif

#ifdef CTS_PIN
    if (cts_sense) {
        rc |= USART_MF_CTSSENSE;
    } else {
        rc &= ~USART_MF_CTSSENSE;
    }
#endif

    if (hdx_control) {
        rc |= USART_MF_HALFDUPLEX;
    } else {
        rc &= ~USART_MF_HALFDUPLEX;
    }

    return rc;
}

/*!
 * \brief Set flow control mode.
 *
 * This function is called by ioctl function of the upper level USART
 * driver through the USARTDCB jump table.
 * Nove pridan parametr USART_MF_HALFDUPLEX_YZ, ktery se vklada spolecne
 * s parametrem USART_MF_HALFDUPLEX. Pokud USART_MF_HALFDUPLEX_YZ je 
 * zadan komonikuje se pres porty XY, pokud neni tak pres AB.
 *
 * \param flags See UsartIOCtl().
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5UsartSetFlowControl(uint32_t flags)
{
#ifdef XONXOFF
    /*
     * Set software handshake mode.
     */
    if (flags & USART_MF_XONXOFF) {
        if(!flow_control) {
            NutEnterCritical();
            flow_control = 1 | XOFF_SENT;  /* force XON to be sent on next read */
            NutExitCritical();
        }
    } else if (flow_control){
        NutEnterCritical();
        flow_control = 0;
        NutExitCritical();
    }
#endif

#ifdef RTS_PIN
    /*
     * Set hardware RTS control mode.
     */
    if (flags & USART_MF_RTSCONTROL) {
        rts_control = 1;
        reg_uart.umr1 |= MCF_UART_UMR_RXRTS;
        MCF_UART_UCR(BASE) = MCF_UART_UCR_RESET_MR;
        MCF_UART_UMR(BASE) = reg_uart.umr1;
    } else {
        rts_control = 0;
        reg_uart.umr1 &= ~MCF_UART_UMR_RXRTS;
        MCF_UART_UCR(BASE) = MCF_UART_UCR_RESET_MR;
        MCF_UART_UMR(BASE) = reg_uart.umr1;
    }
#endif

#ifdef CTS_PIN
    /*
     * Set hardware CTS sense mode.
     */
    if (flags & USART_MF_CTSSENSE) {
        cts_sense = 1;
        reg_uart.umr2 |= MCF_UART_UMR_TXCTS;
        MCF_UART_UMR(BASE) = reg_uart.umr2;
    } else {
        cts_sense = 0;
        reg_uart.umr2 &= ~MCF_UART_UMR_TXCTS;
        MCF_UART_UMR(BASE) = reg_uart.umr2;
    }
#endif

    /*
     * Set duplex mode.
     */
    if (flags & USART_MF_HALFDUPLEX) {
        hdx_control = 1;
        /* Switch RS485 bus driver to half duplex */
        HDX_CTRL_HALF();
    } else {
        hdx_control = 0;
        /* Switch RS485 bus driver to receive/transmit mode */
        HDX_CTRL_FULL();
        RS485_CTRL_RX_ENA();
        RS485_CTRL_TX_ENA();
        /* Enable receiver */
        MCF_UART_UCR(BASE) = MCF_UART_UCR_RX_ENABLED;
    }

    /*
     * Verify the result.
     */
    if (Mcf5UsartGetFlowControl() != flags) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Start the USART transmitter hardware.
 *
 * The upper level USART driver will call this function through the
 * USARTDCB jump table each time it added one or more bytes to the
 * transmit buffer.
 */
static void Mcf5UsartTxStart(void)
{
    if (hdx_control) {
        /*
         * Disable receiver in half-duplex mode
         */
        MCF_UART_UCR(BASE) = MCF_UART_UCR_RX_DISABLED;

        /*
         * Switch RS485 bus driver to transmit mode
         */
        RS485_CTRL_RX_DIS();
        RS485_CTRL_TX_ENA();
    }

    /* Enable Transmit Ready Interrupt */
    SET_TXRDY_INTERRUPT();
}

/*!
 * \brief Start the USART receiver hardware.
 *
 * The upper level USART driver will call this function through the
 * USARTDCB jump table each time it removed enough bytes from the
 * receive buffer. Enough means, that the number of bytes left in
 * the buffer is below the low watermark.
 */
static void Mcf5UsartRxStart(void)
{
#ifdef XONXOFF
    /*
     * Do any required software flow control.
     */
    if (flow_control && (flow_control & XOFF_SENT)) {
        NutEnterCritical();
        if (MCF_UART_USR(BASE) & MCF_UART_USR_TXRDY) {
            MCF_UART_UTB(BASE) = ASCII_XON;
            flow_control &= ~XON_PENDING;
        } else {
            flow_control |= XON_PENDING;
        }
        flow_control &= ~(XOFF_SENT | XOFF_PENDING);
        NutExitCritical();
    }
#endif

    /*
     * Enable receive interrupt. It could be disabled by flow control.
     */
    SET_RXRDY_INTERRUPT();
}

/*
 * \brief Initialize the USART hardware driver.
 *
 * This function is called during device registration by the upper level
 * USART driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5UsartInit(void)
{
    int result = 0;

    /*
     * Disable UART's interrupts
     */
    CLR_ALL_INTERRUPT();

    /*
     * UART Reset
     */
    MCF_UART_UCR(BASE) = MCF_UART_UCR_RESET_TX;
    MCF_UART_UCR(BASE) = MCF_UART_UCR_RESET_RX;
    MCF_UART_UCR(BASE) = MCF_UART_UCR_RESET_MR;

    /*
     * Initialize UART's write only registers
     */
    memset(&reg_uart, 0, sizeof(reg_uart));

    /*
     *  GPIO Configuration
     */
    result |= GpioPinConfigSet(RXD_PORT, RXD_PIN, RXD_PERIPHERAL);
    result |= GpioPinConfigSet(TXD_PORT, TXD_PIN, TXD_PERIPHERAL);
#ifdef RTS_PIN
    result |= GpioPinConfigSet(RTS_PORT, RTS_PIN, RTS_PERIPHERAL);
#endif
#ifdef CTS_PIN
    result |= GpioPinConfigSet(CTS_PORT, CTS_PIN, CTS_PERIPHERAL);
#endif
#ifdef HDX_CTRL_PIN
    result |= GpioPinConfigSet(HDX_CTRL_PORT, HDX_CTRL_PIN, GPIO_CFG_OUTPUT);
#endif
#ifdef RS485_CTRL_DE_PIN
    result |= GpioPinConfigSet(RS485_CTRL_DE_PORT, RS485_CTRL_DE_PIN, GPIO_CFG_OUTPUT);
#endif
#ifdef RS485_CTRL_RE_PIN
    result |= GpioPinConfigSet(RS485_CTRL_RE_PORT, RS485_CTRL_RE_PIN, GPIO_CFG_OUTPUT);
#endif

    /*
     * Use internal bus clock as the clock source for Rx and Tx
     */
    MCF_UART_UCSR(BASE) = MCF_UART_UCSR_RCS_SYS_CLK | MCF_UART_UCSR_TCS_SYS_CLK;

    /*
     * UART Configuration
     */

    result |= Mcf5UsartSetFlowControl(0
#ifdef HDX_ENABLED
                 | USART_MF_HALFDUPLEX
#endif
                 );
    result |= Mcf5UsartSetDataBits(8);
    result |= Mcf5UsartSetStopBits(1);
    result |= Mcf5UsartSetParity(0);
    result |= Mcf5UsartSetSpeed(USART_INITSPEED);

    /*
     * Trigger interrupt when /CTS has change of state
     */
    MCF_UART_UACR(BASE) = MCF_UART_UACR_IEC;

    /*
     * Disable UART's interrupts
     */
    CLR_ALL_INTERRUPT();

    /*
     * Register and enable Interrupt handler
     */
    if (result
     || NutRegisterIrqHandler(&sig_uart, Mcf5UsartInterrupts, &dcb_uart)
     || NutIrqEnable(&sig_uart))
        return -1;

    /*
     * Start receiving immediatelly
     * NOTE: Transmitting is started too, but it is stopped after while in Mcf5UsartTxReady()
     */
    Mcf5UsartEnable();

    return 0;
}

/*
 * \brief Deinitialize the USART hardware driver.
 *
 * This function is called during device deregistration by the upper
 * level USART driver through the USARTDCB jump table.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Mcf5UsartDeinit(void)
{
    /* Deregister receive and transmit interrupts. */
    NutIrqDisable(&sig_uart);

    /* Deregister receive and transmit interrupts. */
    NutRegisterIrqHandler(&sig_uart, 0, 0);

    return 0;
}

/*@}*/
