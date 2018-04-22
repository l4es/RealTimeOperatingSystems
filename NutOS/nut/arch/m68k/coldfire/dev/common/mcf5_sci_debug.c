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

#include <dev/debug.h>
#include <dev/gpio.h>

/*!
 * \brief Handle I/O controls for the debug device.
 *
 * \param dev  Identifies the device that receives the device-control
 *             request.
 * \param req  Requested control function.
 * \param conf Points to a buffer that contains any data required for
 *             the given control function or receives data from that
 *             function.
 *
 * \return 0 on success, -1 if the function fails or is not available.
 */
static int IOCtl(NUTDEVICE * dev, int req, void *conf)
{
    return -1;
}

/*!
 * \brief Send a single character to debug device.
 *
 * The function will automatically prepend any newline character
 * (ASCII 10) with a carriage return character (ASCII 13).
 *
 * \param ch The character to send.
 */
static void Put(uintptr_t devnum, char ch)
{
    /* Prepend NL with CR. */
    if (ch == '\n') {
        Put(devnum, '\r');
    }

    /* Wait until space is available in the FIFO */
//    while (!(MCF_UART_USR(devnum) & MCF_UART_USR_TXRDY))
//        ;

    /* Send the character */
//    MCF_UART_UTB(devnum) = (uint8_t) ch;
}

/*!
 * \brief Send a buffer contents to the debug device.
 *
 * This function is called by the low level input routines of the
 * \ref xrCrtLowio "C runtime library", using the _NUTDEVICE::dev_read
 * entry.
 *
 * Applications must not call this function, but use the stdio
 * functions instead.
 *
 * The debug driver is not interrupt driven and has no internal buffer
 * and no timeout control. Thus, it can be safely used in interrupt routines.
 * Of course, this will significantly decrease interrupt latency and overall
 * system performance.
 *
 * \param fp     Identifies the device to send to. This may be used by
 *               the driver to retrieve the \ref NUTDEVICE pointer.
 * \param buffer Pointer to the data to be written.
 * \param len    Number of characters to write. If 0, then the caller
 *               requests to flush the drivers internal output buffer.
 *               In this case the pointer to the data is ignored.
 *
 * \return The number of characters written, which is the same as the
 *         number of characters specified.
 */
static int Write(NUTFILE * fp, const void *buffer, int len)
{
    int c = len;
    const char *cp = (const char *) buffer;
    uintptr_t devnum = fp->nf_dev->dev_base;

    while (c--) {
        Put(devnum, *cp++);
    }

    return len;
}

/*!
 * \brief Read bytes from file
 *
 * \return Number of characters read.
 */
static int Read(NUTFILE * nf, void *buffer, int len)
{
    return 0;
}

/*!
 * \brief Open debug device.
 *
 * \return Pointer to a static NUTFILE structure.
 */
static NUTFILE *Open(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    NUTFILE *fp = (NUTFILE *) (dev->dev_dcb);

    fp->nf_dev = dev;
    fp->nf_fcb = NULL;

    return fp;
}

/*!
 * \brief Close debug device.
 *
 * \param fp Pointer to a \ref _NUTFILE structure, obtained by a
 *           previous call to ZeroDebugOpen().
 *
 * \return Always 0.
 */
static int Close(NUTFILE * fp)
{
    /* Nothing to do for this simple driver. */
    return 0;
}

/*!
 * \brief Initialize debug device.
 *
 * This function is called by NutRegisterDevice(), using the
 * _NUTDEVICE::dev_init entry.
 *
 * Applications must not call this function, they must call
 * NutRegisterDevice() instead.
 *
 * \param dev Pointer to the device information structure.
 *
 * \return Always 0.
 */
static int Init(NUTDEVICE * dev)
{
    uint8_t devnum = dev->dev_base;
//    uint32_t sysclk = NutGetCpuClock();
//    int baud = 115200;
//    register uint16_t ubgs;

    /* Enable SCI Port on GPIO */
    switch (devnum) {
    case 1:
        GpioPinConfigSet(SCI1_TXD_PORT, SCI1_TXD_PIN, SCI1_TXD_PERIPHERAL);
        GpioPinConfigSet(SCI1_RXD_PORT, SCI1_RXD_PIN, SCI1_RXD_PERIPHERAL);
        break;

    case 2:
        GpioPinConfigSet(SCI2_TXD_PORT, SCI2_TXD_PIN, SCI2_TXD_PERIPHERAL);
        GpioPinConfigSet(SCI2_RXD_PORT, SCI2_RXD_PIN, SCI2_RXD_PERIPHERAL);
        break;

    case 3:
        GpioPinConfigSet(SCI3_TXD_PORT, SCI3_TXD_PIN, SCI3_TXD_PERIPHERAL);
        GpioPinConfigSet(SCI3_RXD_PORT, SCI3_RXD_PIN, SCI3_RXD_PERIPHERAL);
        break;

    default:
        return 0;
    }

    /* Reset Transmitter */
//    MCF_UART_UCR(devnum) = MCF_UART_UCR_RESET_TX;

    /* Reset Receiver */
//    MCF_UART_UCR(devnum) = MCF_UART_UCR_RESET_RX;

    /* Reset Mode Register */
//    MCF_UART_UCR(devnum) = MCF_UART_UCR_RESET_MR;

    /* No parity, 8-bits per character */
//    MCF_UART_UMR(devnum) = MCF_UART_UMR_PM_NONE | MCF_UART_UMR_BC_8;

    /* Normal mode(No echo or loopback), 1 stop bit */
//    MCF_UART_UMR(devnum) = MCF_UART_UMR_CM_NORMAL | MCF_UART_UMR_SB_STOP_BITS_1;

    /* Select Rx and Tx clocks */
//    MCF_UART_UCSR(devnum) = MCF_UART_UCSR_RCS_SYS_CLK | MCF_UART_UCSR_TCS_SYS_CLK;

    /* Calculate baud settings */
//    ubgs = (uint16_t)(sysclk / (baud * 32));

//    MCF_UART_UBG1(devnum) = (uint8_t)((ubgs & 0xFF00) >> 8);
//    MCF_UART_UBG2(devnum) = (uint8_t)(ubgs & 0x00FF);

    /* Enable receiver and transmitter */
//    MCF_UART_UCR(devnum) = MCF_UART_UCR_TX_ENABLED | MCF_UART_UCR_RX_ENABLED;

    return 0;
}

/*
 * While most drivers allocate this structure from the heap during
 * the open call and release it during close, this driver uses a
 * static structure. Therefore, concurrent open calls are not
 * allowed.
 */
static NUTFILE dbgfile;

/*!
 * \brief Debug device information structure.
 *
 * Usually, the device structure is the only public symbol that may be
 * referenced by the application code using
 *
 * \code
 * #include <dev/debug.h>
 *
 * {
 *     ...
 *     NutRegisterDevice(&devDebug0, 0, 0);
 *     ...
 * }
 * \endcode
 *
 * If not referenced, the driver code (and this structure) will not be
 * included in the final binary.
 *
 * The name of the structure may differ among platforms. Portable
 * applications should avoid it and instead make use of dev/board.h.
 *
 * \code
 * #include <dev/board.h>
 *
 * {
 *     ...
 *     NutRegisterDevice(&DEV_DEBUG, 0, 0);
 *     ...
 * }
 * \endcode
 *
 * While output is supported by default, input may be not. If input is
 * required, applications may replace \ref DEV_DEBUG by \ref DEV_CONSOLE.
 * In this case the debug driver is selected only, if it has input
 * capability (see \ref NUT_DEV_DEBUG_READ). Otherwise an interrupt
 * driven SCI driver will be used.
 *
 * Note, that this polling driver has certain advantages
 * - very low memory usage
 * - allows stdio output functions in interrupt context
 * - allows stdio output functions in early system stage
 * - no internal buffering, output is synchronous
 * - atomic output with multiple threads
 *
 * but also some disadvantages
 * - concurrent threads are blocked during output
 * - most or all SCI settings are hard coded
 * - may not work with non-ASCII (binary) data
 * - often only output is supported, not input
 * - only one instance (open) is allowed
 *
 * When used with Harvard architectures, additional functions may
 * be offered to access data in program space.
 */
NUTDEVICE devDebug1 = {
        NULL,           /*!< _NUTDEVICE::dev_next, must be NULL */
        { 's', 'c', 'i', '1', 0, 0, 0, 0, 0 }, /*!< _NUTDEVICE::dev_name, use for all sci1 drivers. */
        IFTYP_CHAR,     /*!< _NUTDEVICE::dev_type, probably not used, may be 0. */
        1,              /*!< _NUTDEVICE::dev_base, device number = 2(COM2). */
        0,              /*!< _NUTDEVICE::dev_irq, not used by this driver. */
        NULL,           /*!< _NUTDEVICE::dev_icb, not used by this driver. */
        &dbgfile,       /*!< _NUTDEVICE::dev_dcb, stores the \ref NUTFILE handle. */
        Init,           /*!< _NUTDEVICE::dev_init. */
        IOCtl,          /*!< _NUTDEVICE::dev_ioctl. */
        Read,           /*!< _NUTDEVICE::dev_read, optional, may be NULL. */
        Write,          /*!< _NUTDEVICE::dev_write. */
        Open,           /*!< _NUTDEVICE::dev_open. */
        Close,          /*!< _NUTDEVICE::dev_close. */
        NULL            /*!< _NUTDEVICE::dev_size, optional, may be NULL. */
        NULL,           /*!< _NUTDEVICE::dev_select, optional, may be NULL */
};

NUTDEVICE devDebug2 = {
        NULL,           /*!< _NUTDEVICE::dev_next, must be NULL */
        { 's', 'c', 'i', '2', 0, 0, 0, 0, 0 }, /*!< _NUTDEVICE::dev_name, use for all sci2 drivers. */
        IFTYP_CHAR,     /*!< _NUTDEVICE::dev_type, probably not used, may be 0. */
        2,              /*!< _NUTDEVICE::dev_base, device number = 2(COM2). */
        0,              /*!< _NUTDEVICE::dev_irq, not used by this driver. */
        NULL,           /*!< _NUTDEVICE::dev_icb, not used by this driver. */
        &dbgfile,       /*!< _NUTDEVICE::dev_dcb, stores the \ref NUTFILE handle. */
        Init,           /*!< _NUTDEVICE::dev_init. */
        IOCtl,          /*!< _NUTDEVICE::dev_ioctl. */
        Read,           /*!< _NUTDEVICE::dev_read, optional, may be NULL. */
        Write,          /*!< _NUTDEVICE::dev_write. */
        Open,           /*!< _NUTDEVICE::dev_open. */
        Close,          /*!< _NUTDEVICE::dev_close. */
        NULL            /*!< _NUTDEVICE::dev_size, optional, may be NULL. */
        NULL,           /*!< _NUTDEVICE::dev_select, optional, may be NULL */
};

NUTDEVICE devDebug3 = {
        NULL,           /*!< _NUTDEVICE::dev_next, must be NULL */
        { 's', 'c', 'i', '3', 0, 0, 0, 0, 0 }, /*!< _NUTDEVICE::dev_name, use for all sci3 drivers. */
        IFTYP_CHAR,     /*!< _NUTDEVICE::dev_type, probably not used, may be 0. */
        3,              /*!< _NUTDEVICE::dev_base, device number = 2(COM2). */
        0,              /*!< _NUTDEVICE::dev_irq, not used by this driver. */
        NULL,           /*!< _NUTDEVICE::dev_icb, not used by this driver. */
        &dbgfile,       /*!< _NUTDEVICE::dev_dcb, stores the \ref NUTFILE handle. */
        Init,           /*!< _NUTDEVICE::dev_init. */
        IOCtl,          /*!< _NUTDEVICE::dev_ioctl. */
        Read,           /*!< _NUTDEVICE::dev_read, optional, may be NULL. */
        Write,          /*!< _NUTDEVICE::dev_write. */
        Open,           /*!< _NUTDEVICE::dev_open. */
        Close,          /*!< _NUTDEVICE::dev_close. */
        NULL            /*!< _NUTDEVICE::dev_size, optional, may be NULL. */
        NULL,           /*!< _NUTDEVICE::dev_select, optional, may be NULL */
};
