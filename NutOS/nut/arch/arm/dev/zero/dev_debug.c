/*
 * Copyright (C) 2011 by egnite GmbH
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

/*!
 * \file arch/arm/dev/zero/dev_debug.c
 * \brief Debug output device for Zero CPU.
 *
 * This code implements a polling UART driver for an imaginary CPU.
 * It may serve as a template when porting Nut/OS to a new target.
 * Implementers should look out for 'TODO' comments.
 *
 * We intentionally do not check, whether the parameters passed to
 * any function of this driver are valid, using NUTASSERT or similar.
 * There are two main reasons:
 * - The driver code is quite simple (and should be kept simple)
 * - If an error occurs, this debug device will be used to report it
 *
 * If you choose add more functions to this driver, avoid Nut/OS API
 * calls, or make sure, that these calls will not introduce a context
 * switch or allocate heap memory. The read function may be an exception,
 * if its use is limited to normal application context.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/* The Configurator will place UART specific configuration parameters
   in nutbld/include/cfg/uart.h. If nothing specific has been configured,
   the compiler will include the original nut/include/cfg/uart.h. */
#include <cfg/uart.h>

/*
 * TODO: Include architecture specific header files, if required.
 *
 * To implement your UART hardware, you need to access several
 * hardware registers and therefore need to include the header files,
 * where they are defined. However, if properly configured, this is usually
 * done automatically via compiler.h, which in turn is included almost
 * anywhere.
 *
 * Sometimes the deep level of included headers may become tricky to
 * follow or something may get broken by later changes. It is legitimate
 * to include essential header files, even if they are already included
 * indirectly by other header files.
 */

/*
 * TODO: Add your target CPU or board to this file.
 *
 * Example:
 * #elif defined(MCU_xxxxx)
 * extern NUTDEVICE devDebug0;
 *
 */
#include <dev/debug.h>

#ifdef NUT_DEV_DEBUG_READ
/* If input is enabled and if it calls NutSleep(), then we need this. */
#include <sys/timer.h>
#endif

/*!
 * \addtogroup xgDevDebugZero
 */
/*@{*/

/*!
 * \brief Handle I/O controls for the debug device.
 *
 * Most debug device drivers support UART_SETSPEED only, to allow to
 * configure the baud rate. However, none is really required and the
 * driver may choose a fixed baud rate when initialized.
 *
 * Nut/OS calls this function via the _NUTDEVICE::dev_ioctl pointer.
 * In order to save code space, drivers may choose to eliminate this
 * function and point to the related function of the NULL device driver.
 *
 * Applications must not call this function, but use the ioctl()
 * function instead.
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
static int ZeroDebugIOCtl(NUTDEVICE * dev, int req, void *conf)
{
    /*
     * TODO: You may implement some UART_xxx functions.
     *
     * However, all Nut/OS samples should work fine when leaving
     * it as it is.
     */
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
static void ZeroDebugPut(char ch)
{
    if (ch == '\n') {
        /* Prepend NL with CR. */
        ZeroDebugPut('\r');
    }
    /*
     * TODO: Write the character to UART transmit register.
     *
     * Consult the datasheet of your hardware. You should
     * check the status register first and then write to
     * the transmit data register. This way the last character
     * is transmitted, while the application continues.
     */
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
 * The debug driver is not interrupt driven and typically has no
 * internal buffer and no timeout control. Thus, it can be safely
 * used in interrupt routines. Of course, this will significantly
 * decrease interrupt latency and overall system performance.
 *
 * \param dev    Identifies the device to send to. This may be used by
 *               the driver to retrieve the \ref NUTDEVICE pointer.
 * \param buffer Pointer to the data to be written.
 * \param len    Number of characters to write. If 0, then the caller
 *               requests to flush the drivers internal output buffer.
 *               In this case the pointer to the data is ignored.
 *
 * \return The number of characters written, which may be less than the
 *         number of characters specified, if a timeout occurred.
 *         A return value of -1 indicates an error.
 */
static int ZeroDebugWrite(NUTFILE * fp, const void *buffer, int len)
{
    int c = len;
    const char *cp = (const char *) buffer;

    while (c--) {
        ZeroDebugPut(*cp++);
    }
    /* Assume, that we always successfully sent all data. */
    return len;
}

#ifdef NUT_DEV_DEBUG_READ
/*!
 * \brief Read characters from debug device.
 *
 * This function is called by the low level input routines of the
 * \ref xrCrtLowio "C runtime library", using the _NUTDEVICE::dev_read
 * entry. The function is optional and available only, if
 * \ref NUT_DEV_DEBUG_READ is configured,
 *
 * The debug driver is not interrupt driven and typically has no
 * internal buffer and no timeout control. It will block the calling
 * thread until at least one character has been received. Characters,
 * that are received before this function is called, may get lost.
 *
 * Typically it suspends the current thread for 1 millisecond, if no
 * input character is available. Therefore, input functions are not
 * allowed in interrupt routines.
 *
 * \param fp     Pointer to a \ref _NUTFILE structure, obtained by a
 *               previous call to ZeroDebugOpen().
 * \param buffer Pointer to the buffer that receives the data. If NULL,
 *               then all characters in the input buffer will be
 *               removed.
 * \param size   Maximum number of bytes to read. If the buffer pointer
 *               is NULL, then this must be zero as well.
 *
 * \return The number of bytes read, which may be less than the number
 *         of bytes specified. A return value of -1 indicates an error,
 *         while zero is returned in case of a timeout.
 */
static int ZeroDebugRead(NUTFILE * fp, void *buffer, int size)
{
    int rc;
    unsigned int ch;
    char *bp = (char *) buffer;
    /* Dummy flag to simulate data availability. */
    int available = 0;

    /* Wait for the first character, forever. */
    for (rc = 0; rc < size; rc++) {
        /*
         * TODO: Check, if a character is available.
         *
         * Consult the datasheet of your target CPU about how to check
         * the status register.
         */
        while (!available) {
            /* Nothing available, take a small nap to let other threads
               continue. Due to this call we can not do any input in
               interrupt context. */
            NutSleep(1);
            if ((rc || bp == NULL) && !available) {
                /* Still no new data. Return, if we already got something
                   or if we had been called to flush the buffer. */
                return rc;
            }
        }

        /*
         * TODO: Get the character from the receiver.
         *
         * Consult the datasheet of your hardware about how to get the
         * last received character from the receiver data register.
         */
        ch = 0;
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
 * Applications must not call this function, they may use kbhit()
 * instead. Although not standard, it is quite popular.
 *
 * This function is called by the low level size routine of the C runtime
 * library, using the _NUTDEVICE::dev_size entry. It is used by Nut/OS to
 * implement kbhit(). The function is optional and available only, if
 * \ref NUT_DEV_DEBUG_READ is configured,
 *
 * \param fp Pointer to a \ref _NUTFILE structure, obtained by a
 *           previous call to ZeroDebugOpen().
 *
 * \return The number of bytes currently stored in input buffer. The
 *         debug driver is not interrupt driven and has no input buffer.
 *         Thus, it will only return 0 or 1., unless the hardware
 *         provides a FIFO.
 *
 */
static long ZeroDebugSize(NUTFILE *fp)
{
    /*
     * TODO: Return 0 if the receiver ready flag is inactive.
     *
     * You should only check the UART status register for the receiver
     * ready flag. Do not retrieve the character from the data register.
     *
     * If available, you may query the number of characters in the
     * hardware FIFO. However, in most cases this is not worth the
     * effort, because almost all applications just want to check,
     * if something is available before entering the blocking
     * read function.
     */
    return 1;
}

#else /* NUT_DEV_DEBUG_READ */

/* The driver is write-only. */
#define ZeroDebugRead   NULL
#define ZeroDebugSize   NULL

#endif /* NUT_DEV_DEBUG_READ */

/*!
 * \brief Open debug device.
 *
 * \return Pointer to a static NUTFILE structure.
 */
static NUTFILE *ZeroDebugOpen(NUTDEVICE * dev, const char *name, int mode, int acc)
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
static int ZeroDebugClose(NUTFILE * fp)
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
static int ZeroDebugInit(NUTDEVICE * dev)
{
    /*
     * TODO: Initialize the UART hardware.
     *
     * Most targets require to enable specific clocks and to configure
     * peripheral pins.
     *
     * It is recommended to setup the UART for
     * - 115200 Baud
     * - 1 start bit
     * - 1 stop bit
     * - no parity
     */
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
 * driven UART driver will be used.
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
 * - most or all UART settings are hard coded
 * - may not work with non-ASCII (binary) data
 * - often only output is supported, not input
 * - only one instance (open) is allowed
 *
 * When used with Harvard architectures, additional functions may
 * be offered to access data in program space.
 */
NUTDEVICE devDebug0 = {
    NULL,               /*!< _NUTDEVICE::dev_next, must be NULL */
    {'u', 'a', 'r', 't', '0', 0, 0, 0, 0}
    ,                   /*!< _NUTDEVICE::dev_name, use for all UART0 drivers. */
    IFTYP_CHAR,         /*!< _NUTDEVICE::dev_type, probably not used, may be 0. */
    0,                  /*!< _NUTDEVICE::dev_base, not used by this driver. */
    0,                  /*!< _NUTDEVICE::dev_irq, not used by this driver. */
    NULL,               /*!< _NUTDEVICE::dev_icb, not used by this driver. */
    &dbgfile,           /*!< _NUTDEVICE::dev_dcb, stores the \ref NUTFILE handle. */
    ZeroDebugInit,      /*!< _NUTDEVICE::dev_init. */
    ZeroDebugIOCtl,     /*!< _NUTDEVICE::dev_ioctl. */
    ZeroDebugRead,      /*!< _NUTDEVICE::dev_read, optional, may be NULL. */
    ZeroDebugWrite,     /*!< _NUTDEVICE::dev_write. */
    ZeroDebugOpen,      /*!< _NUTDEVICE::dev_open. */
    ZeroDebugClose,     /*!< _NUTDEVICE::dev_close. */
    ZeroDebugSize,      /*!< _NUTDEVICE::dev_size, optional, may be NULL. */
    NULL,               /*!< _NUTDEVICE::dev_select, optional, not yet implemented */
};

/*@}*/
