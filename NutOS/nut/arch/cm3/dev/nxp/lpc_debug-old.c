/*
 * Copyright (C) 2011-2012 by egnite GmbH
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

#include <cfg/uart.h>
#include <dev/debug.h>
#include <sys/timer.h>

#include <arch/cm3/nxp/mach/lpc1700.h>
#include <arch/cm3/nxp/lpc_debug.h>

/*!
 * \addtogroup xgDevDebugLpc
 */
/*@{*/

/*!
 * \brief Open debug device.
 *
 * \param dev  Pointer to a previously registered \ref NUTDEVICE structure.
 * \param name Ignored, typically points to an empty string.
 * \param mode Ignored, operation mode.
 * \param acc  Ignored, should be zero.
 *
 * \return Pointer to a static NUTFILE structure.
 */
NUTFILE *LpcDevDebugOpen(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    NUTFILE *fp = (NUTFILE *) (dev->dev_dcb);

    fp->nf_next = NULL;
    fp->nf_dev = dev;
    fp->nf_fcb = NULL;

    return fp;
}

/*!
 * \brief Close debug device.
 *
 * \param fp Pointer to a \ref _NUTFILE structure, obtained by a previous
 *           call to LpcDevDebugOpen().
 *
 * \return Always 0.
 */
int LpcDevDebugClose(NUTFILE * fp)
{
    return 0;
}

/*!
 * \brief Handle I/O controls for debug device 2.
 *
 * The debug device supports UART_SETSPEED only.
 *
 * \param dev  Identifies the device that receives the device-control
 *             function.
 * \param req  Requested control function. May be set to one of the
 *             following constants:
 *             - UART_SETSPEED, conf points to an uint32_t value containing the baudrate.
 * \param conf Points to a variable that contains any data required for
 *             the given control function or receives data from that
 *             function.
 *
 * \return 0 on success, -1 otherwise.
 */
int LpcDevDebugIOCtl(NUTDEVICE * dev, int req, void *conf)
{
    return -1;
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
    while ((mem_rd32(dev->dev_base + UART_LSR_OFF) & UART_THRE) == 0);
    mem_wr32_mb(dev->dev_base + UART_THR_OFF, ch);
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
int LpcDevDebugWrite(NUTFILE * fp, const void *buffer, int len)
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
int LpcDevDebugRead(NUTFILE * fp, void *buffer, int size)
{
    int rc;
    unsigned int ch;
    char *bp = (char *) buffer;

    /* Wait for the first character, forever. */
    for (rc = 0; rc < size; rc++) {
        while ((mem_rd32(fp->nf_dev->dev_base + UART_LSR_OFF) & UART_RDR) == 0) {
            NutSleep(1);
            if ((rc || bp == NULL) &&
                (mem_rd32(fp->nf_dev->dev_base + UART_LSR_OFF) & UART_RDR) == 0) {
                return rc;
            }
        }
        ch = mem_rd32_mb(fp->nf_dev->dev_base + UART_RBR_OFF);
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
long LpcDevDebugSize(NUTFILE *fp)
{
    if (mem_rd32(fp->nf_dev->dev_base + UART_LSR_OFF) & UART_RDR) {
        return 1;
    }
    return 0;
}

#endif /* NUT_DEV_DEBUG_READ */

/*@}*/
