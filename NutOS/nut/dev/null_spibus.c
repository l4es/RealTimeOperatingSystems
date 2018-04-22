/*
 * Copyright (C) 2016 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

#include <dev/spibus.h>

/*!
 * \brief Set clock rate of a specified SPI device.
 *
 * The new clock rate will be used for the next transfer. If the given
 * rate is beyond the capabilities of the bus controller, it will
 * automatically adjusted before the transfer starts.
 *
 * \param node Specifies the SPI bus node.
 * \param rate New clock rate, given in bits per second. If the value is
 *             SPI_CURRENT_RATE, then the current rate is kept.
 *
 * \return Previous rate.
 */
uint_fast32_t NullSpiBusSetRate(NUTSPINODE * node, uint_fast32_t rate)
{
    return 0;
}

/*!
 * \brief Transfer data on the SPI bus.
 *
 * \param node  Specifies the SPI bus node.
 * \param txbuf Pointer to the transmit buffer. If NULL, undetermined
 *              byte values are transmitted.
 * \param rxbuf Pointer to the receive buffer. If NULL, then incoming
 *              data is discarded.
 * \param xlen  Number of bytes to transfer.
 *
 * \return Always 0.
 */
int NullSpiBusTransfer(
    NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen)
{
    return 0;
}

/*!
 * \brief Initialize an SPI bus node.
 *
 * This routine is called for each SPI node, which is registered via
 * NutRegisterSpiDevice().
 *
 * \param node Specifies the SPI bus node.
 *
 * \return 0 on success or -1 if there is no valid chip select.
 */
int NullSpiBusNodeInit(NUTSPINODE * node)
{
    return -1;
}

/*! \brief Select a device on the SPI bus.
 *
 * Locks and activates the bus for the specified node.
 *
 * \param node Specifies the SPI bus node.
 * \param tmo Timeout in milliseconds. To disable timeout, set this
 *            parameter to NUT_WAIT_INFINITE.
 *
 * \return -1 is returned and the bus
 *         is not locked.
 */
int NullSpiBusSelect(NUTSPINODE * node, uint32_t tmo)
{
    return -1;
}

/*! \brief Deselect a device on the SPI bus.
 *
 * Deactivates the chip select and unlocks the bus.
 *
 * \param node Specifies the SPI bus node.
 *
 * \return Always 0.
 */
int NullSpiBusDeselect(NUTSPINODE * node)
{
    return 0;
}

/*!
 * \brief Null SPI bus driver implementation structure.
 */
NUTSPIBUS spiBusNull = {
    NULL,               /*!< Bus mutex semaphore (bus_mutex). */
    NULL,               /*!< Bus ready signal (bus_ready). */
    0,                  /*!< Unused bus base address (bus_base). */
    NULL,               /*!< Bus interrupt handler (bus_sig). */
    NullSpiBusNodeInit, /*!< Initialize the bus (bus_initnode). */
    NullSpiBusSelect,   /*!< Select the specified device (bus_alloc). */
    NullSpiBusDeselect, /*!< Deselect the specified device (bus_release). */
    NullSpiBusTransfer, /*!< Transfer data to and from a specified device (bus_transfer). */
    NutSpiBusWait,      /*!< Wait for bus transfer ready (bus_wait). */
    NutSpiBusSetMode,   /*!< Set SPI mode of a specified device (bus_set_mode). */
    NullSpiBusSetRate,  /*!< Set clock rate of a specified device (bus_set_rate). */
    NutSpiBusSetBits,   /*!< Set number of data bits of a specified device (bus_set_bits). */
    NULL,               /*!< Private data of the hardware specific implementation. */
    NULL,               /*!< Pointer to the bus driver's device control block. */
};
