/*
 * Copyright (C) 2009 by egnite GmbH
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
 * \file arch/avr/dev/spibus_gpio.c
 * \brief General GPIO SPI bus driver rotuines.
 *
 * \verbatim
 * $Id: spibus_gpio.c 6019 2015-01-23 23:14:58Z olereinhardt $
 * \endverbatim
 */

#include <sys/timer.h>
#include <sys/nutdebug.h>

#include <dev/spibus_gpio.h>

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
uint_fast32_t GpioSpiBusSetRate(NUTSPINODE * node, uint_fast32_t rate)
{
    uint32_t rc;
    GSPIREG *gspi;

    /* Sanity check. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_stat != NULL);
    gspi = (GSPIREG *)node->node_stat;

    if(gspi->gspi_dly_rate)
        rc = 500000/ gspi->gspi_dly_rate;
    else
        rc = 500000;

    if (rate != SPI_CURRENT_RATE) {
        gspi->gspi_dly_rate = 500000 / rate;
        if (gspi->gspi_dly_rate) {
            node->node_rate = (500000 /gspi->gspi_dly_rate);
        } else {
            node->node_rate = 500000;
            gspi->gspi_dly_rate = 1;
        }
        node->node_mode |= SPI_MODE_UPDATE;
    }
    return rc;
}

/*!
 * \brief Update SPI settings.
 */
int GpioSpiSetup(NUTSPINODE * node)
{
    /* We support 8 bit only. */
    node->node_bits = 8;
    /* Calculate rate at the cost of a division*/
    GpioSpiBusSetRate(node, node->node_rate);
    /* Update done. */
    node->node_mode &= ~SPI_MODE_UPDATE;

    return 0;
}
