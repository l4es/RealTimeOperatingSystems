/*
 * Copyright (C) 2012 by egnite GmbH
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
 * \file dev/i2cbus.c
 * \brief I2C bus driver.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <stdint.h>
#include <errno.h>

#include <sys/timer.h>
#include <sys/event.h>

#include <dev/i2cbus.h>

/*!
 * \addtogroup xgI2cBus
 */
/*@{*/

/*!
 * \brief Register and initialize an I2C device attached to a specified bus.
 *
 * Calls the bus controller initialization.
 *
 * Applications should call this function during initialization for each
 * I2C slave device they intend to use.
 *
 * \param slave Pointer to the \ref NUTI2C_SLAVE structure, which is provided
 *              by the I2C slave device driver.
 * \param bus   Pointer to the \ref NUTI2C_BUS structure, which is provided
 *              by the I2C bus driver.
 *
 * \return 0 on success, or -1 if the bus initialization failed.
 */
int NutRegisterI2cSlave(NUTI2C_SLAVE *slave, NUTI2C_BUS *bus)
{
    int rc = 0;

    /* Check if initialization is required. */
    if ((bus->bus_flags & I2C_BF_INITIALIZED) == 0) {
        rc = bus->bus_init(bus);
        if (rc == 0) {
            bus->bus_flags |= I2C_BF_INITIALIZED;
            /* Initialize mutex semaphore. */
            NutEventPost(&bus->bus_mutex);
        }
    }
    if (rc == 0) {
        slave->slave_bus = bus;
    }
    return rc;
}

/*!
 * \brief Set or get the clock rate.
 *
 * When setting a new value, then the caller must make sure that all
 * bus transfers had been completed before calling this function.
 *
 * \param bus  Specifies the I2C bus.
 * \param rate New clock rate, given in bits per second. If the value is
 *             I2C_CURRENT_RATE, then the current rate is kept. If it is
 *             zero, then the default rate will be set.
 *
 * \return Old clock rate or I2C_CURRENT_RATE in case of an error.
 */
long NutI2cBusRate(NUTI2C_BUS *bus, long rate)
{
    long rc = bus->bus_rate;

    if (rate != I2C_CURRENT_RATE) {
        /* Configure a new rate. */
        bus->bus_rate = rate;
        if (bus->bus_configure(bus)) {
            /* If failed, restore the previous value. */
            bus->bus_rate = rc;
            rc = I2C_CURRENT_RATE;
        }
    }
    return rc;
}

/*!
 * \brief Set or get a bus access timeout value.
 *
 * \param bus Specifies the I2C bus.
 * \param tmo New timeout value, given in milliseconds. If the value is
 *            I2C_CURRENT_TIMEOUT, then the current rate is kept. If it
 *            is zero, then time monitoring is disabled and a data
 *            transfer call may never return.
 *
 * \return Old timeout value.
 */
uint32_t NutI2cBusTimeout(NUTI2C_BUS *bus, uint32_t tmo)
{
    uint32_t rc = bus->bus_timeout;

    if (tmo != I2C_CURRENT_TIMEOUT) {
        bus->bus_timeout = tmo;
    }
    return rc;
}

/*!
 * \brief Scan bus for I2C slaves.
 *
 * If not initialized, the bus will be configured with the current
 * settings. To change these, applications may call NutI2cBusRate()
 * and NutI2cBusTimeout() prior to this call.
 *
 * \param bus Specifies the I2C bus.
 * \param first First slave address to probe.
 * \param last  Last slave address to probe.
 *
 * \return First slave address found in the given range. If no slave was
 *         detected, then I2C_SLA_NONE is returned.
 */
int NutI2cBusScan(NUTI2C_BUS *bus, int first, int last)
{
    uint_fast8_t initialized = (bus->bus_flags & I2C_BF_INITIALIZED) != 0;
    int e;

    /* If not initialized, try to configure the bus. */
    if (initialized || bus->bus_configure(bus) == 0) {
        while (first <= last) {
            /* Try to get mutex access to an initialized bus. */
            if (initialized && NutEventWait(&bus->bus_mutex, bus->bus_timeout)) {
                break;
            }
            /* Call low level bus probing. */
            e = bus->bus_probe(bus, first);
            /* Release mutex access to an initialized bus. */
            if (initialized) {
                NutEventPost(&bus->bus_mutex);
            }
            if (e == 0) {
                /* Slave found. */
                return first;
            }
            /* No slave, try next address. */
            first++;
        }
    }
    return I2C_SLA_NONE;
}

/*!
 * \brief Set or get a slave's I2C address.
 *
 * \param slave Specifies the slave device.
 * \param tmo   New slave address. If the value is I2C_SLA_NONE, then
 *              the current address is kept.
 *
 * \return Old slave address.
 */
int NutI2cSlaveAddress(NUTI2C_SLAVE *slave, int sla)
{
    int rc = slave->slave_address;

    if (sla != I2C_SLA_NONE) {
        slave->slave_address = sla;
    }
    return rc;
}

/*!
 * \brief Set or get a slave's timeout value.
 *
 * \param slave Specifies the slave device.
 * \param tmo   New timeout value, given in milliseconds. If the value is
 *              I2C_CURRENT_TIMEOUT, then the current rate is kept. If it
 *              is zero, then time monitoring is disabled and a data
 *              transfer call may never return.
 *
 * \return Old timeout value.
 */
uint32_t NutI2cSlaveTimeout(NUTI2C_SLAVE *slave, uint32_t tmo)
{
    uint32_t rc = slave->slave_timeout;

    if (tmo != I2C_CURRENT_TIMEOUT) {
        slave->slave_timeout = tmo;
    }
    return rc;
}

/*!
 * \brief Communicate with an I2C slave.
 *
 * \param slave Pointer to a slave device structure, which had been
 *              registered by a previous call to NutRegisterI2cSlave().
 * \param wdat  Points to the data to write. Ignored, if the number of
 *              data bytes to write is zero.
 * \param wlen  Number of data bytes to write to the device. If zero,
 *              then the interface will only read data from the device.
 *              However, this is quite unlikely. Most I2C slaves require
 *              to write an internal register address before reading.
 * \param rdat  Points to a buffer, where the received data will be
 *              stored. Ignored, if the maximum number of bytes to
 *              read is zero.
 * \param rsiz  Maximum number of bytes to read from the device. Set
 *              this to zero, if no bytes are expected from the slave.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int NutI2cMasterTransceive(NUTI2C_SLAVE *slave, const void *wdat, int wlen, void *rdat, int rsiz)
{
    int rc;
    NUTI2C_BUS *bus;

    /* Allocate the bus. */
    bus = slave->slave_bus;
    rc = NutEventWait(&bus->bus_mutex, bus->bus_timeout);
    if (rc) {
        errno = EIO;
    } else {
        NUTI2C_MSG msg;

        /* Setup a message structure to be passed to the low level driver. */
        msg.msg_wdat = (uint8_t *) wdat;
        msg.msg_wlen = wlen;
        msg.msg_widx = 0;
        msg.msg_rdat = (uint8_t *) rdat;
        msg.msg_rsiz = rsiz;
        msg.msg_ridx = 0;
        /* Call the low level driver. */
        (*bus->bus_transceive) (slave, &msg);
        /* Release the bus. */
        NutEventPost(&bus->bus_mutex);
        /* Return the number of bytes received. */
        rc = msg.msg_ridx;
    }
    return rc;
}

/*@}*/
