#ifndef DEV_I2CBUS_H
#define DEV_I2CBUS_H

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
 * \file include/dev/i2cbus.h
 * \brief I2C bus declarations.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <stdint.h>
#include <sys/types.h>

/*!
 * \addtogroup xgI2cBus
 */
/*@{*/

#define I2C_CURRENT_RATE    -1L
#define I2C_CURRENT_TIMEOUT ((uint32_t) -1)
#define I2C_SLA_NONE        -1

#define I2C_BF_INITIALIZED  0x01

/*!
 * \brief I2C bus structure type.
 */
typedef struct _NUTI2C_BUS NUTI2C_BUS;

/*!
 * \brief I2C slave structure type.
 */
typedef struct _NUTI2C_SLAVE NUTI2C_SLAVE;

/*!
 * \brief I2C message structure type.
 */
typedef struct _NUTI2C_MSG NUTI2C_MSG;

/*!
 * \brief I2C message structure.
 *
 * This structure is used by platform specific implementations of an I2C
 * bus driver, not by application code.
 *
 * While it is more convenient for an application to pass the data buffers
 * and related information as function parameters when calling
 * NutI2cMasterTransceive(), it is easier to implement the hardware specific
 * driver when all parameters are kept in a structure.
 */
struct _NUTI2C_MSG {
    /*! \brief Data to write to slave. */
    const uint8_t *msg_wdat;
    /*! \brief Number of bytes to write to slave. */
    int msg_wlen;
    /*! \brief Number of bytes written. */
    volatile int msg_widx;
    /*! \brief Data to read from slave. */
    uint8_t *msg_rdat;
    /*! \brief Maximum number of bytes to read from slave. */
    int msg_rsiz;
    /*! \brief Number of bytes read. */
    volatile int msg_ridx;
};

/*!
 * \brief I2C bus structure.
 *
 * Each hardware specific bus driver offers a global variable of this
 * type, which applications must pass to NutRegisterI2cSlave() to
 * attach a specific device to a specific bus.
 */
struct _NUTI2C_BUS {
    /*! \brief Private data of the hardware specific implementation. */
    void *bus_icb;
    /*! \brief Hardware initialization. */
    int (*bus_init)(NUTI2C_BUS *);
    /*! \brief Hardware configuration. */
    int (*bus_configure)(NUTI2C_BUS *);
    /*! \brief Hardware specific probe routine. */
    int (*bus_probe)(NUTI2C_BUS *, int sla);
    /*! \brief Hardware specific transfer routine. */
    int (*bus_transceive)(NUTI2C_SLAVE *, NUTI2C_MSG *);
    /*! \brief Bus access timeout. */
    uint32_t bus_timeout;
    /*! \brief Bus speed. */
    long bus_rate;
    /*! \brief Miscellaneous status flags, see  I2C_BF_XXX. */
    uint_fast8_t bus_flags;
    /*! \brief Bus access queue. */
    HANDLE bus_mutex;
};

/*!
 * \brief I2C slave structure.
 *
 * Each hardware specific driver offers a global variable of this type,
 * which applications must pass to NutRegisterI2cSlave() to attach
 * a specific device to a specific bus.
 */
struct _NUTI2C_SLAVE {
    /*! \brief Pointer to the bus driver. */
    NUTI2C_BUS *slave_bus;
    /*! \brief Device's 7 bit slave address. */
    int slave_address;
    /*! \brief Slave access timeout. */
    uint32_t slave_timeout;
    /*! \brief Transfer message. */
    NUTI2C_MSG *slave_msg;
};

/*@}*/

extern int NutRegisterI2cSlave(NUTI2C_SLAVE *slave, NUTI2C_BUS *bus);
extern long NutI2cBusRate(NUTI2C_BUS *bus, long rate);
extern uint32_t NutI2cBusTimeout(NUTI2C_BUS *bus, uint32_t tmo);
extern int NutI2cBusScan(NUTI2C_BUS *bus, int first, int last);

extern int NutI2cSlaveAddress(NUTI2C_SLAVE *slave, int sla);
extern uint32_t NutI2cSlaveTimeout(NUTI2C_SLAVE *slave, uint32_t tmo);
extern int NutI2cMasterTransceive(NUTI2C_SLAVE *slave, const void *wdat, int wlen, void *rdat, int rsiz);

#endif
