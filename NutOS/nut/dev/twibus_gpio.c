/*
 * Copyright (C) 2005-2007 by egnite Software GmbH. All rights reserved.
 * Copyright (C) 2012 Uwe Bonnes
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
 *
 */

/*!
 * \file dev/twibus_gpio.c
 * \brief Bit banging base on the GPIO API.
 *
 *
 */

#include <cfg/os.h>
#include <cfg/twi.h>
#include <cfg/arch/gpio.h>

#include <sys/heap.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <string.h>

#include <dev/twif.h>
#include <dev/gpio.h>

#if defined(SDA_PORT) && defined(SDA_PIN) \
    && defined(SCL_PORT) && defined(SCL_PIN)
#define SDA_INIT GpioPinConfigSet \
(SDA_PORT, SDA_PIN, GPIO_CFG_OUTPUT | GPIO_CFG_MULTIDRIVE | GPIO_CFG_PULLUP)

#define SCL_INIT GpioPinConfigSet \
(SCL_PORT, SCL_PIN, GPIO_CFG_OUTPUT | GPIO_CFG_MULTIDRIVE | GPIO_CFG_PULLUP)


#define SCL_HIGH GpioPinSetHigh(SCL_PORT, SCL_PIN)
#define SCL_LOW  GpioPinSetLow(SCL_PORT, SCL_PIN)
#define SDA_HIGH GpioPinSetHigh(SDA_PORT, SDA_PIN)
#define SDA_LOW  GpioPinSetLow(SDA_PORT, SDA_PIN)
#define SDA_STAT GpioPinGet(SDA_PORT, SDA_PIN)

#define SCL_RELEASE GpioPinRelease(SCL_PORT, SCL_PIN)
#define SCL_DRIVE   GpioPinDrive(SCL_PORT, SCL_PIN)
#define SDA_RELEASE GpioPinRelease(SDA_PORT, SDA_PIN)
#define SDA_DRIVE   GpioPinDrive(SDA_PORT, SDA_PIN)
#else
#define SDA_INIT
#define SCL_INIT
#define SCL_HIGH
#define SCL_LOW
#define SDA_HIGH
#define SDA_LOW
#define SDA_STAT 0
#define SCL_RELEASE
#define SCL_DRIVE
#define SDA_RELEASE
#define SDA_DRIVE
#define TWI_GPIO_UNDEFINED
#endif


static int_fast8_t twibb_initialized;
static uint_fast16_t delay_unit;
static volatile int_fast8_t tw_mm_error;

/*
 * Short delay.
 *
 * Our bit banging code relies on pull-up resistors. The I/O ports mimic
 * open collector outputs by switching to input mode for high level and
 * switching to output mode for low level. This is much slower than
 * switching an output between low to high. Thus we need some delay.
 */
static void TwDelay(int nops)
{
    NutMicroDelay(nops * delay_unit);
}

/*
 * Falling edge on the data line while the clock line is high indicates
 * a start condition.
 *
 * Entry: SCL any, SDA any
 * Exit: SCL low, SDA low
 */
static void TwStart(void)
{
    SDA_RELEASE; SDA_HIGH;
    TwDelay(1);
    SCL_RELEASE; SCL_HIGH;
    TwDelay(1);
    SDA_LOW; SDA_DRIVE;
    TwDelay(1);
    SCL_LOW; SCL_DRIVE;
    TwDelay(1);
}

/*
 * Rising edge on the data line while the clock line is high indicates
 * a stop condition.
 *
 * Entry: SCL low, SDA any
 * Exit: SCL high, SDA high
 */
static void TwStop(void)
{
    SDA_LOW; SDA_DRIVE;
    TwDelay(1);
    SCL_RELEASE; SCL_HIGH;
    TwDelay(2);
    SDA_RELEASE; SDA_HIGH;
    TwDelay(8);
}

/*
 * Toggles out a single byte in master mode.
 *
 * Entry: SCL low, SDA any
 * Exit: SCL low, SDA high
 */
static int TwPut(uint8_t octet)
{
    int i;

    for (i = 0x80; i; i >>= 1) {
        /* Set the data bit. */
        if (octet & i) {
            SDA_RELEASE; SDA_HIGH;
        } else {
            SDA_LOW; SDA_DRIVE;
        }
        /* Wait for data to stabelize. */
        TwDelay(1);
        /* Toggle the clock. */
        SCL_RELEASE; SCL_HIGH;
        TwDelay(2);
        SCL_LOW; SCL_DRIVE;
        TwDelay(1);
    }

    /* Set data line high to receive the ACK bit. */
    SDA_RELEASE; SDA_HIGH;

    /* ACK should appear shortly after the clock's rising edge. */
    SCL_RELEASE; SCL_HIGH;
    TwDelay(2);
    if (SDA_STAT) {
        i = -1;
    } else {
        i = 0;
    }
    SCL_LOW; SCL_DRIVE;

    return i;
}

/*
 * Toggles in a single byte in master mode.
 *
 * Entry: SCL low, SDA any
 * Exit: SCL low, SDA high
 */
static uint8_t TwGet(void)
{
    uint8_t rc = 0;
    int i;

    /* SDA is input. */
    SDA_RELEASE; SDA_HIGH;
    TwDelay(1);
    for (i = 0x80; i; i >>= 1) {
        TwDelay(1);
        /* Data should appear shortly after the clock's rising edge. */
        SCL_RELEASE; SCL_HIGH;
        TwDelay(2);
        /* SDA read. */
        if (SDA_STAT) {
            rc |= i;
        }
        SCL_LOW; SCL_DRIVE;
    }
    return rc;
}

/*
 * Toggles out an acknowledge bit in master mode.
 *
 * Entry: SCL low, SDA any
 * Exit: SCL low, SDA high
 */
static void TwAck(void)
{
    SDA_LOW; SDA_DRIVE;
    TwDelay(1);
    SCL_RELEASE; SCL_HIGH;
    TwDelay(2);
    SCL_LOW; SCL_DRIVE;
    TwDelay(1);
    SDA_RELEASE; SDA_HIGH;
}

/*!
 * \brief Initialize TWI interface.
 *
 * The specified slave address is used only, if the local system
 * is running as a slave. Anyway, care must be taken that it doesn't
 * conflict with another connected device.
 *
 * \param sla Slave address, must be specified as a 7-bit address,
 *            always lower than 128.
 *
 * \return Always 0.
 *
 */
static int TwGpioInit(void)
{
    SDA_RELEASE; SDA_HIGH;
    SCL_RELEASE; SCL_HIGH;
    SDA_INIT;
    SCL_INIT;
    twibb_initialized = 1;

    return 0;
}

/*!
 * \brief Transmit and/or receive data as a master.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \param sla    Slave address of the destination. This slave address
 *               must be specified as a 7-bit address. For example, the
 *               PCF8574A may be configured to slave addresses from 0x38
 *               to 0x3F.
 * \param txdata Points to the data to transmit. Ignored, if the number
 *               of data bytes to transmit is zero.
 * \param txlen  Number of data bytes to transmit. If zero, then the
 *               interface will not send any data to the slave device
 *               and will directly enter the master receive mode.
 * \param rxdata Points to a buffer, where the received data will be
 *               stored. Ignored, if the maximum number of bytes to
 *               receive is zero.
 * \param rxsiz  Maximum number of bytes to receive. Set to zero, if
 *               no bytes are expected from the slave device.
 * \param tmo    Timeout in milliseconds. To disable timeout, set this
 *               parameter to NUT_WAIT_INFINITE.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 *
 * \note Timeout is not used in the bit banging version.
 */
int NutTwiMasterTranceive(NUTTWIBUS *bus, uint8_t sla, const void *txdata, uint16_t txlen, void *rxdata, uint16_t rxsiz, uint32_t tmo)

{
    int rc = 0;
    uint8_t *cp;

    if (!twibb_initialized) {
        TwGpioInit();
    }

    /* This routine is marked reentrant, so lock the interface. */
    if (NutEventWait(&bus->bus_mutex, tmo)) {
        tw_mm_error = TWERR_IF_LOCKED;
        return -1;
    }

    TwStart();
    /* Send SLA+W and check for ACK. */
    if ((rc = TwPut(sla << 1)) == 0) {
        for (cp = (uint8_t *)txdata; txlen--; cp++) {
            if ((rc = TwPut(*cp)) != 0) {
                break;
            }
        }
    }
    if (rc == 0 && rxsiz) {
        TwStart();
        /* Send SLA+R and check for ACK. */
        if ((rc = TwPut((sla << 1) | 1)) == 0) {
            for (cp = rxdata;; cp++) {
                *cp = TwGet();
                if (++rc >= rxsiz) {
                    break;
                }
                TwAck();
            }
        }
    }
    TwStop();

    if (rc == -1) {
        tw_mm_error = TWERR_SLA_NACK;
    }

    /* Release the interface. */
    NutEventPost(&bus->bus_mutex);

    return rc;
}

/*!
 * \brief Get last master mode error.
 *
 * You may call this function to determine the specific cause
 * of an error after twi transaction failed.
 *
 */
int NutTwiMasterError(NUTTWIBUS *bus)
{
    int rc = tw_mm_error;
    tw_mm_error = 0;
    return rc;
}

/*!
 * \brief Listen for incoming data from a master.
 *
 * If this function returns without error, the bus is blocked. The caller
 * must immediately process the request and return a response by calling
 * TwSlaveRespond().
 *
 * \note Slave mode is not implemented in the bitbanging driver.
 *       Thus the function always returns -1.
 *
 * \param sla    Points to a byte variable, which receives the slave
 *               address sent by the master. This can be used by the
 *               caller to determine whether the the interface has been
 *               addressed by a general call or its individual address.
 * \param rxdata Points to a data buffer where the received data bytes
 *               are stored.
 * \param rxsiz  Specifies the maximum number of data bytes to receive.
 * \param tmo    Timeout in milliseconds. To disable timeout,
 *               set this parameter to NUT_WAIT_INFINITE.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 *
 */
int NutTwiSlaveListen(NUTTWIBUS *bus, uint8_t *sla, void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
    return -1;
}

/*!
 * \brief Send response to a master.
 *
 * This function must be called as soon as possible after TwSlaveListen()
 * returned successfully, even if no data needs to be returned. Not doing
 * so will completely block the bus.
 *
 * \note Slave mode is not implemented in the bitbanging driver.
 *       Thus the function always returns -1.
 *
 * \param txdata Points to the data to transmit. Ignored, if the
 *               number of bytes to transmit is zero.
 * \param txlen  Number of data bytes to transmit.
 * \param tmo    Timeout in milliseconds. To disable timeout,
 *               set this parameter to NUT_WAIT_INFINITE.
 *
 * \return The number of bytes transmitted, -1 in case of an error or timeout.
 */
extern int NutTwiSlaveRespond(NUTTWIBUS *bus, void *txdata, uint16_t txlen, uint32_t tmo)
{
    return -1;
}

/*!
 * \brief Get last slave mode error.
 *
 * You may call this function to determine the specific cause
 * of an error after TwSlaveListen() or TwSlaveRespond() failed.
 *
 * \return Error code or 0 if no error occurred.
 *
 * \note Slave mode is not implemented in the bitbanging driver.
 *       Thus the function always returns TWERR_BUS.
 */
extern int NutTwiSlaveError(NUTTWIBUS *bus)
{
    return TWERR_BUS;
}


/*!
 * \brief Set Speed of I2C Interface.
 *
 * Setup Interface Speed
 */
static int NutTwiSetSpeed( NUTTWIBUS *bus, uint32_t *speed)
{
    int rc = -1;

    if (bus==NULL) {
        /* No bus selected */
        return rc;
    }

    if (*speed > 400000) {
        /* Speed out of range */
        return rc;
    }
    if (*speed < 4)
        delay_unit = 0xffff;
    else
        delay_unit = 250000/(*speed);
    if (delay_unit == 0)
        delay_unit = 1;

    return 0;
}

/*!
 * \brief Request Current Speed of I2C Interface.
 *
 * \return the caculated TWI Speed
 */
static uint32_t NutTwiGetSpeed(void)
{
    return 250000L/(uint32_t)delay_unit;
}

/*!
 * \brief Perform TWI control functions.
 *
 * Not implemented in the bit banging version.
 *
 * \param req  Requested control function.
 * \param conf Points to a buffer that contains any data required for
 *         the given control function or receives data from that
 *         function.
 *
 * \return Always 0.
 */
int NutTwiIOCtl( NUTTWIBUS *bus, int req, void *conf )
{
    int rc = 0;

    switch (req) {

    case TWI_SETSPEED:
        rc = NutTwiSetSpeed(bus, (uint32_t *)conf);
        break;

    case TWI_GETSPEED:
        *((uint32_t *)conf) = NutTwiGetSpeed();
        break;

    case TWI_GETSTATUS:
        rc = 0;
        break;

    case TWI_SETSTATUS:
        rc = 0;
        break;

    default:
        rc = -1;
        break;
    }
    return rc;
}

/*!
 * \brief Initialize TWI interface bus.
 *
 * The specified slave address is not used here as we don't support twi-slave
 * on the bitbanging interface
 *
 * \param sla Slave address, must be specified as a 7-bit address,
 *            always lower than 128.
 */
int NutRegisterTwiBus( NUTTWIBUS *bus, uint8_t sla )
{
    int rc = 0;

#if defined(TWI_GPIO_UNDEFINED)
    return -1;
#endif
    /* Check if bus was already registered */
    if( bus->bus_icb) {
        return 0;
    }

    bus->bus_icb = (void*)1;

    /* Initialize interface hardware */
    if (bus->bus_initbus) {
        rc = bus->bus_initbus();
    }

    /* Initialize mutex semaphores. */
    NutEventPost(&bus->bus_mutex);

    return rc;
}

int NutDestroyTwiBus( NUTTWIBUS *bus)
{
    return 0;
}


/*!
 * \brief TWI/I2C bus structure.
 */
NUTTWIBUS TwGpioBus = {
  /*.bus_base =   */  0,           /* Bus base address. */
  /*.bus_sig_ev = */  NULL,        /* Bus data and event interrupt handler. */
  /*.bus_sig_er = */  NULL,        /* Bus error interrupt handler. */
  /*.bus_mutex =  */  NULL,        /* Bus lock queue. */
  /*.bus_icb   =  */  NULL,        /* Bus Runtime Data Pointer */
  /*.bus_dma_tx = */  0,           /* DMA channel for TX direction. */
  /*.bus_dma_rx = */  0,           /* DMA channel for RX direction. */
  /*.bus_initbus =*/  TwGpioInit,  /* Initialize bus controller. */
  /*.bus_recover =*/  NULL,        /* Recover bus controller */
};
