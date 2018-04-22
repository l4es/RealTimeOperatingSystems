/*
 * Copyright (C) 2001-2005 by egnite Software GmbH
 * Copyright (C) 2012, Ole Reinhardt <ole.reinhardt@embedded-it.de>
 * Copyright (C) 2013 by egnite GmbH
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
 *
 */

/*!
 * \file arch/avr/dev/i2cbus_avr.c
 * \brief AVR I2C bus driver.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/twi.h>
#include <dev/irqreg.h>

#include <sys/nutdebug.h>
#include <sys/timer.h>
#include <sys/event.h>

#include <dev/i2cbus_avr.h>

#ifndef I2C_DEFAULT_SPEED
#define I2C_DEFAULT_SPEED   100L
#endif

/*!
 * \brief Local data of the AVR TWI bus driver.
 *
 * This structure is passed to the interrupt handler.
 */
typedef struct _AVR_TWICB {
    /*! \brief Slave address. */
    uint8_t icb_slave;
    /*! \brief System interrupt handler. */
    IRQ_HANDLER *icb_sig;
    /*! \brief I2C message. */
    NUTI2C_MSG *icb_msg;
    /*! \brief Thread waiting for completion. */
    HANDLE icb_queue;
} AVR_TWICB;

/*!
 * \brief TWI interrupt handler.
 *
 * The application thread will enable interrupts and initiate sending
 * a start condition. As soon as the hardware sent out this start
 * condition, the first interrupt of a transfer is triggered.
 *
 * A pointer to the interface control block structure is passed
 * to this interrupt function, which, among other things, contains
 * the slave address and number of bytes to transmit and to receive.
 * This enables the interrupt function to process the complete
 * transfer in the background. When done or in case of an error, the
 * waiting thread will be woken up.
 */
static void TwInterrupt(void *arg)
{
    AVR_TWICB *icb = (AVR_TWICB *) arg;
    NUTI2C_MSG *msg = icb->icb_msg;
    uint8_t twsr;
    uint8_t twcr;

    /* Read control register and clear start and stop conditions.
     * We also clear the ACK enable bit, which may be set further
     * down on specific states. Note, that the bus is frozen until
     * the TWINT bit is cleared. */
    twcr = inb(TWCR) & ~(_BV(TWSTA) | _BV(TWSTO) | _BV(TWEA));

    /* Read the status register, remove the prescaler bits and
     * handle the current state in a large switch statement. */
    twsr = inb(TWSR) & 0xF8;

    switch (twsr) {

    /*
     * 0x08: Start condition has been transmitted.
     * 0x10: Repeated start condition has been transmitted.
     */
    case TW_START:
    case TW_REP_START:
        /* If outgoing data is available, transmit SLA+W. Logic is in
         * master transmit mode. */
        if (msg->msg_widx < msg->msg_wlen) {
            outb(TWDR, icb->icb_slave);
        }
        /* If no outgoing data is available, transmit SLA+R. Logic will
         * switch to master receive mode. */
        else {
            outb(TWDR, icb->icb_slave | 1);
        }
        break;

    /*
     * 0x18: SLA+W has been transmitted and ACK has been received.
     * 0x28: Data byte has been transmitted and ACK has been received.
     */
    case TW_MT_SLA_ACK:
    case TW_MT_DATA_ACK:
        /* If outgoing data is left to send, put the next byte in the
         * data register. */
        if (msg->msg_widx < msg->msg_wlen) {
            outb(TWDR, msg->msg_wdat[msg->msg_widx++]);
            break;
        }
        /* All outgoing data has been sent. If a response is expected,
         * transmit a repeated start condition. */
        else if (msg->msg_rsiz) {
            twcr |= _BV(TWSTA);
            break;
        }
        /* If no more data, then fall through to send a stop condition
         * and wake up the waiting thread. */

    /*
     * 0x20: SLA+W has been transmitted, but not acknowledged.
     * 0x30: Data byte has been transmitted, but not acknowledged.
     * 0x38: Arbitration lost while in master mode.
     * 0x48: SLA+R has been transmitted, but not acknowledged.
     */
    case TW_MT_SLA_NACK:
    case TW_MT_DATA_NACK:
    case TW_MT_ARB_LOST:
    case TW_MR_SLA_NACK:
        /* Send stop condition and wake up the waiting thread. */
        twcr |= _BV(TWSTO);
        NutEventPostFromIrq(&icb->icb_queue);
        break;

    /*
     * 0x50: Data byte has been received and acknowledged.
     */
    case TW_MR_DATA_ACK:
        /* Store the data byte in the receive buffer. */
        msg->msg_rdat[msg->msg_ridx++] = inb(TWDR);
        /* Fall through to acknowledge this byte. */

    /*
     * 0x40: SLA+R has been transmitted and ACK has been received.
     */
    case TW_MR_SLA_ACK:
        /* Acknowledge next data bytes except the last one. */
        if (msg->msg_ridx + 1 < msg->msg_rsiz) {
            twcr |= _BV(TWEA);
        }
        break;

    /*
     * 0x58: Data byte has been received, but not acknowledged.
     */
    case TW_MR_DATA_NACK:
        /* Store the last data byte. */
        msg->msg_rdat[msg->msg_ridx++] = inb(TWDR);
        /* Send a stop condition. */
        twcr |= _BV(TWSTO);
        break;

    /*
     * Send stop and return immediately on all unexpected states.
     * However, when all bits are set, then we must not touch the
     * control register.
     */
    default:
        if (twsr != 0xf8) {
            outb(TWCR, twcr | _BV(TWSTO));
        }
        return;
    }
    /* Finally update the control register. If sending a stop condition,
     * then wake up the application. */
    outb(TWCR, twcr);
    if (twcr & _BV(TWSTO)) {
        NutEventPostFromIrq(&icb->icb_queue);
    }
}

/*!
 * \brief Configure the I2C bus controller.
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_conf function pointer. Most implementations will
 * also call this function during initialization to set the
 * default configuration.
 *
 * Right now only the bus clock rate is configurable.
 */
static int AvrTwiBusConf(NUTI2C_BUS *bus)
{
    uint32_t twbr;
    uint8_t twps;
    uint32_t pck = NutClockGet(NUT_HWCLK_PERIPHERAL);

    /* Check parameter. */
    NUTASSERT(bus != NULL);

    /* Calculate the bit rate divider:
       TWBR = (CLOCK / (2 * RATE) - 8 */
    twbr = pck;
    twbr += bus->bus_rate;
    twbr >>= 1;
    twbr /= bus->bus_rate;
    twbr -= 8;
    /* Increase the pre-scaler until the divider fits. */
    for (twps = 0; twbr > 255 && twps < 4; twps++) {
        twbr >>= 2;
    }
    /* Select minimum rate on divider overflow. */
    if (twps > 3) {
        twps = 3;
        twbr = 255;
    }

    /* Set hardware divider registers and enable the bus. */
    outb(TWBR, (uint8_t) twbr);
    outb(TWSR, twps);
    outb(TWCR, _BV(TWEN));

    /* Calculate the actual rate:
       RATE = CLOCK / (2 * TWBR * 4^TWPS + 16) */
    twbr <<= (twps << 1);
    twbr += 8;
    bus->bus_rate = (pck + twbr) / (twbr << 1);

    return 0;
}

/*!
 * \brief Wait for specific bit values in the control register.
 *
 * Using a separate function saves us about 50 bytes of code.
 */
static void AvrTwiBusWait(uint8_t msk, uint8_t val, uint8_t *wt)
{
    while ((inb(TWCR) & msk) != val && wt) {
        NutSleep(1);
        wt--;
    }
}

/*!
 * \brief Probe the I2C bus for a specified slave address.
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_probe function pointer. This may happen even if no
 * slave device had been attached to the bus and thus without any
 * previous call to NUTI2C_BUS::bus_init. However, in that case
 * NUTI2C_BUS::bus_configure will have been called.
 *
 * The bus scan is done in polling mode, interrupts are disabled.
 *
 * In order to reduce the code size, the bus timeout is limited to 255ms.
 */
static int AvrTwiBusProbe(NUTI2C_BUS *bus, int sla)
{
    int rc = -1;
    uint8_t wt = 255;

    /* Check parameter. */
    NUTASSERT(bus != NULL);
    NUTASSERT(bus->bus_icb != NULL);
    if (bus->bus_timeout < 256) {
        wt = bus->bus_timeout;
    }

    /* Send start condition and wait for TWINT. */
    outb(TWCR, _BV(TWINT) | _BV(TWSTA) | _BV(TWEN));
    AvrTwiBusWait(_BV(TWINT), _BV(TWINT), &wt);

    if (wt) {
        /* Send SLA+R and wait for TWINT. */
        outb(TWDR, (sla << 1) | 1);
        outb(TWCR, _BV(TWINT) | _BV(TWEN));
        AvrTwiBusWait(_BV(TWINT), _BV(TWINT), &wt);
        /* Check for acknowledge. */
        if (wt && (inb(TWSR) & 0xF8) == TW_MR_SLA_ACK) {
            /* If address has been acknowledged, then we found a slave.
             * In this case we must do a dummy transfer. */
            rc = 0;
            outb(TWCR, _BV(TWINT) | _BV(TWEN));
            AvrTwiBusWait(_BV(TWINT), _BV(TWINT), &wt);
        }
    }
    /* Send stop condition and wait for auto reset. */
    outb(TWCR, _BV(TWINT) | _BV(TWSTO) | _BV(TWEN));
    AvrTwiBusWait(_BV(TWSTO), 0, &wt);

    return wt ? rc : -1;
}

/*!
 * \brief I2C bus transfer.
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_tran function pointer.
 */
static int AvrTwiBusTran(NUTI2C_SLAVE *slave, NUTI2C_MSG *msg)
{
    int rc;
    AVR_TWICB *icb;

    /* Check parameters. */
    NUTASSERT(slave != NULL);
    NUTASSERT(slave->slave_bus != NULL);
    NUTASSERT(slave->slave_bus->bus_icb != NULL);

    /* Setup the interface control block. This structure is accessed
     * by the interrupt handler. */
    icb = (AVR_TWICB *) slave->slave_bus->bus_icb;
    icb->icb_slave = (uint8_t) slave->slave_address << 1;
    icb->icb_msg = msg;

    /* Send start condition and enable TWI interrupts. */
    outb(TWCR, _BV(TWINT) | _BV(TWSTA) | _BV(TWEN));
    NutIrqEnable(icb->icb_sig);
    /* Wait for transfer complete and disable interrupts. */
    rc = NutEventWait(&icb->icb_queue, slave->slave_timeout);
    NutIrqDisable(icb->icb_sig);

    /* On timeouts we assume, that the bus is no longer active. If an
     * interrupt is still pending, then send a stop condition. */
    if (rc) {
        if (inb(TWCR) & _BV(TWINT)) {
            outb(TWCR, _BV(TWINT) | _BV(TWSTO) | _BV(TWEN));
        }
    }
    /* Otherwise return the number of bytes received. */
    else {
        rc = msg->msg_ridx;
    }
    return rc;
}

/*!
 * \brief Initialize TWI hardware.
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_init function pointer when the application calls
 * NutRegisterI2cSlave().
 */
static int AvrTwiBusInit(NUTI2C_BUS *bus)
{
    AVR_TWICB *icb;

    /* Check parameter. */
    NUTASSERT(bus != NULL);
    NUTASSERT(bus->bus_icb != NULL);
    icb = (AVR_TWICB *) bus->bus_icb;
    /* Enable the bus with default settings. */
    AvrTwiBusConf(bus);
    /* Register IRQ Handler */
    return NutRegisterIrqHandler(icb->icb_sig, TwInterrupt, icb);
}

static AVR_TWICB twi0cb = {
    0,      /*!< \brief AVR_TWICB::icb_base */
    &sig_2WIRE_SERIAL,  /*!< \brief AVR_TWICB::icb_sig */
    NULL,   /*!< \brief AVR_TWICB::icb_msg */
    NULL    /*!< \brief AVR_TWICB::icb_queue */
};

/*!
 * \brief I2C bus driver for AVR TWI hardware.
 *
 * This is an interrupt driven driver, which supports master mode only.
 */
NUTI2C_BUS i2cBus0Avr = {
    &twi0cb,        /*!< \brief NUTI2C_BUS::bus_icb */
    AvrTwiBusInit,  /*!< \brief NUTI2C_BUS::bus_init */
    AvrTwiBusConf,  /*!< \brief NUTI2C_BUS::bus_configure */
    AvrTwiBusProbe, /*!< \brief NUTI2C_BUS::bus_probe */
    AvrTwiBusTran,  /*!< \brief NUTI2C_BUS::bus_transceive */
    100,            /*!< \brief NUTI2C_BUS::bus_timeout */
    I2C_DEFAULT_SPEED * 1000L,  /*!< \brief NUTI2C_BUS::bus_rate */
    0,              /*!< \brief NUTI2C_BUS::bus_flags */
    NULL            /*!< \brief NUTI2C_BUS::bus_mutex */
};
