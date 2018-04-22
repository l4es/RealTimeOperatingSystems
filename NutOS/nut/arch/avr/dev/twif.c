/*
 * Copyright (C) 2001-2005 by egnite Software GmbH. All rights reserved.
 * Copyright (C) 2012, Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 * \file arch/avr/dev/twif.c
 * \brief AVR TWI support.
 *
 * \verbatim
 * $Id: twif.c 4937 2013-01-22 11:38:42Z haraldkipp $
 * \endverbatim
 */

#include <string.h>

#include <dev/irqreg.h>

#include <sys/event.h>
#include <sys/atom.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/heap.h>

#include <dev/twif.h>

#ifdef __AVR_ENHANCED__

/*
TWINT TWEA TWSTA TWSTO TWWC TWEN  0  TWIE
  C                      R        R
*/

#define TWGO    (_BV(TWINT) | _BV(TWEN) | _BV(TWIE))

/*
 * TWI interrupt handler.
 */
static void TwInterrupt(void *arg)
{
    NUTTWIBUS *bus = arg;
    NUTTWIICB *icb = bus->bus_icb;
    uint8_t twsr;
    register uint8_t twcr = inb(TWCR);

    /*
     * Read the status and interpret its contents.
     */
    twsr = inb(TWSR) & 0xF8;
    switch (twsr) {

    /*
     * 0x08: Start condition has been transmitted.
     * 0x10: Repeated start condition has been transmitted.
     */
    case TW_START:
    case TW_REP_START:
        /* We are entering the master mode. Mark the interface busy. */
        icb->tw_if_busy = 1;
        icb->tw_mm_txidx = 0;
        icb->tw_mm_rxidx = 0;

        /*
         * If outgoing data is available, transmit SLA+W. Logic is in
         * master transmit mode.
         */
        if (icb->tw_mm_txlen) {
            outb(TWDR, icb->tw_mm_sla);
        }

        /*
         * If outgoing data not available, transmit SLA+R. Logic will
         * switch to master receiver mode.
         */
        else {
            outb(TWDR, icb->tw_mm_sla | 1);
        }
        outb(TWCR, TWGO | (twcr & _BV(TWEA)));
        break;

    /*
     * 0x18: SLA+W has been transmitted and ACK has been received.
     * 0x28: Data byte has been transmitted and ACK has been received.
     */
    case TW_MT_SLA_ACK:
    case TW_MT_DATA_ACK:
        /*
         * If outgoing data left to send, put the next byte in the data
         * register.
         */
        if (icb->tw_mm_txidx < icb->tw_mm_txlen) {
            outb(TWDR, icb->tw_mm_txbuf[icb->tw_mm_txidx]);
            /* Late increment fixes ICCAVR bug. Thanks to Andreas Siebert and Michael Fischer. */
            icb->tw_mm_txidx++;
            outb(TWCR, TWGO | (twcr & _BV(TWEA)));
            break;
        }

        /*
         * All outgoing data has been sent. If a response is expected,
         * transmit a repeated start condition.
         */
        icb->tw_mm_txlen = 0;
        if (icb->tw_mm_rxlen) {
            outb(TWCR, TWGO | (twcr & _BV(TWEA)) | _BV(TWSTA));
            break;
        }

    /*
     * 0x20: SLA+W has been transmitted, but not acknowledged.
     * 0x30: Data byte has been transmitted, but not acknowledged.
     * 0x48: SLA+R has been transmitted, but not acknowledged.
     */
    case TW_MT_SLA_NACK:
    case TW_MT_DATA_NACK:
    case TW_MR_SLA_NACK:
        /* Set unique error code. */
        if (twsr == TW_MT_SLA_NACK || twsr == TW_MR_SLA_NACK) {
            icb->tw_mm_err = TWERR_SLA_NACK;
            icb->tw_mm_txlen = 0;
            icb->tw_mm_rxlen = 0;
        }

        /* Wake up the application. */
        NutEventPostFromIrq(&icb->tw_mm_mtx);

        /*
         * Send a stop condition. If we have a listener, generate
         * an acknowlegde on an incoming address byte.
         */
        if(icb->tw_sm_rxlen) {
            outb(TWCR, TWGO | _BV(TWEA) | _BV(TWSTO));
        }
        else {
            outb(TWCR, TWGO | _BV(TWSTO));
        }

        /* The interface is idle. */
        icb->tw_if_busy = 0;
        break;

    /*
     * 0x38: Arbitration lost while in master mode.
     */
    case TW_MT_ARB_LOST:
        /*
         * The start condition will be automatically resend after
         * the bus becomes available.
         */
        sbi(TWCR, TWSTA);
        /* The interface is idle. */
        icb->tw_if_busy = 0;
        break;

    /*
     * 0x50: Data byte has been received and acknowledged.
     */
    case TW_MR_DATA_ACK:
        /*
         * Store the data byte in the master receive buffer.
         */
        icb->tw_mm_rxbuf[icb->tw_mm_rxidx] = inb(TWDR);
        /* Late increment fixes ICCAVR bug. Thanks to Andreas Siebert and Michael Fischer. */
        icb->tw_mm_rxidx++;

    /*
     * 0x40: SLA+R has been transmitted and ACK has been received.
     */
    case TW_MR_SLA_ACK:
        /*
         * Acknowledge next data bytes except the last one.
         */
        if (icb->tw_mm_rxidx + 1 < icb->tw_mm_rxlen) {
            outb(TWCR, TWGO | _BV(TWEA));
        }
        else {
            outb(TWCR, TWGO);
        }
        break;

    /*
     * 0x58: Data byte has been received, but not acknowledged.
     */
    case TW_MR_DATA_NACK:
        /*
         * Store the last data byte.
         */
        icb->tw_mm_rxbuf[icb->tw_mm_rxidx] = inb(TWDR);
        /* Late increment fixes ICCAVR bug. Thanks to Andreas Siebert and Michael Fischer. */
        icb->tw_mm_rxidx++;
        icb->tw_mm_rxlen = 0;

        /* Wake up the application. */
        NutEventPostFromIrq(&icb->tw_mm_mtx);

        /*
         * Send a stop condition. If we have a listener, generate
         * an acknowlegde on an incoming address byte.
         */
        if(icb->tw_sm_rxlen) {
            outb(TWCR, TWGO | _BV(TWEA) | _BV(TWSTO));
        }
        else {
            outb(TWCR, TWGO | _BV(TWSTO));
        }

        /* The interface is idle. */
        icb->tw_if_busy = 0;
        break;

    /*
     * 0x60: Own SLA+W has been received and acknowledged.
     * 0x68: Arbitration lost as master. Own SLA+W has been received
     *       and acknowledged.
     * 0x70: General call address has been received and acknowledged.
     * 0x78: Arbitration lost as master. General call address has been
     *       received and acknowledged.
     */
    case TW_SR_SLA_ACK:
    case TW_SR_ARB_LOST_SLA_ACK:
    case TW_SR_GCALL_ACK:
    case TW_SR_ARB_LOST_GCALL_ACK:
        /*
         * Do only acknowledge incoming data bytes, if we got receive
         * buffer space. Fetch the slave address from the data register
         * and reset the receive index.
         */
        if (icb->tw_sm_rxlen) {
            /* We are entering the slave receive mode. Mark the interface busy. */
            icb->tw_if_busy = 1;

            icb->tw_sm_sla = inb(TWDR);
            outb(TWCR, TWGO | _BV(TWEA));
            icb->tw_sm_rxidx = 0;
        }

        /*
         * Do not acknowledge incoming data.
         */
        else {
            outb(TWCR, TWGO);
        }
        break;

    /*
     * 0x80: Data byte for own SLA has been received and acknowledged.
     * 0x90: Data byte for general call address has been received and
     *       acknowledged.
     */
    case TW_SR_DATA_ACK:
    case TW_SR_GCALL_DATA_ACK:
        /*
         * If the receive buffer isn't filled up, store data byte.
         */
        if (icb->tw_sm_rxidx < icb->tw_sm_rxlen) {
            icb->tw_sm_rxbuf[icb->tw_sm_rxidx] = inb(TWDR);
            /* Late increment fixes ICCAVR bug. Thanks to Andreas Siebert and Michael Fischer. */
            icb->tw_sm_rxidx++;
        }
        else {
            icb->tw_sm_rxlen = 0;
        }

        /*
         * If more space is available for incoming data, then continue
         * receiving. Otherwise do not acknowledge new data bytes.
         */
        if (icb->tw_sm_rxlen) {
            outb(TWCR, TWGO | _BV(TWEA));
            break;
        }

    /*
     * 0x88: Data byte received, but not acknowledged.
     * 0x98: Data byte for general call address received, but not
     *       acknowledged.
     */
    case TW_SR_DATA_NACK:
    case TW_SR_GCALL_DATA_NACK:
        /*
         * Continue not accepting more data.
         */
        if (icb->tw_mm_txlen || icb->tw_mm_rxlen) {
            outb(TWCR, inb(TWCR) | _BV(TWEA) | _BV(TWSTA));
        }
        else {
            outb(TWCR, inb(TWCR) | _BV(TWEA));
        }
        break;

    /*
     * 0xA0: Stop condition or repeated start condition received.
     */
    case TW_SR_STOP:
        /*
         * Wake up the application. If successful, do nothing. This
         * will keep SCL low and thus block the bus. The application
         * must now setup the transmit buffer and re-enable the
         * interface.
         */
        if (icb->tw_sm_rxmtx == 0 || icb->tw_sm_err) {
            /*
             * If no one has been waiting on the queue, the application
             * probably gave up waiting. So we continue on our own, either
             * in idle mode or switching to master mode if a master
             * request is waiting.
             */
            if (icb->tw_mm_txlen || icb->tw_mm_rxlen) {
                outb(TWCR, TWGO | _BV(TWSTA));
            }
            else {
                outb(TWCR, TWGO);
            }
            icb->tw_if_busy = 0;
        }
        else {
            NutEventPostFromIrq(&icb->tw_sm_rxmtx);
            icb->tw_sm_rxlen = 0;
            outb(TWCR, twcr & ~(_BV(TWINT) | _BV(TWIE)));
        }
        break;

    /*
     * 0xA8: Own SLA+R has been received and acknowledged.
     * 0xB0: Arbitration lost in master mode. Own SLA has been received
     *       and acknowledged.
     */
    case TW_ST_SLA_ACK:
    case TW_ST_ARB_LOST_SLA_ACK:
        /* Not idle. */
        icb->tw_if_busy = 1;
        /* Reset transmit index and fall through for outgoing data. */
        icb->tw_sm_txidx = 0;

    /*
     * 0xB8: Data bytes has been transmitted and acknowledged.
     */
    case TW_ST_DATA_ACK:
        /*
         * If outgoing data left to send, put the next byte in the
         * data register. Otherwise transmit a dummy byte.
         */
        if (icb->tw_sm_txidx < icb->tw_sm_txlen) {
            outb(TWDR, icb->tw_sm_txbuf[icb->tw_sm_txidx]);
            /* Do not set acknowledge on the last data byte. */
            /* Early increment fixes ICCAVR bug. Thanks to Andreas Siebert and Michael Fischer. */
            ++icb->tw_sm_txidx;
            if (icb->tw_sm_txidx < icb->tw_sm_txlen) {
                outb(TWCR, TWGO | _BV(TWEA));
            }
            else {
                icb->tw_sm_txlen = 0;
                outb(TWCR, TWGO);
            }
            break;
        }

        /* No more data. Continue sending dummies. */
        outb(TWDR, 0);
        outb(TWCR, TWGO);
        break;

    /*
     * 0xC0: Data byte has been transmitted, but not acknowledged.
     * 0xC8: Last data byte has been transmitted and acknowledged.
     */
    case TW_ST_DATA_NACK:
    case TW_ST_LAST_DATA:
        NutEventPostFromIrq(&icb->tw_sm_txmtx);

        /* Transmit start condition, if a master transfer is waiting. */
        if (icb->tw_mm_txlen || icb->tw_mm_rxlen) {
            outb(TWCR, TWGO | _BV(TWSTA) | /**/ _BV(TWEA));
        }
        /* Otherwise enter idle state. */
        else {
            outb(TWCR, TWGO | _BV(TWEA));
        }
        icb->tw_if_busy = 0;
        break;

    /*
     * 0x00: Bus error.
     */
    case TW_BUS_ERROR:
        outb(TWCR, inb(TWCR) | _BV(TWSTO));
#if 1
        icb->tw_if_busy = 0;
        icb->tw_mm_err = TWERR_BUS;
        icb->tw_sm_err = TWERR_BUS;
        NutEventPostFromIrq(&icb->tw_sm_rxmtx);
        NutEventPostFromIrq(&icb->tw_sm_txmtx);
        NutEventPostFromIrq(&icb->tw_mm_mtx);
#endif
        break;
    }
}

#endif /* __AVR_ENHANCED__ */

/*!
 * \brief Transmit and/or receive data as a master.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \note This function is only available on ATmega128 systems.
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
 */
int NutTwiMasterTranceive(NUTTWIBUS *bus, uint8_t sla, const void *txdata, uint16_t txlen, void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    NUTTWIICB *icb = bus->bus_icb;
    int rc = -1;
    /* This routine is marked reentrant, so lock the interface. */
    if(NutEventWait(&bus->bus_mutex, tmo)) {
        icb->tw_mm_err = TWERR_IF_LOCKED;
        NutEventPost(&bus->bus_mutex);
        return -1;
    }
    if (icb->tw_if_busy)
        NutSleep(1);
    while(icb->tw_if_busy) {
        if (tmo == 1)
            return -1;
        NutSleep(1);
        tmo --;
    }
    NutEnterCritical();
    /*
     * Set all parameters for master mode.
     */
    icb->tw_mm_sla = sla << 1;
    icb->tw_mm_err = 0;
    icb->tw_mm_txlen = txlen;
    icb->tw_mm_txbuf = txdata;
    icb->tw_mm_rxlen = rxsiz;
    icb->tw_mm_rxbuf = rxdata;

    /*
     * Send a start condition if the interface is idle. If busy, then
     * the interrupt routine will automatically initiate the transfer
     * as soon as the interface becomes ready again.
     */
    if(icb->tw_if_busy == 0) {
        uint8_t twcr = inb(TWCR);
        uint8_t twsr = inb(TWSR);
        if((twsr & 0xF8) == TW_NO_INFO) {
            if(icb->tw_sm_rxlen) {
                outb(TWCR, _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWSTA) | (twcr & _BV(TWSTO)));
            }
            else {
                outb(TWCR, _BV(TWEN) | _BV(TWIE) | _BV(TWSTA) | (twcr & _BV(TWSTO)));
            }
        }
    }

    /* Clear the queue. */
    //*broken?! NutEventBroadcastAsync(&tw_mm_mtx);
    if (icb->tw_mm_mtx == SIGNALED) {
        icb->tw_mm_mtx = 0;
    }
    NutExitCritical();

    /*
     * Wait for master transmission done.
     */
    rc = -1;
    if (NutEventWait(&icb->tw_mm_mtx, tmo)) {
        icb->tw_mm_error = TWERR_TIMEOUT;
    } else {
        NutEnterCritical();
        if (icb->tw_mm_err) {
            icb->tw_mm_error = icb->tw_mm_err;
        } else {
            rc = icb->tw_mm_rxidx;
        }
        NutExitCritical();
    }

    /*
     * Release the interface.
     */
    NutEventPost(&bus->bus_mutex);
    return rc;
#endif /* __AVR_ENHANCED__ */
}

/*!
 * \brief Get last master mode error.
 *
 * You may call this function to determine the specific cause
 * of an error after TwMasterTransact() failed.
 *
 * \note This function is only available on ATmega128 systems.
 *
 */
int NutTwiMasterError(NUTTWIBUS *bus)
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    int rc = bus->bus_icb->tw_mm_error;
    bus->bus_icb->tw_mm_error = 0;
    return rc;
#endif
}

/*!
 * \brief Listen for incoming data from a master.
 *
 * If this function returns without error, the bus is blocked. The caller
 * must immediately process the request and return a response by calling
 * TwSlaveRespond().
 *
 * \note This function is only available on ATmega128 systems. The
 *       function is not reentrant.
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
 */
int NutTwiSlaveListen(NUTTWIBUS *bus, uint8_t *sla, void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    int rc = -1;
    NUTTWIICB *icb = bus->bus_icb;

    NutEnterCritical();

    /* Initialize parameters for slave receive. */
    icb->tw_sm_err = 0;
    icb->tw_sm_rxlen = rxsiz;
    icb->tw_sm_rxbuf = rxdata;

    /*
     * If the interface is currently not busy then enable it for
     * address recognition.
     */
    if(icb->tw_if_busy == 0) {
        uint8_t twsr = inb(TWSR);
        if((twsr & 0xF8) == TW_NO_INFO) {
            if(icb->tw_mm_txlen || icb->tw_mm_rxlen)
                outb(TWCR, _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA));
            else
                outb(TWCR, _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
        }
    }

    /* Clear the queue. */
    //*broken?! NutEventBroadcastAsync(&tw_sm_rxmtx);
    if (icb->tw_sm_rxmtx == SIGNALED) {
        icb->tw_sm_rxmtx = 0;
    }

    NutExitCritical();

    /* Wait for a frame on the slave mode queue. */
    if (NutEventWait(&icb->tw_sm_rxmtx, tmo)) {
        NutEnterCritical();
        icb->tw_sm_err = TWERR_TIMEOUT;
        icb->tw_sm_rxlen = 0;
        NutExitCritical();
    }

    /*
     * Return the number of bytes received and the destination slave
     * address, if no slave error occured. In this case the bus is
     * blocked.
     */
    if(icb->tw_sm_err == 0) {
        rc = icb->tw_sm_rxidx;
        *sla = icb->tw_sm_sla;
    }
    return rc;
#endif /* __AVR_ENHANCED__ */
}

/*!
 * \brief Send response to a master.
 *
 * This function must be called as soon as possible after TwSlaveListen()
 * returned successfully, even if no data needs to be returned. Not doing
 * so will completely block the bus.
 *
 * \note This function is only available on ATmega128 systems.
 *
 * \param txdata Points to the data to transmit. Ignored, if the
 *               number of bytes to transmit is zero.
 * \param txlen  Number of data bytes to transmit.
 * \param tmo    Timeout in milliseconds. To disable timeout,
 *               set this parameter to NUT_WAIT_INFINITE.
 *
 * \return The number of bytes transmitted, -1 in case of an error or timeout.
 */

int NutTwiSlaveRespond(NUTTWIBUS *bus, void *txdata, uint16_t txlen, uint32_t tmo)
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    int rc = -1;
    NUTTWIICB *icb = bus->bus_icb;

    /* The bus is blocked. No critical section required. */
    icb->tw_sm_txbuf = txdata;
    icb->tw_sm_txlen = txlen;

    /*
     * If there is anything to transmit, start the interface.
     */
    if (txlen) {
        NutEnterCritical();
        /* Clear the queue. */
        //*broken?! NutEventBroadcastAsync(&tw_sm_txmtx);
        if (icb->tw_sm_txmtx == SIGNALED) {
            icb->tw_sm_txmtx = 0;
        }

        /* Release the bus, accepting SLA+R. */
        outb(TWCR, TWGO | _BV(TWEA));

        NutExitCritical();
        if (NutEventWait(&icb->tw_sm_txmtx, tmo)) {
            icb->tw_sm_err = TWERR_TIMEOUT;
        }

        NutEnterCritical();
        icb->tw_sm_txlen = 0;
        if (icb->tw_sm_err) {
            icb->tw_sm_error = icb->tw_sm_err;
        } else {
            rc = icb->tw_sm_txidx;
        }
        NutExitCritical();
    }

    /*
     * Nothing to transmit.
     */
    else {
        rc = 0;
        /* Release the bus, not accepting SLA+R. */

        NutEnterCritical();
        inb(TWCR);
        inb(TWSR);
        /* Transmit start condition, if a master transfer is waiting. */
        if (icb->tw_mm_txlen || icb->tw_mm_rxlen) {
            outb(TWCR, TWGO | _BV(TWSTA));
        }
        /* Otherwise enter idle state. */
        else {
            icb->tw_if_busy = 0;
            outb(TWCR, TWGO);
        }

        NutExitCritical();
    }
    return rc;
#endif /* __AVR_ENHANCED__ */
}

/*!
 * \brief Get last slave mode error.
 *
 * You may call this function to determine the specific cause
 * of an error after TwSlaveListen() or TwSlaveRespond() failed.
 *
 * \note This function is only available on ATmega128 systems.
 *
 */

int NutTwiSlaveError(NUTTWIBUS *bus)
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    int rc = bus->bus_icb->tw_sm_error;
    bus->bus_icb->tw_sm_error = 0;
    return rc;
#endif
}


int NutTwiSetSpeed( NUTTWIBUS *bus, uint32_t speed)
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    int rc = -1;
    uint32_t lval;

    if (speed > 400000) {
        /* Speed out of range */
        return rc;
    }

    if (bus==NULL) {
        /* No bus selected */
        return rc;
    }

    lval = ((2UL * NutGetCpuClock() / speed + 1UL) / 2UL - 16UL) / 2UL;
    if (lval > 1020UL) {
        lval /= 16UL;
        sbi(TWSR, TWPS1);
    } else {
        cbi(TWSR, TWPS1);
    }
    if (lval > 255UL) {
        lval /= 4UL;
        sbi(TWSR, TWPS0);
    } else {
        cbi(TWSR, TWPS0);
    }
    if (lval > 9UL && lval < 256UL) {
        outb(TWBR, (uint8_t) lval);
        rc = 0;
    } else {
        rc = -1;
    }

    return rc;
#endif
}

/*!
 * \brief Request Current Speed of I2C Interface.
 *
 * \return 0..400000 for speed, -1 in case of error.
 */
int NutTwiGetSpeed( NUTTWIBUS *bus, uint32_t *speed)
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    uint32_t lval;

    if (bus) {
        lval = 2UL;
        if (bit_is_set(TWSR, TWPS0)) {
            lval *= 4UL;
        }
        if (bit_is_set(TWSR, TWPS1)) {
            lval *= 16UL;
        }
        *speed = NutGetCpuClock() / (16UL + lval * (uint32_t) inb(TWBR));
    }
    return 0;
#endif
}

/*!
 * \brief Perform TWI control functions.
 *
 * This function is only available on ATmega128 systems.
 *
 * \param req  Requested control function. May be set to one of the
 *         following constants:
 *         - TWI_SETSPEED, if conf points to an uint32_t value containing the bitrate.
 *         - TWI_GETSPEED, if conf points to an uint32_t value receiving the current bitrate.
 * \param conf Points to a buffer that contains any data required for
 *         the given control function or receives data from that
 *         function.
 * \return 0 on success, -1 otherwise.
 *
 * \note Timeout is limited to the granularity of the system timer.
 *
 */

int NutTwiIOCtl( NUTTWIBUS *bus, int req, void *conf )
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    int rc = 0;
    NUTTWIICB *icb = bus->bus_icb;

    switch (req) {

    case TWI_SETSPEED:
        rc = NutTwiSetSpeed(bus, *((uint32_t *)conf));
        break;

    case TWI_GETSPEED:
        rc = NutTwiGetSpeed(bus, ((uint32_t *)conf));
        break;

    case TWI_GETSTATUS:
        rc = 0;
        break;

    case TWI_SETSTATUS:
        rc = 0;
        break;

    case TWI_GETSLAVEADDRESS:
        // TODO: Slave handling
        *((uint8_t *)conf) = TWAR;
        break;

    case TWI_SETSLAVEADDRESS:
        // TODO: Slave handling
        TWAR = (*((uint8_t *) conf) << 1) | 1;
        icb->tw_mm_sla = *((uint16_t*)conf);
        break;

    default:
        rc = -1;
        break;
    }
    return rc;
#endif /* __AVR_ENHANCED__ */
}

/*!
 * \brief Initialize TWI interface bus.
 *
 * The specified slave address is used only, if the local system
 * is running as a slave. Anyway, care must be taken that it doesn't
 * conflict with another connected device.
 *
 * \note This function is only available on ATmega128 systems.
 *
 * \param sla Slave address, must be specified as a 7-bit address,
 *            always lower than 128.
 */
int NutRegisterTwiBus( NUTTWIBUS *bus, uint8_t sla )
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    int rc = 0;
    uint32_t speed = 2400;
    NUTTWIICB *icb = NULL;

    /* Check if bus was already registered */
    if( bus->bus_icb) {
        return 0;
    }

    /* Allocate ICB for this bus */
    icb = NutHeapAlloc(sizeof(NUTTWIICB));
    if( icb == NULL) {
        return rc;
    }
    memset( icb, 0, sizeof(NUTTWIICB));

    /* Link bus and ICB */
    bus->bus_icb = icb;

    /* Initialize interface hardware */
    if (bus->bus_initbus) {
        rc = bus->bus_initbus();
    }

    /*
     * Set address register, enable general call address, set transfer
     * speed and enable interface.
     */
    outb(TWAR, (sla << 1) | 1);

#ifdef I2C_DEFAULT_SPEED
    speed = I2C_DEFAULT_SPEED*1000UL;
#endif
    /* Set initial rate. */
    if( (rc = NutTwiSetSpeed( bus, speed))) {
        return rc;
    }

    /* Register IRQ Handler */
    if( (rc = NutRegisterIrqHandler( bus->bus_sig_ev, TwInterrupt, bus))) {
        return rc;
    }

    /* Enable level triggered interrupts. */
    NutIrqEnable(bus->bus_sig_ev);

    outb(TWCR, _BV(TWINT));
    outb(TWCR, _BV(TWEN) | _BV(TWIE));

    /* Initialize mutex semaphores. */
    NutEventPost(&bus->bus_mutex);

    return rc;
#endif
}

int NutDestroyTwiBus( NUTTWIBUS *bus)
{
#ifndef __AVR_ENHANCED__
    return -1;
#else
    if (bus->bus_icb) {
        NutIrqDisable(bus->bus_sig_ev);
        NutHeapFree( bus->bus_icb);
    }

    return 0;
#endif
}

/*!
 * \brief TWI/I2C bus structure.
 */
NUTTWIBUS AVRTwiBus = {
  /*.bus_base =   */  0,                   /* Bus base address. */
#ifndef __AVR_ENHANCED__
  /*.bus_sig_ev = */  NULL,                /* Bus data and event interrupt handler. */
#else
  /*.bus_sig_ev = */ &sig_2WIRE_SERIAL,    /* Bus data and event interrupt handler. */
#endif
  /*.bus_sig_er = */  NULL,                /* Bus error interrupt handler. */
  /*.bus_mutex =  */  NULL,                /* Bus lock queue. */
  /*.bus_icb   =  */  NULL,                /* Bus Runtime Data Pointer */
  /*.bus_dma_tx = */  0,                   /* DMA channel for TX direction. */
  /*.bus_dma_rx = */  0,                   /* DMA channel for RX direction. */
  /*.bus_initbus =*/  NULL,                /* Initialize bus controller. */
  /*.bus_recover =*/  NULL,                /* Recover bus controller */
};
