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

#include <string.h>
#include <cfg/twi.h>
#include <arch/m68k.h>
#include <dev/twif.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <sys/heap.h>

#define MODE_READ       1       /* Work as Receiver */
#define MODE_WRITE      0       /* Work as Transmitter */

/*
 * TWI interrupt handler.
 */
static void TwInterrupt(void *arg)
{
    NUTTWIBUS *bus = (NUTTWIBUS *) arg;
    NUTTWIICB *icb = bus->bus_icb;

    /* Is device in master mode? */
    if (MCF_I2C_I2CR(bus->bus_base) & MCF_I2C_I2CR_MSTA) {

        /* Is device in Tx mode? */
        if (MCF_I2C_I2CR(bus->bus_base) & MCF_I2C_I2CR_MTX) {

            /* Terminate transaction if NACK received */
            if (MCF_I2C_I2SR(bus->bus_base) & MCF_I2C_I2SR_RXAK) {
                icb->tw_mm_err = icb->tw_mm_sla_found ? TWERR_DATA_NACK : TWERR_SLA_NACK;
                goto stop;
            }

            /* ACK Received - transmit next byte or switch to receive mode */
            else {

                /* Slave device is responding */
                icb->tw_mm_sla_found = 1;

                /* Transmit address byte is any */
                if (icb->tw_mm_iadrlen) {
                    icb->tw_mm_iadrlen--;
                    MCF_I2C_I2DR(bus->bus_base) = *(icb->tw_mm_iadr)++;
                }

                /* Or transmit data byte if any */
                else if (icb->tw_mm_txlen) {
                    icb->tw_mm_txlen--;
                    MCF_I2C_I2DR(bus->bus_base) = *(icb->tw_mm_txbuf)++;
                }

                /* Or switch to Rx mode if receiving is configured */
                else if (icb->tw_mm_rxlen) {

                    /* Repeated start is required for changing from Tx mode to Rx mode */
                    if (icb->tw_mm_dir == MODE_WRITE) {
                        icb->tw_mm_dir = MODE_READ;

                        /* Generate repeated START condition */
                        MCF_I2C_I2CR(bus->bus_base) |= MCF_I2C_I2CR_RSTA;

                        /* Write slave id byte with READ flag */
                        MCF_I2C_I2DR(bus->bus_base) = (uint8_t) (icb->tw_mm_sla | 0x01);
                    }

                    /* Start receiving */
                    else {

                        /* Suppress ACK signal in response if only one char to receive */
                        if (icb->tw_mm_rxlen == 1U) {
                            MCF_I2C_I2CR(bus->bus_base) |= MCF_I2C_I2CR_TXAK;
                        } else {
                            MCF_I2C_I2CR(bus->bus_base) &= ~MCF_I2C_I2CR_TXAK;
                        }

                        /* Switch to Rx mode */
                        MCF_I2C_I2CR(bus->bus_base) &= ~MCF_I2C_I2CR_MTX;

                        /* Start receiving by dummy read */
                        (void) MCF_I2C_I2DR(bus->bus_base);
                    }
                }

                /* Or finish the transaction */
                else {
                    goto stop;
                }
            }
        } else {

            /* Decrease number of chars for the receive */
            icb->tw_mm_rxlen--;

            /* Suppress ACK signal in response if only one char to receive */
            if (icb->tw_mm_rxlen == 1U) {
                MCF_I2C_I2CR(bus->bus_base) |= MCF_I2C_I2CR_TXAK;
            }

            /* Finish the transaction (Generate STOP) if all data are received */
            else if (!icb->tw_mm_rxlen) {
                MCF_I2C_I2CR(bus->bus_base) &= ~MCF_I2C_I2CR_MSTA;
            }

            /* Receive character */
            *(icb->tw_mm_rxbuf)++ = MCF_I2C_I2DR(bus->bus_base);

            /* Finish the transaction (Generate STOP) if all data are received */
            if (!icb->tw_mm_rxlen)
                goto stop;
        }
    } else {

        /* Arbitration lost? */
        if (MCF_I2C_I2SR(bus->bus_base) & MCF_I2C_I2SR_IAL) {
            icb->tw_mm_err = TWERR_ARBLOST;
            goto stop;
        }
    }

    return;

stop:
    /* Switch master to slave if required (Generate STOP) */
    if (MCF_I2C_I2CR(bus->bus_base) & MCF_I2C_I2CR_MSTA)
        MCF_I2C_I2CR(bus->bus_base) &= ~MCF_I2C_I2CR_MSTA;

    /* Switch to Rx mode */
    MCF_I2C_I2CR(bus->bus_base) &= ~MCF_I2C_I2CR_MTX;

    /* Wake up waiting thread */
    NutEventPostFromIrq(&icb->tw_mm_mtx);

    return;
}

/*
 * TWI Transfer starter
 */
static int TwiInitTransfer(NUTTWIBUS *bus, uint32_t tmo)
{
    NUTTWIICB *icb = bus->bus_icb;

    /* Wait until the bus is not busy (e.g. another master is communicating) */
    while (MCF_I2C_I2SR(bus->bus_base) & MCF_I2C_I2SR_IBB) {

        NutSleep(1);

        if (tmo == NUT_WAIT_INFINITE) {
            continue;
        }

        if (!--tmo) {
            icb->tw_mm_error = TWERR_BUSY;
            return -1;
        }
    }

    /* Set TX mode */
    MCF_I2C_I2CR(bus->bus_base) |= MCF_I2C_I2CR_MTX;

    /* Clear status register */
    MCF_I2C_I2SR(bus->bus_base) = 0;

    // TODO: I'm not sure this critical section is really required
    // try to use NutSleep() here to simulate interrupt
    //NutEnterCritical();

    /* Generate "start" or "repeat start" */
    if (MCF_I2C_I2CR(bus->bus_base) & MCF_I2C_I2CR_MSTA) {
        MCF_I2C_I2CR(bus->bus_base) |= MCF_I2C_I2CR_RSTA;
    } else {
        MCF_I2C_I2CR(bus->bus_base) |= MCF_I2C_I2CR_MSTA;
    }

    /* Transmit the slave address */
    MCF_I2C_I2DR(bus->bus_base) = icb->tw_mm_sla;

    //NutExitCritical();

    return 0;
}

/*
 * TWI Transfer
 */
static int TwiMasterLow(NUTTWIBUS *bus, uint8_t sla, uint32_t iadr, uint8_t iadrlen, CONST void *txdata, uint16_t txlen,
        void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
    int rc = -1;
    NUTTWIICB *icb = bus->bus_icb;

    /* Quit if nothing to do */
    if ((txlen == 0) && (rxsiz == 0)) {
        return 0;
    }

    /* This routine is marked reentrant, so lock the interface. */
    if (NutEventWait(&bus->bus_mutex, tmo)) {
        icb->tw_mm_error = TWERR_IF_LOCKED;
        return rc;
    }

    /* Fetch transfer parameters for current transaction */
    icb->tw_mm_sla = sla << 1;
    icb->tw_mm_sla_found = 0;
    icb->tw_mm_iadrlen = iadrlen;
    if (iadrlen) {
        /* Big-endian machine! */
        icb->tw_mm_iadr = ((uint8_t*) &iadr) + 4 - iadrlen;
    }
    icb->tw_mm_txbuf = (uint8_t*) txdata;
    icb->tw_mm_txlen = txlen;
    icb->tw_mm_rxbuf = rxdata;
    icb->tw_mm_rxlen = rxsiz;
    icb->tw_mm_err = TWERR_OK;

    if (icb->tw_mm_iadrlen | icb->tw_mm_txlen)
        icb->tw_mm_dir = MODE_WRITE;
    else
        icb->tw_mm_dir = MODE_READ;

    /* Issue start and wait till transmission completed */
    if (!TwiInitTransfer(bus, tmo)) {

        /* Wait for transfer complete. */
        if (NutEventWait(&icb->tw_mm_mtx, tmo)) {
            icb->tw_mm_error = TWERR_TIMEOUT;
        }

        /* Check for errors that may have been detected by the interrupt routine. */
        if (icb->tw_mm_err) {
            icb->tw_mm_error = icb->tw_mm_err;
        } else {
            if (rxsiz)
                rc = (int) (rxsiz - icb->tw_mm_rxlen);
            else
                rc = (int) (txlen - icb->tw_mm_txlen);
        }
    }

    /* Release the interface. */
    NutEventPost(&bus->bus_mutex);

    return rc;
}

/*!
 * \brief Transmit and/or receive data as a master.
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \param bus     Pointer to the \ref NUTTWIBUS structure, which is provided
 *                by the bus controller driver.
 * \param sla     Slave address of the destination. This slave address
 *                must be specified as a 7-bit address. For example, the
 *                PCF8574A may be configured to slave addresses from 0x38
 *                to 0x3F.
 * \param txdata  Points to the data to transmit. Ignored, if the number
 *                of data bytes to transmit is zero.
 * \param txlen   Number of data bytes to transmit. If zero, then the
 *                interface will not send any data to the slave device
 *                and will directly enter the master receive mode.
 * \param rxdata  Points to a buffer, where the received data will be
 *                stored. Ignored, if the maximum number of bytes to
 *                receive is zero.
 * \param rxsiz   Maximum number of bytes to receive. Set to zero, if
 *                no bytes are expected from the slave device.
 * \param tmo     Timeout in milliseconds. To disable timeout, set this
 *                parameter to NUT_WAIT_INFINITE.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int NutTwiMasterTranceive(NUTTWIBUS *bus, uint8_t sla, CONST void *txdata, uint16_t txlen, void *rxdata, uint16_t rxsiz,
        uint32_t tmo)
{
    return TwiMasterLow(bus, sla, 0, 0, txdata, txlen, rxdata, rxsiz, tmo);
}

/*!
 * \brief Receive data as a master from a device having internal addressable registers
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \param bus     Pointer to the \ref NUTTWIBUS structure, which is provided
 *                by the bus controller driver.
 * \param sla     Slave address of the destination. This slave address
 *                must be specified as a 7-bit address. For example, the
 *                PCF8574A may be configured to slave addresses from 0x38
 *                to 0x3F.
 * \param iadr    Address send to the device to access certain registers
 *                or memory addresses of it.
 * \param iadrlen Number of bytes to send as address, maximum 3 bytes are
 *                supported from AT91SAM7
 * \param rxdata  Points to a buffer, where the received data will be
 *                stored.
 * \param rxsiz   Maximum number of bytes to receive.
 * \param tmo     Timeout in milliseconds. To disable timeout, set this
 *                parameter to NUT_WAIT_INFINITE.
 *
 * \return The number of bytes received, -1 in case of an error or timeout.
 */
int NutTwiMasterRegRead(NUTTWIBUS *bus, uint8_t sla, uint32_t iadr, uint8_t iadrlen, void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
    return TwiMasterLow(bus, sla, iadr, iadrlen, NULL, 0, rxdata, rxsiz, tmo);
}

/*!
 * \brief Transmit data as a master to a device having internal addressable registers
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
 * \param bus     Pointer to the \ref NUTTWIBUS structure, which is provided
 *                by the bus controller driver.
 * \param sla     Slave address of the destination. This slave address
 *                must be specified as a 7-bit address. For example, the
 *                PCF8574A may be configured to slave addresses from 0x38
 *                to 0x3F.
 * \param iadr    Address send to the device to access certain registers
 *                or memory addresses of it.
 * \param iadrlen Number of bytes to send as address, maximum 3 bytes are
 *                supported from AT91SAM7
 * \param txdata  Points to a buffer, where the data to transmit will be
 *                stored.
 * \param txsiz   Maximum number of bytes to transmit.
 * \param tmo     Timeout in milliseconds. To disable timeout, set this
 *                parameter to NUT_WAIT_INFINITE.
 *
 * \return        The number of bytes transmitted, -1 in case of an error
 *                or timeout. Number could be less if slave end transmission
 *                with NAK.
 */

int NutTwiMasterRegWrite(NUTTWIBUS *bus, uint8_t sla, uint32_t iadr, uint8_t iadrlen, CONST void *txdata, uint16_t txsiz,
        uint32_t tmo)
{
    return TwiMasterLow(bus, sla, iadr, iadrlen, txdata, txsiz, NULL, 0, tmo);
}

/*!
 * \brief Get last master mode error.
 *
 * You may call this function to determine the specific cause
 * of an error after twi transaction failed.
 *
 * \param bus    Pointer to the \ref NUTTWIBUS structure, which is provided
 *               by the bus controller driver.
 *
 */
int NutTwiMasterError(NUTTWIBUS *bus)
{
    int rc = bus->bus_icb->tw_mm_error;
    bus->bus_icb->tw_mm_error = 0;
    return rc;
}

/*!
 * \brief Listen for incoming data from a master.
 *
 * If this function returns without error, the bus is blocked. The caller
 * must immediately process the request and return a response by calling
 * TwSlaveRespond().
 *
 * \note Slave mode is not implemented in the COLDFIRE driver.
 *       Thus the function always returns -1.
 *
 * \param bus    Pointer to the \ref NUTTWIBUS structure, which is provided
 *               by the bus controller driver.
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
 * \note Slave mode is not implemented in the COLDFIRE driver.
 *       Thus the function always returns -1.
 *
 * \param bus    Pointer to the \ref NUTTWIBUS structure, which is provided
 *               by the bus controller driver.
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
 * \param bus    Pointer to the \ref NUTTWIBUS structure, which is provided
 *               by the bus controller driver.
 *
 * \return Error code or 0 if no error occurred.
 *
 * \note Slave mode is not implemented in the COLDFIRE driver.
 *       Thus the function always returns TWERR_BUS.
 */
extern int NutTwiSlaveError(NUTTWIBUS *bus)
{
    return TWERR_BUS;
}

/*!
 * \brief Get last transfer results.
 *
 * \param bus    Pointer to the \ref NUTTWIBUS structure, which is provided
 *               by the bus controller driver.
 * \param idx    Index requested
 * \todo Do we really need this. It may not work with competing threads.
 * Answer: We really need this! It is bus dependand and is used by
 * EEPROM driver to determine the number of bytes transferred and
 * detection of EEPROM busy event (ACK-Polling).
 *
 * You may call this function to determine how many bytes where
 * transferred before the twi transaction failed.
 *
 */
uint16_t NutTwiIndexes(NUTTWIBUS *bus, uint8_t idx)
{
    NUTTWIICB *icb = bus->bus_icb;

    switch (idx) {
    case 0:
        return (uint16_t) icb->tw_mm_error;
    case 1:
        return (uint16_t) icb->tw_mm_rxlen;
    case 2:
        return (uint16_t) icb->tw_mm_txlen;
    default:
        return 0;
    }
}

/*
 * Set Speed of I2C Interface.
 */
#define IC_SIZE 64
uint16_t Dividers[IC_SIZE] = { 28, 30, 34, 40, 44, 48, 56, 68, 80, 88, 104, 128, 144, 160, 192, 240, 288, 320, 384, 480, 576,
        640, 768, 960, 1152, 1280, 1536, 1920, 2304, 2560, 3072, 3840, 20, 22, 24, 26, 28, 32, 36, 40, 48, 56, 64, 72, 80, 96,
        112, 128, 160, 192, 224, 256, 320, 384, 448, 512, 640, 768, 896, 1024, 1280, 1536, 1792, 2048 };

static int TwiSetSpeed(NUTTWIBUS *bus, uint32_t speed)
{
    uint8_t ic = 0x1F;
    uint16_t div_selected = 0xFFFF;
    uint16_t div_counted = NutGetCpuClock() / speed;
    int i;

    for (i = 0; i < IC_SIZE; ++i) {
        if (Dividers[i] >= div_counted && Dividers[i] < div_selected) {
            div_selected = Dividers[i];
            ic = i;
        }
    }

    MCF_I2C_I2FDR(bus->bus_base) = MCF_I2C_I2FDR_IC(ic);

    return 0;
}

/*
 * Request Current Speed of I2C Interface.
 */
static int TwiGetSpeed(NUTTWIBUS *bus)
{
    return NutGetCpuClock() / Dividers[MCF_I2C_I2FDR(bus->bus_base)];
}

/*!
 * \brief Perform TWI control functions.
 *
 * \param bus    Pointer to the \ref NUTTWIBUS structure, which is provided
 *               by the bus controller driver.
 * \param req    Requested control function. May be set to one of the
 *               following constants:
 *                  - TWI_SETSPEED, if conf points to an uint32_t value containing the bitrate.
 *                  - TWI_GETSPEED, if conf points to an uint32_t value receiving the current bitrate.
 * \param conf   Points to a buffer that contains any data required for
 *               the given control function or receives data from that
 *               function.
 * \return 0 on success, -1 otherwise.
 *
 */
int NutTwiIOCtl(NUTTWIBUS *bus, int req, void *conf)
{
    int rc = 0;
    NUTTWIICB *icb = bus->bus_icb;

    switch (req) {
    case TWI_SETSPEED:
        TwiSetSpeed(bus, *(uint32_t*) conf);
        break;

    case TWI_GETSPEED:
        *(uint32_t*) conf = TwiGetSpeed(bus);
        break;

    case TWI_GETSTATUS:
        break;

    case TWI_SETSTATUS:
        break;

    case TWI_GETSLAVEADDRESS:
        *((uint32_t*) conf) = icb->tw_mm_sla;

        break;

    case TWI_SETSLAVEADDRESS:
        icb->tw_mm_sla = MCF_I2C_I2ADR_ADR(*(uint32_t*)conf);
        MCF_I2C_I2ADR(bus->bus_base) = icb->tw_mm_sla;
        break;

    default:
        rc = -1;
        break;
    }

    return rc;
}

/*!
 * \brief Initialize TWI interface.
 *
 * The specified slave address is not used here as we don't support twi-slave
 * on this architecture for now.
 *
 * \note This function is only available on AT91SAM7xxxx systems.
 *
 * \param bus    Pointer to the \ref NUTTWIBUS structure, which is provided
 *               by the bus controller driver.
 * \param sla    Slave address, must be specified as a 7-bit address,
 *               always lower than 128.
 */
int NutRegisterTwiBus(NUTTWIBUS *bus, uint8_t sla)
{
    int rc = -1;
    NUTTWIICB *icb = NULL;
    uint32_t speed = 100000;

    /* Check if bus was already registered */
    if (bus->bus_icb) {
        return 0;
    }

    /* Allocate ICB for this bus */
    if ((icb = NutHeapAllocClear(sizeof(NUTTWIICB))) == NULL) {
        return rc;
    }

    /* Link bus and ICB */
    bus->bus_icb = icb;

    if (NutRegisterIrqHandler(bus->bus_sig_ev, TwInterrupt, bus)) {
        goto err;
    }

    /* Initialize GPIO Hardware */
    if ((bus->bus_initbus == NULL) || ((rc = bus->bus_initbus()))) {
        goto err;
    }

    /* Disable and reset this bus */
    MCF_I2C_I2CR(bus->bus_base) = 0;

    /* Set initial rate. */
#ifdef I2CBUS0_DEFAULT_SPEED
    if (bus->bus_base == 0)
        speed = I2CBUS0_DEFAULT_SPEED * 1000UL;
#endif
#ifdef I2CBUS1_DEFAULT_SPEED
    if (bus->bus_base == 1)
        speed = I2CBUS1_DEFAULT_SPEED * 1000UL;
#endif

    if ((rc = TwiSetSpeed(bus, speed))) {
        goto err;
    }

    /* Define slave address (the address the I2C responds to when addressed as a slave) */
    icb->tw_mm_sla = MCF_I2C_I2ADR_ADR(sla);
    MCF_I2C_I2ADR(bus->bus_base) = icb->tw_mm_sla;

    /* Enable the I2C */
    MCF_I2C_I2CR(bus->bus_base) = MCF_I2C_I2CR_IEN;

    /* Recover the bus if a slave hangs with SCL low */
    if (MCF_I2C_I2SR(bus->bus_base) & MCF_I2C_I2SR_IBB) {

        /* Send a START condition*/
        MCF_I2C_I2CR(bus->bus_base) |= MCF_I2C_I2CR_MSTA;

        /* Dummy read */
        (void) MCF_I2C_I2DR(bus->bus_base);

        /* Clear status register */
        MCF_I2C_I2SR(bus->bus_base) = 0;

        /* Clear control register */
        MCF_I2C_I2CR(bus->bus_base) = 0;

        /* Enable the I2C */
        MCF_I2C_I2CR(bus->bus_base) = MCF_I2C_I2CR_IEN;
    }

    /* Enable I2C Interrupts */
    MCF_I2C_I2CR(bus->bus_base) |= MCF_I2C_I2CR_IIEN;
    if ((rc = NutIrqEnable(bus->bus_sig_ev))) {
        goto err;
    }

    /* Initialize mutex semaphores. */
    NutEventPost(&bus->bus_mutex);

    return rc;

err:
    if (icb) {
        NutHeapFree(icb);
    }

    return rc;
}

/*@}*/
