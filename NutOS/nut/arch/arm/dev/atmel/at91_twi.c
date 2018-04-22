/*
 * Copyright (C) 2001-2005 by EmbeddedIT, Ole Reinhardt
 * Copyright (C) 2009 by Rittal GmbH & Co. KG, Ulrich Prinz
 * Copyright 2009 by egnite GmbH
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

/*
 * $Id: at91_twi.c 5472 2013-12-06 00:16:28Z olereinhardt $
 */

#include <arch/arm.h>
#include <dev/irqreg.h>

#include <string.h>

#include <sys/event.h>
#include <sys/atom.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/heap.h>

#include <dev/twif.h>

/*!
 * \addtogroup xgNutArchArmAt91Twi
 */
/*@{*/

/*
 * Set ports for known targets.
 */
#if defined(MCU_AT91SAM7X)
#define TWI_TWD     PA10_TWD_A
#define TWI_TWCK    PA11_TWCK_A
#elif defined(MCU_AT91SAM7S) || defined(MCU_AT91SAM7SE)
#define TWI_TWD     PA3_TWD_A
#define TWI_TWCK    PA4_TWCK_A
#elif defined (MCU_AT91SAM9260) || defined(MCU_AT91SAM9XE)
#define TWI_TWD     PA23_TWD_A
#define TWI_TWCK    PA24_TWCK_A
#elif defined (MCU_AT91SAM9G45)
#define TWI_TWD     PA20_TWD0_A
#define TWI_TWCK    PA21_TWCK0_A
#endif

/*
 * Set port defaults, assume PIOA.
 */
#ifndef TWI_PIO_ASR
#define TWI_PIO_ASR     PIOA_ASR
#endif
#ifndef TWI_PIO_PDR
#define TWI_PIO_PDR     PIOA_PDR
#endif
#ifndef TWI_PIO_MDER
#define TWI_PIO_MDER    PIOA_MDER
#endif

#if 0
static uint8_t *tw_mm_buf;              /* Pointer to the master transfer buffer. */
static volatile size_t tw_mm_len;       /* Number of bytes to transmit in master mode. */
static volatile size_t tw_mm_idx;       /* Current master transmit buffer index. */
#endif

/*
 * TWI interrupt handler.
 */
static void TwInterrupt(void *arg)
{
    register unsigned int twsr;
    NUTTWIBUS *bus = arg;
    NUTTWIICB *icb = bus->bus_icb;

    /* Read the status register and check for errors. */
    twsr = inr(TWI_SR);
    if (twsr & (TWI_NACK | TWI_OVRE | TWI_ARBLST)) {
        if (twsr & TWI_NACK) {
            icb->tw_mm_err = TWERR_SLA_NACK;
        } else {
            icb->tw_mm_err = TWERR_BUS;
        }
    }

    /* Mask inactive interrupt flags. */
    twsr &= inr(TWI_IMR);
    if (twsr & TWI_TXRDY) {
        /* Byte has been transmitted. */
        if (icb->tw_mm_idx < icb->tw_mm_len) {
            /* More bytes in buffer, send next one. */
            outb(TWI_THR, icb->tw_mm_buf[icb->tw_mm_idx]);
            icb->tw_mm_idx++;
        } else {
            /* All bytes sent, wait for completion. */
#if defined(MCU_AT91SAM9XE)
            /* On the SAM9XE the stop condition is not sent automatically
               when the transmitter runs out of data. */
            outr(TWI_CR, TWI_STOP);
#endif
            outr(TWI_IDR, TWI_TXRDY);
            outr(TWI_IER, TWI_TXCOMP);
        }
    } else if (twsr & TWI_RXRDY) {
        /* Byte has been received. */
        if (icb->tw_mm_idx < icb->tw_mm_len) {
            /* Store byte in receive buffer. */
            icb->tw_mm_buf[icb->tw_mm_idx++] = inb(TWI_RHR);
            if (icb->tw_mm_idx == icb->tw_mm_len - 1) {
                /* Next byte will be last, set STOP condition. */
                outr(TWI_CR, TWI_STOP);
            } else if (icb->tw_mm_idx == icb->tw_mm_len) {
                /* This was the last byte, wait for completion. */
                outr(TWI_IDR, TWI_RXRDY);
                outr(TWI_IER, TWI_TXCOMP);
            }
        }
    } else if (twsr & TWI_TXCOMP) {
        /* Transfer is complete, disable interrupts
        ** and signal waiting threads */
        outr(TWI_IDR, 0xFFFFFFFF);
        NutEventPostFromIrq(&icb->tw_mm_mtx);
    }
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
 */
int NutTwiMasterTranceive(NUTTWIBUS *bus, uint8_t sla, const void *txdata, uint16_t txlen, void *rxdata, uint16_t rxsiz, uint32_t tmo)
{
    int rc = -1;
    NUTTWIICB *icb = bus->bus_icb;

    /* This routine is marked reentrant, so lock the interface. */
    if (NutEventWait(&bus->bus_mutex, tmo)) {
        icb->tw_mm_error = TWERR_IF_LOCKED;
        return -1;
    }

    /* Clear errors. */
    icb->tw_mm_err = 0;

    /* Check if there is something to transmit. */
    if (txlen) {
        /* Set transmit parameters. */
        icb->tw_mm_buf = (uint8_t *) txdata;
        icb->tw_mm_len = (size_t) txlen;
        icb->tw_mm_idx = 1;

        /* Start transmit to specified slave address. */
        outr(TWI_MMR, (unsigned int) sla << TWI_DADR_LSB);
        outr(TWI_THR, icb->tw_mm_buf[0]);
        outr(TWI_IER, TWI_TXRDY);

        /* Wait for transmission complete. */
        // TODO: Mustn't this be NutEventWaitNext() ??
        if (NutEventWait(&icb->tw_mm_mtx, tmo)) {
            icb->tw_mm_err = TWERR_TIMEOUT;
        }
    }

    if (icb->tw_mm_err == 0) {
        /* Reset receive counter. We need this below to set
        ** the return value. */
        icb->tw_mm_idx = 0;
        /* Check if there is something to receive. */
        if (rxsiz) {
            /* Set remaining receive parameters. */
            icb->tw_mm_buf = (uint8_t *) rxdata;
            icb->tw_mm_len = (size_t) rxsiz;

            /* Start receive from specified slave address. */
            outr(TWI_MMR, ((unsigned int) sla << TWI_DADR_LSB) | TWI_MREAD);
            if (rxsiz == 1) {
                /* Set STOP condition if this is the last byte. */
                outr(TWI_CR, TWI_START | TWI_STOP);
            } else {
                outr(TWI_CR, TWI_START);
            }
            outr(TWI_IER, TWI_RXRDY);

            /* Wait for receiver complete. */
            if (NutEventWait(&icb->tw_mm_mtx, tmo)) {
                icb->tw_mm_err = TWERR_TIMEOUT;
            }
        }
    }
    /* Make sure that all interrupts are disabled. */
    outr(TWI_IDR, 0xFFFFFFFF);

    /* Check for errors that may have been detected
    ** by the interrupt routine. */
    if (icb->tw_mm_err) {
        icb->tw_mm_error = icb->tw_mm_err;
    } else {
        rc = (int) icb->tw_mm_idx;
    }

    /* Release the interface. */
    NutEventPost(&bus->bus_mutex);

    return rc;
}

/*!
 * \brief Receive data as a master from a device having internal addressable registers
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
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
    int rc = -1;
    NUTTWIICB *icb = bus->bus_icb;

    if (rxsiz == 0) {
        return -1;
    }
    /* This routine is marked reentrant, so lock the interface. */
    if (NutEventWait(&bus->bus_mutex, tmo)) {
        icb->tw_mm_error = TWERR_IF_LOCKED;
        return -1;
    }

    icb->tw_mm_err = 0;

    if (rxsiz) {
        icb->tw_mm_buf = rxdata;
        icb->tw_mm_len = (size_t) rxsiz;
        icb->tw_mm_idx = 0;

        /* Set TWI Internal Address Register if needed */
        outr(TWI_IADRR, iadr);
        /* Set the TWI Master Mode Register */
        outr(TWI_MMR, ((unsigned int) sla << TWI_DADR_LSB) | (((unsigned int) iadrlen & 3) << 8) | TWI_MREAD);
        /* Send start condition. If read only one byte send stop as well */
        if (rxsiz == 1) {
            outr(TWI_CR, TWI_START | TWI_STOP);
        } else {
            outr(TWI_CR, TWI_START);
        }
        outr(TWI_IER, TWI_RXRDY);

        /* Wait for master transmission to be done. */
        if (NutEventWait(&icb->tw_mm_mtx, tmo)) {
            icb->tw_mm_error = TWERR_TIMEOUT;
        } else if (icb->tw_mm_err) {
            icb->tw_mm_error = icb->tw_mm_err;
        } else {
            rc = (int) icb->tw_mm_idx;
        }
        outr(TWI_IDR, 0xFFFFFFFF);
    }

    /* Release the interface. */
    NutEventPost(&bus->bus_mutex);

    return rc;
}

/*!
 * \brief Transmit data as a master to a device having internal addressable registers
 *
 * The two-wire serial interface must have been initialized by calling
 * TwInit() before this function can be used.
 *
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

int NutTwiMasterRegWrite(NUTTWIBUS *bus,
                         uint8_t sla,
                         uint32_t iadr,
                         uint8_t iadrlen,
                         const void *txdata,
                         uint16_t txsiz,
                         uint32_t tmo)
{
    int rc = -1;
    NUTTWIICB *icb = bus->bus_icb;

    /* This routine is marked reentrant, so lock the interface. */
    if (NutEventWait(&bus->bus_mutex, tmo)) {
        icb->tw_mm_err = TWERR_IF_LOCKED;
        return -1;
    }

   icb->tw_mm_err = 0;

    if (txsiz) {
        icb->tw_mm_buf = (uint8_t *) txdata;
        icb->tw_mm_len = (size_t) txsiz;
        icb->tw_mm_idx = 1;

        /* Set TWI Internal Address Register */
        outr(TWI_IADRR, iadr);
        /* Set the TWI Master Mode Register */
        outr(TWI_MMR, ((unsigned int) sla << TWI_DADR_LSB) | (((unsigned int) iadrlen & 3) << 8));
        outb(TWI_THR, icb->tw_mm_buf[0]);
        outr(TWI_IER, TWI_TXRDY);

        /* Wait for master transmission to be done. */
        if (NutEventWait(&icb->tw_mm_mtx, tmo)) {
            icb->tw_mm_error = TWERR_TIMEOUT;
        } else if (icb->tw_mm_err) {
            icb->tw_mm_error = icb->tw_mm_err;
        } else {
            rc = (int) icb->tw_mm_idx;
        }
        outr(TWI_IDR, 0xFFFFFFFF);
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
 * \note Slave mode is not implemented in the AT91 driver.
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
 * \note Slave mode is not implemented in the AT91 driver.
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
 * \note Slave mode is not implemented in the AT91 driver.
 *       Thus the function always returns TWERR_BUS.
 */
extern int NutTwiSlaveError(NUTTWIBUS *bus)
{
    return TWERR_BUS;
}

/*!
 * \brief Get last transfer results.
 *
 * \todo Do we really need this. It may not work with competing threads.
 *
 * You may call this function to determine how many bytes where
 * transferred before the twi transaction failed.
 *
 */
uint16_t NutTwiIndexes( NUTTWIBUS *bus, uint8_t idx )
{
    NUTTWIICB *icb = bus->bus_icb;

    switch ( idx ) {
    case 0:
        return (uint16_t) icb->tw_mm_error;
    case 1:
        return (uint16_t) icb->tw_mm_len;
    case 2:
        return (uint16_t) icb->tw_mm_len;
    default:
        return 0;
    }
}

/*!
 * \brief Set Speed of I2C Interface.
 *
 * Setup Interface Speed
 */
int NutTwiSetSpeed( NUTTWIBUS *bus, uint32_t speed)
{
    int rc = -1;
    uint32_t ckdiv = 1;
    uint32_t cldiv;

    if (speed > 400000) {
        /* Speed out of range */
        return rc;
    }

    if (bus==NULL) {
        /* No bus selected */
        return rc;
    }

    /*
     * CLDIV = ((Tlow x 2^CKDIV) -3) x Tmck
     * CHDIV = ((THigh x 2^CKDIV) -3) x Tmck
     * Only CLDIV is computed since CLDIV = CHDIV (50% duty cycle)
     */

    while ((cldiv = ((NutClockGet(NUT_HWCLK_PERIPHERAL) / (2 * speed)) - 3) / (1 << ckdiv)) > 255) {
        ckdiv++;
    }

    /* BUG 41.2.7.1, datasheet SAM7X256  p. 626 */
    if (cldiv * (1 << ckdiv) > 8191) {
        return rc;
    }
    else {
        outr(TWI_CWGR, (ckdiv << 16) | ((unsigned int) cldiv << 8) | (unsigned int) cldiv);
        rc = 0;
    }

    return 0;
}

/*!
 * \brief Request Current Speed of I2C Interface.
 *
 * \return 0..400000 for speed, -1 in case of error.
 */
uint32_t NutTwiGetSpeed( NUTTWIBUS *bus)
{
    uint32_t rc;
    uint32_t ckdiv = 1;
    uint32_t cldiv;

    if (bus) {
        cldiv = inr(TWI_CWGR) & 0x000000FF;
        ckdiv = (inr(TWI_CWGR) >> 16) & 0x00000007;

        rc = NutGetCpuClock() / ((cldiv * 2 << ckdiv) - 3);
    }
    else
        rc = -1;
    return rc;
}
/*!
 * \brief Perform TWI control functions.
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
    int rc = 0;
    NUTTWIICB *icb = bus->bus_icb;

    switch (req) {

    case TWI_SETSPEED:
        rc = NutTwiSetSpeed(bus, *((uint32_t *)conf));
        break;

    case TWI_GETSPEED:
        *((uint32_t *)conf) = NutTwiGetSpeed(bus);
        break;

    case TWI_GETSTATUS:
        rc = 0;
        break;

    case TWI_SETSTATUS:
        rc = 0;
        break;

    case TWI_GETSLAVEADDRESS:
        // TODO: Slave handling
        *(uint32_t*)conf = (uint32_t)icb->tw_mm_sla;
        break;

    case TWI_SETSLAVEADDRESS:
        // TODO: Slave handling
        icb->tw_mm_sla = *((uint16_t*)conf);
        break;

    default:
        rc = -1;
        break;
    }
    return rc;
}

/*!
 * \brief Initialize TWI interface hardware.
 */
int At91TwiInit(void)
{
    int rc = 0;

    /* Set TWD and TWCK as peripheral line. */
    outr(TWI_PIO_ASR, _BV(TWI_TWD) | _BV(TWI_TWCK));
    /* Let periperal control the PIO lines. */
    outr(TWI_PIO_PDR, _BV(TWI_TWD) | _BV(TWI_TWCK));
    /* Enabled OpenDrain output on both lines. */
    outr(TWI_PIO_MDER, _BV(TWI_TWD) | _BV(TWI_TWCK));
    /* Enable TWI clock in PMC. */
    outr(PMC_PCER, _BV(TWI_ID));

    /* Disable all interrupts. */
    outr(TWI_IDR, 0xFFFFFFFF);
    /* Reset bus. */
    outr(TWI_CR, TWI_SWRST);
    /* Enable master mode. */
    outr(TWI_CR, TWI_MSEN | TWI_SVDIS);

    return rc;
}

/*!
 * \brief Initialize TWI interface bus.
 *
 * The specified slave address is not used here as we don't support twi-slave
 * on AT91SAM7X
 *
 * \note This function is only available on AT91SAM7xxxx systems.
 *
 * \param sla Slave address, must be specified as a 7-bit address,
 *            always lower than 128.
 */
int NutRegisterTwiBus( NUTTWIBUS *bus, uint8_t sla )
{
    int rc = 0;
    uint32_t speed = 100000;
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
    NutIrqSetMode(bus->bus_sig_ev, NUT_IRQMODE_LEVEL);
    NutIrqEnable(bus->bus_sig_ev);

    /* Initialize mutex semaphores. */
    NutEventPost(&bus->bus_mutex);

    return rc;
}

int NutDestroyTwiBus( NUTTWIBUS *bus)
{
    if (bus->bus_icb) {
        NutIrqDisable(bus->bus_sig_ev);
        NutHeapFree( bus->bus_icb);
    }

    return 0;
}

/*!
 * \brief TWI/I2C bus structure.
 */
NUTTWIBUS At91TwiBus = {
  /*.bus_base =   */  TWI_CR,      /* Bus base address. */
  /*.bus_sig_ev = */ &sig_TWI,     /* Bus data and event interrupt handler. */
  /*.bus_sig_er = */  NULL,        /* Bus error interrupt handler. */
  /*.bus_mutex =  */  NULL,        /* Bus lock queue. */
  /*.bus_icb   =  */  NULL,        /* Bus Runtime Data Pointer */
  /*.bus_dma_tx = */  0,           /* DMA channel for TX direction. */
  /*.bus_dma_rx = */  0,           /* DMA channel for RX direction. */
  /*.bus_initbus =*/  At91TwiInit, /* Initialize bus controller. */
  /*.bus_recover =*/  NULL,        /* Recover bus controller */
};

/*@}*/
