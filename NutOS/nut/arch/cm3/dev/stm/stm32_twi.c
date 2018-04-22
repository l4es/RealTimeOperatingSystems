/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
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
 * \verbatim
 * $Id: stm32_twi.c 6367 2016-03-15 10:27:09Z u_bonnes $
 * \endverbatim
 */

#include <arch/cm3.h>
#include <dev/irqreg.h>

#include <sys/event.h>
#include <sys/atom.h>
#include <sys/timer.h>
#include <sys/thread.h>
#include <sys/heap.h>

#include <cfg/twi.h>
#include <dev/twif.h>
#include <stdlib.h>
#include <string.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_clk.h>
#if defined(I2CBUS1_USE_DMA) || defined(I2CBUS2_USE_DMA)
#if defined(MCU_STM32F1)
    #include <arch/cm3/stm/stm32f1_dma.h>
#else
#warning "Unhandled STM32 family"
#endif
#endif

//#define TWI_DEBUG

#ifdef TWI_DEBUG
#include <stdio.h>
#include <dev/gpio.h>
#define TPRINTF(...) printf(__VA_ARGS__); fflush(stdout)
#define DBGP1_INIT() GpioPinConfigSet(NUTGPIO_PORTB, 1, GPIO_CFG_OUTPUT)
#define DBGP1(v) GpioPinSet(NUTGPIO_PORTB, 1, v)
#define DBGP2_INIT() GpioPinConfigSet(NUTGPIO_PORTB, 9, GPIO_CFG_OUTPUT)
#define DBGP2(v) GpioPinSet(NUTGPIO_PORTB, 9, v)
#else
#define TPRINTF(...)
#define DBGP1_INIT()
#define DBGP1(v)
#define DBGP2_INIT()
#define DBGP2(v)
#endif


#define MODE_FLAG_Mask          ((uint32_t)0x00FFFFFF)
#define CR1_CLEAR_Mask          ((uint16_t)0xFBF5) /* I2C registers Masks */
#define CCR_CCR_Set             ((uint16_t)0x0FFF) /* I2C CCR mask */
#define CCR_FS_Set              ((uint16_t)0x8000) /* I2C F/S mask */

#define MODE_READ       1       /* Work as Receiver */
#define MODE_WRITE      0       /* Work as Transmitter */

#define ENABLE 1
#define DISABLE 0
#define ERROR 0
#define SUCCESS 1


/* Enable DMA trensfer mode at given bus */
inline void I2C_DMA_Enable(I2C_TypeDef* I2Cx){
    I2Cx->CR2 |= I2C_CR2_DMAEN;
};

/* Disable DMA trensfer mode at given bus */
inline void I2C_DMA_Disable(I2C_TypeDef* I2Cx){
    I2Cx->CR2 &= (uint16_t)~I2C_CR2_DMAEN;
};

/* Check if interface is busy */
inline uint8_t I2C_Is_Busy(I2C_TypeDef* I2Cx)
{
    return (uint8_t)(I2Cx->SR2&I2C_SR2_BUSY);
}

/* Answer with NACK to next received byte and set STOP */
#define I2C_NACK_STOP(x) { \
        x->CR1 &= (uint16_t)~I2C_CR1_ACK; \
        x->CR1 |= I2C_CR1_STOP; \
    }
#define I2C_ENA_ACK(x)  x->CR1 |= I2C_CR1_ACK

/* Send (RE-)START condition */
#define I2C_RESTART(x)  x->CR1 |= I2C_CR1_START

/* Send STOP after current transmission finished */
#define I2C_STOP(x)     x->CR1 |= I2C_CR1_STOP

/* Disable Event Interrupt */
#define I2C_DIS_EVT(x)  x->CR2 &= (uint16_t)~I2C_CR2_ITEVTEN

/* Control Buffer Interrupt */
#define I2C_DIS_BUF(x)  x->CR2 &= (uint16_t)~I2C_CR2_ITBUFEN
#define I2C_ENA_BUF(x)  x->CR2 |= I2C_CR2_ITBUFEN
/*
 * TWI Event & Data Interrupt Handler
 */
void TwEventIrq( void *arg)
{
    NUTTWIBUS *bus = arg;
    NUTTWIICB *icb = bus->bus_icb;
    I2C_TypeDef* I2Cx = (I2C_TypeDef*)bus->bus_base;
    uint32_t twsr1 = I2Cx->SR1;
    uint32_t twsr2 = I2Cx->SR2;
//    int i;

#if 0
    i=16;
    do {
        i--;
        DBGP1(1);
        DBGP2(twsr1&(1<<i));
        DBGP1(0);
    } while(i);
#endif

    if( twsr1 & I2C_SR1_SB) {
        /* Send Slave Address and direction bit */
        // TODO: 10-Bit Addressing
        I2Cx->DR = (icb->tw_mm_sla | icb->tw_mm_dir);
    }

    if( twsr2 & I2C_SR2_MSL)
    {
        DBGP1(1);
        /* Interface is in master mode */

        if( twsr1 & I2C_SR1_ADDR) {
            /*
             * This Section is called only after
             * Slave Address has been sent
             */
            if( icb->tw_mm_dir == MODE_WRITE) {
                /*
                 * Master Transmitter 1st Byte
                 */
                if( icb->tw_mm_iadrlen) {
                    /* we have to send a register address to the slave */
                    /* little-endian machine! */
                    I2Cx->DR = *icb->tw_mm_iadr--;
                    icb->tw_mm_iadrlen--;
                }
                else if( icb->tw_mm_txlen) {
                    /* we have to send data */
                    I2Cx->DR = *icb->tw_mm_txbuf++;
                    icb->tw_mm_txlen--;
                }

                /* If there was only one byte to send */
                if( (icb->tw_mm_txlen==0) && (icb->tw_mm_iadrlen==0)) {
                    /* Disable TXE Interrupt */
                    I2C_DIS_BUF(I2Cx);
                }
            }
            else {
                /*
                 * Master Receiver if only one byte awaited
                 */
                /* Check if only one byte to read */
                if( icb->tw_mm_rxlen == 1) {
                    /* Last byte awaited: send NACK and STOP */
                    I2C_NACK_STOP(I2Cx);
                }
            }
            goto TwEvExit;
        }


        /*
         * Master in transmission 2nd to last byte
         */
        if( (twsr1 & (I2C_SR1_TXE|I2C_SR1_BTF)) == I2C_SR1_TXE) {
            if( icb->tw_mm_iadrlen) {
                /* send next register address byte */
                /* little-endian machine! */
                I2Cx->DR = *icb->tw_mm_iadr--;
                icb->tw_mm_iadrlen--;
            }
            else if( icb->tw_mm_txlen) {
                /* send next data byte */
                I2Cx->DR = *icb->tw_mm_txbuf++;
                icb->tw_mm_txlen--;
            }
            /* Check if end of tx */
            if( (icb->tw_mm_txlen==0) && (icb->tw_mm_iadrlen==0)) {
                /* Nothing left to send */
                I2C_DIS_BUF(I2Cx);
            }
            goto TwEvExit;
        }

        /*
         * Master transmitted last byte
         */
        if( (twsr1 & (I2C_SR1_TXE|I2C_SR1_BTF)) == (I2C_SR1_TXE|I2C_SR1_BTF)) {
            /* Check if Write->Read transition */
            if( icb->tw_mm_rxlen) {
                /* Switch to read direction */
                icb->tw_mm_dir = MODE_READ;
                /* Issue a RESTART */
                I2C_RESTART(I2Cx);
                /* Enable Buffer Interrupt again */
                I2C_ENA_BUF(I2Cx);
            }
            else {
                /* We are complete: Clear ACK, send STOP */
                I2C_NACK_STOP(I2Cx);
                /* Disable EV_IT */
                I2C_DIS_EVT(I2Cx);
            }
            goto TwEvExit;
        }

        /*
         * Master is receiving
         */
        if( twsr1 & I2C_SR1_RXNE) {
            /* Read data */
            *icb->tw_mm_rxbuf++ = (uint8_t)I2Cx->DR;
            icb->tw_mm_rxlen--;
            if( icb->tw_mm_rxlen == 1) {
                /* Only one byte left to receive: Clear ACK, set STOP */
                I2C_NACK_STOP(I2Cx);
            }
            if( icb->tw_mm_rxlen == 0) {
                /* Only one byte left to receive: Clear ACK, set STOP */
                I2C_DIS_BUF(I2Cx);
                goto TwEvExit;
            }
            goto TwEvExit;
        }
    }
    else {
        /* Slave mode */
        I2C_DIS_BUF(I2Cx);
        I2C_DIS_EVT(I2Cx);
        DBGP2(1);
    }

TwEvExit:
   DBGP2(0);
   DBGP1(0);
}


void TwErrorIrq( void *arg)
{
    NUTTWIBUS *bus = arg;
    NUTTWIICB *icb = bus->bus_icb;
    I2C_TypeDef* I2Cx = (I2C_TypeDef*)bus->bus_base;
    register uint32_t twsr1 = (I2Cx->SR1);

    /*
     *** ERROR HANDLING ***
     */
    if( twsr1 & I2C_SR1_AF) {
        /* Acknowledge failed */
        I2Cx->SR1 &= (uint16_t)~I2C_SR1_AF;
        icb->tw_mm_err = TWERR_SLA_NACK;
        I2C_STOP(I2Cx);
    }
    if( twsr1 & I2C_SR1_ARLO) {
        /* Arbitration lost */
        I2Cx->SR1 &= (uint16_t)~I2C_SR1_ARLO;
        icb->tw_mm_err = TWERR_ARBLOST;
    }
    if( twsr1 & I2C_SR1_BERR) {
        /* Misplaced Start/STOP condition detected */
        I2Cx->SR1 &= (uint16_t)~I2C_SR1_BERR;
        icb->tw_mm_err = TWERR_BUS;
    }
    if( twsr1 & I2C_SR1_TIMEOUT) {
        /* Timeout occured */
        I2Cx->SR1 &= (uint16_t)~I2C_SR1_TIMEOUT;
        icb->tw_mm_err = TWERR_TIMEOUT;
    }
    if( twsr1 & I2C_SR1_OVR) {
        /* Overrun/Underrun detected */
        I2Cx->SR1 &= (uint16_t)~I2C_SR1_OVR;
        icb->tw_mm_err = TWERR_OVRE;
    }
}


/*!
 * \brief Reset the I2C peripheral by issueing soft-reset.
 */
void NutTwiSoftReset( NUTTWIBUS *bus)
{
    I2C_TypeDef* I2Cx = (I2C_TypeDef*)bus->bus_base;
    I2Cx->CR1 |= I2C_CR1_SWRST;
    I2Cx->CR1 &= ~I2C_CR1_SWRST;
}

int TwWaitForFlag( volatile uint32_t *reg, uint32_t mask, uint16_t flag)
{
    int ret = 0;
    uint32_t sptout = 0xFFFF;
    while( (*(volatile uint16_t*)reg & mask) != flag) {
        if (sptout-- == 0) {
            ret = TWERR_SPCTOUT;
            break;
        }
    }
    return ret;
}

#if 0
int TwWaitBusFree( NUTTWIBUS *bus)
{
    int ret = -1;
    uint32_t tout = 100;
    NUTTWIICB *icb = bus->bus_icb;
    I2C_TypeDef* I2Cx = (I2C_TypeDef*)bus->bus_base;

    /* Wait till last transfer finished */
    tout = 100;
    while (I2C_Is_Busy(I2Cx) && tout) {
        TPRINTF("B");
        NutSleep(1);
        tout--;
    }
    if (tout) {
        /* Interface released in time */
        ret = 0;
    }
    else {
        TPRINTF("R");
        I2Cx->CR1 &= ~I2C_CR1_PE;
        I2Cx->CR1 |= I2C_CR1_SWRST;
        bus->bus_recover();
        I2Cx->CR1 &= ~I2C_CR1_SWRST;
        I2Cx->CR1 &= ~I2C_CR1_PE;
        NutSleep(1);
        if (I2C_Is_Busy(I2Cx)) {
            /* Bus still blocked */
            icb->tw_mm_error = TWERR_BUS;
            /* Release the interface. */
            NutEventPost( &bus->bus_mutex );
        }
        else {
            /* We made it! */
            ret = 0;
        }
    }
    return ret;
}
#endif

/*!
 * \brief Set START condition and wait for its appearance on the bus.
 */
void NutTwiStartRolling( NUTTWIBUS *bus, uint32_t tmo)
{
    int ret = 0;
    uint32_t tout = tmo;
    NUTTWIICB *icb = bus->bus_icb;
    I2C_TypeDef* I2Cx = (I2C_TypeDef*)bus->bus_base;

    /* Enable Interrupts */
    I2Cx->CR2 |= (I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN|I2C_CR2_ITERREN);
    /* Generate Start gets Irq System rolling */
    I2Cx->CR1 = (I2C_CR1_ACK|I2C_CR1_START|I2C_CR1_PE);
    /* Wait till the START has been sent */
    ret = TwWaitForFlag(&I2Cx->CR1, I2C_CR1_START, I2C_CR1_START);

    /* Continue only if START has been sent */
    if (ret == 0) {
        /* Go to background till we are through */
        while( I2C_Is_Busy(I2Cx)) {
//        while( icb->tw_mm_rxlen+icb->tw_mm_txlen) {
            NutSleep(1);
            if( tout-- == 0) {
                ret = TWERR_TIMEOUT;
                break;
            }
        }
        // TODO: Go back into Slave Mode
        I2Cx->CR1 |= I2C_CR1_ACK;
    }
    else
        icb->tw_mm_err |= ret;
    return;
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
int NutTwiMasterTranceive( NUTTWIBUS  *bus,
                           uint8_t    sla,
                           const void *txdata, uint16_t txlen,
                           void       *rxdata, uint16_t rxsiz,
                           uint32_t   tmo )
{
    int rc = -1;
    NUTTWIICB *icb = bus->bus_icb;

//    TPRINTF( "TMT ");

    /* This routine is marked reentrant, so lock the interface. */
    if( NutEventWait( &bus->bus_mutex, tmo ) ) {
        icb->tw_mm_error = TWERR_IF_LOCKED;
        TPRINTF("! mtx=LOCKED... OUT\n");
        return rc;
    }

#if 0
    /* Check if bus is blocked for any reason */
    if (TwWaitBusFree( bus))
    {
        return rc;
    }
#endif

    /* Clear errors. */
    icb->tw_mm_err = 0;
    /* fetch transfer parameters for current transaction */
    icb->tw_mm_sla = sla<<1;
    icb->tw_mm_iadr = NULL;
    icb->tw_mm_iadrlen = 0;
    icb->tw_mm_txbuf = (uint8_t*)txdata;
    icb->tw_mm_txlen = txlen;
    icb->tw_mm_rxbuf = rxdata;
    icb->tw_mm_rxlen = rxsiz;
    icb->tw_mm_err = 0;

    if (icb->tw_mm_rxlen)
        icb->tw_mm_dir = MODE_READ;
    else
        icb->tw_mm_dir = MODE_WRITE;

    /* Issue start and wait till transmission completed */
    NutTwiStartRolling( bus, tmo);

    /* Check for errors that may have been detected
     * by the interrupt routine.
     */
    if( icb->tw_mm_err ) {
        icb->tw_mm_error = icb->tw_mm_err;
        rc = -1;
        /* Soft-Reset this Bus */
        // NutTwiSoftReset( bus);
    } else {
        if( rxsiz)
            rc = (int)(rxsiz - icb->tw_mm_rxlen);
        else
            rc = (int)(txlen - icb->tw_mm_txlen);
    }

    /* Release the interface. */
    NutEventPost( &bus->bus_mutex );
    TPRINTF( "* Bye rc=%d\n", rc);

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
int NutTwiMasterRegRead( NUTTWIBUS  *bus,
                     uint8_t sla,
                     uint32_t iadr, uint8_t iadrlen,
                     void *rxdata, uint16_t rxsiz,
                     uint32_t tmo )
{
    int rc = -1;
    NUTTWIICB *icb = bus->bus_icb;

//    TPRINTF( "TMRR ");

    /* Quit if nothing to do */
    if( rxsiz == 0 ) {
        TPRINTF("! rxs=0... OUT\n");
        return rc;
    }

    /* This routine is marked reentrant, so lock the interface. */
    if( NutEventWait( &bus->bus_mutex, tmo ) ) {
        icb->tw_mm_error = TWERR_IF_LOCKED;
        TPRINTF("! LOCKED... OUT\n");
        return rc;
    }
#if 0
    /* Check if bus is blocked for any reason */
    if (TwWaitBusFree( bus))
    {
        return rc;
    }
#endif
    /* Clear errors. */
    icb->tw_mm_err = 0;
    /* fetch transfer parameters for current transaction */
    icb->tw_mm_sla = sla<<1;
    icb->tw_mm_iadrlen = iadrlen;
    if( iadrlen) {
        /* little-endian machine! */
        icb->tw_mm_iadr = ((uint8_t*)&iadr)+iadrlen-1;
    }
    icb->tw_mm_txbuf = NULL;
    icb->tw_mm_txlen = 0;
    icb->tw_mm_rxbuf = rxdata;
    icb->tw_mm_rxlen = rxsiz;
    icb->tw_mm_dir = MODE_WRITE;
    icb->tw_mm_err = 0;

    /* Issue start and wait till transmission completed */
    NutTwiStartRolling( bus, tmo);

    /* Check for errors that may have been detected
     * by the interrupt routine.
     */
    if( icb->tw_mm_err ) {
        icb->tw_mm_error = icb->tw_mm_err;
        rc = -1;
        TPRINTF( "! Error 0x%d\n", icb->tw_mm_err);
        /* Soft-Reset this Bus */
        // NutTwiSoftReset( bus);
    }
    else {
        rc = (int)(rxsiz - icb->tw_mm_rxlen);
    }

    /* Release the interface. */
    NutEventPost( &bus->bus_mutex );
//    TPRINTF( "* Bye rc=%d\n", rc);

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

int NutTwiMasterRegWrite( NUTTWIBUS  *bus,
                          uint8_t sla,
                          uint32_t iadr, uint8_t iadrlen,
                          const void *txdata, uint16_t txsiz,
                          uint32_t tmo )
{
    int rc = -1;
    NUTTWIICB *icb = bus->bus_icb;

//    TPRINTF( "TMRW ");

    /* Quit if nothing to do */
    if( txsiz==0) {
        TPRINTF("! txs=0... OUT\n");
        return rc;
    }

    /* This routine is marked reentrant, so lock the interface. */
    if( NutEventWait( &bus->bus_mutex, tmo ) ) {
        icb->tw_mm_error = TWERR_IF_LOCKED;
        TPRINTF("! mtx=LOCKED... OUT\n");
        return rc;
    }
#if 0
    /* Check if bus is blocked for any reason */
    if (TwWaitBusFree( bus))
    {
        return rc;
    }
#endif
    /* Clear errors. */
    icb->tw_mm_err = 0;
    /* fetch transfer parameters for current transaction */
    icb->tw_mm_sla = sla<<1;
    icb->tw_mm_iadrlen = iadrlen;
    if( iadrlen) {
        /* little-endian machine! */
        icb->tw_mm_iadr = ((uint8_t*)&iadr)+iadrlen-1;
    }
    icb->tw_mm_txbuf = (uint8_t*)txdata;
    icb->tw_mm_txlen = txsiz;
    icb->tw_mm_rxbuf = 0;
    icb->tw_mm_rxlen = 0;
    icb->tw_mm_dir = MODE_WRITE;
    icb->tw_mm_err = 0;

    /* Issue start and wait till transmission completed */
    NutTwiStartRolling( bus, tmo);

    /* Check for errors that may have been detected
     * by the interrupt routine.
     */
    if( icb->tw_mm_err ) {
        icb->tw_mm_error = icb->tw_mm_err;
        rc = -1;
        TPRINTF( "! Error 0x%d\n", icb->tw_mm_err);
        /* Soft-Reset this Bus */
        // NutTwiSoftReset( bus);
    }
    else {
        rc = (int)(txsiz - icb->tw_mm_txlen);
    }

    /* Release the interface. */
    NutEventPost( &bus->bus_mutex );
//    TPRINTF( "* Bye rc=%d\n", rc);

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
 * \note Slave mode is not implemented in the STM32 driver.
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
 * \note Slave mode is not implemented in the STM32 driver.
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
 * \note Slave mode is not implemented in the STM32 driver.
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
 * Answer: We really need this! It is bus dependand and is used by
 * EEPROM driver to determine the number of bytes transferred and
 * detection of EEPROM busy event (ACK-Polling).
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
        return (uint16_t) icb->tw_mm_rxlen;
    case 2:
        return (uint16_t) icb->tw_mm_txlen;
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
    int rc = 0;

    register uint16_t ccr;
    I2C_TypeDef* I2Cx = (I2C_TypeDef*)bus->bus_base;
    uint32_t apbclk = NutClockGet(HWCLK_APB1);
    uint16_t frqrange = (uint16_t)(apbclk/1000000);
    uint16_t cr1 = I2Cx->CR1;

    /* Disable I2C peripheral */
    I2Cx->CR1 &= ~I2C_CR1_PE;

    /* sanity check */
    if( speed > 400000 ) {
        speed = 400000;
        rc = -1;
    }

    /* Configure speed in fast mode */
    if( speed > 100000 ) {
        /* calculate CCR value */
        ccr = (uint16_t)(apbclk/(speed*25));
        if( ccr == 0 ) {
            /* keep minimum allowed value */
            ccr = 0x0001;
        }
        /* Set DUTY bit and set F/S bit for fast mode */
        ccr |= (I2C_CCR_DUTY|I2C_CCR_FS);
        /* Set Maximum Rise Time for fast mode */
        I2Cx->TRISE = (uint16_t)(((frqrange*300)/1000)+1);
    }
    else {
        /* Standard mode speed calculate */
        ccr = (uint16_t)(apbclk/(speed<<1));
        /* Test if CCR value is under 0x4 */
        if( ccr < 4 ) {
            /* Set minimum allowed value */
            ccr = 4;
        }
        else if ( ccr > I2C_CCR_CCR ) {
            ccr = I2C_CCR_CCR;
        }
        /* Set Maximum Rise Time for standard mode */
        I2Cx->TRISE = frqrange+1;
    }
    /* Write CCR register */
    I2Cx->CCR = ccr;

    /* Restore the CR1 register */
    I2Cx->CR1 = cr1;

    return rc;
}

/*!
 * \brief Request Current Speed of I2C Interface.
 *
 */
uint32_t NutTwiGetSpeed( NUTTWIBUS *bus)
{
    uint32_t speed = 0;
    uint32_t ccr = 0;
    uint32_t apbclk = NutClockGet(HWCLK_APB1);
    I2C_TypeDef* I2Cx = (I2C_TypeDef*)bus->bus_base;

    ccr=I2Cx->CCR;

    if(ccr & I2C_CCR_FS) {
        /* High speed */
        speed=(int)(apbclk/(25UL*(ccr&I2C_CCR_CCR)));
    }
    else {
        /* Low speed */
        speed=(int)(apbclk/(2UL*(ccr&I2C_CCR_CCR)));
    }
    return speed;
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

    switch (req)
    {
        case TWI_SETSPEED:
            NutTwiSetSpeed( bus, *(uint32_t*)conf);
            break;

        case TWI_GETSPEED:
            *(uint32_t*)conf = NutTwiGetSpeed( bus);
            break;

        case TWI_GETSTATUS:
            break;

        case TWI_SETSTATUS:
            break;

        case TWI_GETSLAVEADDRESS:
            // TODO: Slave handling
            icb->tw_mm_sla = *((uint16_t*)conf);
            break;

        case TWI_SETSLAVEADDRESS:
            // TODO: Slave handling
            *(uint32_t*)conf = (uint32_t)icb->tw_mm_sla;
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
 * \param sla Slave address, must be specified as a 7-bit address,
 *            always lower than 128.
 */
int NutRegisterTwiBus( NUTTWIBUS *bus, uint8_t sla )
{
    int rc = -1;

    uint32_t speed = 80000; /* Errata Doc 14574 Rev. 9 Chapter 2.11: Avoid 88kHz to 100kHz */
//    uint16_t tmpreg = 0;
    I2C_TypeDef* I2Cx = (I2C_TypeDef*)bus->bus_base;
    NUTTWIICB *icb = NULL;

    DBGP1_INIT();
    DBGP2_INIT();

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

    if( NutRegisterIrqHandler( bus->bus_sig_ev, TwEventIrq, bus ) ) {
        free( icb);
        return rc;
    }

    if( NutRegisterIrqHandler( bus->bus_sig_er, TwErrorIrq, bus ) ) {
        free( icb);
        return rc;
    }

    /* Initialize GPIO Hardware */
    if( bus->bus_initbus != NULL) {
        rc = bus->bus_initbus();
    }
    if( rc) {
        return rc;
    }

    /* Disable and reset this bus */
    I2Cx->CR1 &= ~I2C_CR1_PE;
    I2Cx->CR1 |= I2C_CR1_SWRST;
    I2Cx->CR1 &= ~I2C_CR1_SWRST;


#ifdef I2C_DEFAULT_SPEED
    if( bus->bus_base == I2C1_BASE)
        speed = I2CBUS1_DEFAULT_SPEED*1000UL;
#endif
#ifdef I2CBUS2_DEFAULT_SPEED
    if( bus->bus_base == I2C2_BASE)
        speed = I2CBUS2_DEFAULT_SPEED*1000UL;
#endif
    /* Set initial rate. */
    if( (rc = NutTwiSetSpeed( bus, speed))) {
        return rc;
    }

    /* Setup 7-Bit Addressing Mode not acknowledged */
    I2Cx->OAR1 = 0x4000; /* FIXME: OAR1 Bit 14 is reserved */

#if 0
    /* Setup CR1 */
    tmpreg = I2Cx->CR1;
    tmpreg &= ~(I2C_CR1_SMBUS|I2C_CR1_SMBTYPE|I2C_CR1_PEC);
    tmpreg |= I2C_CR1_ACK;
    // TODO: Set SMBTYPE and SMBUS bits
    I2Cx->CR1 = tmpreg;

    /* Setup Interrupts */
    tmpreg = I2Cx->CR2;
    tmpreg |= (I2C_CR2_ITBUFEN|I2C_CR2_ITEVTEN|I2C_CR2_ITERREN);
    I2Cx->CR2 = tmpreg;
#endif
    I2Cx->CR2 = 0x0000;
    I2Cx->CR2 &= I2C_CR2_FREQ;
    I2Cx->CR2 |= (NutClockGet(HWCLK_APB1)/1000000);
    I2Cx->CR1 = I2C_CR1_PE;

    // TODO: Slave Address Setup

    NutIrqSetPriority(bus->bus_sig_ev, 0);
    rc = NutIrqEnable(bus->bus_sig_ev);
    if( rc) {
        return rc;
    }
    NutIrqSetPriority(bus->bus_sig_er, 1);
    rc = NutIrqEnable(bus->bus_sig_er);
    if( rc) {
        return rc;
    }

    /* Initialize mutex semaphores. */
    NutEventPost(&bus->bus_mutex);

    return rc;
}

/*@}*/
