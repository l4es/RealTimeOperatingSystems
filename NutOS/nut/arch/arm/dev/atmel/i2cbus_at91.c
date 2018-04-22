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
 * \file arch/arm/dev/atmel/i2cbus_at91.c
 * \brief I2C bus driver for AT91 family.
 *
 * This driver is in an early stage and has been tested on Ethernut 5 only.
 *
 * It is intended that this driver replaces the current AT91 TWI driver,
 * which doesn't allow to have different types of busses in a single
 * application, for example TWI hardware and bit banging interfaces.
 * This new I2C driver layout allows to attach any I2C slave driver to
 * any I2C bus driver by calling NutRegisterI2cSlave().
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <dev/irqreg.h>
#include <sys/nutdebug.h>
#include <sys/timer.h>
#include <sys/event.h>

#include <stdlib.h>

#include <dev/i2cbus_at91.h>

/*!
 * \addtogroup xgI2cBusAt91
 */
/*@{*/

/*!
 * \brief Local data of the AT91 TWI bus driver.
 */
typedef struct _AT91_TWICB {
    /*! \brief Register base. */
    uint32_t icb_base;
    /*! \brief System interrupt handler. */
    IRQ_HANDLER *icb_sig;
    /*! \brief I2C message. */
    NUTI2C_MSG *icb_msg;
    /*! \brief Thread waiting for completion. */
    HANDLE icb_queue;
} AT91_TWICB;

#if 0
static void DumpTwiRegs(AT91_TWICB *icb)
{
    printf("\nMMR  %08lx\n", mem_rd32(icb->icb_base + TWI_MMR_OFF));
    printf("SMR  %08lx\n", mem_rd32(icb->icb_base + TWI_SMR_OFF));
    printf("IADR %08lx\n", mem_rd32(icb->icb_base + TWI_IADRR_OFF));
    printf("CWGR %08lx\n", mem_rd32(icb->icb_base + TWI_CWGR_OFF));
    printf("SR   %08lx\n", mem_rd32(icb->icb_base + TWI_SR_OFF));
    printf("IMR  %08lx\n", mem_rd32(icb->icb_base + TWI_IMR_OFF));
}
#endif

/*
 * AT91 TWI interrupt function.
 */
static void TwiBusIrqHandler(void *arg)
{
    AT91_TWICB *icb = (AT91_TWICB *) arg;
    NUTI2C_MSG *msg = icb->icb_msg;
    uint32_t sr = mem_rd32(icb->icb_base + TWI_SR_OFF);
    unsigned int ba = icb->icb_base;

    /* Process enabled interrupts only. */
    sr &= mem_rd32(ba + TWI_IMR_OFF);

    /*
     * Process transmit interrupt.
     */
    if (sr & TWI_TXRDY) {
        /* Check if more bytes to transmit. */
        if (msg->msg_widx < msg->msg_wlen) {
            /* Transmit the next byte. */
            mem_wr32(ba + TWI_THR_OFF, msg->msg_wdat[msg->msg_widx]);
            msg->msg_widx++;
        } else {
            /* All bytes sent, stop transfer. */
            mem_wr32(ba + TWI_IDR_OFF, TWI_TXRDY);
            mem_wr32(ba + TWI_CR_OFF, TWI_STOP);
        }
    }

    /*
     * Process receive interrupt.
     */
    else if (sr & TWI_RXRDY) {
        /* Check if more bytes to receive. */
        if (msg->msg_ridx < msg->msg_rsiz) {
            /* Store byte in receive buffer. */
            msg->msg_rdat[msg->msg_ridx++] = mem_rd32(ba + TWI_RHR_OFF);
            /* Check if the next byte will be last. */
            if (msg->msg_ridx + 1 == msg->msg_rsiz) {
                /* Set STOP condition. */
                mem_wr32(ba + TWI_CR_OFF, TWI_STOP);
            }
        }
    }

    /*
     * Process completion interrupt.
     */
    if (sr & TWI_TXCOMP) {
        /* Transfer complete, disable interrupts and wake up thread. */
        mem_wr32(ba + TWI_IDR_OFF, 0xFFFFFFFF);
        NutEventPostFromIrq(&icb->icb_queue);
    }
}

/*!
 * \brief I2C bus transfer (AT91 TWI implementation).
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_tran function pointer.
 */
static int TwiBusTran(NUTI2C_SLAVE *slave, NUTI2C_MSG *msg)
{
    NUTI2C_BUS *bus;
    AT91_TWICB *icb;
    uint32_t ba;

    bus = slave->slave_bus;
    icb = (AT91_TWICB *) bus->bus_icb;
    icb->icb_msg = msg;
    ba = icb->icb_base;

    /*
     * Process I2C read operation.
     */
    if (msg->msg_rsiz) {
        uint32_t iadr = 0;
        int i;

        /* Check internal address size limit. */
        if (msg->msg_wlen > 3) {
            return -1;
        }
        /* Set internal address register. */
        for (i = 0; i < msg->msg_wlen; i++) {
            iadr <<= 8;
            iadr |= msg->msg_wdat[i];
        }
        mem_wr32(ba + TWI_IADRR_OFF, iadr);
        /* Set master mode register. */
        mem_wr32(ba + TWI_MMR_OFF,
            ((uint32_t) slave->slave_address << TWI_DADR_LSB) |
            TWI_MREAD |
            ((uint32_t) msg->msg_wlen << TWI_IADRSZ_LSB));
        /* Start transfer. */
        if (msg->msg_rsiz == 1) {
            /* Set STOP condition if this is the last byte. */
            mem_wr32(ba + TWI_CR_OFF, TWI_START | TWI_STOP);
        } else {
            mem_wr32(ba + TWI_CR_OFF, TWI_START);
        }
        mem_wr32(ba + TWI_IER_OFF, TWI_ARBLST | TWI_NACK | TWI_RXRDY | TWI_TXCOMP);
    }

    /*
     * Process I2C write operation.
     */
    else if (msg->msg_wlen) {
        mem_wr32(ba + TWI_MMR_OFF, (uint32_t) slave->slave_address << TWI_DADR_LSB);
        mem_wr32(ba + TWI_THR_OFF, msg->msg_wdat[0]);
        msg->msg_widx++;
        mem_wr32(ba + TWI_IER_OFF, TWI_ARBLST | TWI_NACK | TWI_TXRDY | TWI_TXCOMP);
    }
    /* Wait for transfer complete. */
    if (NutEventWait(&icb->icb_queue, slave->slave_timeout)) {
        mem_wr32(ba + TWI_IDR_OFF, 0xFFFFFFFF);
        return -1;
    }
    return msg->msg_ridx;
}

/*!
 * \brief Configure the I2C bus controller (AT91 TWI implementation).
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_conf function pointer. Most implementations will
 * also call this function during initialization to set the
 * default configuration.
 *
 * Right now only the bus clock rate is configurable.
 */
static int TwiBusConf(NUTI2C_BUS *bus)
{
    AT91_TWICB *icb;
    uint32_t mck;
    uint32_t cdiv;
    uint32_t ckdiv;
    long rate;

    /* Check parameters. */
    NUTASSERT(bus != NULL);
    NUTASSERT(bus->bus_icb != NULL);
    icb = (AT91_TWICB *) bus->bus_icb;

    /* Get requested rate or use the default. */
    rate = bus->bus_rate;
    if (rate == 0) {
        rate = 100000L;
    }
    /* rate = MCK / (((CLDIV + CHDIV) * 2^CKDIV) + 4) */
    mck = NutClockGet(NUT_HWCLK_PERIPHERAL);
    cdiv = (mck >> 1) / rate - 4;
    for (ckdiv = 0; cdiv > 255; ckdiv++) {
        cdiv >>= 1;
    }
    if (ckdiv > 7) {
        /* Requested rate is not available. */
        return -1;
    }
    /* Set the actual rate. */
    bus->bus_rate = mck / (2 * cdiv * (1 << ckdiv) + 4);
    /* Set calculated clock divider. */
    mem_wr32(icb->icb_base + TWI_CWGR_OFF,
        (ckdiv << TWI_CKDIV_LSB) | (cdiv << TWI_CHDIV_LSB) | (cdiv << TWI_CLDIV_LSB));
    /* Enable master mode. */
    mem_wr32(icb->icb_base + TWI_CR_OFF, TWI_MSEN | TWI_SVDIS);

    return 0;
}

/*!
 * \brief Initialize the I2C bus controller (AT91 implementation).
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_init function pointer when the first slave is
 * attached to this bus. Note, that NUTI2C_BUS::bus_rate must be zero
 * initially. Otherwise no call to this function will take place.
 *
 * This function must do all required initializations so that the
 * driver will be ready to process transfers via NUTI2C_BUS::bus_tran.
 *
 * On many platforms it is required to configure the pins in the
 * board initialization function.
 *
 * This function must return 0 on success or -1 otherwise.
 */
static int TwiBusInit(NUTI2C_BUS *bus)
{
    AT91_TWICB *icb;

    icb = (AT91_TWICB *) bus->bus_icb;

    /* Disable all interrupts. */
    mem_wr32(icb->icb_base + TWI_IDR_OFF, 0xFFFFFFFF);
    /* Reset bus. */
    mem_wr32(icb->icb_base + TWI_CR_OFF, TWI_SWRST);
    /* Try to configure the bus and register the IRQ Handler */
    if (TwiBusConf(bus) || NutRegisterIrqHandler(icb->icb_sig, TwiBusIrqHandler, icb)) {
        return -1;
    }
    /* Enable interrupts. */
    NutIrqEnable(icb->icb_sig);

    return 0;
}

/*!
 * \brief Probe the I2C bus for a specified slave address (AT91 implementation).
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_probe function pointer. This may happen even if no
 * slave device had been attached to the bus and thus without any
 * previous call to NUTI2C_BUS::bus_init. However, in that case
 * NUTI2C_BUS::bus_configure will have been called.
 */
static int TwiBusProbe(NUTI2C_BUS *bus, int sla)
{
    int rc = -1;
    uint_fast8_t wt = 100;
    AT91_TWICB *icb;
    uint32_t sr;

    icb = (AT91_TWICB *) bus->bus_icb;

    /* Try to read one byte. */
    mem_wr32(icb->icb_base + TWI_MMR_OFF, (uint32_t) sla << TWI_DADR_LSB | TWI_MREAD);
    mem_wr32(icb->icb_base + TWI_CR_OFF, TWI_START | TWI_STOP);

    wt = bus->bus_timeout;
    do {
        sr = mem_rd32(icb->icb_base + TWI_SR_OFF) & (TWI_NACK | TWI_ARBLST | TWI_TXCOMP);
        if (sr) {
            if (sr == TWI_TXCOMP) {
                mem_rd32(icb->icb_base + TWI_RHR_OFF);
                rc = 0;
            }
            break;
        }
        NutSleep(1);
    } while (wt--);

    return rc;
}

static AT91_TWICB twi0cb = {
    TWI_BASE,   /* icb_base */
    &sig_TWI,   /* icb_sig */
    NULL,       /* icb_msg */
    NULL        /* icb_queue */
};

/*!
 * \brief I2C bus driver for AT91 TWI hardware.
 *
 * This is an interrupt driven driver, which supports master mode only.
 */
NUTI2C_BUS i2cBus0At91 = {
    &twi0cb,    /* bus_icb */
    TwiBusInit, /* bus_init */
    TwiBusConf, /* bus_configure */
    TwiBusProbe,/* bus_probe */
    TwiBusTran, /* bus_transceive */
    100,        /* bus_timeout */
    0,          /* bus_rate */
    0,          /* bus_flags */
    NULL        /* bus_mutex */
};

/* Second interface currently disabled because of missing IRQ handler. */
#if 0 && defined(TWI1_BASE)

static AT91_TWICB twi1cb = {
    TWI1_BASE,  /* icb_base */
    &sig_TWI1,  /* icb_sig */
    NULL,       /* icb_msg */
    NULL        /* icb_queue */
};

NUTI2C_BUS i2cBus1At91 = {
    &twi1cb,    /* bus_icb */
    TwiBusInit, /* bus_init */
    TwiBusConf, /* bus_configure */
    TwiBusProbe,/* bus_probe */
    TwiBusTran, /* bus_transceive */
    100,        /* bus_timeout */
    0,          /* bus_rate */
    0,          /* bus_flags */
    NULL        /* bus_mutex */
};

#endif

/*@}*/
