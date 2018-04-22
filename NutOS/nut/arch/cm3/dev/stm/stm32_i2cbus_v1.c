/*
 * Copyright (C) 2013, 2017 Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de
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
 * \file arch/cm3/dev/stm/stm32_i2cbus_v1.c
 * \brief I2C bus driver for I2C hardware in STM32F1/2/4 and STM32L1.
 *
 * It doesn't consider Slave operation yet
 *
 * It is intended that this driver replaces the current STM TWI driver,
 * which doesn't allow to have different types of busses in a single
 * application, for example TWI hardware and bit banging interfaces.
 * This new I2C driver layout allows to attach any I2C slave driver to
 * any I2C bus driver by calling NutRegisterI2cSlave().
 *
 * \verbatim
 * $Id: stm32_i2cbus_v1.c 6675 2017-09-20 15:10:18Z u_bonnes $
 * \endverbatim
 */

/*!
 * \brief I2C bus driver for STM32F1/2/4 and L1 hardware.
 *
 * This is an interrupt driver, which supports master mode only.
 * Most error handling
 */

#include <dev/irqreg.h>
#include <sys/nutdebug.h>
#include <sys/timer.h>
#include <sys/event.h>

#include <stdlib.h>

#include <cfg/arch.h>
#include <arch/cm3.h>
#include <dev/gpio.h>
#include <arch/cm3/stm/stm32_i2c_pinmux.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_clk.h>
#include <arch/cm3/stm/stm32_irqreg.h>
#include <dev/i2cbus.h>
#include <cfg/twi.h>
#include <arch/cm3/stm/stm32_gpio.h>

/*!
 * \addtogroup xgI2cBusSTM32
 */
/*@{*/

/*!
 * \brief Constant local data of the STM32 I2C hardware.
 */
typedef struct _STM32_I2C_HW{
    /*! \brief Register base. */
    const uptr_t icb_base;
    /*! \brief SDA_PIN. */
    const nutgpio_t sda;
    /*! \brief SDA Pinmux. */
    const uint8_t sda_af;
    /*! \brief SCL_PIN. */
    const nutgpio_t scl;
    /*! \brief SCL Pinmux. */
    const uint8_t scl_af;
    /*! \brief SMBA_PIN. */
    const nutgpio_t smba;
    /*! \brief SMBA Pinmux. */
    const uint8_t smba_af;
}STM32_I2C_HW;

/*!
 * \brief Local data of the STM32 I2C bus driver.
 */
typedef struct _STM32_I2CCB {
    const STM32_I2C_HW *hw;
    /*! \brief System event handler. */
    IRQ_HANDLER *icb_sig_ev;
    /*! \brief System error handler. */
    IRQ_HANDLER *icb_sig_er;
    /*! \brief I2C message. */
    NUTI2C_MSG *icb_msg;
    /*! \brief Thread waiting for completion. */
    HANDLE icb_queue;
    /*! \brief Current Slave/ */
    NUTI2C_SLAVE *slave;
    uint32_t errors;
} STM32_I2CCB;

/*
 * STM32V1 I2C Event interrupt function.
 */
static void I2cEventBusIrqHandler(void *arg)
{
    STM32_I2CCB *icb = (STM32_I2CCB *) arg;
    NUTI2C_MSG *msg = icb->icb_msg;
    I2C_TypeDef *I2Cx = (I2C_TypeDef *) icb->hw->icb_base;

    if(I2Cx->SR1 & I2C_SR1_SB) { /* Receiver EV5 */
        uint8_t address = icb->slave->slave_address << 1;
        int do_stop = 0;
        if (msg->msg_widx >= msg->msg_wlen) {
            if (msg->msg_ridx < msg->msg_rsiz) {
                address |= 1; /* Read transaction.*/
            } else {
                do_stop = 1;  /* Single adress transfer.*/
            }
        }
        I2Cx->DR = address;
        if (do_stop) {
            I2Cx->CR1 |= I2C_CR1_STOP;
        }
    }
    if(I2Cx->SR1 & I2C_SR1_ADDR) {
        if (!(I2Cx->SR2 & I2C_SR2_TRA) && (msg->msg_rsiz > 1)) {
            I2Cx->CR1 |= I2C_CR1_ACK;
        }
        (void) I2Cx->SR1;
        (void) I2Cx->SR2; /* Clear SR1_ADDR. */
        if ((msg->msg_widx >=  msg->msg_wlen) &&
            (msg->msg_ridx >=  msg->msg_rsiz)) {
            /* Only address cycle requested! Done!*/
            I2Cx->CR1 &= ~I2C_CR1_STOP;
            NutEventPostFromIrq(&icb->icb_queue);
        } else {
            I2Cx->CR2 |= I2C_CR2_ITBUFEN;
            if (!(I2Cx->SR2 & I2C_SR2_TRA)) {
                if (msg->msg_rsiz == 1) {
                    I2Cx->CR1 |= I2C_CR1_STOP;
                }
                if (msg->msg_rsiz > 1) {
                    I2Cx->CR1 |= I2C_CR1_ACK;
                }
            }
        }
    }
    if (I2Cx->SR1 & I2C_SR1_TXE) {
        if (msg->msg_widx < msg->msg_wlen) {
            I2Cx->DR = msg->msg_wdat[msg->msg_widx++];
        } else {
            /* Last byte already written to DR. Now last byte is in shift
             * register. Disable TXE Interrupts and set stop or repeated start
             * for this last transfer. */
            I2Cx->CR2 &= ~I2C_CR2_ITBUFEN;
            if (msg->msg_rsiz) {
                /* Repeated start for reading. */
                I2Cx->CR1 |= I2C_CR1_START;
            } else {
                /* Wait for BTF to indicate end.*/
                I2Cx->CR1 |= I2C_CR1_STOP;
            }
        }
    }
    if (I2Cx->SR1 & I2C_SR1_RXNE) { /* Receiver EV7*/
        msg->msg_rdat[msg->msg_ridx++]= I2Cx->DR;
        if (msg->msg_ridx >= msg->msg_rsiz - 1) { /* Ev 7.1 */
            I2Cx->CR1 &= ~I2C_CR1_ACK;
            if (msg->msg_rsiz  > 1) {
                I2Cx->CR1 |= I2C_CR1_STOP;
            }
        }
        if (msg->msg_ridx >= msg->msg_rsiz) {
            I2Cx->CR1 &= ~I2C_CR1_STOP;
            I2Cx->CR2 &= ~I2C_CR2_ITEVTEN;
            NutEventPostFromIrq(&icb->icb_queue);
        }
    }
    if (I2Cx->SR1 & I2C_SR1_BTF) {
        (void) I2Cx->DR;
        if (msg->msg_rsiz == 0) {
            I2Cx->CR1 &= ~I2C_CR1_STOP;
            I2Cx->CR2 &= ~I2C_CR2_ITEVTEN;
            NutEventPostFromIrq(&icb->icb_queue);
        }
    }
}

/*
 * STM32V1 I2C Error interrupt function.
 */
static void I2cErrorBusIrqHandler(void *arg)
{
    STM32_I2CCB *icb = (STM32_I2CCB *) arg;
    I2C_TypeDef *I2Cx = (I2C_TypeDef *) icb->hw->icb_base;
    uint32_t errors;

    errors = I2Cx->SR1;
    if (errors & I2C_SR1_ARLO) {
        NUTI2C_MSG *msg = icb->icb_msg;
        if (msg->msg_ridx) {
            msg->msg_rdat[msg->msg_ridx++]= I2Cx->DR;
            I2Cx->CR1 &= ~I2C_CR1_STOP;
            I2Cx->CR2 &= ~I2C_CR2_ITEVTEN;
            NutEventPostFromIrq(&icb->icb_queue);
            errors &= ~I2C_SR1_ARLO;
        }
    }
    I2Cx->SR1 = 0;
    NutEventPostFromIrq(&icb->icb_queue);

}

/*!
 * \brief I2C bus transfer (STM I2C implementation).
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_tran function pointer.
 */
static int I2cBusTran(NUTI2C_SLAVE *slave, NUTI2C_MSG *msg)
{
    NUTI2C_BUS *bus;
    STM32_I2CCB *icb;
    I2C_TypeDef *I2Cx;
    int rc;

    bus = slave->slave_bus;
    NUTASSERT(bus != NULL);
    NUTASSERT(bus->bus_icb != NULL);
    icb = (STM32_I2CCB *) bus->bus_icb;
    icb->icb_msg = msg;
    I2Cx = (I2C_TypeDef *) icb->hw->icb_base;
    msg->msg_widx = 0;
    msg->msg_ridx = 0;
    icb->slave = slave;
    icb->errors = 0;
    /* Enable Interrupts */
    I2Cx->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    I2Cx->CR1 &= ~I2C_CR1_STOP;
    I2Cx->CR1 |= I2C_CR1_START;
    rc = NutEventWait(&icb->icb_queue, slave->slave_timeout);
    I2Cx->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    if (!rc && (I2Cx->SR2 & I2C_SR2_BUSY)) {
        /* Wait some time until busy is released!*/
        int i;
        for (i = 0; (I2Cx->SR2 & I2C_SR2_BUSY) && (i < slave->slave_timeout);
             i++) {
            /* There is no interrupt for I2C peripheral getting idle.
             * Use Nutsleep*/
            NutSleep(1);
        }
        if (i >= slave->slave_timeout) {
            rc = -1;
        }
    }
    if(icb->errors)
    {
        I2Cx->CR1 |= I2C_CR1_STOP;
        msg->msg_ridx = -1;
    }
    if(rc)
        msg->msg_ridx = -1;
    return msg->msg_ridx;
}

static int checkpin_and_config(STM32_I2CCB *icb)
{
    const STM32_I2C_HW *hw = icb->hw;
    Stm32GpioConfigSet(
        hw->sda, GPIO_CFG_OUTPUT| GPIO_CFG_PERIPHAL|
        GPIO_CFG_MULTIDRIVE| GPIO_CFG_PULLUP | GPIO_CFG_SPEED_FAST,
        hw->sda_af);
    Stm32GpioConfigSet(
        hw->scl, GPIO_CFG_OUTPUT| GPIO_CFG_PERIPHAL|
        GPIO_CFG_MULTIDRIVE| GPIO_CFG_PULLUP | GPIO_CFG_SPEED_FAST,
        hw->scl_af);
    Stm32GpioConfigSet(
        hw->smba, GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL| GPIO_CFG_MULTIDRIVE|
        GPIO_CFG_PULLUP | GPIO_CFG_SPEED_FAST,
        hw->smba_af);
    if (hw->icb_base == I2C1_BASE)
    {
#if defined (MCU_STM32F1)
        CM3BBSETVAL(AFIO_BASE, AFIO_TypeDef, MAPR, _BI32(AFIO_MAPR_I2C1_REMAP),
            I2C1_REMAP_I2C);
#endif
        CM3BBSET(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_I2C1RST));
        CM3BBSET(RCC_BASE, RCC_TypeDef, APB1ENR,  _BI32(RCC_APB1ENR_I2C1EN));
        CM3BBCLR(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_I2C1RST));
    }
#if defined(I2C2_BASE)
    else if (hw->icb_base == I2C2_BASE)
    {
        CM3BBSET(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_I2C2RST));
        CM3BBSET(RCC_BASE, RCC_TypeDef, APB1ENR,  _BI32(RCC_APB1ENR_I2C2EN));
        CM3BBCLR(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_I2C2RST));
    }
#endif
#if defined(I2C3_BASE)
    else if (hw->icb_base == I2C3_BASE)
    {
        CM3BBSET(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_I2C3RST));
        CM3BBSET(RCC_BASE, RCC_TypeDef, APB1ENR,  _BI32(RCC_APB1ENR_I2C3EN));
        CM3BBCLR(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_I2C3RST));
    }
#endif
    else
        return -1;
    return 0;
}

/*!
 * \brief Configure the I2C bus controller (STM32V1 I2C implementation).
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_conf function pointer. Most implementations will
 * also call this function during initialization to set the
 * default configuration.
 *
 */
static int I2cBusConf(NUTI2C_BUS *bus)
{
    STM32_I2CCB *icb;
    uint32_t speed;
    I2C_TypeDef *I2Cx;
    uint16_t ccr;
    uint32_t apbclk = NutClockGet(HWCLK_APB1);
    uint16_t frqrange = (uint16_t)(apbclk/1000000);
    uint16_t cr1;

    /* Check parameters. */
    NUTASSERT(bus != NULL);
    NUTASSERT(bus->bus_icb != NULL);
    icb = (STM32_I2CCB *) bus->bus_icb;
    I2Cx = (I2C_TypeDef*) icb->hw->icb_base;
    cr1  = I2Cx->CR1;

    /* Get requested speed or use the default. */
    speed = bus->bus_rate;
    if (speed == 0)
    {
        speed = 100000L;
    }
    if (speed > 400000)
    {
        /* Speed out of range */
        speed = 400000;
    }
    /* Configure speed in fast mode */
    if( speed > 100000 )
    {
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
    else
    {
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

    return 0;
}

/*!
 * \brief Initialize the I2C bus controller (STM32 implementation).
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_init function pointer when the first slave is
 * attached to this bus. Note, that NUTI2C_BUS::bus_rate must be zero
 * initially. Otherwise no call to this function will take place.
 *
 * This function must do all required initializations so that the
 * driver will be ready to process transfers via NUTI2C_BUS::bus_tran.
 *
 * This function must return 0 on success or -1 otherwise.
 */
static int I2cBusInit(NUTI2C_BUS *bus)
{
    I2C_TypeDef *I2Cx;
    STM32_I2CCB *icb;
    uint32_t apbclk;

    icb = (STM32_I2CCB *) bus->bus_icb;

    if (checkpin_and_config(icb))
        return -1;
    /* Try to configure the bus*/
    if (I2cBusConf(bus)) {
        return -1;
    }
    apbclk = NutClockGet(HWCLK_APB1);
    if (apbclk < 2000000) {
        /* PCLK1 clock to slow for valid setup/hold timing */
        return -1;
    }
    I2Cx = (I2C_TypeDef*) icb->hw->icb_base;
    I2Cx->CR2 &= ~I2C_CR2_FREQ;
    I2Cx->CR2 |= apbclk/1000000;
    I2Cx->CR1 |= I2C_CR1_PE;
    if (NutRegisterIrqHandler(icb->icb_sig_ev, I2cEventBusIrqHandler, icb))
        return -1;
    if (NutRegisterIrqHandler(icb->icb_sig_er, I2cErrorBusIrqHandler, icb))
        return -1;
    NutIrqEnable(icb->icb_sig_ev);
    NutIrqEnable(icb->icb_sig_er);
    return 0;
}

/*!
 * \brief Probe the I2C bus for a specified slave address
 *        (STM32V1 implementation).
 *
 * This function is called by the platform independent code via the
 * NUTI2C_BUS::bus_probe function pointer. This may happen even if no
 * slave device had been attached to the bus and thus without any
 * previous call to NUTI2C_BUS::bus_init. However, in that case
 * NUTI2C_BUS::bus_configure will have been called.
 *
 *
 */
static int I2cBusProbe(NUTI2C_BUS *bus, int sla)
{
    STM32_I2CCB *icb;
    I2C_TypeDef *I2Cx;
    uint32_t tmo;
    int res = -1;

    icb = (STM32_I2CCB *) bus->bus_icb;
    I2Cx = (I2C_TypeDef*) icb->hw->icb_base;
    if ((I2Cx->CR1 & I2C_CR1_PE)== 0)
    {
        int res;
        res = I2cBusInit(bus);
        if (res)
            return res;
    }
    I2Cx->CR1 &= ~I2C_CR1_STOP;
    I2Cx->CR1 |= I2C_CR1_START;
    /* Wait until START has been sent*/
    for(tmo = NutGetMillis(); NutGetMillis() - tmo < bus->bus_timeout;)
    {
        res = !(I2Cx->SR1 & I2C_SR1_SB);
        if (res == 0)
            break;
    }
    if (res)
        return -1;
    I2Cx->DR = sla<<1;
    /* Sequence Read SR1, write DR clears SB*/
    /* Wait until Adress has been sent*/
    for(; NutGetMillis() - tmo < bus->bus_timeout;)
    {
        res = I2Cx->SR1;
        if (res & I2C_SR1_ADDR)
        {
            (void)I2Cx->SR2;
            break;
        }
        if (res & I2C_SR1_AF)
        {
            I2Cx->SR1 &= ~I2C_SR1_AF;
            break;
        }
    }
    I2Cx->CR1 &= (uint16_t)~I2C_CR1_ACK;
    I2Cx->CR1 |= I2C_CR1_STOP;
    return (res & I2C_SR1_ADDR)?0:-1;
}
static const STM32_I2C_HW i2c1_hw = {
    .icb_base = I2C1_BASE,
#if defined(MCU_STM32F1)
    .smba     = PB05,
# if I2C1_REMAP_I2C == 1
    .sda      = PB09,
    .scl      = PB08,
# else
    .sda      = PB07,
    .scl      = PB06,
# endif
#else
    .sda      = I2C1_SDA,
    .scl      = I2C1_SCL,
    .smba     = I2C1_SMBA,
#endif
    .sda_af   = I2C1_SDA_AF,
    .scl_af   = I2C1_SCL_AF,
    .smba_af  = I2C1_SMBA_AF,
};

static STM32_I2CCB i2c1cb = {
    &i2c1_hw,               /* Constant hardware data*/
    &sig_TWI1_EV,           /* Event signal   */
    &sig_TWI1_ER,           /* Error signal   */
    NULL,                   /* icb_msg        */
    NULL                    /* icb_queue      */
};

NUTI2C_BUS i2cBus1Stm32 = {
    &i2c1cb,    /* bus_icb */
    I2cBusInit, /* bus_init */
    I2cBusConf, /* bus_configure */
    I2cBusProbe,/* bus_probe */
    I2cBusTran, /* bus_transceive */
    100,        /* bus_timeout */
    0,          /* bus_rate */
    0,          /* bus_flags */
    NULL        /* bus_mutex */
};

#if defined(I2C2_BASE)
static const STM32_I2C_HW i2c2_hw = {
    .icb_base = I2C2_BASE,
#if defined(MCU_STM32F1)
    .sda      = PB11,
    .scl      = PB10,
    .smba     = PB12,
#else
    .sda      = I2C2_SDA,
    .scl      = I2C2_SCL,
    .smba     = I2C2_SMBA,
#endif
    .sda_af   = I2C2_SDA_AF,
    .scl_af   = I2C2_SCL_AF,
    .smba_af  = I2C2_SMBA_AF,
};

static STM32_I2CCB i2c2cb = {
    &i2c2_hw,               /* Constant hardware data*/
    &sig_TWI2_EV,           /* Event signal   */
    &sig_TWI2_ER,           /* Error signal   */
    NULL,                   /* icb_msg        */
    NULL                    /* icb_queue      */
};

NUTI2C_BUS i2cBus2Stm32 = {
    &i2c2cb,    /* bus_icb */
    I2cBusInit, /* bus_init */
    I2cBusConf, /* bus_configure */
    I2cBusProbe,/* bus_probe */
    I2cBusTran, /* bus_transceive */
    100,        /* bus_timeout */
    0,          /* bus_rate */
    0,          /* bus_flags */
    NULL        /* bus_mutex */
};
#endif

#if defined(I2C3_BASE)
static const STM32_I2C_HW i2c3_hw = {
    .icb_base = I2C3_BASE,
    .sda      = I2C3_SDA,
    .sda_af   = I2C3_SDA_AF,
    .scl      = I2C3_SCL,
    .scl_af   = I2C3_SCL_AF,
    .smba     = I2C3_SMBA,
    .smba_af  = I2C3_SMBA_AF,
 };

static STM32_I2CCB i2c3cb = {
    &i2c3_hw,               /* Constant hardware data*/
    &sig_TWI3_EV,           /* Event signal   */
    &sig_TWI3_ER,           /* Error signal   */
    NULL,                   /* icb_msg        */
    NULL                    /* icb_queue      */
};

NUTI2C_BUS i2cBus3Stm32 = {
    &i2c3cb,    /* bus_icb */
    I2cBusInit, /* bus_init */
    I2cBusConf, /* bus_configure */
    I2cBusProbe,/* bus_probe */
    I2cBusTran, /* bus_transceive */
    100,        /* bus_timeout */
    0,          /* bus_rate */
    0,          /* bus_flags */
    NULL        /* bus_mutex */
};
#endif
