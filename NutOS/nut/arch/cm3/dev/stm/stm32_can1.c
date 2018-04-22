/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2012, 2015 Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de).
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

/*
 * \verbatim
 * $Id: stm32_can1.c 6219 2015-10-07 08:42:51Z u_bonnes $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/can_dev.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/canbus.h>

#include <arch/cm3.h>
#include <arch/cm3/stm/stm32_can_pinmux.h>
#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_gpio.h>

/*!
 * \brief Processor specific Hardware Initiliaization
 *
 */
int Stm32CanHw1Init(void)
{
#if defined (CAN2_ACCEPTANCE_FILTERS)
    uint32_t fmr;
#endif
    /* Reset CAN Bus 1 IP */
    CM3BBSET(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN1RST));
    /* Enable CAN Bus 1 peripheral clock. */
    CM3BBSET(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_CAN1EN));

    /* Release Reset on CAN Bus 1 IP */
    CM3BBCLR(RCC_BASE, RCC_TypeDef, APB1RSTR, _BI32(RCC_APB1RSTR_CAN1RST));

#if defined (CAN2_ACCEPTANCE_FILTERS)
    /* Set the CAN1/CAN2 Filter split */
    fmr = CAN1->FMR;
    fmr &= ~0x3f00;
    fmr |= CAN2_ACCEPTANCE_FILTERS<<8;
    CAN1->FMR = fmr;
#endif

    /* Setup Related GPIOs. */
    Stm32GpioConfigSet(CAN1_RX, GPIO_CFG_PULLUP | GPIO_CFG_PERIPHAL,
                       CAN1_RX_AF);
    Stm32GpioConfigSet(CAN1_TX,
                       GPIO_CFG_PULLUP | GPIO_CFG_PERIPHAL | GPIO_CFG_OUTPUT,
                       CAN1_TX_AF);

#if defined (MCU_STM32F1)
    AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
    AFIO->MAPR |=  CAN1_REMAP * AFIO_MAPR_CAN_REMAP;
#endif

    return 0;
}

NUTCANBUS Stm32CanBus1 = {
    CAN1_BASE,
    CM3BB_BASE(CAN1_BASE),
    &sig_CAN1_RX0,
    &sig_CAN1_TX,
    &sig_CAN1_SCE,
    0,
    Stm32CanHw1Init,
};

NUTCANBUS Stm32CanBus1C = {
    CAN1_BASE,
    CM3BB_BASE(CAN1_BASE),
    &sig_CAN1_RX1,
    0,
    0,
    0,
    0,
};
