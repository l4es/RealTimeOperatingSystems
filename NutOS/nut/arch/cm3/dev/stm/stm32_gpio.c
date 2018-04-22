/*
 * Copyright 2015 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/dev/stm32_gpio.c
 * \brief Common GPIO routines
 */

#include <stddef.h>
#include <stdint.h>

#include <dev/gpio.h>
#include <arch/cm3/stm/stm32_gpio.h>

/*!
 * \brief Lookup-table to get GPIO structure from GPIO number.
 *
 * \index  STM32_NR_GPIO one-base GPIO number.
 * \entry  GPIO structure
 */

GPIO_TypeDef *const stm32_port_nr2gpio[STM32_NR_GPIO] = {
    NULL,
    GPIOA,
    GPIOB,
    GPIOC,
#if defined(GPIOD_BASE)
    GPIOD,
#elif STM32_NR_GPIO >4
    NULL,
#endif
#if defined(GPIOE_BASE)
    GPIOE,
#elif STM32_NR_GPIO >5
    NULL,
#endif
#if defined(GPIOF_BASE)
    GPIOF,
#elif STM32_NR_GPIO >6
    NULL,
#endif
#if defined(GPIOG_BASE)
    GPIOG,
#elif STM32_NR_GPIO >7
    NULL,
#endif
#if defined(GPIOH_BASE)
    GPIOH,
#elif STM32_NR_GPIO >8
    NULL,
#endif
#if defined(GPIOI_BASE)
    GPIOI,
#elif STM32_NR_GPIO >9
    NULL,
#endif
#if defined(GPIOJ_BASE)
    GPIOJ,
#elif STM32_NR_GPIO >10
    NULL,
#endif
#if defined(GPIOK_BASE)
    GPIOK,
#elif STM32_NR_GPIO >11
    NULL,
#endif
};
