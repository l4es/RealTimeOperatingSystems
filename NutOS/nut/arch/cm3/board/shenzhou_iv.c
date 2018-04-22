/*
 * Copyright (C) 2016 by kontais (kontais@aliyun.com)
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
 * \file arch/cm3/board/shenzhou_iv.c
 * \brief shenzhou_iv board initialization.
 *
 * cpu:       STM32F107VCT6
 * schematic: http://pan.baidu.com/s/1jGTdHOU
 *
 * \verbatim
 * $Id: shenzhou_iv.c 6299 2016-03-09 11:26:30Z u_bonnes $
 * \endverbatim
 */
#include "cfg/arch/stm32.h"


#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_gpio.h>

/*!
 * \brief   Early hardware initialization.
 *
 * This routine is called during system initalization.
 */
void NutBoardInit(void)
{
  GpioPinConfigSet(NUTGPIO_PORTD, 3, GPIO_CFG_OUTPUT|GPIO_CFG_SPEED_MED);
  GpioPinConfigSet(NUTGPIO_PORTD, 4, GPIO_CFG_OUTPUT|GPIO_CFG_SPEED_MED);
  GpioPinConfigSet(NUTGPIO_PORTD, 7, GPIO_CFG_OUTPUT|GPIO_CFG_SPEED_MED);
}
