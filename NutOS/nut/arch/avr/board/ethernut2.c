/*
 * Copyright 2013 by egnite GmbH
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
 * \file arch/avr/board/ethernut2.c
 * \brief Ethernut 2 board initialization.
 *
 * \verbatim
 * $Id: ethernut2.c 5217 2013-06-28 18:43:40Z haraldkipp $
 * \endverbatim
 */

#include <cfg/memory.h>
#include <cfg/arch.h>
#include <cfg/arch/avr.h>
#include <dev/board.h>

#include <toolchain.h>

#ifdef CUSTOM_IT21

static NutIdleCallback It21IdleChain;
/*
 * Ethernut 2 based custom board with DS1232 watchdog hardware.
 * The final application takes care of this. However, in order
 * to run standard Nut/OS applications we need to toggle PD7
 * regularly.
 */
static void It21Idle(void)
{
    static uint8_t toggle;

    sbi(DDRD, 7);
    if (toggle) {
        sbi(PORTD, 7);
    } else {
        cbi(PORTD, 7);
    }
    toggle ^= 1;

    if (It21IdleChain) {
        It21IdleChain();
    }
}

#endif

#ifdef NUT_INIT_MAIN
/*!
 * \brief Ethernut 2 initialization.
 *
 * When called, the idle thread just started and is ready to jump into
 * application code.
 */
void NutMainInit(void)
{
#ifdef CUSTOM_IT21
    It21IdleChain = NutRegisterIdleCallback(It21Idle);
#endif
}

#endif
