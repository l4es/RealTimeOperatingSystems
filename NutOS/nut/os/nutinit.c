/*
 * Copyright (C) 2001-2005 by egnite Software GmbH. All rights reserved.
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
 * $Id: nutinit.c 5472 2013-12-06 00:16:28Z olereinhardt $
 */

#define __NUTINIT__
#include <compiler.h>
#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/timer.h>

#include <sys/confos.h>
#include <string.h>

/*!
 * \addtogroup xgNutInit
 */
/*@{*/

/*!
 * \brief System tick counter.
 *
 * For the time being we put this here to ensure that it will be placed
 * in lower RAM. This is essential for the AVR platform, where we use
 * this counter to determine the system clock and calculate the correct
 * baudrate factors. If this counter would be placed in external RAM,
 * additional wait states may apply.
 *
 * \todo To be removed.
 */
volatile uint8_t ms62_5 = 0;

/*@}*/

#ifdef __NUT_EMULATION__
// avoid stdio nut wrapper */
#define NO_STDIO_NUT_WRAPPER
#include "../arch/unix/os/nutinit.c"
#include "../arch/unix/os/options.c"
#include "../arch/unix/dev/eeprom.c"
#elif defined(__AVR__)
#include "../arch/avr/os/nutinit.c"
#elif defined(__arm__) && !defined(__CORTEX__)
#include "../arch/arm/os/nutinit.c"
#elif defined(__arm__) && defined(__CORTEX__)
#include "../arch/cm3/os/nutinit.c"
#elif defined(__AVR32__)
#include "../arch/avr32/os/nutinit.c"
#elif defined(__H8300H__) || defined(__H8300S__)
#include "../arch/h8300h/os/nutinit.c"
#elif defined(__m68k__)
#include "../arch/m68k/coldfire/os/nutinit.c"
#endif

