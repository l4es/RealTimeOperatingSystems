#ifndef _ARCH_CM3_CORTEXM3_H_
#define _ARCH_CM3_CORTEXM3_H_

/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
 * $Id: cortexM3.h $
 * \endverbatim
 */

#include <stdint.h>
#include <cfg/arch.h>
#include <cfg/os.h>

#if defined(MCU_SAM3U)
#include <arch/cm3/atmel/sam3u.h>
#elif defined(MCU_STM32)
#include <arch/cm3/stm/stm32xxxx.h>
#elif defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#else
#warning "Unknown CM3 family"
#endif

#define NUM_INTERRUPTS  (IRQn_MAX + 16)

/* Export initial startup vectors */
#if defined(NUTDEBUG_RAM)
extern void (* g_pfnVectors[])(void *);
#else
extern void (* const g_pfnVectors[])(void *);
#endif

extern void Cortex_RegisterInt(IRQn_Type int_id, void (*pfnHandler)(void*));
extern int  Cortex_ResetCause(void);
extern void Cortex_Start(void);

/*! \addtogroup xgNutArchCm3 */
/*@{*/

#endif  /* _ARCH_CM3_CORTEXM3_H_ */
