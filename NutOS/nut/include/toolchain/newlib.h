#ifndef TOOLCHAIN_NEWLIB_H_
#define TOOLCHAIN_NEWLIB_H_
/*
 * Copyright (C) 2012 by egnite GmbH
 * Copyright (C) 2001-2006 by egnite Software GmbH
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

#ifdef __arm__
#ifdef __CORTEX__
#include <arch/cm3.h>
#endif
#include <arch/arm.h>
#endif

#if defined(MCU_AT91SAM7SE)

/*!
 * \brief Function running in internal RAM.
 *
 * When running in flash or external RAM, certain time critical functions
 * may be executed in internal RAM for optimal performance. Another use is
 * self re-programming, where the programming function itself cannot run in
 * flash.
 *
 * This section will be copied to internal RAM during runtime initialization.
 */
#ifndef SECTION_FUNC_IRAM
#define SECTION_FUNC_IRAM   __attribute__ ((long_call, section(".text_iram")))
#endif

/*!
 * \brief Initialized variables in internal RAM.
 *
 * If variables are by default in external RAM, this attribute can be used
 * to place certain variables in internal RAM. Actually this is rarely used.
 *
 * The initial values in this section will be copied to internal RAM during
 * runtime initialization.
 */
#ifndef SECTION_DATA_IRAM
#define SECTION_DATA_IRAM   __attribute__ ((section(".data_iram")))
#endif

/*!
 * \brief Variables in internal RAM.
 *
 * If variables are by default in external RAM, this attribute can be used
 * to place certain variables in internal RAM. Often used for buffers, when
 * peripheral DMA does not properly work in external SDRAM.
 *
 * This section will be cleared to zero during runtime initialization.
 */
#ifndef SECTION_BSS_IRAM
#define SECTION_BSS_IRAM    __attribute__ ((section(".bss_iram")))
#endif

/*!
 * \brief Function running in external RAM.
 *
 * Typically used for self re-programming functions, if the default code
 * is located in flash.
 *
 * This section will be copied to external RAM during runtime initialization.
 */
#ifndef SECTION_FUNC_XRAM
#define SECTION_FUNC_XRAM   __attribute__ ((long_call, section(".text_xram")))
#endif

/*!
 * \brief Variables in external RAM.
 *
 * If variables are by default in internal RAM, this attribute can be used
 * to place certain variables in external RAM. Rarely used for large buffers,
 * which will not fit in internal RAM.
 *
 * This section will be cleared to zero during runtime initialization.
 */
#ifndef SECTION_BSS_XRAM
#define SECTION_BSS_XRAM    __attribute__ ((section(".bss_xram")))
#endif

#endif

#include <toolchain/generic.h>

#endif
