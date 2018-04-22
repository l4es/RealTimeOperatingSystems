/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2012 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
 * Copyright (C) 2013 by Michael Fischer
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


#include <arch/cm3.h>
#include <sys/nutdebug.h>

/*
 * __vectors_ram_start__ will be provided by CrossWorks for ARM
 */
extern uint32_t *__vectors_ram_start__;
static uint32_t *g_pfnRAMVectors = (uint32_t*)&__vectors_ram_start__;


/* Default interrupt handler */
static void IntDefaultHandler(void *arg)
{
    for (;;);
}


/*!
 * \brief Register interrupt handler in RAM vector table
 *
 * \param int_id    Specifies interrupt ID to register
 *
 * \param pfnHandler Interrupt handler function to be called
 */
void Cortex_RegisterInt(IRQn_Type int_id, void (*pfnHandler)(void*))
{
    uint16_t idx = int_id + 16;

    /* Check for valid interrupt number */
    NUTASSERT(idx < NUM_INTERRUPTS);

    /* Make sure that the RAM vector table is correctly aligned. */
    NUTASSERT(((uint32_t)g_pfnRAMVectors & 0x000003ff) == 0);

    if (pfnHandler != NULL) {
        /* Save the interrupt handler. */
        g_pfnRAMVectors[idx] = (uint32_t)pfnHandler;
    } else {
        /* Reset to default interrupt handler */
        g_pfnRAMVectors[idx] = (uint32_t)&IntDefaultHandler;
    }
}

/*!
 * \brief CortexM3 Startup.
 *
 * This function runs the basic cpu initialization.
 */
void Cortex_Start(void)
{
   /*
    * "MemInit" and "IntInit" was done by the CrossWorks startup
    */
}

/*** EOF ***/
