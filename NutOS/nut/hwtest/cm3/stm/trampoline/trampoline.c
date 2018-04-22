/*
 * Copyright (C) 2012 Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de
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

/*!
 * \example trampoline
 *
 * Flash this file if you compile to "Run from RAM" with Boot selection
 * "Run from Flash"
 *
 * This trampoline on reset loads the stack adress from the value at
 * 0x20000000 and jumps to the function pointed to by 0x20000004.
 *
 */

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/cortexM3.h>
#if defined(MCU_STM32F4)
#include <arch/cm3/core_cmFunc.h>
#endif
#ifndef MSP_STACK_SIZE
#define MSP_STACK_SIZE 128
#endif

/*!
 * \brief CortexM3 MSP processor stack.
 */
__attribute__ ((section(".stack")))
volatile static uint32_t mspStack[MSP_STACK_SIZE];


static void IntDefaultHandler(void *arg) __attribute__ ((naked));
void TrampolineInit(void *arg) __attribute__ ((naked));

#ifndef NUT_BOOT_FUNCTION
#define NUT_BOOT_FUNCTION TrampolineInit
#endif

static void IntDefaultHandler(void *arg)
{
    while (1);
}

typedef struct
{
    uint32_t  stack;
    void (*RamNutInit)(void);
}SRAM_Vector;

void TrampolineInit(void *arg)
{
    SRAM_Vector *sram = (SRAM_Vector*)0x20000000;
    __ASM volatile ("MSR msp, %0\n" : : "r" (sram->stack) );
    sram->RamNutInit();
}

__attribute__ ((section(".isr_vector")))
void (* g_pfnVectors[NUM_INTERRUPTS])(void*) =
{
    (void (*)(void *))((uint32_t)mspStack + sizeof(mspStack)), /* Initial Stack Pointer */
    NUT_BOOT_FUNCTION, /* Reset_Handler */
    IntDefaultHandler, /* NMI_Handler */
    IntDefaultHandler, /* HardFault_Handler */
    IntDefaultHandler, /* MemManage_Handler */
    IntDefaultHandler, /* BusFault_Handler */
    IntDefaultHandler, /* UsageFault_Handler */
    0,                 /* Reserved */
    0,                 /* Reserved */
    0,                 /* Reserved */
    0,                 /* Reserved */
};
