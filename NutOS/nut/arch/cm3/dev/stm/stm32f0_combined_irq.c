/*
 * Copyright (C) 2015-2017
 *                Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de
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

/* ToDo: Allow to deregister DMA Signalhandlers.
 */

#include <stdlib.h>

#include <cfg/arch.h>
#include <cfg/uart.h>
#include <cfg/devices.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>

#include <arch/cm3/stm/stm32_uart.h>

#if defined(HW_USART_COMBINED_IRQ_STM32)
IRQ_HANDLER sig_USART3 = {0};
IRQ_HANDLER sig_USART4 = {0};
# if   defined(HW_USART3_4_STM32)
#  define GROUPHANDLER &sig_USART3_4
# elif defined(HW_USART3_6_STM32)
#  define GROUPHANDLER &sig_USART3_6;
IRQ_HANDLER sig_USART5 = {0};
IRQ_HANDLER sig_USART6 = {0};
# elif defined(HW_USART3_8_STM32)
#  define GROUPHANDLER &sig_USART3_8;
IRQ_HANDLER sig_USART5 = {0};
IRQ_HANDLER sig_USART6 = {0};
IRQ_HANDLER sig_USART7 = {0};
IRQ_HANDLER sig_USART8 = {0};
# endif

/*!
 * \brief Stm32UsartGroupHandler
 *
 * Try to serve the source of the interrupt. Only F09x has
 * a bit for active interrupts.
 *
 * \param  arg: Arguments for the interrupt
 *
 */
static void Stm32UsartGroupHandler(void *arg) {
# if defined(HW_USART3_8_STM32)
    if ((sig_USART3.ir_handler) &&
        (SYSCFG->IT_LINE_SR[29] & SYSCFG_ITLINE29_SR_USART3_GLB)) {
#if defined(NUT_PERFMON)
        sig_USART3.ir_count++;
#endif
        sig_USART3.ir_handler(sig_USART3.ir_arg);
    }
    if ((sig_USART4.ir_handler) &&
        (SYSCFG->IT_LINE_SR[29] & SYSCFG_ITLINE29_SR_USART4_GLB)) {
#if defined(NUT_PERFMON)
        sig_USART4.ir_count++;
#endif
        sig_USART4.ir_handler(sig_USART2.ir_arg);
    }
    if ((sig_USART5.ir_handler) &&
        (SYSCFG->IT_LINE_SR[29] & SYSCFG_ITLINE29_SR_USART5_GLB)) {
#if defined(NUT_PERFMON)
        sig_USART5.ir_count++;
#endif
        sig_USART5.ir_handler(sig_USART5.ir_arg);
    }
    if ((sig_USART6.ir_handler) &&
        (SYSCFG->IT_LINE_SR[29] & SYSCFG_ITLINE29_SR_USART6_GLB)) {
#if defined(NUT_PERFMON)
        sig_USART6.ir_count++;
#endif
        sig_USART6.ir_handler(sig_USART6.ir_arg);
    }
    if ((sig_USART7.ir_handler) &&
        (SYSCFG->IT_LINE_SR[29] & SYSCFG_ITLINE29_SR_USART7_GLB)) {
#if defined(NUT_PERFMON)
        sig_USART7.ir_count++;
#endif
        sig_USART7.ir_handler(sig_USART7.ir_arg);
    }
    if ((sig_USART8.ir_handler) &&
        (SYSCFG->IT_LINE_SR[29] & SYSCFG_ITLINE29_SR_USART8_GLB)) {
#if defined(NUT_PERFMON)
        sig_USART8.ir_count++;
#endif
        sig_USART8.ir_handler(sig_USART8.ir_arg);
    }
# else
    if ((sig_USART3.ir_handler) && USART3->ISR) {
#if defined(NUT_PERFMON)
        sig_USART3.ir_count++;
#endif
        sig_USART3.ir_handler(sig_USART3.ir_arg);
    }
    if ((sig_USART4.ir_handler) && USART4->ISR) {
#if defined(NUT_PERFMON)
        sig_USART4.ir_count++;
#endif
        sig_USART4.ir_handler(sig_USART4.ir_arg);
    }
#  if defined(HW_USART3_6_STM32) || defined(HW_USART3_8_STM32)
    if ((sig_USART5.ir_handler) && USART5->ISR) {
#if defined(NUT_PERFMON)
        sig_USART5.ir_count++;
#endif
        sig_USART5.ir_handler(sig_USART5.ir_arg);
    }
    if ((sig_USART6.ir_handler) && USART6->ISR) {
#if defined(NUT_PERFMON)
        sig_USART6.ir_count++;
#endif
        sig_USART6.ir_handler(sig_USART6.ir_arg);
    }
#  endif
# endif
}

/*!
 * \brief Install handler for USART2_X combined interrupt
 *
 * If handler is already install, do nothing
 *
 * \param  void
 *
 * \return 0 on success, anything else on failure
 */
static int Stm32F0UsartGroupInstallHandler(IRQ_HANDLER *group)
{
    if (group->ir_handler == NULL) {
        int ret;
        ret = NutRegisterIrqHandler(group, Stm32UsartGroupHandler, NULL);
        if (ret) {
            return -1;
        } else {
            NutIrqEnable(group);
        }
    }
    return 0;
}
#endif

/*!
 * \brief Stm32UsartInstallHandler
 *
 * For single IRQs, enable the IRQ and return the IRQ.
 * For combined IRQs, install the handler for the combined IRQ
 * and return the separated IRQ
 *
 * \param  usart_nr: Zero-based UART number.
 *         LPUART1 is STM32_LPUART1_INDEX.
 * \param  sig: Uart signal handler.
 *
 * \return 0 on success, anything else on failure
 */
IRQ_HANDLER *Stm32UsartInstallHandler(int usart_nr, IRQ_HANDLER *sig)
{
    switch (usart_nr) {
    case 0:
#if defined(HW_USART2_STM32)
    case 1:
#endif
        NutIrqEnable(sig);
        break;
#if defined(HW_USART_COMBINED_IRQ_STM32)
    case 2:
    case 3:
# if defined(HW_USART3_6_STM32) || defined(HW_USART3_8_STM32)
    case 4:
    case 5:
#  if defined(HW_USART3_8_STM32)
    case 6:
    case 7:
#  endif
# endif
        if (Stm32F0UsartGroupInstallHandler(&sig_USART_GROUP)) {
            return NULL;
        }
#endif
    }
    return sig;
}
