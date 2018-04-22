/*
 * Copyright (C) 2017 Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de
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
#if defined(HW_LPUART1_COMBINED_IRQ_STM32)
IRQ_HANDLER sig_LPUART1 = {0};
# if defined(HW_RNG_STM32)
IRQ_HANDLER sig_RNG = {0};
# endif
# if defined(HW_AES_COMBINED_IRQ_STM32)
IRQ_HANDLER sig_AES = {0};
# endif
#endif

/*!
 * \brief Handle the combined AES/RNG/LPUART1 interrupt
 *
 * We assume for each activated device that a handler has been
 * registered before!
 * Otherwise the interrupt will be unhandled!
 *
 * \param arg  Not used
 *
 * \return None
 */
static void Stm32AesRngLpuart1Handler(void *arg)
{
    if ((sig_LPUART1.ir_handler) && LPUART1->ISR) {
#if defined(NUT_PERFMON)
        sig_LPUART1.ir_count++;
#endif
        sig_LPUART1.ir_handler(sig_LPUART1.ir_arg);
    }
# if defined(HW_RNG_STM32)
    if ((sig_RNG.ir_handler) &&
        (RNG->CR & RNG_CR_IE) &&
        (RNG->SR & (RNG_SR_SEIS | RNG_SR_CEIS | RNG_SR_DRDY))) {
#if defined(NUT_PERFMON)
        sig_RNG.ir_count++;
#endif
        sig_RNG.ir_handler(sig_RNG.ir_arg);
    }
# endif
# if defined(HW_AES_STM32)
    if ((sig_AES.ir_handler) && (AES->SR)) {
#if defined(NUT_PERFMON)
        sig_AES.ir_count++;
#endif
        sig_AES.ir_handler(sig_AES.ir_arg);
    }
# endif
}

/*!
 * \brief Install handler for AES/RNG/LPUART1 combined interrupt
 *
 * If handler is already install, do nothing
 *
 * \param  void
 *
 * \return 0 on success, anything else on failure
 */
static int Stm32AesRngLpuart1InstallHandler(void)
{
    if (sig_LPUART1.ir_handler == NULL) {
        IRQ_HANDLER *sig = &sig_LPUART1_GROUP;
        int ret;
        ret = NutRegisterIrqHandler(sig, Stm32AesRngLpuart1Handler, NULL);
        if (ret) {
            return -1;
        } else {
            NutIrqEnable(sig);
        }
    }
    return 0;
}

# if defined(HW_USART4_5_STM32)
IRQ_HANDLER sig_USART4  = {0};
IRQ_HANDLER sig_USART5  = {0};
/*!
 * \brief Handle the combined USART4/5 interrupt
 *
 * We assume for each activated device that a handler has been
 * registered before!
 * Otherwise the interrupt will be unhandled!
 *
 * \param arg  Not used
 *
 * \return None
 */
static void Usart45Handler(void *arg)
{
    if ((sig_USART4.ir_handler) && USART4->ISR) {
#if defined(NUT_PERFMON)
        sig_USART4.ir_count++;
#endif
        sig_USART4.ir_handler(sig_USART4.ir_arg);
    }
    if ((sig_USART5.ir_handler) && USART5->ISR) {
 #if defined(NUT_PERFMON)
        sig_USART5.ir_count++;
#endif
        sig_USART5.ir_handler(sig_USART5.ir_arg);
    }
}

/*!
 * \brief Install handler for USART4_5 combined interrupt
 *
 * If handler is already install, do nothing
 *
 * \param  void
 *
 * \return 0 on success, anything else on failure
 */
static int Stm32Usart4_5InstallHandler(void)
{
    if (sig_USART4_5.ir_handler == NULL) {
        IRQ_HANDLER *sig = &sig_USART4_5;
        int ret;
        ret = NutRegisterIrqHandler(sig, Usart45Handler, NULL);
        if (ret) {
            return -1;
        } else {
            NutIrqEnable(sig);
        }
    }
    return 0;
}
# endif

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
# if defined(HW_USART1_STM32)
    case 0:
# endif
    case 1:
# if defined(HW_LPUART1_STM32) && !defined(HW_LPUART1_COMBINED_IRQ_STM32)
    case STM32_LPUART1_INDEX:
# endif
        NutIrqEnable(sig);
        break;
# if defined(HW_USART4_5_STM32)
    case 3:
        if (Stm32Usart4_5InstallHandler()) {
            return NULL;
        }
        break;
    case 4:
        if (Stm32Usart4_5InstallHandler()) {
            return NULL;
        }
        break;
# endif
# if defined(HW_LPUART1_COMBINED_IRQ_STM32)
    case STM32_LPUART1_INDEX:
        if (Stm32AesRngLpuart1InstallHandler()) {
            return NULL;
        }
        break;
# endif
    }
    return sig;
}

#if defined(HW_RNG_COMBINED_IRQ_STM32)
IRQ_HANDLER *Stm32RngInstallHandler(IRQ_HANDLER *sig)
{
    if (Stm32AesRngLpuart1InstallHandler()) {
        return NULL;
    }
    return sig;
}
#endif

#if defined(HW_AES_COMBINED_IRQ_STM32)
IRQ_HANDLER *Stm32AesInstallHandler(IRQ_HANDLER *sig)
{
    if (Stm32AesRngLpuart1InstallHandler()) {
        return NULL;
    }
    return sig;
}
#endif

