#if !defined(_DEV_PWM_H_)
#define      _DEV_PWM_H_

/*
 * Copyright (C) 2015 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file dev/pwm.h
 * \brief Abstraction for Pwm output.
 */

/*!
 * \addtogroup xgPwm
 */
/*@{*/

/*!
 * Pwm Device structure.
 *
 */
typedef struct _NUTPWM NUTPWM;

/*!
 * \brief Device structure.
 *
 */
struct _NUTPWM {
    /*!
     * \brief Generic container for device configuration and data
     */
    uintptr_t    hw;
    /*!
     * \brief Driver initialization routine.
     *
     * This routine is called during device registration.
     */
    int          (*PwmInit) (
        NUTPWM *pwm, unsigned int reload, unsigned int prescaler);
    /*!
     * \brief Set PWM value.
     *
     * Used to modify Pwm settings.
     */
    void         (*PwmSet) (NUTPWM *pwm_dev, unsigned int value);
    /*!
     * \brief Get PWM value.
     *
     * Used to query Pwm settings.
     */
    unsigned int (*PwmGet) (NUTPWM *pwm_dev);
    /*!
     * \brief Get PWM clock.
     *
     * Used to query Pwm clock.
     */
    uint32_t     (*PwmGetClock)(NUTPWM *pwm_dev);
};

extern int PwmInit(
    NUTPWM *pwm_dev, unsigned int reload, unsigned int prescaler);
extern uint32_t PwmGetClock(NUTPWM *pwm_dev);
extern void  PwmSet(NUTPWM *pwm_dev, unsigned int value);
extern unsigned int PwmGet(NUTPWM *pwm_dev);

/* List of implemented PWM devices */
extern NUTPWM Stm32Pwm0Tim;
extern NUTPWM Stm32Pwm1Tim;

#endif
