/*
 * Copyright (C) 2015 Uwe Bonnes (bon@elektron,ikp,physik.tu-darmstadt.de)
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
 * \file dev/pwm.c
 * \brief Hardware independent part Pwm devices.
 *
 * Wrapper functions for easier calling.
 */

/*!
 * \brief Initialize the Pwn device.
 *
 * \param pwm_dev Identifies the Pwm device to initialize.
 * \param reload  Set period of PWM in (prescaled) clock cycles
 * \param prescaler Sets prescaler division factor.
 *
 * \return 0 on success, -1 otherwise, e.g. if reload or prescaler can not
 *              be set to the given function.
 *
 */

#include <stdint.h>
#include <dev/pwm.h>

int PwmInit(NUTPWM *pwm_dev, unsigned int reload, unsigned int prescaler)
{
    if (!pwm_dev->PwmInit) {
        return -1;
    }
    return pwm_dev->PwmInit(pwm_dev, reload, prescaler);
}

/*!
 * \brief Return clock frequency of given PWM device.
 *
 * A call to PwmGetClock before device init should succeed, so that user
 * code can adjust reload and prescaler to reach some specific PWM frequeny
 *
 * \param pwm_dev Identifies the Pwm device to query.
 *
 * \return Clock frequency in Hertz.
 *
 */
uint32_t PwmGetClock(NUTPWM *pwm_dev)
{
    return pwm_dev->PwmGetClock(pwm_dev);
}

/*!
 * \brief Set Duty cycle value of PWM.
 *
 * A call to PwmGetClock before device init should succeed, so that user
 * code can adjust reload and prescaler to reach some specific PWM frequeny.
 *
 * The driver should clip the value with the relaod value!
 *
 * \param pwm_dev Identifies the Pwm device to set.
 * \param value   Pwm clock cycle number when PWM output gets inactive.
 *
 * \return None.
 *
 */
void  PwmSet(NUTPWM *pwm_dev, unsigned int value)
{
    pwm_dev->PwmSet(pwm_dev, value);
}

/*!
 * \brief Query Duty cycle value of PWM.
 *
 * \param pwm_dev Identifies the Pwm device to query.
 *
 * \return Pwm clock cycle number when PWM output gets inactive.
 *
 */
unsigned int PwmGet(NUTPWM *pwm_dev)
{
    return pwm_dev->PwmGet(pwm_dev);
}
