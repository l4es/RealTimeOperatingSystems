/*
 * Copyright 2008 by egnite GmbH
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
 * \file arch/avr32/dev/pwm.c
 * \brief AVR32 Pulse Width Modulation.
 *
 * \verbatim
 * $Id: pwm.c,v 1.2 2008/10/03 11:30:22 haraldkipp Exp $
 * \endverbatim
 */

#include <arch/avr32.h>
#include <arch/avr32/gpio.h>
#include <dev/gpio.h>

#include <avr32/io.h>

/*!
 * \addtogroup xgNutArchAvr32Pwm
 */
/*@{*/

static unsigned short FindClockConfiguration(unsigned int frequency, unsigned int mck)
{
    unsigned char divisor = 0;
    unsigned int prescaler;


    NUTASSERT(frequency < mck);

    // Find prescaler and divisor values
    prescaler = (mck / _BV(divisor)) / frequency;
    while ((prescaler > 255) && (divisor < 11)) {

        divisor++;
        prescaler = (mck / _BV(divisor)) / frequency;
    }

    // Return result
    if (divisor < 11) {
        return prescaler | (divisor << 8);
    } else {
        return 0;
    }
}

/*!
    Configures PWM clocks A & B to run at the given frequencies.
    \param clka  Desired clock A frequency (0 if not used).
    \param clkb  Desired clock B frequency (0 if not used).
*/
int NutPwmInit(unsigned int clka, unsigned int clkb)
{
    unsigned int mode = 0;
    unsigned int result;
    unsigned int mck = NutClockGet(NUT_HWCLK_PERIPHERAL_A);

    // Clock A
    if (clka != 0) {
        result = FindClockConfiguration(clka, mck);
        NUTASSERT(result != 0);
        mode |= (result << AVR32_PWM_MR_DIVA_OFFSET);
    }

    // Clock B
    if (clkb != 0) {
        result = FindClockConfiguration(clkb, mck);
        NUTASSERT(result != 0);
        mode |= (result << AVR32_PWM_DIVB_OFFSET);
    }

    // Configure clocks
    AVR32_PWM.mr = mode;

    return 0;
}


/*!
 * \brief System reset.
 */
int NutPwmChannelInit(unsigned int channel_id, unsigned int mode, unsigned int duty, unsigned int period)
{
    NUTASSERT(channel_id <= AVR32_PWM_LINES_MSB);

    switch (channel_id )
    {
    case 0:
        GpioPinConfigSet( AVR32_GPIO_BANK(AVR32_PWM_0_PIN), AVR32_GPIO_PIN(AVR32_PWM_0_PIN), AVR32_GPIO_FUNCTION(AVR32_PWM_0_FUNCTION) );
        break;
    case 1:
        GpioPinConfigSet( AVR32_GPIO_BANK(AVR32_PWM_1_PIN), AVR32_GPIO_PIN(AVR32_PWM_1_PIN), AVR32_GPIO_FUNCTION(AVR32_PWM_1_FUNCTION) );
        break;
    case 2:
        GpioPinConfigSet( AVR32_GPIO_BANK(AVR32_PWM_2_PIN), AVR32_GPIO_PIN(AVR32_PWM_2_PIN), AVR32_GPIO_FUNCTION(AVR32_PWM_2_FUNCTION) );
        break;
    case 3:
        GpioPinConfigSet( AVR32_GPIO_BANK(AVR32_PWM_3_PIN), AVR32_GPIO_PIN(AVR32_PWM_3_PIN), AVR32_GPIO_FUNCTION(AVR32_PWM_3_FUNCTION) );
        break;
    case 4:
        GpioPinConfigSet( AVR32_GPIO_BANK(AVR32_PWM_4_1_PIN), AVR32_GPIO_PIN(AVR32_PWM_4_1_PIN), AVR32_GPIO_FUNCTION(AVR32_PWM_4_1_FUNCTION) );
        break;
    case 5:
        GpioPinConfigSet( AVR32_GPIO_BANK(AVR32_PWM_5_1_PIN), AVR32_GPIO_PIN(AVR32_PWM_5_1_PIN), AVR32_GPIO_FUNCTION(AVR32_PWM_5_1_FUNCTION) );
        break;
    case 6:
        GpioPinConfigSet( AVR32_GPIO_BANK(AVR32_PWM_6_PIN), AVR32_GPIO_PIN(AVR32_PWM_6_PIN), AVR32_GPIO_FUNCTION(AVR32_PWM_6_FUNCTION) );
        break;
    }

    AVR32_PWM.channel[channel_id].cmr  = mode;   // Channel mode.
    AVR32_PWM.channel[channel_id].cdty = duty; // Duty cycle, should be < CPRD.
    AVR32_PWM.channel[channel_id].cprd = period; // Channel period.

    return 0;
}

int NutPwmStartChannel(unsigned int channel)
{
    AVR32_PWM.ena = _BV(channel);
    return 0;
}

int NutPwmStopChannel(unsigned int channel)
{
    AVR32_PWM.dis = _BV(channel);
    return 0;
}

int NutPwmSetPeriod(unsigned int channel, unsigned short period)
{
    AVR32_PWM.channel[channel].cprd = period;
    return 0;
}

int NutPwmSetDutyCycle(unsigned int channel, unsigned short duty)
{
    AVR32_PWM.channel[channel].cdty = duty;
    return 0;
}
/*@}*/
