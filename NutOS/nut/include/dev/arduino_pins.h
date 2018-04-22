#if !defined(_DEV_ARDUINO_PINS_H)
#define _DEV_ARDUINO_PINS_H
/*
 * Copyright (C) 2016 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de
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

/*!
 * \file include/dev/arduino_pins.h
 * \brief Map Arduino pin name to device pin name
 *
 * Settings may be changed by solder jumpers on the CPU board.
 * So only set the defines if not defined elsewhere.
 *
 * Currently only available for STM32
 *
 */


#if defined(_DEV_PINS_H_)
# warning Include <dev/arduino_pins.h> before <dev/pins.h>
#endif

#include <cfg/arch.h>

#if PLATFORM == NUCLEO
# if !defined(ARDUINO_A0)
#  define ARDUINO_A0   PA00
# endif
# if !defined(ARDUINO_A1)
#  define ARDUINO_A1   PA01
# endif
# if !defined(ARDUINO_A2)
#  define ARDUINO_A2   PA04
# endif
# if !defined(ARDUINO_A3)
#  define ARDUINO_A3   PB00
# endif
# if !defined(ARDUINO_A4)
#  define ARDUINO_A4   PC01
# endif
# if !defined(ARDUINO_A5)
#  define ARDUINO_A5   PC00
# endif

# if !defined(ARDUINO_D0)
#  define ARDUINO_D0   PA03
# endif
# if !defined(ARDUINO_D1)
#  define ARDUINO_D1   PA02
# endif
# if !defined(ARDUINO_D2)
#  define ARDUINO_D2   PA10
# endif
# if !defined(ARDUINO_D3)
#  define ARDUINO_D3   PB03
# endif
# if !defined(ARDUINO_D4)
#  define ARDUINO_D4   PB05
# endif
# if !defined(ARDUINO_D5)
#  define ARDUINO_D5   PB04
# endif
# if !defined(ARDUINO_D6)
#  define ARDUINO_D6   PB10
# endif
# if !defined(ARDUINO_D7)
#  define ARDUINO_D7   PA08
# endif
# if !defined(ARDUINO_D8)
#  define ARDUINO_D8   PA09
# endif
# if !defined(ARDUINO_D9)
#  define ARDUINO_D9   PC07
# endif
# if !defined(ARDUINO_D10)
#  define ARDUINO_D10  PB06
# endif

# if defined(STM32F302x8)
#  if !defined(ARDUINO_D11)
#   define ARDUINO_D11 PB15
#  endif
#  if !defined(ARDUINO_D12)
#   define ARDUINO_D12 PB14
#  endif
#  if !defined(ARDUINO_D13)
#   define ARDUINO_D13 PB13
#  endif
# else
#  if !defined(ARDUINO_D11)
#   define ARDUINO_D11 PA07
#  endif
#  if !defined(ARDUINO_D12)
#   define ARDUINO_D12 PA06
#  endif
#  if !defined(ARDUINO_D13)
#   define ARDUINO_D13 PA05
#  endif
# endif

# if !defined(ARDUINO_D14)
#  define ARDUINO_D14  PB09
# endif
# if !defined(ARDUINO_D15)
#  define ARDUINO_D15  PB08
# endif

#elif PLATFORM == NUCLEO32
# if !defined(ARDUINO_A0)
#  define ARDUINO_A0   PA00
# endif
# if !defined(ARDUINO_A1)
#  define ARDUINO_A1   PA01
# endif
# if !defined(ARDUINO_A2)
#  define ARDUINO_A2   PA03
# endif
# if !defined(ARDUINO_A3)
#  define ARDUINO_A3   PA04
# endif
# if !defined(ARDUINO_A4)
#  define ARDUINO_A4   PA05
# endif
# if !defined(ARDUINO_A5)
#  define ARDUINO_A5   PA06
# endif
# if !defined(ARDUINO_A6)
#  define ARDUINO_A6   PA07
# endif
# if !defined(ARDUINO_A7)
#  define ARDUINO_A7   PA02
# endif
# if !defined(ARDUINO_D0)
#  define ARDUINO_D0   PA10
# endif
# if !defined(ARDUINO_D1)
#  define ARDUINO_D1   PA09
# endif
# if !defined(ARDUINO_D2)
#  define ARDUINO_D2   PA12
# endif
# if !defined(ARDUINO_D3)
#  define ARDUINO_D3   PB00
# endif
# if !defined(ARDUINO_D4)
#  define ARDUINO_D4   PPB7
# endif
# if !defined(ARDUINO_D5)
#  define ARDUINO_D5   PB06
# endif
# if !defined(ARDUINO_D6)
#  define ARDUINO_D6   PB01
# endif
# if !defined(ARDUINO_D7)
#  define ARDUINO_D7   PF00
# endif
# if !defined(ARDUINO_D8)
#  define ARDUINO_D8   PF01
# endif
# if !defined(ARDUINO_D9)
#  define ARDUINO_D9   PA08
# endif
# if !defined(ARDUINO_D10)
#  define ARDUINO_D10  PA11
# endif
# if !defined(ARDUINO_D11)
#  define ARDUINO_D11  PB05
# endif
# if !defined(ARDUINO_D12)
#  define ARDUINO_D12  PA04
# endif

#elif PLATFORM == NUCLEO144
# if !defined(ARDUINO_A0)
#  define ARDUINO_A0   PA03
# endif
# if !defined(ARDUINO_A1)
#  define ARDUINO_A1   PC00
# endif
# if !defined(ARDUINO_A2)
#  define ARDUINO_A2   PC03
# endif

# if defined(STM32F303XE)
#  if !defined(ARDUINO_A3)
#   define ARDUINO_A3  PD11
#  endif
#  if !defined(ARDUINO_A4)
#   define ARDUINO_A4  PD12
#  endif
#  if !defined(ARDUINO_A5)
#   define ARDUINO_A5  PD13
#  endif

#  if !defined(ARDUINO_D0)
#   define ARDUINO_D0  PC05
#  endif
#  if !defined(ARDUINO_D1)
#   define ARDUINO_D1  PC04
#  endif
# else
#  if !defined(ARDUINO_A3)
#   define ARDUINO_A3  PF03
#  endif
#  if !defined(ARDUINO_A4)
#   define ARDUINO_A4  PF05
#  endif
#  if !defined(ARDUINO_A5)
#   define ARDUINO_A5  PF10
#  endif

#  if !defined(ARDUINO_D0)
#   define ARDUINO_D0  PG09
#  endif
#  if !defined(ARDUINO_D1)
#   define ARDUINO_D1  PG14
#  endif
# endif

# if !defined(ARDUINO_D2)
#  define ARDUINO_D2   PF15
# endif
# if !defined(ARDUINO_D3)
#  define ARDUINO_D3   PE13
# endif
# if !defined(ARDUINO_D4)
#  define ARDUINO_D4   PF14
# endif
# if !defined(ARDUINO_D5)
#  define ARDUINO_D5   PE11
# endif
# if !defined(ARDUINO_D6)
#  define ARDUINO_D6   PE09
# endif
# if !defined(ARDUINO_D7)
#  define ARDUINO_D7   PF13
# endif
# if !defined(ARDUINO_D8)
#  define ARDUINO_D8   PF12
# endif
# if !defined(ARDUINO_D9)
#  define ARDUINO_D9   PD15
# endif
# if !defined(ARDUINO_D10)
#  define ARDUINO_D10  PD14
# endif
# if !defined(ARDUINO_D11)
#  define ARDUINO_D11  PA07
# endif
# if !defined(ARDUINO_D12)
#  define ARDUINO_D12  PA06
# endif
# if !defined(ARDUINO_D13)
#  define ARDUINO_D13  PA05
# endif
# if !defined(ARDUINO_D14)
#  define ARDUINO_D14  PB09
# endif
# if !defined(ARDUINO_D15)
#  define ARDUINO_D15  PB08
# endif

#elif PLATFORM == F7_DISCOVERY
# if !defined(ARDUINO_A0)
#  define ARDUINO_A0   PA00
# endif
# if !defined(ARDUINO_A1)
#  define ARDUINO_A1   PF10
# endif
# if !defined(ARDUINO_A2)
#  define ARDUINO_A2   PF09
# endif
# if !defined(ARDUINO_A3)
#  define ARDUINO_A3   PF08
# endif
# if !defined(ARDUINO_A4)
#  define ARDUINO_A4   PF07
# endif
# if !defined(ARDUINO_A5)
#  define ARDUINO_A5   PF06
# endif

# if !defined(ARDUINO_D0)
#  define ARDUINO_D0   PC07
# endif
# if !defined(ARDUINO_D1)
#  define ARDUINO_D1   PC06
# endif
# if !defined(ARDUINO_D2)
#  define ARDUINO_D2   PG06
# endif
# if !defined(ARDUINO_D3)
#  define ARDUINO_D3   PB04
# endif
# if !defined(ARDUINO_D4)
#  define ARDUINO_D4   PG07
# endif
# if !defined(ARDUINO_D5)
#  define ARDUINO_D5   PA08
# endif
# if !defined(ARDUINO_D6)
#  define ARDUINO_D6   PH06
# endif
# if !defined(ARDUINO_D7)
#  define ARDUINO_D7   PI03
# endif
# if !defined(ARDUINO_D8)
#  define ARDUINO_D8   PI02
# endif
# if !defined(ARDUINO_D9)
#  define ARDUINO_D9   PA15
# endif
# if !defined(ARDUINO_D10)
#  define ARDUINO_D10  PI00
# endif
# if !defined(ARDUINO_D11)
#  define ARDUINO_D11  PB15
# endif
# if !defined(ARDUINO_D12)
#  define ARDUINO_D12  PB14
# endif
# if !defined(ARDUINO_D13)
#  define ARDUINO_D13  PI01
# endif
# if !defined(ARDUINO_D14)
#  define ARDUINO_D14  PB09
# endif
# if !defined(ARDUINO_D15)
#  define ARDUINO_D15  PB08
# endif

#elif PLATFORM == F469_DISCOVERY
# if !defined(ARDUINO_A0)
#  define ARDUINO_A0   PB01
# endif
# if !defined(ARDUINO_A1)
#  define ARDUINO_A1   PC02
# endif
# if !defined(ARDUINO_A2)
#  define ARDUINO_A2   PC03
# endif
# if !defined(ARDUINO_A3)
#  define ARDUINO_A3   PC04
# endif
# if !defined(ARDUINO_A4)
#  define ARDUINO_A4   PC05
# endif
# if !defined(ARDUINO_A5)
#  define ARDUINO_A5   PA04
# endif

# if !defined(ARDUINO_D0)
#  define ARDUINO_D0   PG09
# endif
# if !defined(ARDUINO_D1)
#  define ARDUINO_D1   PG14
# endif
# if !defined(ARDUINO_D2)
#  define ARDUINO_D2   PG13
# endif
# if !defined(ARDUINO_D3)
#  define ARDUINO_D3   PA01
# endif
# if !defined(ARDUINO_D4)
#  define ARDUINO_D4   PG12
# endif
# if !defined(ARDUINO_D5)
#  define ARDUINO_D5   PA02
# endif
# if !defined(ARDUINO_D6)
#  define ARDUINO_D6   PA06
# endif
# if !defined(ARDUINO_D7)
#  define ARDUINO_D7   PG11
# endif
# if !defined(ARDUINO_D8)
#  define ARDUINO_D8   PG10
# endif
# if !defined(ARDUINO_D9)
#  define ARDUINO_D9   PA07
# endif
# if !defined(ARDUINO_D10)
#  define ARDUINO_D10  PH06
# endif
# if !defined(ARDUINO_D11)
#  define ARDUINO_D11  PB15
# endif
# if !defined(ARDUINO_D12)
#  define ARDUINO_D12  PB14
# endif
# if !defined(ARDUINO_D13)
#  define ARDUINO_D13  PD03
# endif
# if !defined(ARDUINO_D14)
#  define ARDUINO_D14  PB09
# endif
# if !defined(ARDUINO_D15)
#  define ARDUINO_D15  PB08
# endif

#else
# warning Please define pin mapping for your platform
#endif

#endif
