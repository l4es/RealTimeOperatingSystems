/*
 * Copyright (C) 2015 Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
#if !defined(_STM32_I2C_PINMUX_H_)
# define _STM32_I2C_PINMUX_H_

#if defined(_DEV_PINS_H_)
# warning Include Pinmux before arch/cm3/stm/stm32_gpio.h
#endif

# if defined(MCU_STM32F1)
/* AF definition only to keep the compiler happy*/
#  define I2C1_SDA_AF  -1
#  define I2C1_SCL_AF  -1
#  define I2C1_SMBA_AF -1
#  if I2C1_REMAP_I2C == 1
#   define I2C1_SDA PB09
#   define I2C1_SCL PB08
#  else
#   define I2C1_SDA PB07
#   define I2C1_SCL PB06
#  endif
#  if defined(I2C1_USE_SMBA)
#   define I2C1_SMBA PB05
#  else
#   define I2C1_SMBA PIN_NONE
#  endif
#  define I2C2_SDA_AF  -1
#  define I2C2_SCL_AF  -1
#  define I2C2_SMBA_AF -1
#  define I2C2_SDA  PB11
#  define I2C2_SCL  PB10
#  if defined(I2C2_USE_SMBA)
#   define I2C1_SMBA  PB12
#  else
#   define I2C2_SMBA PIN_NONE
#  endif
# elif defined(MCU_STM32L0)
#  define I2C1_SDA_AF  ((I2C1_SDA == PB09) ? 4 : 1)
#  define I2C1_SCL_AF  ((I2C1_SCL == PB08) ? 4 : 1)
#  define I2C1_SMBA_AF 3
#  define I2C2_SDA_AF  ((I2C2_SDA == PB11) ? 6 : 5)
#  define I2C2_SCL_AF  ((I2C2_SCL == PB10) ? 6 : 5)
#  define I2C2_SMBA_AF 5
#
# elif defined(MCU_STM32F0)
#  define I2C1_SDA_AF  ((I2C1_SDA == PA10) ? 4 : 1)
#  define I2C1_SCL_AF  ((I2C1_SCL == PA09) ? 4 : 1)
#  define I2C1_SMBA_AF 3
#  define I2C2_SDA_AF  ((I2C2_SDA == PB11) ? 1 : 5)
#  define I2C2_SCL_AF  ((I2C2_SCL == PB10) ? 1 : 5)
#  define I2C2_SMBA_AF -1
#
# else
#  define I2C1_SDA_AF  4
#  define I2C1_SCL_AF  4
#  define I2C1_SMBA_AF 4
#
#  define I2C2_SDA_AF  4
#  define I2C2_SCL_AF  4
#  define I2C2_SMBA_AF 4
#
#  if I2C3_SDA == PB04 && (defined(MCU_STM32F401xC) || defined(MCU_STM32F401xE) || defined(MCU_STM32F411xE))
#   define I2C3_SDA_AF 9
#  elif I2C3_SDA == PC09 && defined(MCU_STM32F3)
#   define I2C3_SDA_AF 3
#  else
#   define I2C3_SDA_AF 4
#  endif
#  define I2C3_SCL_AF  4
#  define I2C3_SMBA_AF 4
#
#  define I2C4_SDA_AF  4
#  define I2C4_SCL_AF  4
#  define I2C4_SMBA_AF 4
#
# endif
#endif
