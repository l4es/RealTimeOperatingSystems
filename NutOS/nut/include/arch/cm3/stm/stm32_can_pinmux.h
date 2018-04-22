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
#if !defined(_STM32_CAN_PINMUX_H_)
# define _STM32_CAN_PINMUX_H_

#if defined(_DEV_PINS_H_)
# warning Include Pinmux before arch/cm3/stm/stm32_gpio.h
#endif

# if defined(MCU_STM32F1)
/* AF definition only to keep the compiler happy*/
#  define CAN1_RX_AF 9
#  define CAN1_TX_AF 9
#  define CAN2_RX_AF 9
#  define CAN2_TX_AF 9
#  if   (CAN1_REMAP == 3)
#   define CAN1_RX PD00
#   define CAN1_TX PD01
#  elif (CAN1_REMAP == 1)
#   define CAN1_RX PB08
#   define CAN1_TX PB09
#  else
#   define CAN1_RX PA11
#   define CAN1_TX PA12
#  endif
#  if   (CAN2_REMAP == 1)
#   define CAN2_RX PB12
#   define CAN2_TX PB13
#  else
#   define CAN2_RX PB05
#   define CAN2_TX PB06
#  endif
# elif defined(MCU_STM32F0)
#  define CAN1_RX_AF 4
#  define CAN1_TX_AF 4
# elif defined(MCU_STM32F3)
#  define CAN1_RX_AF ((CAN1_RX == PD00) ? 7 : 9)
#  define CAN1_TX_AF ((CAN1_TX == PD01) ? 7 : 9)
# else
#  define CAN1_RX_AF 9
#  define CAN1_TX_AF 9
#  define CAN2_RX_AF 9
#  define CAN2_TX_AF 9
# endif
#endif
