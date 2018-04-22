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
#if !defined(_STM32_USART_PINMUX_H_)
# define _STM32_USART_PINMUX_H_

#if defined(_DEV_PINS_H_)
# warning Include Pinmux before arch/cm3/stm/stm32_gpio.h
#endif

# if defined(MCU_STM32F1)
/* AF definition only to keep the compiler happy*/
#  define USART1_TX_AF  -1
#  define USART1_RX_AF  -1
#  define USART1_CTS_AF -1
#  define USART1_RTS_AF -1
#  define USART1_CK_AF  -1

#  if defined(USART1_REMAP)
#   define USART1_TX PB06
#   define USART1_RX PB07
#  else
#   define USART1_TX PA09
#   define USART1_RX PA10
#  endif
#  if defined(USART1_USE_CTS)
#   define USART1_CTS PA11
#  else
#   define USART1_CTS PIN_NONE
#  endif
#  if defined(USART1_USE_RTS)
#   define USART1_RTS PA12
#  else
#   define USART1_RTS PIN_NONE
#  endif
#  if defined(USART1_USE_CK)
#   define USART1_CK  PA08
#  else
#   define USART1_CK  PIN_NONE
#  endif
/* End STM32F1*/
# elif defined(MCU_STM32L0)
#  define USART1_TX_AF ((USART1_TX == PB08) ? 0 : 4)
#  define USART1_RX_AF ((USART1_RX == PB07) ? 0 : 4)
#  define USART1_CTS_AF 4
#  define USART1_RTS_AF 4
#  define USART1_CK_AF  4
# elif defined(MCU_STM32F0)
#  define USART1_TX_AF  ((USART1_TX == PB06) ? 0 : 1)
#  define USART1_RX_AF  ((USART1_RX == PB06) ? 0 : 1)
#  define USART1_CTS_AF 1
#  define USART1_RTS_AF 1
#  define USART1_CK_AF  1
# else
#  define USART1_TX_AF  7
#  define USART1_RX_AF  7
#  define USART1_CTS_AF 7
#  define USART1_RTS_AF 7
#  define USART1_CK_AF  7
# endif
/* USART2 */
# if defined(MCU_STM32F1)
#  define USART2_TX_AF  -1
#  define USART2_RX_AF  -1
#  define USART2_CTS_AF -1
#  define USART2_RTS_AF -1
#  define USART2_CK_AF  -1
#  if USART2_REMAP == 1
#   define USART2_TX PD05
#   define USART2_RX PD06
#  else
#   define USART2_TX PA02
#   define USART2_RX PA03
#  endif

#  if defined(USART2_USE_CTS)
#   define USART2_CTS ((USART2_REMAP) ? PD03 : PA00)
#  else
#   define USART2_CTS PIN_NONE
#  endif

#  if defined(USART2_USE_RTS)
#   define USART2_RTS ((USART2_REMAP) ? PD04 : PA01)
#  else
#   define USART2_RTS PIN_NONE
#  endif

#  if defined(USART2_USE_CK)
#   define USART2_CK ((USART2_REMAP) ? PD07 : PA04)
#  else
#   define USART2_CK PIN_NONE
#  endif
/* End STM32F1*/
# elif defined(MCU_STM32L0)
#  define USART2_TX_AF  4
#  define USART2_RX_AF  4
#  define USART2_CTS_AF 4
#  define USART2_RTS_AF 4
#  define USART2_CK_AF  4
# elif defined(MCU_STM32F0)
#  define USART2_TX_AF  ((USART2_TX  == PD05) ? 0 : 1)
#  define USART2_RX_AF  ((USART2_RX  == PD06) ? 0 : 1)
#  define USART2_CTS_AF ((USART2_CTS == PD03) ? 0 : 1)
#  define USART2_RTS_AF ((USART2_RTS == PD04) ? 0 : 1)
#  define USART2_CK_AF  ((USART2_CK  == PD07) ? 0 : 1)
# elif defined(MCU_STM32L4)
#  define USART2_RX_AF  ((USART2_RX  == PA15) ? 3 : 7)
#  define USART2_TX_AF  7
#  define USART2_CTS_AF 7
#  define USART2_RTS_AF 7
#  define USART2_CK_AF  7
# else
#  define USART2_TX_AF  7
#  define USART2_RX_AF  7
#  define USART2_CTS_AF 7
#  define USART2_RTS_AF 7
#  define USART2_CK_AF  7
# endif
/* USART3*/
# if defined(MCU_STM32F1)
#  define USART3_TX_AF  -1
#  define USART3_RX_AF  -1
#  define USART3_CTS_AF -1
#  define USART3_RTS_AF -1
#  define USART3_CK_AF  -1
#  if USART3_REMAP == 0
#   define USART3_TX   PB10
#   define USART3_RX   PB11
#  elif USART3_REMAP == 1
#   define USART3_TX   PC10
#   define USART3_RX   PC11
#  elif USART3_REMAP == 3
#   define USART3_TX   PD08
#   define USART3_RX   PD09
#  endif

#  if defined(USART3_USE_CTS)
#   define USART3_CTS ((USART3_REMAP == 3) ? PD11 : PB13)
#  else
#   define USART3_CTS   PIN_NONE
#  endif

#  if defined(USART3_USE_RTS)
#   define USART3_RTS ((USART3_REMAP == 3) ? PD12 : PB14)
#  else
#   define USART3_RTS   PIN_NONE
#  endif
#  if defined(USART3_USE_CK)
#  define USART3_CK ((USART3_REMAP == 0)? PB12 :(USART3_REMAP == 1)? PC12:PD10)
#  else
#   define USART3_CK   PIN_NONE
#  endif

# elif defined(MCU_STM32F0)
#  define USART3_TX_AF  ((USART3_TX  == PB10)? 4 : (USART3_TX  == PD08)? 0 : 1)
#  define USART3_RX_AF  ((USART3_RX  == PB11)? 4 : (USART3_RX  == PD09)? 0 : 1)
#  define USART3_CK_AF  ((USART3_CK  == PC12)? 1 : (USART3_CK  == PD10)? 0 : 1)
#  define USART3_CTS_AF (                          (USART3_CK  == PD11)? 0 : 4)
#  define USART3_RTS_AF ((USART3_RTS == PD02)? 1 : (USART3_CK  == PD12)? 0 : 4)
# else
#  define USART3_TX_AF  7
#  define USART3_RX_AF  7
#  define USART3_CTS_AF ((USART3_CTS == PB00) ? 8 : 7)
#  define USART3_RTS_AF 7
#  define USART3_CK_AF  7
# endif
/* U(S)ART4 */
# if defined(MCU_STM32F1)
#  define USART4_TX_AF  -1
#  define USART4_RX_AF  -1
#  define USART4_CTS_AF -1
#  define USART4_RTS_AF -1
#  define USART4_CK_AF  -1
#  define USART4_TX   PC10
#  define USART4_RX   PC11
#  define USART4_CTS  PIN_NONE
#  define USART4_RTS  PIN_NONE
#  define USART4_CK   PIN_NONE
# elif defined(MCU_STM32L1)
#  define USART4_TX_AF  8
#  define USART4_RX_AF  8
#  define USART4_CK_AF  -1
#  define USART4_RTS_AF -1
#  define USART4_CTS_AF -1
# elif defined(MCU_STM32F1)
#  define USART4_TX_AF  -1
#  define USART4_RX_AF  -1
#  define USART4_CTS_AF -1
#  define USART4_RTS_AF -1
#  define USART4_CK_AF  -1
#  define USART4_RTS  PIN_NONE
#  define USART4_CTS  PIN_NONE
#  define USART4_CK   PIN_NONE
# elif defined(MCU_STM32F0)
#  define USART4_TX_AF ((USART4_TX == PA00) ? 4 : (USART4_TX == PC10) ? 0 : 1)
#  define USART4_RX_AF ((USART4_RX == PA01) ? 4 : (USART4_RX == PC11) ? 0 : 1)
#  define USART4_RTS_AF 4
#  define USART4_CTS_AF 4
#  define USART4_CK_AF  0
# elif  defined(MCU_STM32F7) || defined(MCU_STM32L4)
#  define USART4_RX_AF   8
#  define USART4_TX_AF   8
#  define USART4_CTS_AF  8
#  define USART4_RTS_AF  8
#  define USART4_CK_AF   -1
# elif  defined(MCU_STM32L0_CAT5)
#  define USART4_RX_AF   6
#  define USART4_TX_AF   6
#  define USART4_CTS_AF  6
#  define USART4_RTS_AF  6
#  define USART4_CK_AF   6
# else
#  define USART4_RX_AF   7
#  define USART4_TX_AF   7
#  define USART4_CTS_AF  -1
#  define USART4_RTS_AF  -1
#  define USART4_CK_AF   -1
# endif
/* U(S)ART5 */
# if defined(MCU_STM32F1)
#  define USART5_TX_AF  -1
#  define USART5_RX_AF  -1
#  define USART5_CTS_AF -1
#  define USART5_RTS_AF -1
#  define USART5_CK_AF  -1
#  define USART5_TX   PC12
#  define USART5_RX   PD02
#  define USART5_CTS  PIN_NONE
#  define USART5_RTS  PIN_NONE
#  define USART5_CK   PIN_NONE
# elif defined(MCU_STM32L1)
#  define USART5_TX_AF  8
#  define USART5_RX_AF  8
#  define USART5_CK_AF  -1
#  define USART5_RTS_AF -1
#  define USART5_CTS_AF -1
# elif defined(MCU_STM32F1)
#  define USART5_TX_AF  -1
#  define USART5_RX_AF  -1
#  define USART5_CTS_AF -1
#  define USART5_RTS_AF -1
#  define USART5_CK_AF  -1
#  define USART5_TX   PC12
#  define USART5_RX   PD02
#  define USART5_RTS  PIN_NONE
#  define USART5_CTS  PIN_NONE
#  define USART5_CK   PIN_NONE
# elif defined(MCU_STM32F0)
#  define USART5_TX_AF  ((USART5_TX  == PB03) ? 4 :(USART5_TX == PC12) ? 2 : 1)
#  define USART5_RX_AF  ((USART5_RX  == PB04) ? 4 :(USART5_RX == PD02) ? 2 : 1)
/* Combined CK_RTS and no CTS on USART5*/
#  define USART5_CTS_AF -1
#  define USART5_RTS_AF ((USART5_RTS == PB05 ) ? 4                         : 1)
#  define USART5_CK_AF  ((USART5_CK  == PB05 ) ? 4                         : 1)
# elif defined(MCU_STM32F2)
#  define USART5_TX_AF  8
#  define USART5_RX_AF  8
#  define USART5_CK_AF  -1
#  define USART5_RTS_AF -1
#  define USART5_CTS_AF -1
# elif defined(MCU_STM32F3)
#  define USART5_TX_AF  5
#  define USART5_RX_AF  5
#  define USART5_CK_AF  -1
#  define USART5_RTS_AF -1
#  define USART5_CTS_AF -1
# elif defined(MCU_STM32F4)
#  if UART5_CTS == PC9
#   define USART5_CTS_AF 7
#  else
#   define USART5_CTS_AF 8
#  endif
#  define USART5_TX_AF   8
#  define USART5_RX_AF   8
#  define USART5_RTS_AF  8
#  define USART5_CK_AF   -1
# elif defined(MCU_STM32F7)
#  define USART5_TX_AF   8
#  define USART5_RX_AF   8
#  define USART5_RTS_AF  7
#  define USART5_CTS_AF  7
#  define USART5_CK_AF   -1
# elif defined(MCU_STM32L4)
#  define USART5_TX_AF   8
#  define USART5_RX_AF   8
#  define USART5_RTS_AF  8
#  define USART5_CTS_AF  8
#  define USART5_CK_AF   -1
# elif defined(MCU_STM32L0_CAT5)
#  define USART5_TX_AF   ((USART5_TX == PC12) ? 2 : 6)
#  define USART5_RX_AF   6
#  define USART5_RTS_AF  6
#  define USART5_CTS_AF  6
#  define USART5_CK_AF   6
# endif

/* USART6 */
# if   defined(MCU_STM32F0)
#  define USART6_TX_AF ((USART6_TX == PA04) ? 5 : (USART6_TX == PC00) ? 2 : 1)
#  define USART6_RX_AF ((USART6_RX == PA05) ? 5 : (USART6_RX == PC01) ? 2 : 1)
#  define USART6_RTS_AF 2
#  define USART6_CTS_AF -1
#  define USART6_CK_AF  2
# else
#  define USART6_TX_AF  8
#  define USART6_RX_AF  8
#  define USART6_CK_AF  8
#  define USART6_RTS_AF 8
#  define USART6_CTS_AF 8
# endif

/* USART7 */
# if   defined(MCU_STM32F0)
#  define USART7_TX_AF  1
#  define USART7_RX_AF  1
#  define USART7_CTS_AF -1
/* Combined CK/RTS and no CTS*/
#  define USART7_RTS_AF 2
#  define USART7_CK_AF  2
# else
#  define USART7_TX_AF  8
#  define USART7_RX_AF  8
#  define USART7_CK_AF  -1
#  define USART7_RTS_AF -1
#  define USART7_CTS_AF -1
# endif

/* USART8 */
# if   defined(MCU_STM32F0)
#  define USART8_TX_AF  ((USART8_TX == PC02) ? 2 : (USART8_TX == PC08) ? 1 : 0)
#  define USART8_RX_AF  ((USART8_RX == PC03) ? 2 : (USART8_RX == PC09) ? 1 : 0)
#  define USART8_CTS_AF -1
/* Combined CK/RTS and no CTS*/
#  define USART8_RTS_AF 2
#  define USART8_CK_AF  2
# else
#  define USART8_TX_AF  8
#  define USART8_RX_AF  8
#  define USART8_CK_AF  -1
#  define USART8_RTS_AF -1
#  define USART8_CTS_AF -1
# endif

/* LPUART1 */
# if defined(MCU_STM32L0_CAT1) || defined(MCU_STM32L0_CAT2)
#  define LPUART1_TX_AF  6
#  define LPUART1_RX_AF  6
#  define LPUART1_CTS_AF ((LPUART1_CTS == PA06)? 4 : 6)
#  define LPUART1_RTS_AF ((LPUART1_RTS == PB01)? 4 : 6)
#  define LPUART1_CK_AF  -1
# elif defined(MCU_STM32L0_CAT3)
#  define LPUART1_TX_AF  ((LPUART1_TX  == PC04) ? 2 : ((LPUART1_TX  == PC10) ? 0 : 4))
#  define LPUART1_RX_AF  ((LPUART1_RX  == PC05) ? 2 : ((LPUART1_RX  == PC11) ? 0 : 4))
#  define LPUART1_CTS_AF 4
#  define LPUART1_RTS_AF ((LPUART1_RTS == PB12) ? 2 : ((LPUART1_RTS == PD02) ? 0 : 4))
#  define LPUART1_CK_AF  -1
# elif defined(MCU_STM32L0_CAT5)
#  define LPUART1_TX_AF  ((LPUART1_TX  == PD08) ? 0 : ((LPUART1_TX  == PC10) ? 0 : ((LPUART1_TX  == PC04) ? 2 : ((LPUART1_TX  == PB11) ? 7 : ((LPUART1_TX  == PB10) ? 4 : 6)))))
#  define LPUART1_RX_AF  ((LPUART1_RX  == PD09) ? 0 : ((LPUART1_RX  == PC11) ? 0 : ((LPUART1_RX  == PC05) ? 2 : ((LPUART1_RX  == PB10) ? 7 : ((LPUART1_RX  == PB11) ? 4 : 6)))))
#  define LPUART1_CTS_AF ((LPUART1_CTS == PD11) ? 0 : 4)
#  define LPUART1_RTS_AF ((LPUART1_RTS == PD12) ? 0 : ((LPUART1_RTS == PD02) ? 0 : ((LPUART1_RTS == PB12) ? 2 : 4 )))
#  define LPUART1_CK_AF  -1
# elif defined(MCU_STM32L4)
#  define LPUART1_TX_AF  8
#  define LPUART1_RX_AF  8
#  define LPUART1_CTS_AF 8
#  define LPUART1_RTS_AF 8
#  define LPUART1_CK_AF  -1
# endif
#endif
