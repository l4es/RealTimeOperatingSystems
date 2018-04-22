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
#if !defined(_STM32_SPI_PINMUX_H_)
# define _STM32_SPI_PINMUX_H_

#if defined(_DEV_PINS_H_)
# warning Include Pinmux before arch/cm3/stm/stm32_gpio.h
#endif

/* SPI1 */
# if defined(MCU_STM32F1)
/* AF definition only to keep the compiler happy*/
#  define SPI1_NSS_AF  -1
#  define SPI1_SCK_AF  -1
#  define SPI1_MISO_AF -1
#  define SPI1_MOSI_AF -1
#  if SPI1_REMAP == 0
#   define SPI1_SCK  PA05
#   define SPI1_MISO PA06
#   define SPI1_MOSI PA07
#  else
#   define SPI1_SCK  PB03
#   define SPI1_MISO PB04
#   define SPI1_MOSI PB05
#  endif
#  if defined(SPI1_USE_NSS)
#   define SPI1_NSS  ((SPI1_REMAP == 0) ? PA04 : PA15)
#  else
#   define SPI1_NSS  PIN_NONE
#  endif
/* End STM32F1*/
# elif defined(MCU_STM32L0)
#  define SPI1_NSS_AF  0
#  define SPI1_SCK_AF  0
#  define SPI1_MISO_AF 0
#  define SPI1_MOSI_AF 0
# elif defined(MCU_STM32F0)
#  define SPI1_NSS_AF  ((SPI1_NSS  == PE12) ? 1 : 0)
#  define SPI1_SCK_AF  ((SPI1_SCK  == PE13) ? 1 : 0)
#  define SPI1_MISO_AF ((SPI1_MISO == PE14) ? 1 : 0)
#  define SPI1_MOSI_AF ((SPI1_MOSI == PE15) ? 1 : 0)
# elif defined(MCU_STM32F3)
#  define SPI1_NSS_AF  5
#  define SPI1_SCK_AF  ((SPI1_SCK  == PA12) ? 6 : 5)
#  define SPI1_MISO_AF ((SPI1_MISO == PA13) ? 6 : 5)
#  define SPI1_MOSI_AF 5
#else
#  define SPI1_NSS_AF  5
#  define SPI1_SCK_AF  5
#  define SPI1_MISO_AF 5
#  define SPI1_MOSI_AF 5
# endif

/* SPI2 */
# if defined(MCU_STM32F1)
#  define SPI2_NSS_AF  -1
#  define SPI2_SCK_AF  -1
#  define SPI2_MISO_AF -1
#  define SPI2_MOSI_AF -1
#  if defined(SPI2_USE_NSS)
#   define SPI2_NSS PB12
#  else
#   define SPI2_NSS  PIN_NONE
#  endif
#  define SPI2_SCK  PB13
#  define SPI2_MISO PB14
#  define SPI2_MOSI PB15
/* End STM32F1*/
# elif defined(MCU_STM32L0)
#  define SPI2_NSS_AF   0
#  define SPI2_SCK_AF   0
#  define SPI2_MISO_AF ((SPI2_MISO == PC02) ? 2 : 0)
#  define SPI2_MOSI_AF ((SPI2_MOSI == PC03) ? 2 : 0)
# elif defined(MCU_STM32F0)
#  define SPI2_NSS_AF  ((SPI2_NSS  == PB09) ? 5 : (SPI2_NSS  == PB12) ? 0 : 1)
#  define SPI2_SCK_AF  ((SPI2_SCK  == PB10) ? 5 : (SPI2_SCK  == PB13) ? 0 : 1)
#  define SPI2_MISO_AF ((SPI2_MISO == PB14) ? 0 : 1)
#  define SPI2_MOSI_AF ((SPI2_MOSI == PB15) ? 0 : 1)
# elif defined(MCU_STM32F3)
#  define SPI2_NSS_AF  ((SPI2_NSS == PD15)? 6:5)
#  define SPI2_SCK_AF   5
#  define SPI2_MISO_AF  5
#  define SPI2_MOSI_AF  5
# elif defined(MCU_STM32L4)
#  define SPI2_NSS_AF   5
#  define SPI2_SCK_AF   5
#  define SPI2_MISO_AF  5
#  define SPI2_MOSI_AF  5
# else
#  define SPI2_NSS_AF  ((SPI2_NSS == PD01)? 7:5)
#  define SPI2_SCK_AF   5
#  define SPI2_MISO_AF  5
#  define SPI2_MOSI_AF  5
# endif

/* SPI3*/
# if defined(MCU_STM32F1)
#  define SPI3_NSS_AF  -1
#  define SPI3_SCK_AF  -1
#  define SPI3_MISO_AF -1
#  define SPI3_MOSI_AF -1
#  if SPI3_REMAP == 0
#   define SPI3_SCK  PB03
#   define SPI3_MISO PB04
#   define SPI3_MOSI PB05
#  else
#   define SPI3_SCK  PC10
#   define SPI3_MISO PC11
#   define SPI3_MOSI PC12
#  endif
#  if defined(SPI3_USE_NSS)
#   define SPI3_NSS  ((SPI3_REMAP) ? PA15 : PA04)
#  else
#   define SPI3_NSS  PIN_NONE
#  endif
/* End STM32F1*/
# elif defined(MCU_STM32F446)
#  define SPI3_NSS_AF   6
#  define SPI3_SCK_AF   6
#  define SPI3_MISO_AF  6
#  define SPI3_MOSI_AF  ((SPI3_MOSI == PC01 || SPI3_MOSI == PD06)? 5 : (SPI3_MOSI == PB00 || SPI3_MOSI == PB02) ? 7 : 6)
# elif defined(MCU_STM32L4)
#  define SPI3_NSS_AF   6
#  define SPI3_SCK_AF   6
#  define SPI3_MISO_AF  6
#  define SPI3_MOSI_AF  6
# else
#  define SPI3_NSS_AF   6
#  define SPI3_SCK_AF   ((SPI3_SCK  == PB12)? 7 : 6)
#  define SPI3_MISO_AF  6
#  define SPI3_MOSI_AF  ((SPI3_MOSI == PB02) ? 7 : ((SPI3_MOSI == PD06) || (SPI3_MOSI == PC01))? 5 : 6)
# endif

/* SPI4*/
# if   defined(MCU_STM32F411)
#  define SPI4_NSS_AF   ((SPI4_NSS  == PB12)? 6 : 5)
#  define SPI4_SCK_AF   ((SPI4_SCK  == PB13)? 6 : 5)
#  define SPI4_MISO_AF  ((SPI4_MISO == PA11)? 6 : 5)
#  define SPI4_MOSI_AF  5
# elif  defined(MCU_STM32F446)
#  define SPI4_NSS_AF   ((SPI4_NSS  == PG14)? 6 : 5)
#  define SPI4_SCK_AF   ((SPI4_SCK  == PG11)? 6 : 5)
#  define SPI4_MISO_AF  ((SPI4_MISO == PG12)? 6 : 5)
#  define SPI4_MOSI_AF  ((SPI4_MOSI == PG13)? 6 : 5)
# else
#  define SPI4_NSS_AF   5
#  define SPI4_SCK_AF   5
#  define SPI4_MISO_AF  5
#  define SPI4_MOSI_AF  5
# endif

/* SPI5*/
# if   defined(MCU_STM32F411)
#  define SPI_NSS_AF    6
#  define SPI5_SCK_AF   6
#  define SPI5_MISO_AF  6
#  define SPI5_MOSI_AF  6
# else
#  define SPI5_NSS_AF   5
#  define SPI5_SCK_AF   5
#  define SPI5_MISO_AF  5
#  define SPI5_MOSI_AF  5
# endif

/* SPI6*/
# define SPI6_NSS_AF    5
# define SPI6_SCK_AF    5
# define SPI6_MISO_AF   5
# define SPI6_MOSI_AF   5

#endif
