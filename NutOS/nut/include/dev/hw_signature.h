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
# if !defined(__UNIQUE_ID_H__)
#  define __UNIQUE_ID_H__

#include <cfg/arch.h>

# if defined(MCU_STM32) && !defined(MCU_STM32F030)
#  if defined(MCU_STM32F0)
#   define UNIQUE_ID_REG 0x1ffff7ac
#  elif defined(MCU_STM32F1)
#   define UNIQUE_ID_REG 0x1ffff7e8
#  elif defined(MCU_STM32F2)
#   define UNIQUE_ID_REG 0x1fff7a10
#  elif defined(MCU_STM32F3)
#   define UNIQUE_ID_REG 0x1ffff7ac
#  elif defined(MCU_STM32F4)
#   define UNIQUE_ID_REG 0x1fff7a10
#  elif defined(MCU_STM32F72)
#   define UNIQUE_ID_REG 0x1ff07a22
#  elif defined(MCU_STM32F7)
#   define UNIQUE_ID_REG 0x1ff0f420
#  elif defined(MCU_STM32L0)
#   define UNIQUE_ID_REG 0x1ff8007c
#  elif defined(MCU_STM32L1)
#   if defined(MCU_STM32L1_CAT1) || defined(MCU_STM32L1_CAT2)
#    define UNIQUE_ID_REG_L 0x1ff80050
#    define UNIQUE_ID_REG_M 0x1ff80054
#    define UNIQUE_ID_REG_H 0x1ff80064
#   else
#    define UNIQUE_ID_REG_L 0x1ff800D0
#    define UNIQUE_ID_REG_M 0x1ff800D4
#    define UNIQUE_ID_REG_H 0x1ff800E4
#   endif
#  elif defined(MCU_STM32L4)
#   define UNIQUE_ID_REG 0x1fff7590
#  else
#   warning UNIQUE_ID_REG unhandled STM32 family
#  endif
#
#  if defined(UNIQUE_ID_REG)
#   define UNIQUE_ID_REG_L (UNIQUE_ID_REG + 0)
#   define UNIQUE_ID_REG_M (UNIQUE_ID_REG + 4)
#   define UNIQUE_ID_REG_H (UNIQUE_ID_REG + 8)
#  endif
# endif

# if defined(UNIQUE_ID_REG_L)
extern void Stm32GetUniquePrivateMac(void * mac);
#  define UNIQUE_PRIVATE_MAC(x) Stm32GetUniquePrivateMac(x)
# endif
#endif
