#ifndef _STM32XXXX_H_
#define _STM32XXXX_H_
/*
 * Copyright (C) 2012-2017 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

/*!
 * \file arch/cm3/stm/atm32xxxx.h
 * \brief Wrapper for the device dependant stm32XZxx.h files.
 * $Id: stm32_flash.h 3220 2010-11-12 13:04:17Z astralix $
 * \verbatim
 */

#include <cfg/arch.h>
#if defined(MCU_STM32F0)
# if   defined(STM32F030x6)
#  include <arch/cm3/stm/vendor/stm32f030x6.h>
# elif defined(STM32F030x8)
#  include <arch/cm3/stm/vendor/stm32f030x8.h>
# elif defined(STM32F030xC)
#  include <arch/cm3/stm/vendor/stm32f030xc.h>
# elif defined(STM32F031x6)
#  include <arch/cm3/stm/vendor/stm32f031x6.h>
# elif defined(STM32F038xx)
#  include <arch/cm3/stm/vendor/stm32f031x6.h>
# elif defined(STM32F042x6)
#  include <arch/cm3/stm/vendor/stm32f042x6.h>
# elif defined(STM32F048xx)
#  include <arch/cm3/stm/vendor/stm32f048xx.h>
# elif defined(STM32F051x8)
#  include <arch/cm3/stm/vendor/stm32f051x8.h>
# elif defined(STM32F058xx)
#  include <arch/cm3/stm/vendor/stm32f058xx.h>
# elif defined(STM32F070x6)
#  include <arch/cm3/stm/vendor/stm32f070x6.h>
# elif defined(STM32F070x8)
#  include <arch/cm3/stm/vendor/stm32f070x8.h>
# elif defined(STM32F070xB)
#  include <arch/cm3/stm/vendor/stm32f070xb.h>
# elif defined(STM32F071xB)
#  include <arch/cm3/stm/vendor/stm32f071xb.h>
# elif defined(STM32F072xB)
#  include <arch/cm3/stm/vendor/stm32f072xb.h>
# elif defined(STM32F078xx)
#  include <arch/cm3/stm/vendor/stm32f078xx.h>
# elif defined(STM32F091xC)
#  include <arch/cm3/stm/vendor/stm32f091xc.h>
# elif defined(STM32F098xx)
#  include <arch/cm3/stm/vendor/stm32f098xx.h>
# else
#  warning Unhandled STM32F0 device
# endif
#elif defined(MCU_STM32F1)
# if   defined(STM32F100xB)
#  include <arch/cm3/stm/vendor/stm32f100xb.h>
# elif defined(STM32F100xE)
#  include <arch/cm3/stm/vendor/stm32f100xe.h>
# elif defined(STM32F101x6)
#  include <arch/cm3/stm/vendor/stm32f101x8.h>
# elif defined(STM32F101xB)
#  include <arch/cm3/stm/vendor/stm32f101xb.h>
# elif defined(STM32F101xE)
#  include <arch/cm3/stm/vendor/stm32f101xe.h>
# elif defined(STM32F102x6)
#  include <arch/cm3/stm/vendor/stm32f102x6.h>
# elif defined(STM32F102xB)
#  include <arch/cm3/stm/vendor/stm32f102xb.h>
# elif defined(STM32F101xG)
#  include <arch/cm3/stm/vendor/stm32f101xg.h>
# elif defined(STM32F103x8)
#  include <arch/cm3/stm/vendor/stm32f103x8.h>
# elif defined(STM32F103xB)
#  include <arch/cm3/stm/vendor/stm32f103xb.h>
# elif defined(STM32F103xE)
#  include <arch/cm3/stm/vendor/stm32f103xe.h>
# elif defined(STM32F103xG)
#  include <arch/cm3/stm/vendor/stm32f103xg.h>
# elif defined(STM32F105xC)
#  include <arch/cm3/stm/vendor/stm32f105xc.h>
# elif defined(STM32F107xC)
#  include <arch/cm3/stm/vendor/stm32f107xc.h>
# endif
#elif defined(MCU_STM32L0)
# if   defined(STM32L011xx)
#  include <arch/cm3/stm/vendor/stm32l011xx.h>
#  define MCU_STM32L0_CAT1
# elif defined(STM32L021xx)
#  include <arch/cm3/stm/vendor/stm32l021xx.h>
#  define MCU_STM32L0_CAT1
# elif defined(STM32L031xx)
#  include <arch/cm3/stm/vendor/stm32l031xx.h>
#  define MCU_STM32L0_CAT2
# elif defined(STM32L041xx)
#  include <arch/cm3/stm/vendor/stm32l041xx.h>
#  define MCU_STM32L0_CAT2
# elif defined(STM32L051xx)
#  include <arch/cm3/stm/vendor/stm32l051xx.h>
#  define MCU_STM32L0_CAT3
# elif defined(STM32L052xx)
#  include <arch/cm3/stm/vendor/stm32l052xx.h>
#  define MCU_STM32L0_CAT3
# elif defined(STM32L053xx)
#  include <arch/cm3/stm/vendor/stm32l053xx.h>
#  define MCU_STM32L0_CAT3
# elif defined(STM32L061xx)
#  include <arch/cm3/stm/vendor/stm32l061xx.h>
#  define MCU_STM32L0_CAT3
# elif defined(STM32L062xx)
#  include <arch/cm3/stm/vendor/stm32l062xx.h>
#  define MCU_STM32L0_CAT3
# elif defined(STM32L063xx)
#  include <arch/cm3/stm/vendor/stm32l063xx.h>
#  define MCU_STM32L0_CAT3
# elif defined(STM32L071xx)
#  include <arch/cm3/stm/vendor/stm32l071xx.h>
#  define MCU_STM32L0_CAT5
# elif defined(STM32L072xx)
#  include <arch/cm3/stm/vendor/stm32l072xx.h>
#  define MCU_STM32L0_CAT5
# elif defined(STM32L073xx)
#  include <arch/cm3/stm/vendor/stm32l073xx.h>
#  define MCU_STM32L0_CAT5
# elif defined(STM32L081xx)
#  include <arch/cm3/stm/vendor/stm32l081xx.h>
# elif defined(STM32L082xx)
#  define MCU_STM32L0_CAT5
#  include <arch/cm3/stm/vendor/stm32l082xx.h>
# elif defined(STM32L083xx)
#  include <arch/cm3/stm/vendor/stm32l083xx.h>
#  define MCU_STM32L0_CAT5
# endif
#elif defined(MCU_STM32L1)
# if   defined(STM32L100xB)
#  include <arch/cm3/stm/vendor/stm32l100xb.h>
# elif defined(STM32L100xBA)
#  include <arch/cm3/stm/vendor/stm32l100xba.h>
# elif defined(STM32L100xC)
#  include <arch/cm3/stm/vendor/stm32l100xc.h>
# elif defined(STM32L151xB)
#  include <arch/cm3/stm/vendor/stm32l151xb.h>
# elif defined(STM32L151xBA)
#  include <arch/cm3/stm/vendor/stm32l151xba.h>
# elif defined(STM32L151xC)
#  include <arch/cm3/stm/vendor/stm32l151xc.h>
# elif defined(STM32L151xCA)
#  include <arch/cm3/stm/vendor/stm32l151xca.h>
# elif defined(STM32L151xD)
#  include <arch/cm3/stm/vendor/stm32l151xd.h>
# elif defined(STM32L151xDX)
#  include <arch/cm3/stm/vendor/stm32l151xdx.h>
# elif defined(STM32L151xE)
#  include <arch/cm3/stm/vendor/stm32l151xe.h>
# elif defined(STM32L152xB)
#  include <arch/cm3/stm/vendor/stm32l152xb.h>
# elif defined(STM32L152xBA)
#  include <arch/cm3/stm/vendor/stm32l152xba.h>
# elif defined(STM32L152xC)
#  include <arch/cm3/stm/vendor/stm32l152xc.h>
# elif defined(STM32L152xCA)
#  include <arch/cm3/stm/vendor/stm32l152xca.h>
# elif defined(STM32L152xD)
#  include <arch/cm3/stm/vendor/stm32l152xd.h>
# elif defined(STM32L152xDX)
#  include <arch/cm3/stm/vendor/stm32l152xdx.h>
# elif defined(STM32L152xE)
#  include <arch/cm3/stm/vendor/stm32l152xe.h>
# elif defined(STM32L162xC)
#  include <arch/cm3/stm/vendor/stm32l162xc.h>
# elif defined(STM32L162xCA)
#  include <arch/cm3/stm/vendor/stm32l162xca.h>
# elif defined(STM32L162xD)
#  include <arch/cm3/stm/vendor/stm32l162xd.h>
# elif defined(STM32L162xDX)
#  include <arch/cm3/stm/vendor/stm32l162xdx.h>
# elif defined(STM32L162xE)
#  include <arch/cm3/stm/vendor/stm32l162xe.h>
#endif
#elif defined(MCU_STM32L4)
# if   defined(STM32L431xx)
#  include <arch/cm3/stm/vendor/stm32l431xx.h>
# elif defined(STM32L432xx)
#  include <arch/cm3/stm/vendor/stm32l432xx.h>
# elif defined(STM32L433xx)
#  include <arch/cm3/stm/vendor/stm32l433xx.h>
# elif defined(STM32L442xx)
#  include <arch/cm3/stm/vendor/stm32l442xx.h>
# elif defined(STM32L443xx)
#  include <arch/cm3/stm/vendor/stm32l443xx.h>
# elif defined(STM32L451xx)
#  include <arch/cm3/stm/vendor/stm32l451xx.h>
# elif defined(STM32L452xx)
#  include <arch/cm3/stm/vendor/stm32l452xx.h>
# elif defined(STM32L462xx)
#  include <arch/cm3/stm/vendor/stm32l462xx.h>
# elif defined(STM32L471xx)
#  include <arch/cm3/stm/vendor/stm32l471xx.h>
# elif defined(STM32L475xx)
#  include <arch/cm3/stm/vendor/stm32l475xx.h>
# elif defined(STM32L476xx)
#  include <arch/cm3/stm/vendor/stm32l476xx.h>
# elif defined(STM32L485xx)
#  include <arch/cm3/stm/vendor/stm32l485xx.h>
# elif defined(STM32L486xx)
#  include <arch/cm3/stm/vendor/stm32l486xx.h>
# elif defined(STM32L496xx)
#  include <arch/cm3/stm/vendor/stm32l496xx.h>
# elif defined(STM32L4A6xx)
#  include <arch/cm3/stm/vendor/stm32l4a6xx.h>
# elif defined(STM32L4R5xx)
#  include <arch/cm3/stm/vendor/stm32l4r5xx.h>
# elif defined(STM32L4R7xx)
#  include <arch/cm3/stm/vendor/stm32l4r7xx.h>
# elif defined(STM32L4R9xx)
#  include <arch/cm3/stm/vendor/stm32l4r9xx.h>
# elif defined(STM32L4S5xx)
#  include <arch/cm3/stm/vendor/stm32l4s5xx.h>
# elif defined(STM32L4S7xx)
#  include <arch/cm3/stm/vendor/stm32l4s7xx.h>
# elif defined(STM32L4S9xx)
#  include <arch/cm3/stm/vendor/stm32l4r9xx.h>
# endif
#elif defined(MCU_STM32F2)
/**
  * @brief CMSIS Device version number V2.0.1
  */
#define __STM32F2xx_CMSIS_DEVICE_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM32F2xx_CMSIS_DEVICE_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM32F2xx_CMSIS_DEVICE_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __STM32F2xx_CMSIS_DEVICE_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32F2xx_CMSIS_DEVICE_VERSION        ((__CMSIS_DEVICE_VERSION_MAIN     << 24)\
                                      |(__CMSIS_DEVICE_HAL_VERSION_SUB1 << 16)\
                                      |(__CMSIS_DEVICE_HAL_VERSION_SUB2 << 8 )\
                                      |(__CMSIS_DEVICE_HAL_VERSION_RC))
#if defined(STM32F205xx)
#include <arch/cm3/stm/vendor/stm32f205xx.h>
#elif defined(STM32F215xx)
#include  <arch/cm3/stm/vendor/stm32f215xx.h>
#elif defined(STM32F207xx)
#include  <arch/cm3/stm/vendor/stm32f207xx.h>
#elif defined(STM32F217xx)
#include  <arch/cm3/stm/vendor/stm32f217xx.h>
#else
#warning "Unknown STM32F2 family"
#endif
#elif defined(MCU_STM32F3)
# if   defined(STM32F301x8)
#  include <arch/cm3/stm/vendor/stm32f301x8.h>
# elif defined(STM32F302x8)
#  include <arch/cm3/stm/vendor/stm32f302x8.h>
# elif defined(STM32F302xC)
#  include <arch/cm3/stm/vendor/stm32f302xc.h>
# elif defined(STM32F302xE)
#  include <arch/cm3/stm/vendor/stm32f302xe.h>
# elif defined(STM32F303x8)
#  include <arch/cm3/stm/vendor/stm32f303x8.h>
# elif defined(STM32F303xC)
#  include <arch/cm3/stm/vendor/stm32f303xc.h>
# elif defined(STM32F303xE)
#  include <arch/cm3/stm/vendor/stm32f303xe.h>
# elif defined(STM32F318xx)
#  include <arch/cm3/stm/vendor/stm32f318xx.h>
# elif defined(STM32F328xx)
#  include <arch/cm3/stm/vendor/stm32f328xx.h>
# elif defined(STM32F334x8)
#  include <arch/cm3/stm/vendor/stm32f334x8.h>
# elif defined(STM32F358xx)
#  include <arch/cm3/stm/vendor/stm32f358xx.h>
# elif defined(STM32F373xC)
#  include <arch/cm3/stm/vendor/stm32f373xc.h>
# elif defined(STM32F378xx)
#  include <arch/cm3/stm/vendor/stm32f378xx.h>
# elif defined(STM32F379xx)
#  include <arch/cm3/stm/vendor/stm32f398xx.h>
# else
#  warning "Unhandled STM32F3 device"
# end
# endif
#elif defined(MCU_STM32F4)
# if defined(STM32F401xC)
#  include <arch/cm3/stm/vendor/stm32f401xc.h>
# elif defined(STM32F401xE)
#  include <arch/cm3/stm/vendor/stm32f401xe.h>
# elif defined(STM32F405xx)
#  include <arch/cm3/stm/vendor/stm32f405xx.h>
# elif defined(STM32F407xx)
#  include <arch/cm3/stm/vendor/stm32f407xx.h>
# elif defined(STM32F410Cx)
#  include <arch/cm3/stm/vendor/stm32f410cx.h>
# elif defined(STM32F410Rx)
#  include <arch/cm3/stm/vendor/stm32f410rx.h>
# elif defined(STM32F410Tx)
#  include <arch/cm3/stm/vendor/stm32f410tx.h>
# elif defined(STM32F411xE)
#  include <arch/cm3/stm/vendor/stm32f411xe.h>
# elif defined(STM32F412Cx)
#  include <arch/cm3/stm/vendor/stm32f412cx.h>
# elif defined(STM32F412Rx)
#  include <arch/cm3/stm/vendor/stm32f412rx.h>
# elif defined(STM32F412Vx)
#  include <arch/cm3/stm/vendor/stm32f412vx.h>
# elif defined(STM32F412Zx)
#  include <arch/cm3/stm/vendor/stm32f412zx.h>
# elif defined(STM32F413xx)
#  include <arch/cm3/stm/vendor/stm32f413xx.h>
# elif defined(STM32F415xx)
#  include <arch/cm3/stm/vendor/stm32f415xx.h>
# elif defined(STM32F417xx)
#  include <arch/cm3/stm/vendor/stm32f417xx.h>
# elif defined(STM32F427xx)
#  include <arch/cm3/stm/vendor/stm32f427xx.h>
# elif defined(STM32F429xx)
#  include <arch/cm3/stm/vendor/stm32f429xx.h>
# elif defined(STM32F437xx)
#  include <arch/cm3/stm/vendor/stm32f437xx.h>
# elif defined(STM32F439xx)
#  include <arch/cm3/stm/vendor/stm32f439xx.h>
# elif defined(STM32F446xx)
#  include <arch/cm3/stm/vendor/stm32f446xx.h>
# elif defined(STM32F469xx)
#  include <arch/cm3/stm/vendor/stm32f469xx.h>
# elif defined(STM32F479xx)
#  include <arch/cm3/stm/vendor/stm32f479xx.h>
# else
#  warning "Unknown STM32F4 family"
# endif
# if !defined(RCC_CSR_BORRSTF)
/* This definition is missing for F412 devices in F4 headers up
 * to at least STM32Cube_FW_F4_V1.14.0. */
#  define RCC_CSR_BORRSTF 0x02000000
# endif
#elif defined(MCU_STM32F7)
# if   defined(STM32F722xx)
#  include <arch/cm3/stm/vendor/stm32f722xx.h>
# elif defined(STM32F723xx)
#  include <arch/cm3/stm/vendor/stm32f723xx.h>
# elif defined(STM32F732xx)
#  include <arch/cm3/stm/vendor/stm32f732xx.h>
# elif defined(STM32F733xx)
#  include <arch/cm3/stm/vendor/stm32f733xx.h>
# elif defined(STM32F745xx)
#  include <arch/cm3/stm/vendor/stm32f745xx.h>
# elif defined(STM32F746xx)
#  include <arch/cm3/stm/vendor/stm32f746xx.h>
# elif defined(STM32F756xx)
#  include <arch/cm3/stm/vendor/stm32f756xx.h>
# elif defined(STM32F765xx)
#  include <arch/cm3/stm/vendor/stm32f765xx.h>
# elif defined(STM32F767xx)
#  include <arch/cm3/stm/vendor/stm32f767xx.h>
# elif defined(STM32F769xx)
#  include <arch/cm3/stm/vendor/stm32f769xx.h>
# elif defined(STM32F777xx)
#  include <arch/cm3/stm/vendor/stm32f777xx.h>
# elif defined(STM32F779xx)
#  include <arch/cm3/stm/vendor/stm32f779xx.h>
# else
#  warning "Unknown STM32F7 family"
# endif
#else
#warning "Unknown STM32 family"
#endif

/* Equalize name changes in newer header versions.*/
#if !defined(FLASH_PECR_FIX) && defined(FLASH_PECR_FTDW)
# define FLASH_PECR_FIX FLASH_PECR_FTDW
#endif

/* Equalize names in a common place. Even recent CUBE uses "random" names.*/
#if defined(GPIO_BRR_BR0) && !defined(GPIO_BRR_BR_0) && !defined(MCU_STM32F4)
# define GPIO_BRR_BR_0 GPIO_BRR_BR0
#endif

#if defined(SRAM1_BASE) && !defined(SRAM_BASE)
#define SRAM_BASE SRAM1_BASE
#endif

#if defined(RCC_APB1ENR_CANEN) && !defined(RCC_APB1ENR_CAN1EN)
#define RCC_APB1ENR_CAN1EN RCC_APB1ENR_CANEN
#endif
#if defined(RCC_APB1RSTR_CANRST) && !defined(RCC_APB1RSTR_CAN1RST)
#define RCC_APB1RSTR_CAN1RST RCC_APB1RSTR_CANRST
#endif
#if defined(CAN) && !defined(CAN1)
#define CAN1 CAN
#endif
#if defined(CAN_BASE) && !defined(CAN1_BASE)
#define CAN1_BASE CAN_BASE
#endif

#if defined(RCC_APB1ENR_DACEN) && !defined(RCC_APB1ENR_DAC1EN)
#define RCC_APB1ENR_DAC1EN RCC_APB1ENR_DACEN
#endif

# if !defined(PWR_CR_DBP) && defined (PWR_CR1_DBP)
#  define PWR_CR_DBP   PWR_CR1_DBP
#  define PWR_CR_VOS   PWR_CR1_VOS
#  define PWR_CR_VOS_0 PWR_CR1_VOS_0
#  define PWR_CR (PWR->CR1)
# else
#  define PWR_CR (PWR->CR)
# endif

# if !defined(RCC_CIFR_LSERDYF) && defined(RCC_CIR_LSERDYF)
#  define RCC_CIFR (RCC->CIR)
#  define RCC_CIER (RCC->CIR)
#  define RCC_CICR (RCC->CIR)
#  define RCC_CIFR_LSERDYF   RCC_CIR_LSERDYF
#  define RCC_CIER_LSERDYIE  RCC_CIR_LSERDYIE
#  define RCC_CICR_LSERDYC   RCC_CIR_LSERDYC
# else
#  define RCC_CIFR (RCC->CIFR)
#  define RCC_CIER (RCC->CIER)
#  define RCC_CICR (RCC->CICR)
#endif

# if !defined(RCC_BDCR_LSEBYP) && defined(RCC_CSR_LSEBYP)
/* L0 and  L1 */
#  define RCC_BDCR (RCC->CSR)
#  define RCC_BDCR_LSEBYP RCC_CSR_LSEBYP
#  define RCC_BDCR_LSERDY RCC_CSR_LSERDY
#  define RCC_BDCR_RTCEN  RCC_CSR_RTCEN
#  define RCC_BDCR_RTCSEL_0  RCC_CSR_RTCSEL_0
#  define RCC_BDCR_RTCSEL_1  RCC_CSR_RTCSEL_1
#  define RCC_BDCR_RTCSEL    RCC_CSR_RTCSEL
#  define RCC_BDCR_BDRST     RCC_CSR_RTCRST
#  define RCC_BDCR_LSIRDY    RCC_CSR_LSIRDY
#  define RCC_BDCR_LSION     RCC_CSR_LSION
#  if defined(RCC_CSR_LSEDRV)
/* L0 */
#   define RCC_BDCR_LSEDRV_0 RCC_CSR_LSEDRV_0
#   define RCC_BDCR_LSEDRV   RCC_CSR_LSEDRV
#  else
/* L1 */
#   define RCC_BDCR_LSEDRV_0 1
#   define RCC_BDCR_LSEDRV   0
#   define NO_LSEDRV
#  endif
# elif defined(RCC_BDCR_LSEBYP)
#  if   defined(RCC_BCDR_LSEMOD)
/* F411, F446, F469 and F479*/
#   define RCC_BDCR_LSEDRV_0 RCC_BDCR_LSEMOD
#   define RCC_BDCR_LSEDRV   RCC_BDCR_LSEMOD
#  elif !defined(RCC_BDCR_LSEDRV)
/* F1, F2 and older F4 */
#   define RCC_BDCR_LSEDRV_0 1
#   define RCC_BDCR_LSEDRV   0
#   define NO_LSEDRV
#  else
/* F0, F3, F7 and F4 */
#  endif
# endif
#if !defined(RCC_BDCR)
# define RCC_BDCR (RCC->BDCR)
#endif

# if !defined(RCC_BDCR_LSEON) && defined(RCC_CSR_LSEON)
#  define RCC_BDCR_LSEON RCC_CSR_LSEON
# endif

# if !defined(RCC_CR_RTCPRE_0) && defined(RCC_CFGR_RTCPRE_0)
#  define RCC_CR (RCC->CFGR)
# else
#  define RCC_CR (RCC->CR)
# endif

# if !defined(FLASH_SR_PGSERR) && defined(FLASH_SR_ERSERR)
#  define FLASH_SR_PGSERR FLASH_SR_ERSERR
# endif

#if defined(MCU_STM32L4)
# define APB1ENR APB1ENR1
# define RCC_APB1ENR_TIM2EN   RCC_APB1ENR1_TIM2EN
# define RCC_APB1ENR_TIM3EN   RCC_APB1ENR1_TIM3EN
# define RCC_APB1ENR_TIM4EN   RCC_APB1ENR1_TIM4EN
# define RCC_APB1ENR_TIM5EN   RCC_APB1ENR1_TIM5EN
# define RCC_APB1ENR_TIM6EN   RCC_APB1ENR1_TIM6EN
# define RCC_APB1ENR_TIM7EN   RCC_APB1ENR1_TIM7EN
# define RCC_APB1ENR_SPI2EN   RCC_APB1ENR1_SPI2EN
# define RCC_APB1ENR_SPI3EN   RCC_APB1ENR1_SPI3EN
# define RCC_APB1ENR_USART2EN RCC_APB1ENR1_USART2EN
# define RCC_APB1ENR_USART3EN RCC_APB1ENR1_USART3EN
# define RCC_APB1ENR_UART4EN  RCC_APB1ENR1_UART4EN
# define RCC_APB1ENR_UART5EN  RCC_APB1ENR1_UART5EN
# define RCC_APB1ENR_I2C1EN   RCC_APB1ENR1_I2C1EN
# define RCC_APB1ENR_I2C2EN   RCC_APB1ENR1_I2C2EN
# define RCC_APB1ENR_I2C3EN   RCC_APB1ENR1_I2C3EN
# define RCC_APB1ENR_CAN1EN   RCC_APB1ENR1_CAN1EN
# define RCC_APB1ENR_PWREN    RCC_APB1ENR1_PWREN
# define RCC_APB1ENR_DAC1EN   RCC_APB1ENR1_DAC1EN
# define RCC_APB1ENR_LCDEN    RCC_APB1ENR1_LCDEN

# define APB1RSTR             APB1RSTR1
# define RCC_APB1RSTR_TIM2RST   RCC_APB1RSTR1_TIM2RST
# define RCC_APB1RSTR_TIM3RST   RCC_APB1RSTR1_TIM3RST
# define RCC_APB1RSTR_TIM4RST   RCC_APB1RSTR1_TIM4RST
# define RCC_APB1RSTR_TIM5RST   RCC_APB1RSTR1_TIM5RST
# define RCC_APB1RSTR_TIM6RST   RCC_APB1RSTR1_TIM6RST
# define RCC_APB1RSTR_TIM7RST   RCC_APB1RSTR1_TIM7RST
# define RCC_APB1RSTR_SPI2RST   RCC_APB1RSTR1_SPI2RST
# define RCC_APB1RSTR_SPI3RST   RCC_APB1RSTR1_SPI3RST
# define RCC_APB1RSTR_USART2RST RCC_APB1RSTR1_USART2RST
# define RCC_APB1RSTR_USART3RST RCC_APB1RSTR1_USART3RST
# define RCC_APB1RSTR_UART4RST  RCC_APB1RSTR1_UART4RST
# define RCC_APB1RSTR_UART5RST  RCC_APB1RSTR1_UART5RST
# define RCC_APB1RSTR_I2C1RST   RCC_APB1RSTR1_I2C1RST
# define RCC_APB1RSTR_I2C2RST   RCC_APB1RSTR1_I2C2RST
# define RCC_APB1RSTR_I2C3RST   RCC_APB1RSTR1_I2C3RST
# define RCC_APB1RSTR_CAN1RST   RCC_APB1RSTR1_CAN1RST
# define RCC_APB1RSTR_PWRRST    RCC_APB1RSTR1_PWRRST
# define RCC_APB1RSTR_DAC1RST   RCC_APB1RSTR1_DAC1RST

# define AHBENR AHB1ENR
# define RCC_AHBENR_DMA1EN RCC_AHB1ENR_DMA1EN
# define RCC_AHBENR_DMA2EN RCC_AHB1ENR_DMA2EN
#endif

#if defined(EXTI_PR1_PIF0) && !defined(EXTI_PR_PR0)
# define EXTI_PR   (EXTI->PR1)
# define EXTI_IMR  (EXTI->IMR1)
# define EXTI_FTSR (EXTI->FTSR1)
# define EXTI_RTSR (EXTI->RTSR1)
#else
# define EXTI_PR   (EXTI->PR)
# define EXTI_IMR  (EXTI->IMR)
# define EXTI_FTSR (EXTI->FTSR)
# define EXTI_RTSR (EXTI->RTSR)
#endif

extern int Stm32ResetCause(void);

/* Allow readable values in the configurator, but avoid
 * name clash when STM libraries are used too.
 */
#ifndef USE_STD_PERIPHERAL_DRIVERS
# define ENABLE  1
# define DISABLE 0
#endif

#endif
