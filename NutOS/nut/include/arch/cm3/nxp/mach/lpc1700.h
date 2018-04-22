#ifndef _ARCH_CM3_NXP_MACH_LPC1700_H_
#define _ARCH_CM3_NXP_MACH_LPC1700_H_

/*
 * Copyright 2011 by egnite GmbH
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
 * \file arch/cm3/nxp/mach/lpc1700.h
 * \brief LPC1700 peripherals
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#define PIN_CFGREG(p, b)    ((p) * 2 + (b) / 16)
#define PIN_CFGLSB(b)       (((b) % 16) * 2)

#define PINSEL_REG(p, b)    PINSEL(PIN_CFGREG(p, b))

#define P0_PINSEL(b)        PINSEL((b) / 16)
#define P1_PINSEL(b)        PINSEL((b) / 16 + 2)
#define P2_PINSEL(b)        PINSEL((b) / 16 + 4)
#define P3_PINSEL(b)        PINSEL((b) / 16 + 6)

#define P0_PINMODE(b)   PINMODE((b) / 16)
#define P1_PINMODE(b)   PINMODE((b) / 16 + 2)
#define P2_PINMODE(b)   PINMODE((b) / 16 + 4)
#define P3_PINMODE(b)   PINMODE((b) / 16 + 6)

#define P0_PINMODE_OD(b)   PINMODE_OD((b) / 16)
#define P1_PINMODE_OD(b)   PINMODE_OD((b) / 16 + 2)
#define P2_PINMODE_OD(b)   PINMODE_OD((b) / 16 + 4)
#define P3_PINMODE_OD(b)   PINMODE_OD((b) / 16 + 6)

#define PIN_CFGMSK(b)       (3 << PIN_CFGLSB(b))

#define PS0_P0_0_MSK    (3 << 0)
#define PS0_P0_1_MSK    (3 << 2)
#define PS0_P0_2_MSK    (3 << 4)
#define PS0_P0_3_MSK    (3 << 6)
#define PS0_P0_4_MSK    (3 << 8)
#define PS0_P0_5_MSK    (3 << 10)
#define PS0_P0_6_MSK    (3 << 12)
#define PS0_P0_7_MSK    (3 << 14)
#define PS0_P0_8_MSK    (3 << 16)
#define PS0_P0_9_MSK    (3 << 18)
#define PS0_P0_10_MSK   (3 << 20)
#define PS0_P0_11_MSK   (3 << 22)
#define PS0_P0_12_MSK   (3 << 24)
#define PS0_P0_13_MSK   (3 << 26)
#define PS0_P0_14_MSK   (3 << 28)
#define PS0_P0_15_MSK   (3 << 30)
#define PS1_P0_16_MSK   (3 << 0)
#define PS1_P0_17_MSK   (3 << 2)
#define PS1_P0_18_MSK   (3 << 4)
#define PS1_P0_19_MSK   (3 << 6)
#define PS1_P0_20_MSK   (3 << 8)
#define PS1_P0_21_MSK   (3 << 10)
#define PS1_P0_22_MSK   (3 << 12)
#define PS1_P0_23_MSK   (3 << 14)
#define PS1_P0_24_MSK   (3 << 16)
#define PS1_P0_25_MSK   (3 << 18)
#define PS1_P0_26_MSK   (3 << 20)
#define PS1_P0_27_MSK   (3 << 22)
#define PS1_P0_28_MSK   (3 << 24)
#define PS1_P0_29_MSK   (3 << 26)
#define PS1_P0_30_MSK   (3 << 28)
#define PS1_P0_31_MSK   (3 << 30)

#define PS2_P1_0_MSK    (3 << 0)
#define PS2_P1_1_MSK    (3 << 2)
#define PS2_P1_2_MSK    (3 << 4)
#define PS2_P1_3_MSK    (3 << 6)
#define PS2_P1_4_MSK    (3 << 8)
#define PS2_P1_5_MSK    (3 << 10)
#define PS2_P1_6_MSK    (3 << 12)
#define PS2_P1_7_MSK    (3 << 14)
#define PS2_P1_8_MSK    (3 << 16)
#define PS2_P1_9_MSK    (3 << 18)
#define PS2_P1_10_MSK   (3 << 20)
#define PS2_P1_11_MSK   (3 << 22)
#define PS2_P1_12_MSK   (3 << 24)
#define PS2_P1_13_MSK   (3 << 26)
#define PS2_P1_14_MSK   (3 << 28)
#define PS2_P1_15_MSK   (3 << 30)
#define PS3_P1_16_MSK   (3 << 0)
#define PS3_P1_17_MSK   (3 << 2)
#define PS3_P1_18_MSK   (3 << 4)
#define PS3_P1_19_MSK   (3 << 6)
#define PS3_P1_20_MSK   (3 << 8)
#define PS3_P1_21_MSK   (3 << 10)
#define PS3_P1_22_MSK   (3 << 12)
#define PS3_P1_23_MSK   (3 << 14)
#define PS3_P1_24_MSK   (3 << 16)
#define PS3_P1_25_MSK   (3 << 18)
#define PS3_P1_26_MSK   (3 << 20)
#define PS3_P1_27_MSK   (3 << 22)
#define PS3_P1_28_MSK   (3 << 24)
#define PS3_P1_29_MSK   (3 << 26)
#define PS3_P1_30_MSK   (3 << 28)
#define PS3_P1_31_MSK   (3 << 30)

#define PS4_P2_0_MSK    (3 << 0)
#define PS4_P2_1_MSK    (3 << 2)
#define PS4_P2_2_MSK    (3 << 4)
#define PS4_P2_3_MSK    (3 << 6)
#define PS4_P2_4_MSK    (3 << 8)
#define PS4_P2_5_MSK    (3 << 10)
#define PS4_P2_6_MSK    (3 << 12)
#define PS4_P2_7_MSK    (3 << 14)
#define PS4_P2_8_MSK    (3 << 16)
#define PS4_P2_9_MSK    (3 << 18)
#define PS4_P2_10_MSK   (3 << 20)
#define PS4_P2_11_MSK   (3 << 22)
#define PS4_P2_12_MSK   (3 << 24)
#define PS4_P2_13_MSK   (3 << 26)
#define PS4_P2_14_MSK   (3 << 28)
#define PS4_P2_15_MSK   (3 << 30)
#define PS5_P2_16_MSK   (3 << 0)
#define PS5_P2_17_MSK   (3 << 2)
#define PS5_P2_18_MSK   (3 << 4)
#define PS5_P2_19_MSK   (3 << 6)
#define PS5_P2_20_MSK   (3 << 8)
#define PS5_P2_21_MSK   (3 << 10)
#define PS5_P2_22_MSK   (3 << 12)
#define PS5_P2_23_MSK   (3 << 14)
#define PS5_P2_24_MSK   (3 << 16)
#define PS5_P2_25_MSK   (3 << 18)
#define PS5_P2_26_MSK   (3 << 20)
#define PS5_P2_27_MSK   (3 << 22)
#define PS5_P2_28_MSK   (3 << 24)
#define PS5_P2_29_MSK   (3 << 26)
#define PS5_P2_30_MSK   (3 << 28)
#define PS5_P2_31_MSK   (3 << 30)

#define PS6_P3_0_MSK    (3 << 0)
#define PS6_P3_1_MSK    (3 << 2)
#define PS6_P3_2_MSK    (3 << 4)
#define PS6_P3_3_MSK    (3 << 6)
#define PS6_P3_4_MSK    (3 << 8)
#define PS6_P3_5_MSK    (3 << 10)
#define PS6_P3_6_MSK    (3 << 12)
#define PS6_P3_7_MSK    (3 << 14)
#define PS6_P3_8_MSK    (3 << 16)
#define PS6_P3_9_MSK    (3 << 18)
#define PS6_P3_10_MSK   (3 << 20)
#define PS6_P3_11_MSK   (3 << 22)
#define PS6_P3_12_MSK   (3 << 24)
#define PS6_P3_13_MSK   (3 << 26)
#define PS6_P3_14_MSK   (3 << 28)
#define PS6_P3_15_MSK   (3 << 30)
#define PS7_P3_16_MSK   (3 << 0)
#define PS7_P3_17_MSK   (3 << 2)
#define PS7_P3_18_MSK   (3 << 4)
#define PS7_P3_19_MSK   (3 << 6)
#define PS7_P3_20_MSK   (3 << 8)
#define PS7_P3_21_MSK   (3 << 10)
#define PS7_P3_22_MSK   (3 << 12)
#define PS7_P3_23_MSK   (3 << 14)
#define PS7_P3_24_MSK   (3 << 16)
#define PS7_P3_25_MSK   (3 << 18)
#define PS7_P3_26_MSK   (3 << 20)
#define PS7_P3_27_MSK   (3 << 22)
#define PS7_P3_28_MSK   (3 << 24)
#define PS7_P3_29_MSK   (3 << 26)
#define PS7_P3_30_MSK   (3 << 28)
#define PS7_P3_31_MSK   (3 << 30)

#define PS8_P4_0_MSK    (3 << 0)
#define PS8_P4_1_MSK    (3 << 2)
#define PS8_P4_2_MSK    (3 << 4)
#define PS8_P4_3_MSK    (3 << 6)
#define PS8_P4_4_MSK    (3 << 8)
#define PS8_P4_5_MSK    (3 << 10)
#define PS8_P4_6_MSK    (3 << 12)
#define PS8_P4_7_MSK    (3 << 14)
#define PS8_P4_8_MSK    (3 << 16)
#define PS8_P4_9_MSK    (3 << 18)
#define PS8_P4_10_MSK   (3 << 20)
#define PS8_P4_11_MSK   (3 << 22)
#define PS8_P4_12_MSK   (3 << 24)
#define PS8_P4_13_MSK   (3 << 26)
#define PS8_P4_14_MSK   (3 << 28)
#define PS8_P4_15_MSK   (3 << 30)
#define PS9_P4_16_MSK   (3 << 0)
#define PS9_P4_17_MSK   (3 << 2)
#define PS9_P4_18_MSK   (3 << 4)
#define PS9_P4_19_MSK   (3 << 6)
#define PS9_P4_20_MSK   (3 << 8)
#define PS9_P4_21_MSK   (3 << 10)
#define PS9_P4_22_MSK   (3 << 12)
#define PS9_P4_23_MSK   (3 << 14)
#define PS9_P4_24_MSK   (3 << 16)
#define PS9_P4_25_MSK   (3 << 18)
#define PS9_P4_26_MSK   (3 << 20)
#define PS9_P4_27_MSK   (3 << 22)
#define PS9_P4_28_MSK   (3 << 24)
#define PS9_P4_29_MSK   (3 << 26)
#define PS9_P4_30_MSK   (3 << 28)
#define PS9_P4_31_MSK   (3 << 30)

#if defined (MCU_LPC1768)
#include <arch/cm3/nxp/mach/lpc1768.h>
#endif

#endif
