#ifndef _DEV_SBBIF3_H_
#define _DEV_SBBIF3_H_
/*
 * Copyright (C) 2007 by egnite Software GmbH. All rights reserved.
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
 * \file dev/sbbif3.h
 * \brief Serial bit banged interface 3.
 *
 * \verbatim
 *
 * \endverbatim
 */

#include <cfg/arch/gpio.h>
#include <stdint.h>

/*!
 * \brief Maximum number of devices (chip selects).
 */
#ifndef SBBI3_MAX_DEVICES
#define SBBI3_MAX_DEVICES   4
#endif

#ifdef SBBI3_CS0_BIT
#undef GPIO_ID
#if defined(SBBI3_CS0_PORT)
#define GPIO_ID SBBI3_CS0_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_CS0_ENA() GPIO_ENABLE(SBBI3_CS0_BIT)
#define SBBI3_CS0_OUTPUT() GPIO_OUTPUT(SBBI3_CS0_BIT)
#define SBBI3_CS0_SET() GPIO_SET_HI(SBBI3_CS0_BIT)
#define SBBI3_CS0_CLR() GPIO_SET_LO(SBBI3_CS0_BIT)
#else
#define SBBI3_CS0_ENA()
#define SBBI3_CS0_OUTPUT()
#define SBBI3_CS0_CLR()
#define SBBI3_CS0_SET()
#endif

#ifdef SBBI3_CS1_BIT
#undef GPIO_ID
#if defined(SBBI3_CS1_PORT)
#define GPIO_ID SBBI3_CS1_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_CS1_ENA() GPIO_ENABLE(SBBI3_CS1_BIT)
#define SBBI3_CS1_OUTPUT() GPIO_SET_HI(SBBI3_CS1_BIT)
#define SBBI3_CS1_SET() GPIO_SET_HI(SBBI3_CS1_BIT)
#define SBBI3_CS1_CLR() GPIO_SET_LO(SBBI3_CS1_BIT)
#else
#define SBBI3_CS1_ENA()
#define SBBI3_CS1_OUTPUT()
#define SBBI3_CS1_CLR()
#define SBBI3_CS1_SET()
#endif

#ifdef SBBI3_CS2_BIT
#undef GPIO_ID
#if defined(SBBI3_CS2_PORT)
#define GPIO_ID SBBI3_CS2_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_CS2_ENA() GPIO_ENABLE(SBBI3_CS2_BIT)
#define SBBI3_CS2_OUTPUT() GPIO_OUTPUT(SBBI3_CS2_BIT)
#define SBBI3_CS2_SET() GPIO_SET_HI(SBBI3_CS2_BIT)
#define SBBI3_CS2_CLR() GPIO_SET_LO(SBBI3_CS2_BIT)
#else
#define SBBI3_CS2_ENA()
#define SBBI3_CS2_OUTPUT()
#define SBBI3_CS2_CLR()
#define SBBI3_CS2_SET()
#endif

#ifdef SBBI3_CS3_BIT
#undef GPIO_ID
#if defined(SBBI3_CS3_PORT)
#define GPIO_ID SBBI3_CS3_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_CS3_ENA() GPIO_ENABLE(SBBI3_CS3_BIT)
#define SBBI3_CS3_OUTPUT() GPIO_OUTPUT(SBBI3_CS3_BIT)
#define SBBI3_CS3_SET() GPIO_SET_HI(SBBI3_CS3_BIT)
#define SBBI3_CS3_CLR() GPIO_SET_LO(SBBI3_CS3_BIT)
#else
#define SBBI3_CS3_ENA()
#define SBBI3_CS3_OUTPUT()
#define SBBI3_CS3_CLR()
#define SBBI3_CS3_SET()
#endif

#ifdef SBBI3_RST0_BIT
#undef GPIO_ID
#if defined(SBBI3_RST0_PORT)
#define GPIO_ID SBBI3_RST0_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_RST0_ENA() GPIO_ENABLE(SBBI3_RST0_BIT)
#define SBBI3_RST0_OUTPUT() GPIO_OUTPUT(SBBI3_RST0_BIT)
#define SBBI3_RST0_SET() GPIO_SET_HI(SBBI3_RST0_BIT)
#define SBBI3_RST0_CLR() GPIO_SET_LO(SBBI3_RST0_BIT)
#else
#define SBBI3_RST0_ENA()
#define SBBI3_RST0_OUTPUT()
#define SBBI3_RST0_CLR()
#define SBBI3_RST0_SET()
#endif

#ifdef SBBI3_RST1_BIT
#undef GPIO_ID
#if defined(SBBI3_RST1_PORT)
#define GPIO_ID SBBI3_RST1_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_RST1_ENA() GPIO_ENABLE(SBBI3_RST1_BIT)
#define SBBI3_RST1_OUTPUT() GPIO_OUTPUT(SBBI3_RST1_BIT)
#define SBBI3_RST1_SET() GPIO_SET_HI(SBBI3_RST1_BIT)
#define SBBI3_RST1_CLR() GPIO_SET_LO(SBBI3_RST1_BIT)
#else
#define SBBI3_RST1_ENA()
#define SBBI3_RST1_OUTPUT()
#define SBBI3_RST1_CLR()
#define SBBI3_RST1_SET()
#endif

#ifdef SBBI3_RST2_BIT
#undef GPIO_ID
#if defined(SBBI3_RST2_PORT)
#define GPIO_ID SBBI3_RST2_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_RST2_ENA() GPIO_ENABLE(SBBI3_RST2_BIT)
#define SBBI3_RST2_OUTPUT() GPIO_OUTPUT(SBBI3_RST2_BIT)
#define SBBI3_RST2_SET() GPIO_SET_HI(SBBI3_RST2_BIT)
#define SBBI3_RST2_CLR() GPIO_SET_LO(SBBI3_RST2_BIT)
#else
#define SBBI3_RST2_ENA()
#define SBBI3_RST2_OUTPUT()
#define SBBI3_RST2_CLR()
#define SBBI3_RST2_SET()
#endif

#ifdef SBBI3_RST3_BIT
#undef GPIO_ID
#if defined(SBBI3_RST3_PORT)
#define GPIO_ID SBBI3_RST3_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_RST3_ENA() GPIO_ENABLE(SBBI3_RST3_BIT)
#define SBBI3_RST3_OUTPUT() GPIO_OUTPUT(SBBI3_RST3_BIT)
#define SBBI3_RST3_SET() GPIO_SET_HI(SBBI3_RST3_BIT)
#define SBBI3_RST3_CLR() GPIO_SET_LO(SBBI3_RST3_BIT)
#else
#define SBBI3_RST3_ENA()
#define SBBI3_RST3_OUTPUT()
#define SBBI3_RST3_CLR()
#define SBBI3_RST3_SET()
#endif

#ifdef SBBI3_SCK_BIT
#undef GPIO_ID
#if defined(SBBI3_SCK_PORT)
#define GPIO_ID SBBI3_SCK_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_SCK_ENA() GPIO_ENABLE(SBBI3_SCK_BIT)
#define SBBI3_SCK_OUTPUT() GPIO_OUTPUT(SBBI3_SCK_BIT)
#define SBBI3_SCK_SET() GPIO_SET_HI(SBBI3_SCK_BIT)
#define SBBI3_SCK_CLR() GPIO_SET_LO(SBBI3_SCK_BIT)
#else
#define SBBI3_SCK_ENA()
#define SBBI3_SCK_OUTPUT()
#define SBBI3_SCK_CLR()
#define SBBI3_SCK_SET()
#endif

#ifdef SBBI3_MOSI_BIT
#undef GPIO_ID
#if defined(SBBI3_MOSI_PORT)
#define GPIO_ID SBBI3_MOSI_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_MOSI_ENA() GPIO_ENABLE(SBBI3_MOSI_BIT)
#define SBBI3_MOSI_OUTPUT() GPIO_OUTPUT(SBBI3_MOSI_BIT)
#define SBBI3_MOSI_SET() GPIO_SET_HI(SBBI3_MOSI_BIT)
#define SBBI3_MOSI_CLR() GPIO_SET_LO(SBBI3_MOSI_BIT)
#else
#define SBBI3_MOSI_ENA()
#define SBBI3_MOSI_OUTPUT()
#define SBBI3_MOSI_CLR()
#define SBBI3_MOSI_SET()
#endif

#ifdef SBBI3_MISO_BIT
#undef GPIO_ID
#if defined(SBBI3_MISO_PORT)
#define GPIO_ID SBBI3_MISO_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI3_MISO_ENA() GPIO_ENABLE(SBBI3_MISO_BIT)
#define SBBI3_MISO_TST() GPIO_GET(SBBI3_MISO_BIT)
#else
#define SBBI3_MISO_ENA()
#define SBBI3_MISO_TST()   0
#endif

#define SBBI3_INIT() \
{ \
    SBBI3_SCK_CLR(); \
    SBBI3_SCK_ENA(); \
    SBBI3_MOSI_CLR(); \
    SBBI3_MOSI_ENA(); \
    SBBI3_MISO_ENA(); \
}

#endif
