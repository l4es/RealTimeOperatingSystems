#ifndef _DEV_SBBIF0_H_
#define _DEV_SBBIF0_H_
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
 * \file dev/sbbif0.h
 * \brief Serial bit banged interface 0.
 *
 * \verbatim
 *
 * $Log$
 * Revision 1.5  2009/01/18 16:44:56  haraldkipp
 * Introduced target independent port numbers.
 * Distinguish between PIO IDs and port numbers.
 *
 * Revision 1.4  2009/01/18 10:45:54  haraldkipp
 * Fixed bit banging SPI for ARM targets.
 *
 * Revision 1.3  2008/08/28 16:11:31  haraldkipp
 * Fixed bitbanging SPI for ARM targets.
 *
 * Revision 1.2  2008/08/11 06:59:59  haraldkipp
 * BSD types replaced by stdint types (feature request #1282721).
 *
 * Revision 1.1  2007/04/12 09:07:54  haraldkipp
 * Configurable SPI added.
 *
 *
 * \endverbatim
 */

#include <cfg/arch/gpio.h>
#include <stdint.h>

/*!
 * \brief Maximum number of devices (chip selects).
 */
#ifndef SBBI0_MAX_DEVICES
#define SBBI0_MAX_DEVICES   4
#endif

#ifdef SBBI0_CS0_BIT
#undef GPIO_ID
#if defined(SBBI0_CS0_PORT)
#define GPIO_ID SBBI0_CS0_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_CS0_ENA() GPIO_ENABLE(SBBI0_CS0_BIT)
#define SBBI0_CS0_OUTPUT() GPIO_OUTPUT(SBBI0_CS0_BIT)
#define SBBI0_CS0_SET() GPIO_SET_HI(SBBI0_CS0_BIT)
#define SBBI0_CS0_CLR() GPIO_SET_LO(SBBI0_CS0_BIT)
#else
#define SBBI0_CS0_ENA()
#define SBBI0_CS0_OUTPUT()
#define SBBI0_CS0_CLR()
#define SBBI0_CS0_SET()
#endif

#ifdef SBBI0_CS1_BIT
#undef GPIO_ID
#if defined(SBBI0_CS1_PORT)
#define GPIO_ID SBBI0_CS1_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_CS1_ENA() GPIO_ENABLE(SBBI0_CS1_BIT)
#define SBBI0_CS1_OUTPUT() GPIO_SET_HI(SBBI0_CS1_BIT)
#define SBBI0_CS1_SET() GPIO_SET_HI(SBBI0_CS1_BIT)
#define SBBI0_CS1_CLR() GPIO_SET_LO(SBBI0_CS1_BIT)
#else
#define SBBI0_CS1_ENA()
#define SBBI0_CS1_OUTPUT()
#define SBBI0_CS1_CLR()
#define SBBI0_CS1_SET()
#endif

#ifdef SBBI0_CS2_BIT
#undef GPIO_ID
#if defined(SBBI0_CS2_PORT)
#define GPIO_ID SBBI0_CS2_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_CS2_ENA() GPIO_ENABLE(SBBI0_CS2_BIT)
#define SBBI0_CS2_OUTPUT() GPIO_OUTPUT(SBBI0_CS2_BIT)
#define SBBI0_CS2_SET() GPIO_SET_HI(SBBI0_CS2_BIT)
#define SBBI0_CS2_CLR() GPIO_SET_LO(SBBI0_CS2_BIT)
#else
#define SBBI0_CS2_ENA()
#define SBBI0_CS2_OUTPUT()
#define SBBI0_CS2_CLR()
#define SBBI0_CS2_SET()
#endif

#ifdef SBBI0_CS3_BIT
#undef GPIO_ID
#if defined(SBBI0_CS3_PORT)
#define GPIO_ID SBBI0_CS3_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_CS3_ENA() GPIO_ENABLE(SBBI0_CS3_BIT)
#define SBBI0_CS3_OUTPUT() GPIO_OUTPUT(SBBI0_CS3_BIT)
#define SBBI0_CS3_SET() GPIO_SET_HI(SBBI0_CS3_BIT)
#define SBBI0_CS3_CLR() GPIO_SET_LO(SBBI0_CS3_BIT)
#else
#define SBBI0_CS3_ENA()
#define SBBI0_CS3_OUTPUT()
#define SBBI0_CS3_CLR()
#define SBBI0_CS3_SET()
#endif

#ifdef SBBI0_RST0_BIT
#undef GPIO_ID
#if defined(SBBI0_RST0_PORT)
#define GPIO_ID SBBI0_RST0_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_RST0_ENA() GPIO_ENABLE(SBBI0_RST0_BIT)
#define SBBI0_RST0_OUTPUT() GPIO_OUTPUT(SBBI0_RST0_BIT)
#define SBBI0_RST0_SET() GPIO_SET_HI(SBBI0_RST0_BIT)
#define SBBI0_RST0_CLR() GPIO_SET_LO(SBBI0_RST0_BIT)
#else
#define SBBI0_RST0_ENA()
#define SBBI0_RST0_OUTPUT()
#define SBBI0_RST0_CLR()
#define SBBI0_RST0_SET()
#endif

#ifdef SBBI0_RST1_BIT
#undef GPIO_ID
#if defined(SBBI0_RST1_PORT)
#define GPIO_ID SBBI0_RST1_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_RST1_ENA() GPIO_ENABLE(SBBI0_RST1_BIT)
#define SBBI0_RST1_OUTPUT() GPIO_OUTPUT(SBBI0_RST1_BIT)
#define SBBI0_RST1_SET() GPIO_SET_HI(SBBI0_RST1_BIT)
#define SBBI0_RST1_CLR() GPIO_SET_LO(SBBI0_RST1_BIT)
#else
#define SBBI0_RST1_ENA()
#define SBBI0_RST1_OUTPUT()
#define SBBI0_RST1_CLR()
#define SBBI0_RST1_SET()
#endif

#ifdef SBBI0_RST2_BIT
#undef GPIO_ID
#if defined(SBBI0_RST2_PORT)
#define GPIO_ID SBBI0_RST2_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_RST2_ENA() GPIO_ENABLE(SBBI0_RST2_BIT)
#define SBBI0_RST2_OUTPUT() GPIO_OUTPUT(SBBI0_RST2_BIT)
#define SBBI0_RST2_SET() GPIO_SET_HI(SBBI0_RST2_BIT)
#define SBBI0_RST2_CLR() GPIO_SET_LO(SBBI0_RST2_BIT)
#else
#define SBBI0_RST2_ENA()
#define SBBI0_RST2_OUTPUT()
#define SBBI0_RST2_CLR()
#define SBBI0_RST2_SET()
#endif

#ifdef SBBI0_RST3_BIT
#undef GPIO_ID
#if defined(SBBI0_RST3_PORT)
#define GPIO_ID SBBI0_RST3_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_RST3_ENA() GPIO_ENABLE(SBBI0_RST3_BIT)
#define SBBI0_RST3_OUTPUT() GPIO_OUTPUT(SBBI0_RST3_BIT)
#define SBBI0_RST3_SET() GPIO_SET_HI(SBBI0_RST3_BIT)
#define SBBI0_RST3_CLR() GPIO_SET_LO(SBBI0_RST3_BIT)
#else
#define SBBI0_RST3_ENA()
#define SBBI0_RST3_OUTPUT()
#define SBBI0_RST3_CLR()
#define SBBI0_RST3_SET()
#endif

#ifdef SBBI0_SCK_BIT
#undef GPIO_ID
#if defined(SBBI0_SCK_PORT)
#define GPIO_ID SBBI0_SCK_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_SCK_ENA() GPIO_ENABLE(SBBI0_SCK_BIT)
#define SBBI0_SCK_OUTPUT() GPIO_OUTPUT(SBBI0_SCK_BIT)
#define SBBI0_SCK_SET() GPIO_SET_HI(SBBI0_SCK_BIT)
#define SBBI0_SCK_CLR() GPIO_SET_LO(SBBI0_SCK_BIT)
#else
#define SBBI0_SCK_ENA()
#define SBBI0_SCK_OUTPUT()
#define SBBI0_SCK_CLR()
#define SBBI0_SCK_SET()
#endif

#ifdef SBBI0_MOSI_BIT
#undef GPIO_ID
#if defined(SBBI0_MOSI_PORT)
#define GPIO_ID SBBI0_MOSI_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_MOSI_ENA() GPIO_ENABLE(SBBI0_MOSI_BIT)
#define SBBI0_MOSI_OUTPUT() GPIO_OUTPUT(SBBI0_MOSI_BIT)
#define SBBI0_MOSI_SET() GPIO_SET_HI(SBBI0_MOSI_BIT)
#define SBBI0_MOSI_CLR() GPIO_SET_LO(SBBI0_MOSI_BIT)
#else
#define SBBI0_MOSI_ENA()
#define SBBI0_MOSI_OUTPUT()
#define SBBI0_MOSI_CLR()
#define SBBI0_MOSI_SET()
#endif

#ifdef SBBI0_MISO_BIT
#undef GPIO_ID
#if defined(SBBI0_MISO_PORT)
#define GPIO_ID SBBI0_MISO_PORT
#endif
#include <cfg/arch/porttran.h>
#define SBBI0_MISO_ENA() GPIO_ENABLE(SBBI0_MISO_BIT)
#define SBBI0_MISO_TST() GPIO_GET(SBBI0_MISO_BIT)
#else
#define SBBI0_MISO_ENA()
#define SBBI0_MISO_TST()   0
#endif

#define SBBI0_INIT() \
{ \
    SBBI0_SCK_CLR(); \
    SBBI0_SCK_ENA(); \
    SBBI0_MOSI_CLR(); \
    SBBI0_MOSI_ENA(); \
    SBBI0_MISO_ENA(); \
}

extern int Sbbi0SetMode(ureg_t ix, ureg_t mode);
extern void Sbbi0SetSpeed(ureg_t ix, uint32_t rate);
extern void Sbbi0Enable(ureg_t ix);
extern void Sbbi0ChipReset(ureg_t ix, ureg_t hi);
extern void Sbbi0ChipSelect(ureg_t ix, ureg_t hi);
extern void Sbbi0SelectDevice(ureg_t ix);
extern void Sbbi0DeselectDevice(ureg_t ix);
extern void Sbbi0NegSelectDevice(ureg_t ix);
extern void Sbbi0NegDeselectDevice(ureg_t ix);
extern uint8_t Sbbi0Byte(uint8_t data);
extern void Sbbi0Transact(const void *wdata, void *rdata, size_t len);

#endif
