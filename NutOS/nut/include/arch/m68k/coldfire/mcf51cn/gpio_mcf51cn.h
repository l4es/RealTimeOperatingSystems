/*
 * Copyright 2012 by Embedded Technologies s.r.o
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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTEON) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

#ifndef _DEV_GPIO_H_
#error "Do not include this file directly. Use dev/gpio.h instead!"
#endif

#include <stdint.h>

/*
 * GPIO Port Definitions
 */
#define PORTA       0
#define PORTB       1
#define PORTC       2
#define PORTD       3
#define PORTE       4
#define PORTF       5
#define PORTG       6
#define PORTH       7
#define PORTJ       8

/*
 * GPIO PortPin Definitions
 */
#define PTA0        0
#define PTA1        1
#define PTA2        2
#define PTA3        3
#define PTA4        4
#define PTA5        5
#define PTA6        6
#define PTA7        7

#define PTB0       10
#define PTB1       11
#define PTB2       12
#define PTB3       13
#define PTB4       14
#define PTB5       15
#define PTB6       16
#define PTB7       17

#define PTC0       20
#define PTC1       21
#define PTC2       22
#define PTC3       23
#define PTC4       24
#define PTC5       25
#define PTC6       26
#define PTC7       27

#define PTD0       30
#define PTD1       31
#define PTD2       32
#define PTD3       33
#define PTD4       34
#define PTD5       35
#define PTD6       36
#define PTD7       37

#define PTE0       40
#define PTE1       41
#define PTE2       42
#define PTE3       43
#define PTE4       44
#define PTE5       45
#define PTE6       46
#define PTE7       47

#define PTF0       60
#define PTF1       61
#define PTF2       62
#define PTF3       63
#define PTF4       64
#define PTF5       65
#define PTF6       66
#define PTF7       67

#define PTG0       80
#define PTG1       81
#define PTG2       82
#define PTG3       83
#define PTG4       84
#define PTG5       85
#define PTG6       86
#define PTG7       87

#define PTH0      100
#define PTH1      101
#define PTH2      102
#define PTH3      103
#define PTH4      104
#define PTH5      105
#define PTH6      106
#define PTH7      107

#define PTJ0      110
#define PTJ1      111
#define PTJ2      112
#define PTJ3      113
#define PTJ4      114
#define PTJ5      115
#define PTJ6      116
#define PTJ7      117

/*
 * GPIO PortPins Initialization
 */
#include "gpio_mcf51cn_iic1.h"
#include "gpio_mcf51cn_iic2.h"
#include "gpio_mcf51cn_sci1.h"
#include "gpio_mcf51cn_sci2.h"
#include "gpio_mcf51cn_sci3.h"

/*
 * GPIO API
 */
#define GPIO_CFG_INPUT              0x00000000
#define GPIO_CFG_ALT1               0x00000001
#define GPIO_CFG_ALT2               0x00000002
#define GPIO_CFG_ALT3               0x00000003
#define GPIO_CFG_OUTPUT             0x00000004
#define GPIO_CFG_PULLUP             0x00000010
#define GPIO_CFG_SLEW_RATE          0x00000020
#define GPIO_CFG_DRIVE_STRENGTH     0x00000040
#define GPIO_CFG_INPUT_FILTER       0x00000080
#define GPIO_CFG_DEBOUNCE           0   // not supported


#define GpioPinGet(bank, bit)           ((MCF_GPIO_D(bank) >> bit) & 0x1)
#define GpioPinSet(bank, bit, value)    (value) ? (GpioPinSetHigh(bank, bit)) : (GpioPinSetLow(bank, bit))
#define GpioPinSetHigh(bank, bit)       MCF_GPIO_D(bank) |= _BV(bit)
#define GpioPinSetLow(bank, bit)        MCF_GPIO_D(bank) &= ~_BV(bit)

#define GpioPortGet(bank)               MCF_GPIO_D(bank)
#define GpioPortSet(bank, value)        MCF_GPIO_D(bank) = (value)
#define GpioPortSetHigh(bank, mask)     MCF_GPIO_D(bank) |= (mask)
#define GpioPortSetLow(bank, mask)      MCF_GPIO_D(bank) &= ~(mask)

extern uint32_t GpioPinConfigGet(int bank, int bit);
extern int GpioPinConfigSet(int bank, int bit, uint32_t flags);
extern int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags);
