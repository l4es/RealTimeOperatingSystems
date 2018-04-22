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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
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

/*
 * GPIO PortPins Initialization
 */
#include "gpio_mcf5225x_i2c0.h"
#include "gpio_mcf5225x_i2c1.h"
#include "gpio_mcf5225x_uart0.h"
#include "gpio_mcf5225x_uart1.h"
#include "gpio_mcf5225x_uart2.h"

/*
 * GPIO Port Definitions
 */
#define PORTTE       0
#define PORTTF       1
#define PORTTG       2
#define PORTTH       3
#define PORTTI       4
#define PORTTJ       6
#define PORTNQ       8
#define PORTAN      10
#define PORTAS      11
#define PORTQS      12
#define PORTTA      14
#define PORTTC      15
#define PORTUA      17
#define PORTUB      18
#define PORTUC      19
#define PORTDD      20

/*
 * GPIO PortPin Definitions
 */
#define PTE0        0
#define PTE1        1
#define PTE2        2
#define PTE3        3
#define PTE4        4
#define PTE5        5
#define PTE6        6
#define PTE7        7

#define PTF0       10
#define PTF1       11
#define PTF2       12
#define PTF3       13
#define PTF4       14
#define PTF5       15
#define PTF6       16
#define PTF7       17

#define PTG0       20
#define PTG1       21
#define PTG2       22
#define PTG3       23
#define PTG4       24
#define PTG5       25
#define PTG6       26
#define PTG7       27

#define PTH0       30
#define PTH1       31
#define PTH2       32
#define PTH3       33
#define PTH4       34
#define PTH5       35
#define PTH6       36
#define PTH7       37

#define PTI0       40
#define PTI1       41
#define PTI2       42
#define PTI3       43
#define PTI4       44
#define PTI5       45
#define PTI6       46
#define PTI7       47

#define PTJ0       60
#define PTJ1       61
#define PTJ2       62
#define PTJ3       63
#define PTJ4       64
#define PTJ5       65
#define PTJ6       66
#define PTJ7       67

#define PNQ0       80
#define PNQ1       81
#define PNQ2       82
#define PNQ3       83
#define PNQ4       84
#define PNQ5       85
#define PNQ6       86
#define PNQ7       87

#define PAN0      100
#define PAN1      101
#define PAN2      102
#define PAN3      103
#define PAN4      104
#define PAN5      105
#define PAN6      106
#define PAN7      107

#define PAS0      110
#define PAS1      111
#define PAS2      112
#define PAS3      113
#define PAS4      114
#define PAS5      115
#define PAS6      116
#define PAS7      117

#define PQS0      120
#define PQS1      121
#define PQS2      122
#define PQS3      123
#define PQS4      124
#define PQS5      125
#define PQS6      126
#define PQS7      127

#define PTA0      140
#define PTA1      141
#define PTA2      142
#define PTA3      143
#define PTA4      144
#define PTA5      145
#define PTA6      146
#define PTA7      147

#define PTC0      150
#define PTC1      151
#define PTC2      152
#define PTC3      153
#define PTC4      154
#define PTC5      155
#define PTC6      156
#define PTC7      157

#define PUA0      170
#define PUA1      171
#define PUA2      172
#define PUA3      173
#define PUA4      174
#define PUA5      175
#define PUA6      176
#define PUA7      177

#define PUB0      180
#define PUB1      181
#define PUB2      182
#define PUB3      183
#define PUB4      184
#define PUB5      185
#define PUB6      186
#define PUB7      187

#define PUC0      190
#define PUC1      191
#define PUC2      192
#define PUC3      193
#define PUC4      194
#define PUC5      195
#define PUC6      196
#define PUC7      197

#define PDD0      200
#define PDD1      201
#define PDD2      202
#define PDD3      203
#define PDD4      204
#define PDD5      205
#define PDD6      206
#define PDD7      207

/*
 * GPIO API
 */
#define GPIO_CFG_PERIPHERAL_MASK    0x00000003
#define GPIO_CFG_INPUT              0x00000000
#define GPIO_CFG_OUTPUT             0x00000004
#define GPIO_CFG_PERIPHERAL0        0x00000001
#define GPIO_CFG_PERIPHERAL1        0x00000002
#define GPIO_CFG_PERIPHERAL2        0x00000003
#define GPIO_CFG_DEBOUNCE           0
#define GPIO_CFG_PULLUP             0

#define GpioPinGet(bank, bit)           ((MCF_GPIO_PIN(bank) >> bit) & 0x1)
#define GpioPinSet(bank, bit, value)    (value) ? (GpioPinSetHigh(bank, bit)) : (GpioPinSetLow(bank, bit))
#define GpioPinSetHigh(bank, bit)       MCF_GPIO_SET(bank) = _BV(bit)
#define GpioPinSetLow(bank, bit)        MCF_GPIO_CLR(bank) = _BV(bit)

#define GpioPortGet(bank)               MCF_GPIO_PIN(bank)
#define GpioPortSet(bank, value)        MCF_GPIO_PORT(bank) = (value)
#define GpioPortSetHigh(bank, mask)     MCF_GPIO_SET(bank) = (mask)
#define GpioPortSetLow(bank, mask)      MCF_GPIO_CLR(bank) = (mask)

extern uint32_t GpioPinConfigGet(int bank, int bit);
extern int GpioPinConfigSet(int bank, int bit, uint32_t flags);
extern int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags);
