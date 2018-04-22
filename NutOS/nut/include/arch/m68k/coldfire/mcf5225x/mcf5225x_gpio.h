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

#ifndef MCF5225X_GPIO_H_
#define MCF5225X_GPIO_H_

/*
 * GPIO Registers
 */
#define MCF_GPIO_PORT(bank)  (*(volatile uint8_t *)(0x40100000 + (bank)))
#define MCF_GPIO_DDR(bank)   (*(volatile uint8_t *)(0x40100018 + (bank)))
#define MCF_GPIO_PIN(bank)   (*(volatile uint8_t *)(0x40100030 + (bank)))
#define MCF_GPIO_SET(bank)   (*(volatile uint8_t *)(0x40100030 + (bank)))
#define MCF_GPIO_CLR(bank)   (*(volatile uint8_t *)(0x40100048 + (bank)))
#define MCF_GPIO_PAR8(bank)  (*(volatile uint8_t *)(0x40100060 + (bank)))
#define MCF_GPIO_PAR16(bank) (*(volatile uint16_t *)(0x40100060 + (bank == 3) ? 0x30 : (bank)))

#endif /* MCF5225X_GPIO_H_ */
