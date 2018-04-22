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

#ifndef MCF5225X_I2C_H_
#define MCF5225X_I2C_H_

/* I2C Registers */
#define MCF_I2C_I2ADR(x)                    (*(volatile uint8_t *)(0x40000300 + ((x) * 0x80)))
#define MCF_I2C_I2FDR(x)                    (*(volatile uint8_t *)(0x40000304 + ((x) * 0x80)))
#define MCF_I2C_I2CR(x)                     (*(volatile uint8_t *)(0x40000308 + ((x) * 0x80)))
#define MCF_I2C_I2SR(x)                     (*(volatile uint8_t *)(0x4000030C + ((x) * 0x80)))
#define MCF_I2C_I2DR(x)                     (*(volatile uint8_t *)(0x40000310 + ((x) * 0x80)))

/* MCF_I2C_I2ADR */
#define MCF_I2C_I2ADR_ADR(x)                (((x) & 0x7F) << 0x1)

/* MCF_I2C_I2FDR */
#define MCF_I2C_I2FDR_IC(x)                 (((x) & 0x3F) << 0)

/* MCF_I2C_I2CR */
#define MCF_I2C_I2CR_RSTA                   0x4
#define MCF_I2C_I2CR_TXAK                   0x8
#define MCF_I2C_I2CR_MTX                    0x10
#define MCF_I2C_I2CR_MSTA                   0x20
#define MCF_I2C_I2CR_IIEN                   0x40
#define MCF_I2C_I2CR_IEN                    0x80

/* MCF_I2C_I2SR */
#define MCF_I2C_I2SR_RXAK                   0x1
#define MCF_I2C_I2SR_IIF                    0x2
#define MCF_I2C_I2SR_SRW                    0x4
#define MCF_I2C_I2SR_IAL                    0x10
#define MCF_I2C_I2SR_IBB                    0x20
#define MCF_I2C_I2SR_IAAS                   0x40
#define MCF_I2C_I2SR_ICF                    0x80

/* MCF_I2C_I2DR */
#define MCF_I2C_I2DR_DATA(x)                (((x) & 0xFF) << 0)

#endif /* MCF5225X_I2C_H_ */
