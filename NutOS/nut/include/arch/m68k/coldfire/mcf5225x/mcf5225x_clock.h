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

#ifndef MCF5225X_CLOCK_H_
#define MCF5225X_CLOCK_H_

/* CLOCK Registers */
#define MCF_CLOCK_SYNCR                      (*(volatile uint16_t*)(0x40120000))
#define MCF_CLOCK_SYNSR                      (*(volatile uint8_t *)(0x40120002))
#define MCF_CLOCK_ROCR                       (*(volatile uint16_t*)(0x40120004))
#define MCF_CLOCK_LPDR                       (*(volatile uint8_t *)(0x40120007))
#define MCF_CLOCK_CCHR                       (*(volatile uint8_t *)(0x40120008))
#define MCF_CLOCK_CCLR                       (*(volatile uint8_t *)(0x40120009))
#define MCF_CLOCK_OCHR                       (*(volatile uint8_t *)(0x4012000A))
#define MCF_CLOCK_OCLR                       (*(volatile uint8_t *)(0x4012000B))
#define MCF_CLOCK_RTCCR                      (*(volatile uint8_t *)(0x40120012))
#define MCF_CLOCK_BWCR                       (*(volatile uint8_t *)(0x40120013))

/* Bit definitions and macros for MCF_CLOCK_SYNCR */
#define MCF_CLOCK_SYNCR_PLLEN                (0x1)
#define MCF_CLOCK_SYNCR_PLLMODE              (0x2)
#define MCF_CLOCK_SYNCR_CLKSRC               (0x4)
#define MCF_CLOCK_SYNCR_FWKUP                (0x20)
#define MCF_CLOCK_SYNCR_DISCLK               (0x40)
#define MCF_CLOCK_SYNCR_LOCEN                (0x80)
#define MCF_CLOCK_SYNCR_RFD(x)               (((x)&0x7)<<0x8)
#define MCF_CLOCK_SYNCR_LOCRE                (0x800)
#define MCF_CLOCK_SYNCR_MFD(x)               (((x)&0x7)<<0xC)
#define MCF_CLOCK_SYNCR_LOLRE                (0x8000)

/* Bit definitions and macros for MCF_CLOCK_SYNSR */
#define MCF_CLOCK_SYNSR_LOCS                 (0x4)
#define MCF_CLOCK_SYNSR_LOCK                 (0x8)
#define MCF_CLOCK_SYNSR_LOCKS                (0x10)
#define MCF_CLOCK_SYNSR_CRYOSC               (0x20)
#define MCF_CLOCK_SYNSR_OCOSC                (0x40)
#define MCF_CLOCK_SYNSR_EXTOSC               (0x80)

/* Bit definitions and macros for MCF_CLOCK_ROCR */
#define MCF_CLOCK_ROCR_TRIM(x)               (((x)&0x3FF)<<0)

/* Bit definitions and macros for MCF_CLOCK_LPDR */
#define MCF_CLOCK_LPDR_LPD(x)                (((x)&0xF)<<0)

/* Bit definitions and macros for MCF_CLOCK_CCHR */
#define MCF_CLOCK_CCHR_CCHR(x)               (((x)&0x7)<<0)

/* Bit definitions and macros for MCF_CLOCK_CCLR */
#define MCF_CLOCK_CCLR_OSCSEL0               (0x1)
#define MCF_CLOCK_CCLR_OSCSEL1               (0x2)

/* Bit definitions and macros for MCF_CLOCK_OCHR */
#define MCF_CLOCK_OCHR_STBY                  (0x40)
#define MCF_CLOCK_OCHR_OCOEN                 (0x80)

/* Bit definitions and macros for MCF_CLOCK_OCLR */
#define MCF_CLOCK_OCLR_RANGE                 (0x10)
#define MCF_CLOCK_OCLR_LPEN                  (0x20)
#define MCF_CLOCK_OCLR_REFS                  (0x40)
#define MCF_CLOCK_OCLR_OSCEN                 (0x80)

/* Bit definitions and macros for MCF_CLOCK_RTCCR */
#define MCF_CLOCK_RTCCR_RTCSEL               (0x1)
#define MCF_CLOCK_RTCCR_LPEN                 (0x2)
#define MCF_CLOCK_RTCCR_REFS                 (0x4)
#define MCF_CLOCK_RTCCR_KHZEN                (0x8)
#define MCF_CLOCK_RTCCR_OSCEN                (0x10)
#define MCF_CLOCK_RTCCR_EXTALEN              (0x40)

/* Bit definitions and macros for MCF_CLOCK_BWCR */
#define MCF_CLOCK_BWCR_BWDSEL                (0x1)
#define MCF_CLOCK_BWCR_BWDSTOP               (0x2)

#endif /* MCF5225X_CLOCK_H_ */
