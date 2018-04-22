#ifndef _ARCH_CM3_DWT_H_
#define _ARCH_CM3_DWT_H_

/*
 * Copyright (C) 2012 Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de).
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
 * THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
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
 */

/*!
 * \file arch/cm3/cm3_dwt.h
 * \brief Data watchpoint register support.
 */

/* Optional module as described in the Cortex-M3/4 Technical Reference Manual
 *
 * Reset state of DWT-CTRL
 * 0x40000000 if four comparators for watchpoints and triggers are present
 * 0x4F000000 if four comparators for watchpoints only are present
 * 0x10000000 if only one comparator is present
 * 0x1F000000 if one comparator for watchpoints and not triggers is present
 * 0x00000000 if DWT is not present.
 */

typedef struct
{
  __IO uint32_t CTRL;      /*!< DWT Control register,                        Address offset: 0x00 */
  __IO uint32_t CYCCNT;    /*!< DWT Current PC Sampler Cycle Count,          Address offset: 0x04 */
  __IO uint32_t CPICNT;    /*!< DWT CPI Count Register,                      Address offset: 0x08 */
  __IO uint32_t EXCCNT;    /*!< DWT Exception Overhead Count Register,       Address offset: 0x0C */
  __IO uint32_t SLEEPCNT;  /*!< DWT Sleep Count Register,                    Address offset: 0x10 */
  __IO uint32_t LSUCNT;    /*!< DWT LSU Count Register,                      Address offset: 0x14 */
  __IO uint32_t FOLDCNT;   /*!< DWT Fold Count Register,                     Address offset: 0x18 */
  __IO uint32_t PCSR;      /*!< DWT Program Counter Sample Register,         Address offset: 0x1C */
  __IO uint32_t COMP0;     /*!< DWT Comparator Registers 0,                  Address offset: 0x20 */
  __IO uint32_t MASK0;     /*!< DWT Mask Registers 0,                        Address offset: 0x24 */
  __IO uint32_t FUNCTION0; /*!< DWT Function Registers 0,                    Address offset: 0x28 */
  __IO uint32_t RSVR0;     /*!< DWT Reserver 0,                              Address offset: 0x2C */
  __IO uint32_t COMP1;     /*!< DWT Comparator Registers 1,                  Address offset: 0x30 */
  __IO uint32_t MASK1;     /*!< DWT Mask Registers 1,                        Address offset: 0x34 */
  __IO uint32_t FUNCTION1; /*!< DWT Function Registers 1,                    Address offset: 0x38 */
  __IO uint32_t RSVR1;     /*!< DWT Reserver 1,                              Address offset: 0x3C */
  __IO uint32_t COMP2;     /*!< DWT Comparator Registers 2,                  Address offset: 0x40 */
  __IO uint32_t MASK2;     /*!< DWT Mask Registers 2,                        Address offset: 0x44 */
  __IO uint32_t FUNCTION2; /*!< DWT Function Registers 2,                    Address offset: 0x48 */
  __IO uint32_t RSVR2;     /*!< DWT Reserver 2,                              Address offset: 0x4C */
  __IO uint32_t COMP3;     /*!< DWT Comparator Registers 3,                  Address offset: 0x50 */
  __IO uint32_t MASK3;     /*!< DWT Mask Registers 3,                        Address offset: 0x54 */
  __IO uint32_t FUNCTION3; /*!< DWT Function Registers 3,                    Address offset: 0x58 */
} DWT_TypeDef;
#define DWT_BASE           (0xE0001000)
#define DWT                ((DWT_TypeDef *) DWT_BASE)
#define DWT_CYCCNTENA  ((uint32_t)0x00000001)
#endif
