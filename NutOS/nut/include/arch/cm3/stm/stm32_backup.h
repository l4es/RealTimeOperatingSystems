#if !defined(_STM32_BACKUP_H)
#define _STM32_BACKUP_H
/*
 * Copyright (C) 2015 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de).
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
 *
 */

/*!
 * \file arch/cm3/stm/stm32_backup.h
 * \brief On-Chip backup register access.
 * \verbatim
 */
#include <cfg/arch.h>
#if defined(RTC_BKP0R) || defined(BKP_DR4_D)
extern int Stm32BkupRegSave(unsigned int pos, const void *data, unsigned int len);
extern int Stm32BkupRegLoad(unsigned int pos, void *data, unsigned int len);
#endif

#if defined(BKPSRAM_BASE) && !defined(STM32F401)
extern int Stm32BkupMemSave(unsigned int pos, const void *data, unsigned int len);
extern int Stm32BkupMemLoad(unsigned int pos, void *data, unsigned int len);
extern void* Stm32BkupMemGet(unsigned int pos);
#endif

#endif
