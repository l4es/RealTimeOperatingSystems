/*
 * Copyright (C) 2017 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/stm/stm32f_backup..c
 * \brief Handle parameters in backup registers.
 *
 * F1 has 10 or 42 16-bit backup registers. There is gap between after DR10.
 * 16-bit registers are 4-byte aligned!

 * For register write access, PWR and BKP needs to be clocked and
 * backup domain write enabled.
 * RCC_APB1ENR_PWREN is set during startup.
 * Write access is disable after Save().
 *
 * \verbatim
 * $Id: stm32f1_backup.c 6679 2017-10-10 18:30:02Z u_bonnes $
 * \endverbatim
 */

#include <sys/types.h>
#include <string.h>

#include <cfg/arch.h>
#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_clk.h>

#if defined(BKP_DR42_D)
# define RTC_BKP_NUMBER   42
#define IDX2DR_BASE(x) ((x > 19)? &BKP->DR1: bkp = &BKP->DR0)
#else
# define RTC_BKP_NUMBER   10
#define IDX2DR_BASE(x) (&BKP->DR0)
#endif

static uint32_t Pos2Offset(uint32_t byte_idx)
{
    uint32_t res;

    res  = (((byte_idx >> 1) << 2) | (byte_idx & 1));
#if defined(BKP_DR42_D)
    if (res >= 40) {
        res += offsetof(BKP_TypeDef, DR11) - offsetof(BKP_TypeDef, RTCCR);
    }
#endif
    return res;
}

/*!
 * \brief Load parameters from Backup Registers.
 *
 * \param pos Offset in bytes of parameter.
 * \param data Pointer where to copy data  to.
 * \param len Number of bytes to be copied.
 *
 * \return 0 on success, -1 otherwise.
 */
int Stm32BkupRegLoad(uint32_t pos, void *data, size_t len)
{
    union {
        uint8_t  byte[4];
        uint32_t word;
    } bkp_data;
    int  i;
    uint8_t *data_byte= (uint8_t *) data;
    uint32_t *bkp = (uint32_t *)&BKP->DR1;

    if ((!data) || (pos + len > RTC_BKP_NUMBER * 2)) {
        return -1;
    }
    for (i = 0; i < len; i++) {
        int j = Pos2Offset(pos + i);
        bkp_data.word = bkp[j >> 2];
        data_byte[i] = bkp_data.byte[j & 1];
    }
    return 0;
}

/*!
 * \brief Store parameters in Backup registers.
 *
 * \param pos Offset in byte of parameter.
 * \param data Pointer to source data.
 * \param len Number of bytes to be written.
 *
 * \return 0 on success, -1 otherwise.
 */
int Stm32BkupRegSave(unsigned int pos, const void *data, size_t len)
{
    union {
        uint8_t  byte[4];
        uint32_t word;
    } bkp_data;
    int i;
    uint8_t *data_byte= (uint8_t *) data;
    uint32_t *bkp= (uint32_t *)&BKP->DR1;


    if ((!data) || (pos + len > RTC_BKP_NUMBER * 2)) {
        return -1;
    }
    RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
    PWR_CR |= PWR_CR_DBP;
    for (i = 0; i < len; i++) {
        int j;
        j = Pos2Offset(pos + i);
        bkp_data.word =  bkp[j >> 2];
        bkp_data.byte[j & 1 ] = data_byte[i];
        bkp[j >> 2] = bkp_data.word;
    }
    RCC->APB1ENR &= ~RCC_APB1ENR_BKPEN;
    PWR_CR &= ~PWR_CR_DBP;
    return 0;
}
