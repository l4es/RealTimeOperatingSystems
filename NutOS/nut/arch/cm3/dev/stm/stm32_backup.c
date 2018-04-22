/*
 * Copyright (C) 2015, 2017 Uwe Bonnes
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
 * \file arch/cm3/stm/stm32_backup..c
 * \brief Handle parameters in backup registers and backup memory
 *
 * For register access, RTC needs to be running.
 * RCC_APB1ENR_PWREN is set during startup.
 * Write access is disable after Save().
 *
 * \verbatim
 * $Id: stm32_backup.c 6610 2017-02-15 15:36:53Z u_bonnes $
 * \endverbatim
 */

#include <sys/types.h>
#include <string.h>

#include <cfg/arch.h>
#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_clk.h>

#if !defined(RTC_BKP_NUMBER)
# if defined(RTC_BKP0R)
#  if !defined(RTC_BKP5R)
#   define RTC_BKP_NUMBER    5
#  elif !defined(RTC_BKP10R)
#   define RTC_BKP_NUMBER   10
#  elif !defined(RTC_BKP16R)
#   define RTC_BKP_NUMBER   16
#  elif !defined(RTC_BKP20R)
#   define RTC_BKP_NUMBER   20
#  elif defined(RTC_BKP31R)
#   define RTC_BKP_NUMBER   32
#  endif
# endif
#endif

#if defined(RTC_BKP_NUMBER) && !defined(RTC_BKP_BYTES)
# define RTC_BKP_BYTES (RTC_BKP_NUMBER * 4)

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
        uint8_t byte[4];
        uint32_t word;
    } bkp_data;
    uint32_t *bkp= (uint32_t *)&RTC->BKP0R;
    uint8_t *data_byte= (uint8_t *)data;
    int i;

    if ((!data) || (pos + len > RTC_BKP_BYTES)) {
        return -1;
    }
    for (i = 0; i < len ; i++) {
        int j = pos + i;
        bkp_data.word = bkp[j >> 2];
        data_byte[i] = bkp_data.byte[j & 3];
    }
    return 0;
}

/*!
 * \brief Store parameters in Backup registers.
 *
 * At least for L0, only aligned uint32_t can be written!
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
        uint8_t byte[4];
        uint32_t word;
    } shadow;
    uint8_t *data_byte  = (uint8_t *)data;
    uint32_t *bkp  = (uint32_t *)&RTC->BKP0R;
    int i;

    if ((!data) || (pos + len > RTC_BKP_BYTES)) {
        return -1;
    }
    PWR_CR |= PWR_CR_DBP;
    RTC->WPR = 0xca;
    RTC->WPR = 0x53;
    for (i = 0; i < len; i++) {
        shadow.word = bkp[(pos + i) >> 2];
        shadow.byte[(pos + i) & 3 ] = data_byte[i];
        bkp[(pos + i) >> 2] = shadow.word;
    }
    RTC->WPR = 0;
    PWR_CR &= ~PWR_CR_DBP;
    return 0;
}
#endif

#if defined(BKPSRAM_BASE)
#define BKPSRAM_SIZE 0x1000
/*!
 * \brief Get (pointer to read parameters from Backup memory.
 *
 * \param pos Offset of parameter.
 *
 * \return pointer to requested parameter, NULL on error.
 */
const void *Stm32BkupMemGet(unsigned int pos)
{
    if (RCC_AHB1ENR_BKPSRAMEN != (RCC->AHB1ENR & RCC_AHB1ENR_BKPSRAMEN)) {
        return 0;
    }
    return (void*)BKPSRAM_BASE + pos;
}
/*!
 * \brief Load parameters from Backup memory.
 *
 * \param pos Offset in bytes of parameter.
 * \param data Pointer where to copy data  to.
 * \param len Number of bytes to be copied.
 *
 * \return 0 on success, -1 otherwise.
 */
int Stm32BkupMemLoad(uint32_t pos, void *data, size_t len)
{
    const void *bkp;

    if ((!data) || (pos + len > BKPSRAM_SIZE)) {
        return -1;
    }
    bkp = Stm32BkupMemGet(pos);
    if (NULL == bkp) {
        return -1;
    }
    memcpy(data, bkp, len);
    return 0;
}

/*!
 * \brief Store parameters in Backup memory.
 *
 * \param pos Offset in byte of parameter.
 * \param data Pointer to source data.
 * \param len Number of bytes to be written.
 *
 * \return 0 on success, -1 otherwise.
 */
int Stm32BkupMemSave(unsigned int pos, const void *data, size_t len)
{
    const void *bkp;

    if ((!data) || (pos + len > BKPSRAM_SIZE)) {
        return -1;
    }
    bkp = Stm32BkupMemGet(pos);
    if (NULL == bkp) {
        return -1;
    }
    PWR_CR |= PWR_CR_DBP;
    memcpy((void *)bkp , data, len);
    PWR_CR &= ~PWR_CR_DBP;
    return 0;
}
#endif
