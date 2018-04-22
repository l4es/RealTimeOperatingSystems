/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 *
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
 */

/*
 * \verbatim
 * $Id: stm32_crc.c 3110 2010-09-16 08:32:42Z astralix $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <cfg/arch/gpio.h>

#include <arch/cm3.h>
#include <arch/cm3/stm/stm32xxxx.h>


/**
  * @brief  Resets the CRC Data register (DR).
  * @param  None
  * @retval : None
  */
void CRC_ResetDR(void)
{
    /* Reset CRC generator */
    CRC->CR = CRC_CR_RESET;
}

/**
  * @brief  Computes the 32-bit CRC of a given data word(32-bit).
  * @param Data: data word(32-bit) to compute its CRC
  * @retval : 32-bit CRC
  */
uint32_t CRC_CalcCRC(uint32_t Data)
{
    CRC->DR = Data;
    return (CRC->DR);
}

/**
  * @brief  Computes the 32-bit CRC of a given buffer of data word(32-bit).
  * @param pBuffer: pointer to the buffer containing the data to be
  *   computed
  * @param BufferLength: length of the buffer to be computed
  * @retval : 32-bit CRC
  */
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength)
{
    uint32_t index = 0;

    for(index = 0; index < BufferLength; index++)
    {
        CRC->DR = pBuffer[index];
    }
    return (CRC->DR);
}

/**
  * @brief  Returns the current CRC value.
  * @param  None
  * @retval : 32-bit CRC
  */
uint32_t CRC_GetCRC(void)
{
    return (CRC->DR);
}


/**
  * @brief  Stores a 8-bit data in the Independent Data(ID) register.
  * @param IDValue: 8-bit value to be stored in the ID register
  * @retval : None
  */
void CRC_SetIDRegister(uint8_t IDValue)
{
    CRC->IDR = IDValue;
}

/**
  * @brief  Returns the 8-bit data stored in the Independent Data(ID) register
  * @param  None
  * @retval : 8-bit value of the ID register
  */
uint8_t CRC_GetIDRegister(void)
{
    return (CRC->IDR);
}

/**
  * @brief  Init the CRC machine
  * @param  None
  * @retval :None
  */
void CRC_Init(void)
{
#if defined(RCC_AHBENR_CRCEN)
    RCC->AHBENR |= RCC_AHBENR_CRCEN;
#elif defined(RCC_AHB1ENR_CRCEN)
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
#else
#warning "Unknown STM32 family"
#endif
    /* Reset CRC generator */
    CRC->CR = CRC_CR_RESET;
}
