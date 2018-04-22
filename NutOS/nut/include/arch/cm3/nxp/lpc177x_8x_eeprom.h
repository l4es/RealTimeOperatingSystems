#ifndef _LPC177X_8X_EEPROM_H_
#define _LPC177X_8X_EEPROM_H_

/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 *
 *
 * Parts taken from lpc177x_8x_eeprom.h         2011-06-02
 * file     lpc177x_8x_eeprom.h
 * brief    Contains all macro definitions and function prototypes
 *          support for EEPROM firmware library on LPC177x_8x
 * version  1.0
 * date     02. June. 2011
 * author   NXP MCU SW Application Team
 *
 * Copyright(C) 2011, NXP Semiconductor
 * All rights reserved.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/

#include <inttypes.h>

/* We can not use the last page, as writing to it will result in a bus fault!!! */
#define EEPROM_SIZE                     (4096 - 64)

/*----------------------------------------------------------------------------*
  Macro defines for EEPROM command register
 *----------------------------------------------------------------------------*/

#define EEPROM_CMD_8_BIT_READ           0
#define EEPROM_CMD_16_BIT_READ          1
#define EEPROM_CMD_32_BIT_READ          2
#define EEPROM_CMD_8_BIT_WRITE          3
#define EEPROM_CMD_16_BIT_WRITE         4
#define EEPROM_CMD_32_BIT_WRITE         5
#define EEPROM_CMD_ERASE_PRG_PAGE       6

#define EEPROM_CMD_RDPREFETCH           _BV(3)

#define EEPROM_PAGE_SIZE                64
#define EEPROM_PAGE_NUM                 64


/*----------------------------------------------------------------------------*
 Macro defines for EEPROM address register
 *----------------------------------------------------------------------------*/

#define EEPROM_PAGE_OFFSET_MASK         0x3F
#define EEPROM_PAGE_NUM_MASK            (0x3F << 6)
#define EEPROM_PAGE_OFFSET(n)           ((n) & 0x3F)
#define EEPROM_PAGE_ADRESS(n)           (((n) & 0x3F) << 6)


/*----------------------------------------------------------------------------*
 Macro defines for EEPROM write data register
 *----------------------------------------------------------------------------*/

#define EEPROM_WDATA_8_BIT(n)           ((n) & 0x000000FF)
#define EEPROM_WDATA_16_BIT(n)          ((n) & 0x0000FFFF)
#define EEPROM_WDATA_32_BIT(n)          ((n) & 0xFFFFFFFF)


/*----------------------------------------------------------------------------*
 Macro defines for EEPROM read data register
 *----------------------------------------------------------------------------*/

#define EEPROM_RDATA_8_BIT(n)           ((n) & 0x000000FF)
#define EEPROM_RDATA_16_BIT(n)          ((n) & 0x0000FFFF)
#define EEPROM_RDATA_32_BIT(n)          ((n) & 0xFFFFFFFF)


/*----------------------------------------------------------------------------*
 Macro defines for EEPROM power down register
 *----------------------------------------------------------------------------*/

#define EEPROM_PWRDWN                   _BV(0)

#define EEPROM_ENDOF_RW                 26
#define EEPROM_ENDOF_PROG               28

/*----------------------------------------------------------------------------*
 Public functions
 *----------------------------------------------------------------------------*/

void Lpc177x_8x_EepromInit(void);
int Lpc177x_8x_EepromRead(uint16_t addr, void* buff, size_t size);
int Lpc177x_8x_EepromWrite(uint16_t addr, const void* buff, size_t size);

#endif /* _LPC177X_8X_EEPROM_H_ */




