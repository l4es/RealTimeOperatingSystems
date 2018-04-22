#ifndef _DEV_SHT21_H_
#define _DEV_SHT21_H_
/*
 * Copyright (C) 2010 by Rittal GmbH & Co. KG,
 * Dawid Sadji <sadji.d@rittal.de> All rights reserved.
 * Ulrich Prinz <prinz.u@rittal.de> All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY EMBEDDED IT AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EMBEDDED IT
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 */

/*
 * \verbatim
 * $Id:$
 * \endverbatim
 */
#include <cfg/sht21.h>

#ifndef I2C_SLA_SHT21
#define I2C_SLA_SHT21     0x40
#endif

/*
 * Sensirion common sensor commands
 */
#ifdef SHT_ACK_POLLING
#define SHT_GET_TEMP    0xF3    /* Get Temperature Command */
#define SHT_GET_HUM     0xF5    /* Get Humidity Command */
#else
#define SHT_GET_TEMP    0xE3    /* Get Temperature Command */
#define SHT_GET_HUM     0xE5    /* Get Humidity Command */
#endif
#define SHT_SET_USER    0xE6    /* Set User Register Command */
#define SHT_GET_USER    0xE7    /* Get User Register Command */
#define SHT_SOFT_RESET  0xFE    /* Soft Reset Command */
#define SHT_READ_OCM    0xFA    /* Read On Chip Memory Command */
#define SHT_GET_SERNR   0x0F    /* Read OCM: Serial Number */

/* Bit definitions of User Register */
#define SHT_RES_12_14   0x00    /* Resolution: RH=12bit, T=14bit */
#define SHT_RES_8_12    0x01    /* Resolution: RH= 8bit, T=12bit */
#define SHT_RES_10_13   0x80    /* Resolution: RH=10bit, T=13bit */
#define SHT_RES_11_11   0x81    /* Resolution: RH=11bit, T=11bit */
#define SHT_RES_MASK    0x81    /* Resolution: Mask for res. bits */

#define SHT_USER_EOB    0x40    /* User Register: End Of Battery Flag */
#define SHT_USER_HTR    0x04    /* User Register: Heater On Flag */

int ShtCrc(uint8_t *Data, uint8_t Size);
int ShtCommand(uint8_t cmd, uint16_t *data);
int ShtRead( uint8_t cmd, int16_t *val);
int ShtInit(void);

#endif /* _DEV_SHT21_H_ */
