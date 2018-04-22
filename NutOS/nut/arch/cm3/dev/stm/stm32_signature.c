/*
 * Copyright (C) 2015 Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 *
 */
#include <stdint.h>
#include <cfg/arch.h>
#include <dev/hw_signature.h>

#if !defined(MCU_STM32F030)
void Stm32GetUniquePrivateMac(void* mac)
{
    uint32_t id_w;
    uint16_t  id, *mac_p;

    mac_p = (uint16_t *) mac;
    id_w = *(uint32_t *) UNIQUE_ID_REG_L;
    id = (id_w & 0xffff) ^ (id_w >> 16);
    id &= 0xff00;
    id |= 0x0002;
    *mac_p = id;
    mac_p++;
    id_w = *(uint32_t *) UNIQUE_ID_REG_M;
    id = (id_w & 0xffff) ^ (id_w >> 16);
    *mac_p = id;
    mac_p++;
    id_w = *(uint32_t *) UNIQUE_ID_REG_H;
    id = (id_w & 0xffff) ^ (id_w >> 16);
    *mac_p = id;
    mac_p++;
}
#endif
