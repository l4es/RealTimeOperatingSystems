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

#ifndef MCF5225X_FBCS_H_
#define MCF5225X_FBCS_H_

/* Mini-FlexBus Chip Select Registers */
#define MCF_FBCS_CSAR(x)                     (*(volatile uint32_t*)(0x40000080 + ((x) * 0xC)))
#define MCF_FBCS_CSMR(x)                     (*(volatile uint32_t*)(0x40000084 + ((x) * 0xC)))
#define MCF_FBCS_CSCR(x)                     (*(volatile uint32_t*)(0x40000088 + ((x) * 0xC)))


/* MCF_FBCS_CSAR */
#define MCF_FBCS_CSAR_BA(x)                  ((x) & 0xFFFF0000)

/* MCF_FBCS_CSMR */
#define MCF_FBCS_CSMR_V                      0x1
#define MCF_FBCS_CSMR_WP                     0x100
#define MCF_FBCS_CSMR_BAM(x)                 (((x) & 0xFFFF) << 0x10)
#define MCF_FBCS_CSMR_BAM_4G                 0xFFFF0000
#define MCF_FBCS_CSMR_BAM_2G                 0x7FFF0000
#define MCF_FBCS_CSMR_BAM_1G                 0x3FFF0000
#define MCF_FBCS_CSMR_BAM_1024M              0x3FFF0000
#define MCF_FBCS_CSMR_BAM_512M               0x1FFF0000
#define MCF_FBCS_CSMR_BAM_256M               0xFFF0000
#define MCF_FBCS_CSMR_BAM_128M               0x7FF0000
#define MCF_FBCS_CSMR_BAM_64M                0x3FF0000
#define MCF_FBCS_CSMR_BAM_32M                0x1FF0000
#define MCF_FBCS_CSMR_BAM_16M                0xFF0000
#define MCF_FBCS_CSMR_BAM_8M                 0x7F0000
#define MCF_FBCS_CSMR_BAM_4M                 0x3F0000
#define MCF_FBCS_CSMR_BAM_2M                 0x1F0000
#define MCF_FBCS_CSMR_BAM_1M                 0xF0000
#define MCF_FBCS_CSMR_BAM_1024K              0xF0000
#define MCF_FBCS_CSMR_BAM_512K               0x70000
#define MCF_FBCS_CSMR_BAM_256K               0x30000
#define MCF_FBCS_CSMR_BAM_128K               0x10000
#define MCF_FBCS_CSMR_BAM_64K                0

/* MCF_FBCS_CSCR */
#define MCF_FBCS_CSCR_BSTW                   0x8
#define MCF_FBCS_CSCR_BSTR                   0x10
#define MCF_FBCS_CSCR_PS(x)                  (((x) & 0x3) << 0x6)
#define MCF_FBCS_CSCR_PS_8                   0x40
#define MCF_FBCS_CSCR_PS_16                  0x80
#define MCF_FBCS_CSCR_AA                     0x100
#define MCF_FBCS_CSCR_MUX                    0x200
#define MCF_FBCS_CSCR_WS(x)                  (((x) & 0x3F) << 0xA)
#define MCF_FBCS_CSCR_WRAH(x)                (((x) & 0x3) << 0x10)
#define MCF_FBCS_CSCR_RDAH(x)                (((x) & 0x3) << 0x12)
#define MCF_FBCS_CSCR_ASET(x)                (((x) & 0x3) << 0x14)
#define MCF_FBCS_CSCR_SWSEN                  0x800000
#define MCF_FBCS_CSCR_SWS(x)                 (((x) & 0x3F) << 0x1A)

#endif /* MCF5225X_FBCS_H_ */
