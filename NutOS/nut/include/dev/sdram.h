#ifndef _SDRAM_H_
#define _SDRAM_H_

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
 */

/*
 * \file dev/sdram.h
 * \brief Datastructs to specify SDRAM timings for several sdrams
 *
 *
 * \verbatim
 * $Id: $
 * \endverbatim
 */

typedef struct _sdram_params {
    uint32_t    base_addr;
    uint32_t    size;
    uint32_t    bus_width;

    uint8_t     rows;
    uint8_t     cols;

    uint8_t     ras_latency;    /* row address strobe latency (cycles)        */
    uint8_t     cas_latency;    /* collumn address strobe latency (cycles)    */

    uint8_t     tRP;            /* precharge to activate time (ns)            */
    uint8_t     tRAS;           /* active to precharge time (ns)              */
    uint8_t     tSREX;          /* self refresh exit time. (ns) for devices
                                   without this parameter use the same value
                                   as tXSR                                    */
    uint8_t     tAPR;           /* last active data out to active (cycles)    */
    uint8_t     tDAL;           /* data-in to active time (cycles)            */

    uint8_t     tWR;            /* write recovery time (cycles)               */
    uint8_t     tRC;            /* active to active time (ns)                 */
    uint8_t     tRFC;           /* auto-refresh, and auto-refresh to active time (ns) */
    uint8_t     tXSR;           /* exit self-refresh to active time (ns)      */
    uint8_t     tRRD;           /* active bank A to active bank B latency (ns)*/
    uint8_t     tMRD;           /* load mode register to active time (cycles) */

    uint16_t    refresh;        /* refresh time for array (us)                */
} SDRAM;

#endif

