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

#include <stdio.h>
#include <stdint.h>

//------------------------------------------------------------------------------
void InitRegions(void)
{
    struct __region
    {
        unsigned *p_region_start;   // start address of region
        unsigned region_size;
        unsigned *p_region_data;    // initial contents of this region (p_region_data == p_region_start means "clear the region only")
    };

    extern const struct __region __regions_start[];
    extern const struct __region __regions_end[];

    const struct __region   *p_reg;
    const struct __region   *p_reg_end;

    /* get regions pointer and count (they was defined in linker file) */
    p_reg = __regions_start;
    p_reg_end = __regions_end;
    
    /* initialize all regions */
    for (; p_reg < p_reg_end; p_reg++)
    {
        unsigned *p_src = p_reg->p_region_data;
        unsigned *p_dst = p_reg->p_region_start;
        unsigned count = p_reg->region_size;
        
        if (p_src != p_dst)
        {
            // init data present --> copy them
            for (; count; count -= sizeof(unsigned))
                *p_dst++ = *p_src++;
        }
        else
        {
            // no init data --> bss section --> clear only
            for (; count >= 4; count -= sizeof(unsigned))
                *p_dst++ = 0;

            char *pdst_char = (char *)p_dst;

            for (; count; count--)
                *pdst_char++ = 0;
        }
    }
}
