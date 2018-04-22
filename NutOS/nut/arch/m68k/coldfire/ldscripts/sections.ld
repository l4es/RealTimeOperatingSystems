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

SECTIONS
{
    .text :
    {
        PROVIDE (__vectors_start = .) ;
        KEEP(*(.vectors))
        
        /* Emit a CFM Key */
        . = __flashbar + 0x400;
        LONG (0x01234567) /* Backdoor Key */
        LONG (0x89abcdef) /* Backdoor Key */
        LONG (0xffffffff) /* Sector Protection */
        LONG (0x00000000) /* Supervisor Access */
        LONG (0x00000000) /* Data Access */
        LONG (0xffffffff) /* Security Word */

        KEEP(*(.init0))   /* Basic CPU initialization (family dependant code). */
        KEEP(*(.init1))
        KEEP(*(.init2))   /* Set up stack pointer.  */
        KEEP(*(.init3))
        KEEP(*(.init4))   /* External Memories initialization (board dependant code). */
        KEEP(*(.init5))
        KEEP(*(.init6))   /* Initialize memory regions */
        KEEP(*(.init7))
        KEEP(*(.init8))   /* Relocate vectors, Enable interrupts */
        KEEP(*(.init9))
        KEEP(*(.init10))  /* CPU and Peripherals Initialization (family dependant code). */
        KEEP(*(.init11))
        KEEP(*(.init12))  /* C++ constructors (NOT implemented yet). */
        KEEP(*(.init13))
        KEEP(*(.init14))  /* Enter Nut/OS Initialization. */

        *(.text)
        *(.text.*)
        *(.gnu.linkonce.t.* .gcc_except_table .gcc_except_table.*)

        /* following table is used in InitRegions() */
        . = ALIGN(4);
        PROVIDE (__regions_start = .) ;

        LONG (__dataint_start)
        LONG (__dataint_size)
        LONG (__dataint_load_start)

        LONG (__data_start)
        LONG (__data_size)
        LONG (__data_load_start)
        
        LONG (__bssint_start)
        LONG (__bssint_size)
        LONG (__bssint_start)    /* No init data */

        LONG (__bss_start)
        LONG (__bss_size)
        LONG (__bss_start)       /* No init data */

        PROVIDE (__regions_end = .);
    } > REGION_TEXT AT > REGION_LOAD

    .rodata : ALIGN (4)
    {
        *(.rodata)
        *(.rodata.*)
        *(.gnu.linkonce.r.*)
    } > REGION_RODATA AT > REGION_LOAD

    .dataint : ALIGN(4)
    {
        PROVIDE (__dataint_start = .) ;
        *(.dataint)
        *(.dataint.*)
        . = ALIGN(4);
        PROVIDE (__dataint_end = .) ;
    } > REGION_DATA_INT AT > REGION_LOAD
    PROVIDE (__dataint_size = SIZEOF(.dataint));
    PROVIDE (__dataint_load_start = LOADADDR(.dataint));
    
    .bssint : ALIGN (4)
    {
        PROVIDE (__bssint_start = .) ;
        *(.bssint)
        *(.bssint.*)
        PROVIDE (__bssint_end = .) ;
    } > REGION_BSS_INT AT > REGION_LOAD
    PROVIDE (__bssint_size = SIZEOF(.bssint));

    .data : ALIGN(4)
    {
        PROVIDE (__data_start = .) ;
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        . = ALIGN(4);
        PROVIDE (__data_end = .) ;
    }  > REGION_DATA AT > REGION_LOAD
    PROVIDE (__data_size = SIZEOF(.data));
    PROVIDE (__data_load_start = LOADADDR(.data));

    .bss : ALIGN (4)
    {
        PROVIDE (__bss_start = .) ;
        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        PROVIDE (__bss_end = .) ;
    } > REGION_BSS AT > REGION_LOAD
    PROVIDE (__bss_size = SIZEOF(.bss));
}
