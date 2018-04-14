/*
 kmain.c - main kernel entry point
  
 Author:	Paul Barker
 Part of:	COS
 Created:	16/09/04
 Last Modified:	05/11/04

 Copyright (C) 2004 Paul Barker
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

                     (See file "Copying")
*/

/*
 This is based on the way the multiboot demo kernel dumps the
	multiboot_info_t structure.
*/

#include <cosbase.h>
#include "../version.h"

#include <multiboot.h>
#include <cos/debug.h>
#include <x86-asm.h>
#include <cos/symbols.h>
#include <cos/init.h>

// the stack, should be in bss
u8_t g_stack[16384];

// this is here so it can be checked before sysinfo has been created
u32_t g_kernel_state = 0;

void kmain(multiboot_info_t* info)
{
	TRACE(("cos v%s : {%d,%d}\n", COS_VERSION_STR, COS_V_UNIQUE_HI,
		COS_V_UNIQUE_LO));
	TRACE(("\t{%s}\n\n", COS_V_UNIQUE_STR));
	
	TRACE(("Dumping multiboot info struct at 0x%x\n", info));
	TRACE(("flags = 0x%x\n", info->flags));
	
	/* Are mem_* valid?  */
	if (info->flags & 0x01)
		TRACE(("mem_lower = %uKB, mem_upper = %uKB\n",
		       (unsigned) info->mem_lower,
		       (unsigned) info->mem_upper));
	else
		TRACE(("No memory size given\n"));

	/* Is boot_device valid?  */
	if (info->flags & 0x02)
		TRACE(("boot_device = 0x%x\n", (unsigned) info->boot_device));
	else
		TRACE(("No boot device given\n"));

	/* Has a command line been passed?  */
	if (info->flags & 0x04)
		TRACE(("cmdline = %s\n", (char *) info->cmdline));
	else
		TRACE(("No command line given\n"));

	/* Are mods_* valid?  */
	if (info->flags & 0x08)
	{
		module_t *mod;
		int i;

		TRACE(("mods_count = %d, mods_addr = 0x%x\n",
		       (int) info->mods_count, (int) info->mods_addr));
		for (i = 0, mod = (module_t *) info->mods_addr;
		     i < info->mods_count;
		     i++, mod += sizeof (module_t))
			TRACE((" mod_start = 0x%x, mod_end = 0x%x,"
				" string = %s\n",
			       (unsigned) mod->mod_start,
			       (unsigned) mod->mod_end,
			       (char *) mod->string));
	}
	else
		TRACE(("No module info given\n"));

	/* Bits 4 and 5 are mutually exclusive!  */
	if ((info->flags & 0x10) && (info->flags & 0x20))
	{
		TRACE(("Both bits 4 and 5 are set. This is illegal!\n"));
	}
	/* Is the symbol table of a.out valid?  */
	else if (info->flags & 0x10)
	{
		aout_symbol_table_t *aout_sym = &(info->u.aout_sym);

		TRACE(("aout_symbol_table: tabsize = 0x%0x, "
		       "strsize = 0x%x, addr = 0x%x\n",
		       (unsigned) aout_sym->tabsize,
		       (unsigned) aout_sym->strsize,
		       (unsigned) aout_sym->addr));
	}
	/* Is the section header table of ELF valid?  */
	else if (info->flags & 0x20)
	{
		elf_section_header_table_t *elf_sec = &(info->u.elf_sec);

		TRACE(("elf_sec: num = %u, size = 0x%x,"
		       " addr = 0x%x, shndx = 0x%x\n",
		       (unsigned) elf_sec->num, (unsigned) elf_sec->size,
		       (unsigned) elf_sec->addr, (unsigned) elf_sec->shndx));
	}
	/* neither the above are valid */
	else
		TRACE(("Neither an aout symbol table or elf section header "
			"table given\n"));

	/* Are mmap_* valid?  */
	if (info->flags & 0x40)
	{
		memory_map_t *mmap;

		TRACE(("mmap_addr = 0x%x, mmap_length = 0x%x\n",
		       (unsigned) info->mmap_addr,
		       (unsigned) info->mmap_length));
		for (mmap = (memory_map_t *) info->mmap_addr;
		     (unsigned long) mmap < info->mmap_addr
			     + info->mmap_length;
		     mmap = (memory_map_t *) ((unsigned long) mmap
		     + mmap->size + sizeof (mmap->size)))
			TRACE((" size = 0x%x, base_addr = 0x%x%x,"
			       " length = 0x%x%x, type = 0x%x\n",
			       (unsigned) mmap->size,
			       (unsigned) mmap->base_addr_high,
			       (unsigned) mmap->base_addr_low,
			       (unsigned) mmap->length_high,
			       (unsigned) mmap->length_low,
			       (unsigned) mmap->type));
	}
	else
		TRACE(("No memory map given\n"));
	
	TRACE(("Finished dumping multiboot info\n\n"));
	
	TRACE(("Dumping kernel memory info:\n"));
	TRACE(("kernel=%x, end_kernel=%x\n", &kernel, &end_kernel));
	TRACE(("cosinit=%x, end_cosinit=%x\n", &cosinit, &end_cosinit));
	TRACE(("code=%x, end_code=%x\n", &code, &end_code));
	TRACE(("data=%x, end_data=%x\n", &data, &end_data));
	TRACE(("bss=%x, end_bss=%x\n", &bss, &end_bss));
	TRACE(("rodata=%x, end_rodata=%x\n", &rodata, &end_rodata));
	TRACE(("g_stack=%x, end of g_stack=%x\n", g_stack, g_stack + 16384));
	TRACE(("Finished dumping kernel memory info\n\n"));

	sys_init(info);
	gdt_init();
	tss_init();
	int_init();
	
	phys_init();
	kalloc_init();
	thread_init();
	
	do_tests();
	
	TRACE(("Going into infinite loop...\n"));
	
	while (1) halt();
}
