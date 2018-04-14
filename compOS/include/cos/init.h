/*
 init.h - initialisation functions
  
 Author:        Paul Barker
 Part of:       COS
 Created:       02/09/04
 Last Modified: 06/11/04

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

#ifndef _COS_INIT_H_
#define _COS_INIT_H_

/*
	All init functions have no return-value since they will only return
	if they are successful.
*/

void sys_init(multiboot_info_t* info);
void gdt_init();
void tss_init();
void int_init();
void idt_init();
void phys_init();
void kalloc_init();
void thread_init();

void do_tests();

#endif // !_COS_INIT_H_
