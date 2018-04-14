/*
 test.c - test various kernel features
  
 Author:	Paul Barker
 Part of:	COS
 Created:	05/11/04
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

#include <cosbase.h>

#include <cos/thread.h>
#include <cos/mem.h>
#include <cos/page.h>

/*
 page test:	total = 540 pages = 2.1MB
 kalloc test 1:	total = 540 * 512 = 270k
 kalloc test 2:	total = 540 * 8 = 4.2k
*/
static count_t test_map[32] =
{
	 2,  7, 68,  3,  5,  9, 18, 25,
	33,  1,  7, 12, 40, 20,  6, 10,
	 4,  8, 11, 19, 28, 30, 56, 50,
	 1,  2,  1,  5,  8, 14, 22, 15
};

static mutex_t test_mutex;

void dummy_thread_1()
{
	while (1)
		yield();
}

void dummy_thread_2()
{
	count_t i;
	
	while (1)
	{
		for (i = 0; i < 100000; i++);
		yield();
	}
}

void dummy_thread_3()
{
	count_t i;

	while (1)
	{
		for (i = 0; i < 800000; i++);
		mlock(&test_mutex);
		yield();
		munlock(&test_mutex);
	}
}

void do_tests()
{
	count_t i;
	ptr_t p[32];
	
	kthread_t* thr[16];
		/* just incase we ever need to do anything with these */
	
	/* Test page allocator */
	for (i = 0; i < 32; i++)
		p[i] = phys_alloc(test_map[i], PG_CLUSTER_IN_USE,
				  PG_CLUSTER_FREE);
		
	for (i = 0; i < 32; i++)
		phys_free(p[i], PG_CLUSTER_FREE);
	
	/* Test kalloc, pass 1 */
	for (i = 0; i < 32; i++)
		p[i] = kalloc(test_map[i] * 512);
	
	for (i = 0; i < 32; i++)
		kfree(p[i]);
	
	/* Test kalloc, pass 2 */
	for (i = 0; i < 32; i++)
		p[i] = kalloc(test_map[i] * 8);
	
	for (i = 0; i < 32; i++)
		kfree(p[i]);
	
	/* thread and mutex test */
	minit(&test_mutex);
	thr[0] = spawn_thread(dummy_thread_1);
	thr[1] = spawn_thread(dummy_thread_2);
	thr[2] = spawn_thread(dummy_thread_1);
	thr[3] = spawn_thread(dummy_thread_2);
	thr[4] = spawn_thread(dummy_thread_3);
	thr[5] = spawn_thread(dummy_thread_1);
	thr[6] = spawn_thread(dummy_thread_2);
	thr[7] = spawn_thread(dummy_thread_2);
	thr[8] = spawn_thread(dummy_thread_2);
	thr[9] = spawn_thread(dummy_thread_2);
	thr[10] = spawn_thread(dummy_thread_3);
	thr[11] = spawn_thread(dummy_thread_2);
	thr[12] = spawn_thread(dummy_thread_1);
	thr[13] = spawn_thread(dummy_thread_1);
	thr[14] = spawn_thread(dummy_thread_2);
	thr[15] = spawn_thread(dummy_thread_3);
	
	for (i = 0; i < 500; i++)
		yield();
	
	/* poke a character so we can see when this finishes */
	*((char_t*)0xB8000) = '*';
	*((char_t*)0xB8001) = 0x07;	/* white on black */
}
