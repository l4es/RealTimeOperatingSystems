/*
 thread.h - header for kernel thread support
  
 Author:        Paul Barker
 Part of:       COS
 Created:       31/08/04
 Last Modified: 02/09/04

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
02/09/04:	moved InitThread() to init.h
		Added mutex support and Wait(), Wake*()
29/10/04:	updated things for the new multiboot COS
		moved init_timer() back again
*/

#ifndef _COS_THREAD_H_
#define _COS_THREAD_H_

// sadly we need this for the mlock() macros
#include <cos/sysinfo.h>

struct kernel_thread;
typedef struct kernel_thread kthread_t;

struct thread_queue;
typedef struct thread_queue thread_queue_t;

struct thread_node;
typedef struct thread_node thread_node_t;

struct thread_queue
{
	thread_node_t* head;
};

struct thread_node
{
	kthread_t* thread;
	thread_node_t* next;
};

struct kernel_thread
{
	iptr_t esp;	// must be offset 0, should never be used except in stack switches
	volatile u32_t uTicks;
	iptr_t Stack;
	u32_t StackSize;
	kthread_t* parent;
	kthread_t* next;
	kthread_t* child;
	thread_queue_t* queue;
};

typedef void (*thread_function_t)();

#define START_TICKS 20
// 5 thread switches per second ignoring yields or waits

// public stuff in thread.c
kthread_t* get_current_thread();
void yield();
kthread_t* spawn_thread(thread_function_t fn);
void wait(thread_queue_t* tq);
void wake_one(thread_queue_t* tq);
void wake_all(thread_queue_t* tq);

// timer
void init_timer();

//////////////////////////////////////////////////////////////////////////
// Mutex definitions

#define MUTEX_LOCKED	1
#define MUTEX_UNLOCKED	0

typedef struct _mutex
{
	dword_t 		state;
	kthread_t*	owner;
	thread_queue_t		queue;
} mutex_t;

void do_mlock(mutex_t* m);
void do_munlock(mutex_t* m);
void minit(mutex_t* m);

extern u32_t g_kernel_state;

#define mlock(xx) if (g_kernel_state & THREAD_ENABLED) do_mlock(xx)
#define munlock(xx) if (g_kernel_state & THREAD_ENABLED) do_munlock(xx)

#endif // !_COS_THREAD_H_
