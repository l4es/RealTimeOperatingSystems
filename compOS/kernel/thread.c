/*
 thread.c - multi-threading support
  
 Author:		Paul Barker
 Part of:		COS
 Created:		31/08/04
 Last Modified:		05/11/04

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
#include <cos/int.h>
#include <cos/symbols.h>
#include <cos/string.h>
#include <cos/debug.h>
#include <x86-asm.h>

#include <cos/sysinfo.h>

// implemented in thr.s
extern void Schedule(interrupt_state_t* state);
extern u32_t Get_EFLAGS();

// pointer to the current thread
static kthread_t* volatile pThisThread;

// queue of runnable threads
static thread_queue_t tqRun;

// defined in int.s, fake return address for creating a thread
extern char FakeReturnAddr;

/////////////////////////////////////////////////////////////////////
// Thread Queues

bool_t thread_queue_empty(thread_queue_t* tq)
{
	thread_node_t* p = tq->head;
	return (p && p->thread);
}

void thread_queue_dump(thread_queue_t* tq)
{
	thread_node_t* p = tq->head;
	TRACE(("thread_queue_dump %x Head=%x\n", tq, p));
	
	while (p)
	{
		TRACE(("Node %x : thread=%x, next=%x\n", p, p->thread, p->next));
		p = p->next;
	}
}

kthread_t* thread_queue_head(thread_queue_t* tq)
{
	thread_node_t* p = tq->head;
	if (!p)
		return pThisThread;
	
	tq->head = p->next;	// detach the node
	kthread_t* thr = p->thread;
	kfree(p);
	return thr;
}

void queue_thread(kthread_t* thr)
{
	thread_queue_t* q = thr->queue;
	TRACE(("QueueThread %x in %x\n", thr, q));
	if (!(q->head))
	{
		q->head = kalloc(sizeof(thread_node_t));
		q->head->thread = thr;
		q->head->next = NULL;
		return;
	}
	
	thread_node_t* p = q->head;
	while (p->next)
		p = p->next;	// find end of queue

	// last node will have a thread
	p->next = kalloc(sizeof(thread_node_t));
	p->next->thread = thr;
	p->next->next = NULL;
}

/////////////////////////////////////////////////////////////////////
// Private functions

// called by low-level scheduler
//	assume the current thread is already queued before the interrupt
//	currently we only deal with Yield() so this is true
kthread_t* do_sched()
{
	thread_queue_dump(&tqRun);
	// assume we always have a runnable thread, for now
	kthread_t* thr = thread_queue_head(&tqRun);
	
	TRACE(("Schedule, thr=%x esp=%x\n", thr, thr->esp));
	pThisThread = thr;
	
	return thr;		// assembly code expects this in eax
}

// setup a child thread
void add_child_thread(kthread_t* thr)
{
	kthread_t* tmp = thr->parent->child;

	if (tmp == NULL)
	{
		thr->parent->child = thr;
		return;
	}

	// find end of child threads
	while (tmp->next)
		tmp = tmp->next;

	tmp->next = thr;
}

/////////////////////////////////////////////////////////////////////
// Public functions

kthread_t* get_current_thread()
{
	return pThisThread;
}

void yield()
{
	TRACE(("Yield()\n"));
	queue_thread(pThisThread);
	__asm__ __volatile__ ("int $0x50"::);
}

void thread_init()
{
	TRACE(("InitThread()\n"));
	// setup the current thread
	pThisThread = kalloc(sizeof(kthread_t));
	pThisThread->Stack = 0xA000;		// 40k
	pThisThread->StackSize = 0x5000;	// 24k
	pThisThread->esp = NULL;			// invalid until we store it in a stack switch
	pThisThread->uTicks = START_TICKS;	// assume we just started running
	pThisThread->parent = pThisThread->next = pThisThread->child = NULL;
			// no parent or child, no other threads allowed at this level
	
	pThisThread->queue = &tqRun;
	
	install_handler(0x50, &Schedule);
	
	init_timer();
	
	g_kernel_state |= THREAD_ENABLED;
	TRACE(("InitThread() done\n"));
}

// TODO: currently processor will become lost in hyperspace if fn returns
kthread_t* spawn_thread(thread_function_t fn)
{
	TRACE(("SpawnThread(fn=%x)\n", fn));
	kthread_t* thr = kalloc(sizeof(kthread_t));
	thr->Stack = (iptr_t)kalloc(8192);	// 2 page stack (8k)
	thr->StackSize = 8192;
	dword_t* p = (dword_t*)(thr->Stack + 8188);	// top dword of stack
	count_t i;
	
	// setup stack as if an interrupt happened
	*p-- = Get_EFLAGS() | (1 << 9);	// eflags with interrupt flag forced
	*p-- = KERNEL_CS;		// cs
	*p-- = (dword_t)fn;		// eip
	for (i = 0; i < 8; i++)
		*p-- = 0;		// fill errorCode, intNum and registers with zeroes
	*p-- = (thr->Stack + 8188);	// C frame pointer in ebp
	for (i = 0; i < 4; i++)
		*p-- = KERNEL_DS;	//fill segment selectors with kernel data segment
	
	--p;	// Stack now looks as if we were in an interrupt handler
	*p = (dword_t)&FakeReturnAddr;	// return address for function call, see int.s
	
	thr->esp = (iptr_t)p;

	thr->parent = pThisThread;
	thr->next = thr->child = NULL;
	add_child_thread(thr);
	
	thr->queue = &tqRun;
	queue_thread(thr);
	
	return thr;
}

// wait on a given thread queue
void wait(thread_queue_t* tq)
{
	TRACE(("Thread %x waiting on queue %x\n", pThisThread, tq));
	assert_ptr(tq);
	pThisThread->queue = tq;
	yield();
}

// wake first thread on a queue, returns number of threads woken
count_t wake_one(thread_queue_t* tq)
{
	assert_ptr(tq);
	kthread_t* thr = thread_queue_head(tq);
	
	// silently fail on an empty thread queue
	if (thr != pThisThread)
	{
		TRACE(("Waking thread %x from queue %x\n", thr, tq));
		thr->queue = &tqRun;
		queue_thread(thr);
		return 1;
	}
	
	return 0;
}

// wake all threads on a queue, returns number of threads woken
count_t wake_all(thread_queue_t* tq)
{
	assert_ptr(tq);
	kthread_t* thr;
	count_t i = 0;
	TRACE(("Waking all threads from queue %x\n", tq));
	
	// silently fail on an empty thread queue
	while (!thread_queue_empty(tq))
	{
		thr = thread_queue_head(tq);
		TRACE(("\tWaking Thread %x\n", thr));
		thr->queue = &tqRun;
		queue_thread(thr);
		i++;
	}
	TRACE(("WakeAll() Done\n"));
	return i;
}

//////////////////////////////////////////////////////////////////////////
// Mutex functions, based on GeekOS implementation

void do_mlock(mutex_t* m)
{
	cli();
	assert_ptr(m);
	assert(m->owner != pThisThread);
	
	// wait for the mutex to become free
	while (m->state == MUTEX_LOCKED)
		wait(&(m->queue));
	
	m->state = MUTEX_LOCKED;
	m->owner = pThisThread;
	
	sti();
}

void do_munlock(mutex_t* m)
{
	cli();
	assert_ptr(m);
	assert(m->owner == pThisThread);
	assert(m->state == MUTEX_LOCKED);
	
	m->state = MUTEX_UNLOCKED;
	m->owner = NULL;
	
	wake_one(&m->queue);
	
	sti();
}

void minit(mutex_t* m)
{
	assert_ptr(m);
	m->state = MUTEX_UNLOCKED;
	m->owner = NULL;
	m->queue.head = NULL;
}

/*
 TODO:	this needs some serious attention
	change mutex code to use test_and_set
*/
