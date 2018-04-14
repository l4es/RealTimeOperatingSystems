/*
 * Copyright (C) 2004-2005, Marko Panger
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
 * 3. Neither the name of the author may be used to endorse or promote products
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information send an email to marko.panger@siol.net
 *
 */
 
#include <usmartx.h>
#include <queue.h>

/*! \mainpage \image html smartx_logo.JPG Users manual
 *	
 *	\n\n
 *	\b uSmartX is a non-preemptive, multitasking, priority based RTOS. It features mechanisms for inter-task communication
 *	and basic task and time control functions. \n\n
 *	<b> Main features: </b>
 *					- task control
 *					- time control 
 *					- mailboxes
 *					- sempahores
 *					- cyclic buffers with trigger functions
 *					- memory management
 *					- platform indipendent
 *
 *  \section system_overview Scheduling policy
 *
 *	On system initialization the uSmartX kernel builds a table of tasks to be executed. Task are scheduled on a priority 
 *  based fashion. The scheduling policy can be reduced to a simple round robin scheme if tasks have equal priorities.
 *  Each time a task is run a system event is passed to it signaling the task what action to take.
 *  Beeing a \e non-preemptive kernel context switching is performed only when a task returns. This is very important as
 *  a task can block the system if it never returns. When a task exits it passes a control event to the kernel. \n\n 
 *
 *	The uSmartX kernel ensures that the current running task has the highest priority and is in the \e ready state. If there are no
 *	ready tasks to run the scheduler returns with the \e idle code. It is up to the user to act on this event.
 *	A task could be suspended or delayed from execution. It could be explicitely delayed via a dedicated system call or it could
 *	be indirectly delayed due to a resource pending. \n\n
 *	
 *	Each time the task is run the kernel passes a event to it. Events are usualy responses to previous task's actions. 
 *	With the current kernel version the task could receive up to seven different events:
 *	-	\e \b SYS_NONE Task didn't receive any special event from the kernel.
 *	-	\e \b SYS_ERROR Task was pending on a resource and the resource was reset. An exaple of such a situation is when a task
 *		is pending on a semaphore and the semaphore is being reset. Please consult the specific topics for such situations.
 *	-	\e \b SYS_TOUT Task was delayed from execution and the timeout has expired
 *	-	\e \b SYS_MBX Task was pending on a mailbox an the maibox was posted within the timeout.
 *	-	\e \b SYS_MBX_TOUT Task was pending on a mailbox an the timeout has expired.
 *	-	\e \b SYS_SEM Task was pending on a semaphore an the semaphore was posted within the timeout.
 *	-	\e \b SYS_SEM_TOUT Task was pending on a semaphore an the timeout has expired.
 *	\n
 *	Special care must be taken when pending on resources. In a typical situation is when a task is pending on a mailbox and the timeout expires.
 *	Again, beeing a non-preemptive kernel the task must return even if the (mailbox) resource is not avaible at the moment.
 *	If a timeout was given when pending and if the timeout expires the task will be waked up by this event.
 *
 *  \section system_initialisation System initialisation
 *	
 *	On system startup the task table must be passed to the kernel. The kernel then builds a task table and initialises its internal
 *	strctures. The task table consist of three members:
 *	- Address of the task entry point
 *	- Address of the task control structure (TCB)
 *	- Task's priority. Note the priority can be altered later by a related system call.
 *  - Taks's name. Note this string must be null terminated.
 *
 *	Please note the table \b must be ended with a \e NULL entry. In this way the kernel figuers out the number of tasks. \n
 *	
 *	Before running the scheduler the tasks table must be passed to the \c uSMARTX_Init() function. Also interrupts should be 
 *  enabled if needed. Please see the example below.
 *	\code
 	#include <uSMARTX.h>
 	
 	TSK_CREATE(TSK1_tcb);
	TSK_CREATE(TSK3_tcb);
	TSK_CREATE(TSK2_tcb);
	
	STATUS TSK1(STATUS evt) {
		return SYS_OK;
	}
	STATUS TSK1(STATUS evt) {
		return SYS_OK;
	}
	STATUS TSK1(STATUS evt) {
		return SYS_OK;
	}
	
	task_entry_t task_tbl[] = {	{&TSK1, &TSK1_tcb, 1, "TASK1"},
								{&TSK2, &TSK2_tcb, 1, "TASK2"},
								{&TSK3, &TSK3_tcb, 1, "TASK3"},
								{0, 0} };
	
	int main(void) {
				
		uSMARTX_Init(task_tbl);
		.
		.
		.
		INT_Enable();
		.
		.
		.
	} 
 *	\endcode
 *
 *  \section system_scheduler Calling the scheduler
 *	
 *	The kernel is run by calling the uSMARTX_Scheduler() function. The scheduler always runs the task with the highest priority.
 *	When a task exits an event code is passed back to the kernel. It is up to the user to pass this code. This code will be then returned by
 *	the uSMARTX_Scheduler() function. In this way the user can act upon the event code by examining the returned value. Such an example could be
 *  resetting the system by passing SYS_ERROR, signalling a fatal error to the main loop.
 *  \code
 	
 	while(1) {		
		if(uSMARTX_Scheduler() == SYS_ERROR)
			break;
	}	
	SystemReset();
 
 *	\endcode
 *
 *  \section system_tick System timer tick
 *
 *  System calls dealing with timmed pending on resources need a system timer tick. The uSmartX kernel features an advanced system timer tick
 *  which allows system calls to be time dependant. Beside system calls, software timers relay on system timer tick too. All timers in the system
 *  are evaluated when the uSMARTX_Tick() function is executed. It is up to the user from where the function will be called. The only constraint
 *  is that the function must be executed with interrupts disabled. Usually this function is placed in an interrupt service rutine which is triggered
 *  by a timer or by an external event. The system timer tick resolution is determined by the frequency the function is called.
 *	
 *	All timers (pending system calls incorporate a timer) are enqueued in a relative timer queue. This means only one timer is evaluated. All others
 *  are relative to the first one which has the shortest expiration time. This increases performance drasticaly when dealing with multiple software timers.
 * 
 *	\warning If the system timer tick functionality is not needed, pending system calls musn't be called with a timeout value. \e NO_WAIT should
 *	be used for specifying the timeout. 
 *
 *  \section porting Porting guide 
 *
 *	uSmartX RTOS is highly portable. Most of the code is platform indipendant. The only functions needs to be ported are functions
 *	that controls enabling and disabling of interrupts. The function INT_Disable() returns the status of the global interrupt flag(s) just before
 *	the interrupts are disabled. These flags are used later when calling INT_Restore() to restore the status of interrupts before disabling them.
 *
 *	- INT_Enable() Enables interrupts.
 *	- INT_Disable() Disables interrupts and returns the interrupts enable flags before acctualy disabling them.
 *  - INT_Restore() Restores the state of interrupts to the value passed to it.
 *
 *	\subsection arm_port ARM7TDMI
 *	
 *	The ARM7TDI architecture has two bits in the CPSR register that controls interrupts. This is the F and I bit. Functions dealing with
 *	interrupts should manage these two bits accordingly to the interrupt usage. For example in fast interrupts aren't used there is no need
 *	to handle the F bit in the CPSR.
 *
 *	\subsection avr_port AVR
 *
 *	The AVR architecture disables/enables interrupts by clearing/setting the I bit in the SREG register.
 *
 *	\subsection h8_port H8
 *
 *	The H8 architecture disables/enables interrupts by
 *	clearing/setting the I bit in the CCR register.
 *
 *  \section examples Sample applications
 *	
 *	Plase see the related pages for some sample applications. The examples are to be used with an ATmega64 device
 *	running at 16 Mhz, but can be simply modified for some other CPU. In particular just the timer tick interrupt must be
 *	modified to suit the CPU in question.
 *
 */

/*! \page page1 Building uSmartX
 * 
 *	Building uSmartX means building a kernel library for a perticular target. The kernel can be then used when linking
 *	against a specific application. The toolchain used for building the library is the GCC toolchain. \n
 *	
 *	\section building_arm uSmartX for ARM
 *
 *	The \c makefile which build the kernel library is located in usmartx/arm/gcc/lib/. CD to this directory and execute \c make.
 *	After the build process the kernel library is located located in this directory under the \e libusmartx.a filename. Please see the
 *	provided examples on how to link with the library. The GCC toolcahin used to build the kernel can be obtained by visiting the 
 *  http://www.gnuarm.com web site.
 *
 *	\section building_avr uSmartX for AVR
 *	
 *	The \c makefile which build the kernel library is located in usmartx/avr/mega64/lib/. CD to this directory and execute \c make.
 *	After the build process the kernel library is located located in this directory under the \e libusmartx.a filename. Please see the
 *	provided examples on how to link with the library. Only the \c makefile for the ATmega64 is provided. For build the kernel library for
 *	an other AVR device just modify the makefile to refelct the desired cpu.
 *
 *	\section building_h8 uSmartX for H8
 *  
 *	The \c makefile which build the kernel library is located in usmartx/h8/3048/lib/. CD to this directory and execute \c make.
 *	After the build process the kernel library is located located in this directory under the \e libusmartx.a filename. Please see the
 *	provided examples on how to link with the library. The GCC toolchain used to build the kernel can be obtained by visiting 
 *	http://www.kpitgnutools.com web site.
 *
 */
 	
/*!	\example buffer.c
 */
/*!	\example timers.c
 */

/* Timer queue */
volatile queue_t g_timerq;			/* Timer queue */

/* Elapsed timer ticks since OS start */
volatile uint32 g_ticks;

/* Pointer to current running task */
volatile tcb_t *pcurr_ctx;

volatile queue_t g_taskq;		    /* Global tasks queue. All tasks waiting for CPU time are placed in this queue. */


/*!	\defgroup uSMARTX_api_system Kernel
 *	@{
 */
 
/*! \brief uSMARTX kernel initialisation function
 *
 *	The uSmartX kernel must initialise its internal structures and tasks. Priori to start the system and call the scheduler
 *	for the first time this function must be called.
 *  \param ptbl			pointer to tasks table
 *  \attention	 The initialisation function must be called before running the scheduler.
 */
void uSMARTX_Init(task_entry_t *ptbl) {
	tcb_t *ptcb;
	/* Init timers pointer */	
	g_timerq.pobject = 0;
	
	/* Reset system ticks */
	g_ticks = 0;
		 
	/* Init all task. The initial task state is normal running and without an active timer */
	while( ptbl->ph )	{
		ptcb = (tcb_t*) ptbl->ph;
		ptcb->flags = TSK_READY;
		ptcb->priority = ptbl->pri;
		ptcb->ptsk_fxn = ptbl->ptsk_fxn;
		ptcb->unMaxTime = 0;
		ptcb->unMinTime = 0xFFFF;
		ptcb->unAvgTime = 0;		
		ptcb->tic.flags = TMR_NOT_ACTIVE;		
		priority_enqueue_tsk( ptcb );
						
		ptbl++;
	}
	/* Assume the current running task is the task with the highest priority or the first
	 * task in the ready queue.
	 */
	pcurr_ctx = (tcb_t*) g_taskq.pobject;	
}

/*!	\brief Task scheduler
 * 	
 *
 *	\retval Task return value. It is up to the user to decide how to act on return values.
 */
STATUS uSMARTX_Scheduler(void) {
	size_t flags;
	tcb_t *ptcb;
	STATUS event;
	uint32 ulTicks;
	uint16 unTicks;
	
	if( !querry_queue((queue_t*) &g_taskq) )
		return SYS_IDLE;
	
	flags = INT_Disable();
	
	/* Get the next task from the priority based tasks queue */
	if( (ptcb = (tcb_t*) dequeue_top_object((queue_t*) &g_taskq) ) ) {
		pcurr_ctx = ptcb;		
		event = ptcb->event;
		/* Task is not anymore in any queue from that point so clear the pointer */
		ptcb->pqueue = NULL;
		
		INT_Restore( flags );
		
		/* Run the task and get the state of of the free running counter before adn after running the task
		 * to get the number of tick it was executing
		 */ 
		ulTicks = g_ticks;
		event = ptcb->ptsk_fxn(event);
		unTicks = g_ticks - ulTicks;
		
		if( ptcb->unMaxTime < unTicks )
			ptcb->unMaxTime = unTicks;
			
		if( ptcb->unMinTime > unTicks )
			ptcb->unMinTime = unTicks;
			
		/* If SYS_OK is returned the task must be put in the ready list, because it was not blocked in any other queue */
		flags = INT_Disable();
		ptcb->event = SYS_NONE;
		if( ptcb->flags == TSK_READY ) { {
			if( !ptcb->pqueue )				
				priority_enqueue_tsk( ptcb );
		}
			
		}
	}
	else 
		event = SYS_IDLE;
						
	INT_Restore( flags );
					
	return event;
}

/*!	\brief uSMARTX system tick
 *
 *	This function takes care of the kernel system tick evaluation. Put this function where you will be incrementing your system tick.
 *	Normally this is placed in a timer ISR rutine.
 *
 *	\attention This function must not be interrupted.
 */
void uSMARTX_Tick(void) {
	tic_t *ptic = (tic_t*) g_timerq.pobject;	
		
    g_ticks++;
           	
	/* If no active timer return. */
    if( !ptic )
    	return; 
    
    /* Decrement relative ticks. If zero dequeue the timer(s) from the list. */	
    if( --ptic->rel ) {    	
    	return;
    }
    		
	while( !ptic->rel ) {
		ptic = (tic_t*) dequeue_top_object( (queue_t*) &g_timerq );
		
		/* Clear the active flag and exec the callback function */
		ptic->flags &= ~TMR_ACTIVE;									
		ptic->pfxn(ptic->evt, ptic->parg1, ptic->parg2);
		
		/* If timer is of periodic type enqueue it again */
		if( ptic->flags & TMR_PERIODIC )
			timer_enqueue( ptic );
		
		/* Check if there are more elapsed timers */
		if( !(ptic = (tic_t*) g_timerq.pobject) )		
			return;
	}		
}
/*!	@}
 *
 * End of kernel group documentation.
 */

/*!	\defgroup uSMARTX_api_task Tasking
 *	@{  
 *	The uSMARTX kernel provides some basic control over task.
 */	
 
/*! \brief Get current running task handle
 *
 *	\retval current running task handle  
 */
HANDLE* TSK_Self(void) {
	return (HANDLE*) pcurr_ctx;
	
}
/*! \brief Task sleep function
 *
 *	Delay a task from execution for the specified amount of timer ticks. When the timeout expires the task is unblocked and is ready to execute.
 *	The \e TSK_TOUT value is passed to it.
 *  \param ph		handle to task
 *  \param tout 	timeout value  
 */
void TSK_Sleep(HANDLE *ph, uint16 tout) {
	tcb_t *ptcb = (tcb_t*) ph;
	ptcb->flags = TSK_DELAY;	
	TMR_Start(&ptcb->tic, tout, &tsk_delay_clb, 0, ptcb, 0, TMR_ONE_SHOT);		
}

/*! \brief Task suspend function
 *
 *	Suspend a task from execution, until the task is it explicitely resumed.
 *  \param ph	handle to task
 */
void TSK_Suspend(HANDLE *ph) {
	tcb_t *ptcb = (tcb_t*) ph;
	size_t flags;
			
	flags = INT_Disable();
	ptcb->flags = TSK_SUSPENDED;
	/* If the task is waiting in a queue dequeue it and stop its timer. */
	if( ptcb->pqueue ) {		
		dequeue_object(ptcb->pqueue, (dll_t*) ptcb);
		remove_timeout( &ptcb->tic );
		ptcb->pqueue = NULL;
	}
	INT_Restore( flags );	
}

/*! \brief Task resume function
 *
 *	Resume a previously suspended task.
 *  \param ph	handle to task 
 */
void TSK_Resume(HANDLE *ph) {
	size_t flags;
	tcb_t *ptcb = (tcb_t*) ph;
	
	flags = INT_Disable();
	
	if( ptcb->pqueue ) {		
		dequeue_object(ptcb->pqueue, (dll_t*) ptcb);
		remove_timeout( &ptcb->tic );
		ptcb->pqueue = NULL;
	}
	ptcb->flags = TSK_READY;
	ptcb->event = SYS_RESUMED;
	priority_enqueue_tsk( ptcb );
	INT_Restore( flags );
}
/*!	@}
 *
 * End of tasking group documentation.
 */
 
 
/*!	\defgroup uSMARTX_api_timer Timers
 *	@{
 *	In addition to time control over tasks the uSmartX kernel provides another way of time control via software timers. Each system tick
 *	software timers are evaluated. If one or more timers expires the tiemr associated call-back function is executed. the uSmartX kernel places
 *	timers in a so called relative time queue, where each timer expiration is relative to the preceeding timer. Timers that will expire at the
 *	same time have 0 relative ticks. In this way each system tick only one timer is evaluated. This increase performances drasticaly.
 *	Please note that the call-back function will be executed from uSMARTX_Tick() function context.
 *	\n\n
 *	The call back function must be of type:
 *	\code STATUS my_callback_func(uint8, void*, void*)
 *	\endcode
 */
 
/*! \brief Timer start function
 *
 *	This function starts a software timer. A valid call back function must be passed to the function. The timer is re-started if it was
 *	already started. Timers can be one shot or periodic.
 *  \param ph		handle to timer
 *  \param tout 	timeout value  
 *  \param pfxn		address of the callback function
 *  \param evt		event to be passed to the call-back function
 *  \param parg1	argument 1 to be passed to the call-back function
 *  \param parg2	argument 2 to be passed to the call-back function
 *  \param mode		one-shot or periodic mode
 *	\attention		The callback function will be executed from the context where uSMARTX_Tick function was called.
 */
void TMR_Start(HANDLE *ph, uint16 tout, STATUS(*pfxn)(uint8, void*, void*), uint8 evt, void *parg1, void *parg2, uint8 mode) {
	size_t flags;
	tic_t *ptic = (tic_t*) ph;

	flags = INT_Disable();
	
	/* If timer already active dequeue it first */
	if( ptic->flags & TMR_ACTIVE )
	  timer_dequeue( ptic );

    ptic->abs = tout;
	ptic->pfxn = pfxn;
	ptic->evt = evt;
	ptic->parg1 = parg1;
	ptic->parg2 = parg2;
	ptic->flags	|= mode;	
	timer_enqueue( ptic );
	
	INT_Restore( flags ); 
}
/*! \brief Timer re-start function
 *
 *	This function re-starts a software timer. Be careful to apply this system call only on an already
 *	started timer. Its functionality is very similar to TMR_Start() function but it is faster.
 *
 *  \param ph		handle to timer
 * 
 */
void TMR_ReStart(HANDLE *ph) {
	size_t flags;
	tic_t *ptic = (tic_t*) ph;
	
	flags = INT_Disable();
	
	/* If timer already active dequeue it first */
	if( ptic->flags & TMR_ACTIVE )
	  timer_dequeue( ptic );
	  
	timer_enqueue( ptic );
	
	INT_Restore( flags );
}

/*! \brief Timer stop function
 *
 *	This function stops a software timer without executin its call-back function.
 *  \param ph		handle to timer
 */
void TMR_Stop(HANDLE *ph) {
	size_t flags;
	tic_t *ptic = (tic_t*) ph;

	flags = INT_Disable();
	
	if( ptic->flags & TMR_ACTIVE )
		timer_dequeue( ptic );
		
	INT_Restore( flags ); 
}

/*! \brief Return system ticks
 *
 *	This function returns elapsed system ticks since system startup.
 *	\retval elapsed system ticks
 */
uint32 TMR_GetTicks(void) {
	uint32 tmp1, tmp2; 
 
	tmp1 = g_ticks;
	tmp2 = g_ticks;
 
	while(tmp2 - tmp1) {
  		tmp1 = g_ticks;
  		tmp2 = g_ticks;
	}

	return tmp2;
}

///*! \brief Get free CPU running counter
// *
// *	This function is called from the scheduler to get the curret ticks of a CPU timer.
// *  The implementation of this function is up to the user. Please see the Porting guide section for more details.
// *
// *	\retval value of the CPU timer in 16 bit resolution
// */
//extern uint16 TMR_GetFreeRunningCounter(void);

/*!	@}
 *
 * End of timers group documentation.
 */
//------------------------------------------------------------------------------
/*						INTERNAL FUNCTIONS		
 *
 * These are uSMARTX internal function. They must not be called from a task.
 */
 
 
 /* Insert a timer in a timer the queue */
void timer_enqueue(tic_t *ptic) {
	uint16 ticks;
	tic_t *t;
	
    ptic->flags |= TMR_ACTIVE;
	ticks = ptic->abs;
	if( (t = (tic_t*) g_timerq.pobject) ) {
		while( t->rel <= ticks) {
			ticks -= t->rel;
			if( t->pnxt )			
				t = (tic_t*) t->pnxt;
			else {
				ptic->rel = ticks;
				enqueue_bottom_object( (queue_t*) &g_timerq, (dll_t*) ptic );
				return;
			}
		}	
		ptic->rel = ticks;
		t->rel -= ticks;		
		if( t->pprv )
			enqueue_middle_object( (dll_t*) t, (dll_t*) ptic );			
		else 
			enqueue_top_object( (queue_t*) &g_timerq, (dll_t*) ptic );
	}
    else {
    	ptic->rel = ticks;
	    enqueue_top_object( (queue_t*) &g_timerq, (dll_t*) ptic );
    }
}

/* Dequeue a timer from timer list */
void timer_dequeue(tic_t *ptic) {
	ptic->flags &= ~TMR_ACTIVE;
	/* If this is not the last timer add the relative ticks to the next timer */
	if( ptic->pnxt )		
		((tic_t*)ptic->pnxt)->rel += ptic->rel;
	
	dequeue_object( (queue_t*) &g_timerq, (dll_t*) ptic );
}

/* Append a timer to the task */
void append_timeout(tcb_t *ptcb, STATUS (*pfxn)(uint8 evt, void *parg1, void *parg2), uint16 tout) {
	tic_t *ptic;

	if( tout == WAIT_FOREVER )
		return;

	ptic = &ptcb->tic;
	ptic->flags	|= TMR_ONE_SHOT;
	ptic->evt = 0;
	ptic->parg1 = ptcb;
	ptic->parg2 = 0;
	
	ptic->abs = tout;
	ptic->pfxn = pfxn;		
	timer_enqueue( ptic );
}
/* Remove a pending timer */
void remove_timeout(tic_t *ptic) {
	if( ptic )
		if( ptic->flags & TMR_ACTIVE)
			timer_dequeue( ptic );
}

/* Task delay call back function */
STATUS tsk_delay_clb(uint8 evt, void *parg1, void *parg2) {		
	tcb_t *ptcb = (tcb_t*) parg1;
	ptcb->event = SYS_TOUT;
	ptcb->flags	= TSK_READY;
	priority_enqueue_tsk( ptcb );
	return SYS_OK;			
}

/* Enqueue tasks on a priority based scheme */
void priority_enqueue_tsk(/*queue_t *pqueue, */tcb_t *ptcb) {
	tcb_t *t;
	uint8 p	= ptcb->priority;
	queue_t *pqueue = (queue_t*) &g_taskq;
	
	ptcb->pqueue = (queue_t*) &g_taskq;
	
	if( pqueue->pobject ) {
		t = (tcb_t*) pqueue->pobject;
		while( t->priority <= p  ) 		{
			if( t->pnxt )
				t = (tcb_t*) t->pnxt;
			else {
				enqueue_bottom_object(pqueue, (dll_t*) ptcb);
				return;
			}
		}
		if( t->pprv ) {	
			enqueue_middle_object((dll_t*) t, (dll_t*) ptcb);
			return;
		}
	}	
	enqueue_top_object( pqueue, (dll_t*) ptcb );
}

/* Get current running task tcb */
tcb_t* get_curr_tsk(void) {
	return (tcb_t*)pcurr_ctx;	
}
