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
 
#ifndef _uSMARTX_H_
#define _uSMARTX_H_

#include <stddef.h>

/*! \addtogroup uSMARTX_api_system
 *	@{
 */
 
typedef signed char			int8;
typedef unsigned char		uint8;
typedef signed short		int16;
typedef unsigned short		uint16;
typedef	signed long			int32;
typedef unsigned long		uint32;

#define HANDLE	void

#ifndef NULL
#define NULL	0
#endif
 	
/*! \brief System calls return and task entry codes */
typedef enum {	
	SYS_OK,				/*!< System call or scheduler returned successefuly */
	SYS_ERROR,			/*!< System call or scheduler returned with error (resource not avaible) */
	SYS_IDLE,			/*!< Scheduler returned in idle state, meaning there was no ready task to run */
	SYS_MBX,			/*!< Mailbox was posted or freed */
	SYS_SEM,			/*!< Semaphore was posted */
	SYS_RESUMED,		/*!< Task was explicitely resumed */
	
	SYS_TOUT,			/*!< Task sleep timeout expired */
	SYS_MBX_TOUT,		/*!< Task timeout expired when pending on a mailbox */
	SYS_SEM_TOUT,		/*!< Task timeout expired when pending on a semaphore */
	SYS_NONE			/*!< Task didn't recived any system events */
			
} STATUS;

/*! \brief Startup task table
 *
 *  On startup a table of task properities is passed to the system. Each entry in the table contains: \n
 * - address of the task entry point
 * - address of the TCB structure
 * - task's priority
 *
 * \warning The table must end with a null entry.
 */
 
typedef struct {
	STATUS (*ptsk_fxn)(STATUS event);	/*!< Address of the task entry point */
	HANDLE *ph;							/*!< Handle to task task control block structure (TCB) */
	uint8 pri;							/*!< Task's priority */
	uint8 TaskName[10];					/*!< Task's name */
	
} task_entry_t;

/*!	@} */
 
typedef struct dll_s {

	struct dll_s *pnxt;  /* Pointer to \e next object in queue. */
	struct dll_s *pprv;  /* Pointer to \e previous object in queue. */
} dll_t;

typedef struct queue_s {
	dll_t *pobject;			// pointer to first object in queue
	dll_t *plast;			// pointer to last object in queue
} queue_t;

typedef struct msg_s {
	dll_t *pnxt;			
	dll_t *pprv;			
	void *pdata;			/* pointer to data buffer */
} msg_t;

typedef struct mcb_s {   	
	queue_t fullq;		    /* Actual mailbox full queue */
    queue_t emptyq;		    /* Actual mailbox empty queue */
	queue_t free_list;		/* List of free containers */
	queue_t full_list;		/* List of full containers */
	uint8 *pbuff;			/* Pointer to messages buffer */
	msg_t *pmsg;			/* Pointer to first message descriptor */
	uint8 len;				/* Message length */
	uint8 nmsg;				/* Number off messages in mailbox */

} mcb_t;

typedef struct scb_s {
	queue_t queue;			/* Queue on which tasks are pending */
	uint8 cnt;				/* Semaphore count */
	uint8 max_cnt;			/* Maximum semaphore count */
	
} scb_t;

typedef struct buf_s {	
	void *pin;			/* Pointer to the newest data in buffer */
	void *pout;			/* Pointer to the oldest data in buffer */
	void *ptop;			/* Pointer to the top of the buffer */
	void *pbot;			/* Pointer to the bottom of the buffer */
	uint16 n;			/* Number of elements currently in buffer */
	uint16 trg;			/* Numbers of elements when the trigger function will be fired */
	uint8 len;			/* Entry data length */
	uint8 btrgfxn;		/* Trigger trg callback just once */
	STATUS (*ptrgfxn)(uint8 evt, void *parg1, void *parg2); /* Pointer to the trigger function */
	struct tic_s *ptic;	/* Pointer to timer structure. */		
			
} bcb_t;

/*!	\addtogroup uSMARTX_api_timer
 *	@{
 */

/*!	\brief Create a software timer
 *
 *	This macro creates a timer
 *
 *	\hideinitializer
 */
#define TMR_CREATE(name)	tic_t name

 
/*!	\brief Timeout period
 *
 *	When a system call's timeout parameter has this value the system call won't return until the resource will be avaible.
 */
#define WAIT_FOREVER	0xFFFF

/*!	\brief Timeout period
 *
 *	When a system call timeout parameter has this value the system call won't wait the resource but  will return immediately even if
 *	the resource isn't avaible. 
 */
#define NO_WAIT			0
 
/*!	\brief Software timer mode
 *
 *	The started SW timer will run in periodic mode. Once the timeout will expire and the call-back function will be executed
 *	the task will be re-started again automaticaly.
 */
#define TMR_PERIODIC	1

/*!	\brief Software timer mode
 *
 *	The started SW timer will run in one shot mode. Once the timeout will expire and the call-back function will be executed. 
 *	The timer won't be restarted automaticaly.
 */
#define TMR_ONE_SHOT	2
/*!	@}*/

#define TMR_NOT_ACTIVE	4
#define TMR_ACTIVE		8


typedef struct tic_s {
	struct dll_s *pnxt;
	struct dll_s *pprv;

	uint16 abs;			/* Absolute ticks */
	uint16 rel;			/* Relative ticks to next timer in queue */
	
	/* Callback function of timer. It is called with three parameters:
	 * - event number
	 * - first argument
	 * - second argument
	 */
	STATUS (*pfxn)(uint8 evt, void *parg1, void *parg2);
	uint8 evt;
	void *parg1;
	void *parg2;
	
	uint8 flags;		/* timer flags */
	
} tic_t;

enum {	
	TSK_READY		= 0,	/*!< Task is ready to run and is waiting kernel gives him time */
	TSK_DELAY,				/*!< Task is delayed from execution */
	TSK_SUSPENDED,			/*!< Task is suspended */
	TSK_MBX_PEND,			/*!< Task is pending on a mailbox (free on empty) */
	TSK_SEM_PEND			/*!< Task is pending on semaphore */
};      

typedef struct tcb_s {
	struct dll_s *pnxt;
	struct dll_s *pprv;
	
	uint8 flags;	
	uint8 priority;
	STATUS event;
	STATUS (*ptsk_fxn)(STATUS event);
		
	struct msg_s *pmsg_entry;
	struct queue_s *pqueue;
	struct tic_s tic;
	
	uint16 unMaxTime;		/*!< Maximum time the task was executing */
	uint16 unMinTime;		/*!< Minimum time the task was executing */
	uint16 unAvgTime;		/*!< Long term average time the task was executing */
			
} tcb_t;

/*!	\addtogroup uSMARTX_api_task
 *	@{
 */

/*!	\brief Create a task
 *
 *	This macro creates a task.
 *	\hideinitializer
 *
 */
#define TSK_CREATE(name) tcb_t name

/*!	@} */



/* API system call */
void uSMARTX_Init(task_entry_t *ptbl);
STATUS uSMARTX_Scheduler(void);
void uSMARTX_Tick(void);

void TSK_Sleep(HANDLE *ph, uint16 tout);
void TSK_Suspend(HANDLE *ph);
void TSK_Resume(HANDLE *ph);
HANDLE* TSK_Self(void);

void TMR_Start(HANDLE *ph, uint16 tout, STATUS(*pfxn)(uint8, void*, void*), uint8 evt, void *parg1, void *parg2, uint8 mode);
void TMR_ReStart(HANDLE *ph);
void TMR_Stop(HANDLE *ph);
uint32 TMR_GetTicks(void);


/*!	\addtogroup uSMARTX_api_system
 *	@{
 */

/*! \brief Globaly disable interrupts
 *
 *	This function globaly disables interrupts and returns the state of global interrupts flags before disabling interrupts.
 *	This value can later be used by calling INT_Restore() function to restore the state of interrupts disable flags.
 *
 *  \retval size_t status of global interrupt disable flags befor disabling interrupts
 *  \attention Please refer to specific architecture implementation to see which interrupts are being disabled.
 */
size_t INT_Disable(void);

/*! \brief Globaly enable interrupts
 *
 *	This function globaly enables interrupts.
 *
 *  \attention Please refer to specific architecture implementation to see which interrupts are being enabled.
 */
void INT_Enable(void);

/*! \brief Restore status of interrupts
 *
 *	This function restore the status of interrupts to the given state. Usually this function is used in conjunction
 *	with INT_Disable() function.
 *
 *  \param flags value to which restore the status of global interrupts flags
 *  \attention Please refer to specific architecture implementation to see which interrupts are being enabled/disabled.
 */
void INT_Restore(size_t flags);

/*!	@} */


/* Internal functions */

void timer_enqueue(tic_t *ptic);
void timer_dequeue(tic_t *ptic);
void remove_timeout(tic_t *ptic);
void append_timeout(tcb_t *ptcb, STATUS (*pfxn)(uint8 evt, void *parg1, void *parg2), uint16 tout);

void priority_enqueue_tsk(/*queue_t *pqueue, */tcb_t *ptcb);

tcb_t* get_curr_tsk(void);

STATUS tsk_delay_clb(uint8 evt, void *parg1, void *parg2);

#endif
