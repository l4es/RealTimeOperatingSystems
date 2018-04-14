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
#include <semaphore.h>

/*! \defgroup uSMARTX_api_semaphore Semaphore 
 *	@{
 *	Semaphores are a mechanism for inter-task communication and interrupt-task communication.
 *  Each semaphore has it own semaphore control block which is used by system calls to handle
 *	the caller request associated with the semaphore. Tasks and interrupts can post a semaphore, but only
 *	tasks are allowed to pend for a semaphore. A timeout value can be passed to the pend function. Multiple
 *  posts increment the semaphore count, but only up to the semaphore maximum count. This value is specified
 *  when the sempahore is created. If the count is 0 the semaphore isn't avaible.
 */

/*! \brief Semaphore pend function
 *
 *	A task can pend for a semaphore. If the semaphore count is greater than 0 the count is decremented
 *	and the function returns with \e SYS_OK. If the count is zero the function returns with \e SYS_ERROR. When a timeout
 *	value is specified, a timer is started and the task is delayed from execution for the timeout period. If in the mean
 *	time the semaphore is posted the \e SYS_SEM value is passed to the task, otherwise the task receives a \e SYS_SEM_TOUT
 *	entry value. 
 *	
 *  \param ph		handle to semaphore
 *  \param tout 	timeout value  
 *	\retval	SYS_OK semaphore count was > 0
 *	\retval SYS_ERR semaphore count was 0
 *  \attention Only tasks can pend on a semaphore.
 */
STATUS SEM_Pend(HANDLE *ph, uint16 tout) {
	tcb_t *ptcb;
	scb_t *pscb;
	size_t flags;
	
	pscb = (scb_t*) ph;
	ptcb = get_curr_tsk();

	flags = INT_Disable();
	
	/* If resources not availbe put the calling task on the pending list */
	if( !pscb->cnt ) {		
		if( tout == NO_WAIT ) {
    		INT_Restore( flags );
    	    return SYS_ERROR;
    	}
		
		ptcb->pqueue = (queue_t*) &pscb->queue;
		ptcb->flags = TSK_SEM_PEND;
		
		enqueue_bottom_object(&pscb->queue, (dll_t*) ptcb);
		append_timeout(ptcb, &sem_tout_clb, tout);
				    	     	
   	   	INT_Restore( flags );
   	    return SYS_ERROR;				
	}
	
	pscb->cnt--;
	INT_Restore( flags );
	return SYS_OK;	
}
/*! \brief Semaphore post function
 *
 *	A task or interrupt ISR can post a semaphore. If there was no other task waitnig for the semaphore the semaphore
 *	count is incremented by one, otherwise the first waiting task is unblocked. Also any timer
 *	associated with the pending task is removed. The value \e SYS_SEM is passed to the newly dequeued task.
 * 
 *  \param ph		handle to semaphore
 */
void SEM_Post(HANDLE *ph) {
	tcb_t *ptcb;
	scb_t *pscb;
	size_t flags;
	
	pscb = (scb_t*) ph;

	flags = INT_Disable();	
		
	if( (ptcb = (tcb_t*) dequeue_top_object( (queue_t*) &pscb->queue )) ) {				
		ptcb->flags	= TSK_READY;
		ptcb->event = SYS_SEM;
		remove_timeout( &ptcb->tic );
		priority_enqueue_tsk( ptcb );			
	}
	else {
		if( pscb->cnt < pscb->max_cnt )
			pscb->cnt++;
	}	
	INT_Restore( flags );	
}
/*! \brief Semaphore reset function
 *
 *	This function resets the semaphore count to the specified value. All pending tasks are dequeud and
 *	\e SYS_ERROR is passed to them.
 * 
 *  \param ph		handle to semaphore
 *	\param cnt	 	initial semaphore count
 */
void SEM_Reset(HANDLE *ph, uint8 cnt) {
	tcb_t *ptcb;
	scb_t *pscb;
	size_t flags;
	
	pscb = (scb_t*) ph;

	flags = INT_Disable();
     
    while( (ptcb = (tcb_t*) dequeue_top_object( (queue_t*) &pscb->queue)) ) {
        ptcb->flags	= TSK_READY;
        ptcb->event = SYS_ERROR;
		remove_timeout( &ptcb->tic );
		priority_enqueue_tsk( ptcb );	
    }    
    pscb->cnt = cnt;    
    INT_Restore( flags );
}
/*! \brief Semaphore querry function
 *
 *	This function returns the acctual semaphore count.
 * 
 *  \param ph		handle to semaphore
 */
uint8 SEM_Querry(HANDLE *ph) {
	scb_t *pscb;	
	pscb = (scb_t*) ph;
	return pscb->cnt;
}

/*!	@}*/
//------------------------------------------------------------------------------
/*						INTERNAL FUNCTIONS		
 *
 * These are uSMARTX internal function. They must not be called from a task.
 */
/*! \internal
 *	\brief Semaphore timeout callback function.
 *
 *	If it is called a semaphore timeout has occured. Remove the task from the semaphore queue
 *	(full/empty) and clear delay flag.
 *  \param evt		not used
 *	\param parg1	pointer to tcb which requested a timeout
 *	\param parg1	not used
 *	\return SYS_OK
 */
STATUS sem_tout_clb(uint8 evt, void *parg1, void *parg2) {	
	tcb_t *ptcb = (tcb_t*) parg1;
	ptcb->event = SYS_SEM_TOUT;
	ptcb->flags	= TSK_READY;
	dequeue_object( ptcb->pqueue, (dll_t*) ptcb);
	priority_enqueue_tsk( ptcb );
	return SYS_OK;
}
