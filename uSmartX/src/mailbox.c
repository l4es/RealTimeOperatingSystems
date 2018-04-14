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
#include <mailbox.h>
#include <string.h>

/*! \defgroup uSMARTX_api_mailbox Mailbox 
 *	@{
 *	Mailboxes are a mechanism for inter-task communication and interrupt-task communication.
 *	Each mailbox has its own mailbox control structure.
 *	Different tasks and interrupts can post a mailbox, but only one task can pend on a certain mailbox.
 *	Tasks can pend or post a mailbox with a timeout value.
 */

/*!  \brief Mailbox initialisation function
 *
 *	A mailbox must be initialised before its usage. Calling this function initialises the mailbox.
 *  \param ph		handle to mailbox
 *  \attention	 The initialisation function must be called before running the scheduler.
 */
void MBX_Init(HANDLE *ph) {
	uint8 i;
	uint8 *pdata;
	mcb_t *pmcb;

	pmcb = (mcb_t*) ph;
	
	/* Build a dll list containing nmsg elements and place them in the free list */		
	for(i = 0; i < pmcb->nmsg; i++)
		enqueue_top_object( (queue_t*) &pmcb->free_list, (dll_t*) &pmcb->pmsg[i] );				 
	
	
	/* Each msg_t structure contains a pointer, which points to the message buffer. The code below
	 * initializes this pointer.
	 */
	pdata = pmcb->pbuff;
	for(i = 0; i < pmcb->nmsg; i++) {
		pmcb->pmsg[i].pdata = pdata;
		pdata = pdata + pmcb->len;
	}	
}

/*! \brief Mailbox post function
 *
 *	A mailbox can be post from an interrupt ISR or from a task. When posted from a task a timeout value can be 
 *  specified. \n
 *	If a mailbox has a empty message container the message is stored in the mailbox and the system call returns
 *	with SYS_OK. When there is no empty message containers the task is pending and SYS_ERROR is returned. If the caller 
 *	specifed a timeout value a timer is started and the task is delayed from execution. After the timeout expires
 *	(no task freed a message container) the task is unblocked and \e SYS_MBX_TOUT is passed to the task's entry 
 *	parameter. If a task freed a message container the \e SYS_MBX value is passed to the task.
 *  \param ph		handle to mailbox
 *  \param pmsg 	pointer caller to message
 *  \param tout 	timeout value  
 *	\retval SYS_OK if there was a free message container
 *	\retval SYS_ERR if all message containers are full
 */
STATUS MBX_Post(HANDLE *ph, void *pmsg, uint16 tout) {		
	tcb_t *ptcb;
	mcb_t *pmcb;
    msg_t *pmsg_entry;        
	size_t flags;

	ptcb = get_curr_tsk();
	pmcb = (mcb_t*) ph;
	
	flags = INT_Disable();
	
	/* Check if the task was given an entry by the MBX_Pend function. If it was take this entry and not the one
	 * in the free_list. This is needed beacause a MBX_Pend caused a free container in the free_list. As soon this container
	 * become avaible we must give it to the first waiting task. Otherwise we could dequeue a task, make it runable, but an other task
	 * could take its container. So this would be an invalid situation.
	 */
	if( !ptcb->pmsg_entry ) {			
		/* Wait for a message entry to become free*/
    	if( !(pmsg_entry = (msg_t*) dequeue_top_object( (queue_t*) &pmcb->free_list)) ) {
    		if( tout == NO_WAIT ) {
    			INT_Restore( flags );
    	        return SYS_ERROR;
    		}
    	    ptcb->pqueue = (queue_t*) &pmcb->fullq;
    	    ptcb->flags = TSK_MBX_PEND;
			
			enqueue_top_object(&pmcb->fullq, (dll_t*) ptcb);
    	    append_timeout(ptcb, &mbx_tout_clb, tout);
    	
    	   	INT_Restore( flags );
    	    return SYS_ERROR;
		}
	}
	else {
		/* If the task was given an msg entry operate on that entry. */
		pmsg_entry = ptcb->pmsg_entry;
		ptcb->pmsg_entry = 0;
	}

	/* Copy the message data into the container */
	memcpy( pmsg_entry->pdata, pmsg, pmcb->len);

	/* Move the message container to the full list. */
	enqueue_bottom_object( &pmcb->full_list, (dll_t*) pmsg_entry );	
	         
	/* Check if there was a task waiting for a message. If it was remove its timeout timer.*/
	if( (ptcb = (tcb_t*) dequeue_top_object( (queue_t*) &pmcb->emptyq)) ) {										
		ptcb->flags	= TSK_READY;
		ptcb->event = SYS_MBX;
		priority_enqueue_tsk( ptcb );
		remove_timeout( &ptcb->tic );
	}

	INT_Restore( flags );        	
	return SYS_OK;	
}
/*! \brief Mailbox pend function
 *
 *	Only tasks can pend on mailboxes. If a mailbox is not empty the message is returned to the caller and the system call
 *	returns with SYS_OK. When there is no message the task is pending and SYS_ERROR is returned. If the caller 
 *	specifed a timeout value a timer is started and the task is delayed from execution. After the timeout expires
 *	(no task or interrupt posted a message) the task is unblocked and \e SYS_MBX_TOUT is passed to the task's entry 
 *	parameter. If somebody posted a message the timer is removed and the \e SYS_MBX values is passed to the task.
 *  \param ph		handle to mailbox
 *  \param pmsg 	pointer caller where the message will be copied
 *  \param tout 	timeout value 
 *	\retval SYS_OK if there was a message in mailbox
 *	\retval SYS_ERR if there was no messages in mailbox
 *  \attention MBX_pend can not be called from an ISR.
 */
STATUS MBX_Pend(HANDLE *ph, void *pmsg, uint16 tout) {	
	tcb_t *ptcb;
	mcb_t *pmcb;
    msg_t *pmsg_entry;        
	size_t flags;
	
	ptcb = get_curr_tsk();
	pmcb = (mcb_t*) ph;

	flags = INT_Disable();
	
	/* Check if there is a message already in the mailbox */
	if( !(pmsg_entry = (msg_t*) dequeue_top_object( (queue_t*) &pmcb->full_list)) ) {
		if( tout == NO_WAIT ) {
    		INT_Restore( flags );
            return SYS_ERROR;
    	}
        ptcb->pqueue = (queue_t*) &pmcb->emptyq;
        ptcb->flags = TSK_MBX_PEND;
		
		enqueue_top_object(&pmcb->emptyq, (dll_t*) ptcb);
		append_timeout(ptcb, &mbx_tout_clb, tout);
          
       	INT_Restore( flags );
        return SYS_ERROR;
	}
	/* Copy message to task */ 
	memcpy( pmsg, pmsg_entry->pdata, pmcb->len );
	
	/* Check if there was a task waiting for an empty entry in the mailbox */
	if( (ptcb = (tcb_t*) dequeue_top_object( (queue_t*) &pmcb->fullq)) ) {
		ptcb->flags	= TSK_READY;
		ptcb->event = SYS_MBX;
		ptcb->pmsg_entry = pmsg_entry;
		priority_enqueue_tsk( ptcb );
		remove_timeout( &ptcb->tic );		
	}
	else {
		enqueue_bottom_object( (queue_t*) &pmcb->free_list, (dll_t*) pmsg_entry );
	}
    INT_Restore( flags );
    return SYS_OK;
}

/*! \brief Mailbox flush function
 *
 *	Flush a mailbox and unblock all pending tasks that were waiting for free message containers. \e SYS_ERROR
 *	is passed to the task.
 *  \param ph		handle to mailbox
 */
void MBX_Flush(HANDLE *ph) {	
	tcb_t *ptcb;
	mcb_t *pmcb;
	msg_t *pmsg_entry;
    size_t flags;
	
	pmcb = (mcb_t*) ph;
	
	flags = INT_Disable();
	
	/* Empty all containers in the FullList. If there is a task waiting for an empty container give the dequeued
	 * container to it. If not put the container in the free list.
	 */
	while( (pmsg_entry = (msg_t*) dequeue_top_object( (queue_t*) &pmcb->full_list)) ) {
		if( (ptcb = (tcb_t*) dequeue_top_object( (queue_t*) &pmcb->fullq)) ) {
			ptcb->flags	= TSK_READY;
			ptcb->event = SYS_ERROR;
			ptcb->pmsg_entry = pmsg_entry;
			remove_timeout( &ptcb->tic );
			priority_enqueue_tsk( ptcb );					
		}
		else
			enqueue_bottom_object( (queue_t*) &pmcb->free_list, (dll_t*) pmsg_entry );		
	}	
	INT_Restore( flags ); 
}
/*!	@}*/
//------------------------------------------------------------------------------
/*						INTERNAL FUNCTIONS		
 *
 * These are uSMARTX internal function. They must not be called from a task.
 */
/*! \internal
 *	\brief Mailbox timeout callback function.
 *
 *	If it is called a mailbox timeout has occured. Remove the task from the mailbox queue
 *	(full/empty) and clear delay flag.
 *  \param evt		not used
 *	\param parg1	pointer to tcb which requested a timeout
 *	\param parg1	not used
 *	\return SYS_OK
 */
STATUS mbx_tout_clb(uint8 evt, void *parg1, void *parg2) {
	tcb_t *ptcb = (tcb_t*) parg1;
	ptcb->event = SYS_MBX_TOUT;
	ptcb->flags	= TSK_READY;	
	dequeue_object( ptcb->pqueue, (dll_t*) ptcb);
	priority_enqueue_tsk( ptcb );
	return SYS_OK;
}
