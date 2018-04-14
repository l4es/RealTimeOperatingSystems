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
 
//-------------------------------------------------------------------------------
// queue.c
//-------------------------------------------------------------------------------
//                              Include
//-------------------------------------------------------------------------------
#include <usmartx.h>
#include <queue.h>
//------------------------------------------------------------------------------
void enqueue_top_object(queue_t *pqueue, dll_t *pobject) {
	if( pqueue->pobject ) {
		pobject->pnxt =  pqueue->pobject;
		pqueue->pobject->pprv = pobject;		
	}
	else {
		pobject->pnxt = 0;
		pqueue->plast = pobject;
	}
	pobject->pprv = 0;
  	pqueue->pobject = pobject;
}
//------------------------------------------------------------------------------
// pNxt -> pointer to next object
// pObject -> pointer to object to be enqueued
void enqueue_middle_object(dll_t *pnxt, dll_t *pobject) {
	pnxt->pprv->pnxt = pobject;
	pobject->pprv = pnxt->pprv;

	pnxt->pprv = pobject;
	pobject->pnxt = pnxt;
}
//------------------------------------------------------------------------------
// pQueue -> pointer to queue head
// pObject -> pointer to object to be enqueued
void enqueue_bottom_object(queue_t *pqueue, dll_t *pobject) {
		
	if( pqueue->plast ) {		
		pqueue->plast->pnxt = pobject;
	   	pobject->pprv = pqueue->plast;	   	
	}
	else {
		pobject->pprv = 0;		//		EnqueueTopObject( pQueue, pObject );
		pqueue->pobject = pobject;	   
	}
	pobject->pnxt = 0;				 
	pqueue->plast = pobject;				// remember last object	
}
//------------------------------------------------------------------------------
// pQueue -> pointer to queue head
// pObject -> returned dequeued object
dll_t* dequeue_top_object(queue_t *pqueue) {
    dll_t *pobject;
	if( pqueue->pobject ) {					//if queue not empty
       	pobject = pqueue->pobject;
		if( pqueue->pobject->pnxt ) {			//if more that one object
            pqueue->pobject = pqueue->pobject->pnxt;
			pqueue->pobject->pprv = 0;
		}
		else {
			pqueue->pobject = 0;
			pqueue->plast = 0;
		}
		return pobject;
	}
	return 0;
}
//------------------------------------------------------------------------------
// pQueue -> pointer to queue head
// pObject -> pointer to object to be dequeued
void dequeue_object(queue_t *pqueue, dll_t *pobject) {
  	
	if( pobject->pprv )                       // if not first object
        pobject->pprv->pnxt = pobject->pnxt;     	
	else {
		pqueue->pobject = pobject->pnxt;
	}

    if( pobject->pnxt )						// if not last object
    	pobject->pnxt->pprv = pobject->pprv;
	else 
		pqueue->plast = pobject->pprv;
}
//------------------------------------------------------------------------------
// pQueue -> pointer to queue head
uint8 querry_queue(queue_t *pqueue) {
	if( pqueue->pobject )
		return 1;
		
	return 0;
}

