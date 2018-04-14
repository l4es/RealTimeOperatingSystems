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
 
/*! \defgroup uSMARTX_api_cyc_buffer Fifo buffer 
 *	@{
 *	Buffers	structures implemented in uSmartX are standard cyclic fifo buffers with some additional
 *	features. Basicaly they are very similar to mailboxes with the exception that a tasks could't pend
 *	on it when they are requesting or putting data into it. The buffer strucure is not fixed but,
 *	it is defined at compile time, particulary the buffer size and the element size. Other parameters of
 *	the buffer can be changed in run time, although some default values are entered at compile time.
 *	
 *	Buffers have two event functions associated. One of them is invoked when the buffer fills to a certain
 *	level and the second one is executed when the buffer contains unread data which is older that the 
 *	specified timeout.
 *	 
 */
 
#include <string.h>
#include <usmartx.h>
#include <queue.h>
#include <cycbuff.h>

//----------------------------------------------------------------------------
/*!	\brief Put data into the buffer
 *
 *	This function writes data into the buffer if there is enough free space. If there is a
 *	trigger function set and the function has reached the trigger level the trigger callback
 *	function is executed. Note the execution context is the same as BUF_Put. Each write access 
 *	resets the timeout callback function timeout.
 *
 *  \param ph	handle to buffer
 *	\param pdata pointer to data to write in
 *	\retval SYS_OK	data was stored into the buffer
 *	\retval SYS_ERROR	there was no enough free space
 */
STATUS BUF_Put(HANDLE *ph, void *pdata) {
	size_t flags;
	uint8 *p;
	uint8 bexetrg = 0;
	bcb_t *pbcb = (bcb_t*) ph;
	
	flags = INT_Disable();
	
	p = pbcb->pin;
	p += pbcb->len;
	if( p >= (uint8*) pbcb->pbot )
		p = pbcb->ptop;
	
	if( p == pbcb->pout ) {
		INT_Restore( flags );
		return SYS_ERROR;
	}

	memcpy( pbcb->pin, pdata, pbcb->len);	
	pbcb->pin = p;	
	pbcb->n++;
	
	/* Check if we reached the trigger level. If yes fire the trigger function only once. Set the variable to execute
	 * the trigger function. This is needed as we will execute it in the BUF_Put context.
	 */
	if( pbcb->n >= pbcb->trg && pbcb->btrgfxn ) {
		pbcb->btrgfxn = 0;
		bexetrg = 1;
	}	
	TMR_ReStart(pbcb->ptic);
	
	INT_Restore( flags );
					
	if( bexetrg )
		pbcb->ptrgfxn(0, pbcb, 0);	
			
	return SYS_OK;
}
//----------------------------------------------------------------------------
/*!	\brief Get data from buffer
 *
 *	If there is some data in the buffer this function retrieves it. Each read access on the
 *	buffer also reset the associated timeout callback function timeout. 
 *
 *  \param ph	handle to buffer
 *	\param pdata pointer to where the data will be written
 *	\retval SYS_OK data was retrived from buffer
 *	\retval SYS_ERROR there was no data in buffer
 */
STATUS BUF_Get(HANDLE *ph, void *pdata) {
	size_t flags;
	uint8 *p;
	bcb_t *pbcb = (bcb_t*) ph;
	
	flags = INT_Disable();

	if( pbcb->pin != pbcb->pout ) {
		memcpy(pdata, pbcb->pout, pbcb->len);
		
		/* If the buffer gets empty remove the pending timer and retrigger trg callback function,
		 * otherwise restart timer.
		 */
		if( !--pbcb->n ) {
			TMR_Stop(pbcb->ptic);
			pbcb->btrgfxn = 1;
		}
		else {
			TMR_ReStart(pbcb->ptic);	
		}
    	
		p = pbcb->pout;
		p += pbcb->len;
		
		if( p >= (uint8*)pbcb->pbot )
			p = pbcb->ptop;
		pbcb->pout = p;
		    	
		INT_Restore( flags );
		return SYS_OK;
	}
	INT_Restore( flags );
	return SYS_ERROR;
}
//----------------------------------------------------------------------------
/*!	\brief Empty buffer
 * 
 *  \param ph	handle to buffer 
 */
void BUF_Flush(HANDLE *ph) {
	size_t flags;
	bcb_t *pbcb = (bcb_t*) ph;
	
	flags = INT_Disable();	
	pbcb->n = 0;
	pbcb->btrgfxn = 1;
	pbcb->pin = pbcb->pout = pbcb->ptop;
	TMR_Stop(pbcb->ptic);
	INT_Restore( flags );
}
//----------------------------------------------------------------------------
/*!	\brief Querry buffer
 * 
 *  \param ph	handle to buffer 
 */
uint16 BUF_Querry(HANDLE *ph) {
	bcb_t *pbcb = (bcb_t*) ph;
	return pbcb->n;
}
//----------------------------------------------------------------------------
/*!	\brief Set buffer timeout
 * 
 *	By default the buffer structure has no timeout callback function associated. This function
 *	appends a timeout callback function that will be fired if the fifo contains data that is older
 *	that the timeout period. The handle to the buffer structure is passed thrue the parg1 argument
 *	of the callback function. Each read or write access on the buffer re-triggers the timer. 
 *	Note that the callback funcion is executed from the uSMARTX_Tick() context.
 *
 *  \param ph handle to buffer 
 *	\param timeout timeout in system ticks
 *	\param pfxn address of the callback function
 *
 */
void BUF_AppendToutFxn(HANDLE *ph, uint16 timeout, STATUS (*pfxn)(uint8 evt, void *parg1, void *parg2)) {
	size_t flags;
	bcb_t *pbcb = (bcb_t*) ph;
	
	flags = INT_Disable();
	/* If there is no data in the buffer don't start the timer but just remember its parameter. */
	if( !pbcb->n ) {
		pbcb->ptic->abs = timeout;
		pbcb->ptic->pfxn = pfxn;
		INT_Restore( flags );
		return;
	}	
	INT_Restore( flags );
	TMR_Start(pbcb->ptic, timeout, pfxn, 0, pbcb, 0, TMR_PERIODIC);	
}
//----------------------------------------------------------------------------
/*!	\brief Set buffer trigger level
 * 
 *	By default the buffer structure has no trigger callback function associated. This function
 *	appends the trigger callback function with a defined trigger level. The function is executed
 *	only once and is re-triggered when the buffer is completely emptied out (also BUF_Flush()).
 *	The handle to the buffer structure is passed to the function via the parg1 argument.
 *
 *  \param ph handle to buffer 
 *	\param trigger trigger level
 *	\param pfxn address of the callback function
 */
void BUF_AppendTrgFxn(HANDLE *ph, uint16 trigger, STATUS (*pfxn)(uint8 evt, void *parg1, void *parg2)) {
	bcb_t *pbcb = (bcb_t*) ph;
	pbcb->trg = trigger;
	pbcb->ptrgfxn = pfxn;
	pbcb->btrgfxn = 1;
}
/*!	@}*/
//------------------------------------------------------------------------------
/*						INTERNAL FUNCTIONS		
 *
 * These are uSMARTX internal function. They must not be called from a task.
 */
/*! \internal
 *	\brief Dummy callback funcion
 *
 *	If there is no trigger or timeout callback function this function is executed.
 *
 *  \param evt		not used
 *	\param parg1	not used
 *	\param parg1	not used
 *	\return SYS_OK
 */
STATUS buff_dummy_clb(uint8 evt, void *parg1, void *parg2) {
	return SYS_OK;	
}
