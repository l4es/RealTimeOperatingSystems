@/*
@ * Copyright (C) 2004-2005, Marko Panger
@ *
@ * Redistribution and use in source and binary forms, with or without
@ * modification, are permitted provided that the following conditions
@ * are met:
@ *
@ * 1. Redistributions of source code must retain the above copyright
@ *    notice, this list of conditions and the following disclaimer.
@ * 2. Redistributions in binary form must reproduce the above copyright
@ *    notice, this list of conditions and the following disclaimer in the
@ *    documentation and/or other materials provided with the distribution.
@ * 3. Neither the name of the author may be used to endorse or promote products
@ *	  derived from this software without specific prior written permission.
@ *
@ * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS
@ * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
@ * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
@ * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR
@ * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
@ * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
@ * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
@ * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
@ * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
@ * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
@ * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
@ * SUCH DAMAGE.
@ *
@ * For additional information send an email to marko.panger@siol.net
@ *
@ */
 
#define _MODULE(A)		
#define _GLOBAL(A)		.global A
#define _ARM_CODE		.code 32
#define GET_CPSR(A)		mrs A, cpsr
#define SET_CPSR(A)		msr	cpsr_c, A
	
	_ARM_CODE
	
@/*!	\brief Globaly disable interrupts
@ *
@ *	Calling this function will globaly disable interrupts and return the interrupts mask register before disabling
@ *	interrupts. 
@ *
@ *	\retval Interrupts mask
@ */
@   
	.global INT_Disable
	.func INT_Disable
INT_Disable:	
	mrs 	r1, cpsr		@ get current program status register
	orr 	r0, r1, #0x80	@ set I bit to disable interrupts
	msr		cpsr_c, r0		@ write back cpsr
	and		r0, r1, #0x80	@ mask out the I bit (original cpsr) and return it via r0	
	bx		lr				@ return in thumb mode
	.endfunc
	
@/*!	\brief Globaly enable interrupts
@ *
@ *	Calling this function will globaly enable interrupts. 
@ */	
	.global INT_Enable
	.func INT_Enable
INT_Enable:    
	GET_CPSR(r1)			@ get current cpsr
	bic		r1, r1, #0x80	@ clear the I bit
	SET_CPSR(r1)			@ write back cpsr
	bx		lr				@ return in thumb mode
	.endfunc

@/*!	\brief Globaly restore interrupts
@ *
@ *	Calling this function will restore the interrupts state as were before calling the disable function.
@ *	\param flags interrupts state before disabling them
@ */
	.global INT_Restore
	.func INT_Restore	
INT_Restore:
	ands	r0,r0,#0x80
	beq		INT_Enable
	bx		lr				 @return in thumb mode
	.endfunc
