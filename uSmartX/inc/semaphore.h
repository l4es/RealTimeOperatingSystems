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
 
#ifndef _SEM_H_
#define _SEM_H_

/********************************************************************/
/*					uSMARTX semaphore definitions						*/
/********************************************************************/
/*!	\addtogroup uSMARTX_api_semaphore
 *	@{
 */

/*!	\brief Create a semaphore
 *
 *	This macro creates a semaphore and initilises its initail count and its maximum value.
 * 	The semaphore maximum value can be up to 255.
 *
 *	\param name name of the semaphore
 *	\param cnt number of resources at cration time
 *	\param max_cnt maximun value of semaphore
 *	\hideinitializer 
 */
#define SEM_CREATE(name, cnt, max_cnt)	scb_t name = {{0,0}, cnt, max_cnt}

/*!	@} */

/********************************************************************/
/*					uSMARTX semaphore system calls						*/
/********************************************************************/
void SEM_Post(HANDLE *ph);
STATUS SEM_Pend(HANDLE *ph, uint16 tout);
void SEM_Reset(HANDLE *ph, uint8 cnt);
uint8 SEM_Querry(HANDLE *ph);

/********************************************************************/
/*					uSMARTX semaphore internal functions				*/
/********************************************************************/
STATUS sem_tout_clb(uint8 evt, void *parg1, void *parg2);


#endif
