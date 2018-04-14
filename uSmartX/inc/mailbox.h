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
 
#ifndef _MAILBOX_H_
#define _MAILBOX_H_

/********************************************************************/
/*					uSMARTX mailbox definitions						*/
/********************************************************************/

/*!	\addtogroup uSMARTX_api_mailbox
 *	@{
 */

/*!	\brief Create a mailbox
 *
 *	This macro creates and partialy initilises the mailbox control structure. It also allocates
 *	the needed message buffers.
 *
 *	\param name name of the mailbox
 *	\param n number of messages a mailbox can hold
 *	\param msg_size size of a mailbox message 
 *	\hideinitializer  
 */
#define MBX_CREATE(name, n, msg_size)	msg_t mcb_msg_##name[n]; \
										uint8 mcb_buff_##name[msg_size][n]; \
										mcb_t name = {	{0,0}, \
														{0,0}, \
														{0,0}, \
														{0,0}, \
														&mcb_buff_##name[0][0], \
														&mcb_msg_##name[0], \
														msg_size, \
														n}
														
/*!	@} */
/********************************************************************/
/*					uSMARTX mailbox system calls						*/
/********************************************************************/
void MBX_Init(HANDLE *ph);
STATUS MBX_Post(HANDLE *ph, void *pmsg, uint16 tout);
STATUS MBX_Pend(HANDLE *ph, void *pmsg, uint16 tout);
void MBX_Flush(HANDLE *ph);

/********************************************************************/
/*					uSMARTX mailbox internal functions					*/
/********************************************************************/
STATUS mbx_tout_clb(uint8 evt, void *parg1, void *parg2);

#endif

