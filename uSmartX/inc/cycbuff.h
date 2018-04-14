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
 
#ifndef _CYCBUFF_H_
#define _CYCBUFF_H_

/********************************************************************/
/*					uSMARTX cyclic buffer definitions				*/
/********************************************************************/

/*!	\addtogroup uSMARTX_api_cyc_buffer
 *	@{
 */

/*!	\brief Create a cyclic fifo buffer
 *
 *	This macro creates a cyclic fifo buffer structure and allocates needed memory.
 *
 *
 *	\param name name of the buffer
 *	\param n number of data containers
 *	\param data_size data size of each entry
 *	\hideinitializer 
 */
#define BUF_CREATE(name, n, data_size)	struct tic_s bcb_tic_##name = {	0, \
																		0, \
																		0xFFFF, \
																		0, \
																		&buff_dummy_clb, \
																		0, \
																		0, \
																		0, \
																		0 \
																		}; \
											uint8 bcb_buff_##name[n+1][data_size]; \
											bcb_t name = {	&bcb_buff_##name[0][0], /* pin */ \
															&bcb_buff_##name[0][0],	/* pout */ \
															&bcb_buff_##name[0][0],	/* ptop */ \
															&bcb_buff_##name[n][data_size], /* pbot */ \
															0,	/* n */ \
															0,	/* trg */ \
															data_size,	/* len */ \
															0,	/* btrgfxn */ \
															&buff_dummy_clb, \
															&bcb_tic_##name \
															};
														
/*!	@} */
/********************************************************************/
/*				uSMARTX cyclic buffer system calls					*/
/********************************************************************/
STATUS BUF_Put(HANDLE *ph, void *pdata);
STATUS BUF_Get(HANDLE *ph, void *pdata);
void BUF_Flush(HANDLE *ph);
uint16 BUF_Querry(HANDLE *ph);
void BUF_AppendToutFxn(HANDLE *ph, uint16 timeout, STATUS (*pfxn)(uint8 evt, void *parg1, void *parg2));
void BUF_AppendTrgFxn(HANDLE *ph, uint16 trigger, STATUS (*pfxn)(uint8 evt, void *parg1, void *parg2));
/********************************************************************/
/*			uSMARTX cyclic buffer internal functions				*/
/********************************************************************/
STATUS buff_dummy_clb(uint8 evt, void *parg1, void *parg2);

#endif

