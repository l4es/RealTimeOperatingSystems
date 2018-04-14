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
 
#ifndef _MBALLOC_H
#define _MBALLOC_H

/********************************************************************/
/*				uSMARTX memory allocator definitions					*/
/********************************************************************/
/* This structure is the head of a memory pool. */
typedef struct {
	uint16 alloc_size;	/* Block size in bytes */
	uint8 nmb;			/* Number of memory blocks */
	uint8 *pmbd;		/* Pointer to memory block descripotors */
	uint8 *pmem;		/* Pointer to memory blocks - acctualy the first mem block */

} memd_t;

/*!	\addtogroup uSMARTX_api_mballoc
 *	@{
 */
 
/*! \enum mb_size_t
 *	
 *	These defines define the valid memory block sizes.
 *
 *	\hideinitializer
 */
typedef enum {		
	MEM_MB_4_BYTE		= (1 << 2),	/*!< Memory block contains 4 bytes */
	MEM_MB_8_BYTE		= (1 << 3),	/*!< Memory block contains 8 bytes */
	MEM_MB_16_BYTE		= (1 << 4),	/*!< Memory block contains 16 bytes */
	MEM_MB_32_BYTE		= (1 << 5),	/*!< Memory block contains 32 bytes */
	MEM_MB_64_BYTE		= (1 << 6),	/*!< Memory block contains 64 bytes */
	MEM_MB_128_BYTE		= (1 << 7),	/*!< Memory block contains 128 bytes */
	MEM_MB_256_BYTE		= (1 << 8),	/*!< Memory block contains 256 bytes */
	MEM_MB_512_BYTE		= (1 << 9),	/*!< Memory block contains 512 bytes */
	MEM_MB_1024_BYTE	= (1 << 10),	/*!< Memory block contains 1024 bytes */
	MEM_MB_2048_BYTE	= (1 << 11),   /*!< Memory block contains 2048 bytes */
	MEM_MB_4096_BYTE	= (1 << 12),   /*!< Memory block contains 4096 bytes */
	MEM_MB_8192_BYTE	= (1 << 13)    /*!< Memory block contains 8192 bytes */

} mb_size_t;

/*!	\brief Create a memory heap
 *
 *	This macro creates and initilises a memory heap. The parameter alloc size should be of type \c mb_size_t.
 *
 *	\param name name of the memory heap
 *	\param n number of memory blocks
 *	\param alloc_size size of each memory block
 *	\hideinitializer 
 *
 *	\internal
 *	mem_buff_##name is the heap space. This variable must be allocated on a 4byte boundary as the user can
 *	byte, short or word access it.
 */
#define MEM_HEAP_CREATE(name, n, alloc_size)	uint32 mem_buff_##name[(n*alloc_size)>>2]; \
												uint8 mbd_##name[n]; \
												memd_t name = {alloc_size, n, &mbd_##name[0], (uint8*)&mem_buff_##name[0]}

/*!	@} */										

/********************************************************************/
/*					uSMARTX memory handling system calls				*/
/********************************************************************/
void* MEM_Alloc(HANDLE *ph, uint16 size);
void MEM_Free(HANDLE *ph, void* pmem);
STATUS MEM_IsHeapOwner(HANDLE *ph, void* pmem);

#endif
