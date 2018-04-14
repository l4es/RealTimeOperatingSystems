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
#include <mballoc.h>

/*! \defgroup uSMARTX_api_mballoc Memory management
 *	@{
 *	The uSMARTX kernel implements a fixed size memory allocator with different heaps or memory pools.
 *	The allocating policy is first fit. Memory is always allocated from a pre-defined memory heap.
 *	Each memory heap has its own defined minimum allocatable size. In practice this is the size of
 *	a memory block. The allocated memory must alwasy be returned to the heap from which was allocated.
 *  
 */

/*! \brief Allocate memory
 *
 *	Allocate the required memory from the specified memory heap. The allocation is rounded
 *	to the memory block size.
 *
 *  \param ph		handle to memory heap
 *  \param size 	amount of memory to allocate
 *	\retval pointer to allocated memory or null in there is no avaible memory
 *	\todo Check what if the required number of memory blocks is 255
 */
void* MEM_Alloc(HANDLE *ph, uint16 size) {
	uint8 i, j, k;
	uint16 sum;		
	size_t flags;
	memd_t *pmemd = (memd_t*)ph;
	
	uint8 *offset = (uint8*) pmemd->pmem;
			
	j = 0;	

	flags = INT_Disable();

	while( j < pmemd->nmb ) {				
		/* Cycle thrue mem blocks till we found a free mem block */
		sum  = 0;
		i = j;
		while( i < pmemd->nmb ) {		
			if( *(pmemd->pmbd+i) ) {
				offset += (*(pmemd->pmbd+i)) * (pmemd->alloc_size);
				i += *(pmemd->pmbd+i);				 
				continue;
			}
			break;				
		}
		j = i;
		
		/* At this point j is the index of the first unused mem block and offset is the offset in the memory buffer. */		
		for(k = 1, i = j; i < pmemd->nmb; k++, i++) {
			if( *(pmemd->pmbd+i) ) {				
				/* We haven't found the required amount of mem blocks. Continue with the next free mem block.
		 		 * Increment j for the number of used mem blocks. 
		 		 */
				j = i + *(pmemd->pmbd+i);
				offset = (uint8*) pmemd->pmem;
				offset += j * (pmemd->alloc_size);
				break;
			}
								
			sum += pmemd->alloc_size;
			if( sum >= size ) {
				*(pmemd->pmbd+j) = k;
				INT_Restore( flags );
				return (void*) offset;
			}						
		}
		if( i == pmemd->nmb )
			break;
	}
	/* Memory not avaible */
	INT_Restore( flags );
	return 0;
}

/*! \brief Free memory
 *
 *	Free the previously allocated memory by returning it to the specified memory pool.
 * 
 *  \param ph		handle to memory heap
 *  \param pmem 	pointer to the previously allocated memory
 
 *	\attention The memory must be returned to the pool from where was allocated
 */
void MEM_Free(HANDLE *ph, void* pmem) {	
	uint8 i, *p;
	memd_t *pmemd = (memd_t*)ph;	
	size_t flags;

	flags = INT_Disable();		

	/* Calculate the index of the mem block descriptor:
	 * index = (user memory - top of memory buffer) / alloc_size
	 */
	p = (uint8*) pmemd->pmem;
	for(i = 0; i < pmemd->nmb; i++) {
		if( p == pmem ) {
			*(pmemd->pmbd + i) = 0;
			break;
		}
		p += pmemd->alloc_size;
	}
	INT_Restore( flags );
}
/*! \brief Test parent of memory block
 *
 *	Test if the pointed memory belongs to a given memory heap.
 * 
 *  \param ph		handle to memory heap
 *  \param pmem 	pointer to memory to test
 *	\retval SYS_OK if the pointed memory belongs to the memory heap
 *	\retval SYS_EROR specified heap is not parent of the pointed memory 
 */
STATUS MEM_IsHeapOwner(HANDLE *ph, void* pmem) {
	memd_t *pmemd = (memd_t*)ph;
	uint8 *pend;
		
	pend = (uint8*)(pmemd->pmem + pmemd->nmb*pmemd->alloc_size);
	
	if( pmemd->pmem <= (uint8*)pmem && (uint8*)pmem < pend )
		return SYS_OK;
	
	return SYS_ERROR;
}
/*!	\@}*/
