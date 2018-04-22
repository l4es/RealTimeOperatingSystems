#ifndef _SYS_HEAP_H_
#define _SYS_HEAP_H_

/*
 * Copyright (C) 2001-2003 by egnite Software GmbH. All rights reserved.
 * Copyright (C) 2003 by Peter Scandrett.
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
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 * -
 * Portions Copyright (C) 2000 David J. Hudson <dave@humbug.demon.co.uk>
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You can redistribute this file and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software Foundation;
 * either version 2 of the License, or (at your discretion) any later version.
 * See the accompanying file "copying-gpl.txt" for more details.
 *
 * As a special exception to the GPL, permission is granted for additional
 * uses of the text contained in this file.  See the accompanying file
 * "copying-liquorice.txt" for details.
 */

/*
 * $Log$
 * Revision 1.1  2003/05/09 14:41:24  haraldkipp
 * Initial revision
 *
 */

#include <stddef.h>
#include <sys/types.h>

/*!
 * \file sys/heap2.h
 * \brief Heap management definitions.
 */
/*!
 * \define FRONT_GUARDED_HEAP
 * \brief Reduces HEAPNODE structure if not defined
 *
 * This is provided to check someone else does not 'slime' the front of my
 *   block.
 */
#define     FRONT_GUARDED_HEAP


/*!
 * \file sys/heap2.h
 * \brief Heap management definitions.
 */
/*!
 * \define USED_HEAP_LIST
 * \brief Minimises HEAPNODE structure if not defined
 *
 * This is provided for heap management.
 */
#define     USED_HEAP_LIST


/*!
 * \struct _HEAPNODE heap.h sys/heap.h
 * \brief Heap memory node information structure.
 */
/*!
 * \typedef HEAPNODE
 * \brief Heap memory node type.
 */
typedef struct _HEAPNODE
{
#ifdef FRONT_GUARDED_HEAP
    u_long              hn_guard;   /*!< \brief Protection check guard. */
#endif
    u_short             hn_size;    /*!< \brief Size of this node.      */
    //  The next element must be the last.
    struct _HEAPNODE  * hn_next;    /*!< \brief Link to next free node. */
} HEAPNODE;


/*!
 * \struct _HEAPTAIL heap.h sys/heap.h
 * \brief Heap memory node information structure.
 */
/*!
 * \typedef HEAPTAIL
 * \brief Heap memory allocated tail guard.
 */
typedef struct _HEAPTAIL
{
    u_long              ht_guard;   /*!< \brief Protection check guard. */
} HEAPTAIL;


/*!
 * \brief Allocation threshold.
 *
 * Might be increased to avoid creating
 * too many small nodes.
 */
#define ALLOC_THRESHOLD 6

extern HEAPNODE * volatile heapFreeList;

/*!
 * \brief Define structure sizes
 */
#ifdef USED_HEAP_LIST
    extern HEAPNODE * volatile heapUsedList;
    #define     HEAP_HEAD_SIZE  ( sizeof( HEAPNODE ) )
#else
    //  When there is NOT a used heap list, we use the HN_NEXT element of
    //  HEAPNODE as part of the user's data block. It saves two bytes.
    #define     HEAP_HEAD_SIZE  ( sizeof( HEAPNODE ) - sizeof( HEAPNODE * ) )
#endif
#define     HEAP_TAIL_SIZE      ( sizeof( HEAPTAIL ) )


extern void   * NutHeapAlloc        ( u_short size );
extern void   * NutHeapAllocClear   ( u_short size );
extern int      NutHeapFree         ( void * block );
extern void     NutHeapAdd          ( void * addr, u_short size );
extern u_short  NutHeapAvailable    ( void );

//  Returns
//      0 - guards OK
//      1 - tail guard damaged
//      2 - head guard damaged  - if FRONT_GUARDED_HEAP defined
//      3 - both guards damaged - if FRONT_GUARDED_HEAP defined
extern u_char   NutHeapCheckGuards1 ( void * block );
extern u_char   NutHeapCheckGuards2 ( HEAPNODE * node );

#endif
