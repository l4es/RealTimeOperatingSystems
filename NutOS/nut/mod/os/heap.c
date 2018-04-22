/*
 * Copyright (C) 2001-2003 by egnite Software GmbH. All rights reserved.
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
 * Revision 1.3  2005/04/30 16:42:42  chaac
 * Fixed bug in handling of NUTDEBUG. Added include for cfg/os.h. If NUTDEBUG
 * is defined in NutConf, it will make effect where it is used.
 *
 * Revision 1.2  2003/07/20 16:05:00  haraldkipp
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2003/05/09 14:41:25  haraldkipp
 * Initial using 3.2.1
 *
 * Revision 1.15  2002/06/26 17:29:44  harald
 * First pre-release with 2.4 stack
 *
 */

//--------------------------------------------------------------------------//

/*!
 * \addtogroup xgModHeap
 */

/*@{*/

#include    <cfg/os.h>
#include    <string.h>

#include    <sys/atom.h>
#include    <sys/heap2.h>

#ifdef NUTDEBUG
#include    <sys/print.h>
#include    <sys/osdebug.h>
#endif

//--------------------------------------------------------------------------//

//      Local definitions
//  #define         DO_CRITICAL
//#define         LOG_UART

//--------------------------------------------------------------------------//

#define         deadmeat    0xDEADBEEFL
#define         slime_1     0xffffff00L
#define         slime_2     0xffff0000L
#define         slime_3     0xff000000L

#ifdef LOG_UART
extern NUTDEVICE          * uart0;
#define         log_dev     uart0
#else
#define         log_dev     0
#endif

//--------------------------------------------------------------------------//

static u_char CheckHeapGuards( HEAPNODE * p_node, PGM_P str );
static void AddHeapGuards( HEAPNODE * p_node );

#ifdef USED_HEAP_LIST
static void InsertInUsedList( HEAPNODE *p_node );
static void RemoveFromUsedList( HEAPNODE *fnode );
#endif

//--------------------------------------------------------------------------//

/*!
 * \brief List of free nodes.
 */
HEAPNODE     * volatile heapFreeList    =   0;

#ifdef USED_HEAP_LIST
HEAPNODE     * volatile heapUsedList    =   0;
#endif

/*!
 * \brief Number of bytes available.
 */
u_short available = 0;

//--------------------------------------------------------------------------//

/*!
 * \brief
 * Allocate a block from heap memory.
 *
 * This functions allocates a memory block of the specified
 * size and returns a pointer to that block.
 *
 * The actual size of the allocated block is larger than the
 * requested size because of space required for maintenance
 * information. This additional information is invisible to
 * the application.
 *
 * The routine looks for the smallest block that will meet
 * the required size and releases it to the caller. If the
 * block being requested is usefully smaller than the smallest
 * free block then the block from which the request is being
 * met is split in two. The unused portion is put back into
 * the free-list.
 *
 * The contents of the allocated block is unspecified.
 * To allocate a block with all bytes set to zero use
 * NutHeapAllocClear().
 *
 * \note Do not use this function in interrupt routines.
 *
 * \param size Size of the requested memory block.
 *
 * \return Pointer to the allocated memory block if the
 *         function is successful or NULL if the requested
 *         amount of memory is not available.
 */
void * NutHeapAlloc( u_short req_size )
{
HEAPNODE              * node;
HEAPNODE   * volatile * npp;
HEAPNODE              * fit         =   0;
HEAPNODE   * volatile * fpp         =   0;
u_short                 block_size;

#ifdef DO_CRITICAL
    NutEnterCritical( );
#endif

    block_size = req_size + HEAP_HEAD_SIZE + HEAP_TAIL_SIZE;
    if ( block_size > available )
    {
#ifdef NUTDEBUG
        if ( heap_trace != 0 )
            NutPrintFormat_P( log_dev, PSTR( "MEMOVR-%d\r\n" ), req_size );
#endif
        return 0;
    }

    /*
     * We need additional space in front of the allocated memory
     * block to store its size. If this is still less than the
     * space required by a free node, increase it.
     */
    if ( block_size < ( HEAP_HEAD_SIZE + HEAP_TAIL_SIZE ) )
        block_size = sizeof( HEAPNODE ) + HEAP_TAIL_SIZE;   //  2 byte block

#ifdef NUTDEBUG
    if ( heap_trace != 0 )
        NutPrintFormat_P( log_dev, PSTR( "A(%d," ), req_size );
#endif

    /*
     * Walk through the linked list of free nodes and find the best fit.
     */
    node =  heapFreeList;
    npp  = &heapFreeList;
#ifdef NUTDEBUG
    if ( heap_trace != 0 )
        NutPrintFormat_P( log_dev, PSTR( "node=0x%04x\n" ), node );
#endif
    while ( node != 0 )
    {
#ifdef NUTDEBUG
        if ( heap_trace != 0 )
            NutPrintFormat_P( log_dev, PSTR( "node=0x%04x\n" ), node );
#endif

        /*
         * Found a node that fits?
         */
        if ( node->hn_size >= block_size )
        {
            /*
             * If it's an exact match, we don't
             * search any further.
             */
            if ( node->hn_size == block_size )
            {
                fit = node;
                fpp = npp;
                break;
            }

            /*
             * Is it the first one we found
             * or was the previous one larger?
             */
            if ( fit == 0 || ( fit->hn_size > node->hn_size ) )
            {
                fit = node;
                fpp = npp;
            }
        }
        npp  = &node->hn_next;
        node =  node->hn_next;
    }

    if ( fit != 0 )
    {
        CheckHeapGuards( fit, PSTR( "alloc" ) );

        /*
         * If the node we found is larger than the
         * required space plus the space needed for
         * a new node plus a defined threshold, then
         * we split it.
         */
        if ( fit->hn_size > ( block_size + HEAP_HEAD_SIZE + HEAP_TAIL_SIZE + ALLOC_THRESHOLD ) )
        {
            node = (HEAPNODE *)( (u_short)fit + block_size );
            node->hn_size = fit->hn_size - block_size;
            node->hn_next = fit->hn_next;
            AddHeapGuards( node );
            fit->hn_size = block_size;
            *fpp = node;
        }
        else
            *fpp = fit->hn_next;

        available -= fit->hn_size;
        AddHeapGuards( fit );
#ifdef USED_HEAP_LIST
        InsertInUsedList( fit );
#endif
        fit = (HEAPNODE *)( (char *)fit + HEAP_HEAD_SIZE );
    }

#ifdef NUTDEBUG
    if ( heap_trace != 0 )
        NutPrintFormat_P( log_dev, PSTR( "%x) " ), (u_short)fit );
#endif

#ifdef DO_CRITICAL
    NutExitCritical( );
#endif
    return (void *)fit;
}

//--------------------------------------------------------------------------//

/*!
 * \brief Allocate an initialized block from heap memory.
 *
 * This functions allocates a memory block of the specified
 * size with all bytes initialized to zero and returns a
 * pointer to that block.
 *
 * \param size Size of the requested memory block.
 *
 * \return Pointer to the allocated memory block if the
 *         function is successful or NULL if the requested
 *         amount of memory is not available.
 */
void * NutHeapAllocClear( u_short size )
{
void      * ptr;

    if ( ( ptr = NutHeapAlloc( size ) ) != 0 )
    {
        memset( ptr, 0,
((HEAPNODE *)( (char *)ptr - HEAP_HEAD_SIZE ) )->hn_size - HEAP_HEAD_SIZE - HEAP_TAIL_SIZE );
    }

    return ptr;
}

//--------------------------------------------------------------------------//

/*!
 * \brief Return a block to heap memory.
 *
 * An application calls this function, when a previously
 * allocated memory block is no longer needed.
 *
 * The heap manager checks, if the released block adjoins any
 * other free regions. If it does, then the adjacent free regions
 * are joined together to form one larger region.
 *
 * \note Do not use this function in interrupt routines.
 *
 * \param block Points to a memory block previously allocated
 *              through a call to NutHeapAlloc().
 *
 * \return 0 on success, -1 if the caller tried to free
 *         a block which had been previously released.
 */
int NutHeapFree( void * block )
{
HEAPNODE              * node;
HEAPNODE   * volatile * npp;
HEAPNODE              * fnode;

#ifdef DO_CRITICAL
    NutEnterCritical( );
#endif

#ifdef NUTDEBUG
    if ( block == 0 )
    {
        NutPrintString_P( log_dev, PSTR( "\r\nMEMnull" ) );
#ifdef DO_CRITICAL
        NutExitCritical( );
#endif
        return -1;
    }
#endif

    /*
     * Convert our block into a node.
     */
    fnode = (HEAPNODE *)( (u_char *)block - HEAP_HEAD_SIZE );
    CheckHeapGuards( fnode, PSTR( "free" ) );

#ifdef NUTDEBUG
    if ( heap_trace != 0 )
        NutPrintFormat_P( log_dev, PSTR( "F(%d,%x) " ), fnode->hn_size, (u_short)block );
#endif
    available += fnode->hn_size;

#ifdef USED_HEAP_LIST
    RemoveFromUsedList( fnode );
#endif

    /*
     * Walk through the linked list of free nodes and try
     * to link us in.
     */
    node =  heapFreeList;
    npp  = &heapFreeList;
    while ( node != 0 )
    {
        /*
         * If there' s a free node in front of us, merge it.
         */
        if ( ( (u_short)node + node->hn_size ) == (u_short)fnode )
        {
            CheckHeapGuards( node, PSTR( "prev" ) );
            node->hn_size += fnode->hn_size;

            /*
             * If a free node is following us, merge it.
             */
            if ( ( (u_short)node + node->hn_size ) == (u_short)node->hn_next )
            {
                CheckHeapGuards( node->hn_next, PSTR( "next" ) );
                node->hn_size += node->hn_next->hn_size;
                node->hn_next  = node->hn_next->hn_next;
            }
            AddHeapGuards( node );
            break;
        }

        /*
         * If we walked past our address, link us to the list.
         */
        if ( (u_short)node > (u_short)fnode )
        {
            *npp = fnode;

            /*
             * If a free node is following us, merge it.
             */
            if ( ( (u_short)fnode + fnode->hn_size ) == (u_short)node )
            {
                fnode->hn_size += node->hn_size;
                fnode->hn_next  = node->hn_next;
            }
            else
                fnode->hn_next = node;
            break;
        }

        /*
         * If we are within a free node, somebody tried
         * to free a block twice.
         */
        if ( ( (u_short)node + node->hn_size) > (u_short)fnode )
        {
#ifdef NUTDEBUG
            if ( heap_trace != 0 )
                NutPrintString_P( log_dev, PSTR( "\r\nTWICE\r\n" ) );
#endif
#ifdef DO_CRITICAL
            NutExitCritical( );
#endif
            return -1;
        }

        npp  = &node->hn_next;
        node =  node->hn_next;
    }

    /*
     * If no link was found, put us at the end of the list
     */
    if ( node == 0 )
    {
        fnode->hn_next = node;
        *npp = fnode;
    }

#ifdef DO_CRITICAL
    NutExitCritical( );
#endif
    return 0;
}

//--------------------------------------------------------------------------//

/*!
 * \brief
 * Add a new memory region to the free heap.
 *
 * This function is automatically called by Nut/OS during
 * initialization.
 *
 * Applications typically do not call this function.
 *
 * \param addr Start address of the memory region.
 * \param size Number of bytes of the memory region.
 */
void NutHeapAdd( void * addr, u_short size )
{
HEAPNODE      * p_node;

    p_node = (HEAPNODE *)addr;

#ifdef DO_CRITICAL
    NutEnterCritical( );
#endif

    //  Pretend this is a clean block so no errors appear.
    p_node->hn_size = size;
    AddHeapGuards( p_node );

#ifdef USED_HEAP_LIST
    InsertInUsedList( p_node );
#else
    p_node->hn_next = 0;
#endif

#ifdef DO_CRITICAL
    NutExitCritical( );
#endif

    NutHeapFree( (void *)( (char *)addr + HEAP_HEAD_SIZE ) );
}

//--------------------------------------------------------------------------//

/*!
 * \brief Return the number of bytes available.
 *
 * \return Number of bytes.
 */
u_short NutHeapAvailable( void )
{
    return available;
}

//--------------------------------------------------------------------------//

u_char NutHeapCheckGuards1( void * block )
{
    return NutHeapCheckGuards2( (HEAPNODE *)( (char *)block - HEAP_HEAD_SIZE ) );
}

//--------------------------------------------------------------------------//

u_char NutHeapCheckGuards2( HEAPNODE * p_node )
{
u_char          ret     =   0;

#ifdef FRONT_GUARDED_HEAP
    if ( p_node->hn_guard != deadmeat )
        ret = 2;
#endif

    if ( ( (HEAPTAIL *)( (char *)p_node + p_node->hn_size - HEAP_TAIL_SIZE ) )->ht_guard != deadmeat )
        ret += 1;

    return ret;
}

//--------------------------------------------------------------------------//

static void AddHeapGuards( HEAPNODE * p_node )
{
#ifdef FRONT_GUARDED_HEAP
    p_node->hn_guard = deadmeat;
#endif

    ( (HEAPTAIL *)( (char *)p_node + p_node->hn_size - HEAP_TAIL_SIZE ) )->ht_guard = deadmeat;
}

//--------------------------------------------------------------------------//

static u_char CheckHeapGuards( HEAPNODE * p_node, PGM_P str )
{
HEAPTAIL      * p_tail;
u_char          ret     =   0;
int             slime;
#ifdef NUTDEBUG
char            temp[ 20 ];
#endif

#ifdef FRONT_GUARDED_HEAP
    if ( p_node->hn_guard != deadmeat )
    {
        ret = 2;
        //  determine how many bytes got slimed.
        if ( ( p_node->hn_guard & slime_1 ) == ( deadmeat & slime_1 ) )
            slime = 1;
        else
        {
            if ( ( p_node->hn_guard & slime_2 ) == ( deadmeat & slime_2 ) )
                slime = 2;
            else
            {
                if ( ( p_node->hn_guard & slime_3 ) == ( deadmeat & slime_3 ) )
                    slime = 3;
                else
                    slime = 4;
            }
        }
#ifdef NUTDEBUG
    /* This will not work on ICCAVR */
        strncpy_P( temp, str, 19 );
        NutPrintFormat( 0, "\r\nMEMCORRUPT-%s-%d-%d\r\n",
                                            temp, p_node->hn_size, slime );
#endif
    }
#endif

    p_tail = (HEAPTAIL *)( (char *)p_node + p_node->hn_size - HEAP_TAIL_SIZE );
    if ( p_tail->ht_guard != deadmeat )
    {
        ret += 1;
        //  determine how many bytes got slimed.
        if ( ( p_tail->ht_guard & slime_1 ) == ( deadmeat & slime_1 ) )
            slime = 1;
        else
        {
            if ( ( p_tail->ht_guard & slime_2 ) == ( deadmeat & slime_2 ) )
                slime = 2;
            else
            {
                if ( ( p_tail->ht_guard & slime_3 ) == ( deadmeat & slime_3 ) )
                    slime = 3;
                else
                    slime = 4;
            }
        }
#ifdef NUTDEBUG
    /* This will not work on ICCAVR */
        strncpy_P( temp, str, 19 );
        NutPrintFormat( log_dev, "\r\nMemCorrupt-%s-%d-%d\r\n",
                                            temp, p_node->hn_size, slime );
#endif
    }

    return ret;
}

//--------------------------------------------------------------------------//

#ifdef USED_HEAP_LIST
static void InsertInUsedList( HEAPNODE *p_node )
{
HEAPNODE              * p_used;
HEAPNODE   * volatile * pp_prev;

    p_used  =  heapUsedList;
    pp_prev = &heapUsedList;
    while ( p_used != 0 )
    {
        if ( p_used->hn_next > p_node )
        {
            p_node->hn_next = ( *pp_prev )->hn_next;
            ( *pp_prev )->hn_next = p_node;
            break;
        }

        pp_prev = &p_used->hn_next;
        p_used  =  p_used->hn_next;
    }

    if ( p_used == 0 )
    {
        p_node->hn_next = heapUsedList;
        heapUsedList = p_node;
    }
}
#endif

//--------------------------------------------------------------------------//

#ifdef USED_HEAP_LIST
static void RemoveFromUsedList( HEAPNODE *p_node )
{
HEAPNODE   * volatile * pp_used;

    pp_used = &heapUsedList;
    while ( *pp_used != 0 )
    {
        if ( *pp_used == p_node )
        {
            *pp_used = ( *pp_used )->hn_next;
            break;
        }
        pp_used = &( *pp_used )->hn_next;
    }

#ifdef NUTDEBUG
    if ( *pp_used == 0 )
        NutPrintString( log_dev, "\r\nMemCorrupt-used\r\n" );
#endif
}
#endif

/*@}*/

//*************************** end of file HEAP.C ***************************//
