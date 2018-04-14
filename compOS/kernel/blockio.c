/*
 blockio.c - generic block device i/o
  
 Author:        Paul Barker
 Part of:       COS
 Created:       29/10/04
 Last Modified: 29/10/04

 Copyright (C) 2004 Paul Barker
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

                     (See file "Copying")
*/

#include <cosbase.h>

#include <cos/debug.h>
#include <cos/block_dev.h>

block_device_t* block_getdev(block_request_t* rq)
{
	// for now, only support selection by pointer
	assert(rq->selector_type == BSELECT_BY_PTR);
	
	return rq->selector.ptr;
}

void block_dump_rq(block_request_t* rq)
{
	TRACE(("Dumping block request at 0x%x\n", rq));
	
	TRACE(("selector: type=0x%x, value=0x%x\n", rq->selector_type,
		rq->selector.value));
	
	TRACE(("mode=0x%x\n", rq->mode));
	
	TRACE(("Finished\n"));
}

void block_dump_dev(block_device_t* d)
{
	TRACE(("Dumping block device at 0x%x\n", d));
	
	TRACE(("default_mode=0x%x, fn at 0x%x\n", d->default_mode, d->fn));
	
	TRACE(("Finished\n"));
}

fresult_t block_dispatch(block_request_t* rq, block_device_t* d)
{
	// just dispatch, assume everything is fine
	
	return (*(d->fn))(rq, d);	// TODO: re-do this with more thought
}

fresult_t block_addrq(block_request_t* rq)
{
	assert_ptr(rq);
	
	block_device_t* dev = block_getdev(rq);
	
	assert_ptr(dev);
	
	// dont support overrides yet
	assert(rq->mode == BMODE_DEFAULT);
	
	// only support immediate mode
	assert(dev->default_mode == BMODE_IMMEDIATE);
	
	TRACE(("Adding block request:\n"));
	block_dump_rq(rq);
	block_dump_dev(dev);
	
	return block_dispatch(rq, dev);
}

/*
 TODO:	Write TODO list!
*/
