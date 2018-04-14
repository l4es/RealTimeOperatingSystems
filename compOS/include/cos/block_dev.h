/*
 block_dev.h - block device definitions
  
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

#ifndef _COS_BLOCK_DEV_H_
#define _COS_BLOCK_DEV_H_

struct block_device;
typedef struct block_device block_device_t;

struct block_request;
typedef struct block_request block_request_t;

typedef fresult_t (*block_function_t)(block_request_t* rq, block_device_t* dev);

struct block_device
{
	block_function_t	fn;
	u32_t			default_mode;
};

struct block_request
{
	u32_t			mode;
	u32_t			selector_type;
	union {
		u32_t value;
		block_device_t*	ptr;
	} selector;
};

block_device_t* block_getdev(block_request_t* rq);
void block_dump_rq(block_request_t* rq);
void block_dump_dev(block_device_t* d);
fresult_t block_dispatch(block_request_t* rq, block_device_t* d);
fresult_t block_addrq(block_request_t* rq);

// selector types
#define BSELECT_UNKNOWN	0
#define BSELECT_BY_PTR	1

// request modes
#define BMODE_DEFAULT	0
#define BMODE_IMMEDIATE 1

#endif // !_COS_BLOCK_DEV_H_
