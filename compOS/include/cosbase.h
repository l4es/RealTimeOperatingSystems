/*
 cosbase.h - basic definitions as used by COS.
  
 Author:        Paul Barker
 Part of:       COS
 Created:       25/04/04
 Last Modified: 11/09/04

 Copyright (C) 2004 Paul Barker
    
    This particular header could be useful to any project and provides
    several "utility" definitions. Therefore I've decided you can do what
    you want with it, its now Public Domain.
*/

/*
26/08/04:	Changed NULL from ((void*)0) to (0)

02/09/04:	Changed name to cosbase.h
		Added UNUSED()
*/

#ifndef _COSBASE_H_
#define _COSBASE_H_

// word types, platform-dependent sizes
typedef unsigned char		half_t;
typedef unsigned short		word_t;
typedef unsigned long		dword_t;
typedef unsigned long long	quad_t;

// integer types, independent sizes
typedef signed char		i8_t;	// 8-bit, signed
typedef unsigned char		u8_t;	// 8-bit, unsigned
typedef signed short		i16_t;	// 16-bit, signed
typedef unsigned short		u16_t;	// 16-bit, unsigned
typedef signed long		i32_t;	// 32-bit, signed
typedef unsigned long		u32_t;	// 32-bit, unsigned
typedef signed long long	i64_t;	// 64-bit, signed
typedef unsigned long long	u64_t;	// 64-bit, unsigned

// integer types, dependent sizes
typedef int			int_t;
typedef unsigned int		uint_t;
typedef short			short_t;
typedef unsigned short		ushort_t;
typedef long long		long_t;
typedef unsigned long long	ulong_t;

// string types
typedef char			char_t;
typedef char*			string_t;
typedef const char*		cstring_t;

// pointer and memory types (should all be the same size)
typedef void*			pvoid_t;
typedef void*			ptr_t;
typedef const void*		cptr_t;
typedef unsigned long		ptrdiff_t;
typedef unsigned long		size_t;
typedef unsigned long		iptr_t;

// others
typedef int			count_t;	// allow negative counts
typedef signed char		bool_t;		// allow -ve values for bool

// register size
typedef unsigned int		register_t;

// files
typedef dword_t			fileh;
typedef unsigned long		fresult_t;

// boolean values
#define True 1
#define False 0
#define IsTrue(x) (x != 0)
#define IsFalse(x) (x == 0)

#define NULL (0)

// some other generally useful things
#define UNUSED(x)

#endif // !_COS_STDTYPES_H_
