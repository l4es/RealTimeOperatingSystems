#ifndef _TOOLCHAIN_CROSSWORKS_H_
#define _TOOLCHAIN_CROSSWORKS_H_
/*
 * Copyright (C) 2011-2014 by Michael Fischer
 *
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

#ifndef __ASSEMBLER__

#include <stddef.h>

/*!
 * \brief Case insensitive string comparisons.
 *
 * Not supported by CrossWorks and temporarily redirected
 * to the stricmp and strnicmp routines.
 *
 */
#define strcasecmp(s1, s2)      stricmp(s1, s2)
#define strncasecmp(s1, s2, n)  strnicmp(s1, s2, n)

/*
 * Not supported by CrossWorks, added prototypes here.
 */
int stricmp(const char *s1, const char *s2);
int strnicmp(const char *s1, const char *s2, size_t n);
char *strdup(const char *str);

/*
 * If "Enforce ANSI Checking" is enabled, which is
 * the default from the v2.x version of CrossWorks
 * the keyword asm will not be recognized. Therefore
 * the next define is needed to solve the problem.
 */
#define asm __asm__

/*
 * Define for __BEGIN_DECLS and __END_DECLS
 */
#ifdef  __cplusplus
# define __BEGIN_DECLS  extern "C" {
# define __END_DECLS    }
#else
# define __BEGIN_DECLS
# define __END_DECLS
#endif

#endif /* __ASSEMBLER__ */ 

#endif /* _TOOLCHAIN_CROSSWORKS_H_ */
/*** EOF ***/
