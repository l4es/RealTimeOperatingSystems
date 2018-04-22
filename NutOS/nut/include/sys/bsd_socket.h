/**************************************************************************
*  Copyright (c) 2014 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Some parts are from the original BSD source, therefor:
*
*  Partial Copyright (c) 1982, 1986, 1988, 1990, 1993
*  The Regents of the University of California.  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*  
*  1. Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may 
*     be used to endorse or promote products derived from this software 
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
*  SUCH DAMAGE.
*
***************************************************************************
*  History:
*
*  26.01.2014  mifi  First Version
*  27.01.2014  mifi  More comments and connect added
*  28.01.2014  mifi  Backlog functionality added
**************************************************************************/
#if !defined(__BSD_SOCKET_H__)
#define __BSD_SOCKET_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>

/**************************************************************************
*  All Structures and Common Constants
**************************************************************************/

/*
 * Error code of the BSD socket api.  
 */
#define SOCKET_ERROR    -1


/*
 * Level number for (get/set)sockopt() to apply to socket itself.
 */
#define SOL_SOCKET      0xffff   /* Options for socket level */


/*
 * shutdown "How" options
 */
#define SHUT_RD         0
#define SHUT_WR         1
#define SHUT_RDWR       2


/*
 * sockaddr
 */
struct sockaddr 
{
   uint8_t sa_len;
   uint8_t sa_family;
   char    sa_data[14];
};

/*
 * in_addr
 */
struct in_addr 
{
   uint32_t s_addr;
};

/*
 * sockaddr_in
 */
struct sockaddr_in
{
   uint8_t        sin_len;
   uint8_t        sin_family;
   uint16_t       sin_port;
   struct in_addr sin_addr;
   char           sin_zero[8];
};

/*
 * socket_t
 */ 
typedef int32_t   socket_t;  

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Funtions Definitions
**************************************************************************/

/*
 * The BSD socket functions are grouped in several categories.
 * For more information take a look in: "TCP/IP Illustrated, Volume 2"
 */

/*
 * Setup category
 */
socket_t socket (int domain, int type, int proto);
int      bind (socket_t sock, struct sockaddr *addr, int addr_len);

/*
 * Server category
 */
int      listen (socket_t sock, int backlog);    /* Only backlog of 1 */
socket_t accept (socket_t sock, struct sockaddr *addr, int *addr_len);

/*
 * Client category
 */
int      connect (socket_t sock, struct sockaddr *addr, int addr_len); 

/*
 * Input category
 */
int      recv (socket_t sock, void *data, size_t len, int flags);
int      recvfrom (socket_t sock, void *data, size_t len, int flags, struct sockaddr *addr, int *addr_len);

/*
 * Output category
 */
int      send (socket_t sock, void *data, size_t len, int flags);
int      sendto (socket_t sock, void *data, size_t len, int flags, struct sockaddr *addr, int addr_len);


/*
 * Termination category
 */ 
int      closesocket (socket_t sock);
int      shutdown (socket_t sock, int how);   /* Dummy only */


/*
 * Administration category
 */
int      getsockname (socket_t sock, struct sockaddr *addr, int *addr_len);
int      getpeername (socket_t sock, struct sockaddr *addr, int *addr_len);
int      getsockopt (socket_t sock, int level, int optname, void *optval, int optlen);
int      setsockopt (socket_t sock, int level, int optname, void *optval, int optlen);

#endif /* !__BSD_SOCKET_H__ */
/*** EOF ***/
