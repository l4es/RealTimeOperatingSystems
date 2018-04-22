/**************************************************************************
*  Copyright (c) 2014 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Some parts are from the original BSD source, therefor:
*
*  Partial Copyright (c) 1982, 1986, 1988, 1990, 1993
*	The Regents of the University of California.  All rights reserved.
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
*  26.01.2014  mifi  First Version.
*  27.01.2014  mifi  More comments and connect added.
*  28.01.2014  mifi  Backlog functionality added.
*  02.02.2014  mifi  - Backlog does not work correct.
*                    - Corrected sendto.
*                    - Added support for setsockopt IP_ADD_MEMBERSHIP and
*                      IP_DROP_MEMBERSHIP.
*  04.02.2014  mifi  Corrected backlog functionality.
**************************************************************************/
#define __BSD_SOCKET_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <net/if_var.h>
#include <sys/semaphore.h>
#include <sys/bsd_socket.h>
#include <sys/socket.h>

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*
 * Wrapper for the BSD socket
 */
typedef struct _bsd_socket_
{
   int                  type;             /* Used to distinguish TCP and UDP */
   uint16_t             port;             /* Used by accept */
   SEM                  backlog_sem;      /* Used by accept and listen */
   int                  is_listen_sock;   /* Used by accept */
   uint32_t             udp_rcv_timeo;    /* Used by NutUdpReceiveFrom */
   struct _bsd_socket_ *listen;           /* Used by accept */
   
   TCPSOCKET           *nut_tcp_sock;     /* Handle to the NutNET TCP socket */
   UDPSOCKET           *nut_udp_sock;     /* Handle to the NutNET USP socket */
} bsd_socket_t;


/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  socket                                                               */
/*                                                                       */
/*  Create a new unnamed socket for the given domain and type.           */
/*                                                                       */
/*  In    : domain, type, proto                                          */
/*  Out   : none                                                         */
/*  Return: Socket on success / -1 otherwise                             */
/*************************************************************************/
socket_t socket (int domain, int type, int proto)
{
   bsd_socket_t *bsd_sock = (bsd_socket_t*)-1;
   TCPSOCKET    *tcp_sock;
   UDPSOCKET    *udp_sock;
            
   (void)proto;   /* Not used */
   
   /* Check for the correct domain */
   if (AF_INET == domain)
   {
      /* Select the type */
      switch (type)
      {
         case SOCK_STREAM:
         {
            /* Create a NutNET socket */
            tcp_sock = NutTcpCreateSocket();
            if (tcp_sock != NULL)
            {
               /* Get memory for the wrapper */
               bsd_sock = (bsd_socket_t*)malloc(sizeof(bsd_socket_t));
               if (bsd_sock != NULL)
               {
                  /* Clear the memory first */
                  memset(bsd_sock, 0x00, sizeof(bsd_socket_t));
                  
                  /* Setup wrapper */
                  bsd_sock->type         = type;
                  bsd_sock->nut_tcp_sock = tcp_sock;
                  bsd_sock->nut_udp_sock = NULL;
               }
               else
               {
                  /* No memory for the wrapper available, close NutNET socket */
                  NutTcpCloseSocket(tcp_sock);
               }
            }
            break;
         } /* SOCK_STREAM */
         
         case SOCK_DGRAM:
         {
            /* Create a NutNET socket withg port 0 first, will be changed by bind */
            udp_sock = NutUdpCreateSocket(0);
            if (udp_sock != NULL)
            {
               /* Get memory for the wrapper */
               bsd_sock = (bsd_socket_t*)malloc(sizeof(bsd_socket_t));
               if (bsd_sock != NULL)
               {
                  /* Clear the memory first */
                  memset(bsd_sock, 0x00, sizeof(bsd_socket_t));

                  /* Setup wrapper */
                  bsd_sock->type         = type;
                  bsd_sock->nut_udp_sock = udp_sock;
                  bsd_sock->nut_tcp_sock = NULL;
               }
               else
               {
                  /* No memory for the wrapper available, close NutNET socket */
                  NutUdpDestroySocket(udp_sock);
               }
            }         
            break;
         } /* SOCK_DGRAM */
         
         default:
         {
            /* Wrong type */
            bsd_sock = (bsd_socket_t*)-1;   
            break;
         }
      } /* end switch (type) */
   } /* end if (AF_INET == domain) */
   
   return( (socket_t)bsd_sock );
} /* socket */

/*************************************************************************/
/*  closesocket                                                          */
/*                                                                       */
/*  Close the socket and release the memory for the wrapper.             */
/*                                                                       */
/*  Note: The socket may not be immediately destroyed after calling this */
/*        function. However, the application must not use the socket     */
/*        after this call.                                               */
/*                                                                       */
/*  In    : sock                                                         */
/*  Out   : none                                                         */
/*  Return: 0 on success / -1 otherwise                                  */
/*************************************************************************/
int closesocket (socket_t sock)
{
   int           rc = -1;
   bsd_socket_t *bsd_sock = (bsd_socket_t*)sock;
   
   /* Select the type */
   switch (bsd_sock->type)
   {
      case SOCK_STREAM:
      {
         /* Check if the socket was not closed before */
         if (bsd_sock->nut_tcp_sock != NULL)
         {
            /* Close NutNET socket */
            rc = NutTcpCloseSocket(bsd_sock->nut_tcp_sock);
         }
         
         /* 
          * Check if a listen socket is available, this is the case
          * if this socket was created by an accept. Now the backlog
          * must be handled too. The listen socket will be closed separately.
          */
         if (bsd_sock->listen)
         {
            /* Handle backlog */            
            NutSemPost(&bsd_sock->listen->backlog_sem);
            
            /* Clear listen socket info */
            bsd_sock->listen = NULL;
         }
         
         /*
          * Check if this is the listen socket itself. In this
          * case the backlog semaphore must be deleted too.
          */
         if (bsd_sock->is_listen_sock)
         {
            NutSemDestroy(&bsd_sock->backlog_sem);
         }

         /* Clear socket info */         
         bsd_sock->nut_tcp_sock = NULL;
         
         /* Free the wrapper memory */   
         free(bsd_sock);
         break;
      } /* SOCK_STREAM */
      
      case SOCK_DGRAM:
      {
         /* Check if the socket was not closed before */
         if (bsd_sock->nut_udp_sock != NULL)
         {
            /* Close NutNET socket */
            rc = NutUdpDestroySocket(bsd_sock->nut_udp_sock);
         }   
         
         /* Free the wrapper memory */
         free(bsd_sock);
         break;
      } /* SOCK_DGRAM */
      
      default:
      {
         /* Wrong type */
         rc = -1;   
         break;
      }
   } /* end switch (bsd_sock->type) */
   
   return(rc);
} /* closesocket */

/*************************************************************************/
/*  shutdown                                                             */
/*                                                                       */
/*  Close one or both direction of the socket.                           */
/*                                                                       */
/*  Note: The socket itself must be still closed with closesocket.       */
/*                                                                       */
/*  In    : sock, how                                                    */
/*  Out   : none                                                         */
/*  Return: 0 on success / -1 otherwise                                  */
/*************************************************************************/
int shutdown (socket_t sock, int how)
{
   (void)sock;
   (void)how;
   
   /* 
    * Functionality not supported.
    * But we need the function to setup
    * the linker if the application use it.
    */
   
   return(0);
} /* shutdown */

/*************************************************************************/
/*  bind                                                                 */
/*                                                                       */
/*  Bind the local address to the socket.                                */
/*                                                                       */
/*  In    : sock, addr, addr_len                                         */
/*  Out   : none                                                         */
/*  Return: 0 on success / -1 otherwise                                  */
/*************************************************************************/
int bind (socket_t sock, struct sockaddr *addr, int addr_len)
{
   int                 rc = -1;
   bsd_socket_t       *bsd_sock = (bsd_socket_t*)sock;
   struct sockaddr_in *addr_in  = (struct sockaddr_in*)addr;
   
   /* Check for correct length */
   if (sizeof(struct sockaddr_in) == addr_len)
   {
      /* Store port info for later use, e.g. accept */
      bsd_sock->port = ntohs(addr_in->sin_port);
 
      /* Correct port in case of UDP */
      if (SOCK_DGRAM == bsd_sock->type)
      {
         /* 
          * The UDP socket was created with port 0, 
          * set now the correct port.
          */
         bsd_sock->nut_udp_sock->so_local_port = addr_in->sin_port;
      }
      
      rc = 0;
   }   
   
   return(rc);
} /* bind */

/*************************************************************************/
/*  listen                                                               */
/*                                                                       */
/*  Prepare a socket to accept incomming connections.                    */
/*                                                                       */
/*  In    : sock, backlog                                                */
/*  Out   : none                                                         */
/*  Return: 0 on success / -1 otherwise                                  */
/*************************************************************************/
int listen (socket_t sock, int backlog)
{
   int           rc       = -1;
   bsd_socket_t *bsd_sock = (bsd_socket_t*)sock;
   
   /* listen is not supported for UDP */
   if (SOCK_DGRAM == bsd_sock->type)
   {
      return(-1);
   }

   if (backlog > 0)
   {
      /* Mark socket as listen */
      bsd_sock->is_listen_sock = 1;
      
      /* Create backlog semaphore */
      NutSemInit(&bsd_sock->backlog_sem, (short)backlog);
      
      rc = 0;
   }   
   
   return(rc);
} /* listen */

/*************************************************************************/
/*  accept                                                               */
/*                                                                       */
/*  Wait for incoming connect from a remote socket.                      */
/*                                                                       */
/*  In    : sock, addr, addr_len                                         */
/*  Out   : none                                                         */
/*  Return: socket on success / -1 otherwise                             */
/*************************************************************************/
socket_t accept (socket_t sock, struct sockaddr *addr, int *addr_len)
{
   int                 rc;
   socket_t            ClientSocket = -1;
   bsd_socket_t       *bsd_sock_listen = (bsd_socket_t*)sock;
   struct sockaddr_in *addr_in     = (struct sockaddr_in*)addr;
   bsd_socket_t       *bsd_sock_client;
   
   /* accept is not supported for UDP */
   if (SOCK_DGRAM == bsd_sock_listen->type)
   {
      return(-1);
   }

   /* Check for correct length */   
   if (sizeof(struct sockaddr_in) == *addr_len)
   {
      /* Clear addr, and set default values */
      memset(addr, 0x00, *addr_len);
      addr_in->sin_family = AF_INET; 
      addr_in->sin_len    = sizeof(struct sockaddr_in);
      
      /* Check if we could create a new socket */      
      NutSemWait(&bsd_sock_listen->backlog_sem);
   
      /* Create a new TCP socket */   
      ClientSocket = socket(AF_INET, SOCK_STREAM, 0);
      if (ClientSocket != -1)
      {
         /* Convert to BSD socket */
         bsd_sock_client = (bsd_socket_t *)ClientSocket;
            
         /* Set listen information */
         bsd_sock_client->listen = bsd_sock_listen;         
            
         /* Wait for incoming connect from a remote socket */
         rc = NutTcpAccept(bsd_sock_client->nut_tcp_sock, bsd_sock_listen->port);
         if (0 == rc)
         {
            /* Set address information of the remote side */
            addr_in->sin_port        = bsd_sock_listen->nut_tcp_sock->so_remote_port;
            addr_in->sin_addr.s_addr = bsd_sock_listen->nut_tcp_sock->so_remote_addr;
         }
         else
         {
            /* Error, This should not happen */
            NutSemPost(&bsd_sock_listen->backlog_sem);
            NutTcpCloseSocket(bsd_sock_client->nut_tcp_sock);
            ClientSocket = -1;
         }
      }         
   } /* end if (sizeof(struct sockaddr_in) == *addr_len) */      
   
   return(ClientSocket);
} /* accept */

/*************************************************************************/
/*  connect                                                              */
/*                                                                       */
/*  Connect to a remote socket.                                          */
/*                                                                       */
/*  In    : sock, addr, addr_len                                         */
/*  Out   : none                                                         */
/*  Return: 0 on success / -1 otherwise                                  */
/*************************************************************************/
int connect (socket_t sock, struct sockaddr *addr, int addr_len)
{
   int                 rc = -1;
   bsd_socket_t       *bsd_sock = (bsd_socket_t*)sock;
   struct sockaddr_in *addr_in  = (struct sockaddr_in*)addr;
   uint32_t            remote_addr;
   uint16_t            remote_port;
   
   /* accept is not supported for UDP */
   if (SOCK_DGRAM == bsd_sock->type)
   {
      return(-1);
   }
   
   /* Check for correct length */   
   if (sizeof(struct sockaddr_in) == addr_len)
   {
      remote_addr = addr_in->sin_addr.s_addr;
      remote_port = htons(addr_in->sin_port);
   
      /* Connect to a remote socket */
      rc = NutTcpConnect(bsd_sock->nut_tcp_sock, remote_addr, remote_port);
   }
   
   return(rc);
} /* connect */

/*************************************************************************/
/*  recv                                                                 */
/*                                                                       */
/*  Receive data on a connected TCP socket.                              */
/*                                                                       */
/*  In    : sock, data, len, flags                                       */
/*  Out   : none                                                         */
/*  Return: 0 or > 0 on success / -1 otherwise                           */
/*************************************************************************/
int recv (socket_t sock, void *data, size_t len, int flags)
{
   int           rc;
   bsd_socket_t *bsd_sock = (bsd_socket_t*)sock;

   (void)flags;
   
   /* recv is not supported for UDP */
   if (SOCK_DGRAM == bsd_sock->type)
   {
      return(-1);
   }
   
   /* Receive data on a connected TCP socket */
   rc = NutTcpReceive(bsd_sock->nut_tcp_sock, data, (int)len);
   
   return(rc);
} /* recv */

/*************************************************************************/
/*  recvfrom                                                             */
/*                                                                       */
/*  Receive a UDP datagram.                                              */
/*                                                                       */
/*  In    : sock, data, len, flags, addr, addr_len                       */
/*  Out   : none                                                         */
/*  Return: 0 or > 0 on success / -1 otherwise                           */
/*************************************************************************/
int recvfrom (socket_t sock, void *data, size_t len, int flags, struct sockaddr *addr, int *addr_len)
{
   int                 rc = -1;
   bsd_socket_t       *bsd_sock = (bsd_socket_t*)sock;
   struct sockaddr_in *addr_in  = (struct sockaddr_in*)addr;
   uint32_t            remote_addr;
   uint16_t            remote_port;

   (void)flags;
   
   /* recvfrom is not supported for TCP */
   if (SOCK_STREAM == bsd_sock->type)
   {
      return(-1);
   }

   /* Check for correct length */   
   if (sizeof(struct sockaddr_in) == *addr_len)
   {
      /* Clear addr, and set default values */
      memset(addr, 0x00, *addr_len);
      addr_in->sin_family = AF_INET; 
      addr_in->sin_len    = sizeof(struct sockaddr_in);

      /* Receive a UDP datagram */
      rc = NutUdpReceiveFrom(bsd_sock->nut_udp_sock, &remote_addr, &remote_port,
                             data, (int)len, bsd_sock->udp_rcv_timeo);
      if (rc > 0)
      {              
         /* Set address information of the remote side */         
         addr_in->sin_port        = htons(remote_port);
         addr_in->sin_addr.s_addr = remote_addr;
      }   
   }      
                                       
   return(rc);
} /* recvfrom */

/*************************************************************************/
/*  send                                                                 */
/*                                                                       */
/*  Send data on a connected TCP socket.                                 */
/*                                                                       */
/*  In    : sock, data, len, flags                                       */
/*  Out   : none                                                         */
/*  Return: 0 or > 0 on success / -1 otherwise                           */
/*************************************************************************/
int send (socket_t sock, void *data, size_t len, int flags)
{
   int           rc = 0;
   bsd_socket_t *bsd_sock = (bsd_socket_t*)sock;
   
   (void)flags;
   
   /* send is not supported for UDP */
   if (SOCK_DGRAM == bsd_sock->type)
   {
      return(-1);
   }
   
   /* Send data on a connected TCP socket */
   rc = NutTcpSend(bsd_sock->nut_tcp_sock, data, (int)len);
   
   return(rc);
} /* send */

/*************************************************************************/
/*  sendto                                                               */
/*                                                                       */
/*  Send a UDP datagram.                                                 */
/*                                                                       */
/*  In    : sock, data, len, flags, addr, addr_len                       */
/*  Out   : none                                                         */
/*  Return: 0 or > 0 on success / -1 otherwise                           */
/*************************************************************************/
int sendto (socket_t sock, void *data, size_t len, int flags, struct sockaddr *addr, int addr_len)
{
   int                 rc = -1;
   bsd_socket_t       *bsd_sock = (bsd_socket_t*)sock;
   struct sockaddr_in *addr_in  = (struct sockaddr_in*)addr;
   uint32_t            remote_addr;
   uint16_t            remote_port;

   (void)flags;
   
   /* recvfrom is not supported for TCP */
   if (SOCK_STREAM == bsd_sock->type)
   {
      return(-1);
   }

   /* Check for correct length */   
   if (sizeof(struct sockaddr_in) == addr_len)
   {
      remote_addr = addr_in->sin_addr.s_addr;
      remote_port = htons(addr_in->sin_port);
      
      /* Send a UDP datagram */
      rc = NutUdpSendTo(bsd_sock->nut_udp_sock, remote_addr, remote_port,
                        data, (int)len);  
      if (0 == rc)
      {
         rc = (int)len;
      }
   }

   return(rc);
} /* sendto */

/*************************************************************************/
/*  getsockname                                                          */
/*                                                                       */
/*  Return the local address information bound to the socket.            */
/*                                                                       */
/*  In    : sock, addr, addr_len                                         */
/*  Out   : none                                                         */
/*  Return: 0 on success / -1 otherwise                                  */
/*************************************************************************/
int getsockname (socket_t sock, struct sockaddr *addr, int *addr_len)
{
   int                 rc = -1;
   bsd_socket_t       *bsd_sock = (bsd_socket_t*)sock;
   struct sockaddr_in *addr_in  = (struct sockaddr_in*)addr;

   /* Check for correct length */   
   if (sizeof(struct sockaddr_in) == *addr_len)
   {
      /* Clear address info first  */
      memset(addr, 0x00, *addr_len);
      
      /* Set local address information */
      addr_in->sin_len         = sizeof(struct sockaddr_in);
      addr_in->sin_family      = AF_INET; 
      addr_in->sin_port        = bsd_sock->nut_tcp_sock->so_local_port;
      addr_in->sin_addr.s_addr = bsd_sock->nut_tcp_sock->so_local_addr;
      
      rc = 0;
   }   
   
   return(rc);
} /* getsockname */  

/*************************************************************************/
/*  getpeername                                                          */
/*                                                                       */
/*  Return the remote address information bound to the socket.           */
/*                                                                       */
/*  In    : sock, addr, addr_len                                         */
/*  Out   : none                                                         */
/*  Return: 0 on success / -1 otherwise                                  */
/*************************************************************************/
int getpeername (socket_t sock, struct sockaddr *addr, int *addr_len)
{
   int                 rc = -1;
   bsd_socket_t       *bsd_sock = (bsd_socket_t*)sock;
   struct sockaddr_in *addr_in  = (struct sockaddr_in*)addr;

   /* Check for correct length */   
   if (sizeof(struct sockaddr_in) == *addr_len)
   {
      /* Clear address info first  */
      memset(addr, 0x00, *addr_len);

      /* Set remote address information */
      addr_in->sin_len         = sizeof(struct sockaddr_in);
      addr_in->sin_family      = AF_INET; 
      addr_in->sin_port        = bsd_sock->nut_tcp_sock->so_remote_port;
      addr_in->sin_addr.s_addr = bsd_sock->nut_tcp_sock->so_remote_addr;
      
      rc = 0;
   }   
   
   return(rc);
} /* getpeername */  

/*************************************************************************/
/*  getsockopt                                                           */
/*                                                                       */
/*  Get a TCP / UDP socket option value.                                 */
/*                                                                       */
/*  In    : sock, level, optname, optval, optlen                         */
/*  Out   : none                                                         */
/*  Return: 0 on success / -1 otherwise                                  */
/*************************************************************************/
int getsockopt (socket_t sock, int level, int optname, void *optval, int optlen)
{
   int           rc = -1;
   bsd_socket_t *bsd_sock = (bsd_socket_t*)sock;
   
   /* Check for the correct level */
   if (SOL_SOCKET == level)
   {
      switch (optname)
      {
         case SO_ERROR:    /* Last error is available for TCP and UDP */
         {
            /* Get the last error information, and clear it */
            if ((optval != NULL) && (sizeof(int) == optlen))
            {
               /* Last error */
               *((int*)optval) = NutTcpError(bsd_sock->nut_tcp_sock);
               
               /* Clear info */
               bsd_sock->nut_tcp_sock->so_last_error = 0;
               
               rc = 0;
            }   
            break;
         } /* SO_ERROR */
         
         default:
         {
            /* Check for stream or datagram */ 
            if (SOCK_STREAM == bsd_sock->type)
            {
               /* It is a stream socket, get the option value */
               rc = NutTcpGetSockOpt(bsd_sock->nut_tcp_sock, optname, optval, optlen);
            }
            else
            {
               /* It must be a datagram socket */
               if (SO_RCVTIMEO == optname)   /* <= special case, receive timeout */
               {
                  /* Yes, receive timeout, check optval and optlen */
                  if ((optval != NULL) && (sizeof(uint32_t) == optlen))
                  {
                     /* Get requested information */
                     *((uint32_t*)optval) = bsd_sock->udp_rcv_timeo;
                     rc = 0;
                  }   
               }
               else
               { 
                  /* Get the datagram option value */
                  rc = NutUdpGetSockOpt(bsd_sock->nut_udp_sock, optname, optval, optlen);
               }   
            }   
            break;
         } /* default */
      } /* end switch (optname) */            
   } /* end if (SOL_SOCKET == level) */
   
   return(rc);
} /* getsockopt */

/*************************************************************************/
/*  setsockopt                                                           */
/*                                                                       */
/*  Set value of a TCP socket option.                                    */
/*                                                                       */
/*  In    : sock, level, optname, optval, optlen                         */
/*  Out   : none                                                         */
/*  Return: 0 on success / -1 otherwise                                  */
/*************************************************************************/
int setsockopt (socket_t sock, int level, int optname, void *optval, int optlen)
{
   int           rc = -1;
   bsd_socket_t *bsd_sock = (bsd_socket_t*)sock;
   
   /* Check for the correct level */
   if (SOL_SOCKET == level)
   {
      /* Check for stream or datagram */
      if (SOCK_STREAM == bsd_sock->type)
      {
         /* It is a stream socket, set the option value */
         rc = NutTcpSetSockOpt(bsd_sock->nut_tcp_sock, optname, optval, optlen);
      }
      else
      {
         /* Check for receive timeout */
         if (SO_RCVTIMEO == optname)
         {
            /* Yes, receive timeout, check optval and optlen */
            if ((optval != NULL) && (sizeof(uint32_t) == optlen))
            {
               /* Set requested information */
               bsd_sock->udp_rcv_timeo = *((uint32_t*)optval);
               rc = 0;
            }   
         }
         else
         { 
            /* Set the datagram option value */
            rc = NutUdpSetSockOpt(bsd_sock->nut_udp_sock, optname, optval, optlen);
         } /* end if (SO_RCVTIMEO == optname) */   
      } /* end if (SOCK_STREAM == bsd_sock->type) */  
   } /* end if (SOL_SOCKET == level) */
      
   /* Check for the correct level */
   if (IPPROTO_IP == level)
   {
      switch (optname)
      {
         case IP_ADD_MEMBERSHIP:
         {
            struct ip_mreq *mreq; 
            
            if ((optlen == sizeof(struct ip_mreq)) && (optval != NULL))
            {
               mreq = (struct ip_mreq *)optval;
               rc = NutNetIfAddMcastAddr("eth0", mreq->imr_multiaddr.s_addr);
            }
            break;
         } /* IP_ADD_MEMBERSHIP */
         
         case IP_DROP_MEMBERSHIP:
         {
            struct ip_mreq *mreq; 
            
            if ((optlen == sizeof(struct ip_mreq)) && (optval != NULL))
            {
               mreq = (struct ip_mreq *)optval;
               rc = NutNetIfDelMcastAddr("eth0", mreq->imr_multiaddr.s_addr);
            }
            break;
         } /* IP_DROP_MEMBERSHIP */
         
         default:
         {
            /* Do nothing */
            break;
         }
      }
   } /* end if (IPPROTO_IP == level) */
      
   return(rc);
} /* setsockopt */

/*** EOF ***/
