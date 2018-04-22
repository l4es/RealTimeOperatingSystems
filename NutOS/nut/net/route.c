/*
 * Copyright (C) 2001-2003 by egnite Software GmbH
 * Copyright (c) 1993 by Digital Equipment Corporation
 * Copyright (c) 1983, 1993 by The Regents of the University of California
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

/*!
 * \file net/route.c
 * \brief IP routing functions.
 *
 * \verbatim
 * $Id: route.c 6620 2017-03-22 11:00:02Z u_bonnes $
 * \endverbatim
 */

#include <cfg/os.h>
#include <sys/heap.h>

#include <net/if_var.h>
#include <net/route.h>

#include <netinet/ip.h>

#include <stdlib.h>
#include <string.h>
#include <memdebug.h>

#ifdef NUTDEBUG
#include <sys/osdebug.h>
#endif

/*!
 * \addtogroup xgIP
 */
/*@{*/

RTENTRY *rteList;           /*!< Linked list of routing entries. */

/*!
 * \brief Add a new entry to the IP routing table.
 *
 * Note, that there is currently no support for detecting duplicates.
 * Anyway, newer entries will be found first, because they are inserted
 * in front of older entries. However, this works only for equal masks,
 * i.e. new network routes will never overwrite old host routes.
 *
 * \param ip   Network or host IP address to be routed.
 *             Set 0 for default route.
 * \param mask Mask for this entry. -1 for host routes,
 *             0 for default or net mask for net routes.
 * \param gate Route through this gateway, otherwise 0.
 * \param dev  Network interface to use.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutIpRouteAdd(uint32_t ip, uint32_t mask, uint32_t gate, NUTDEVICE * dev)
{
    int rc = -1;
    RTENTRY *rte;
    RTENTRY *rtp;
    RTENTRY **rtpp;

    /*
     * Insert a new entry into the list, which is
     * sorted based on the mask. Host routes are
     * in front, default routes at the end and
     * networks in between.
     */
    rtp = rteList;
    rtpp = &rteList;
    while (rtp && rtp->rt_mask > mask) {
        rtpp = &rtp->rt_next;
        rtp = rtp->rt_next;
    }
    if ((rte = malloc(sizeof(RTENTRY))) != NULL) {
        rte->rt_ip = ip & mask;
        rte->rt_mask = mask;
        rte->rt_gateway = gate;
        rte->rt_dev = dev;
        rte->rt_next = rtp;
        *rtpp = rte;
        rc = 0;
    }
    return rc;
}

/*!
 * \brief Delete all route table entries for the given device.
 *
 * \param dev Pointer to the device. If NULL, it deletes all route table entries.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutIpRouteDelAll(NUTDEVICE * dev)
{
    RTENTRY **rtpp;

    rtpp = &rteList;
    while (*rtpp) {
        RTENTRY *rte = *rtpp;

        if (rte->rt_dev == dev || dev == 0) {
            /* Remove this entry from list and free this entry.*/
            *rtpp = rte->rt_next;
            free(rte);
        } else {
            /* Follow linked list.*/
            rtpp = &rte->rt_next;
        }
    }
    return 0;
}


/*!
 * \brief Delete the specified route table entry.
 *
 * All fields must exactly match an existing entry.
 *
 * \param ip   Network or host IP address of the route entry.
 * \param mask Mask for this entry. -1 for host routes,
 *             0 for default or net mask for net routes.
 * \param gate Route through this gateway, 0 for default gate.
 * \param dev  Network interface to use.
 *
 * \return 0 on success, -1 otherwise.
 */
int NutIpRouteDel(uint32_t ip, uint32_t mask, uint32_t gate, NUTDEVICE * dev)
{
    int rc = -1;
    RTENTRY **rtpp;
    RTENTRY *rte;

    for (rtpp = &rteList; *rtpp; *rtpp = (*rtpp)->rt_next) {
        rte = *rtpp;

        if (rte->rt_ip == ip && rte->rt_mask == mask && rte->rt_gateway == gate && rte->rt_dev == dev) {
            *rtpp = rte->rt_next;
            free(rte);
            rc = 0;
        }
    }
    return rc;
}

/*!
 * \brief Return an array of RTENTRY structures which contain all the current route table entries.
 *
 * The calling function is responsible for deleting the array.
 *
 * \param numEntries Points to an integer, which receives the length of the array.
 *
 * \return Pointer to the array. Will be NULL in case of an error.
 */
RTENTRY *NutIpRouteList(int *numEntries)
{
    RTENTRY *rc;
    RTENTRY *rte;
    int i;

    /* First count the number of entries. */
    *numEntries = 0;
    for (rte = rteList; rte; rte = rte->rt_next) {
        (*numEntries)++;
    }

    /* Allocate space for the result. */
    rc = (RTENTRY *) malloc(sizeof(RTENTRY) * (*numEntries));

    /* Fill in the result. */
    if (rc) {
        i = 0;
        for (rte = rteList; rte; i++, rte = rte->rt_next) {
            memcpy(&rc[i], rte, sizeof(RTENTRY));
        }
    }
    else {
        *numEntries = 0;
    }
    return rc;
}

/*!
 * \brief Used to limit route lookup recursion for gateways.
 *
 * The returned pointer points directly into the linked
 * list of all route entries, which is safe, as entries
 * are never removed or modified.
 *
 * \param ip    IP address in network byte order.
 * \param gate  Points to a buffer which optionally
 *              receives the IP address of a gateway.
 *              The pointer may be NULL, if the caller
 *              is not interested in this information.
 * \param level Recursion level.
 */
static RTENTRY *NutIpRouteRecQuery(uint32_t ip, uint32_t * gate, uint8_t level)
{
    RTENTRY *rte = 0;

    if (level < 4) {
        for (rte = rteList; rte; rte = rte->rt_next) {
            if ((ip & rte->rt_mask) == rte->rt_ip)
                break;
        }
        if (rte) {
            if (gate && rte->rt_gateway)
                *gate = rte->rt_gateway;
            if (rte->rt_dev == 0)
                rte = NutIpRouteRecQuery(rte->rt_gateway, gate, level + 1);
        }
    }
    return rte;
}

/*!
 * \brief Find a device associated with a particular IP route.
 *
 * Gateway routes will be automatically resolved up to
 * four levels of redirection.
 *
 * \param ip    IP address to find a route for, given in
 *              network byte order.
 * \param gate  Points to a buffer which optionally
 *              receives the IP address of a gateway.
 *              The pointer may be NULL, if the caller
 *              is not interested in this information.
 *
 * \return Pointer to the interface structure or NULL
 *         if no route was found.
 */
NUTDEVICE *NutIpRouteQuery(uint32_t ip, uint32_t * gate)
{
    RTENTRY *rte;

    if (gate)
        *gate = 0;
    /* Return the first interface if the IP is broadcast. This solves the
       long existing problem with bad checksums on UDP broadcasts. Many
       thanks to Nicolas Moreau for this patch. */
    if ((ip == 0xFFFFFFFF) || (IP_IS_MULTICAST(ip)))
        rte = rteList;
    else
        rte = NutIpRouteRecQuery(ip, gate, 0);

    return rte ? rte->rt_dev : 0;
}

/*@}*/
