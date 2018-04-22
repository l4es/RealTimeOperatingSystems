/*
 * Copyright (C) 2013 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 *
 */

/*!
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include "cfg/crt.h" /* Must be included first! */

#ifndef CRT_DISABLE_SELECT_POLL

#include "nut_io.h"

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/device.h>
#include <sys/nutdebug.h>
#include <sys/atom.h>
#include <sys/event.h>
#include <sys/select.h>

/*!
 * \addtogroup xgCrtLowio
 */
/*@{*/

#define MAX_TIMEOUT_SEC   ((~0UL) / 1000) - 1

/*!
 * \brief Wake up selects() waiting in the given wait queue list.
 *
 * NutSelectWakeup() is used to wake up the waiting selects() as soon as the
 * file state / driver state has changed.
 *
 * May not be called from interrupt context!!!
 *
 * \param wq_list   Pointer to the linked list of event queues
 *
 * \param flags     Flag set represending the events that are currently
 *                  outstandings. These flags will be matched to the flags of
 *                  the waiting select calls to only wake up selects that are
 *                  realy interested in the current event.
 */
void NutSelectWakeup(WQLIST *wq_list, uint_fast8_t flags)
{
    while (wq_list) {
        if (flags & wq_list->flags) {
            NutEventPostAsync(wq_list->wq);
        }
        wq_list = wq_list->next;
    }
}

/*!
 * \brief Wake up selects() waiting in the given wait queue list from interrupt
 *        context.
 *
 * NutSelectWakeupFromIrq() is the same as NutSelectWakeup() but shall called
 * from interrupt context.
 *
 * \param wq_list   Pointer to the linked list of event queues
 *
 * \param flags     Flag set represending the events that are currently
 *                  outstandings. These flags will be matched to the flags of
 *                  the waiting select calls to only wake up selects that are
 *                  realy interested in the current event.
 */
void NutSelectWakeupFromIrq(WQLIST *wq_list, uint_fast8_t flags)
{
    while (wq_list) {
        if (flags & wq_list->flags) {
            NutEventPostFromIrq(wq_list->wq);
        }
        wq_list = wq_list->next;
    }
}

/*!
 * \brief Helper function to manage wait queue lists
 *
 * NutSelectManageWq() must be called from the drivers dev_slect fop function
 * to manage the driver wait queue list.
 *
 * \param wq_list   Pointer to the linked list of event queues, currently added
 *                  to the drivers wait queue
 * \param wq        Event queue handle to be added or removed
 * \param flags     Flag set represending the different type of monitoring
 *                  requests for the currently managed file descriptor
 * \param cmd       select control command:
 *                  - SELECT_CMD_NOP:     Do nothing
 *                  - SELECT_CMD_INIT:    Add the event queue to the drivers
 *                                        wait queue list
 *                  - SELECT_CMD_CLEANUP: Remove the event queue from the
 *                                        drivers wait queue list
 *
 * \return Returns the current event state of the selected file
 */

void NutSelectManageWq(WQLIST **wq_list, HANDLE *wq, int flags, select_cmd_t cmd)
{
    WQLIST *wl_entry;
    WQLIST **wl_tmp;
    if ((cmd != SELECT_CMD_NOP) && (wq_list != NULL) && (wq != NULL) && (flags != 0)) {
        switch (cmd) {
            case SELECT_CMD_INIT:
                /* Add our waitqueue to the device waitqueue list */

                /* Allocate a new waitqueue list entry and put us on the wait
                   list of the current NUTFILE
                 */
                wl_entry = malloc(sizeof(WQLIST));
                wl_entry->wq = wq;
                wl_entry->flags = flags;

                /* Modifications of the wait list should be done atomic */
                NutEnterCritical();
                wl_entry->next = *wq_list;
                *wq_list = wl_entry;
                NutExitCritical();

                break;
            case SELECT_CMD_CLEANUP:
                /* Cleanup the waitqueu list of the device, remove our entry which is matched by it's address */
                wl_entry = NULL;

                /* Wait queue list processing should be done atomic */
                NutEnterCritical();
                wl_tmp = wq_list;
                /* Iterate through the waitqueue list */
                while (*wl_tmp) {
                    /* Check if we found 'our' entry */
                    if ((*wl_tmp)->wq == wq) {
                        /* Remove the list entry and stop searching */
                        wl_entry = *wl_tmp;
                        *wl_tmp = (*wl_tmp)->next;
                        break;
                    } else {
                        /* Move forward to the next entry */
                        *wl_tmp = (*wl_tmp)->next;
                    }
                }
                NutExitCritical();
                /* Free the allocated memeory of the list entry */
                if (wl_entry) {
                    free(wl_entry);
                }

                break;
            default:
                break;
        }
    }
}

/*!
 * \brief Loop through all file descriptors and call the drivers dev_select fop
 *
 * select_scan() loops through all file descriptos of the fd sets passed to
 * select(). For each fd that is represented by one ore more of the three sets
 * the drivers dev_select fop is called and the return value is added to the
 * corresponding rflags array entry.
 *
 * \param n         the highest-numbered file descriptor in any of the three
 *                  sets plus 1
 * \param flags     Flag set array which represents the different type of
 *                  monitoring requests for this file descriptor (read / write /
 *                  exception)
 * \param rflags    Flag set which will be filled with the events that had been
 *                  recorded in the meantime
 * \param wq        Event queue handle that shall be passed to the driver. This
 *                  event queue will be added to the drivers wait queue list on
 *                  the first call of select_scan and is later removed again.
 * \param cmd       select control command:
 *                  - SELECT_CMD_NOP:     Just scan for events
 *                  - SELECT_CMD_INIT:    Scan for events and add our event
 *                                        queue to the drivers wait queue list
 *                  - SELECT_CMD_CLEANUP: Scan for events and remove our event
 *                                        queue from the drivers wait queue list
 *
 * \return Returns the current event state of the selected file
 */

static int select_scan(int n, uint8_t *flags, uint8_t *rflags, HANDLE *wq, select_cmd_t cmd)
{
    NUTFILE     *fp;
    NUTDEVICE   *dev;
    int          fd;
    uint_fast8_t result;
    int          count;

    count = 0;
    /* Loop through all file descriptors */
    for (fd = 0; fd < n; fd ++) {
        /* If the fd has set one of the flags... */
        if (flags[fd]) {
            fp = __fds[fd];
            dev = fp->nf_dev;

            /* Call the select function of the driver, to check the blocking modes and error state */
            if (((dev != NULL) && (dev->dev_select != NULL) && (result = dev->dev_select(fp, flags[fd], wq, cmd))) ||
                ((dev == NULL) && (((NUTVIRTUALDEVICE *) fp)->vdv_select != NULL) && (result = ((NUTVIRTUALDEVICE *) fp)->vdv_select(fp, flags[fd], wq, cmd)))) {
                /* Update the number of changed fds */
                count ++;

                /* only include the requested flags in the result */
                result &= flags[fd];

                /* Write back the result */
                rflags[fd] |= result;
            }
        }
    }

    /* Return number of filedescriptors that have set an event */
    return count;
}

/*!
 * \brief Loop through all file descriptors and poll their state.
 *
 * do_select() loops up to three times through the file descriptors of the
 * fd sets passed to select().
 * The first loop adds an event queue to the wait queue of each file descriptor
 * and also checks if any events are just pending.
 *
 * If no event is pending, the polling loop is called a second time because the
 * status of some file descriptors could have changed in the time between
 * polling their state and adding the event queue. This prevents a race
 * condition.
 *
 * If still no events are pending we call NutEventWait() on our event queue to
 * go to sleep until any driver is waking us up or the timeout expired.
 *
 * The third polling loop then cleans up the wait queues of the file descriptors
 * by removing our event queue from the list. Further more the status of the
 * file descriptors is finally gathered.
 *
 * \param n         the highest-numbered file descriptor in any of the three
 *                  sets plus 1
 * \param flags     Flag set array which represents the different type of
 *                  monitoring requests for this file descriptor (read / write /
 *                  exception)
 * \param rflags    Flag set which will be filled with the events that had been
 *                  recorded in the meantime
 * \param to        Upper bound on the amount of time elapsed before select()
 *                  returns. If NULL, select can block indefinitely
 *
 * \return On success, the total number filedescriptor whos state has changed
 *         is returned. 0 means a timeout.
 *         On error, -1 is returned, and errno is set appropriately. The content
 *         of the three sets will be undefined.
 */

static inline int do_select(int n, uint8_t *flags, uint8_t *rflags, uint32_t to)
{
    int      count;
    HANDLE   wq = NULL;

    /* Check if any events are just set and add our event queue to the waitqueu list of this device */
    count = select_scan(n, flags, rflags, &wq, SELECT_CMD_INIT);
    if (count == 0) {
        /* We just got no result, so let's wait here. Otherwise skip waiting and continue with cleanup... */

        /* Call select_scan one more time, as events could have happend in the meanwhile. This should prevent
           reace conditions as either the scan will return the event or the event queue gets signalled in the
           meantime.

           TODO: Do we need this second scan?
         */
        count = select_scan(n, flags, rflags, NULL, SELECT_CMD_NOP);

        if (count == 0) {
            /* No events happend so far, so let's go to sleep or directly wake up again if we got signalled
               in the meantime
             */
            NutEventWait(&wq, to);
        }
    }

    /* Finally gather all set events and clean up by removing our even queue from the waitqueue lists */
    count = select_scan(n, flags, rflags, &wq, SELECT_CMD_CLEANUP);

    return count;
}


/*!
 * \brief Examine the status of file descriptors
 *
 * select() allows to monitor multiple file descriptors and wait until one or
 * more descriptors become ready for the selected I/O operation.
 * e.g. characters are available for reading of buffer space is available for
 * writing. In this context ready means that the it is possible to perform
 * the corresponding I/O operation without blocking.
 *
 * Three independend sets of file descriptors can be watched. Each of them can
 * be NULL. You can even call select with all three sets set to NULL, which
 * results in a sleep of the given timeout.
 * The file descriptor sets are modified in place and pon exit will represend
 * the file descriptors that actually changed status.
 *
 * \param n         the highest-numbered file descriptor in any of the three sets plus 1
 * \param rfds      Set of filedescriptors to be watched if a read() call will
 *                  not block (e.g. incomming characters become available for reading)
 * \param wfds      Set of filedescriptors to be watched if a write() call will not block.
 * \param exfds     Set of filedescriptors to be watched for exceptions.
 * \param timeout   Upper bound on the amount of time elapsed before select()
 *                  returns. If NULL, select can block indefinitely
 *
 * \return On success, the total number filedescriptor whos state has changed
 *         is returned. 0 means a timeout.
 *         On error, -1 is returned, and errno is set appropriately. The content
 *         of the three sets will be undefined.
 */

int select(int n, fd_set *rfds, fd_set *wfds, fd_set *exfds, struct timeval *timeout)
{
    int      rc;
    uint32_t to;
    int      i;
    int      max;
    uint_fast8_t flags;
    uint8_t *fdflags;
    uint8_t *rfdflags;
    uint_fast8_t bits;
    int      fd;

    if ((n < 0) || (n > FD_SETSIZE)) {
        errno = EINVAL;
        return -1;
    }

    to = NUT_WAIT_INFINITE;
    if (timeout) {
        if (timeout->tv_sec > MAX_TIMEOUT_SEC) {
            errno = EINVAL;
            return -1;
        }
        /* Calculate the timeout in ms, if given */
        to = ((timeout->tv_sec * 1000) + ((timeout->tv_usec + 500)/1000));
        if (to == 0) {
            /* Wait at least one ms, as 0 == NUT_WAIT_INFINITE */
            to = 1;
        }
    }

    /* Check the file descriptor sets for validity */
    max = -1;
    for (i = 0; i < FD_SETSIZE / 8; i++) {
        bits = 0;
        if (rfds != NULL)
            bits |= rfds->fd_bits[i];
        if (wfds != NULL)
            bits |= wfds->fd_bits[i];
        if (exfds != NULL)
            bits |= exfds->fd_bits[i];

        fd = i << 3; /* fd = i*8 */
        for (; bits; bits >>= 1, fd++) {
            if (fd >= n) {
                goto start_select;
            }
            if (!bits & 0x01) {
                continue;
            }
            if ((!__fds[fd]) || (__fds[fd] == NUTFILE_EOF)) {
                errno = EBADF;
                return -1;
            }
            /* Update the highest file descriptor number */
            max = fd;
        }
    }

start_select:
    /* Update n to the calculated value, which evaluates to 0 if the sets are empty */
    n = max + 1;

    /* allocate memory for the file descriptor flag list */
    fdflags  = malloc(n * sizeof(uint8_t));

    /* allocate clean memory for the file descriptor return flag list */
    rfdflags = calloc(n, sizeof(uint8_t));

    for (fd = 0; fd < n; fd ++) {
        /* Gather the flags by checking each descriptor set if the current fd is part of it */
        flags = 0;
        if ((rfds != NULL) && FD_ISSET(fd, rfds))
            flags |= WQ_FLAG_READ;
        if  ((wfds != NULL) && FD_ISSET(fd, wfds))
            flags |= WQ_FLAG_WRITE;
        if  ((exfds != NULL) && FD_ISSET(fd, exfds))
            flags |= WQ_FLAG_EXCEPT;

        /* and safe the flags into the flag list */
        fdflags[fd] = flags;
    }

    /* call do_select, which finally implemens the select functionality */
    rc = do_select(n, fdflags, rfdflags, to);

    /* Clean up the sets */
    if (rfds != NULL)
        FD_ZERO(rfds);
    if (wfds != NULL)
        FD_ZERO(wfds);
    if (exfds != NULL)
        FD_ZERO(exfds);

    /* And safe the results */
    for (fd = 0; fd < n; fd++) {
        flags = rfdflags[fd];
        if ((rfds != NULL) && (flags & WQ_FLAG_READ))
            FD_SET(fd, rfds);
        if ((wfds != NULL) && (flags & WQ_FLAG_WRITE))
            FD_SET(fd, wfds);
        if ((exfds != NULL) && (flags & WQ_FLAG_EXCEPT))
            FD_SET(fd, exfds);
    }

    /* Free allocated memory */
    free(fdflags);
    free(rfdflags);

    return rc;
}

#endif
/*@}*/
