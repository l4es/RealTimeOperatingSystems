/*
 * Copyright (C) 2005 by egnite Software GmbH. All rights reserved.
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
 * \file fs/phatio.c
 * \brief PHAT File System.
 *
 * \verbatim
 *
 * $Log$
 * Revision 1.5  2008/08/11 06:59:42  haraldkipp
 * BSD types replaced by stdint types (feature request #1282721).
 *
 * Revision 1.4  2006/07/11 12:20:19  haraldkipp
 * PHAT file system failed when accessed from multiple threads. A mutual
 * exclusion semaphore fixes this.
 *
 * Revision 1.3  2006/02/23 15:45:22  haraldkipp
 * PHAT file system now supports configurable number of sector buffers.
 * This dramatically increased write rates of no-name cards.
 * AVR compile errors corrected.
 *
 * Revision 1.2  2006/01/22 17:38:43  haraldkipp
 * *** empty log message ***
 *
 * Revision 1.1  2006/01/05 16:31:45  haraldkipp
 * First check-in.
 *
 *
 * \endverbatim
 */

#include <errno.h>

#include <fs/fs.h>

#include <fs/phatfs.h>
#include <fs/phatvol.h>
#include <dev/blockdev.h>
#include <fs/phatio.h>

#include <sys/event.h>
#include <sys/timer.h>

#if 0
/* Use for local debugging. */
#define NUTDEBUG
#include <stdio.h>
#endif

/*!
 * \addtogroup xgPhatIo
 */
/*@{*/

/*!
 * \brief Directly write consecutive sectors.
 */
int PhatSectorWrite(NUTDEVICE * dev, uint32_t sect, const void *data, int num)
{
    int rc = 0;
    BLKPAR_SEEK pars;
    PHATVOL *vol = (PHATVOL *) dev->dev_dcb;
    NUTFILE *blkmnt = dev->dev_icb;
    NUTDEVICE *blkdev = blkmnt->nf_dev;
    int written;
    uint8_t *dp;
    int sbn;
    int sbc;

    while (num && rc == 0) {
        /* Gain mutex access. */
        NutEventWait(&vol->vol_iomutex, 0);

        /* Check if any of the sectors is locked. */
#if PHAT_SECTOR_BUFFERS
        sbc = PHAT_SECTOR_BUFFERS;
#else
        sbc = 1;
#endif
        for (sbn = 0; sbn < sbc; sbn++) {
            if (vol->vol_buf[sbn].sect_num >= sect && vol->vol_buf[sbn].sect_num < sect + num) {
                if (vol->vol_buf[sbn].sect_lock) {
                    break;
                } else {
                    vol->vol_buf[sbn].sect_dirty = 0;
                }
            }
        }
        if (sbn == sbc) {
            /* Note, that not all drivers may support multi-sector write. */
            dp = (uint8_t *) data;
            while (num) {
                pars.par_nfp = blkmnt;
                pars.par_blknum = sect;
                rc = (*blkdev->dev_ioctl) (blkdev, NUTBLKDEV_SEEK, &pars);
                if (rc) {
                    errno = EIO;
                    break;
                }
                written = (*blkdev->dev_write) (blkmnt, dp, num);
                if (written <= 0) {
                    rc = -1;
                    errno = EIO;
                    break;
                }
                num -= written;
                sect += written;
                dp += (written * vol->vol_sectsz);
            }
        }
        NutEventPost(&vol->vol_iomutex);
    }
    return rc;
}

/*!
 * \brief Flush sector buffers.
 *
 * The volume must be locked before calling this function.
 *
 * \param dev Specifies the file system device.
 * \param bufnum The buffer number to flush. If -1, all buffers are flushed.
 *
 * \return 0 on success, -1 on failures.
 */
int PhatSectorFlush(NUTDEVICE * dev, int bufnum)
{
    BLKPAR_SEEK pars;
    int sbn;
    PHATVOL *vol = (PHATVOL *) dev->dev_dcb;
    NUTFILE *blkmnt = dev->dev_icb;
    NUTDEVICE *blkdev = blkmnt->nf_dev;

    if (bufnum < 0) {
        sbn = 0;
#if PHAT_SECTOR_BUFFERS
        bufnum = PHAT_SECTOR_BUFFERS - 1;
#else
        bufnum = 0;
#endif
    }
    else {
        sbn = bufnum;
    }

    while (sbn <= bufnum) {
        if (vol->vol_buf[sbn].sect_dirty) {
            pars.par_nfp = blkmnt;
            pars.par_blknum = vol->vol_buf[sbn].sect_num;
            if ((*blkdev->dev_ioctl) (blkdev, NUTBLKDEV_SEEK, &pars)) {
                errno = EIO;
                return -1;
            }
            if ((*blkdev->dev_write) (blkmnt, vol->vol_buf[sbn].sect_data, 1) != 1) {
                errno = EIO;
                return -1;
            }
            vol->vol_buf[sbn].sect_dirty = 0;
        }
        sbn++;
    }
    return 0;
}

/*!
 * \brief Read sector.
 *
 * The volume must be locked before calling this function.
 *
 * \param blkmnt Specifies the mounted block device partition.
 * \param sect   Sector to load.
 * \param buf    Points to a buffer which will receive the sector data.
 *
 * \return 0 on success, -1 on failures.
 */
int PhatSectorRead(NUTFILE * blkmnt, uint32_t sect, uint8_t * buf)
{
    BLKPAR_SEEK pars;
    NUTDEVICE *blkdev = blkmnt->nf_dev;

    /* Set the block device's sector position. */
    pars.par_nfp = blkmnt;
    pars.par_blknum = sect;
    if ((*blkdev->dev_ioctl) (blkdev, NUTBLKDEV_SEEK, &pars)) {
        errno = EIO;
        return -1;
    }

    /* Read a single block from the device. */
    if ((*blkdev->dev_read) (blkmnt, buf, 1) != 1) {
        errno = EIO;
        return -1;
    }
    return 0;
}

/*
 * \param dev Specifies the file system device.
 * \return Buffer number on success, -1 on failures.
 */
int PhatSectorLoad(NUTDEVICE * dev, uint32_t sect)
{
    int sbn;
#if PHAT_SECTOR_BUFFERS
    int i;
#endif
    PHATVOL *vol = (PHATVOL *) dev->dev_dcb;

    /* Search for a buffer. */
    for (;;) {
        /* Gain mutex access. */
        NutEventWait(&vol->vol_iomutex, 0);

#if PHAT_SECTOR_BUFFERS
        /* Check if the requested sector is already buffered. */
        for (sbn = 0; sbn < PHAT_SECTOR_BUFFERS; sbn++) {
            if (vol->vol_buf[sbn].sect_num == sect) {
                /* Found, increase the lock and return. */
                vol->vol_buf[sbn].sect_lock++;
                vol->vol_usenext = sbn;
                NutEventPost(&vol->vol_iomutex);
                return sbn;
            }
        }

        /* Sector not loaded. Use round robin to select a new buffer. */
        for (i = vol->vol_usenext + 1;; i++) {
            if (i >= PHAT_SECTOR_BUFFERS) {
                i = 0;
            }
            if (i == vol->vol_usenext) {
                break;
            }
            if (vol->vol_buf[i].sect_num == (uint32_t) -1) {
                /* Unused buffer found, stop searching. */
                sbn = i;
                break;
            }
            if (vol->vol_buf[i].sect_lock == 0) {
                /* Unlocked buffer found, continue searching and hope to
                   find an unused one. */
                sbn = i;
            }
        }
#else
        /* Check if the requested sector is already buffered. */
        sbn = 0;
        if (vol->vol_buf[sbn].sect_num == sect) {
            /* Found, increase the lock and return. */
            vol->vol_buf[sbn].sect_lock++;
            NutEventPost(&vol->vol_iomutex);
            return sbn;
        }
#endif
        /* If the selected buffer is unlocked, use it. */
        if (vol->vol_buf[sbn].sect_lock == 0) {
            break;
        }
        /* All buffers are locked. Allow other threads to try their luck. */
        NutEventPost(&vol->vol_iomutex);
        NutSleep(1);
    }

    if (PhatSectorFlush(dev, sbn)) {
        sbn = -1;
    }
    else if (PhatSectorRead(dev->dev_icb, sect, vol->vol_buf[sbn].sect_data)) {
        sbn = -1;
    }
    else {
        vol->vol_buf[sbn].sect_num = sect;
        vol->vol_buf[sbn].sect_lock++;
#if PHAT_SECTOR_BUFFERS
// TODO: Is this fix correct for non buffered systems?
        vol->vol_usenext = sbn;
#endif
    }

    /* Release mutex access. */
    NutEventPost(&vol->vol_iomutex);

    return sbn;
}

void PhatSectorBufferRelease(NUTDEVICE * dev, int bufnum)
{
    PHATVOL *vol = (PHATVOL *) dev->dev_dcb;

    if (vol->vol_buf[bufnum].sect_lock) {
        vol->vol_buf[bufnum].sect_lock--;
    }
}

/*@}*/
