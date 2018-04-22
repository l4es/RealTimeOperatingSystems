/*
 * Copyright (C) 2015 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
 * Copyright (C) 2008-2011 by egnite GmbH
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
 * \file dev/spi_node_at25df.c
 * \brief Low level routines for Adesto/Atmel AT25DF SPI Flash.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <cfg/memory.h>

#include <dev/blockdev.h>
#include <sys/nutdebug.h>
#include <sys/timer.h>

#include <stdlib.h>
#include <string.h>

#include <dev/spi_node_at25df.h>

/*!
 * \addtogroup xgSpiNodeAt25df
 */
/*@{*/

typedef struct _AT25DF_DCB {
    HANDLE dcb_lock;
} AT25DF_DCB;

static AT25DF_DCB dcbAt25df0 = {
    SIGNALED
};

static AT25DF_DCB dcbAt25df1 = {
    SIGNALED
};

static AT25DF_DCB dcbAt25df2 = {
    SIGNALED
};

static AT25DF_DCB dcbAt25df3 = {
    SIGNALED
};

int At25dfNodeLock(NUTSPINODE * node)
{
    AT25DF_DCB *dcb = (AT25DF_DCB *) node->node_dcb;

    return NutEventWait(&dcb->dcb_lock, NUT_WAIT_INFINITE);
}

void At25dfNodeUnlock(NUTSPINODE * node)
{
    AT25DF_DCB *dcb = (AT25DF_DCB *) node->node_dcb;

    NutEventPost(&dcb->dcb_lock);
}

/*!
 * \brief Transmit DataFlash command.
 *
 * \param node  Specifies the SPI node.
 * \param op    Command code.
 * \param parm  Command parameter.
 * \param oplen Command length.
 *
 * \return 0 on success, -1 on errors.
 */
static int At25dfNodeTransmitCmd(NUTSPINODE * node, uint8_t op, uint32_t parm, uint_fast8_t oplen)
{
    uint8_t cmd[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    NUTASSERT(oplen <= sizeof(cmd));

    cmd[0] = op;
    if (parm) {
        cmd[1] = (uint8_t) (parm >> 16);
        cmd[2] = (uint8_t) (parm >> 8);
        cmd[3] = (uint8_t) parm;
    }
    return (*((NUTSPIBUS *) node->node_bus)->bus_transfer) (node, cmd, NULL, oplen);
}

/*!
 * \brief Execute DataFlash command with data transfer.
 *
 * \param node  Specifies the SPI node.
 * \param op    Command code.
 * \param parm  Command parameter.
 * \param oplen Command length.
 * \param txbuf Pointer to the transmit data buffer, may be set to NULL.
 * \param rxbuf Pointer to the receive data buffer, may be set to NULL.
 * \param xlen  Number of byte to receive and/or transmit.
 *
 * \return 0 on success, -1 on errors.
 */
int At25dfNodeTransfer(NUTSPINODE * node, uint8_t op, uint32_t parm, uint_fast8_t oplen,
                      const void *txbuf, void *rxbuf, int xlen)
{
    int rc;
    NUTSPIBUS *bus;

    /* Sanity checks. */
    NUTASSERT(node != NULL);
    bus = (NUTSPIBUS *) node->node_bus;
    NUTASSERT(bus != NULL);
    NUTASSERT(bus->bus_alloc != NULL);
    NUTASSERT(bus->bus_transfer != NULL);
    NUTASSERT(bus->bus_release != NULL);

    rc = (*bus->bus_alloc) (node, 0);
    if (rc == 0) {
        rc = At25dfNodeTransmitCmd(node, op, parm, oplen);
        if (rc == 0 && xlen) {
            rc = (*bus->bus_transfer) (node, txbuf, rxbuf, xlen);
        }
        (*bus->bus_release) (node);
    }
    return rc;
}

/*!
 * \brief Execute DataFlash command without data.
 *
 * \param node  Specifies the SPI node.
 * \param op    Command operation code.
 * \param parm  Optional command parameter.
 * \param oplen Command length.
 *
 * \return 0 on success, -1 on errors.
 */
int At25dfNodeCommand(NUTSPINODE * node, uint8_t op, uint32_t parm, uint_fast8_t oplen)
{
    return At25dfNodeTransfer(node, op, parm, oplen, NULL, NULL, 0);
}

/*!
 * \brief Query the status of the serial flash.
 *
 * \param node      Specifies the SPI node.
 * \param status    pointer to uint16_t that shall receive the status value 
 *
 * \return 0 on success or -1 in case of an error.
 */
int At25dfNodeStatus(NUTSPINODE * node, uint8_t *status0, uint8_t *status1)
{
    int rc;
    uint8_t cmd[3] = { DFCMD_READ_STATUS, 0xFF, 0xFF };
    NUTSPIBUS *bus;

    /* Sanity checks. */
    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);

    bus = (NUTSPIBUS *) node->node_bus;
    NUTASSERT(bus->bus_alloc != NULL);
    NUTASSERT(bus->bus_transfer != NULL);
    NUTASSERT(bus->bus_wait != NULL);
    NUTASSERT(bus->bus_release != NULL);

    rc = (*bus->bus_alloc) (node, 0);
    if (rc == 0) {
        rc = (*bus->bus_transfer) (node, cmd, cmd, 3);
        if (rc == 0) {
            (*bus->bus_wait) (node, NUT_WAIT_INFINITE);
            *status0 = cmd[1];
            *status1 = cmd[2];
        }
        (*bus->bus_release) (node);
    }
    return rc;
}

/*!
 * \brief Query the device ID of the DataFlash.
 *
 * \param node  Specifies the SPI node.
 *
 * \return 0 on success or -1 in case of an error.
 */
int At25dfDeviceID(NUTSPINODE * node, uint8_t *man_id, uint8_t *dev_id1, uint8_t *dev_id2)
{
    int rc;
    uint8_t cmd[5] = { DFCMD_READ_DEVICEID, 0xFF, 0xFF, 0xFF, 0xFF };
    NUTSPIBUS *bus;

    NUTASSERT(node != NULL);
    NUTASSERT(node->node_bus != NULL);
    bus = (NUTSPIBUS *) node->node_bus;

    NUTASSERT(bus->bus_alloc != NULL);
    NUTASSERT(bus->bus_transfer != NULL);
    NUTASSERT(bus->bus_wait != NULL);
    NUTASSERT(bus->bus_release != NULL);

    rc = (*bus->bus_alloc) (node, 1000);
    if (rc == 0) {
        rc = (*bus->bus_transfer) (node, cmd, cmd, 5);
        if (rc == 0) {
            (*bus->bus_wait) (node, NUT_WAIT_INFINITE);
            *man_id  = cmd[1];
            *dev_id1 = cmd[2];
            *dev_id2 = cmd[3];
        }
        (*bus->bus_release) (node);
    }
    return (uint8_t) rc;
}

/*!
 * \brief Wait until DataFlash memory cycle finished.
 *
 * \param node Specifies the SPI node.
 * \param tmo  Number of loops (or milliseconds) to wait at max.
 * \param poll If 0, the current thread will be suspended for
 *             1 millisecond between each retry. Otherwise the
 *             polling loop will not sleep, but may be still suspended
 *             by the lower level SPI bus driver.
 *
 * \return 0 on success or -1 in case of an error.
 */
int At25dfNodeWaitReady(NUTSPINODE * node, uint32_t tmo, int poll)
{
    int      rc;
    uint8_t  sr0, sr1;

    while (((rc = At25dfNodeStatus(node, &sr0, &sr1)) == 0) && ((sr0 & 0x01) != 0)) {
        if (!poll) {
            NutSleep(1);
        }
        if (tmo-- == 0) {
            return -1;
        }
    }

    return rc;
}

/*!
 * \brief Determine the DataFlash type.
 *
 * If the board contains a known DataFlash chip, but couldn't be
 * determined by this routine, then most probably the chip select
 * configuration is wrong.
 *
 * \param node Specifies the SPI node.
 *
 * \return Pointer to a \ref AT25DF_INFO structure, which contains
 *         the relevant DataFlash parameters. If no known DataFlash
 *         is available, then NULL is returned.
 */
AT25DF_INFO *At25dfNodeProbe(NUTSPINODE * node)
{
    int_fast8_t i;
    uint8_t man_id; 
    uint8_t dev_id1; 
    uint8_t dev_id2;

    if (At25dfNodeWaitReady(node, 10, 1) == 0) {
        if (At25dfDeviceID(node, &man_id, &dev_id1, &dev_id2) == 0) {
            for (i = at25df_known_types; --i >= 0;) {
                /* Check for known DataFlash type */                
                if ((dev_id1 == at25df_info[i].at25df_id1) && 
                    (dev_id2 == at25df_info[i].at25df_id2)) {
                    return &at25df_info[i];
                }
            }
        }
    }
    return NULL;
}


#ifndef SPI_RATE_AT25DF0
#define SPI_RATE_AT25DF0  33000000
#endif

#ifndef SPI_MODE_AT25DF0
#define SPI_MODE_AT25DF0 SPI_MODE_3
#endif

/*!
 * \brief First AT25DF DataFlash SPI node implementation structure.
 */
NUTSPINODE nodeAt25df0 = {
    NULL,                       /*!< \brief Pointer to the bus controller driver, node_bus. */
    NULL,                       /*!< \brief Pointer to the bus device driver specific settings, node_stat. */
    SPI_RATE_AT25DF0,           /*!< \brief Initial clock rate, node_rate. */
    SPI_MODE_AT25DF0,           /*!< \brief Initial mode, node_mode. */
    8,                          /*!< \brief Initial data bits, node_bits. */
    0,                          /*!< \brief Chip select, node_cs. */
    &dcbAt25df0                 /*!< \brief Pointer to our private device control block, node_dcb. */
};


#ifndef SPI_RATE_AT25DF1
#define SPI_RATE_AT25DF1  33000000
#endif

#ifndef SPI_MODE_AT25DF1
#define SPI_MODE_AT25DF1 SPI_MODE_3
#endif


/*!
 * \brief Second AT25DF DataFlash SPI node implementation structure.
 */
NUTSPINODE nodeAt25df1 = {
    NULL,                       /*!< \brief Pointer to the bus controller driver, node_bus. */
    NULL,                       /*!< \brief Pointer to the bus device driver specific settings, node_stat. */
    SPI_RATE_AT25DF1,           /*!< \brief Initial clock rate, node_rate. */
    SPI_MODE_AT25DF1,           /*!< \brief Initial mode, node_mode. */
    8,                          /*!< \brief Initial data bits, node_bits. */
    0,                          /*!< \brief Chip select, node_cs. */
    &dcbAt25df1                 /*!< \brief Pointer to our private device control block, node_dcb. */
};


#ifndef SPI_RATE_AT25DF2
#define SPI_RATE_AT25DF2  33000000
#endif

#ifndef SPI_MODE_AT25DF2
#define SPI_MODE_AT25DF2 SPI_MODE_3
#endif

/*!
 * \brief Third AT25DF DataFlash SPI node implementation structure.
 */
NUTSPINODE nodeAt25df2 = {
    NULL,                       /*!< \brief Pointer to the bus controller driver, node_bus. */
    NULL,                       /*!< \brief Pointer to the bus device driver specific settings, node_stat. */
    SPI_RATE_AT25DF2,           /*!< \brief Initial clock rate, node_rate. */
    SPI_MODE_AT25DF2,           /*!< \brief Initial mode, node_mode. */
    8,                          /*!< \brief Initial data bits, node_bits. */
    0,                          /*!< \brief Chip select, node_cs. */
    &dcbAt25df2                 /*!< \brief Pointer to our private device control block, node_dcb. */
};


#ifndef SPI_RATE_AT25DF3
#define SPI_RATE_AT25DF3  33000000
#endif

#ifndef SPI_MODE_AT25DF3
#define SPI_MODE_AT25DF3 SPI_MODE_3
#endif


/*!
 * \brief Forth AT25DF DataFlash SPI node implementation structure.
 */
NUTSPINODE nodeAt25df3 = {
    NULL,                       /*!< \brief Pointer to the bus controller driver, node_bus. */
    NULL,                       /*!< \brief Pointer to the bus device driver specific settings, node_stat. */
    SPI_RATE_AT25DF3,           /*!< \brief Initial clock rate, node_rate. */
    SPI_MODE_AT25DF3,           /*!< \brief Initial mode, node_mode. */
    8,                          /*!< \brief Initial data bits, node_bits. */
    0,                          /*!< \brief Chip select, node_cs. */
    &dcbAt25df3                 /*!< \brief Pointer to our private device control block, node_dcb. */
};

/*@}*/
