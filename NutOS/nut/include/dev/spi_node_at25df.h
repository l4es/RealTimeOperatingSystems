#ifndef _DEV_SPI_NODE_AT25DF_H_
#define _DEV_SPI_NODE_AT25DF_H_
/*
 * Copyright (C) 2015 by Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 * \file include/dev/spi_node_at25df.h
 * \brief Low level access for Atmel AT25DF SPI DataFlash.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

#include <dev/spibus.h>
#include <dev/serialflash.h>
#include <dev/spi_at25df.h>

extern NUTSPINODE nodeAt25df0;
extern NUTSPINODE nodeAt25df1;
extern NUTSPINODE nodeAt25df2;
extern NUTSPINODE nodeAt25df3;

extern NUTSERIALFLASH flashAt25df0;
extern NUTSERIALFLASH flashAt25df1;
extern NUTSERIALFLASH flashAt25df2;
extern NUTSERIALFLASH flashAt25df3;

extern NUTDEVICE devSpiBlkAt25df0;
extern NUTDEVICE devSpiBlkAt25df1;
extern NUTDEVICE devSpiBlkAt25df2;
extern NUTDEVICE devSpiBlkAt25df3;

extern AT25DF_INFO *At25dfNodeProbe(NUTSPINODE * node);
extern int At25dfNodeLock(NUTSPINODE * node);
extern void At25dfNodeUnlock(NUTSPINODE * node);

extern int At25dfNodeTransfer(NUTSPINODE * node, uint8_t op, uint32_t parm, uint_fast8_t oplen, const void *txbuf, void *rxbuf, int xlen);
extern int At25dfNodeCommand(NUTSPINODE * node, uint8_t op, uint32_t parm, uint_fast8_t oplen);
extern int At25dfNodeWaitReady(NUTSPINODE * node, uint32_t tmo, int poll);
extern int At25dfNodeStatus(NUTSPINODE * node, uint8_t *status0, uint8_t *status1);
extern int At25dfDeviceID(NUTSPINODE * node, uint8_t *man_id, uint8_t *dev_id1, uint8_t *dev_id2);

#endif
