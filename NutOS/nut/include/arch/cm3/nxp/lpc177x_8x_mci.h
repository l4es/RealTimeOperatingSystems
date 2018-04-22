#ifndef _LPC177X_8X_MCI_H_
#define _LPC177X_8X_MCI_H_

/*
 * Copyright (C) 2012 by Rob van Lieshout (info@pragmalab.nl)
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 **************************************************************************
 *
 * Parts taken from lpc177x_8x_mci.h       2011-06-02
 *
 * file     lpc177x_8x_mci.h
 * brief    Contains all macro definitions and function prototypes
 *           support for MCI firmware library on LPC177x_8x
 * version  2.0
 * date     29. June. 2011
 * author   NXP MCU SW Application Team
 *
 * Copyright(C) 2011, NXP Semiconductor
 * All rights reserved.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors'
 * relevant copyright in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 **********************************************************************/

/* Peripheral group ----------------------------------------------------------- */

#include <cfg/arch.h>

/*----------------------------------------------------------------------------*
  MCI Public Macros
 *----------------------------------------------------------------------------*/


//#define MCI_DMA_ENABLED       (1)

#define HIGH_LVL        (1)
#define LOW_LVL         (0)

/*----------------------------------------------------------------------------*
  SD/MMC Command list, per MMC spec. SD Memory Card Spec. Simplified version
 *----------------------------------------------------------------------------*/

#define CMD0_GO_IDLE_STATE          0   /* GO_IDLE_STATE(MMC) or RESET(SD) */

#define CMD1_SEND_OP_COND           1   /* SEND_OP_COND(MMC) or ACMD41(SD) */

#define CMD2_ALL_SEND_CID           2   /* ALL_SEND_CID */

#define CMD3_SET_RELATIVE_ADDR      3   /* SET_RELATE_ADDR */

#define ACMD6_SET_BUS_WIDTH         6   /* Set Bus-Width 1 bit or 4 bits */

#define CMD7_SELECT_CARD            7   /* SELECT/DESELECT_CARD */

#define CMD8_SEND_IF_COND           8   /* Sending interface condition cmd */

#define CMD9_SEND_CSD               9   /* SEND_CSD */

#define CMD12_STOP_TRANSMISSION     12  /* Stop either READ or WRITE operation */

#define CMD13_SEND_STATUS           13  /* SEND_STATUS */

#define CMD16_SET_BLOCK_LEN         16  /* SET_BLOCK_LEN */

#define CMD17_READ_SINGLE_BLOCK     17  /* SET_BLOCK_LEN */

#define CMD18_READ_MULTIPLE_BLOCK   18  /* READ_MULTIPLE_BLOCK */

#define CMD24_WRITE_BLOCK           24  /* WRITE_BLOCK */

#define CMD25_WRITE_MULTIPLE_BLOCK  25  /* WRITE_MULTIPLE_BLOCK */

#define CMD32_ERASE_WR_BLK_START    32  /* Start erase block number */

#define CMD33_ERASE_WR_BLK_END      33  /* End erase block number */

#define CMD38_ERASE                 38  /* Start erase */



#define ACMD41_SEND_APP_OP_COND     41  /* ACMD41 for SD card */


#define CMD55_APP_CMD               55  /* APP_CMD, the following will a ACMD */

#define OCR_INDEX                   0x00FF8000


/*----------------------------------------------------------------------------*
  Card status defines
 *----------------------------------------------------------------------------*/

#define CARD_STATUS_ACMD_ENABLE     (1 << 5)
#define CARD_STATUS_RDY_DATA        (1 << 8)
#define CARD_STATUS_CURRENT_STATE   (0x0F << 9)
#define CARD_STATUS_ERASE_RESET     (1 << 13)

#define MCI_SLOW_RATE       1
#define MCI_NORMAL_RATE     2

#define SD_1_BIT            0
#define SD_4_BIT            1

#define CARD_UNKNOWN        0
#define MMC_CARD            1
#define SD_CARD             2


#define PCLK                48000000
#define SDCLK_SLOW            400000
#define SDCLK_NORMAL        10000000 


/* MCI clk freq = PCLK/(2* (Clkdiv +1) -> LPC manual sais: MCI clk freq = MCLK(2*(ClkDiv+1) !*/
#define MCLKDIV_SLOW        ((PCLK / SDCLK_SLOW) / 2 - 1)
#define MCLKDIV_NORMAL      ((PCLK / SDCLK_NORMAL) / 2 - 1)

#define DATA_TIMER_VALUE_R           ((PCLK / (2 * (MCLKDIV_NORMAL + 1))) / 4)   // 250ms
#define DATA_TIMER_VALUE_W           (PCLK / (2 * (MCLKDIV_NORMAL + 1)))         // 1000ms

#define EXPECT_NO_RESP      0
#define EXPECT_SHORT_RESP   1
#define EXPECT_LONG_RESP    2

#define MCI_OUTPUT_MODE_PUSHPULL        (0)
#define MCI_OUTPUT_MODE_OPENDRAIN       (1)

#define NOT_ALLOW_CMD_TIMER 0
#define ALLOW_CMD_TIMER     1

#define MCI_DISABLE_CMD_TIMER   (1<<8)

/*
 *  For the SD card I tested, the minimum block length is 512
 *  For MMC, the restriction is loose, due to the variety of SD and MMC
 *  card support, ideally, the driver should read CSD register to find the
 *  speed and block length for the card, and set them accordingly.
 *
 *  In this driver example, it will support both MMC and SD cards, it
 *  does read the information by send SEND_CSD to poll the card status,
 *  but, it doesn't configure them accordingly. this is not intended to
 *  support all the SD and MMC card.
 *
 */

/*
  DATA_BLOCK_LEN table

  DATA_BLOCK_LEN            Actual Size( BLOCK_LENGTH )
    11                              2048
    10                              1024
    9                                 512
    8                                 256
    7                                 128
    6                                 64
    5                                 32
    4                                 16
    3                                 8
    2                                 4
    1                                 2
*/

/* This is the size of the buffer of origin data */
#define MCI_DMA_SIZE            (1000UL)

/* This is the area original data is stored or data to be written to the SD/MMC card. */
#define MCI_DMA_SRC_ADDR        LPC_PERI_RAM_BASE

/* This is the area, after reading from the SD/MMC*/
#define MCI_DMA_DST_ADDR        (MCI_DMA_SRC_ADDR + MCI_DMA_SIZE)

/* SD-HC uses byte addressing, SD-normal uses block addressing */
#define MMC_BLOCK_MODE                  0
#define MMC_BYTE_MODE                   1   /* acces card using byte-addresses iso sector addresses */

/* To simplify the programming, please note that, BLOCK_LENGTH is a multiple of FIFO_SIZE */
#define DATA_BLOCK_LEN      9   /* Block size field in DATA_CTRL */
#define BLOCK_LENGTH        (1 << DATA_BLOCK_LEN)
                                /* for SD card, 128, the size of the flash */
                                /* card is 512 * 128 = 64K */
#define BLOCK_NUM           0x80
#define FIFO_SIZE           16

#define BUS_WIDTH_1BIT      0
#define BUS_WIDTH_4BITS     10

/* MCI Status register bit information */
#define MCI_CMD_CRC_FAIL    (1 << 0)
#define MCI_DATA_CRC_FAIL   (1 << 1)
#define MCI_CMD_TIMEOUT     (1 << 2)
#define MCI_DATA_TIMEOUT    (1 << 3)
#define MCI_TX_UNDERRUN     (1 << 4)
#define MCI_RX_OVERRUN      (1 << 5)
#define MCI_CMD_RESP_END    (1 << 6)
#define MCI_CMD_SENT        (1 << 7)
#define MCI_DATA_END        (1 << 8)
#define MCI_START_BIT_ERR   (1 << 9)
#define MCI_DATA_BLK_END    (1 << 10)
#define MCI_CMD_ACTIVE      (1 << 11)
#define MCI_TX_ACTIVE       (1 << 12)
#define MCI_RX_ACTIVE       (1 << 13)
#define MCI_TX_HALF_EMPTY   (1 << 14)
#define MCI_RX_HALF_FULL    (1 << 15)
#define MCI_TX_FIFO_FULL    (1 << 16)
#define MCI_RX_FIFO_FULL    (1 << 17)
#define MCI_TX_FIFO_EMPTY   (1 << 18)
#define MCI_RX_FIFO_EMPTY   (1 << 19)
#define MCI_TX_DATA_AVAIL   (1 << 20)
#define MCI_RX_DATA_AVAIL   (1 << 21)


/*----------------------------------------------------------------------------*
  MCI Data control register definitions
 *----------------------------------------------------------------------------*/

/* Data transfer enable */
#define MCI_DATACTRL_ENABLE_POS         (0)
#define MCI_DATACTRL_ENABLE_MASK        (0x01)
#define MCI_DATACTRL_ENABLE             (1 << MCI_DATACTRL_ENABLE_POS)
#define MCI_DATACTRL_DISABLE            (0 << MCI_DATACTRL_ENABLE_POS)

/* Data transfer direction */
#define MCI_DATACTRL_DIR_POS            (1)
#define MCI_DATACTRL_DIR_MASK           (0x01)
#define MCI_DATACTRL_DIR_FROM_CARD      (1 << MCI_DATACTRL_DIR_POS)
#define MCI_DATACTRL_DIR_TO_CARD        (0 << MCI_DATACTRL_DIR_POS)


/* Data transfer mode */
#define MCI_DATACTRL_XFER_MODE_POS      (2)
#define MCI_DATACTRL_XFER_MODE_MASK     (0x01)
#define MCI_DATACTRL_XFER_MODE_STREAM   (1 << MCI_DATACTRL_XFER_MODE_POS)
#define MCI_DATACTRL_XFER_MODE_BLOCK    (0 << MCI_DATACTRL_XFER_MODE_POS)

/* Enable DMA */
#define MCI_DATACTRL_DMA_ENABLE_POS     (3)
#define MCI_DATACTRL_DMA_ENABLE_MASK    (0x01)
#define MCI_DATACTRL_DMA_ENABLE         (1 << MCI_DATACTRL_DMA_ENABLE_POS)
#define MCI_DATACTRL_DMA_DISABLE        (0 << MCI_DATACTRL_DMA_ENABLE_POS)

/** Data block length macro */
#define MCI_DTATCTRL_BLOCKSIZE(n)   _SBF(4, (n & 0xF))


#define CMD_INT_MASK        (MCI_CMD_CRC_FAIL | MCI_CMD_TIMEOUT | MCI_CMD_RESP_END | \
                             MCI_CMD_SENT     | MCI_CMD_ACTIVE)

#define DATA_ERR_INT_MASK   (MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT | MCI_TX_UNDERRUN | \
                             MCI_RX_OVERRUN | MCI_START_BIT_ERR)

#define ACTIVE_INT_MASK     (MCI_TX_ACTIVE | MCI_RX_ACTIVE)

#define FIFO_INT_MASK       (MCI_TX_HALF_EMPTY | MCI_RX_HALF_FULL | \
                             MCI_TX_FIFO_FULL  | MCI_RX_FIFO_FULL | \
                             MCI_TX_FIFO_EMPTY | MCI_RX_FIFO_EMPTY | \
                             MCI_DATA_BLK_END )

#define FIFO_TX_INT_MASK    (MCI_TX_HALF_EMPTY)
#define FIFO_RX_INT_MASK    (MCI_RX_HALF_FULL )

#define DATA_END_INT_MASK   (MCI_DATA_END | MCI_DATA_BLK_END)

#define ERR_TX_INT_MASK     (MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT | MCI_TX_UNDERRUN | MCI_START_BIT_ERR)
#define ERR_RX_INT_MASK     (MCI_DATA_CRC_FAIL | MCI_DATA_TIMEOUT | MCI_RX_OVERRUN  | MCI_START_BIT_ERR)

/* Error code on the command response. */
#define INVALID_RESPONSE    0xFFFFFFFF



/*----------------------------------------------------------------------------*
  MCI_Public_Types MCI Public Types
 *----------------------------------------------------------------------------*/

typedef enum mci_card_state
{
    MCI_CARDSTATE_IDLE = 0,
    MCI_CARDSTATE_READY,
    MCI_CARDSTATE_IDENDTIFIED,
    MCI_CARDSTATE_STBY,
    MCI_CARDSTATE_TRAN,
    MCI_CARDSTATE_DATA,
    MCI_CARDSTATE_RCV,
    MCI_CARDSTATE_PRG,
    MCI_CARDSTATE_DIS,
} en_Mci_CardState;


typedef enum mci_func_error
{
    MCI_FUNC_OK = 0,
    MCI_FUNC_FAILED = -1,
    MCI_FUNC_BAD_PARAMETERS = -2,
    MCI_FUNC_BUS_NOT_IDLE = -3,
    MCI_FUNC_TIMEOUT = -3,
    MCI_FUNC_ERR_STATE = -4,
    MCI_FUNC_NOT_READY = -5,
} en_Mci_Func_Error;

typedef enum mci_card_type
{
    MCI_SDHC_SDXC_CARD = 3,
    MCI_SDSC_V2_CARD = 2,
    MCI_MMC_CARD = 1,
    MCI_SDSC_V1_CARD = 0,
    MCI_CARD_UNKNOWN = -1,
} en_Mci_CardType;

/*!
 * \brief Multimedia card identification register.
 */
typedef struct mci_cid
{
    /* Manufacturer ID */
    uint8_t MID;
    /* OEM/Application ID */
    uint16_t OID;
    /* Product name 8-bits higher */
    uint8_t PNM_H;
    /* Product name 32-bits Lower */
    uint32_t PNM_L;
    /* Product revision */
    uint8_t PRV;
    /* Product serial number */
    uint32_t PSN;
    /* reserved: 4 bit */
    uint8_t reserved;
    /* Manufacturing date: 12 bit */
    uint16_t MDT;
    /* CRC7 checksum: 7 bit */
    uint8_t CRC;
    /* not used, always: 1 bit always 1 */
    uint8_t unused;
} st_Mci_CardId;




/** @defgroup MCI_Public_Functions MCI Public Functions
 * @{
 */

int32_t  Lpc177x_8x_MciInit(uint8_t powerActiveLevel);
void     Lpc177x_8x_MciSendCmd(uint32_t CmdIndex, uint32_t Argument, uint32_t ExpectResp, uint32_t AllowTimeout);
int32_t  Lpc177x_8x_MciGetCmdResp(uint32_t CmdIndex, uint32_t NeedRespFlag, uint32_t *CmdRespStatus);
int32_t  Lpc177x_8x_MciCmdResp(uint32_t CmdIndex, uint32_t Argument, uint32_t ExpectResp, uint32_t *CmdResp, uint32_t AllowTimeout);

void     Lpc177x_8x_MciSetClock(uint32_t clockrate);
int32_t  Lpc177x_8x_MciSetBusWidth(uint32_t width);
int32_t  Lpc177x_8x_MciAcmd_SendOpCond(uint8_t hcsVal);
int32_t  Lpc177x_8x_MciCardInit(void);
en_Mci_CardType Lpc177x_8x_MciGetCardType(void);
int32_t  Lpc177x_8x_MciCardReset(void);
int32_t  Lpc177x_8x_MciCmd_SendIfCond(void);
int32_t  Lpc177x_8x_MciGetCID(st_Mci_CardId* cidValue);
int32_t  Lpc177x_8x_MciSetCardAddress(void);
uint32_t Lpc177x_8x_MciGetCardAddress(void);
int32_t  Lpc177x_8x_MciGetCSD(uint32_t* csdVal);
int32_t  Lpc177x_8x_MciCmd_SelectCard(void);
int32_t  Lpc177x_8x_MciGetCardStatus(int32_t* cardStatus);
uint32_t Lpc177x_8x_MciGetDataXferEndState(void);
uint32_t Lpc177x_8x_MciGetXferErrState(void);
int32_t  Lpc177x_8x_MciSetBlockLen(uint32_t blockLength);
int32_t  Lpc177x_8x_MciAcmd_SendBusWidth(uint32_t buswidth);
int32_t  Lpc177x_8x_MciCmd_StopTransmission(void);

int32_t  Lpc177x_8x_MciCmd_WriteBlock(uint32_t blockNum, uint32_t numOfBlock);
int32_t  Lpc177x_8x_MciCmd_ReadBlock(uint32_t blockNum, uint32_t numOfBlock);

int32_t  Lpc177x_8x_MciWriteBlock(uint8_t* memblock, uint32_t blockNum, uint32_t numOfBlock);
int32_t  Lpc177x_8x_MciReadBlock(uint8_t* destBlock, uint32_t blockNum, uint32_t numOfBlock);

int      Lpc177x_8x_MciIrqGetPriority(void);
void     Lpc177x_8x_MciIrqSetPriority(int);

#if MCI_DMA_ENABLED
void     Lpc177x_8x_MciDMA_IRQHandler (void);
#endif

#endif /* end _LPC177X_8X_MCI_H_ */
