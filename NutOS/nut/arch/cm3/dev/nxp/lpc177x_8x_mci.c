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
 *
 **************************************************************************
 *
 * Parts taken from lpc177x_8x_mci.c       2011-06-02
 *
 * file     lpc177x_8x_mci.c
 * brief    Contains all functions support for MCI firmware library
 *          on LPC177x_8x
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

#include <cfg/arch.h>
#include <sys/timer.h>
#include <dev/gpio.h>
#include <dev/irqreg.h>

#if defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#else
#warning "Unknown LPC familiy"
#endif

#include <arch/cm3/nxp/lpc177x_8x_mci.h>

//#define NUTDEBUG

#ifdef NUTDEBUG
#include <stdio.h>
#endif

#define _SHIFT(x)               (1 << x)
#define _XSHIFT(x, y)           (x << y)

#define SHIFT_(x)               (1 >> x)
#define XSHIFT_(x, y)           (x >> y)

/* _SBF(f,v) sets the bit field starting at position "f" to value "v".
 * _SBF(f,v) is intended to be used in "OR" and "AND" expressions:
 * e.g., "((_SBF(5,7) | _SBF(12,0xF)) & 0xFFFF)"
 */
#define _SBF(f,v) (v<<f)

#define MCI_CARDSTATUS_READYFORDATA_P0S           8

#define MCI_CARDSTATUS_CURRENTSTATE_POS           9
#define MCI_CARDSTATUS_CURRENTSTATE_BMASK         0x0F

#define CARDSTATEOF(x)      (XSHIFT_(x, MCI_CARDSTATUS_CURRENTSTATE_POS) & MCI_CARDSTATUS_CURRENTSTATE_BMASK)

#define MCI_CMD8_VOLTAGESUPPLIED_POS              8
#define MCI_CMD8_VOLTAGESUPPLIED_BMASK            0xFF

#define MCI_CMD8_CHECKPATTERN_POS                 0
#define MCI_CMD8_CHECKPATTERN_BMASK               0xFF

#define MCI_ACMD41_HCS_POS                        30

#define MCI_PWRCTRL_BMASK                         0xC3

#define MCI_PWRCTRL_OPENDRAIN_POS                 6
#define MCI_PWRCTRL_OPENDRAIN_NUMBIT              1
#define MCI_PWRCTRL_OPENDRAIN_BMASK               0x01

#define MCI_CID_MANUFACTURER_ID_WPOS              24            /* in word 0 */
#define MCI_CID_MANUFACTURER_ID_WBMASK            0xFF

#define MCI_CID_OEMAPPLICATION_ID_WPOS            8             /* in word 0 */
#define MCI_CID_OEMAPPLICATION_ID_WBMASK          0xFFFF

#define MCI_CID_PRODUCTNAME_ID_H_WPOS             0             /* in word 0 */
#define MCI_CID_PRODUCTNAME_ID_H_WBMASK           0xFF

#define MCI_CID_PRODUCTNAME_ID_L_WPOS             0             /* in word 1 */
#define MCI_CID_PRODUCTNAME_ID_L_WBMASK           0xFFFFFFFF

#define MCI_CID_PRODUCTREVISION_ID_WPOS           24            /* in word 2 */
#define MCI_CID_PRODUCTREVISION_ID_WBMASK         0xFF

#define MCI_CID_PRODUCTSERIALNUM_ID_H_WPOS        0             /* in word 2 */
#define MCI_CID_PRODUCTSERIALNUM_ID_H_WBMASK      0x00FFFFFF
#define MCI_CID_PRODUCTSERIALNUM_ID_L_WPOS        24            /* in word 3 */
#define MCI_CID_PRODUCTSERIALNUM_ID_L_WBMASK      0xFF
#define MCI_CID_PRODUCTSERIALNUM_ID_WBMASK        0xFFFFFFFF

#define MCI_CID_RESERVED_ID_WPOS                  20            /* in word 3 */
#define MCI_CID_RESERVED_ID_WBMASK                0x1F

#define MCI_CID_MANUFACTURINGDATE_ID_WPOS         8             /* in word 3 */
#define MCI_CID_MANUFACTURINGDATE_ID_WBMASK       0x0FFF

#define MCI_CID_CHECKSUM_ID_WPOS                  1             /* in word 3 */
#define MCI_CID_CHECKSUM_ID_WBMASK                0x7F

#define MCI_CID_UNUSED_ID_WPOS                    0             /* in word 3 */
#define MCI_CID_UNUSED_ID_WBMASK                  0x01

#define SD_INTERRUPT_PRIORITY                     3             /* make sure this is higher then DMA or GPIO */

volatile uint32_t Mci_Data_Xfer_End = 0;

volatile uint32_t Mci_Data_Xfer_ERR = 0;

volatile uint8_t fifo_plane = 0;

volatile uint32_t CardRCA;

volatile uint8_t CCS;

volatile en_Mci_CardType MCI_CardType;

/* Terminal Counter flag, Error Counter flag for Channel 0 */
uint32_t dmaWrCh_TermianalCnt, dmaWrCh_ErrorCnt;
uint32_t dmaRdCh_TermianalCnt, dmaRdCh_ErrorCnt;


uint32_t Lpc177x_8x_MciSettingDma(uint8_t* memBuf, uint32_t ChannelNum, uint32_t DMAMode );

static void Lpc177x_8x_MciIRQHandler (void *arg);
static int32_t Lpc177x_8x_MciReadFifo(uint32_t * dest);
static int32_t Lpc177x_8x_MciWriteFifo(uint32_t * src);

void Lpc177x_8x_MciTXEnable( void );
void Lpc177x_8x_MciRXEnable( void );
void Lpc177x_8x_MciTXDisable( void );
void Lpc177x_8x_MciRXDisable( void );

void Lpc177x_8x_MciCmdProcess( void );
void Lpc177x_8x_MciDataErrorProcess( void );
void Lpc177x_8x_MciDATA_END_InterruptService( void );
void Lpc177x_8x_MciFIFOInterruptService( void );

int32_t Lpc177x_8x_MciCheckStatus(void);



/* These pointers get an initial value here, but will be overwritten when 'Lpc177x_8x_MciWriteBlock()' is called.... */
volatile int32_t  dataLenTx = 0;
volatile int32_t  dataLenRx = 0;
volatile uint8_t* dataSrcBlock = (uint8_t *) MCI_DMA_SRC_ADDR;
volatile uint8_t* dataDestBlock = (uint8_t *) MCI_DMA_DST_ADDR;;

volatile uint32_t txBlockCnt=0, rxBlockCnt=0;


/*********************************************************************//**
 * \brief       MCI_IRQHandler is to manage the reasons that cause the
 *              interrupt.
 *
 * \details     It controls the data-block writing and reading by access
 *              the FIFO register.
 *              It handle the state changes on the MCI bus...
 *
 * \param       None
 *
 * \return      None
 **********************************************************************/
static void Lpc177x_8x_MciIRQHandler (void * arg)
{
    uint32_t MCI_Status;

    MCI_Status = LPC_MCI->STATUS;


    /* handle MCI_STATUS interrupt */
    if (MCI_Status & DATA_ERR_INT_MASK)
    {
        Lpc177x_8x_MciDataErrorProcess();

        return;
    }

    if (MCI_Status & DATA_END_INT_MASK)
    {
        Lpc177x_8x_MciDATA_END_InterruptService();

        return;
    }
    else if (MCI_Status & FIFO_INT_MASK)
    {
        Lpc177x_8x_MciFIFOInterruptService();

        return;
    }
    else if (MCI_Status & CMD_INT_MASK)
    {
        Lpc177x_8x_MciCmdProcess();

        return;
    }
}

/*************************************************************************
 * \brief       Read data from FIFO (after a transmission with card) to
 *              a destination buffer
 *
 * \param       *dest The buffer to store the data that read from card
 *
 * \return      MCI_FUNC_OK
 *************************************************************************/
static int32_t Lpc177x_8x_MciReadFifo(uint32_t * dest)
{
    uint8_t i;
    uint8_t start, end;

    if (fifo_plane == 0)
    {
        start = 0;
        end = 7;
    }
    else
    {
        start = 8;
        end = 15;
    }
    fifo_plane = (fifo_plane) ? 0:1;

    for (i = start; i <= end; i++)
    {
        *dest = LPC_MCI->FIFO[i];

        dest++;
    }

    return(MCI_FUNC_OK);
}


/*********************************************************************//**
 * \brief       Write data from a source buffer to FIFO for transmission
 *
 * \param       *src The buffer hold the data need to write to card
 *
 * \return      MCI_FUNC_OK
 *************************************************************************/
static int32_t Lpc177x_8x_MciWriteFifo(uint32_t * src)
{
    uint8_t i;
    uint8_t start, end;

    if (fifo_plane == 0)
    {
        start = 0;
        end = 7;
    }
    else
    {
        start = 8;
        end = 15;
    }
    fifo_plane = (fifo_plane) ? 0:1;

    for (i = start; i <= end; i++)
    {
        LPC_MCI->FIFO[i] = *src;

        src++;
    }

    return(MCI_FUNC_OK);
}


/*********************************************************************//**
 * \brief       Enable Transmit data interrupt
 *
 * \param       None
 *
 * \return      None
 *************************************************************************/
void Lpc177x_8x_MciTXEnable( void )
{
    /* FIFO TX interrupts only */
    LPC_MCI->MASK0 |= ((FIFO_TX_INT_MASK)|(DATA_END_INT_MASK)|(ERR_TX_INT_MASK));

    return;
}


/*********************************************************************//**
 * \brief       Disable Transmit data interrupt
 *
 * \param       None
 *
 * \return      None
 *************************************************************************/
void Lpc177x_8x_MciTXDisable( void )
{
    /* FIFO TX interrupts only */
    LPC_MCI->MASK0 &= ~((FIFO_TX_INT_MASK)|(DATA_END_INT_MASK)|(ERR_TX_INT_MASK));

    return;
}


/*********************************************************************//**
 * \brief       Enable Receive data interrupt
 *
 * \param       None
 *
 * \return      None
 *************************************************************************/
void Lpc177x_8x_MciRXEnable( void )
{
    /* FIFO RX interrupts only */
    LPC_MCI->MASK0 |= ((FIFO_RX_INT_MASK)|(DATA_END_INT_MASK)|(ERR_RX_INT_MASK));

    return;
}


/*********************************************************************//**
 * \brief       Disable Receive data interrupt
 *
 * \param       None
 *
 * \return      None
 *************************************************************************/
void Lpc177x_8x_MciRXDisable( void )
{
    /* FIFO TX interrupts only */
    LPC_MCI->MASK0 &= ~((FIFO_RX_INT_MASK)|(DATA_END_INT_MASK)|(ERR_RX_INT_MASK));

    return;
}


/*********************************************************************//**
 * \brief       Check status when working with data-block transfer
 *
 * \details     Right after the block read and write, this routine is important
 *              that, even the FIFO is empty, complete block has been sent, but,
 *              data is still being written to the card. This routine is to
 *              ensure that the data has been written based on the state of
 *              the card, not by the length being set.
 *
 * \param       None
 *
 * \return      MCI_FUNC_OK if all success
 *************************************************************************/
int32_t Lpc177x_8x_MciCheckStatus(void)
{
    int32_t respValue, retval = MCI_FUNC_FAILED;

    /*
     *  this code will keep reading the status of the card, untill the
     *  card is READYFORDATA or the card is in TRAN state.
     *  Only if the card command itself fails (so no repsonse could be
     *  retrieved), this code will stop.
     */
    while (1)
    {
        if (Lpc177x_8x_MciGetCardStatus(&respValue) != MCI_FUNC_OK)
        {
#ifdef NUTDEBUG
            printf("%s() GetCardStatus failed\n", __FUNCTION__ );
#endif
            break;
        }
        else
        {
            /*
             *  The only valid state is TRANS per MMC and SD state diagram.
             *  RCV state may be seen, but, it happens only when TX_ACTIVE or
             *  RX_ACTIVE occurs before the WRITE_BLOCK and READ_BLOCK cmds are
             *  being sent, which is not a valid sequence.
             */

            if (!(respValue & _SHIFT(MCI_CARDSTATUS_READYFORDATA_P0S)))
            {
                retval = MCI_FUNC_NOT_READY;
#ifdef NUTDEBUG
                printf("%s() Card not ready\n", __FUNCTION__ );
#endif
            }
            else if (CARDSTATEOF(respValue) != MCI_CARDSTATE_TRAN)
            {
                /* Should be in STANDBY state now and ready */
                retval = MCI_FUNC_ERR_STATE;
#ifdef NUTDEBUG
                printf("%s() Invalid state\n", __FUNCTION__ );
#endif
            }
            else
            {
                return(MCI_FUNC_OK);
            }
        }
    }

    return(retval);
}


/*********************************************************************//**
 * \brief       Called by MCI interrupt handler to simplify the command
 *              process.
 *
 * \param       None
 *
 * \return      None
 *
 * \note        In card initialization, the commnad interrupts are disabled
 *************************************************************************/
void Lpc177x_8x_MciCmdProcess( void )
{
    uint32_t MCIStatus;

    MCIStatus = LPC_MCI->STATUS;

    if (MCIStatus &  MCI_CMD_CRC_FAIL)
    {
        LPC_MCI->CLEAR =  MCI_CMD_CRC_FAIL;
    }

    if (MCIStatus &  MCI_CMD_TIMEOUT)
    {
        LPC_MCI->CLEAR =  MCI_CMD_TIMEOUT;
    }

    /* Cmd Resp End or Cmd Sent */
    if (MCIStatus &  MCI_CMD_RESP_END)
    {
        LPC_MCI->CLEAR =  MCI_CMD_RESP_END;
    }

    if (MCIStatus &  MCI_CMD_SENT)
    {
        LPC_MCI->CLEAR =  MCI_CMD_SENT;
    }

    if (MCIStatus &  MCI_CMD_ACTIVE)
    {
        LPC_MCI->CLEAR =  MCI_CMD_ACTIVE;
    }

    return;
}


/*********************************************************************//**
 * \brief       Called by MCI interrupt handler to manage error on the bus
 *
 * \param       None
 *
 * \return      None
 *************************************************************************/
void Lpc177x_8x_MciDataErrorProcess( void )
{
    uint32_t MCIStatus;

    MCIStatus = LPC_MCI->STATUS;

    if (MCIStatus &  MCI_DATA_CRC_FAIL)
    {
        LPC_MCI->CLEAR = MCI_DATA_CRC_FAIL;
    }

    if (MCIStatus &  MCI_DATA_TIMEOUT)
    {
        LPC_MCI->CLEAR =  MCI_DATA_TIMEOUT;
    }

    /* Underrun or overrun */
    if (MCIStatus &  MCI_TX_UNDERRUN)
    {
        LPC_MCI->CLEAR = MCI_TX_UNDERRUN;
    }

    if (MCIStatus &  MCI_RX_OVERRUN)
    {
        LPC_MCI->CLEAR =  MCI_RX_OVERRUN;
    }

    /* Start bit error on data signal */
    if (MCIStatus &  MCI_START_BIT_ERR)
    {
        LPC_MCI->CLEAR =  MCI_START_BIT_ERR;
    }

    Mci_Data_Xfer_End = 0;
    Mci_Data_Xfer_ERR = MCIStatus;

    return;
}


/*********************************************************************//**
 * \brief   Called by MCI interrupt handler. This is the last interrupt
 *                manipulates the process of the data-block write and read
 *                to/with card
 *
 * \details This service is also used with/without DMA support. It simply
 *                clears the flag messages the in-process data-block transfer
 *                has been done/ended
 *
 * \param       None
 *
 * \return  None
 *************************************************************************/
void Lpc177x_8x_MciDATA_END_InterruptService( void )
{
    uint32_t MCIStatus;

    MCIStatus = LPC_MCI->STATUS;

    if (MCIStatus &  MCI_DATA_END)        /* Data end, and Data block end  */
    {
        LPC_MCI->CLEAR = MCI_DATA_END;

        Mci_Data_Xfer_End = 0;

        Mci_Data_Xfer_ERR = 0;

        Lpc177x_8x_MciTXDisable();

        Lpc177x_8x_MciRXDisable();

        return;
    }

    if (MCIStatus &  MCI_DATA_BLK_END)
    {
        LPC_MCI->CLEAR =  MCI_DATA_BLK_END;

        return;
    }

    /* Tx active  */
    if (MCIStatus & MCI_TX_ACTIVE)
    {

    }

    /* Rx active  */
    if (MCIStatus & MCI_RX_ACTIVE)
    {

    }

    return;
}


/*********************************************************************//**
 * \brief       Called by MCI interrupt handler if requiring to using FIFO
 *              for data transferring. It copy data to/from FIFO register
 *              from/to a data buffer.
 *
 * \param       None
 *
 * \return      None
 *
 * \note        This function is done without DMA transfer support
 **********************************************************************/
void Lpc177x_8x_MciFIFOInterruptService( void )
{
    uint32_t MCIStatus;


    MCIStatus = LPC_MCI->STATUS;

    if (MCIStatus & (FIFO_TX_INT_MASK ))
    {

        // empty is multiple of 512 block size
        if (MCIStatus & MCI_TX_HALF_EMPTY)
        {

            // There's no data, return
            if ((dataSrcBlock == NULL) || (dataLenTx <= 0))
            {
                /* disable FIFO int until next block write */
                LPC_MCI->MASK0 &= ~(FIFO_TX_INT_MASK);
                return;
            }

            // write 8 words to fifo
            Lpc177x_8x_MciWriteFifo((uint32_t *)&dataSrcBlock[txBlockCnt]);

            txBlockCnt += 32;
            dataLenTx  -= 32;

//            LPC_MCI->CLEAR = MCI_TX_HALF_EMPTY;       // no need to clear the pending interrupt here?
        }

        if (txBlockCnt == BLOCK_LENGTH) /* block complete */
        {
            dataSrcBlock += BLOCK_LENGTH;

            txBlockCnt = 0;

            // now wait for SD card to complete sending data i.e MCI_DATA_BLK_END interrupt
        }
    }
    else if (MCIStatus & (FIFO_RX_INT_MASK))
    {
        // if using RX_HALF_FULL remove one ReadFIFO below
        if (MCIStatus & MCI_RX_HALF_FULL)
        {
            if ((dataDestBlock == NULL) || (dataLenRx <= 0))
            {
                /* disable FIFO int until next block read */
                LPC_MCI->MASK0 &= ~(FIFO_RX_INT_MASK);
                return;
            }

            // read 8 words from fifo
            Lpc177x_8x_MciReadFifo((uint32_t *)&dataDestBlock[rxBlockCnt]);

            rxBlockCnt += 32;
            dataLenRx  -= 32;

//            LPC_MCI->CLEAR = MCI_RX_HALF_FULL;        // no need to clear the pending interrupt here?
        }

        // block complete
        if (rxBlockCnt == BLOCK_LENGTH)
        {
            dataDestBlock += BLOCK_LENGTH;

            rxBlockCnt = 0;

            // now wait for SD card to complete sending data i.e MCI_DATA_BLK_END interrupt
        }
    }
    return;
}


/*********************************************************************//**
 * \brief       Set MCI clock rate, during initialization phase < 400K
 *              during data phase < 20Mhz
 *
 * \param[in]   ClockRate Clock rate to be set
 *
 * \return      None
 **********************************************************************/
void Lpc177x_8x_MciSetClock( uint32_t ClockRate )
{
    uint32_t ClkValue = 0;

#ifdef USE_CLOCK_SETTINGS
    uint32_t PeriClock;

    /*
     *  ClockRate is fixed in this code (#define in headerfile), but better would be to get
     *  the peripheral clock and see what the maximum speed could be.
     *  However, tests with the IAR LPC1788 dev-board show that the maximum clock rate MCI_NORMAL
     *  is only slighty higher then the clock rate that goes with MCI_SLOW_RATE.
     *  So no use to calculate the clockrate here....
     */
    PeriClock = Lpc17xx_ClockGet(NUT_HWCLK_PERIPHERAL);
#endif

    if (ClockRate == MCI_SLOW_RATE)
    {
        /* slow clock */
        ClkValue |= MCLKDIV_SLOW;
    }
    else if (ClockRate == MCI_NORMAL_RATE)
    {
        /* normal clock */
        ClkValue |= MCLKDIV_NORMAL;
    }

    LPC_MCI->CLOCK &= ~(0xFF); /* clear clock divider */

    LPC_MCI->CLOCK |= (1 << 8)  | ClkValue;

    NutMicroDelay(1);               //  delay 3MCLK + 2PCLK before next write

    return;
}


/**********************************************************************//**
 * \brief       Set the Width to 1-bit Bus or 4-bit Bus
 *
 * \param[in]   width buswidth expected to set
 *
 * \return      MCI_FUNC_OK in case of success
 *************************************************************************/
int32_t Lpc177x_8x_MciSetBusWidth( uint32_t width )
{
    NutMicroDelay(1);   /* delay 3MCLK + 2PCLK  */

    if (width == SD_1_BIT)
    {
        LPC_MCI->CLOCK &=  ~(1 << 11);  /* 1 bit bus */
    }
    else if (width == SD_4_BIT)
    {
        LPC_MCI->CLOCK |=  (1 << 11);/* 4 bit bus */
    }

    if (Lpc177x_8x_MciAcmd_SendBusWidth( BUS_WIDTH_4BITS ) != MCI_FUNC_OK)
    {
        return(MCI_FUNC_FAILED);
    }

    return(MCI_FUNC_OK);
}


/************************************************************************//**
 * \brief       Do initialization the MCI block as set its clock, registers,
 *              setup NVIC for interrupts, configure the pins used for MCI
 *              function, do initialize the card in slot...
 *
 * \param[in]   powerActiveLevel the power level to activate the card in slot
 *
 * \return      MCI_FUNC_OK in case of success
 ***************************************************************************/
int32_t Lpc177x_8x_MciInit(uint8_t powerActiveLevel )
{
    MCI_CardType = MCI_CARD_UNKNOWN;


#if 0
    /******* GPIO Pin initialisation has to be done in the users program prior to device initialisation ********/

    /*
     *  Following block of code added to ensure card VCC drops to zero before card is initialized
     *
     *  PragmaLab 15-08-2012: -> with next lines of code enabled, it is not possible to execute the Init more then
     *  one time in a row. We need 2 times (RegisterDevive and _open both call this Init routine)
     *
     */

    // Force all MCI control pins to basic I/O mode
    GpioPinConfigSet(NUTGPIO_PORT0, 19, GPIO_CFG_OUTPUT);      // SD_CLK
    GpioPinConfigSet(NUTGPIO_PORT0, 20, GPIO_CFG_OUTPUT);      // SD_CMD
    GpioPinConfigSet(NUTGPIO_PORT1, 5, GPIO_CFG_OUTPUT);       // SD_PWR
    GpioPinConfigSet(NUTGPIO_PORT0, 22, GPIO_CFG_OUTPUT);      // SD_DAT_0
    GpioPinConfigSet(NUTGPIO_PORT1, 7, GPIO_CFG_OUTPUT);       // SD_DAT_1
    GpioPinConfigSet(NUTGPIO_PORT1, 11, GPIO_CFG_OUTPUT);      // SD_DAT_2
    GpioPinConfigSet(NUTGPIO_PORT1, 12, GPIO_CFG_OUTPUT);      // SD_DAT_3

    // Force all pins low (except power control pin)
    GpioPinSetLow(NUTGPIO_PORT0, 19);       // CLK
    GpioPinSetLow(NUTGPIO_PORT0, 20);       // CMD
    GpioPinSetHigh(NUTGPIO_PORT1, 5);       // PWR
    GpioPinSetLow(NUTGPIO_PORT0, 22);       // DAT0
    GpioPinSetLow(NUTGPIO_PORT1, 7);        // DAT1
    GpioPinSetLow(NUTGPIO_PORT1, 11);       // DAT2
    GpioPinSetLow(NUTGPIO_PORT1, 12);       // DAT3

    // Crude delay of 50ms
    NutSleep(50);
#endif

    LPC_SC->PCONP |= ( 1 << 28 );           // Enable clock to the MCI block

    if (LPC_MCI->CLOCK & (1 << 8))
    {
        LPC_MCI->CLOCK &= ~(1 << 8);        // disable clock for now
    }

    if (LPC_MCI->POWER & 0x02)
    {
        LPC_MCI->POWER = 0x00;
    }

    NutDelay(1);

    // Disable all interrupts for now
    LPC_MCI->MASK0 = 0;

    /*
     *  now re-configure all pins to serve as SDIO
     *
     *  PragmaLab 18-09-2012: make sure you configure the SDIO pins only once at startup, no need
     *  to re-configure them here
     *

    GpioPinConfigSet(NUTGPIO_PORT0, 19, GPIO_CFG_PERIPHERAL2);      // SD_CLK
    GpioPinConfigSet(NUTGPIO_PORT0, 20, GPIO_CFG_PERIPHERAL2);      // SD_CMD
    GpioPinConfigSet(NUTGPIO_PORT1, 5, GPIO_CFG_PERIPHERAL2);       // SD_PWR
    GpioPinConfigSet(NUTGPIO_PORT0, 22, GPIO_CFG_PERIPHERAL2);      // SD_DAT_0
    GpioPinConfigSet(NUTGPIO_PORT1, 7, GPIO_CFG_PERIPHERAL2);       // SD_DAT_1
    GpioPinConfigSet(NUTGPIO_PORT1, 11, GPIO_CFG_PERIPHERAL2);      // SD_DAT_2
    GpioPinConfigSet(NUTGPIO_PORT1, 12, GPIO_CFG_PERIPHERAL2);      // SD_DAT_3
     */

    // SD_PWR is active high (follows the output of the SD Card interface block).
    if (powerActiveLevel == LOW_LVL)
    {
        LPC_SC->SCS &= ~ 0x08;
    }
    else
    {
        LPC_SC->SCS |= 0x08;
    }

    // Setting for timeout problems
    LPC_MCI->DATATMR = 0x1FFFFFFF;

    // set up clocking default mode, clear any registers as needed
    LPC_MCI->COMMAND = 0;
    LPC_MCI->DATACTRL = 0;
    LPC_MCI->CLEAR = 0x7FF;             // clear all pending interrupts

    LPC_MCI->POWER = 0x02;              // power up
    while (!(LPC_MCI->POWER & 0x02));   // wait till power is there....

    NutMicroDelay(10);

    /*
     *  During identification phase, the clock should be less than
     *  400Khz. Once we pass this phase, the normal clock can be set up
     *  to 25Mhz on SD card and 20Mhz on MMC card.
     */
    Lpc177x_8x_MciSetClock(MCI_SLOW_RATE );

    LPC_MCI->POWER |= 0x01;     /* bit 1 is set already, from power up to power on */

    NutMicroDelay(200);

    NutRegisterIrqHandler(&sig_MCI, Lpc177x_8x_MciIRQHandler, 0);
    NutIrqEnable (&sig_MCI);

    Lpc177x_8x_MciIrqSetPriority(SD_INTERRUPT_PRIORITY);

#ifdef NUTDEBUG
    printf("SD int priority: %d\n", Lpc177x_8x_MciIrqGetPriority());
#endif

    /*  During the initialization phase, to simplify the process, the CMD related
     *  interrupts are disabled. The DATA related interrupts are enabled when
     *  the FIFOs are used and just before WRITE_BLOCK READ_BLOCK cmds are issues, and
     *  disabled after the data block has been written and read. Please also note,
     *  before WRITE_BLOCK only TX related data interrupts are enabled, and before
     *  READ_BLOCK only RX related data interrupts are enabled.
     */

    return(Lpc177x_8x_MciCardInit());
}


/************************************************************************//**
 * \brief       Set output in open drain mode or pushpull mode
 *
 * \param[in]   mode the mode going to set
 *
 * \return      None
 ***************************************************************************/
void Lpc177x_8x_MciSetOutputMode(uint32_t mode)
{
    if (mode == MCI_OUTPUT_MODE_OPENDRAIN)
    {
        /* Set Open Drain output control for MMC */
        LPC_MCI->POWER |= (1 << MCI_PWRCTRL_OPENDRAIN_POS) & MCI_PWRCTRL_BMASK;
    }
    else
    {
        /* Clear Open Drain output control for SD */
        LPC_MCI->POWER &= (~(1 << MCI_PWRCTRL_OPENDRAIN_POS) & MCI_PWRCTRL_BMASK);
    }
}


/************************************************************************//**
 * \brief       The routine is used to send a CMD to the card
 *
 * \param[in]   CmdIndex the command to be sent to cards
 *
 * \param[in]   Argument the argument follows the command
 *
 * \param[in]   ExpectResp the response type for the command. They may be:
 *              - EXPECT_NO_RESP: means no response required
 *              - EXPECT_SHORT_RESP: means a response in a word needed
 *              - EXPECT_LONG_RESP: means a response in 4 words needed
 *
 * \param[in]   AllowTimeout allow timeout the command or not
 *
 * \return      None
 ***************************************************************************/
void Lpc177x_8x_MciSendCmd(uint32_t CmdIndex, uint32_t Argument, uint32_t ExpectResp, uint32_t AllowTimeout)
{
    uint32_t CmdData = 0;
    uint32_t CmdStatus;

    /*
     * the command engine must be disabled when we modify the argument
     * or the peripheral resends
     */
    while ((CmdStatus = LPC_MCI->STATUS) & MCI_CMD_ACTIVE)    /* Command in progress. */
    {
        LPC_MCI->COMMAND = 0;
        LPC_MCI->CLEAR = CmdStatus | MCI_CMD_ACTIVE;
    }

    NutMicroDelay(10);

    // set the command details, the CmdIndex should 0 through 0x3F only
    CmdData |= (CmdIndex & 0x3F);                 /* bit 0 through 5 only */

    if (ExpectResp == EXPECT_NO_RESP)             /* no response */
    {
        CmdData &= ~((1 << 6) | (1 << 7));        /* Clear long response bit as well */
    }
    else if (ExpectResp == EXPECT_SHORT_RESP)     /* expect short response */
    {
        CmdData |= (1 << 6);
    }
    else if (ExpectResp == EXPECT_LONG_RESP)      /* expect long response */
    {
        CmdData |= (1 << 6) | (1 << 7);
    }

    if (AllowTimeout == ALLOW_CMD_TIMER)          /* allow timeout or not */
    {
        CmdData &= ~ MCI_DISABLE_CMD_TIMER;
    }
    else
    {
        CmdData |= MCI_DISABLE_CMD_TIMER;
    }

    /* send the command */
    CmdData |= (1 << 10);                         /* This bit needs to be set last. */

    LPC_MCI->ARGUMENT = Argument;                 /* Set the argument first, finally command */

    LPC_MCI->COMMAND = CmdData;

    return;
}


/************************************************************************//**
 * \brief       The routine is to get the reponse from card after commands.
 *              This function is always used in pair of Lpc177x_8x_MciSendCmd() func
 *
 * \param[in]   ExpectCmdData specify the command of which the data will be
 *              retrieved. This field should be the same with CmdIndex of
 *              Lpc177x_8x_MciSendCmd() function.
 *
 * \param[in]   ExpectResp the response type for the command. They may be:
 *              - EXPECT_NO_RESP: means no response required
 *              - EXPECT_SHORT_RESP: means a response in a word needed
 *              - EXPECT_LONG_RESP: means a response in 4 words needed
 *
 * \param[out]  CmdResp the buffer stored the data replied from cards
 *
 * \return      MCI_FUNC_OK in case of success
 ***************************************************************************/
int32_t Lpc177x_8x_MciGetCmdResp(uint32_t ExpectCmdData, uint32_t ExpectResp, uint32_t *CmdResp)
{
    uint32_t CmdRespStatus = 0;
    uint32_t LastCmdIndex;

    if (ExpectResp == EXPECT_NO_RESP)
    {
        return(MCI_FUNC_OK);
    }

    while (1)
    {
        // Get the status of the component
        CmdRespStatus = LPC_MCI->STATUS;

        if (CmdRespStatus & (MCI_CMD_TIMEOUT))
        {
            LPC_MCI->CLEAR = CmdRespStatus | MCI_CMD_TIMEOUT;


            LPC_MCI->COMMAND = 0;
            LPC_MCI->ARGUMENT = 0xFFFFFFFF;

            return(CmdRespStatus);
        }

        if (CmdRespStatus & MCI_CMD_CRC_FAIL)
        {
            LPC_MCI->CLEAR = CmdRespStatus | MCI_CMD_CRC_FAIL;
            LastCmdIndex = LPC_MCI->COMMAND & 0x003F;

            if ((LastCmdIndex == CMD1_SEND_OP_COND)
                || (LastCmdIndex == ACMD41_SEND_APP_OP_COND)
                || (LastCmdIndex == CMD12_STOP_TRANSMISSION)
               )
            {
                LPC_MCI->COMMAND = 0;
                LPC_MCI->ARGUMENT = 0xFFFFFFFF;

                // ignore CRC error if it's a resp for SEND_OP_COND or STOP_TRANSMISSION.
                break;
            }
            else
            {
                return(CmdRespStatus);
            }
        }
        else if (CmdRespStatus & MCI_CMD_RESP_END)
        {
            LPC_MCI->CLEAR = CmdRespStatus | MCI_CMD_RESP_END;
            break;          // cmd response is received, expecting response
        }

    }

    if ((LPC_MCI->RESP_CMD & 0x3F) != ExpectCmdData)
    {
        /*
         *  If the response is not R1, in the response field, the Expected Cmd data
         *  won't be the same as the CMD data in SendCmd(). Below four cmds have
         *  R2 or R3 response. We don't need to check if MCI_RESP_CMD is the same
         *  as the Expected or not.
         */
        if ((ExpectCmdData != CMD1_SEND_OP_COND)
            && (ExpectCmdData != ACMD41_SEND_APP_OP_COND)
            && (ExpectCmdData != CMD2_ALL_SEND_CID)
            && (ExpectCmdData != CMD9_SEND_CSD)
           )
        {
            CmdRespStatus = INVALID_RESPONSE;   /* Reuse error status */
            return(INVALID_RESPONSE);
        }
    }

    /* Read MCI_RESP0 register assuming it's not long response. */
    if (CmdResp != NULL)
    {
        if (ExpectResp == EXPECT_SHORT_RESP)
        {
            *(CmdResp + 0) = LPC_MCI->RESP0;
            *(CmdResp + 1) = 0;
            *(CmdResp + 2) = 0;
            *(CmdResp + 3) = 0;
        }
        else if (ExpectResp == EXPECT_LONG_RESP)
        {
            *(CmdResp + 0) = LPC_MCI->RESP0;
            *(CmdResp + 1) = LPC_MCI->RESP1;
            *(CmdResp + 2) = LPC_MCI->RESP2;
            *(CmdResp + 3) = LPC_MCI->RESP3;
        }
    }

    return(MCI_FUNC_OK);
}


/************************************************************************//**
 * \brief       The routine is to send command to cards then get back the
 *              reponses (if required).
 *
 * \param[in]   CmdIndex the command to be sent to cards
 *
 * \param[in]   Argument the argument follows the command
 *
 * \param[in]   ExpectResp the response type for the command. They may be:
 *              - EXPECT_NO_RESP: means no response required
 *              - EXPECT_SHORT_RESP: means a response in a word needed
 *              - EXPECT_LONG_RESP: means a response in 4 words needed
 *
 * \param[out]  CmdResp the buffer stored the data replied from cards
 *
 * \param[in]   AllowTimeout allow timeout the command or not
 *
 * \return      MCI_FUNC_OK in case of success
 ***************************************************************************/
int32_t Lpc177x_8x_MciCmdResp(uint32_t CmdIndex, uint32_t Argument,
                    uint32_t ExpectResp, uint32_t *CmdResp, uint32_t AllowTimeout)
{
    int32_t respStatus;

    Lpc177x_8x_MciSendCmd(CmdIndex, Argument, ExpectResp, AllowTimeout);

    if ((CmdResp != NULL) || (ExpectResp != EXPECT_NO_RESP))
    {
        respStatus = Lpc177x_8x_MciGetCmdResp(CmdIndex, ExpectResp, CmdResp);
    }
    else
    {
        respStatus = MCI_FUNC_BAD_PARAMETERS;
    }

#ifdef NUTDEBUG
    if(respStatus & (MCI_CMD_TIMEOUT))
    {
        printf("%s() failed with MCI_CMD_TIMEOUT\n", __FUNCTION__ );
    }
    if(respStatus & (MCI_CMD_CRC_FAIL))
    {
        printf("%s() failed with MCI_CMD_CRC_FAIL\n", __FUNCTION__ );
    }
    if(respStatus == INVALID_RESPONSE)
    {
        printf("%s() failed with INVALID_RESPONSE\n", __FUNCTION__ );
    }
    if(respStatus == MCI_FUNC_BAD_PARAMETERS)
    {
        printf("%s() failed with MCI_FUNC_BAD_PARAMETERS\n", __FUNCTION__ );
    }
#endif

    return(respStatus);
}


/************************************************************************//**
 * \brief       To reset the card, the CMD0 is sent and then the card is put
 *              in idle state. This is the very first command to be sent to
 *              initialize either MMC or SD card.
 *
 * \param       None
 *
 * \return      Always MCI_FUNC_OK
 ***************************************************************************/
int32_t Lpc177x_8x_MciCardReset(void)
{
    /*
     *  Because CMD0 command to put the device to idle state does not need response
     *  since, it's only sending commad
     */
    Lpc177x_8x_MciSendCmd(CMD0_GO_IDLE_STATE, 0x00000000, EXPECT_NO_RESP, 0);

    return(MCI_FUNC_OK);
}


/************************************************************************//**
 * \brief       Send CMD1 (SEND_OP_COND) to card.
 *
 * \param       None
 *
 * \return      MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t Lpc177x_8x_MciCmd_SendOpCond( void )
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    int32_t retval = MCI_FUNC_FAILED;

    retryCount = 0x200;         /* reset retry counter */

    while (retryCount > 0)
    {
        respStatus = Lpc177x_8x_MciCmdResp(CMD1_SEND_OP_COND, OCR_INDEX, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus & MCI_CMD_TIMEOUT)
        {
            retval = MCI_FUNC_TIMEOUT;
        }
        else if ((respValue[0] & 0x80000000) == 0)
        {
            // the card has not finished the power up routine
            retval = MCI_FUNC_BUS_NOT_IDLE;
        }
        else
        {
            retval = MCI_FUNC_OK;
            break;
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_TIMEOUT)
    {
        printf("%s() failed with MCI_FUNC_TIMEOUT\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_BUS_NOT_IDLE)
    {
        printf("%s() failed with MCI_FUNC_BUS_NOT_IDLE\n", __FUNCTION__ );
    }
#endif

    return(retval);
}


/************************************************************************//**
 * \brief       Send CMD8 (SEND_IF_COND) for interface condition to card.
 *
 * \param       None
 *
 * \return      MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t Lpc177x_8x_MciCmd_SendIfCond(void)
{
    uint32_t retryCount;
    uint32_t CmdArgument;
    uint32_t respStatus;
    uint32_t respValue[4];

    int32_t retval = MCI_FUNC_FAILED;

    uint8_t voltageSupplied = 0x01; // in range 2.7-3.6V
    uint8_t checkPattern = 0xAA;

    CmdArgument = (voltageSupplied << MCI_CMD8_VOLTAGESUPPLIED_POS) | checkPattern;

    retryCount = 0x20;

    while (retryCount > 0)
    {
        respStatus = Lpc177x_8x_MciCmdResp(CMD8_SEND_IF_COND, CmdArgument, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus & (MCI_CMD_TIMEOUT))
        {
            // consider as no response
            retval = MCI_FUNC_TIMEOUT;
        }
        else if ((respValue[0] & MCI_CMD8_CHECKPATTERN_BMASK) != checkPattern)
        {
            retval = MCI_FUNC_FAILED;
        }
        else if (((respValue[0] >> MCI_CMD8_VOLTAGESUPPLIED_POS) & MCI_CMD8_VOLTAGESUPPLIED_BMASK)
                 != voltageSupplied)
        {
            retval = MCI_FUNC_BAD_PARAMETERS;
        }
        else
        {
            retval = MCI_FUNC_OK;
            break;
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_TIMEOUT)
    {
        printf("%s() failed with MCI_FUNC_TIMEOUT\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_FAILED)
    {
        printf("%s() failed with MCI_FUNC_FAILED\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_BAD_PARAMETERS)
    {
        printf("%s() failed with MCI_FUNC_BAD_PARAMETERS\n", __FUNCTION__ );
    }
#endif

    return(retval);
}


/************************************************************************//**
 * \brief       Send CMD55 (APP_CMD) to indicate to the card that the next
 *              command is an application specific command rather than a
 *              standard command. Before an ACMD, call this routine first
 *
 * \param       None
 *
 * \return      MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t Lpc177x_8x_MciCmd_SendACMD( void )
{
    uint32_t retryCount;
    uint32_t CmdArgument;
    uint32_t respStatus;
    uint32_t respValue[4];

    int32_t retval = MCI_FUNC_FAILED;

    if ((MCI_CardType == MCI_SDSC_V1_CARD) ||
        (MCI_CardType == MCI_SDSC_V2_CARD) ||
        (MCI_CardType == MCI_SDHC_SDXC_CARD))
    {
        CmdArgument = CardRCA;  /* Use the address from SET_RELATIVE_ADDR cmd */
    }
    else            /* if MMC or unknown card type, use 0x0. */
    {
        CmdArgument = 0x00000000;
    }

    retryCount = 0x20;

    while (retryCount > 0)
    {
        respStatus = Lpc177x_8x_MciCmdResp(CMD55_APP_CMD, CmdArgument, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus & (MCI_CMD_TIMEOUT))     // filter out TIMEOUT error to be able to quickly respond missing cards
        {
            retval = MCI_FUNC_TIMEOUT;
        }
        else if (respStatus != 0)               // not one of MCI_ errors coming from the status register
        {
            retval = MCI_FUNC_FAILED;
        }
        else if (respValue[0] & CARD_STATUS_ACMD_ENABLE)
        {
            retval = MCI_FUNC_OK;
            break;
        }
        else
        {
            retval = MCI_FUNC_NOT_READY;
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_TIMEOUT)
    {
        printf("%s() failed with MCI_FUNC_TIMEOUT\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_FAILED)
    {
        printf("%s() failed with MCI_FUNC_FAILED\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_NOT_READY)
    {
        printf("%s() failed with MCI_FUNC_NOT_READY\n", __FUNCTION__ );
    }
#endif

    return(retval);
}


/************************************************************************//**
 * \brief       Send ACMD41 (SEND_APP_OP_COND) to Host Capacity Support (HCS)
 *              information and asks the accessed card to send its operating
 *              condition (OCR).
 *
 * \param[in]   hcsVal input the Host Capacity Support
 *
 * \return      MCI_FUNC_OK if all success
 *
 * \note        If SEND_APP_OP_COND is timeout, the card in the slot is not MMC
 *              type, try this combination to see if we can communicate with
 *              a SD type.
 ****************************************************************************/
int32_t Lpc177x_8x_MciAcmd_SendOpCond(uint8_t hcsVal)
{
    uint32_t retryCount;
    uint32_t respStatus, argument;
    uint32_t respValue[4];

    int32_t retval = MCI_FUNC_FAILED;

    argument = OCR_INDEX | (hcsVal << MCI_ACMD41_HCS_POS);

    /*
     * timeout on SEND_OP_COND command on MMC, now, try SEND_APP_OP_COND
     * command to SD
     */
    retryCount = 0x200;         /* reset retry counter */

    while (retryCount > 0)
    {
        if ((retval = Lpc177x_8x_MciCmd_SendACMD()) == MCI_FUNC_OK)
        {
            respStatus = Lpc177x_8x_MciCmdResp(ACMD41_SEND_APP_OP_COND, argument, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

            if (respStatus & (MCI_CMD_TIMEOUT))
            {
                retval = MCI_FUNC_TIMEOUT;
            }
            else if (!(respValue[0] & 0x80000000))
            {
                retval = MCI_FUNC_BUS_NOT_IDLE;
            }
            else
            {
                CCS = (respValue[0] & (1<<30)) ? 1:0;
                retval = MCI_FUNC_OK;
                break;
            }
        }
        else
        {
            /*
             *  check for TIMEOUT errors here and treat them in a special way. In case no card is present, we
             *  want to quit the interrogration asap.
             */
            if (retval & (MCI_CMD_TIMEOUT))
            {
                retval = MCI_FUNC_TIMEOUT;
                break;
            }
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_CMD_TIMEOUT)
    {
        printf("%s() failed with MCI_CMD_TIMEOUT\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_BUS_NOT_IDLE)
    {
        printf("%s() failed with MCI_FUNC_BUS_NOT_IDLE\n", __FUNCTION__ );
    }
#endif

    return(retval);
}

/************************************************************************//**
 * \brief       Do initialization for the card in the slot
 *
 * \details     Try CMD1 first for MMC, if it's timeout, try CMD55
 *              and CMD41 for SD, if both failed, initialization faliure,
 *              bailout with unknown card type. Otherwise, return the
 *              card type, either MMC or SD. <<<KHOA_110708: checking>>>
 *
 *              This is followed Figure 4-2: Card Initialization and
 *              Identification Flow (SD mode) in Physical Layer Simplified
 *              Specification Version 2.00 document
 *
 * \param       None
 *
 * \return      MCI_FUNC_OK if success
 ****************************************************************************/
int32_t Lpc177x_8x_MciCardInit( void )
{
    int32_t retval = MCI_FUNC_FAILED;

    MCI_CardType = MCI_CARD_UNKNOWN;

    // expect no response, so no returnvalue to check...
    Lpc177x_8x_MciCardReset();

    /* Clear Open Drain output control for SD */
    Lpc177x_8x_MciSetOutputMode(MCI_OUTPUT_MODE_PUSHPULL);

    NutMicroDelay(300);

    retval = Lpc177x_8x_MciCmd_SendIfCond();

    if (retval == MCI_FUNC_BAD_PARAMETERS)
    {
        // Unknow card is unusable
        return(retval);
    }

    /*
     *  please note that MCI_FUNC_TIMEOUT error is allowed here
     */

    if (retval == MCI_FUNC_OK) /* Ver2.00 or later*/
    {
        // Check in case of High Capacity Supporting Host
        if ((retval = Lpc177x_8x_MciAcmd_SendOpCond(1)) == MCI_FUNC_OK)
        {
            MCI_CardType = MCI_SDSC_V2_CARD;//SDSC

            if (CCS)
            {
                MCI_CardType = MCI_SDHC_SDXC_CARD;//SDHC or SDXC
            }

            return(MCI_FUNC_OK); /* Found the card, it's a HD */
        }
    }

    if (retval != MCI_FUNC_OK) /* voltage mismatch (ver2.00) or ver1.X SD Card or not SD Card*/
    {

        // Check in case of Standard Capacity Supporting Host
        if ((retval = Lpc177x_8x_MciAcmd_SendOpCond(0)) == MCI_FUNC_OK)
        {
            MCI_CardType = MCI_SDSC_V1_CARD;// Support Standard Capacity only

            return(MCI_FUNC_OK); /* Found the card, it's a SD */
        }
    }

    if (retval != MCI_FUNC_OK)
    {
        /* Set Open Drain output control for MMC */
        Lpc177x_8x_MciSetOutputMode(MCI_OUTPUT_MODE_OPENDRAIN);
        NutMicroDelay(300);

        /*
         *  Try CMD1 first for MMC, if it's timeout, try CMD55 and CMD41 for SD,
         *  if both failed, initialization faIlure, bailout.
         */
        if (Lpc177x_8x_MciCmd_SendOpCond() == MCI_FUNC_OK)
        {
            MCI_CardType = MCI_MMC_CARD;

            return(MCI_FUNC_OK); /* Found the card, it's a MMC */
        }
    }

#ifdef NUTDEBUG
    printf("%s()failed\n", __FUNCTION__ );
#endif

    /* tried both MMC and SD card, give up */
    return(MCI_FUNC_FAILED);
}


/************************************************************************//**
 * \brief       Get the type of card that is currently used in the slot
 *
 * \param       None
 *
 * \return      Card Type: MMC Card or SD card
 ****************************************************************************/
en_Mci_CardType Lpc177x_8x_MciGetCardType(void)
{
    return(MCI_CardType);
}


/************************************************************************//**
 * \brief       Get the all the Identifier (CID) of the card by sending the
 *              CMD2 (ALL_SEND_CID) command. Then parse 4-byte data obtained
 *              from the card by the CID meaning.
 *
 * \param[out]  cidValue the CID Result after parsing the data from the card
 *
 * \return      MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t Lpc177x_8x_MciGetCID(st_Mci_CardId* cidValue)
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    /* This command is normally after CMD1(MMC) or ACMD41(SD). */
    retryCount = 0x20;      /* reset retry counter */

    while (retryCount > 0)
    {
        respStatus = Lpc177x_8x_MciCmdResp(CMD2_ALL_SEND_CID, 0, EXPECT_LONG_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        /* bit 0 and bit 2 must be zero, or it's timeout or CRC error */
        //if ((!(respStatus & MCI_CMD_TIMEOUT)) && (!(respStatus & MCI_CMD_CRC_FAIL)))
        if (!(respStatus & MCI_CMD_TIMEOUT))
        {
            // Parsing the data retrieved
            if (cidValue != NULL)
            {
                cidValue->MID = (respValue[0] >> MCI_CID_MANUFACTURER_ID_WPOS) & MCI_CID_MANUFACTURER_ID_WBMASK;

                cidValue->OID = (respValue[0] >> MCI_CID_OEMAPPLICATION_ID_WPOS) & MCI_CID_OEMAPPLICATION_ID_WBMASK;

                cidValue->PNM_H = (respValue[0] >> MCI_CID_PRODUCTNAME_ID_H_WPOS) & MCI_CID_PRODUCTNAME_ID_H_WBMASK;

                cidValue->PNM_L = (respValue[1] >> MCI_CID_PRODUCTNAME_ID_L_WPOS) & MCI_CID_PRODUCTNAME_ID_L_WBMASK;

                cidValue->PRV = (respValue[2] >> MCI_CID_PRODUCTREVISION_ID_WPOS) & MCI_CID_PRODUCTREVISION_ID_WBMASK;

                cidValue->PSN = (((respValue[2] >> MCI_CID_PRODUCTSERIALNUM_ID_H_WPOS) & MCI_CID_PRODUCTSERIALNUM_ID_H_WBMASK) << 8)
                                | ((respValue[3] >> MCI_CID_PRODUCTSERIALNUM_ID_L_WPOS) & MCI_CID_PRODUCTSERIALNUM_ID_L_WBMASK);

                cidValue->reserved = (respValue[3] >> MCI_CID_RESERVED_ID_WPOS) & MCI_CID_RESERVED_ID_WBMASK;

                cidValue->MDT = (respValue[3] >> MCI_CID_MANUFACTURINGDATE_ID_WPOS) & MCI_CID_MANUFACTURINGDATE_ID_WBMASK;

                cidValue->CRC = (respValue[3] >> MCI_CID_CHECKSUM_ID_WPOS) & MCI_CID_CHECKSUM_ID_WBMASK;

                cidValue->unused = (respValue[3] >> MCI_CID_UNUSED_ID_WPOS) & MCI_CID_UNUSED_ID_WBMASK;

            }

            return(MCI_FUNC_OK);    /* response is back and correct. */
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    printf("%s() failed\n", __FUNCTION__ );
#endif

    return(MCI_FUNC_TIMEOUT);
}


/************************************************************************//**
 * \brief       Set the address for the card in the slot by sending CMD3
 *              (SET_RELATIVE_ADDR) command. To get the address of the card
 *              currently in used, needs to call Lpc177x_8x_MciGetCardAddress()
 *
 * \param       None
 *
 * \return      MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t Lpc177x_8x_MciSetCardAddress( void )
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];
    uint32_t CmdArgument;

    int32_t retval = MCI_FUNC_FAILED;

    /*
     * If it's a SD card, SET_RELATIVE_ADDR is to get the address
     * from the card and use this value in RCA, if it's a MMC, set default
     * RCA addr. 0x00010000.
     */
    if ((MCI_CardType == MCI_SDSC_V1_CARD) ||
        (MCI_CardType == MCI_SDSC_V2_CARD) ||
        (MCI_CardType == MCI_SDHC_SDXC_CARD))
    {
        CmdArgument = 0;
    }
    else            /* If it's unknown or MMC_CARD, fix the RCA address */
    {
        CmdArgument = 0x00010000;
    }

    retryCount = 0x20;          /* reset retry counter */

    while (retryCount > 0)
    {
        /* Send CMD3 command repeatedly until the response is back correctly */
        respStatus = Lpc177x_8x_MciCmdResp(CMD3_SET_RELATIVE_ADDR, CmdArgument, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus & (MCI_CMD_TIMEOUT))
        {
            retval = MCI_FUNC_TIMEOUT;
        }
        else if (!(XSHIFT_(respValue[0], MCI_CARDSTATUS_READYFORDATA_P0S) & 0x01))
        {
            retval = MCI_FUNC_NOT_READY;
        }
        else if ((CARDSTATEOF(respValue[0]) != MCI_CARDSTATE_IDENDTIFIED))
        {
            retval = MCI_FUNC_ERR_STATE;
        }
        else
        {
            CardRCA = respValue[0] & 0xFFFF0000;   /* Save the RCA value from SD card */

            return(MCI_FUNC_OK);   /* response is back and correct. */
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_TIMEOUT)
    {
        printf("%s() failed with MCI_FUNC_TIMEOUT\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_NOT_READY)
    {
        printf("%s() failed with MCI_FUNC_NOT_READY\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_ERR_STATE)
    {
        printf("%s() failed with MCI_FUNC_ERR_STATE\n", __FUNCTION__ );
    }
#endif

    return(retval);
}


/************************************************************************//**
 * \brief       Get the address for the card in the slot
 *
 * \param       None
 *
 * \return      MCI_FUNC_OK if all success
 *
 * \note        This function must be called after Lpc177x_8x_MciSetCardAddress() executing
 ****************************************************************************/
uint32_t Lpc177x_8x_MciGetCardAddress(void)
{
    return(CardRCA);
}


/************************************************************************//**
 * \brief       Get the Card-Specific Data of in-slot card by sending CMD9
 *              (SEND_CSD) command
 *
 * \param[out]  csdVal a buffer stored the value of CSD obtained from the card
 *
 * \return      MCI_FUNC_OK if all success
 *
 * \note        CMD9 (SEND_CSD) command should be sent only at standby state
 *              (STBY) after CMD3
 ****************************************************************************/
int32_t Lpc177x_8x_MciGetCSD(uint32_t* csdVal)
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];
    uint32_t CmdArgument;

    if ((MCI_CardType == MCI_SDSC_V1_CARD) ||
        (MCI_CardType == MCI_SDSC_V2_CARD) ||
        (MCI_CardType == MCI_SDHC_SDXC_CARD))
    {
        CmdArgument = CardRCA;
    }
    else            /* if MMC or unknown card type, use default RCA addr. */
    {
        CmdArgument = 0x00010000;
    }

    retryCount = 0x20;

    while (retryCount > 0)
    {
        /* Send SET_BLOCK_LEN command before read and write */
        LPC_MCI->CLEAR |= (MCI_CMD_TIMEOUT | MCI_CMD_CRC_FAIL | MCI_CMD_RESP_END);

        respStatus = Lpc177x_8x_MciCmdResp(CMD9_SEND_CSD, CmdArgument, EXPECT_LONG_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (!respStatus)
        {
            if (csdVal != NULL)
            {
                csdVal[0] = respValue[0];
                csdVal[1] = respValue[1];
                csdVal[2] = respValue[2];
                csdVal[3] = respValue[3];
            }

            return(MCI_FUNC_OK);
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    printf("%s() failed\n", __FUNCTION__ );
#endif

    return(MCI_FUNC_FAILED);
}


/************************************************************************//**
 * \brief       Select the card by the specified address. This is done by sending
 *              out the CMD7 with the address argument to needed card
 *
 * \param       None
 *
 * \return      MCI_FUNC_OK if all success
 *
 * \note        CMD7 (SELECT_CARD) command should be sent after CMD9 ((SEND_CSD).
 *              The state will be inter-changed between STBY to TRANS by this
 *              CMD7 command
 ****************************************************************************/
int32_t Lpc177x_8x_MciCmd_SelectCard( void )
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];
    uint32_t CmdArgument;

    int32_t retval = MCI_FUNC_FAILED;

    if ((MCI_CardType == MCI_SDSC_V1_CARD) ||
        (MCI_CardType == MCI_SDSC_V2_CARD) ||
        (MCI_CardType == MCI_SDHC_SDXC_CARD))
    {
        CmdArgument = CardRCA;
    }
    else            /* if MMC or unknown card type, use default RCA addr. */
    {
        CmdArgument = 0x00010000;
    }

    retryCount = 0x20;

    while (retryCount > 0)
    {
        /* Send CMD7_SELECT_CARD command before read and write */
        LPC_MCI->CLEAR |= (MCI_CMD_TIMEOUT | MCI_CMD_CRC_FAIL | MCI_CMD_RESP_END);

        respStatus = Lpc177x_8x_MciCmdResp(CMD7_SELECT_CARD, CmdArgument, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus)
        {
            retval = MCI_FUNC_FAILED;
        }
        else if (!(respValue[0] & _SHIFT(MCI_CARDSTATUS_READYFORDATA_P0S)))
        {
            retval = MCI_FUNC_NOT_READY;
        }
        else if (CARDSTATEOF(respValue[0]) != MCI_CARDSTATE_STBY)
        {
            /* Should be in STANDBY state now and ready */
            retval = MCI_FUNC_ERR_STATE;
        }
        else
        {
            return(MCI_FUNC_OK);
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_FAILED)
    {
        printf("%s() failed with MCI_FUNC_FAILED\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_NOT_READY)
    {
        printf("%s() failed with MCI_FUNC_NOT_READY\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_ERR_STATE)
    {
        printf("%s() failed with MCI_FUNC_ERR_STATE\n", __FUNCTION__ );
    }
#endif

    return(retval);
}


/************************************************************************//**
 * \brief       Get the status of the card. The return is from the card.
 *              By sending CMD13 (SEND_STATUS), the status of the card
 *              will be responded from card addressed
 *
 * \param[out]  cardStatus the status returned from the card
 *
 * \return      MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t Lpc177x_8x_MciGetCardStatus(int32_t* cardStatus)
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];
    uint32_t CmdArgument;

    int32_t retval = MCI_FUNC_FAILED;

    if ((MCI_CardType == MCI_SDSC_V1_CARD) ||
        (MCI_CardType == MCI_SDSC_V2_CARD) ||
        (MCI_CardType == MCI_SDHC_SDXC_CARD))
    {
        CmdArgument = CardRCA;
    }
    else            /* if MMC or unknown card type, use default RCA addr. */
    {
        CmdArgument = 0x00010000;
    }

    /*
     *  Note that, since it's called before the block write and read, this timeout
     *  is important based on the clock you set for the data communication.
     */
    retryCount = 0x300;

    while (retryCount > 0)
    {
        /* Clear previous status */
        LPC_MCI->CLEAR |= (MCI_CMD_TIMEOUT | MCI_CMD_CRC_FAIL | MCI_CMD_RESP_END);

        /* Send SELECT_CARD command before read and write */
        respStatus = Lpc177x_8x_MciCmdResp(CMD13_SEND_STATUS, CmdArgument, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus)
        {
            retval = MCI_FUNC_FAILED;
        }
/*
        // Don't check the response here already, that is not the responsibility of this routine.
        // The caller routine (in this case: GetStatus() is responsible for checking the value

        else if (!(respValue[0] & _SHIFT(MCI_CARDSTATUS_READYFORDATA_P0S)))
        {
            retval = MCI_FUNC_NOT_READY;
        }
*/
        else
        {
            /* The ready bit should be set, it should be in either TRAN or RCV state now */
            if (cardStatus != NULL)
            {
                *cardStatus = respValue[0];
            }

            return(MCI_FUNC_OK);
        }

        retryCount--;

        NutMicroDelay(200);
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_FAILED)
    {
        printf("%s() failed with MCI_FUNC_FAILED\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_NOT_READY)
    {
        printf("%s() failed with MCI_FUNC_NOT_READY\n", __FUNCTION__ );
    }
#endif

    return(retval);
}

/************************************************************************//**
 * \brief       Set the length for the blocks in the next action on data
 *              manipulation (as read, write, erase). This function is to
 *              send CMD16 (SET_BLOCK_LEN) to cards.
 *
 * \param[in]   blockLength the value for the length of block will be handled
 *
 * \return      MCI_FUNC_OK if all success
 *
 * \note        CMD16 command should be sent after the card is selected by CMD7
 *              (SELECT_CARD).
 ****************************************************************************/
int32_t Lpc177x_8x_MciSetBlockLen(uint32_t blockLength)
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    int32_t retval = MCI_FUNC_FAILED;

    retryCount = 0x20;
    while (retryCount > 0)
    {
        /* Send SET_BLOCK_LEN command before read and write */
        LPC_MCI->CLEAR |= (MCI_CMD_TIMEOUT | MCI_CMD_CRC_FAIL | MCI_CMD_RESP_END);

        respStatus = Lpc177x_8x_MciCmdResp(CMD16_SET_BLOCK_LEN, blockLength, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus)
        {
            retval = MCI_FUNC_FAILED;
        }
        else if (!(respValue[0] & _SHIFT(MCI_CARDSTATUS_READYFORDATA_P0S)))
        {
            retval = MCI_FUNC_NOT_READY;
        }
        else if ((CARDSTATEOF(respValue[0]) != MCI_CARDSTATE_TRAN))
        {
            retval = MCI_FUNC_ERR_STATE;
        }
        else
        {
            return(MCI_FUNC_OK);
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_FAILED)
    {
        printf("%s() failed with MCI_FUNC_FAILED\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_NOT_READY)
    {
        printf("%s() failed with MCI_FUNC_NOT_READY\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_ERR_STATE)
    {
        printf("%s() failed with MCI_FUNC_ERR_STATE\n", __FUNCTION__ );
    }
#endif

    return(retval);
}


/************************************************************************//**
 * \brief       Set bus-width (1 bit or 4 bit) to work with the card by command
 *              CMD6 (SET_ACMD_BUS_WIDTH).
 *
 * \param[in]   buswidth The value represented for bus-width
 *              - 0b00: 1-bit bus-width
 *              - 0b10: 4-bit bus-width
 *
 * \return      MCI_FUNC_OK if all success
 *
 * \note
 *              - If SD card is currently in used, it's possible to enable 4-bit
 *              bus-width instead of 1-bit to speed up.
 *              - This command can only be transferred during TRANS state.
 *              - Since, it's a ACMD, CMD55 (APP_CMD) needs to be sent out first
 ****************************************************************************/
int32_t Lpc177x_8x_MciAcmd_SendBusWidth( uint32_t buswidth )
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    int32_t retval = MCI_FUNC_FAILED;

    retryCount = 0x20;          /* reset retry counter */

    while (retryCount > 0)
    {
        if (Lpc177x_8x_MciCmd_SendACMD() == MCI_FUNC_OK)
        {
            respStatus = Lpc177x_8x_MciCmdResp(ACMD6_SET_BUS_WIDTH, buswidth, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

            if (respStatus)
            {
                retval = MCI_FUNC_FAILED;
            }
            else if (!(respValue[0] & _SHIFT(MCI_CARDSTATUS_READYFORDATA_P0S)))
            {
                retval = MCI_FUNC_NOT_READY;
            }
            else if ((CARDSTATEOF(respValue[0]) != MCI_CARDSTATE_TRAN))
            {
                retval = MCI_FUNC_ERR_STATE;
            }
            else
            {
                return(MCI_FUNC_OK);
            }
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_FAILED)
    {
        printf("%s() failed with MCI_FUNC_FAILED\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_NOT_READY)
    {
        printf("%s() failed with MCI_FUNC_NOT_READY\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_ERR_STATE)
    {
        printf("%s() failed with MCI_FUNC_ERR_STATE\n", __FUNCTION__ );
    }
#endif

    return(retval);
}


/************************************************************************//**
 * \brief       Get the state of  data transfer to see if it is ended or not
 *
 * \param       None
 *
 * \return      Transfer state (stored by Mci_Data_Xfer_End variable)
 ****************************************************************************/
uint32_t Lpc177x_8x_MciGetDataXferEndState(void)
{
    return(Mci_Data_Xfer_End);
}
/************************************************************************//**
 * \brief       Get the error state of  the lastest data transfer
 *
 * \param       None
 *
 * \return      Error state (stored by Mci_Data_Xfer_ERR variable)
 ****************************************************************************/
uint32_t Lpc177x_8x_MciGetXferErrState(void)
{
    return(Mci_Data_Xfer_ERR);
}

/************************************************************************//**
 * \brief       Stop the current transmission on the bus by sending command CMD12
 *              (STOP_TRANSMISSION). In this case, the card may be in a unknown
 *              state. So that it need a warm reset for normal operation.
 *
 * \param[in]   None
 *
 * \return      MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t Lpc177x_8x_MciCmd_StopTransmission( void )
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    int32_t retval = MCI_FUNC_FAILED;

    retryCount = 0x20;

    while (retryCount > 0)
    {
        LPC_MCI->CLEAR = 0x7FF;

        respStatus = Lpc177x_8x_MciCmdResp(CMD12_STOP_TRANSMISSION, 0x00000000, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus)
        {
            retval = MCI_FUNC_FAILED;
        }
        else if (!(respValue[0] & _SHIFT(MCI_CARDSTATUS_READYFORDATA_P0S)))
        {
            retval = MCI_FUNC_NOT_READY;
        }
        else
        {
            return(MCI_FUNC_OK);
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_FAILED)
    {
        printf("%s() failed with MCI_FUNC_FAILED\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_NOT_READY)
    {
        printf("%s() failed with MCI_FUNC_NOT_READY\n", __FUNCTION__ );
    }
#endif

    return(retval);
}


/************************************************************************//**
 * \brief       Write blocks to card by sending command CMD24 (WRITE_BLOCK) or
 *              command CMD25 (WRITE_MULTIPLE_BLOCK) followed by the blocks of
 *              data to be written.
 *
 * \param[in]   blockNum The block number to start writting
 *
 * \param[in]   numOfBlock Determine how many blocks will be written (from the
 *              starting block)
 *
 * \return      MCI_FUNC_OK if all success
 *
 * \note        These commands should be sent in TRANS state.
 ****************************************************************************/
int32_t Lpc177x_8x_MciCmd_WriteBlock(uint32_t blockNum, uint32_t numOfBlock)
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];

    uint32_t commandID;

    int32_t retval = MCI_FUNC_FAILED;

    if (numOfBlock > 1)
    {
        commandID = CMD25_WRITE_MULTIPLE_BLOCK;
    }
    else
    {
        commandID = CMD24_WRITE_BLOCK;
    }

    retryCount = 0x20;

    while (retryCount > 0)
    {
        LPC_MCI->CLEAR = 0x7FF;

        respStatus = Lpc177x_8x_MciCmdResp(commandID, blockNum, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus)
        {
            retval = MCI_FUNC_FAILED;
        }
        else if (!(XSHIFT_(respValue[0], MCI_CARDSTATUS_READYFORDATA_P0S) & 0x01))
        {
            retval = MCI_FUNC_NOT_READY;
        }
        else if ((CARDSTATEOF(respValue[0]) != MCI_CARDSTATE_TRAN))
        {
            retval = MCI_FUNC_ERR_STATE;
        }
        else
        {
            /* ready and in TRAN state */
            return(MCI_FUNC_OK);
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_FAILED)
    {
        printf("%s() failed with MCI_FUNC_FAILED\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_NOT_READY)
    {
        printf("%s() failed with MCI_FUNC_NOT_READY\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_ERR_STATE)
    {
        printf("%s() failed with MCI_FUNC_ERR_STATE\n", __FUNCTION__ );
    }
#endif

    return(retval);              /* Fatal error */
}




/************************************************************************//**
 * \brief       Read blocks to card by sending CMD17 (READ_SINGLE_BLOCK) or
 *              CMD18 (READ_MULTIPLE_BLOCK) commands followed by the blocks of
 *              data to be read.
 *
 * \param[in]   blockNum The block number to start reading
 *
 * \param[in]   numOfBlock Determine how many blocks will be read (from the
 *              starting block)
 *
 * \return      MCI_FUNC_OK if all success
 *
 * \note        These commands should be sent in TRANS state.
 ****************************************************************************/
int32_t Lpc177x_8x_MciCmd_ReadBlock(uint32_t blockNum, uint32_t numOfBlock)
{
    uint32_t retryCount;
    uint32_t respStatus;
    uint32_t respValue[4];
    uint32_t commandID;

    int32_t retval = MCI_FUNC_FAILED;

    if (numOfBlock > 1)
    {
        commandID = CMD18_READ_MULTIPLE_BLOCK;
    }
    else
    {
        commandID = CMD17_READ_SINGLE_BLOCK;
    }

    retryCount = 0x20;

    while (retryCount > 0)
    {
        LPC_MCI->CLEAR = 0x7FF;

        respStatus = Lpc177x_8x_MciCmdResp(commandID, blockNum, EXPECT_SHORT_RESP, (uint32_t *)&respValue[0], ALLOW_CMD_TIMER);

        if (respStatus)
        {
            retval = MCI_FUNC_FAILED;
        }
        else if (!(respValue[0] & _SHIFT(MCI_CARDSTATUS_READYFORDATA_P0S)))
        {
            retval = MCI_FUNC_NOT_READY;
        }
        else if ((CARDSTATEOF(respValue[0]) != MCI_CARDSTATE_TRAN))//((CARDSTATEOF(respValue[0]) != MCI_CARDSTATE_READY))
        {
            retval = MCI_FUNC_ERR_STATE;
        }
        else
        {
            /* ready and in TRAN state */
            return(MCI_FUNC_OK);
        }

        NutMicroDelay(2);

        retryCount--;
    }

#ifdef NUTDEBUG
    if(retval == MCI_FUNC_FAILED)
    {
        printf("%s() failed with MCI_FUNC_FAILED\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_NOT_READY)
    {
        printf("%s() failed with MCI_FUNC_NOT_READY\n", __FUNCTION__ );
    }
    if(retval == MCI_FUNC_ERR_STATE)
    {
        printf("%s() failed with MCI_FUNC_ERR_STATE\n", __FUNCTION__ );
    }
#endif

    return(retval);
}


/************************************************************************//**
 * \brief       Write data at a specific address to starting block with number
 *              of blocks will be written from first block
 * \details
 *              - At preparation
 *                  - Set MCI data control register, data length and data timeout
 *                  - Send CMD24 (WRITE_BLOCK) or CMD25 (WRITE_MULTIPLE_BLOCK)
 *                  commands to card
 *                  - Enable interupt for MCI component
 *              - At completion
 *                  - TX_ACTIVE interrupt is occured
 *                  - Write data to FIFO register continuously until the data block
 *                  length is reached
 *
 * \param[in]   *memblock The pointer to location stored required data to be
 *              written to card
 *
 * \param[in]   blockNum The block number to start writting
 *
 * \param[in]   numOfBlock Determine how many blocks will be written (from the
 *              starting block)
 *
 * \return      MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t Lpc177x_8x_MciWriteBlock(uint8_t* memblock, uint32_t blockNum, uint32_t numOfBlock)
{
    uint32_t DataCtrl = 0;

    dataSrcBlock = memblock;

    LPC_MCI->CLEAR = 0x7FF;

    LPC_MCI->DATACTRL = 0;

    NutMicroDelay(1);

    /*
     *  Below status check is redundant, but ensure card is in TRANS state
     *  before writing and reading to from the card.
     */
    if (Lpc177x_8x_MciCheckStatus() != MCI_FUNC_OK)
    {
#ifdef NUTDEBUG
        printf("%s() failed I [blocknum %lu]\n", __FUNCTION__ , blockNum);
#endif
        Lpc177x_8x_MciCmd_StopTransmission();

        return(MCI_FUNC_FAILED);
    }

    LPC_MCI->DATATMR = DATA_TIMER_VALUE_W;

    LPC_MCI->DATALEN = BLOCK_LENGTH*numOfBlock;
    dataLenTx        = BLOCK_LENGTH*numOfBlock;

    Mci_Data_Xfer_End = 1;
    Mci_Data_Xfer_ERR = 0;
    fifo_plane = 0;

    txBlockCnt = 0;

    Lpc177x_8x_MciTXEnable();

    if (Lpc177x_8x_MciCmd_WriteBlock(blockNum, numOfBlock) != MCI_FUNC_OK)
    {
#ifdef NUTDEBUG
        printf("%s() failed II [blocknum %lu]\n", __FUNCTION__ , blockNum);
#endif
        return( MCI_FUNC_FAILED );
    }

    /* set Write Enable, data direction and data length code */
    DataCtrl |= MCI_DATACTRL_ENABLE  | MCI_DATACTRL_DIR_TO_CARD  | MCI_DTATCTRL_BLOCKSIZE(DATA_BLOCK_LEN);

    LPC_MCI->DATACTRL = DataCtrl;       // Go!

    NutMicroDelay(10);

    return(MCI_FUNC_OK);
}


/************************************************************************//**
 * \brief       Read data at a specific address to starting block with number
 *              of blocks will be read from first block
 *
 * \details
 *              - At preparation
 *                  - Set MCI data control register, data length and data timeout
 *                  - Send CMD17 (READ_SINGLE_BLOCK) or CMD18 (READ_MULTIPLE_BLOCK)
 *                  commands to card
 *                  - Enable interupt for MCI component
 *              - At completion
 *                  - RX_ACTIVE interrupt is occured
 *                  - Read data from FIFO register continuously until the data block
 *                  length is reached to retrieve needed data
 *
 * \param[in]   *destBlock The pointer to location will captured data read
 *              from card
 *
 * \param[in]   blockNum The block number to start reading
 *
 * \param[in]   numOfBlock Determine how many blocks will be read (from the
 *              starting block)
 *
 * \return      MCI_FUNC_OK if all success
 ****************************************************************************/
int32_t Lpc177x_8x_MciReadBlock(uint8_t* destBlock, uint32_t blockNum, uint32_t numOfBlock)
{
    uint32_t DataCtrl = 0;

    dataDestBlock = destBlock;

    LPC_MCI->CLEAR = 0x7FF;

    LPC_MCI->DATACTRL = 0;

    NutMicroDelay(1);

    /*
     *  Below status check is redundant, but ensure card is in TRANS state
     *  before writing and reading to from the card.
     */
    if (Lpc177x_8x_MciCheckStatus() != MCI_FUNC_OK)
    {
#ifdef NUTDEBUG
        printf("%s() failed I [blocknum %lu]\n", __FUNCTION__ , blockNum);
#endif
        Lpc177x_8x_MciCmd_StopTransmission();
        return(MCI_FUNC_FAILED);
    }


    LPC_MCI->DATATMR = DATA_TIMER_VALUE_R;

    LPC_MCI->DATALEN = BLOCK_LENGTH*numOfBlock;
    dataLenRx        = BLOCK_LENGTH*numOfBlock;

    Mci_Data_Xfer_End = 1;
    Mci_Data_Xfer_ERR = 0;
    fifo_plane = 0;

    rxBlockCnt = 0;

    Lpc177x_8x_MciRXEnable();

    if (Lpc177x_8x_MciCmd_ReadBlock(blockNum, numOfBlock) != MCI_FUNC_OK)
    {
#ifdef NUTDEBUG
        printf("%s() failed II [blocknum %lu]\n", __FUNCTION__ , blockNum);
#endif
        return(MCI_FUNC_FAILED);
    }

    // Read, enable, block transfer, and data length
    DataCtrl |= MCI_DATACTRL_ENABLE | MCI_DATACTRL_DIR_FROM_CARD | MCI_DTATCTRL_BLOCKSIZE(DATA_BLOCK_LEN);

    LPC_MCI->DATACTRL = DataCtrl;       // Go!

    NutMicroDelay(10);

    return(MCI_FUNC_OK);
}


/************************************************************************//**
 * \brief       Turn off the MCI power by disabling the Power Register for MCI
 *
 * \param       None
 *
 * \return      None
 ****************************************************************************/
void Lpc177x_8x_MciPowerOff(void)
{
    LPC_MCI->POWER = 0;

    NutMicroDelay(10);

    return;
}

/************************************************************************//**
 * \brief       Get the priority level of the SD interrupt
 *              please note that only 1 level is present for _ALL_ different
 *              types of SD-interrupts
 *
 * \param       None
 *
 * \return      the interrupt level [0..31]
 ****************************************************************************/
int Lpc177x_8x_MciIrqGetPriority()
{
    return      NutIrqGetPriority(&sig_MCI);
}


/************************************************************************//**
 * \brief       Set the priority level of the SD interrupt
 *              please note that only 1 level is present for _ALL_ different
 *              types of SD-interrupts
 *
 * \param       the interrupt level [0..31]
 *
 * \return      None
 ****************************************************************************/
void Lpc177x_8x_MciIrqSetPriority(int priority)
{
    if ((priority >= 0) && (priority < 32)) {
        NutIrqSetPriority(&sig_MCI, priority);
    }
}

