/*!
 * Copyright (C) 2001-2010 by egnite Software GmbH
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


/*
* $Log: macb.c,v $
*
*/

#include <cfg/os.h>
#include <cfg/dev.h>
#include <cfg/arch/gpio.h>
#include <cfg/phycfg.h>

#include <arch/avr32.h>
#include <arch/avr32/gpio.h>

#include <sys/atom.h>
#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <netinet/if_ether.h>
#include <net/ether.h>
#include <net/if_var.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/phy.h>

#include <avr32/io.h>

#ifdef NUTDEBUG
#include <stdio.h>
#endif

#ifndef NUT_THREAD_NICRXSTACK
#define NUT_THREAD_NICRXSTACK   768
#endif

#ifndef EMAC_RX_BUFFERS
#define EMAC_RX_BUFFERS         32
#endif
#define EMAC_RX_BUFSIZ          128

#define EMAC_TX_BUFFERS         2
#ifndef EMAC_TX_BUFSIZ
#define EMAC_TX_BUFSIZ          1536
#endif

#ifndef EMAC_LINK_LOOPS
#define EMAC_LINK_LOOPS         1000000
#endif

/*!
* \brief PHY address.
*
* Any other than 0 seems to create problems with Atmel's evaluation kits.
*/
#ifndef NIC_PHY_ADDR_DEPRECATED
#define NIC_PHY_ADDR_DEPRECATED            1
#endif

/*!
* \brief Network interface controller information structure.
*/
struct _EMACINFO {
#ifdef NUT_PERFMON
    uint32_t ni_rx_packets;     /*!< Number of packets received. */
    uint32_t ni_tx_packets;     /*!< Number of packets sent. */
    uint32_t ni_overruns;       /*!< Number of packet overruns. */
    uint32_t ni_rx_frame_errors;        /*!< Number of frame errors. */
    uint32_t ni_rx_crc_errors;  /*!< Number of CRC errors. */
    uint32_t ni_rx_missed_errors;       /*!< Number of missed packets. */
#endif
    HANDLE volatile ni_rx_rdy;  /*!< Receiver event queue. */
    HANDLE volatile ni_tx_rdy;  /*!< Transmitter event queue. */
    HANDLE ni_mutex;            /*!< Access mutex semaphore. */
    volatile int ni_tx_queued;  /*!< Number of packets in transmission queue. */
    volatile int ni_tx_quelen;  /*!< Number of bytes in transmission queue not sent. */
    volatile int ni_insane;     /*!< Set by error detection. */
    int ni_iomode;              /*!< 8 or 16 bit access. 32 bit is not supported. */
};

/*!
* \brief Network interface controller information type.
*/
typedef struct _EMACINFO EMACINFO;

/*
* TODO: Buffers and their descriptors should be part of the EMACINFO
* structure. Actually there will be no dual Ethernet chip (sure?),
* but just to keep the code clean.
*/
/*! Receive Transfer descriptor structure.
*/
typedef struct _RxTdDescriptor {
    uint32_t addr;
    uint32_t status;
} RxTdDescriptor;
//! @}

/*! Transmit Transfer descriptor structure.
*/
//! @{
typedef struct _TxTdDescriptor {
    uint32_t addr;
    uint32_t status;
} TxTdDescriptor;
//! @}

static volatile TxTdDescriptor txBufTab[EMAC_TX_BUFFERS] NUT_ALIGNED_TYPE(8);
static volatile uint8_t txBuf[EMAC_TX_BUFFERS * EMAC_TX_BUFSIZ] NUT_ALIGNED_TYPE(4);
static unsigned int txBufIdx = 0;

static volatile RxTdDescriptor rxBufTab[EMAC_RX_BUFFERS] NUT_ALIGNED_TYPE(8);
static volatile uint8_t rxBuf[EMAC_RX_BUFFERS * EMAC_RX_BUFSIZ] NUT_ALIGNED_TYPE(4);
static unsigned int rxBufIdx = 0;

#define RXBUF_OWNERSHIP     0x00000001
#define RXBUF_WRAP          0x00000002
#define RXBUF_ADDRMASK      0xFFFFFFFC

#define RXS_BROADCAST_ADDR  0x80000000  /*!< \brief Broadcast address detected. */
#define RXS_MULTICAST_HASH  0x40000000  /*!< \brief Multicast hash match. */
#define RXS_UNICAST_HASH    0x20000000  /*!< \brief Unicast hash match. */
#define RXS_EXTERNAL_ADDR   0x10000000  /*!< \brief External address match. */
#define RXS_SA1_ADDR        0x04000000  /*!< \brief Specific address register 1 match. */
#define RXS_SA2_ADDR        0x02000000  /*!< \brief Specific address register 2 match. */
#define RXS_SA3_ADDR        0x01000000  /*!< \brief Specific address register 3 match. */
#define RXS_SA4_ADDR        0x00800000  /*!< \brief Specific address register 4 match. */
#define RXS_TYPE_ID         0x00400000  /*!< \brief Type ID match. */
#define RXS_VLAN_TAG        0x00200000  /*!< \brief VLAN tag detected. */
#define RXS_PRIORITY_TAG    0x00100000  /*!< \brief Priority tag detected. */
#define RXS_VLAN_PRIORITY   0x000E0000  /*!< \brief VLAN priority. */
#define RXS_CFI_IND         0x00010000  /*!< \brief Concatenation format indicator. */
#define RXS_EOF             0x00008000  /*!< \brief End of frame. */
#define RXS_SOF             0x00004000  /*!< \brief Start of frame. */
#define RXS_RBF_OFFSET      0x00003000  /*!< \brief Receive buffer offset mask. */
#define RXS_LENGTH_FRAME    0x000007FF  /*!< \brief Length of frame including FCS. */

#define TXS_USED            0x80000000  /*!< \brief Used buffer. */
#define TXS_WRAP            0x40000000  /*!< \brief Last descriptor. */
#define TXS_ERROR           0x20000000  /*!< \brief Retry limit exceeded. */
#define TXS_UNDERRUN        0x10000000  /*!< \brief Transmit underrun. */
#define TXS_NO_BUFFER       0x08000000  /*!< \brief Buffer exhausted. */
#define TXS_NO_CRC          0x00010000  /*!< \brief CRC not appended. */
#define TXS_LAST_BUFF       0x00008000  /*!< \brief Last buffer of frame. */


/*!
* \addtogroup xgNutArchAVR32Macb
*/
/*@{*/

/*!
* \brief Read contents of PHY register.
*
* \param reg PHY register number.
*
* \return Contents of the specified register.
*/
static uint16_t phy_inw(uint8_t reg)
{
    uint16_t value;

    // initiate transaction: enable management port
    AVR32_MACB.ncr |= AVR32_MACB_NCR_MPE_MASK;

    // Write the PHY configuration frame to the MAN register
    AVR32_MACB.man = (AVR32_MACB_SOF_MASK & (0x01 << AVR32_MACB_SOF_OFFSET)) // SOF
        | (2 << AVR32_MACB_CODE_OFFSET) // Code
        | (2 << AVR32_MACB_RW_OFFSET)   // Read operation
        | ((NIC_PHY_ADDR_DEPRECATED & 0x1f) << AVR32_MACB_PHYA_OFFSET)     // Phy Add
        | (reg << AVR32_MACB_REGA_OFFSET);      // Reg Add

    // wait for PHY to be ready
    while (!(AVR32_MACB.nsr & AVR32_MACB_NSR_IDLE_MASK));

    // read the register value in maintenance register
    value = AVR32_MACB.man & 0x0000ffff;

    // disable management port
    AVR32_MACB.ncr &= ~AVR32_MACB_NCR_MPE_MASK;

    return value;
}

/*!
* \brief Write value to PHY register.
*
* \param reg PHY register number.
* \param val Value to write.
*/
static void phy_outw(uint8_t reg, uint16_t val)
{
    // initiate transaction : enable management port
    AVR32_MACB.ncr |= AVR32_MACB_NCR_MPE_MASK;

    // Write the PHY configuration frame to the MAN register
    AVR32_MACB.man = ((AVR32_MACB_SOF_MASK & (0x01 << AVR32_MACB_SOF_OFFSET))        // SOF
                 | (2 << AVR32_MACB_CODE_OFFSET)        // Code
                 | (1 << AVR32_MACB_RW_OFFSET)  // Write operation
                 | ((NIC_PHY_ADDR_DEPRECATED & 0x1f) << AVR32_MACB_PHYA_OFFSET)    // Phy Add
                 | (reg << AVR32_MACB_REGA_OFFSET))     // Reg Add
        | (val & 0xffff);       // Data

    // wait for PHY to be ready
    while (!(AVR32_MACB.nsr & AVR32_MACB_NSR_IDLE_MASK));

    // disable management port
    AVR32_MACB.ncr &= ~AVR32_MACB_NCR_MPE_MASK;
}

/*!
* \brief Reset the Ethernet controller.
*
* \return 0 on success, -1 otherwise.
*/
static int EmacReset(NUTDEVICE * dev)
{
    int rc = 0;
    uint32_t phyval;
	int link_wait;
	volatile unsigned long reg_ncfgr;
    volatile avr32_macb_t *macb = (avr32_macb_t *) dev->dev_base;
    const uint32_t hclk_hz = NutArchClockGet(NUT_HWCLK_PERIPHERAL_B);

    /* Disable TX and RX */
    macb->ncr = 0;

    /* Clear status registers */
    macb->NCR.clrstat = 1;

    /* Clear all status flags */
    macb->tsr = ~0UL;
    macb->rsr = ~0UL;

    /* Disable all interrupts */
    NutEnterCritical();
    macb->idr = ~0UL;
    macb->isr;
    NutExitCritical();

#if defined(PHY_MODE_RMII_DEPRECATED)
    macb->usrio &= ~AVR32_MACB_RMII_MASK;
#else
    macb->usrio |= AVR32_MACB_RMII_MASK;
#endif

    /* Set MII management clock divider */
    if (hclk_hz <= 20000000)
        macb->ncfgr |= (AVR32_MACB_NCFGR_CLK_DIV8 << AVR32_MACB_NCFGR_CLK_OFFSET);
    else if (hclk_hz <= 40000000)
        macb->ncfgr |= (AVR32_MACB_NCFGR_CLK_DIV16 << AVR32_MACB_NCFGR_CLK_OFFSET);
    else if (hclk_hz <= 80000000)
        macb->ncfgr |= (AVR32_MACB_NCFGR_CLK_DIV32 << AVR32_MACB_NCFGR_CLK_OFFSET);
    else
        macb->ncfgr |= (AVR32_MACB_NCFGR_CLK_DIV64 << AVR32_MACB_NCFGR_CLK_OFFSET);

    /* Wait for PHY ready. */
    NutDelay(255);

    /* Register PHY */
    rc = NutRegisterPhy( NIC_PHY_ADDR_DEPRECATED, phy_outw, phy_inw);
	
#ifndef PHY_MODE_RMII_DEPRECATED
	/* Clear MII isolate. */
	phyval = 0;
	NutPhyCtl(PHY_CTL_ISOLATE, &phyval);
#endif

	/* Restart auto negotiation */
	phyval = 1;
	NutPhyCtl(PHY_CTL_AUTONEG_RE, &phyval);

	/* Wait for auto negotiation completed and link established. */
	for (link_wait = EMAC_LINK_LOOPS;; link_wait--) {
		phyval = 0;
		NutPhyCtl(PHY_GET_STATUS, &phyval);

		if((phyval & PHY_STATUS_HAS_LINK) && (phyval & PHY_STATUS_AUTONEG_OK)) {
			/* Check link state and configure EMAC accordingly */
			reg_ncfgr = macb->ncfgr;
			if (phyval & PHY_STATUS_FULLDUPLEX) {
				reg_ncfgr |= AVR32_MACB_FD_MASK;
            } else {
				reg_ncfgr &= ~AVR32_MACB_FD_MASK;
			}

			if (phyval & PHY_STATUS_100M) {
				reg_ncfgr |= AVR32_MACB_SPD_MASK;
				} else {
				reg_ncfgr &= ~AVR32_MACB_SPD_MASK;
			}
			macb->ncfgr = reg_ncfgr;

			break;
		}
		if (link_wait == 0) {
			/* Return error on link timeout. */
			return -1;
		}
		NutSleep(10);
	}
  
	return 0;
}

/*
* NIC interrupt entry.
*/
static void EmacInterrupt(void *arg)
{
    volatile unsigned int isr;
    unsigned int event;
    NUTDEVICE *dev = (NUTDEVICE *) arg;
    volatile avr32_macb_t *macb = (avr32_macb_t *) dev->dev_base;
    EMACINFO *ni = dev->dev_dcb;

    /* Read interrupt status and disable interrupts. */
    isr = macb->isr;
    event = macb->rsr;

    /* Receiver interrupt. */
    if ((isr & (AVR32_MACB_IMR_RCOMP_MASK | AVR32_MACB_IMR_ROVR_MASK | AVR32_MACB_IMR_RXUBR_MASK)) || (event & (AVR32_MACB_REC_MASK | AVR32_MACB_BNA_MASK))) {
        macb->rsr = AVR32_MACB_REC_MASK | AVR32_MACB_BNA_MASK;        // Clear
        macb->rsr;              // Read to force the previous write
        macb->idr = AVR32_MACB_IDR_RCOMP_MASK | AVR32_MACB_IDR_ROVR_MASK | AVR32_MACB_IDR_RXUBR_MASK;
        NutEventPostFromIrq(&ni->ni_rx_rdy);
    }

    /* Transmitter interrupt. */
    if (isr & AVR32_MACB_IMR_TCOMP_MASK) {
        macb->tsr = AVR32_MACB_TSR_COMP_MASK;   // Clear
        macb->tsr;              // Read to force the previous write

        NutEventPostFromIrq(&ni->ni_tx_rdy);
    }
}

/*!
* \brief Fetch the next packet out of the receive buffers.
*
* \return 0 on success, -1 otherwise.
*/
static int EmacGetPacket(EMACINFO * ni, NETBUF ** nbp)
{
    int rc = -1;
    unsigned int fbc = 0;
    unsigned int i;
    *nbp = NULL;

    /*
     * Search the next frame start. Release any fragment.
     */
    while ((rxBufTab[rxBufIdx].addr & RXBUF_OWNERSHIP) != 0 && (rxBufTab[rxBufIdx].status & RXS_SOF) == 0) {
        rxBufTab[rxBufIdx].addr &= ~(RXBUF_OWNERSHIP);
        rxBufIdx++;
        if (rxBufIdx >= EMAC_RX_BUFFERS) {
            rxBufIdx = 0;
        }
    }

    /*
     * Determine the size of the next frame.
     */
    i = rxBufIdx;
    while (rxBufTab[i].addr & RXBUF_OWNERSHIP) {
        if (i != rxBufIdx && (rxBufTab[i].status & RXS_SOF) != 0) {
            do {
                rxBufTab[rxBufIdx].addr &= ~(RXBUF_OWNERSHIP);
                rxBufIdx++;
                if (rxBufIdx >= EMAC_RX_BUFFERS) {
                    rxBufIdx = 0;
                }
            } while ((rxBufTab[rxBufIdx].addr & RXBUF_OWNERSHIP) != 0 && (rxBufTab[rxBufIdx].status & RXS_SOF) == 0);
            break;
        }
        if ((fbc = rxBufTab[i].status & RXS_LENGTH_FRAME) != 0) {
            break;
        }
        i++;
        if (i >= EMAC_RX_BUFFERS) {
            i = 0;
        }
    }

    if (fbc) {
        *nbp = NutNetBufAlloc(0, NBAF_DATALINK, (uint16_t) fbc);
        if (*nbp != NULL) {
            uint8_t *bp = (uint8_t *) (*nbp)->nb_dl.vp;
            unsigned int len;

            while (fbc) {
                if (fbc > EMAC_RX_BUFSIZ) {
                    len = EMAC_RX_BUFSIZ;
                } else {
                    len = fbc;
                }
                memcpy(bp, (void *) (rxBufTab[rxBufIdx].addr & RXBUF_ADDRMASK), len);
                rxBufTab[rxBufIdx].addr &= ~RXBUF_OWNERSHIP;
                rxBufIdx++;
                if (rxBufIdx >= EMAC_RX_BUFFERS) {
                    rxBufIdx = 0;
                }
                fbc -= len;
                bp += len;
            }
            rc = 0;
        }
    }
    return rc;
}

/*!
* \brief Load a packet into the nic's transmit ring buffer.
*
* \todo This routine simply does not work. Any idea?
*
* \param nb Network buffer structure containing the packet to be sent.
*           The structure must have been allocated by a previous
*           call NutNetBufAlloc(). This routine will automatically
*           release the buffer in case of an error.
*
* \return 0 on success, -1 in case of any errors. Errors
*         will automatically release the network buffer
*         structure.
*/
static int EmacPutPacket(int bufnum, NUTDEVICE * dev, NETBUF * nb)
{
    volatile avr32_macb_t *macb = (avr32_macb_t *) dev->dev_base;
    int rc = -1;
    unsigned int sz;
    uint8_t *buf;
    EMACINFO *ni = dev->dev_dcb;


    /*
     * Calculate the number of bytes to be send. Do not send packets
     * larger than the Ethernet maximum transfer unit. The MTU
     * consist of 1500 data bytes plus the 14 byte Ethernet header
     * plus 4 bytes CRC. We check the data bytes only.
     */
    if ((sz = nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz) > ETHERMTU) {
        return -1;
    }
    sz += nb->nb_dl.sz;
    if (sz & 1) {
        sz++;
    }

    /* Disable EMAC interrupts. */
    NutIrqDisable(&sig_MACB);

    /* TODO: Check for link. */
    if (ni->ni_insane == 0) {
        buf = (uint8_t *) txBufTab[bufnum].addr;
        memcpy(buf, nb->nb_dl.vp, nb->nb_dl.sz);
        buf += nb->nb_dl.sz;
        memcpy(buf, nb->nb_nw.vp, nb->nb_nw.sz);
        buf += nb->nb_nw.sz;
        memcpy(buf, nb->nb_tp.vp, nb->nb_tp.sz);
        buf += nb->nb_tp.sz;
        memcpy(buf, nb->nb_ap.vp, nb->nb_ap.sz);
        sz |= TXS_LAST_BUFF;
        if (bufnum) {
            sz |= TXS_WRAP;
        }
        txBufTab[bufnum].status = sz;
        macb->ncr |= AVR32_MACB_TSTART_MASK;
        rc = 0;
#ifdef NUT_PERFMON
        ni->ni_tx_packets++;
#endif
    }

    /* Enable EMAC interrupts. */
    NutIrqEnable(&sig_MACB);

    return rc;
}


/*!
* \brief Fire up the network interface.
*
* NIC interrupts must be disabled when calling this function.
*
* \param mac Six byte unique MAC address.
*/
static int EmacStart(volatile avr32_macb_t * macb, const uint8_t * mac)
{
    unsigned int i;

    /* Set local MAC address. */
    // Must be written SA1B then SA1T.
    macb->sa1b = (( unsigned long ) mac[3] << 24) | ( ( unsigned long ) mac[2] << 16) | ( ( unsigned long ) mac[1] << 8) | mac[0];
    macb->sa1t = (( unsigned long ) mac[5] << 8) | mac[4];

    /* Initialize receive buffer descriptors. */
    for (i = 0; i < EMAC_RX_BUFFERS - 1; i++) {
        rxBufTab[i].addr = (unsigned int) (&rxBuf[i * EMAC_RX_BUFSIZ]) & RXBUF_ADDRMASK;
    }
    rxBufTab[i].addr = ((unsigned int) (&rxBuf[i * EMAC_RX_BUFSIZ]) & RXBUF_ADDRMASK) | RXBUF_WRAP;
    macb->rbqp = (unsigned long) rxBufTab;

    /* Initialize transmit buffer descriptors. */
    for (i = 0; i < EMAC_TX_BUFFERS - 1; i++) {
        txBufTab[i].addr = (unsigned int) (&txBuf[i * EMAC_RX_BUFSIZ]);
        txBufTab[i].status = TXS_USED;
    }
    txBufTab[i].addr = (unsigned int) (&txBuf[i * EMAC_RX_BUFSIZ]);
    txBufTab[i].status = TXS_USED | TXS_WRAP;
    macb->tbqp = (unsigned long) txBufTab;

    /* Clear receiver status. */
    macb->rsr = AVR32_MACB_RSR_BNA_MASK | AVR32_MACB_RSR_OVR_MASK | AVR32_MACB_RSR_REC_MASK;
    macb->rsr;

    /* Discard FCS. */
    macb->ncfgr |= AVR32_MACB_NCFGR_DRFCS_MASK;

    /* Enable receiver, transmitter and statistics. */
    macb->ncr |= AVR32_MACB_NCR_RE_MASK | AVR32_MACB_NCR_TE_MASK;

    return 0;
}

/*! \fn EmacRxThread(void *arg)
* \brief NIC receiver thread.
*
*/
THREAD(EmacRxThread, arg)
{
    NUTDEVICE *dev = (NUTDEVICE *) arg;
    IFNET *ifn;
    EMACINFO *ni;
    NETBUF *nb;
    volatile avr32_macb_t *macb = (avr32_macb_t *) dev->dev_base;

    ifn = (IFNET *) dev->dev_icb;
    ni = (EMACINFO *) dev->dev_dcb;

    /*
     * This is a temporary hack. Due to a change in initialization,
     * we may not have got a MAC address yet. Wait until a valid one
     * has been set.
     */
    while (!ETHER_IS_UNICAST(ifn->if_mac)) {
        NutSleep(10);
    }

    /*
     * Do not continue unless we managed to start the NIC. We are
     * trapped here if the Ethernet link cannot be established.
     * This happens, for example, if no Ethernet cable is plugged
     * in.
     */
    while (EmacStart(macb, ifn->if_mac)) {
        EmacReset(dev);
        NutSleep(1000);
    }

    /* Initialize the access mutex. */
    NutEventPost(&ni->ni_mutex);

    /* Run at high priority. */
    NutThreadSetPriority(9);

    /* Enable receive and transmit interrupts. */
    macb->ier = AVR32_MACB_IER_ROVR_MASK | AVR32_MACB_IER_TCOMP_MASK | AVR32_MACB_IER_TUND_MASK |
        AVR32_MACB_IER_RXUBR_MASK | AVR32_MACB_IER_RCOMP_MASK;
    NutIrqEnable(&sig_MACB);

    for (;;) {
        /*
         * Wait for the arrival of new packets or poll the receiver every
         * 500 milliseconds. This short timeout helps a bit to deal with
         * the SAM9260 Ethernet problem.
         *
         * Sometimes an interrupt status change doesn't trigger an interrupt.
         * We need to read the status register, so that the flags get cleared
         * and the next change triggers an interrupt again.
         */
        if (NutEventWait(&ni->ni_rx_rdy, 500)) {
            macb->isr;
        }

        /*
         * Fetch all packets from the NIC's internal buffer and pass
         * them to the registered handler.
         */
        while (EmacGetPacket(ni, &nb) == 0) {
            /* Discard short packets. */
            if (nb->nb_dl.sz < 60) {
                NutNetBufFree(nb);
            } else {
                (*ifn->if_recv) (dev, nb);
            }
        }
        macb->ier = AVR32_MACB_IER_ROVR_MASK | AVR32_MACB_IER_RXUBR_MASK | AVR32_MACB_IER_RCOMP_MASK;

        /* We got a weird chip, try to restart it. */
        while (ni->ni_insane) {
            EmacReset(dev);
            if (EmacStart(macb, ifn->if_mac) == 0) {
                ni->ni_insane = 0;
                ni->ni_tx_queued = 0;
                ni->ni_tx_quelen = 0;
                NutIrqEnable(&sig_MACB);
            } else {
                NutSleep(1000);
            }
        }
    }
}

/*!
* \brief Send Ethernet packet.
*
* \param dev Identifies the device to use.
* \param nb  Network buffer structure containing the packet to be sent.
*            The structure must have been allocated by a previous
*            call NutNetBufAlloc().
*
* \return 0 on success, -1 in case of any errors.
*/
int EmacOutput(NUTDEVICE * dev, NETBUF * nb)
{
    volatile avr32_macb_t *macb = (avr32_macb_t *) dev->dev_base;
    static uint32_t mx_wait = 5000;
    int rc = -1;
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;

    /*
     * After initialization we are waiting for a long time to give
     * the PHY a chance to establish an Ethernet link.
     */
    while (rc) {
        if (ni->ni_insane) {
            break;
        }
		
        if (NutEventWait(&ni->ni_mutex, mx_wait))
            break;

        /* Check for packet queue space. */
        if ((txBufTab[txBufIdx].status & TXS_USED) == 0) {
            if (NutEventWait(&ni->ni_tx_rdy, 500)) {
                /*
                 * We may have a timeout here because the last status change
                 * didn't trigger an interrupt. Reading the status register
                 * will clear the current status and the next change triggers
                 * an interrupt again, hopefully.
                 */
                macb->isr;
                if ((txBufTab[txBufIdx].status & TXS_USED) == 0) {
                    /* No queue space. Release the lock and give up. */
                    txBufTab[txBufIdx].status |= TXS_USED;
                    txBufIdx++;
                    txBufIdx &= 1;
                    NutEventPost(&ni->ni_mutex);
                    break;
                }
            }
        } else {
            if (macb->tsr & AVR32_MACB_TSR_UND_MASK) {
                txBufIdx = 0;
                macb->tsr = AVR32_MACB_TSR_UND_MASK;
            }
            if (macb->tsr & AVR32_MACB_TSR_COMP_MASK) {
                macb->tsr = AVR32_MACB_TSR_COMP_MASK;
            }

            if ((rc = EmacPutPacket(txBufIdx, dev, nb)) == 0) {
                txBufIdx++;
                txBufIdx &= 1;
            }
        }
        NutEventPost(&ni->ni_mutex);
    }

    /*
     * Probably no Ethernet link. Significantly reduce the waiting
     * time, so following transmission will soon return an error.
     */
    if (rc) {
        mx_wait = 500;
    } else {
        /* Ethernet works. Set a long waiting time in case we
           temporarily lose the link next time. */
        mx_wait = 5000;
    }
    return rc;
}

/*!
* \brief Initialize Ethernet hardware.
*
* Applications should do not directly call this function. It is
* automatically executed during during device registration by
* NutRegisterDevice().
*
* \param dev Identifies the device to initialize.
*/
int EmacInit(NUTDEVICE * dev)
{
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;

    /* Reserve Pins with the GPIO Controller */
#if !defined(PHY_MODE_RMII_DEPRECATED)
    gpio_enable_module_pin(AVR32_MACB_CRS_0_PIN, AVR32_MACB_CRS_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_COL_0_PIN, AVR32_MACB_COL_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_RX_CLK_0_PIN, AVR32_MACB_RX_CLK_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_TX_ER_0_PIN, AVR32_MACB_TX_ER_0_FUNCTION);
#endif
    gpio_enable_module_pin(AVR32_MACB_MDC_0_PIN, AVR32_MACB_MDC_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_MDIO_0_PIN, AVR32_MACB_MDIO_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_RXD_0_PIN, AVR32_MACB_RXD_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_TXD_0_PIN, AVR32_MACB_TXD_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_RXD_1_PIN, AVR32_MACB_RXD_1_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_TXD_1_PIN, AVR32_MACB_TXD_1_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_TX_EN_0_PIN, AVR32_MACB_TX_EN_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_RX_ER_0_PIN, AVR32_MACB_RX_ER_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_RX_DV_0_PIN, AVR32_MACB_RX_DV_0_FUNCTION);
    gpio_enable_module_pin(AVR32_MACB_TX_CLK_0_PIN, AVR32_MACB_TX_CLK_0_FUNCTION);

    /* Reset the controller. */
    if (EmacReset(dev)) {
        return -1;
    }

    /* Clear EMACINFO structure. */
    memset(ni, 0, sizeof(EMACINFO));

    /* Register interrupt handler. */
    if (NutRegisterIrqHandler(&sig_MACB, EmacInterrupt, dev)) {
        return -1;
    }

    /* Start the receiver thread. */
    if (NutThreadCreate("emacrx", EmacRxThread, dev, 
        (NUT_THREAD_NICRXSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD) == NULL) {
        return -1;
    }
    return 0;
}

static EMACINFO dcb_eth0;

/*!
* \brief Network interface information structure.
*
* Used to call.
*/
static IFNET ifn_eth0 = {
    IFT_ETHER,                  /*!< \brief Interface type, if_type. */
    0,                          /*!< \brief Interface flags, if_flags. */
    {0, 0, 0, 0, 0, 0},         /*!< \brief Hardware net address, if_mac. */
    0,                          /*!< \brief IP address, if_local_ip. */
    0,                          /*!< \brief Remote IP address for point to point, if_remote_ip. */
    0,                          /*!< \brief IP network mask, if_mask. */
    ETHERMTU,                   /*!< \brief Maximum size of a transmission unit, if_mtu. */
    0,                          /*!< \brief Packet identifier, if_pkt_id. */
    0,                          /*!< \brief Linked list of arp entries, arpTable. */
    0,                          /*!< \brief Linked list of multicast address entries, if_mcast. */
    NutEtherInput,              /*!< \brief Routine to pass received data to, if_recv(). */
    EmacOutput,                 /*!< \brief Driver output routine, if_send(). */
    NutEtherOutput,             /*!< \brief Media output routine, if_output(). */
    NULL                        /*!< \brief Interface specific control function, if_ioctl(). */
#ifdef NUT_PERFMON
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
#endif
};

/*!
* \brief Device information structure.
*
* A pointer to this structure must be passed to NutRegisterDevice()
* to bind this Ethernet device driver to the Nut/OS kernel.
* An application may then call NutNetIfConfig() with the name \em eth0
* of this driver to initialize the network interface.
*
*/
NUTDEVICE devAvr32macb = {
    0,                          /*!< \brief Pointer to next device. */
    {'e', 't', 'h', '0', 0, 0, 0, 0, 0},        /*!< \brief Unique device name. */
    IFTYP_NET,                  /*!< \brief Type of device. */
    AVR32_MACB_ADDRESS,         /*!< \brief Base address. */
    0,                          /*!< \brief First interrupt number. */
    &ifn_eth0,                  /*!< \brief Interface control block. */
    &dcb_eth0,                  /*!< \brief Driver control block. */
    EmacInit,                   /*!< \brief Driver initialization routine. */
    0,                          /*!< \brief Driver specific control function. */
    0,                          /*!< \brief Read from device. */
    0,                          /*!< \brief Write to device. */
#ifdef __HARVARD_ARCH__
    0,                          /*!< \brief Write from program space data to device. */
#endif
    0,                          /*!< \brief Open a device or file. */
    0,                          /*!< \brief Close a device or file. */
    0,                          /*!< \brief Request file size. */
    0,                          /*!< \brief Select function, optional, not yet implemented */
};

/*@}*/
