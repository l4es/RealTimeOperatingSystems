/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 * \verbatim
 * $Id: $
 * \endverbatim
 */
#include <stdio.h>
#include <arch/cm3.h>
#include <cfg/arch.h>
#include <cfg/os.h>
#include <cfg/dev.h>
#include <cfg/arch/gpio.h>

#include <stdlib.h>
#include <string.h>

#include <sys/atom.h>
#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <sys/confnet.h>

#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#include <arch/cm3/nxp/lpc176x_clk.h>
#include <arch/cm3/nxp/lpc176x_gpio.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_gpio.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#include <arch/cm3/nxp/lpc177x_8x_gpio.h>
#else
#warning "Unknown LPC familiy"
#endif

#include <netinet/if_ether.h>
#include <net/ether.h>
#include <net/if_var.h>

#include <dev/gpio.h>
#include <dev/irqreg.h>
#include <dev/phy.h>
#include <arch/cm3/nxp/lpc17xx_emac.h>

/* WARNING: Variadic macros are C99 and may fail with C89 compilers. */
#ifdef NUTDEBUG
#include <stdio.h>
#include <arpa/inet.h>
#define EMPRINTF(args,...) printf(args,##__VA_ARGS__);
#else
#define EMPRINTF(args,...)
#endif

#define NUT_THREAD_NICLINKSTACK 512

#ifndef NUT_THREAD_NICRXSTACK
#define NUT_THREAD_NICRXSTACK   768
#endif

#ifndef EMAC_LINK_LOOPS
#define EMAC_LINK_LOOPS         1000
#endif

/*!
 * \brief PHY address.
 *
 * Any other than 0 seems to create problems with Atmel's evaluation kits.
 */
#ifndef NIC_PHY_ADDR_DEPRECATED
#define NIC_PHY_ADDR_DEPRECATED            0
#endif

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif

#define min(a,b) ((a>b)?b:a)

#if defined(EMAC_ALLOC_DMA_BUFFER_FROM_HEAP)
uintptr_t emac_dma_buffer_base = 0;
uint8_t *emac_dma_buffer = NULL;
#endif


/* MII Mgmt Configuration register - Clock divider setting */
const uint8_t emac_clkdiv[] = { 4, 6, 8, 10, 14, 20, 28, 36, 40, 44, 48, 52, 56, 60, 64 };

typedef struct _BufDescriptor {
    unsigned int addr;
    unsigned int stat;
} BufDescriptor;

/*!
 * \brief Network interface controller information structure.
 */
struct _EMACINFO {
#ifdef NUT_PERFMON
#warning not yet implemented
    uint32_t ni_rx_packets;       /*!< Number of packets received. */
    uint32_t ni_tx_packets;       /*!< Number of packets sent. */
    uint32_t ni_overruns;         /*!< Number of packet overruns. */
    uint32_t ni_rx_frame_errors;  /*!< Number of frame errors. */
    uint32_t ni_rx_crc_errors;    /*!< Number of CRC errors. */
    uint32_t ni_rx_missed_errors; /*!< Number of missed packets. */
#endif
    HANDLE volatile ni_rx_rdy;    /*!< Receiver event queue. */
    HANDLE volatile ni_tx_rdy;    /*!< Transmitter event queue. */
    HANDLE ni_mutex;              /*!< Access mutex semaphore. */
    volatile int ni_insane;       /*!< Set by error detection. */
};

/*!
 * \brief Network interface controller information type.
 */
typedef struct _EMACINFO EMACINFO;

/*!
 * \addtogroup xgNutArchCm3Lpc17xxEmac
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
    unsigned int tout;

    LPC_EMAC->MADR = (NIC_PHY_ADDR_DEPRECATED << 8) | reg;
    LPC_EMAC->MCMD = EMAC_MCMD_READ;

    /* Wait until operation completed */
    tout = 0;
    for (tout = 0; tout < EMAC_MII_RD_TOUT; tout++) {
        if ((LPC_EMAC->MIND & EMAC_MIND_BUSY) == 0) {
            break;
        }
    }

    LPC_EMAC->MCMD = 0;
    return (LPC_EMAC->MRDD);
}

/*!
 * \brief Write value to PHY register.
 *
 * \param reg PHY register number.
 * \param val Value to write.
 */

static void phy_outw(uint8_t reg, uint16_t val)
{
    unsigned int tout;

    LPC_EMAC->MADR = (NIC_PHY_ADDR_DEPRECATED << 8) | reg;
    LPC_EMAC->MWTD = val;

    /* Wait utill operation completed */
    tout = 0;

    for (tout = 0; tout < EMAC_MII_WR_TOUT; tout++) {
        if ((LPC_EMAC->MIND & EMAC_MIND_BUSY) == 0) {
            break;
        }
    }
}

/*!
 * \brief Set the emac duplex mode.
 *
 * \param   full    1: full duplex, 0: half duplex
 *
 * \return  none;
 */
static void Lpc17xxEmacSetDuplex(int full)
{
    if(full) {
        LPC_EMAC->MAC2    |= EMAC_MAC2_FULL_DUP;
        LPC_EMAC->Command |= EMAC_CR_FULL_DUP;
        LPC_EMAC->IPGT     = EMAC_IPGT_FULL_DUP;
    } else {
        LPC_EMAC->IPGT = EMAC_IPGT_HALF_DUP;
    }
}

/*!
 * \brief Set the emac speed.
 *
 * \param   full    1: 100MBit, 0: 10 MBit
 *
 * \return  none;
 */
static void Lpc17xxEmacSetSpeed(int enable_100mBit)
{
    if(enable_100mBit) {
        LPC_EMAC->SUPP = EMAC_SUPP_SPEED;
    } else {
        LPC_EMAC->SUPP = 0;
    }
}

/*!
 * \brief Set the emac station address.
 *
 * \param   mac    MAC address (station address)
 *
 * \return  none;
 */

static void Lpc17xxEmacSetMACAddr(const uint8_t * mac)
{
    /* Set the Ethernet MAC Address registers */
    LPC_EMAC->SA0 = ((uint32_t)mac[5] << 8) | (uint32_t)mac[4];
    LPC_EMAC->SA1 = ((uint32_t)mac[3] << 8) | (uint32_t)mac[2];
    LPC_EMAC->SA2 = ((uint32_t)mac[1] << 8) | (uint32_t)mac[0];
}

/*! \fn Lpc17xxEmacWaitLinkThread(void *arg)
 * \brief Wait for link establishment and configure the NIC
 *        accordingly.
 */

THREAD(Lpc17xxEmacWaitLinkThread, arg)
{
    uint32_t phyval;
    int      link_wait;

    for (;;) {
        /* Restart autonegotiation */
        phyval = 1;
        NutPhyCtl(PHY_CTL_AUTONEG_RE, &phyval);

        /* Wait for auto negotiation completed and link established. */
        for (link_wait = EMAC_LINK_LOOPS;; link_wait--) {
            phyval = 0;
            NutPhyCtl(PHY_GET_STATUS, &phyval);

            if((phyval & PHY_STATUS_HAS_LINK) && (phyval & PHY_STATUS_AUTONEG_OK)) {
                /* Check link state and configure EMAC accordingly */
                if (phyval & PHY_STATUS_100M) {
                    Lpc17xxEmacSetSpeed(TRUE);
                    EMPRINTF("EMAC: Got link: 100 MBit ");
                } else {
                    Lpc17xxEmacSetSpeed(FALSE);
                    EMPRINTF("Link: Got link: 10 MBit ");
                }

                if (phyval & PHY_STATUS_FULLDUPLEX) {
                    Lpc17xxEmacSetDuplex(TRUE);
                    EMPRINTF("Full Duplex\n");
                } else {
                    Lpc17xxEmacSetDuplex(FALSE);
                    EMPRINTF("Half Duplex\n");
                }
                NutThreadExit();
                break;
            }
            if (link_wait == 0) {
                EMPRINTF("NO LINK!\n");
            }
            NutSleep(10);
        }
    }
}

/*!
 * \brief Reset the Ethernet controller.
 *
 * \return 0 on success, -1 on error
 */

static int Lpc17xxEmacReset(NUTDEVICE *dev)
{
    int      rc = 0;
    uint32_t phyval;
    int      idx;
    int32_t  tmp;
    IFNET   *ifn = (IFNET *) dev->dev_icb;

    EMPRINTF("Lpc17xxEmacReset\n");

    /* Reset all EMAC internal modules */
    LPC_EMAC->MAC1 = EMAC_MAC1_RES_TX | EMAC_MAC1_RES_MCS_TX | EMAC_MAC1_RES_RX |
                     EMAC_MAC1_RES_MCS_RX | EMAC_MAC1_SIM_RES | EMAC_MAC1_SOFT_RES;

    LPC_EMAC->Command = EMAC_CR_REG_RES | EMAC_CR_TX_RES | EMAC_CR_RX_RES | EMAC_CR_PASS_RUNT_FRM;

    /* A short delay after reset. */
    NutSleep(10);

    /* Initialize MAC control registers. */
    LPC_EMAC->MAC1 = EMAC_MAC1_PASS_ALL;
    LPC_EMAC->MAC2 = EMAC_MAC2_CRC_EN | EMAC_MAC2_PAD_EN;
    /* Set maximum frame size. */
    LPC_EMAC->MAXF = ifn->if_mtu + ETHER_HDR_LEN + ETHER_HDR_LEN;

    /* Find the clock that close to desired target clock */
    tmp = NutArchClockGet(NUT_HWCLK_CPU) / EMAC_MCFG_MII_MAXCLK;
    for (idx = 0; idx < sizeof (emac_clkdiv); idx++) {
        if (emac_clkdiv[idx] >= tmp) {
            break;
        }
    }

    if(idx >= sizeof (emac_clkdiv)) {
        return -1;
    }

    idx++;

    /* Write to MAC configuration register and reset */
    LPC_EMAC->MCFG = EMAC_MCFG_CLK_SEL(idx) | EMAC_MCFG_RES_MII;

    /* release reset */
    LPC_EMAC->MCFG &= ~(EMAC_MCFG_RES_MII);
    LPC_EMAC->CLRT = EMAC_CLRT_DEF;
    LPC_EMAC->IPGR = EMAC_IPGR_P2_DEF;

#ifdef PHY_MODE_RMII_DEPRECATED
    /* Enable Reduced MII interface. */
    LPC_EMAC->Command = EMAC_CR_RMII | EMAC_CR_PASS_RUNT_FRM;
#endif

    NutSleep(10);

    LPC_EMAC->SUPP = 0;

    /* Wait for PHY ready. */
    NutSleep(255);

    /* Register PHY
       PHY driver memory is only allocated one, but as NutRegisterPhy reads the
       PHY ID we need to have the (R)MII hardware initialised first
    */
    rc = NutRegisterPhy(1, phy_outw, phy_inw);

#ifndef PHY_MODE_RMII_DEPRECATED
    /* Clear MII isolate. */
    phyval = 0;
    NutPhyCtl(PHY_CTL_ISOLATE, &phyval);
#endif

    /* Restart autonegotiation */
    phyval = 1;
    NutPhyCtl(PHY_CTL_AUTONEG_RE, &phyval);

    if (NutThreadCreate("emaclink", Lpc17xxEmacWaitLinkThread, NULL,
        (NUT_THREAD_NICLINKSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD) == NULL) {
        EMPRINTF("Lpc17xxEmacReset() Could not create link establishment thread\n");
        rc = -1;
    }

    EMPRINTF("Lpc17xxEmacReset() DONE\n");

    return rc;
}

/*!
 * \brief Initialise the Emac RX descriptors.
 *
 * RX descriptors are located in the extra 16K internal SRAM of the CPU
 * and will be used for DMA operations.
 *
 * \param nb Network buffer structure containing the packet to be sent.
 *           The structure must have been allocated by a previous
 *           call NutNetBufAlloc(). This routine will automatically
 *           release the buffer in case of an error.
 *
 * \return  none
 */
static void Lpc17xxEmacRxDescriptorInit(void)
{
    unsigned int i;

    for (i = 0; i < EMAC_NUM_RX_FRAG; i++) {
        RX_DESC_PACKET(i)  = RX_BUF(i);
        RX_DESC_CTRL(i)    = EMAC_RCTRL_INT | (EMAC_ETH_RX_FRAG_SIZE-1);
        RX_STAT_INFO(i)    = 0;
        RX_STAT_HASHCRC(i) = 0;
    }

    /* Set EMAC Receive Descriptor Registers. */
    LPC_EMAC->RxDescriptor = RX_DESC_BASE;
    LPC_EMAC->RxStatus     = RX_STAT_BASE;
    LPC_EMAC->RxDescriptorNumber = EMAC_NUM_RX_FRAG-1;

    /* Rx Descriptors Point to 0 */
    LPC_EMAC->RxConsumeIndex  = 0;
}

/*!
 * \brief Initialise the Emac TX descriptors.
 *
 * RX descriptors are located in the extra 16K internal SRAM of the CPU
 * and will be used for DMA operations.
 *
 * \param nb Network buffer structure containing the packet to be sent.
 *           The structure must have been allocated by a previous
 *           call NutNetBufAlloc(). This routine will automatically
 *           release the buffer in case of an error.
 *
 * \return  none
 */
static void Lpc17xxEmacTxDescriptorInit (void)
{
    unsigned int i;

    for (i = 0; i < EMAC_NUM_TX_FRAG; i++)
    {
        TX_DESC_PACKET(i) = TX_BUF(i);
        TX_DESC_CTRL(i)   = 0;
        TX_STAT_INFO(i)   = 0;
    }

    /* Set EMAC Transmit Descriptor Registers. */
    LPC_EMAC->TxDescriptor = TX_DESC_BASE;
    LPC_EMAC->TxStatus     = TX_STAT_BASE;
    LPC_EMAC->TxDescriptorNumber = EMAC_NUM_TX_FRAG-1;

    /* Tx Descriptors Point to 0 */
    LPC_EMAC->TxProduceIndex  = 0;
}

/*
 * NIC interrupt entry.
 */
static void Lpc17xxEmacInterrupt(void *arg)
{
    uint32_t isr;
    EMACINFO *ni = (EMACINFO *) ((NUTDEVICE *) arg)->dev_dcb;

    /* EMAC Ethernet Controller Interrupt function. */

    /* Get EMAC interrupt status */
    isr = (LPC_EMAC->IntStatus & LPC_EMAC->IntEnable);
    if (isr != 0) {
        /* Clear interrupt status */
        LPC_EMAC->IntClear = isr;

        if (isr & EMAC_INT_RX_OVERRUN) {
            /* An RX overrun error is fatal. We have to soft-reset the RX Datapath */
            LPC_EMAC->Command |= EMAC_CR_RX_RES;
            /* Anyway post the RX queue so the receiver thread can read the just received frames */
            NutEventPostFromIrq(&ni->ni_rx_rdy);
        } else
        if (isr & (EMAC_INT_RX_DONE | EMAC_INT_RX_ERR)) {
            LPC_EMAC->IntEnable &= ~(EMAC_INT_RX_ERR | EMAC_INT_RX_DONE);
            NutEventPostFromIrq(&ni->ni_rx_rdy);
        }

        if (isr & EMAC_INT_TX_UNDERRUN)  {
            /* An TX underrun error is fatal. We have to soft-reset the TX Datapath */
            LPC_EMAC->Command |= EMAC_CR_TX_RES;
            /* Anyway post the TX queue so the transmitter can rwrite new frames */
            NutEventPostFromIrq(&ni->ni_tx_rdy);
        } else
        if (isr & (EMAC_INT_TX_DONE | EMAC_INT_TX_ERR)) {
            NutEventPostFromIrq(&ni->ni_tx_rdy);
        }
    }
}

/*!
 * \brief Fetch the next packet out of the receive buffers.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc17xxEmacGetPacket(NUTDEVICE *dev, NETBUF ** nbp)
{
    int      rc = -1;
    uint32_t idx;

    uint32_t produce_idx;
    uint32_t new_consume_idx;
    int32_t  rxlen;
    register uint32_t status;
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;

    *nbp = NULL;

    produce_idx  = LPC_EMAC->RxProduceIndex;

    idx = LPC_EMAC->RxConsumeIndex;
    rxlen = 0;

    if (idx != produce_idx) {
        int32_t size = 0;
        status = RX_STAT_INFO(idx);

        while (status && idx != produce_idx) {
            size += (status & EMAC_RINFO_SIZE) + 1;

            if (++idx == EMAC_NUM_RX_FRAG) {
                idx = 0;
            }

            new_consume_idx = idx;

            if (status & EMAC_RINFO_LAST_FLAG) {
                rxlen = size;
                break;
            }

            status = RX_STAT_INFO(idx);
        }
    }


    /* Check if there was any data received */

    if(rxlen > 0) {
        idx = LPC_EMAC->RxConsumeIndex;

        /* substract 4 bytes CTC. */
        rxlen -= 4;

        /* "status" represents the status flag of the last frame fragment */

        if (status & (EMAC_RINFO_NO_DESCR)) {
            EMPRINTF("Emac Rx Err, Status: %08lx\n", status & (EMAC_RINFO_NO_DESCR));
        } 

        /* Check if the frame was correctly received */
        if((status & EMAC_RINFO_ERR_MASK) == 0) {
            /*
             * Receiving long packets is unexpected. Let's declare the
             * chip insane. Short packets will be handled by the caller.
             */

            if (rxlen < 0) {
                ni->ni_insane = 1;
            } else {
                *nbp = NutNetBufAlloc(0, NBAF_DATALINK + (2 & (NUTMEM_ALIGNMENT - 1)), (uint16_t)rxlen);

                if (*nbp != NULL) {
                    uint8_t *bp = (uint8_t *) (* nbp)->nb_dl.vp;
                    unsigned int len;

                    idx = LPC_EMAC->RxConsumeIndex;
                    while (rxlen) {
                        if (rxlen > EMAC_ETH_RX_FRAG_SIZE) {
                            len = EMAC_ETH_RX_FRAG_SIZE;
                        } else {
                            len = rxlen;
                        }

                        /* Copy received data from current frame fragment */
                        memcpy(bp, (void*)RX_DESC_PACKET(idx), len);
                        /* Clear status info and crc hash from current frame fragment */
                        RX_STAT_INFO(idx)    = 0;
                        RX_STAT_HASHCRC(idx) = 0;

                        if (++idx == EMAC_NUM_RX_FRAG) {
                            idx = 0;
                        }

                        rxlen -= len;
                        bp += len;
                    }
                    rc = 0;
                }
            }
        } else {
            EMPRINTF("Emac Rx Err, Discard Packet. Status: %08lx\n", status);
            /* Silently discard the frame from EMAC buffer, update the Rx consume index */
        }

        /* Release frame from EMAC buffer, update the Rx consume index */
        LPC_EMAC->RxConsumeIndex = new_consume_idx;
    }
    return rc;
}

/*!
 * \brief Load a packet into the nic's transmit ring buffer.
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
static inline int Lpc17xxEmacPutPacket(NUTDEVICE *dev, NETBUF * nb)
{
    int       rc = -1;
    uint16_t  sz;
    uint8_t  *buf;
    uint32_t  idx;
    IFNET    *ifn = (IFNET *) dev->dev_icb;
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;

    /*
     * Calculate the number of bytes to be send. Do not send packets
     * larger than the Ethernet maximum transfer unit. The MTU
     * consist of 1500 data bytes plus the 14 byte Ethernet header
     * plus 4 bytes CRC. We check the data bytes only.
     */
    if ((sz = nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz) > ifn->if_mtu) {
        return -1;
    }

    sz += nb->nb_dl.sz;

    /* Disable EMAC interrupts. */
    NutIrqDisable(&sig_EMAC);

    /* TODO: Check for link. */
    if (ni->ni_insane == 0) {
        int32_t  txlen = sz;
        int32_t  offs  = 0;

        /* Allocate a descriptors for sending frame and get the coressponding
           buffer address NIC interrupts must be disabled.
         */

        idx = LPC_EMAC->TxProduceIndex;

        while (txlen > 0) {
            /* Get the current transmit fragment data pointer */
            buf = (uint8_t*)TX_DESC_PACKET(idx);

            if (txlen > EMAC_ETH_TX_FRAG_SIZE) {
                /* This is not the last fragment, it is completely filled */
                TX_DESC_CTRL(idx) = ((EMAC_ETH_TX_FRAG_SIZE - 1) & EMAC_TCTRL_SIZE) | EMAC_TCTRL_INT;
            } else {
                /* Round up the len to the next even number and mark this as the last fragment */
                TX_DESC_CTRL(idx) = ((txlen - 1) & EMAC_TCTRL_SIZE) | (EMAC_TCTRL_INT | EMAC_TCTRL_LAST);
            }

            if (txlen == sz) {
                /* This is the first fragment, copy the header bytes and then the application data */
                memcpy(buf, nb->nb_dl.vp, nb->nb_dl.sz);
                buf += nb->nb_dl.sz;
                txlen -= nb->nb_dl.sz;
                memcpy(buf, nb->nb_nw.vp, nb->nb_nw.sz);
                buf += nb->nb_nw.sz;
                txlen -= nb->nb_nw.sz;
                memcpy(buf, nb->nb_tp.vp, nb->nb_tp.sz);
                buf += nb->nb_tp.sz;
                txlen -= nb->nb_tp.sz;

                offs = min (EMAC_ETH_TX_FRAG_SIZE - nb->nb_dl.sz - nb->nb_nw.sz - nb->nb_tp.sz, nb->nb_ap.sz);

                memcpy(buf, nb->nb_ap.vp, offs);
                buf   += offs;
                txlen -= offs;
            } else {
                register int len = min(EMAC_ETH_TX_FRAG_SIZE, nb->nb_ap.sz - offs);

                /* Continue copying the application data */
                memcpy(buf, (uint8_t *)nb->nb_ap.vp + offs, len);
                offs  += len;
                txlen -= len;
            }
            /* Increment produce index counter, taking wrap around into account */
            if (++idx == LPC_EMAC->TxDescriptorNumber + 1) {
                idx = 0;
            }
        }

        /* Update tx produce index
           Increase the TxProduceIndex (after writting to the Transmit buffer
           to enable the Transmit buffer)
         */

        LPC_EMAC->TxProduceIndex = idx;

        rc = 0;
#ifdef NUT_PERFMON
        ni->ni_tx_packets++;
#endif
    }

    /* Enable EMAC interrupts. */
    NutIrqEnable(&sig_EMAC);

    return rc;
}

/*!
 * \brief Fire up the network interface.
 *
 * NIC interrupts must be disabled when calling this function.
 *
 * \param mac Six byte unique MAC address.
 */
static int Lpc17xxEmacStart(NUTDEVICE *dev)
{
    IFNET    *ifn = (IFNET *) dev->dev_icb;

    /* Set local MAC address. */
    Lpc17xxEmacSetMACAddr(ifn->if_mac);

    /* Initialize the rx and tx buffer descriptors */
    Lpc17xxEmacRxDescriptorInit();
    Lpc17xxEmacTxDescriptorInit();

    /* Set Receive Filter register: enable broadcast and multicast */
    LPC_EMAC->RxFilterCtrl = EMAC_RFC_BCAST_EN | EMAC_RFC_PERFECT_EN;

    /* Reset all interrupts, Interrupts will be enabled in the RX-Thread */
    LPC_EMAC->IntClear  = 0xFFFF;

    /* Enable Transmitter and receiver */
    LPC_EMAC->Command |= EMAC_CR_TX_EN | EMAC_CR_RX_EN;
    LPC_EMAC->MAC1 |= EMAC_MAC1_REC_EN;

    EMPRINTF("Lpc17xxEmacStart() DONE\n");

    return 0;
}

/*! \fn EmacRxThread(void *arg)
 * \brief NIC receiver thread.
 *
 */
THREAD(Lpc17xxEmacRxThread, arg)
{
    NUTDEVICE *dev = arg;
    IFNET *ifn = (IFNET *) dev->dev_icb;
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;
    NETBUF *nb;

    EMPRINTF("Lpc17xxEmacRxThread() INIT\n");

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

    Lpc17xxEmacStart(dev);

    /* Initialize the access mutex. */
    NutEventPost(&ni->ni_mutex);

    /* Run at high priority. */
    NutThreadSetPriority(9);

    /* Enable receive and transmit interrupts. */
    LPC_EMAC->IntEnable |= EMAC_INT_RX_OVERRUN | EMAC_INT_RX_ERR | EMAC_INT_RX_DONE |
                           EMAC_INT_TX_UNDERRUN | EMAC_INT_TX_ERR | EMAC_INT_TX_DONE;

    NutIrqEnable(&sig_EMAC);

    for (;;) {
        /*
         * Wait for the arrival of new packets or poll the receiver every
         * 2 seconds. The timeout can be modified if there are interrupt problems.
         */
        NutEventWait(&ni->ni_rx_rdy, 2000);
        /*
         * Fetch all packets from the NIC's internal buffer and pass
         * them to the registered handler.
         */
        while (Lpc17xxEmacGetPacket(dev, &nb) == 0) {
            /* Discard short packets. */
            if (nb->nb_dl.sz < 60) {
                NutNetBufFree(nb);
            } else {
                (*ifn->if_recv) (dev, nb);
            }
        }
        LPC_EMAC->IntEnable |= EMAC_INT_RX_OVERRUN | EMAC_INT_RX_ERR | EMAC_INT_RX_FIN | EMAC_INT_RX_DONE;

        /* We got a weird chip, try to restart it. */
        while (ni->ni_insane) {
            if (Lpc17xxEmacReset(dev)) {
                NutSleep(500);
            } else {
                Lpc17xxEmacStart(dev);
                ni->ni_insane = 0;
                NutIrqEnable(&sig_EMAC);
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
int Lpc17xxEmacOutput(NUTDEVICE * dev, NETBUF * nb)
{
    static uint32_t mx_wait = 5000;
    uint32_t consume_idx;
    uint32_t produce_idx;
    uint32_t num_fragments;
    int rc = -1;
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;
    size_t space;
    size_t pkg_size;

    /*
     * After initialization we are waiting for a long time to give
     * the PHY a chance to establish an Ethernet link.
     */
    while (rc) {
        /* Check if we stay in an error condition */
        if (ni->ni_insane) {
            break;
        }
        if (NutEventWait(&ni->ni_mutex, mx_wait)) {
            break;
        }

        /* Check for packet queue space. */

        consume_idx   = LPC_EMAC->TxConsumeIndex;
        produce_idx   = LPC_EMAC->TxProduceIndex;
        num_fragments = LPC_EMAC->TxDescriptorNumber + 1;

        /* empty */
        if (consume_idx == produce_idx) {
            space = num_fragments * EMAC_ETH_TX_FRAG_SIZE;
        } else
        /* full */
        if ((consume_idx == 0) && (produce_idx == num_fragments - 1)) {
            space = 0;
        } else
        /* Wrap-around */
        if (consume_idx == produce_idx + 1) {
            space = 0;
        } else {
            /* Calc free space */
            if (consume_idx > produce_idx) {
                space = consume_idx - produce_idx;
            } else {
                space = num_fragments - (produce_idx - consume_idx);
            }
            space *= EMAC_ETH_TX_FRAG_SIZE;
        }

        pkg_size = nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz + nb->nb_dl.sz;

        if (space < pkg_size) {
            if (NutEventWait(&ni->ni_tx_rdy, 500)) {
                /* No queue space. Release the lock and give up. */
                NutEventPost(&ni->ni_mutex);
                break;
            }
        } else
        if (Lpc17xxEmacPutPacket(dev, nb) == 0) {
            /* Ethernet works. Set a long waiting time in case we
               temporarly lose the link next time. */
            rc = 0;
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
 * \brief Calculate CRC of a number of bytes (e.g. of an ethernet frame)
 *
 * \param   data    Pointer to the first byte of the data buffer
 * \param   len     length of the data
 *
 * \return  Calculated CRC (32 bit)
 */

static int32_t Lpc17xxEmacCalcCrc(uint8_t *data, int32_t len)
{
    int i, j;
    char byte;
    int crc;
    int q0, q1, q2, q3;

    crc = 0xFFFFFFFF;

    for (i = 0; i < len; i++) {
        byte = *(data++);
        for (j = 0; j < 2; j++) {
            if (((crc >> 28) ^ (byte >> 3)) & 0x00000001) {
                q3 = 0x04C11DB7;
            } else {
                q3 = 0x00000000;
            }

            if (((crc >> 29) ^ (byte >> 2)) & 0x00000001) {
                q2 = 0x09823B6E;
            } else {
                q2 = 0x00000000;
            }

            if (((crc >> 30) ^ (byte >> 1)) & 0x00000001) {
                q1 = 0x130476DC;
            } else {
                q1 = 0x00000000;
            }

            if (((crc >> 31) ^ (byte >> 0)) & 0x00000001) {
                q0 = 0x2608EDB8;
            } else {
                q0 = 0x00000000;
            }

            crc = (crc << 4) ^ q3 ^ q2 ^ q1 ^ q0;

            byte >>= 4;
        }
    }

    return crc;
}

/*!
 * \brief Update the multicast hash.
 *
 * This function must be called after the multicast list changed.
 *
 * \param mclst Pointer to the first entry of the linked list of
 *              multicast entries.
 */
static void Lpc17xxEmacHashUpdate(MCASTENTRY *mclst)
{
    uint32_t idx;
    uint32_t hash[2] = { 0, 0 };

    /* Determine the hash bit for each entry. */
    while (mclst) {
        /* Calc CRC of the hash entry. Bits 23-28 are used as hash index */
        idx = (Lpc17xxEmacCalcCrc(mclst->mca_ha, 6) >> 23) & 0x3F;

        /* Set the bit given by the 6 bit index. */
        hash[idx > 31] |= 1 << (idx & 31);
        mclst = mclst->mca_next;
    }
    /* Set result in the hash register. */
    LPC_EMAC->HashFilterL = hash[0];
    LPC_EMAC->HashFilterH = hash[1];

    /* Enable or disable multicast hash. */
    if (hash[0] || hash[1]) {
        LPC_EMAC->RxFilterCtrl |= EMAC_RFC_MCAST_HASH_EN;
    } else {
        LPC_EMAC->RxFilterCtrl &= ~EMAC_RFC_MCAST_HASH_EN;
    }
}

/*!
 * \brief Get multicast entry of a given IP address.
 *
 * \todo This function should be shared by all Ethernet drivers.
 *
 * \param ifn Pointer to the network interface structure.
 * \param ip  IP address of the entry to retrieve.
 *
 * \return Pointer to the entry or NULL if none exists.
 */
static MCASTENTRY *McastIpEntry(IFNET *ifn, uint32_t ip)
{
    MCASTENTRY *mca = ifn->if_mcast;

    while (mca) {
        if (ip == mca->mca_ip) {
            break;
        }
        mca = mca->mca_next;
    }
    return mca;
}

/*!
 * \brief Add a given IP address to the multicast list.
 *
 * \todo This function should be shared by all Ethernet drivers.
 *
 * \param ifn Pointer to the network interface structure.
 * \param ip  IP address of the new entry.
 *
 * \return Pointer to the requested entry, either a new one or
 *         an already existing entry. In case of a failure, NULL
 *         is returned.
 */
static MCASTENTRY *McastAddEntry(IFNET *ifn, uint32_t ip)
{
    MCASTENTRY *mca;

    mca = McastIpEntry(ifn, ip);
    if (mca == NULL) {
        mca = malloc(sizeof(MCASTENTRY));
        if (mca) {
            mca->mca_ip = ip;
            /* Set the IANA OUI. */
            mca->mca_ha[0] = 0x01;
            mca->mca_ha[1] = 0x00;
            mca->mca_ha[2] = 0x5e;
            /* Map the lower 23 bits of the IP address to the MAC address.
               Note that Nut/Net IP addresses are in network byte order. */
            mca->mca_ha[3] = (ip >> 8) & 0x7f;
            mca->mca_ha[4] = (ip >> 16) & 0xff;
            mca->mca_ha[5] = (ip >> 24) & 0xff;
            /* Add the new entry to the front of the list. */
            mca->mca_next = ifn->if_mcast;
            ifn->if_mcast = mca;
            /* Update the EMAC's multicast hash. */
            Lpc17xxEmacHashUpdate(mca);
        }
    }
    return mca;
}

/*!
 * \brief Remove multicast entry of a given IP address.
 *
 * \param ifn Pointer to the network interface structure.
 * \param ip  IP address of the entry to remove.
 */
static void McastDelEntry(IFNET *ifn, uint32_t ip)
{
    MCASTENTRY *mca = ifn->if_mcast;
    MCASTENTRY **lnk = &ifn->if_mcast;

    while (mca) {
        if (mca->mca_ip == ip) {
            *lnk = mca->mca_next;
            free(mca);
            break;
        }
        lnk = &mca->mca_next;
        mca = *lnk;
    }
    /* Update the EMAC's multicast hash. */
    Lpc17xxEmacHashUpdate(ifn->if_mcast);
}

/*!
 * \brief Perform Ethernet control functions.
 *
 * \param dev  Identifies the device that receives the device-control
 *             function.
 * \param req  Requested control function. May be set to one of the
 *             following constants:
 *             - SIOCSIFADDR sets interface MAC address passed in buffer.
 *             - SIOCGIFADDR copies current interface MAC address to buffer.
 *             - SIOCADDMULTI adds multicast entry for IP in buffer.
 *             - SIOCDELMULTI removes multicast entry of IP in buffer.
 * \param conf Points to a buffer that contains any data required for
 *             the given control function or receives data from that
 *             function.
 * \return 0 on success, -1 otherwise.
 *
 */
static int Lpc17xxEmacIoCtl(NUTDEVICE * dev, int req, void *conf)
{
    int rc = 0;
    IFNET *ifn = (IFNET *) dev->dev_icb;
    uint32_t ip;

    switch (req) {
    case SIOCSIFADDR:
        /* Set interface hardware address. */
        memcpy(ifn->if_mac, conf, sizeof(ifn->if_mac));
        break;
    case SIOCGIFADDR:
        /* Get interface hardware address. */
        memcpy(conf, ifn->if_mac, sizeof(ifn->if_mac));
        break;
    case SIOCADDMULTI:
        /* Add multicast address. */
        memcpy(&ip, conf, sizeof(ip));
        if (McastAddEntry(ifn, ip) == NULL) {
            rc = -1;
        }
        break;
    case SIOCDELMULTI:
        /* Delete multicast address. */
        memcpy(&ip, conf, sizeof(ip));
        McastDelEntry(ifn, ip);
        break;
    default:
        rc = -1;
        break;
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
int Lpc17xxEmacInit(NUTDEVICE * dev)
{
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;

    EMPRINTF("Lpc17xxEmacInit()\n");

#if defined(EMAC_ALLOC_DMA_BUFFER_FROM_HEAP)
    /* Allocate DMA buffer from heap. Make sure it is 128 byte alligned */
    emac_dma_buffer = malloc(EMAC_DMA_BUFFER_SIZE + 128);
    /* Align the buffer to a 128 byte boundary */
    emac_dma_buffer_base = ((uintptr_t)emac_dma_buffer + 128) & (~128L);
#endif

    SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCENET);

    /* Configure P1 Ethernet pins for RMII interface. */
    /* on rev. 'A' and later, P1.6 should NOT be set. */
    GpioPinConfigSet(NUTGPIO_PORT1, 0, GPIO_CFG_PERIPHERAL1);   /* ETH_TXD0 */
    GpioPinConfigSet(NUTGPIO_PORT1, 1, GPIO_CFG_PERIPHERAL1);   /* ETH_TXD1 */
    GpioPinConfigSet(NUTGPIO_PORT1, 4, GPIO_CFG_PERIPHERAL1);   /* ETH_TXEN */
    GpioPinConfigSet(NUTGPIO_PORT1, 8, GPIO_CFG_PERIPHERAL1);   /* ETH_CRS  */
    GpioPinConfigSet(NUTGPIO_PORT1, 9, GPIO_CFG_PERIPHERAL1);   /* ETH_RXD0 */
    GpioPinConfigSet(NUTGPIO_PORT1, 10, GPIO_CFG_PERIPHERAL1);  /* ETH_RXD1 */
    GpioPinConfigSet(NUTGPIO_PORT1, 14, GPIO_CFG_PERIPHERAL1);  /* ETH_RXER */
    GpioPinConfigSet(NUTGPIO_PORT1, 15, GPIO_CFG_PERIPHERAL1);  /* ETH_RXCLK */
    GpioPinConfigSet(NUTGPIO_PORT1, 16, GPIO_CFG_PERIPHERAL1);  /* ETH_MDC  */
    GpioPinConfigSet(NUTGPIO_PORT1, 17, GPIO_CFG_PERIPHERAL1);  /* ETH_MDIO */

#ifndef PHY_MODE_RMII_DEPRECATED
    /* Configure further I/O pins for MII interface */
    GpioPinConfigSet(NUTGPIO_PORT1, 2, GPIO_CFG_PERIPHERAL1);   /* ETH_TXD2 */
    GpioPinConfigSet(NUTGPIO_PORT1, 3, GPIO_CFG_PERIPHERAL1);   /* ETH_TXD3 */
    GpioPinConfigSet(NUTGPIO_PORT1, 5, GPIO_CFG_PERIPHERAL1);   /* ETH_TXER */
    GpioPinConfigSet(NUTGPIO_PORT1, 6, GPIO_CFG_PERIPHERAL1);   /* ETH_TXCLK */
    GpioPinConfigSet(NUTGPIO_PORT1, 7, GPIO_CFG_PERIPHERAL1);   /* ETH_COL  */
    GpioPinConfigSet(NUTGPIO_PORT1, 11, GPIO_CFG_PERIPHERAL1);  /* ETH_RXD2 */
    GpioPinConfigSet(NUTGPIO_PORT1, 12, GPIO_CFG_PERIPHERAL1);  /* ETH_RXD3 */
    GpioPinConfigSet(NUTGPIO_PORT1, 13, GPIO_CFG_PERIPHERAL1);  /* ETH_RXDV */
#endif

    /* Reset the controller. */
    if (Lpc17xxEmacReset(dev) == -1) {
        return -1;
    }

    /* Clear EMACINFO structure. */
    memset(ni, 0, sizeof(EMACINFO));

    /* Register interrupt handler. */
    if (NutRegisterIrqHandler(&sig_EMAC, Lpc17xxEmacInterrupt, dev)) {
        EMPRINTF("EMAC: Registering IRQ failed\n");
        return -1;
    }

    /* Start the receiver thread. */
    if (NutThreadCreate("emacrx", Lpc17xxEmacRxThread, dev,
        (NUT_THREAD_NICRXSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD) == NULL) {
        EMPRINTF("EMAC: Registering RX Thread failed\n");
        return -1;
    }

    EMPRINTF("Lpc17xxEmacInit() DONE\n");

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
    Lpc17xxEmacOutput,          /*!< \brief Driver output routine, if_send(). */
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
NUTDEVICE devLpc17xxEmac = {
    0,                          /*!< \brief Pointer to next device. */
    {'e', 't', 'h', '0', 0, 0, 0, 0, 0},        /*!< \brief Unique device name. */
    IFTYP_NET,                  /*!< \brief Type of device. */
    0,                          /*!< \brief Base address. */
    0,                          /*!< \brief First interrupt number. */
    &ifn_eth0,                  /*!< \brief Interface control block. */
    &dcb_eth0,                  /*!< \brief Driver control block. */
    Lpc17xxEmacInit,            /*!< \brief Driver initialization routine. */
    Lpc17xxEmacIoCtl,           /*!< \brief Driver specific control function. */
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
