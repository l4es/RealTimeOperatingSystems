/*
 * Copyright (C) 2006-2007 by egnite Software GmbH. All rights reserved.
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

/*
 * $Id: at91_emac.c 6125 2015-07-28 14:53:16Z u_bonnes $
 */

#include <cfg/os.h>
#include <cfg/dev.h>
#include <arch/arm.h>
#include <cfg/arch/gpio.h>

#include <stdlib.h>
#include <string.h>

#include <sys/atom.h>
#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <sys/confnet.h>

#include <netinet/if_ether.h>
#include <net/ether.h>
#include <net/if_var.h>

#include <dev/irqreg.h>
#include <dev/at91_emac.h>
#include <dev/phy.h>

#include <stdio.h>

/* WARNING: Variadic macros are C99 and may fail with C89 compilers. */
#ifdef NUTDEBUG
#include <stdio.h>
#include <arpa/inet.h>
#define EMPRINTF(args,...) printf(args,##__VA_ARGS__);fflush(stdout)
#else
#define EMPRINTF(args,...)
#endif

#ifndef NUT_THREAD_NICRXSTACK
/* arm-elf-gcc used 168 bytes with optimized, 412 bytes with debug code. */
#define NUT_THREAD_NICRXSTACK   320
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

/*!
 * \brief PHY ID.
 *
 * If set to 0xffffffff, the PHY id will be ignored.
 */
#ifndef NIC_PHY_UID
#define NIC_PHY_UID 0xffffffff
#endif

/*!
 * \brief Check all known PHY IDs.
 *
 * If defined, perform the old PHY checks. This ensures compatibility
 * at the cost of bloat. Should be removed later on when all boards
 * have their PHY ids in their configuration.
 */
#define CHECK_ALL_KNOWN_PHY_IDS

#if defined (MCU_AT91SAM9260) || defined(MCU_AT91SAM9XE512)

/*!
 * The AT91SAM9260-EK board is delivered with RMII by default. Thus.
 * we use the reduced MII for this CPU. However, this should be
 * handled by the Configurator.
 */
#define PHY_MODE_RMII_DEPRECATED

//#define EMAC_PIO_PER            PIOA_PER
//#define EMAC_PIO_OER            PIOA_OER
//#define EMAC_PIO_CODR           PIOA_CODR
#define EMAC_PIO_ASR            PIOA_ASR
#define EMAC_PIO_BSR            PIOA_BSR
#define EMAC_PIO_PDR            PIOA_PDR

#define PHY_TXD0_BIT            PA12_ETX0_A     /*!< \brief Transmit data bit 0 pin. */
#define PHY_TXD1_BIT            PA13_ETX1_A     /*!< \brief Transmit data bit 1 pin. */
#define PHY_RXD0_AD0_BIT        PA14_ERX0_A     /*!< \brief Receive data bit 0 pin. */
#define PHY_RXD1_AD1_BIT        PA15_ERX1_A     /*!< \brief Receive data bit 1 pin. */
#define PHY_TXEN_BIT            PA16_ETXEN_A    /*!< \brief Transmit enable pin. */
#define PHY_RXDV_TESTMODE_BIT   PA17_ERXDV_A    /*!< \brief Data valid pin. */
#define PHY_RXER_RXD4_RPTR_BIT  PA18_ERXER_A    /*!< \brief Receive error pin. */
#define PHY_TXCLK_ISOLATE_BIT   PA19_ETXCK_A    /*!< \brief Transmit clock pin. */
#define PHY_MDC_BIT             PA20_EMDC_A     /*!< \brief Management data clock pin. */
#define PHY_MDIO_BIT            PA21_EMDIO_A    /*!< \brief Management data I/O pin. */

#ifndef PHY_MODE_RMII_DEPRECATED
#define PHY_TXD2_BIT            PA10_ETX2_B     /*!< \brief Transmit data bit 2 pin. */
#define PHY_TXD3_BIT            PA11_ETX3_B     /*!< \brief Transmit data bit 3 pin. */
#define PHY_TXER_TXD4_BIT       PA22_ETXER_B    /*!< \brief Transmit error pin. */
#define PHY_RXCLK_10BTSER_BIT   PA27_ERXCK_B    /*!< \brief Receive clock pin. */
#define PHY_COL_RMII_BIT        PA29_ECOL_B     /*!< \brief Collision detect pin. */
#endif

#define PHY_RXD2_AD2_BIT        PA25_ERX2_B     /*!< \brief Receive data bit 2 pin. */
#define PHY_RXD3_AD3_BIT        PA26_ERX3_B     /*!< \brief Receive data bit 3 pin. */
#define PHY_CRS_AD4_BIT         PA28_ECRS_B     /*!< \brief Carrier sense pin. */

#define PHY_MII_PINS_A 0 \
    | _BV(PHY_TXD0_BIT) \
    | _BV(PHY_TXD1_BIT) \
    | _BV(PHY_RXD0_AD0_BIT) \
    | _BV(PHY_RXD1_AD1_BIT) \
    | _BV(PHY_TXEN_BIT) \
    | _BV(PHY_RXDV_TESTMODE_BIT) \
    | _BV(PHY_RXER_RXD4_RPTR_BIT) \
    | _BV(PHY_TXCLK_ISOLATE_BIT) \
    | _BV(PHY_MDC_BIT) \
    | _BV(PHY_MDIO_BIT)

#ifdef PHY_MODE_RMII_DEPRECATED
#define PHY_MII_PINS_B 0
#else
#define PHY_MII_PINS_B 0 \
    | _BV(PHY_TXD2_BIT) \
    | _BV(PHY_TXD3_BIT) \
    | _BV(PHY_TXER_TXD4_BIT) \
    | _BV(PHY_RXD2_AD2_BIT) \
    | _BV(PHY_RXD3_AD3_BIT) \
    | _BV(PHY_RXCLK_10BTSER_BIT) \
    | _BV(PHY_CRS_AD4_BIT) \
    | _BV(PHY_COL_RMII_BIT)
#endif

#elif defined (MCU_AT91SAM7X)

#define EMAC_PIO_PER            PIOB_PER
#define EMAC_PIO_OER            PIOB_OER
#define EMAC_PIO_CODR           PIOB_CODR
#define EMAC_PIO_SODR           PIOB_SODR
#define EMAC_PIO_PUER           PIOB_PUER
#define EMAC_PIO_PUDR           PIOB_PUDR
#define EMAC_PIO_ASR            PIOB_ASR
#define EMAC_PIO_BSR            PIOB_BSR
#define EMAC_PIO_PDR            PIOB_PDR

#define PHY_TXCLK_ISOLATE_BIT   0
#define PHY_REFCLK_XT2_BIT      0
#define PHY_TXEN_BIT            1
#define PHY_TXD0_BIT            2
#define PHY_TXD1_BIT            3
#define PHY_CRS_AD4_BIT         4
#define PHY_RXD0_AD0_BIT        5
#define PHY_RXD1_AD1_BIT        6
#define PHY_RXER_RXD4_RPTR_BIT  7
#define PHY_MDC_BIT             8
#define PHY_MDIO_BIT            9
#define PHY_TXD2_BIT            10
#define PHY_TXD3_BIT            11
#define PHY_TXER_TXD4_BIT       12
#define PHY_RXD2_AD2_BIT        13
#define PHY_RXD3_AD3_BIT        14
#define PHY_RXDV_TESTMODE_BIT   15
#define PHY_COL_RMII_BIT        16
#define PHY_RXCLK_10BTSER_BIT   17
#define PHY_MDINTR_BIT          26

#define PHY_MII_PINS_A 0 \
    | _BV(PHY_REFCLK_XT2_BIT) \
    | _BV(PHY_TXEN_BIT) \
    | _BV(PHY_TXD0_BIT) \
    | _BV(PHY_TXD1_BIT) \
    | _BV(PHY_CRS_AD4_BIT) \
    | _BV(PHY_RXD0_AD0_BIT) \
    | _BV(PHY_RXD1_AD1_BIT) \
    | _BV(PHY_RXER_RXD4_RPTR_BIT) \
    | _BV(PHY_MDC_BIT) \
    | _BV(PHY_MDIO_BIT) \
    | _BV(PHY_TXD2_BIT) \
    | _BV(PHY_TXD3_BIT) \
    | _BV(PHY_TXER_TXD4_BIT) \
    | _BV(PHY_RXD2_AD2_BIT) \
    | _BV(PHY_RXD3_AD3_BIT) \
    | _BV(PHY_RXDV_TESTMODE_BIT) \
    | _BV(PHY_COL_RMII_BIT) \
    | _BV(PHY_RXCLK_10BTSER_BIT)

#define PHY_MII_PINS_B 0

#endif

/*!
 * \brief Network interface controller information structure.
 */
struct _EMACINFO {
#ifdef NUT_PERFMON
    uint32_t ni_rx_packets;       /*!< Number of packets received. */
    uint32_t ni_tx_packets;       /*!< Number of packets sent. */
    uint32_t ni_overruns;         /*!< Number of packet overruns. */
    uint32_t ni_rx_frame_errors;  /*!< Number of frame errors. */
    uint32_t ni_rx_crc_errors;    /*!< Number of CRC errors. */
    uint32_t ni_rx_missed_errors; /*!< Number of missed packets. */
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
typedef struct _BufDescriptor {
    unsigned int addr;
    unsigned int stat;
} BufDescriptor;

static volatile BufDescriptor txBufTab[EMAC_TX_BUFFERS];
static volatile uint8_t txBuf[EMAC_TX_BUFFERS * EMAC_TX_BUFSIZ] NUT_ALIGNED_TYPE(8);
static unsigned int txBufIdx;

static volatile BufDescriptor rxBufTab[EMAC_RX_BUFFERS];
static volatile uint8_t rxBuf[EMAC_RX_BUFFERS * EMAC_RX_BUFSIZ] NUT_ALIGNED_TYPE(8);
static unsigned int rxBufIdx;

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

#define MII_DM9161_ID     0x0181b8a0
#define MII_AM79C875_ID   0x00225540
#define MII_MICREL_ID     0x00221610
#define MII_LAN8700_ID    0x0007c0c0
#define MII_LAN8710_ID    0x0007C0F0


/*!
 * \addtogroup xgNutArchArmAt91Emac
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
    /* PHY read command. */
    outr(EMAC_MAN, EMAC_SOF | EMAC_RW_READ | EMAC_CODE |
        (NIC_PHY_ADDR_DEPRECATED << EMAC_PHYA_LSB) | (reg << EMAC_REGA_LSB));

    /* Wait until PHY logic completed. */
    while ((inr(EMAC_NSR) & EMAC_IDLE) == 0);

    /* Get data from PHY maintenance register. */
    return (uint16_t) (inr(EMAC_MAN) >> EMAC_DATA_LSB);
}

/*!
 * \brief Write value to PHY register.
 *
 * \param reg PHY register number.
 * \param val Value to write.
 */
static void phy_outw(uint8_t reg, uint16_t val)
{
    /* PHY write command. */
    outr(EMAC_MAN, EMAC_SOF | EMAC_RW_WRITE | EMAC_CODE |
        (NIC_PHY_ADDR_DEPRECATED << EMAC_PHYA_LSB) | (reg << EMAC_REGA_LSB) | val);

    /* Wait until PHY logic completed. */
    while ((inr(EMAC_NSR) & EMAC_IDLE) == 0);
}

/*!
 * \brief Reset the Ethernet controller.
 *
 * \return 0 on success, -1 otherwise.
 */

static int EmacReset(uint32_t tmo)
{
    int rc = 0;
    uint32_t reg_ncfgr;
    uint32_t phyval;
    int      link_wait;

    EMPRINTF("EmacReset(%lu)\n", tmo);

    /* Enable power sources if not yet enabled */
    outr(PMC_PCER, _BV(PIOA_ID));
    outr(PMC_PCER, _BV(PIOB_ID));
    outr(PMC_PCER, _BV(EMAC_ID));

    /* Configure MII port. */
    outr(EMAC_PIO_ASR, PHY_MII_PINS_A);
    outr(EMAC_PIO_BSR, PHY_MII_PINS_B);
    outr(EMAC_PIO_PDR, PHY_MII_PINS_A | PHY_MII_PINS_B);

    /* Enable receive and transmit clocks and set MII mode. */
#ifdef PHY_MODE_RMII_DEPRECATED
    outr(EMAC_USRIO, EMAC_RMII | EMAC_CLKEN);
#else
    outr(EMAC_USRIO, EMAC_CLKEN);
#endif

    /* Enable management port. */
    outr(EMAC_NCR, inr(EMAC_NCR) | EMAC_MPE);
    outr(EMAC_NCFGR, inr(EMAC_NCFGR) | EMAC_CLK_HCLK_64);

    /* Wait for PHY ready. */
    NutDelay(255);

    /* Register PHY */
    rc = NutRegisterPhy( 1, phy_outw, phy_inw);

#if NIC_PHY_UID == MII_LAN8710_ID
    /* Set LAN8710 to AUTO-MDIX and MII mode.
     * This overides configuration set by config pins of the chip.
     */

    phyval = 18 << 16;  // Store phy register address in upper 16 bits
    NutPhyCtl (PHY_GET_REGVAL, &phyval);
    phyval |= 0x00E0;

    phyval |= 18 << 16;  // Store phy register address in upper 16 bits again
    NutPhyCtl (PHY_SET_REGVAL, &phyval);

    /* Soft Reset LAN7810 */
    phyval = 1;
    NutPhyCtl(PHY_CTL_RESET, &phyval);
#endif

#ifndef PHY_MODE_RMII_DEPRECATED
    /* Clear MII isolate. */
    phyval = 0;
    NutPhyCtl(PHY_CTL_ISOLATE, &phyval);
#endif

    /* Restart autonegotiation */
    phyval = 1;
    NutPhyCtl(PHY_CTL_AUTONEG_RE, &phyval);

    /* Wait for auto negotiation completed and link established. */
    for (link_wait = tmo;; link_wait--) {
        phyval = 0;
        NutPhyCtl(PHY_GET_STATUS, &phyval);

        if((phyval & PHY_STATUS_HAS_LINK) && (phyval & PHY_STATUS_AUTONEG_OK)) {
            /* Check link state and configure EMAC accordingly */
            reg_ncfgr = inr(EMAC_NCFGR);
            if (phyval & PHY_STATUS_FULLDUPLEX) {
                reg_ncfgr |= EMAC_FD;
            } else {
                reg_ncfgr &= ~EMAC_FD;
            }

            if (phyval & PHY_STATUS_100M) {
                reg_ncfgr |= EMAC_SPD;
            } else {
                reg_ncfgr &= ~EMAC_SPD;
            }
            outr(EMAC_NCFGR, reg_ncfgr);

            break;
        }
        if (link_wait == 0) {
            EMPRINTF("NO LINK!\n");

            /* Return error on link timeout. */
            outr(EMAC_NCR, inr(EMAC_NCR) & ~EMAC_MPE);
            return -1;
        }
        NutSleep(10);
    }

    /* Disable management port. */
    outr(EMAC_NCR, inr(EMAC_NCR) & ~EMAC_MPE);

    EMPRINTF("EmacReset() DONE\n");

    return rc;
}

/*
 * NIC interrupt entry.
 */
static void EmacInterrupt(void *arg)
{
    unsigned int isr;
    EMACINFO *ni = (EMACINFO *) ((NUTDEVICE *) arg)->dev_dcb;

    /* Read interrupt status and disable interrupts. */
    isr = inr(EMAC_ISR);

    /* Receiver interrupt. */
    //if ((isr & EMAC_RCOMP) != 0 || (isr & EMAC_ROVR) != 0 || (inr(EMAC_RSR) & EMAC_REC) != 0) {
    if ((isr & (EMAC_RCOMP | EMAC_ROVR | EMAC_RXUBR)) != 0) {
        //outr(EMAC_RSR, EMAC_REC);
        outr(EMAC_IDR, EMAC_RCOMP | EMAC_ROVR | EMAC_RXUBR);
        NutEventPostFromIrq(&ni->ni_rx_rdy);
    }

    /* Transmitter interrupt. */
    if ((isr & EMAC_TCOMP) != 0 || (inr(EMAC_TSR) & EMAC_COMP) != 0) {
        //outr(EMAC_TSR, EMAC_COMP);
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
    while ((rxBufTab[rxBufIdx].addr & RXBUF_OWNERSHIP) != 0 && (rxBufTab[rxBufIdx].stat & RXS_SOF) == 0) {
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
        if (i != rxBufIdx && (rxBufTab[i].stat & RXS_SOF) != 0) {
            do {
                rxBufTab[rxBufIdx].addr &= ~(RXBUF_OWNERSHIP);
                rxBufIdx++;
                if (rxBufIdx >= EMAC_RX_BUFFERS) {
                    rxBufIdx = 0;
                }
            } while ((rxBufTab[rxBufIdx].addr & RXBUF_OWNERSHIP) != 0 && (rxBufTab[rxBufIdx].stat & RXS_SOF) == 0);
            break;
        }
        if ((fbc = rxBufTab[i].stat & RXS_LENGTH_FRAME) != 0) {
            break;
        }
        i++;
        if (i >= EMAC_RX_BUFFERS) {
            i = 0;
        }
    }

    if (fbc) {
        /*
         * Receiving long packets is unexpected. Let's declare the
         * chip insane. Short packets will be handled by the caller.
         */
        if (fbc > 1536) {
            ni->ni_insane = 1;
        } else {
            *nbp = NutNetBufAlloc(0, NBAF_DATALINK + 2, (uint16_t)fbc);
            if (*nbp != NULL) {
                uint8_t *bp = (uint8_t *) (* nbp)->nb_dl.vp;
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
static int EmacPutPacket(int bufnum, EMACINFO * ni, NETBUF * nb)
{
    int rc = -1;
    unsigned int sz;
    uint8_t *buf;

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

    /* Disable EMAC interrupts. */
    NutIrqDisable(&sig_EMAC);

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
        txBufTab[bufnum].stat = sz;
        outr(EMAC_NCR, inr(EMAC_NCR) | EMAC_TSTART);
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
static int EmacStart(const uint8_t * mac)
{
    unsigned int i;

    EMPRINTF("EmacStart(%02x:%02x:%02x:%02x:%02x:%02x)\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] );

    /* Set local MAC address. */
    outr(EMAC_SA1L, (mac[3] << 24) | (mac[2] << 16) | (mac[1] << 8) | mac[0]);
    outr(EMAC_SA1H, (mac[5] << 8) | mac[4]);

    /* Initialize receive buffer descriptors. */
    for (i = 0; i < EMAC_RX_BUFFERS - 1; i++) {
        rxBufTab[i].addr = (unsigned int) (&rxBuf[i * EMAC_RX_BUFSIZ]) & RXBUF_ADDRMASK;
    }
    rxBufTab[i].addr = ((unsigned int) (&rxBuf[i * EMAC_RX_BUFSIZ]) & RXBUF_ADDRMASK) | RXBUF_WRAP;
    outr(EMAC_RBQP, (unsigned int) rxBufTab);

    /* Initialize transmit buffer descriptors. */
    txBufTab[0].addr = (unsigned int) (&txBuf[0]);
    txBufTab[0].stat = TXS_USED;
    txBufTab[1].addr = (unsigned int) (&txBuf[EMAC_TX_BUFSIZ]);
    txBufTab[1].stat = TXS_USED | TXS_WRAP;
    outr(EMAC_TBQP, (unsigned int) txBufTab);

    /* Clear receiver status. */
    outr(EMAC_RSR, EMAC_OVR | EMAC_REC | EMAC_BNA);

    /* Discard FCS. */
    outr(EMAC_NCFGR, inr(EMAC_NCFGR) | EMAC_DRFCS);

    /* Enable receiver, transmitter and statistics. */
    outr(EMAC_NCR, inr(EMAC_NCR) | EMAC_TE | EMAC_RE | EMAC_WESTAT);

    EMPRINTF("EmacStart() DONE\n");

    return 0;
}

/*! \fn EmacRxThread(void *arg)
 * \brief NIC receiver thread.
 *
 */
THREAD(EmacRxThread, arg)
{
    NUTDEVICE *dev = arg;
    IFNET *ifn = (IFNET *) dev->dev_icb;
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;
    NETBUF *nb;

    EMPRINTF("EmacRxThread() INIT\n");

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
    EMPRINTF(" Call EmacStart()\n");

    while (EmacStart(ifn->if_mac)) {
        EmacReset(EMAC_LINK_LOOPS);
        NutSleep(1000);
    }

    /* Initialize the access mutex. */
    NutEventPost(&ni->ni_mutex);

    /* Run at high priority. */
    NutThreadSetPriority(9);

    /* Enable receive and transmit interrupts. */
    outr(EMAC_IER, EMAC_ROVR | EMAC_TCOMP | EMAC_TUND | EMAC_RXUBR | EMAC_RCOMP);
    NutIrqEnable(&sig_EMAC);

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
            inr(EMAC_ISR);
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
        outr(EMAC_IER, EMAC_ROVR | EMAC_RXUBR | EMAC_RCOMP);

        /* We got a weird chip, try to restart it. */
        while (ni->ni_insane) {
            EmacReset(EMAC_LINK_LOOPS);
            if (EmacStart(ifn->if_mac) == 0) {
                ni->ni_insane = 0;
                ni->ni_tx_queued = 0;
                ni->ni_tx_quelen = 0;
                NutIrqEnable(&sig_EMAC);
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
        if (NutEventWait(&ni->ni_mutex, mx_wait)) {
            break;
        }

        /* Check for packet queue space. */
        if ((txBufTab[txBufIdx].stat & TXS_USED) == 0) {
            if (NutEventWait(&ni->ni_tx_rdy, 500)) {
                /*
                 * We may have a timeout here because the last status change
                 * didn't trigger an interrupt. Reading the status register
                 * will clear the current status and the next change triggers
                 * an interrupt again, hopefully.
                 */
                inr(EMAC_ISR);
                if ((txBufTab[txBufIdx].stat & TXS_USED) == 0) {
                    /* No queue space. Release the lock and give up. */
                    txBufTab[txBufIdx].stat |= TXS_USED;
                    txBufIdx++;
                    txBufIdx &= 1;
                    NutEventPost(&ni->ni_mutex);
                    break;
                }
            }
        } else {
            if (inr(EMAC_TSR) & EMAC_UND) {
                txBufIdx = 0;
                outr(EMAC_TSR, EMAC_UND);
            }
            if (inr(EMAC_TSR) & EMAC_COMP) {
                outr(EMAC_TSR, EMAC_COMP);
            }

            if ((rc = EmacPutPacket(txBufIdx, ni, nb)) == 0) {
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

    EMPRINTF("EmacInit()\n");
    /* Reset the controller. */
    if (EmacReset(EMAC_LINK_LOOPS)) {
        if (EmacReset(EMAC_LINK_LOOPS)) {
            return -1;
        }
    }

    /* Clear EMACINFO structure. */
    memset(ni, 0, sizeof(EMACINFO));

    /* Register interrupt handler. */
    if (NutRegisterIrqHandler(&sig_EMAC, EmacInterrupt, dev)) {
        EMPRINTF(" IRQR CRASHED\n");
        return -1;
    }

    EMPRINTF(" IRQR OK\n");

    /* Start the receiver thread. */
    if (NutThreadCreate("emacrx", EmacRxThread, dev,
        (NUT_THREAD_NICRXSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD) == NULL) {
        EMPRINTF(" THREAD CRASHED\n");
        return -1;
    }

    EMPRINTF("EmacInit() DONE\n");
    return 0;
}

/*!
 * \brief Update the multicast hash.
 *
 * This function must be called after the multicast list changed.
 *
 * \param mclst Pointer to the first entry of the linked list of
 *              multicast entries.
 */
static void EmacHashUpdate(MCASTENTRY *mclst)
{
    int i;
    int j;
    int b;
    int idx;
    uint32_t hash[2] = { 0, 0 };

    /* Determine the hash bit for each entry. */
    while (mclst) {
        /* For each bit of the index. */
        for (idx = 0, i = 0; i < 6; i++) {
            /* Xor every 6th bit in the address. */
            for (b = 0, j = i; j < 48; j += 6) {
                b ^= (mclst->mca_ha[j >> 3] & (1 << (j & 0x07))) != 0;
            }
            idx |= b << i;
        }
        /* Set the bit given by the 6 bit index. */
        hash[idx > 31] |= 1 << (idx & 31);
        mclst = mclst->mca_next;
    }
    /* Set result in the hash register. */
    outr(EMAC_HRB, hash[0]);
    outr(EMAC_HRT, hash[1]);
    /* Enable or disable multicast hash. */
    if (hash[0] || hash[1]) {
        outr(EMAC_NCFGR, inr(EMAC_NCFGR) | EMAC_MTI);
    } else {
        outr(EMAC_NCFGR, inr(EMAC_NCFGR) & ~EMAC_MTI);
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
            EmacHashUpdate(mca);
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
    EmacHashUpdate(ifn->if_mcast);
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
 * \warning Timeout values are given in milliseconds and are limited to
 *          the granularity of the system timer.
 *
 * \note For ATmega103, only 8 data bits, 1 stop bit and no parity are allowed.
 *
 */
static int EmacIoCtl(NUTDEVICE * dev, int req, void *conf)
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
NUTDEVICE devAt91Emac = {
    0,                          /*!< \brief Pointer to next device. */
    {'e', 't', 'h', '0', 0, 0, 0, 0, 0},        /*!< \brief Unique device name. */
    IFTYP_NET,                  /*!< \brief Type of device. */
    0,                          /*!< \brief Base address. */
    0,                          /*!< \brief First interrupt number. */
    &ifn_eth0,                  /*!< \brief Interface control block. */
    &dcb_eth0,                  /*!< \brief Driver control block. */
    EmacInit,                   /*!< \brief Driver initialization routine. */
    EmacIoCtl,                  /*!< \brief Driver specific control function. */
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
