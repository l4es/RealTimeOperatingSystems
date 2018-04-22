/*
 * Copyright (C) 2001-2006 by egnite Software GmbH
 * Copyright (C) 2009 by egnite GmbH
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
 * $Id: smsc.c 4115 2012-04-12 21:06:13Z olereinhardt $
 *
 * WARNING! Do not use any part of Basemon for your own applications. WARNING!
 *
 * This is not a typical application sample. It overrides parts of Nut/OS to
 * keep it running on broken hardware.
 */

#include <stdio.h>
#include <sys/timer.h>

#include "utils.h"
#include "uart.h"
#include "smscregs.h"
#include "smsc.h"


/*!
 * \brief Select specified PHY register for reading or writing.
 *
 * \param reg PHY register number.
 * \param we  Should be 1 for write access, 0 for read access.
 *
 * \return Contents of the PHY interface rgister.
 */
static uint8_t NicPhyRegSelect(uint8_t reg, uint8_t we)
{
    uint8_t rs;
    uint8_t msk;
    uint8_t i;

    nic_bs(3);
    rs = (nic_inlb(NIC_MGMT) & ~(MGMT_MCLK | MGMT_MDO)) | MGMT_MDOE;

    /* Send idle pattern. */
    for (i = 0; i < 33; i++) {
        nic_outlb(NIC_MGMT, rs | MGMT_MDO);
        nic_outlb(NIC_MGMT, rs | MGMT_MDO | MGMT_MCLK);
    }

    /* Send start sequence. */
    nic_outlb(NIC_MGMT, rs);
    nic_outlb(NIC_MGMT, rs | MGMT_MCLK);
    nic_outlb(NIC_MGMT, rs | MGMT_MDO);
    nic_outlb(NIC_MGMT, rs | MGMT_MDO | MGMT_MCLK);

    /* Write or read mode. */
    if (we) {
        nic_outlb(NIC_MGMT, rs);
        nic_outlb(NIC_MGMT, rs | MGMT_MCLK);
        nic_outlb(NIC_MGMT, rs | MGMT_MDO);
        nic_outlb(NIC_MGMT, rs | MGMT_MDO | MGMT_MCLK);
    } else {
        nic_outlb(NIC_MGMT, rs | MGMT_MDO);
        nic_outlb(NIC_MGMT, rs | MGMT_MDO | MGMT_MCLK);
        nic_outlb(NIC_MGMT, rs);
        nic_outlb(NIC_MGMT, rs | MGMT_MCLK);
    }

    /* Send PHY address. Zero is used for the internal PHY. */
    for (i = 0; i < 5; i++) {
        nic_outlb(NIC_MGMT, rs);
        nic_outlb(NIC_MGMT, rs | MGMT_MCLK);
    }

    /* Send PHY register number. */
    for (msk = 0x10; msk; msk >>= 1) {
        if (reg & msk) {
            nic_outlb(NIC_MGMT, rs | MGMT_MDO);
            nic_outlb(NIC_MGMT, rs | MGMT_MDO | MGMT_MCLK);
        } else {
            nic_outlb(NIC_MGMT, rs);
            nic_outlb(NIC_MGMT, rs | MGMT_MCLK);
        }
    }
    nic_outlb(NIC_MGMT, rs);

    return rs;
}

/*!
 * \brief Read contents of PHY register.
 *
 * \param reg PHY register number.
 *
 * \return Contents of the specified register.
 */
static uint16_t NicPhyRead(uint8_t reg)
{
    uint16_t rc = 0;
    uint8_t rs;
    uint8_t i;

    /* Select register for reading. */
    rs = NicPhyRegSelect(reg, 0);

    /* Switch data direction. */
    rs &= ~MGMT_MDOE;
    nic_outlb(NIC_MGMT, rs);
    nic_outlb(NIC_MGMT, rs | MGMT_MCLK);

    /* Clock data in. */
    for (i = 0; i < 16; i++) {
        nic_outlb(NIC_MGMT, rs);
        nic_outlb(NIC_MGMT, rs | MGMT_MCLK);
        rc <<= 1;
        rc |= (nic_inlb(NIC_MGMT) & MGMT_MDI) != 0;
    }

    /* This will set the clock line to low. */
    nic_outlb(NIC_MGMT, rs);

    return rc;
}

/*!
 * \brief Write value to PHY register.
 *
 * \param reg PHY register number.
 * \param val Value to write.
 */
static void NicPhyWrite(uint8_t reg, uint16_t val)
{
    uint16_t msk;
    uint8_t rs;

    /* Select register for writing. */
    rs = NicPhyRegSelect(reg, 1);

    /* Switch data direction dummy. */
    nic_outlb(NIC_MGMT, rs | MGMT_MDO);
    nic_outlb(NIC_MGMT, rs | MGMT_MDO | MGMT_MCLK);
    nic_outlb(NIC_MGMT, rs);
    nic_outlb(NIC_MGMT, rs | MGMT_MCLK);

    /* Clock data out. */
    for (msk = 0x8000; msk; msk >>= 1) {
        if (val & msk) {
            nic_outlb(NIC_MGMT, rs | MGMT_MDO);
            nic_outlb(NIC_MGMT, rs | MGMT_MDO | MGMT_MCLK);
        } else {
            nic_outlb(NIC_MGMT, rs);
            nic_outlb(NIC_MGMT, rs | MGMT_MCLK);
        }
    }

    /* Set clock line low and output line int z-state. */
    nic_outlb(NIC_MGMT, rs & ~MGMT_MDOE);
}

/*!
 * \brief Configure the internal PHY.
 *
 * Reset the PHY and initiate auto-negotiation.
 */
static int NicPhyConfig(void)
{
    uint16_t phy_sor;
    uint16_t phy_sr;
    uint16_t phy_to;
    uint16_t mode;

    /*
     * Reset the PHY and wait until this self clearing bit
     * becomes zero. We sleep 63 ms before each poll and
     * give up after 3 retries.
     */
    NicPhyWrite(NIC_PHYCR, PHYCR_RST);
    for (phy_to = 0;; phy_to++) {
        Delay(100000);
        if ((NicPhyRead(NIC_PHYCR) & PHYCR_RST) == 0)
            break;
        if (phy_to > 3) {
            return -1;
        }
    }
    Delay(200000);

    /* Store PHY status output. */
    phy_sor = NicPhyRead(NIC_PHYSOR);

    /* Enable PHY interrupts. */
    NicPhyWrite(NIC_PHYMSK, PHYMSK_MLOSSSYN | PHYMSK_MCWRD | PHYMSK_MSSD |
                PHYMSK_MESD | PHYMSK_MRPOL | PHYMSK_MJAB | PHYMSK_MSPDDT | PHYMSK_MDPLDT);

    /* Set RPC register. */
    mode = RPCR_ANEG | RPCR_LEDA_PAT | RPCR_LEDB_PAT;
    nic_bs(0);
    nic_outw(NIC_RPCR, mode);

    /*
     * Advertise our capabilities, initiate auto negotiation
     * and wait until this has been completed.
     */
    NicPhyWrite(NIC_PHYANAD, PHYANAD_TX_FDX | PHYANAD_TX_HDX | PHYANAD_10FDX | PHYANAD_10_HDX | PHYANAD_CSMA);
    for (phy_to = 0, phy_sr = 0;; phy_to++) {
        /* Give up after long time wait. */
        if (phy_to >= 32) {
            return -1;
        }
        /* Restart auto negotiation every 4 seconds or on failures. */
        if ((phy_to & 127) == 0 /* || (phy_sr & PHYSR_REM_FLT) != 0 */) {
            NicPhyWrite(NIC_PHYCR, PHYCR_ANEG_EN | PHYCR_ANEG_RST);
            Delay(200000);
        }
        /* Check if link status detected. */
        phy_sr = NicPhyRead(NIC_PHYSR);
        if (phy_sr & PHYSR_ANEG_ACK)
            break;
        Delay(300000);
    }
    return 0;
}

/*!
 * \brief Wait until MMU is ready.
 *
 * Poll the MMU command register until \ref MMUCR_BUSY
 * is cleared.
 *
 * \param tmo Timeout in milliseconds.
 *
 * \return 0 on success or -1 on timeout.
 */
static INLINE int NicMmuWait(uint16_t tmo)
{
    while (tmo--) {
        if ((nic_inlb(NIC_MMUCR) & MMUCR_BUSY) == 0)
            break;
        Delay(2000);
    }
    return tmo ? 0 : -1;
}

/*!
 * \brief Reset the Ethernet controller.
 *
 * \return 0 on success, -1 otherwise.
 */
static int NicReset(void)
{
    /* Disable all interrupts. */
    nic_outlb(NIC_MSK, 0);

    /* MAC and PHY software reset. */
    nic_bs(0);
    nic_outw(NIC_RCR, RCR_SOFT_RST);

    /* Enable Ethernet protocol handler. */
    nic_bs(1);
    nic_outw(NIC_CR, CR_EPH_EN);

    Delay(20000);

    /* Disable transmit and receive. */
    nic_bs(0);
    nic_outw(NIC_RCR, 0);
    nic_outw(NIC_TCR, 0);

    /* Enable auto release. */
    nic_bs(1);
    nic_outw(NIC_CTR, CTR_AUTO_RELEASE);

    /* Reset MMU. */
    nic_bs(2);
    nic_outlb(NIC_MMUCR, MMU_RST);
    if (NicMmuWait(1000))
        return -1;

    return 0;
}


int SmscDetect(void)
{
    uint8_t bv;

    /* High byte of base select is always 0x33. */
    if((bv = nic_inhb(NIC_BSR)) != 0x33) {
        return -1;
    }

    /* Read revision. */
    nic_bs(3);
    if(((bv = nic_inlb(NIC_REV)) & 0xF0) != 0x90) {
        return -1;
    }
    return 0;
}

/*!
 * \brief Test NIC interrupt line.
 */
static int SmscTestInterrupt(void)
{
    uint8_t tmo;

#if defined (__AVR__)
    /*
     * Set PE5 to input. This is our interrupt signal line.
     */
    cbi(DDRE, 5);

    if (NicReset()) {
        puts("reset failed");
        return -1;
    }

    if(bit_is_set(PINE, 5)) {
        puts("IRQ stuck");
        return -1;
    }
#endif

    /* Enable receiver. */
    nic_bs(3);
    nic_outlb(NIC_ERCV, 7);
    nic_bs(0);
    nic_outw(NIC_RCR, RCR_RXEN);

    /* Enable transmitter and padding. */
    nic_outw(NIC_TCR, TCR_PAD_EN | TCR_TXENA);

    /* Configure the PHY. */
    if (NicPhyConfig()) {
        puts("link failed");
        return -1;
    }

    /* Allocate packet buffer space. */
    nic_bs(2);
    nic_outlb(NIC_MMUCR, MMU_ALO);
    if (NicMmuWait(100))
        return -1;

    /* Enable interrupts including allocation success. */
    nic_outlb(NIC_MSK, INT_ALLOC);

    /* Wait for allocation success. */
    tmo = 255;
    while ((nic_inlb(NIC_IST) & INT_ALLOC) == 0) {
        if(--tmo == 0) {
            puts("IRQ failed");
            return -1;
        }
        Delay(2000);
    }
#if defined (__AVR__)
    if(bit_is_clear(PINE, 5)) {
        puts("no IRQ");
        return -1;
    }
#endif
    return 0;
}

/*!
 * \brief Test NIC RAM buffer.
 */
static int SmscTestBuffer(void)
{
    ureg_t i;
    ureg_t tmo;
    uint16_t cnt;
    uint16_t val;

    /* Disable interrupts. */
    if (NicReset()) {
        puts("reset failed");
        return -1;
    }

    for(i = 0; i < 3; i++) {
        /* Allocate packet buffer space. */
        nic_bs(2);
        nic_outlb(NIC_MMUCR, MMU_ALO);
        if (NicMmuWait(100)) {
            printf("Alloc failed\n");
            return -1;
        }

        /* Wait for allocation success. */
        tmo = 255;
        while ((nic_inlb(NIC_IST) & INT_ALLOC) == 0) {
            if(--tmo == 0) {
                puts("IRQ failed");
                return -1;
            }
            Delay(2000);
        }

        /*
         * Transfer the allocated packet number to TX packet number.
         */
        nic_outlb(NIC_PNR, nic_inhb(NIC_PNR));

        /*
         * Reset the pointer register, no auto increment.
         */
        nic_outw(NIC_PTR, 0);

        /*
         * Data write.
         */
#ifdef HEARTBEAT_BIT
        HeartBeat();
#endif
        for(cnt = 0; cnt < 1024; cnt++) {
            val = ~cnt;
            nic_outw(NIC_PTR, cnt * 2);
            nic_outlb(NIC_DATA, (uint8_t)val);
            nic_outw(NIC_PTR, cnt * 2 + 1);
            nic_outlb(NIC_DATA, val >> 8);
        }

        /*
         * Data read.
         */
#ifdef HEARTBEAT_BIT
        HeartBeat();
#endif
        for(cnt = 0; cnt < 1024; cnt++) {
            nic_outw(NIC_PTR, PTR_READ | (cnt * 2));
            if((val = ~nic_inw(NIC_DATA)) != cnt) {
                printf("bad val %04X at %04X\n", val, cnt);
                return -1;
            }
        }
    }
    return 0;
}

/*!
 * \brief Test NIC functions.
 */
int SmscTest(void)
{
    if(SmscTestInterrupt()) {
        puts("IRQ failed");
        return -1;
    }
    if(SmscTestBuffer()) {
        puts("Buf failed");
        return -1;
    }
    puts("OK");

    return 0;
}

/*
 * \brief Continously send baroadcasts.
 */
void SmscSend(void)
{
    uint8_t mac[] = { 0x00, 0x06, 0x98, 0x00, 0x00, 0x00 };
    ureg_t i;
    uint16_t sz;
    uint32_t cnt = 0;

    printf("\nInit controller...");
    if (NicReset()) {
        puts("reset failed");
        return;
    }

    /* Enable receiver. */
    nic_bs(3);
    nic_outlb(NIC_ERCV, 7);
    nic_bs(0);
    //nic_outw(NIC_RCR, RCR_RXEN);

    /* Enable transmitter and padding. */
    nic_outw(NIC_TCR, TCR_PAD_EN | TCR_TXENA);

    /* Configure the PHY. */
    if (NicPhyConfig()) {
        puts("link failed");
        return;
    }

    /* Set MAC address. */
    nic_bs(1);
    for (i = 0; i < 6; i++)
        nic_outlb(NIC_IAR + i, mac[i]);

    puts("OK");

    for(cnt = 0;; cnt++) {
        Delay(500000);
        printf("\r%lu", cnt);
        sz = 1500;

        /* Allocate packet buffer space. */
        nic_bs(2);
        nic_outlb(NIC_MMUCR, MMU_ALO);
        if (NicMmuWait(100)) {
            puts("Alloc");
            break;
        }

        /* Enable allocation interrupt. */
        nic_outlb(NIC_MSK, INT_ALLOC);

        /*
         * Wait for allocation success. This fails quite often, possibly
         * because we do not clear our receive buffer.
         */
        if ((nic_inlb(NIC_IST) & INT_ALLOC) == 0) {
            puts(" Alloc");
            Delay(500000);
            if ((nic_inlb(NIC_IST) & INT_ALLOC) == 0) {
                nic_outlb(NIC_MMUCR, MMU_RST);
                NicMmuWait(1000);
                nic_outlb(NIC_MMUCR, MMU_ALO);
                if (NicMmuWait(100) || (nic_inlb(NIC_IST) & INT_ALLOC) == 0) {
                    break;
                }
            }
        }

        /* Disable interrupts. */
        nic_outlb(NIC_MSK, 0);


        nic_outlb(NIC_PNR, nic_inhb(NIC_PNR));

        nic_outw(NIC_PTR, 0x4000);

        /* Transfer control word. */
        nic_outlb(NIC_DATA, 0);
        nic_outlb(NIC_DATA, 0);

        /* Transfer the byte count. */
        nic_outw(NIC_DATA, sz);

        /* Transfer the Ethernet frame. */
        for (i = 0; i < 6; i++) {
            nic_outlb(NIC_DATA, 0xFF);
        }
        for (i = 0; i < 6; i++) {
            nic_outlb(NIC_DATA, mac[i]);
        }

        nic_outlb(NIC_DATA, 0x08);
        nic_outlb(NIC_DATA, 0x00);

        /*
         * Add pad bytes.
         */
        while (sz--) {
            nic_outlb(NIC_DATA, 0x00);
        }

        /* Transfer the control word. */
        nic_outlb(NIC_DATA, 0);
        nic_outlb(NIC_DATA, 0);

        /* Enqueue packet. */
        if (NicMmuWait(100)) {
            puts("Enqueue");
            break;
        }
        nic_outlb(NIC_MMUCR, MMU_ENQ);


        if (GetChar())
            break;

    }
}

void SmscLoop(void)
{
    printf_P(presskey_P);
    for (;;) {
        nic_bs(3);
        printf("\rrev=0x%02X        ", nic_inlb(NIC_REV));
        if (GetChar()) {
            puts("");
            return;
        }
    }
}

