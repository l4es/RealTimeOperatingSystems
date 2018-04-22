#ifndef _PHY_H_
#define _PHY_H_

/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
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
 * $Id: phy_drv.c 3143 2010-09-29 20:13:51Z Astralix $
 * \endverbatim
 */

/* These are the PHYters OUIs.
 * The Organizationally Unique Identifier consists of a vendors model number
 * and a model revision number. By IEEE the two highest bits are ignored.
 */
/* Special tokens */
#define PHY_AUTO    0x00000000  /* Autodetect chip, not recommended */
#define PHY_ANY     0xFFFFFFFF  /* Disable PHY ID check, not recommended) */

/* AMD PHY tranceivers */       /* Boards using this chip / comment: */
#define AM79C875    0x00225540  /* nn. */

/* DAVICOM PHY tranceiver */
#define DM9161      0x0181B880  /* nn. */
#define DM9161A     0x0181B8A0  /* ATMEL EK, eNet-sam7X */
#define DM9161B     0x0181B8B0  /* nn. */
/* DAVICOM EMAC-PHY combinations */
#define DM9000      0x0181B8C0  /* EIR */
#define DM9000A     0x0181B8A0  /* nn. same as DM9161A */
#define DM9000B     0x0181B8B0  /* nn. same as DM9161B */

/* National Semiconductor PHY tranceiver */
#define DP83848     0x20005C90  /* EVK1100, EVK1105, STM3210C-EVAL, STM3220G-EVAL */
#define DP83849     0x20005CA0

/* Micrel PHY tranceiver */
#define KS8721      0x00221610  /* Olimex SAM7-EX256 */
/* Micrel EMAC-PHY combinations */
#define KS8851      0x00008870  /* Located in CIDER (0xC0) register */

/* ST Microelectronics PHY tranceiver */
#define STE100P     0x1C040010  /* Hitex STM32-comStick */
#define ST802RT1    0x02038460  /* STEVAL_PCC0101V2 */

/* SMSC PHY tranceiver */
#define LAN8700     0x0007C0C0  /* Not recommended for new designs! */
#define LAN8700r4   0x0007C0C0  /*   revision 4 of the LAN8700 phy */
#define LAN8710     0x0007C0F0  /* nn. / LAN8710 and LAN8720 share same IDs */
#define LAN8720     0x0007C0F0  /* nn. / not a typo, has same OUI as 8710 */
#define LAN8720A    0x0007C0F0  /*   revision 1 of the LAN8720 phy */
#define LAN8742A    0x0007C130  /* Revision 0 of the LAN8742A phy */

/* Masks for chip detection */
#define OUIMSK_VEN  0x3FFFC000  /* Mask for manufacturers OUI */
#define OUIMSK_DEV  0x00003FF0  /* Mask for manufacturers model number */
#define OUIMSK_REV  0x0000000F  /* Mask for chip revision number */

/* Include nutconf options here as the tokens above need to be defined before */

#include <cfg/phycfg.h>

#define PHY_STATUS_HAS_LINK     0x00000001
#define PHY_STATUS_10M          0x00000002
#define PHY_STATUS_100M         0x00000004
#define PHY_STATUS_1000M        0x00000008
#define PHY_STATUS_FULLDUPLEX   0x00000010
#define PHY_STATUS_AUTONEG_OK   0x00000020
#define PHY_STATUS_CON_UNKNOWN  0x00000040

/* PHY ioctl() Control Tokens */    /* Accepted Values */
#define PHY_CTL_RESET       0x0001  /* 1: Activate reset, wait for completion */
#define PHY_CTL_LOOPBACK    0x0002  /* 1: Enable, 0: disable loopback */
#define PHY_CTL_SPEED       0x0003  /* 10/100: Set interface speed */
#define PHY_CTL_AUTONEG     0x0004  /* 1: Enable, 0: disable Auto-Negotiation */
#define PHY_CTL_POWERDOWN   0x0005  /* 1: Power down, 0: wakeup chip */
#define PHY_CTL_ISOLATE     0x0006  /* 1: Isolate interface, 0: clear isolate state */
#define PHY_CTL_DUPLEX      0x0007  /* 1: Enable full duplex */
#define PHY_CTL_AUTONEG_RE  0x0008  /* 1: Restart autonegotiation process */

#define PHY_GET_LINK        0x0100  /* Request Link Status, 1: link is up */
#define PHY_GET_STATUS      0x0101  /* Request connection status
                                     *   PHY_STATUS_HAS_LINK   is set when link is up
                                     *   PHY_STATUS_10M        is set when speed is 10Mbit/s
                                     *   PHY_STATUS_100M       is set when speed is 100Mbit/s
                                     *   PHY_STATUS_1000M      is set when speed is 1000Mbit/s
                                     *   PHY_STATUS_FULLDUPLEX is set when full duplex is true
                                     *   PHY_STATUS_AUTONEG_OK is set when auto negotiation is finished
                                     *   PHY_STATUS_CON_UNKNWN is set when speed and duplex is unknown
                                     * The value is 0 when link is not established
                                     * The value is negative on error condition */
#define PHY_GET_POE         0x0102  /* Request PoE status, 1: energy is detected */
#define PHY_GET_REGVAL      0x0103  /* Read value of register (par>>16) from the phy */
#define PHY_SET_REGVAL      0x0104  /* Write value (par & 0xFFFF) to register (par>>16) of the phy */

/*
 * Physical Layer Tranceiver - Device Control Block
 *
 */
typedef struct _PHYDCB PHYDCB;

struct _PHYDCB {
    /*
     * Vendor OUI ( including model and revision )
     */
    uint32_t oui;

    /*
     * Address of chip at MDIO bus
     */
    uint8_t  addr;

    /*
     * Functionpointer filled by EMAC driver to access
     * PHYter for write.
     */
    void(*mdiow)(uint8_t, uint16_t);

    /*
     * Functionpointer filled by EMAC driver to access
     * PHYter for read.
     */
    uint16_t(*mdior)(uint8_t);
};

extern uint16_t NutPhyGetStatus(void);
extern int NutPhyCtl( uint16_t ctl, uint32_t *par);
extern int NutRegisterPhy( uint8_t mda, void(*mdiow)(uint8_t, uint16_t), uint16_t(*mdior)(uint8_t));

#endif /* _PHY_H_ */
