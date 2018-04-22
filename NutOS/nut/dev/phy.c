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
#include <cfg/os.h>

#include <stdint.h>
#include <string.h>

#include <sys/heap.h>
#include <sys/timer.h>

#include <dev/phy.h>


/* WARNING: Variadic macros are C99 and may fail with C89 compilers. */
#ifdef NUTDEBUG
#include <stdio.h>
#define PHPRINTF(args,...) printf(args,##__VA_ARGS__); fflush(stdout);
#else
#define PHPRINTF(args,...)
#endif

/*
 * Basic Set Register Map:
 * This register map is valid for all manufactureres and chip.
 * Though some bits might not be supported.
 */
#define PHY_BMCR        0x00    /* Basic Mode Control Register */
#define PHY_BMSR        0x01    /* Basic Mode Status Register */
#define PHY_ID1         0x02    /* OUI High Register */
#define PHY_ID2         0x03    /* OUI Low Register */
#define PHY_ANAR        0x04    /* Auto-Negotiation Ability Register */
#define PHY_ANLP        0x05    /* Auto-Negotiation Link Partner Advertisement Register */
#define PHY_ANER        0x06    /* Auto-Negotiation Expansion Register */
#define PHY_ANTR        0x07    /* Auto-Negotiation Next Page TX Register */

/*
 * Basic Mode Control Register Options
 */
#define PHY_BMCR_RES    0x8000  /* 1: Reset PHY, flips to 0 if reset accomplished. */
#define PHY_BMCR_LOOP   0x4000  /* 1: Enable Loopback Mode. */
#define PHY_BMCR_SPEED  0x2000  /* 1: Manual Speed 100Mbit, 0: 10MBit). */
#define PHY_BMCR_ANEG   0x1000  /* 1: Enable Auto-Negotiation. */
#define PHY_BMCR_PDWN   0x0800  /* 1: Power Down Enabled. */
#define PHY_BMCR_ISO    0x0400  /* 1: Isolate PHY from MII interface, only MDIO is still available. */
#define PHY_BMCR_ANST   0x0200  /* 1: Restart Auto-Negotiation, flips to 0 if successfull. */
#define PHY_BMCR_DUPX   0x0100  /* 1: Enable Full Duplex Operation. */
#define PHYSET_COLTEST  0x0080  /* 1: Collision test enabled. */

/*
 * Basic Mode Status Register Options
 */
/* Chip capabilities */         /* (PR/O = fixed value, R/O read only */
#define PHY_BMSR_CT4    0x8000  /* PR/O: Chip supports 100BASE-T4 mode */
#define PHY_BMSR_C100FD 0x4000  /* PR/O: Chip supports 100BASE-TX Full Duplex mode */
#define PHY_BMSR_C100HD 0x2000  /* PR/O: Chip supports 100BASE-TX Half Duplex mode */
#define PHY_BMSR_C10FD  0x1000  /* PR/O: Chip supports 10BASE-TX Full Duplex mode */
#define PHY_BMSR_C10HD  0x0800  /* PR/O: Chip supports 10BASE-TX Half Duplex mode */
#define PHY_BMSR_CPRE   0x0040  /* PR/O: Chip supports MDIO preamble supression mode */
#define PHY_BMSR_CANEG  0x0008  /* PR/O: Chip supports Auto-Negotiation */
#define PHY_BMSR_CEXT   0x0001  /* PR/O: Chip supports extended MDIO registers */
#define PHY_BMSR_CMSK   0xF849  /* Mask for chip capabilities */

/* Chip status */
#define PHY_BMSR_ANEG   0x0020  /* R/O: 1: Auto-Negotiation complete */
#define PHY_BMSR_RFLT   0x0010  /* R/O: 1: Remote Fault (Partner cut us off) */
#define PHY_BMSR_LNK    0x0004  /* R/O: 1: Link established */
#define PHY_BMSR_JAB    0x0002  /* R/O: 1: Jabber condition detected */
#define PHY_BMSR_SMSK   0x0036  /* Mask for chip status */


#define phyw( reg, val) phydcb->mdiow( reg, val)
#define phyr( reg) phydcb->mdior( reg)

PHYDCB *phydcb = NULL;

enum {
    PHY_BIT_DESCR_10M = 0,
    PHY_BIT_DESCR_100M,
    PHY_BIT_DESCR_1000M,
    PHY_BIT_DESCR_DUPLX,
    PHY_BIT_DESCR_POE,
    PHY_BIT_DESCR_MAX
} phy_bit_descr_nr;

typedef struct {
    uint8_t reg;                /* register number */
    uint16_t mask;              /* bit mask to identify what we are looking */
} phy_bit_descr_t;

typedef struct {
    uint32_t phy_oui;           /* oui chip identifier */
    phy_bit_descr_t phy_bit_descr[PHY_BIT_DESCR_MAX];
                                /* list of descriptors to identify where to get
                                 *   [0] flag about 10Mbit/s speed
                                 *   [1] flag about 100Mbit/s speed
                                 *   [2] flag about 1000Mbit/s speed
                                 *   [3] flag about full duplex
                                 *   [4] flag about POE status */
} phy_status_descr_t;

phy_status_descr_t phy_status_descr[] = {
    /* Davicom DM9000 derivates */
    { DM9000,    { {17, 0x3000}, {17, 0xC000}, {0, 0}, {17, 0xA000}, {0, 0} } },
    { DM9000A,   { {17, 0x3000}, {17, 0xC000}, {0, 0}, {17, 0xA000}, {0, 0} } },
    { DM9000B,   { {17, 0x3000}, {17, 0xC000}, {0, 0}, {17, 0xA000}, {0, 0} } },

    /* Davicom DM9161 derivates */
    { DM9161,    { {17, 0x3000}, {17, 0xC000}, {0, 0}, {17, 0xA000}, {0, 0} } },
/*  DM9161A is the same as DM9000A and DM9161B is the same as DM9000B
    { DM9161A,   { {17, 0x3000}, {17, 0xC000}, {0, 0}, {17, 0xA000}, {0, 0} } },
    { DM9161B,   { {17, 0x3000}, {17, 0xC000}, {0, 0}, {17, 0xA000}, {0, 0} } },
*/
    /* SMSC LAN8700 derivates */
    { LAN8700,   { {31, 0x0004}, {31, 0x0008}, {0, 0}, {31, 0x0010}, {0, 0} } },
    { LAN8700r4, { {31, 0x0004}, {31, 0x0008}, {0, 0}, {31, 0x0010}, {0, 0} } },
    { LAN8710,   { {31, 0x0004}, {31, 0x0008}, {0, 0}, {31, 0x0010}, {0, 0} } },
    { LAN8720A,  { {31, 0x0004}, {31, 0x0008}, {0, 0}, {31, 0x0010}, {0, 0} } },
    { LAN8742A,  { {31, 0x0004}, {31, 0x0008}, {0, 0}, {31, 0x0010}, {0, 0} } },

    /* Micrel KS8721 */
    { KS8721,    { {31, 0x0004}, {31, 0x0008}, {0, 0}, {31, 0x0010}, {0, 0} } },

    /* STM */
    { ST802RT1,  { {0,0}       , {17, 0x0200}, {0, 0}, {17, 0x0100}, {0, 0} } },

    /* NatSemi/TI DP83848 */
    // Table approach does not work as 10BASET and 100BASET
    // status share the same bit. Special treatment in NutPhyCtl required.
    { DP83848,   { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} } },
};

/*!
 * \brief Read PHY's Basic Mode Status Register
 *
 * \param  None.
 *
 * \return 0 on fail or status register of PHY.
 */
#define NutPhyGetStatus(void) phyr( PHY_BMSR))

/*!
 * \brief Control PHY Options
 *
 * This function controls the physical layer chip.
 * Give
 *
 * \param  ctl is the PHY option to control or test.
 * \param  par Pointer to value of 1 to set, 0 to reset an option
 *         or the return of a value test.
 * \return 0 on success, -1 on failure.
 */

int NutPhyCtl( uint16_t ctl, uint32_t *par)
{
    int rc = 0;
    uint16_t bmcr=0, bmsr=0;
    uint16_t p16 = (uint16_t)*par;
    uint8_t  reg = (uint8_t)(*par>>16);

    if (phydcb == NULL) {
        /* Return with an error, if the phy has not yet been registered */
        return -1;
    }

    PHPRINTF("NPCtl(0x%x, 0x%04x)\n", ctl, p16);

    /* Execute standard ioctl function */
    bmcr = phyr( PHY_BMCR);

    PHPRINTF("  bmcr=0x%04x\n", bmcr);

    switch (ctl)
    {
        case PHY_CTL_RESET:
            if (p16) {
                int wait = 0;

                /* Set Reset bit in BMCR register */
                phyw( PHY_BMCR, PHY_BMCR_RES);

                /* Wait till reset bit flips back to 0 */
                while( (phyr( PHY_BMCR) & PHY_BMCR_RES) && wait<10) {
                    NutDelay(100);
                    wait++;
                }
                if(wait >= 10) {
                    rc = -1;
                }
            }
            break;

        case PHY_CTL_LOOPBACK:
            if (p16) {
                bmcr |= PHY_BMCR_LOOP;
            } else {
                bmcr &= ~PHY_BMCR_LOOP;
            }
            phyw( PHY_BMCR, bmcr);
            break;

        case PHY_CTL_SPEED:
            if (p16 == 100) {
                bmcr |= PHY_BMCR_SPEED;
                phyw( PHY_BMCR, bmcr);
            } else
            if (p16==10) {
                bmcr &= ~PHY_BMCR_SPEED;
                phyw( PHY_BMCR, bmcr);
            } else {
                rc = -1;
            }
            break;

        case PHY_CTL_AUTONEG:
            if (p16) {
                bmcr |= PHY_BMCR_ANEG;
            } else {
                bmcr &= ~PHY_BMCR_ANEG;
            }
            phyw( PHY_BMCR, bmcr);
            break;

        case PHY_CTL_POWERDOWN:
            if (p16) {
                bmcr |= PHY_BMCR_PDWN;
            } else {
                bmcr &= ~PHY_BMCR_PDWN;
            }
            phyw( PHY_BMCR, bmcr);
            break;

        case PHY_CTL_ISOLATE:
            if (p16) {
                bmcr |= PHY_BMCR_ISO;
            } else {
                bmcr &= ~PHY_BMCR_ISO;
            }
            phyw( PHY_BMCR, bmcr);
            break;

        case PHY_CTL_DUPLEX:
            if (p16) {
                bmcr |= PHY_BMCR_DUPX;
            } else {
                bmcr &= ~PHY_BMCR_DUPX;
            }
            phyw( PHY_BMCR, bmcr);
            break;

        case PHY_CTL_AUTONEG_RE:
            if (p16) {
                bmcr |= PHY_BMCR_ANST;
                phyw (PHY_BMCR, bmcr);
            }
            break;

        case PHY_GET_LINK:
            *par = (uint32_t)(phyr(PHY_BMSR)&PHY_BMSR_LNK);
            break;

        case PHY_GET_STATUS:
            bmsr = phyr(PHY_BMSR);
            /* only return a value different to zero if link is true */
            if (bmsr & PHY_BMSR_LNK) {
                int count, length;

                *par =  PHY_STATUS_HAS_LINK |
                        ((bmsr & PHY_BMSR_ANEG) ? PHY_STATUS_AUTONEG_OK : 0);

/*              KS8721 needs different interpretation of bits */
                if (phydcb->oui == KS8721)
                {
                    uint16_t tempreg;
                    tempreg = phyr(0x1f);
                    tempreg >>= 2;
                    tempreg &= 0x7;
                    switch (tempreg) {
                    case 1: *par |= PHY_STATUS_10M; break;
                    case 2: *par |= PHY_STATUS_100M; break;
                    case 5: *par |= PHY_STATUS_10M | PHY_STATUS_FULLDUPLEX; break;
                    case 6: *par |= PHY_STATUS_100M | PHY_STATUS_FULLDUPLEX; break;
                    }
                }
                /* Same for the DP83848, table approach does not work as
                   10BASET and 100BASET status share the same bit */
                else if (phydcb->oui == DP83848)
                {
                   uint16_t tempreg;
                   tempreg = phyr(0x10);
                   tempreg &= 0x7;
                   switch (tempreg) {
                   case 1: *par |= PHY_STATUS_100M; break;
                   case 3: *par |= PHY_STATUS_10M; break;
                   case 5: *par |= PHY_STATUS_100M | PHY_STATUS_FULLDUPLEX; break;
                   case 7: *par |= PHY_STATUS_10M | PHY_STATUS_FULLDUPLEX; break;
                   }
                }
                else
                {
                    length = sizeof(phy_status_descr) / sizeof(phy_status_descr[0]);
                    for(count=0; count<length; count++) {
                        if(phy_status_descr[count].phy_oui == phydcb->oui) {
                            break;
                        }
                    }

                    if(count<length) {
                        uint16_t tempreg;

                        /* entry in table found */
                        PHPRINTF("  Reading status of known phy\n");

                        tempreg = phyr(phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_10M].reg);
                        if(tempreg & phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_10M].mask) {
                            *par |= PHY_STATUS_10M;
                        }
                        tempreg = phyr(phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_100M].reg);
                        if(tempreg & phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_100M].mask) {
                            *par |= PHY_STATUS_100M;
                        }
                        tempreg = phyr(phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_1000M].reg);
                        if(tempreg & phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_1000M].mask) {
                            *par |= PHY_STATUS_1000M;
                        }
                        tempreg = phyr(phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_DUPLX].reg);
                        if(tempreg & phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_DUPLX].mask) {
                            *par |= PHY_STATUS_FULLDUPLEX;
                        }
                    }
                    else {
                        *par |= PHY_STATUS_CON_UNKNOWN;
                    }
                }
            }
            else {
                *par = 0;
            }
            break;

        case PHY_GET_POE: {
            int count, length;

            *par = 0;
            length = sizeof(phy_status_descr) / sizeof(phy_status_descr[0]);
            for(count=0; count<length; count++) {
                if(phy_status_descr[count].phy_oui == phydcb->oui) {
                    break;
                }
            }
            if(count<length) {
                uint16_t tempreg;

                /* entry in table found */
                PHPRINTF("  Reading POE status of known phy\n");

                tempreg = phyr(phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_POE].reg);
                if(tempreg & phy_status_descr[count].phy_bit_descr[PHY_BIT_DESCR_POE].mask) {
                    *par = 1;
                }
            }
            else {
                rc = -1;
            }
            break;
        } /* endof case PHY_GET_POE */

        case PHY_GET_REGVAL:
            *par = (uint32_t)phyr(reg);
            break;

        case PHY_SET_REGVAL:
            phyw( reg, p16);
            break;

        default:
            rc = -1;
            break;
    }

    PHPRINTF("  bmcr=0x%04x, rc=%d, par=%x\n", bmcr, rc, (unsigned int) *par);
    return rc;
}

/*!
 * \brief Set Phy Default, overwriting eventual wrong captured strap values,
 * if chips provides this option.
 *
 * When the NIC hardware reset pin is not connected to a dedicated GPIO
 * pin but to system reset, EMI, bad layout or other reasons may result
 * in wrong strap values captured. This happens e.g. on the STM32 Nucleo144
 * boards.
 *
 * Overwriting wrong strapped values with valid values may be an option
 * to resolve this problem. Another option is phy hardware reset with a
 * dedicated GPIO and all strap line in a well defined state.
 *
 * No parameter, no return
 */

void NutPhySetDefault(void)
{
    uint16_t regval;
    switch (phydcb->oui) {
    case LAN8700:
    case LAN8720:
    case LAN8742A:
        /* LAN87xx has the strap values in register 18: Special Modes Register.
         * Register 18 content is protected from Soft reset.
         */
        regval = phyr(18);
        if ((regval & 0x00e1) != (0x00e0 | NIC_PHY_ADDR)) {
#ifdef NUTDEBUG
            uint16_t tmp = regval;
#endif
            regval &= 0xff00;
            regval |= 0x00e0 | NIC_PHY_ADDR;
            phyw(18, regval);
            PHPRINTF("Correcting strap value 0x%04x -> 0x%04x\n", tmp, regval);
        }
        break;
    }
}

/*!
 * \brief Register and initialize PHY communication.
 *
 * This function registers a PHY for use by an EMAC.
 * For communication tests, the function reads out the OUI and
 * Model/Revision registers of the PHY.
 *
 * \param  mda id the PHY's address on the MDIO interface bus of the EMAC.
 * \param  mdiow Function provided by EMAC driver to write PHY regisers.
 * \param  mdior Function provided by EMAC driver to read PHY regisers.
 *
 * \return 0 on success, -1 on communication failure.
 */

int NutRegisterPhy( uint8_t mda, void(*mdiow)(uint8_t, uint16_t), uint16_t(*mdior)(uint8_t))
{
    uint16_t temp1 = 0, temp2 = 0;
    uint_fast16_t count, length = sizeof(phy_status_descr) / sizeof(phy_status_descr[0]);

    PHPRINTF("NRP(%u, %p, %p)\n", mda, mdiow, mdior);

    if (phydcb != NULL)
    {
        /* Phy is just registered */
        return 0;
    }

    if ((mdiow == NULL)||(mdior == NULL)) {
        /* PHY Access functions are not given */
        return -1;
    }

    phydcb = NutHeapAlloc( sizeof(PHYDCB));
    if (phydcb == NULL) {
        /* Not enough memory to register PHY */
        return -1;
    }

    phydcb->addr = mda;
    phydcb->mdiow = mdiow;
    phydcb->mdior = mdior;

    /* Get chip's OUI data */
    temp1 = phyr(PHY_ID1);
    temp2 = phyr(PHY_ID2);

    phydcb->oui = (((uint32_t)temp1<<16)|(uint32_t)temp2);

    /* Mask out revision bits */
    phydcb->oui &=  ~OUIMSK_REV;
    for(count=0; count<length; count++) {
        if(phy_status_descr[count].phy_oui == phydcb->oui) {
            break;
        }
    }
    if(count>=length)
        PHPRINTF("Unknown tranceiver ");

    PHPRINTF("PHY OUI=0x%08lx\n", phydcb->oui);
    NutPhySetDefault();

    return 0;
}
