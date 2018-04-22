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
 *
 * Portions Copyright Atmel Corporation, see the following note.
 */

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 */


#include <avr32/io.h>
#include <arch/avr32.h>
#include <arch/avr32/pm.h>


#include <cfg/os.h>
#include <cfg/clock.h>
#include <arch/avr32.h>
#include <dev/irqreg.h>
#include <sys/timer.h>

#include <arch/avr32/ihndlr.h>

#include <avr32/io.h>
#include "compiler.h"

/*! \name PM Writable Bit-Field Registers
 */

typedef union
{
  unsigned long                 mcctrl;
  avr32_pm_mcctrl_t             MCCTRL;
} u_avr32_pm_mcctrl_t;

typedef union
{
  unsigned long                 cksel;
  avr32_pm_cksel_t              CKSEL;
} u_avr32_pm_cksel_t;

typedef union
{
  unsigned long                 pll;
  avr32_pm_pll_t                PLL;
} u_avr32_pm_pll_t;

typedef union
{
  unsigned long                 oscctrl0;
  avr32_pm_oscctrl0_t           OSCCTRL0;
} u_avr32_pm_oscctrl0_t;

typedef union
{
  unsigned long                 oscctrl1;
  avr32_pm_oscctrl1_t           OSCCTRL1;
} u_avr32_pm_oscctrl1_t;

typedef union
{
  unsigned long                 oscctrl32;
  avr32_pm_oscctrl32_t          OSCCTRL32;
} u_avr32_pm_oscctrl32_t;

typedef union
{
  unsigned long                 ier;
  avr32_pm_ier_t                IER;
} u_avr32_pm_ier_t;

typedef union
{
  unsigned long                 idr;
  avr32_pm_idr_t                IDR;
} u_avr32_pm_idr_t;

typedef union
{
  unsigned long                 icr;
  avr32_pm_icr_t                ICR;
} u_avr32_pm_icr_t;

typedef union
{
  unsigned long                 gcctrl;
  avr32_pm_gcctrl_t             GCCTRL;
} u_avr32_pm_gcctrl_t;

typedef union
{
  unsigned long                 rccr;
  avr32_pm_rccr_t               RCCR;
} u_avr32_pm_rccr_t;

typedef union
{
  unsigned long                 bgcr;
  avr32_pm_bgcr_t               BGCR;
} u_avr32_pm_bgcr_t;

typedef union
{
  unsigned long                 vregcr;
  avr32_pm_vregcr_t             VREGCR;
} u_avr32_pm_vregcr_t;

typedef union
{
  unsigned long                 bod;
  avr32_pm_bod_t                BOD;
} u_avr32_pm_bod_t;


/*! \brief Sets the mode of the oscillator 0.
 *
 * \param pm Base address of the Power Manager (i.e. &AVR32_PM).
 * \param mode Oscillator 0 mode (i.e. AVR32_PM_OSCCTRL0_MODE_x).
 */
static inline void pm_set_osc0_mode(volatile avr32_pm_t *pm, unsigned int mode)
{
  // Read
  u_avr32_pm_oscctrl0_t u_avr32_pm_oscctrl0 = {pm->oscctrl0};
  // Modify
  u_avr32_pm_oscctrl0.OSCCTRL0.mode = mode;
  // Write
  pm->oscctrl0 = u_avr32_pm_oscctrl0.oscctrl0;
}

void pm_enable_osc0_crystal(unsigned int fosc0)
{
  pm_set_osc0_mode(&AVR32_PM, (fosc0 <  900000) ? AVR32_PM_OSCCTRL0_MODE_CRYSTAL_G0 :
                       (fosc0 < 3000000) ? AVR32_PM_OSCCTRL0_MODE_CRYSTAL_G1 :
                       (fosc0 < 8000000) ? AVR32_PM_OSCCTRL0_MODE_CRYSTAL_G2 :
                                           AVR32_PM_OSCCTRL0_MODE_CRYSTAL_G3);
}

void pm_switch_to_osc0(unsigned int fosc0, unsigned int startup)
{
    pm_enable_osc0_crystal(fosc0);            // Enable the Osc0 in crystal mode
    pm_enable_clk0(startup);                  // Crystal startup time - This parameter is critical and depends on the characteristics of the crystal
    pm_switch_to_clock(AVR32_PM_MCSEL_OSC0);  // Then switch main clock to Osc0
}

void pm_enable_clk0(unsigned int startup)
{
    // Read register
    u_avr32_pm_oscctrl0_t u_avr32_pm_oscctrl0 = {AVR32_PM.oscctrl0};
    // Modify
    u_avr32_pm_oscctrl0.OSCCTRL0.startup = startup;
    // Write back
    AVR32_PM.oscctrl0 = u_avr32_pm_oscctrl0.oscctrl0;

    AVR32_PM.mcctrl |= AVR32_PM_MCCTRL_OSC0EN_MASK;

    // Wait for clk0 ready
    while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_OSC0RDY_MASK));
}

void pm_switch_to_clock(unsigned long clock)
{
    // Read
    u_avr32_pm_mcctrl_t u_avr32_pm_mcctrl = {AVR32_PM.mcctrl};
    // Modify
    u_avr32_pm_mcctrl.MCCTRL.mcsel = clock;
    // Write back
    AVR32_PM.mcctrl = u_avr32_pm_mcctrl.mcctrl;
}

void pm_cksel(volatile avr32_pm_t *pm,   
    unsigned int pbadiv,     
    unsigned int pbasel,     
    unsigned int pbbdiv,     
    unsigned int pbbsel,     
    unsigned int hsbdiv,     
    unsigned int hsbsel)     
{    
    u_avr32_pm_cksel_t u_avr32_pm_cksel = {0};   

    u_avr32_pm_cksel.CKSEL.cpusel = hsbsel;      
    u_avr32_pm_cksel.CKSEL.cpudiv = hsbdiv;      
    u_avr32_pm_cksel.CKSEL.hsbsel = hsbsel;      
    u_avr32_pm_cksel.CKSEL.hsbdiv = hsbdiv;      
    u_avr32_pm_cksel.CKSEL.pbasel = pbasel;      
    u_avr32_pm_cksel.CKSEL.pbadiv = pbadiv;      
    u_avr32_pm_cksel.CKSEL.pbbsel = pbbsel;      
    u_avr32_pm_cksel.CKSEL.pbbdiv = pbbdiv;      

    pm->cksel = u_avr32_pm_cksel.cksel;      

    // Wait for ckrdy bit and then clear it      
    while (!(pm->poscsr & AVR32_PM_POSCSR_CKRDY_MASK));      
}

void pm_pll_setup(volatile avr32_pm_t *pm,
                  unsigned int pll,
                  unsigned int mul,
                  unsigned int div,
                  unsigned int osc,
                  unsigned int lockcount)
{
    u_avr32_pm_pll_t u_avr32_pm_pll = {0};

    u_avr32_pm_pll.PLL.pllosc   = osc;
    u_avr32_pm_pll.PLL.plldiv   = div;
    u_avr32_pm_pll.PLL.pllmul   = mul;
    u_avr32_pm_pll.PLL.pllcount = lockcount;

    pm->pll[pll] = u_avr32_pm_pll.pll;
}

void pm_pll_set_option(volatile avr32_pm_t *pm,
                       unsigned int pll,
                       unsigned int pll_freq,
                       unsigned int pll_div2,
                       unsigned int pll_wbwdisable)
{
    u_avr32_pm_pll_t u_avr32_pm_pll = {pm->pll[pll]};
    u_avr32_pm_pll.PLL.pllopt = pll_freq | (pll_div2 << 1) | (pll_wbwdisable << 2);
    pm->pll[pll] = u_avr32_pm_pll.pll;
}

void pm_pll_enable(volatile avr32_pm_t *pm,
                   unsigned int pll)
{
    pm->pll[pll] |= AVR32_PM_PLLEN_MASK;
}

void pm_wait_for_pll0_locked(volatile avr32_pm_t *pm)    
{    
    while (!(pm->poscsr & AVR32_PM_POSCSR_LOCK0_MASK));      
}

typedef union {
    unsigned long fcr;
    avr32_flashc_fcr_t FCR;
} u_avr32_flashc_fcr_t;

static void flashc_set_wait_state(unsigned int wait_state)
{
    u_avr32_flashc_fcr_t u_avr32_flashc_fcr = { AVR32_FLASHC.fcr };
    u_avr32_flashc_fcr.FCR.fws = wait_state;
    AVR32_FLASHC.fcr = u_avr32_flashc_fcr.fcr;
}

void Avr32InitClockTree( void )
{
    uint32_t CPUFrequency;

    /* Switch main clock to Oscillator 0 */
    pm_switch_to_osc0(OSC0_VAL, AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC);

    pm_pll_setup(&AVR32_PM, 0,  /* use PLL0     */
                 PLL_MUL_VAL,   /* MUL          */
                 PLL_DIV_VAL,   /* DIV          */
                 0,             /* Oscillator 0 */
                 16);           /* lockcount in main clock for the PLL wait lock */

    /*
     * This function will set a PLL option.
     *
     * pm             Base address of the Power Manager (i.e. &AVR32_PM)
     * pll            PLL number 0
     * pll_freq       Set to 1 for VCO frequency range 80-180MHz,
     *                set to 0 for VCO frequency range 160-240Mhz.
     * pll_div2       Divide the PLL output frequency by 2 (this settings does
     *                not change the FVCO value)
     * pll_wbwdisable 1 Disable the Wide-Bandwidth Mode (Wide-Bandwidth mode
     *                allow a faster startup time and out-of-lock time). 0 to
     *                enable the Wide-Bandwidth Mode.
     */
    pm_pll_set_option(&AVR32_PM, 0,     /* use PLL0 */
                      PLL_FREQ_VAL,     /* pll_freq */
                      PLL_DIV2_VAL,     /* pll_div2 */
                      PLL_WBWD_VAL);    /* pll_wbwd */

    /* Enable PLL0 */
    pm_pll_enable(&AVR32_PM, 0);

    /* Wait for PLL0 locked */
    pm_wait_for_pll0_locked(&AVR32_PM);

    /* Create PBA, PBB and HSB clock */
    pm_cksel(&AVR32_PM, PLL_PBADIV_VAL, /* pbadiv */
             PLL_PBASEL_VAL,    /* pbasel */
             PLL_PBBDIV_VAL,    /* pbbdiv */
             PLL_PBBSEL_VAL,    /* pbbsel */
             PLL_HSBDIV_VAL,    /* hsbdiv */
             PLL_HSBSEL_VAL);   /* hsbsel */

    /* Calculate CPU frequency */
    CPUFrequency = (OSC0_VAL * (PLL_MUL_VAL + 1)) / PLL_DIV_VAL;
    CPUFrequency = (PLL_DIV2_VAL == 0) ? CPUFrequency : CPUFrequency >> 1;

    if (PLL_HSBDIV_VAL > 0) {
        CPUFrequency = CPUFrequency >> (PLL_HSBSEL_VAL + 1);
    }

    if (CPUFrequency > AVR32_FLASHC_FWS_0_MAX_FREQ) {
        /*
         * Set one wait-state (WS) for the flash controller if the
         * HSB/CPU is more than AVR32_FLASHC_FWS_0_MAX_FREQ.
         */
        flashc_set_wait_state(1);
    }

    /* Switch PLL to main clock */
    pm_switch_to_clock(AVR32_PM_MCSEL_PLL0);
}
