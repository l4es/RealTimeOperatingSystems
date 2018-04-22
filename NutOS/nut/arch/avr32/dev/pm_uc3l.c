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

/*! \name SCIF Writable Bit-Field Registers
 */
//! @{

typedef union
{
  unsigned long                 oscctrl0;
  avr32_scif_oscctrl0_t         OSCCTRL0;
} u_avr32_scif_oscctrl0_t;

typedef union
{
  unsigned long                 oscctrl32;
  avr32_scif_oscctrl32_t         OSCCTRL32;
} u_avr32_scif_oscctrl32_t;

typedef union
{
  unsigned long                 dfll0conf;
  avr32_scif_dfll0conf_t        DFLL0CONF;
} u_avr32_scif_dfll0conf_t;

typedef union
{
  unsigned long                 dfll0ssg;
  avr32_scif_dfll0ssg_t         DFLL0SSG;
} u_avr32_scif_dfll0ssg_t;

#if (UC3L0128 || UC3L0256)
typedef union
{
  unsigned long                 pll0;
  avr32_scif_pll_t              PLL0;
} u_avr32_scif_pll_t;
#endif

//! @}

//! Unlock SCIF register macro
#define SCIF_UNLOCK(reg)  (AVR32_SCIF.unlock = (AVR32_SCIF_UNLOCK_KEY_VALUE << AVR32_SCIF_UNLOCK_KEY_OFFSET)|(reg))


#define SCIF_DFLL0_MODE_OPENLOOP   0
#define SCIF_DFLL0_MODE_CLOSEDLOOP 1

//! The min DFLL output frequency
#if defined(MCU_UC3L0128) || defined(MCU_UC3L0256)
#define SCIF_DFLL_MINFREQ_KHZ         20000
#define SCIF_DFLL_MINFREQ_HZ          20000000UL
#else
#define SCIF_DFLL_MINFREQ_KHZ         40000
#define SCIF_DFLL_MINFREQ_HZ          40000000UL
#endif

//! The max DFLL output frequency
#define SCIF_DFLL_MAXFREQ_KHZ         150000
#define SCIF_DFLL_MAXFREQ_HZ          150000000UL

#define DFLL_COARSE_MAX   (AVR32_SCIF_COARSE_MASK >> AVR32_SCIF_COARSE_OFFSET)
#define DFLL_FINE_MAX     (AVR32_SCIF_FINE_MASK >> AVR32_SCIF_FINE_OFFSET)
#define DFLL_FINE_HALF    (1UL << (AVR32_SCIF_FINE_SIZE - 1))

//! \name System clock source
//@{
#define SYSCLK_SRC_RCSYS        0     //!< System RC oscillator
#define SYSCLK_SRC_OSC0         1     //!< Oscillator 0
#define SYSCLK_SRC_DFLL         2     //!< Digital Frequency Locked Loop
#define SYSCLK_SRC_RC120M       3     //!< 120 MHz RC oscillator
#if (UC3L0128 || UC3L0256)
#define SYSCLK_SRC_PLL0         4       //!< Phase Locked Loop 0
#endif

struct genclk_config {
    uint32_t ctrl;
};

struct dfll_config {
    struct genclk_config    ref_cfg;        //!< Reference clock
    uint32_t                conf;           //!< DFLLnCONF
    uint32_t                mul;            //!< DFLLnMUL
    uint32_t                step;           //!< DFLLnSTEP
    uint32_t                ssg;            //!< DFLLnSSG
};

typedef uint32_t irqflags_t;

#define barrier()      asm volatile("" ::: "memory")
#define sysreg_write(reg, val)         __builtin_mtsr(reg, val)
#define sysreg_read(reg)               __builtin_mfsr(reg)

#define cpu_irq_disable()                       \
    do {                                        \
        __builtin_ssrf(AVR32_SR_GM_OFFSET);     \
        barrier();                              \
    } while (0)

static inline irqflags_t cpu_irq_save(void)
{
    irqflags_t flags;

    flags = sysreg_read(AVR32_SR);
    cpu_irq_disable();

    return flags;
}

static inline void cpu_irq_restore(irqflags_t flags)
{
    barrier();
#if defined(__ICCAVR32__)
    // Barrier " __asm__ __volatile__ ("")"
    // Don't work with sysreg_write(AVR32_SR, flags)
    if( cpu_irq_is_enabled_flags(flags) ) {
        cpu_irq_enable();
    }
#else
    sysreg_write(AVR32_SR, flags);
#endif
    barrier();
}

#define dfll_write_reg(reg, value)                                      \
    do {                                                                \
        irqflags_t dfll_flags;                                          \
        while (!(AVR32_SCIF.pclksr & AVR32_SCIF_DFLL0RDY_MASK));        \
        dfll_flags = cpu_irq_save();                                    \
        SCIF_UNLOCK(AVR32_SCIF_##reg);                                  \
        *(volatile uint32_t *)(AVR32_SCIF_ADDRESS + AVR32_SCIF_##reg)   \
                = (value);                                              \
        cpu_irq_restore(dfll_flags);                                    \
    } while (0)


void dfll_enable_open_loop(const struct dfll_config *cfg,
        unsigned int dfll_id)
{
    irqflags_t flags;

    /* First, enable the DFLL, then configure it */
    flags = cpu_irq_save();
    SCIF_UNLOCK(AVR32_SCIF_DFLL0CONF);
    AVR32_SCIF.dfll0conf = 1U << AVR32_SCIF_DFLL0CONF_EN;
    cpu_irq_restore(flags);
    dfll_write_reg(DFLL0CONF, cfg->conf | (1U << AVR32_SCIF_DFLL0CONF_EN));
    dfll_write_reg(DFLL0SSG, cfg->ssg);
}

static inline void genclk_enable(const struct genclk_config *cfg, unsigned int id)
{
    AVR32_SCIF.gcctrl[id] = cfg->ctrl | (1U << AVR32_SCIF_GCCTRL_CEN);
}

void dfll_enable_closed_loop(const struct dfll_config *cfg,
        unsigned int dfll_id)
{
    irqflags_t flags;
    /* Enable the reference clock */
    genclk_enable(&cfg->ref_cfg, 0);

    /*
     * Enable the DFLL first, but don't wait for the DFLL0RDY bit
     * because if the DFLL has been disabled before, the DFLL0RDY
     * bit stays cleared until it is re-enabled.
     */
    flags = cpu_irq_save();
    AVR32_SCIF.unlock = 0xaa000000 | AVR32_SCIF_DFLL0CONF;
    AVR32_SCIF.dfll0conf = 1U << AVR32_SCIF_DFLL0CONF_EN;
    cpu_irq_restore(flags);

    /*
     * Then, configure the DFLL, taking care to wait for the
     * DFLL0RDY bit before every step.
     */
    dfll_write_reg(DFLL0STEP, cfg->step);
#ifdef AVR32_SCIF_DFLL0FMUL
    dfll_write_reg(DFLL0FMUL, cfg->mul);
#else
    dfll_write_reg(DFLL0MUL, cfg->mul);
#endif
    dfll_write_reg(DFLL0SSG, cfg->ssg);
    dfll_write_reg(DFLL0CONF, cfg->conf | (1U << AVR32_SCIF_DFLL0CONF_EN));
}

static inline void genclk_config_defaults(struct genclk_config *cfg, unsigned int id)
{
    cfg->ctrl = 0;
}

static inline void genclk_config_set_source(struct genclk_config *cfg, int src)
{
    cfg->ctrl = (cfg->ctrl & ~AVR32_SCIF_GCCTRL_OSCSEL_MASK)
    | (src << AVR32_SCIF_GCCTRL_OSCSEL);
}

static inline void genclk_config_set_divider(struct genclk_config *cfg,
    unsigned int divider)
{
//  Assert(divider > 0 && divider <= GENCLK_DIV_MAX);

    /* Clear all the bits we're about to modify */
    cfg->ctrl &= ~(AVR32_SCIF_GCCTRL_DIVEN_MASK
        | AVR32_SCIF_GCCTRL_DIV_MASK);

    if (divider > 1) {
        cfg->ctrl |= 1U << AVR32_SCIF_GCCTRL_DIVEN;
        cfg->ctrl |= (((divider + 1) / 2) - 1) << AVR32_SCIF_GCCTRL_DIV;
    }
}

static inline void dfll_config_init_open_loop_mode(struct dfll_config *cfg)
{
    genclk_config_defaults(&cfg->ref_cfg, 0);
    cfg->conf = 1;
    cfg->mul = 0;
    cfg->step = 0;
    cfg->ssg = 0;
}

static inline void dfll_config_init_closed_loop_mode(struct dfll_config *cfg,
        int refclk, uint16_t div, uint16_t mul)
{
    /*
     * Set up generic clock source with specified reference clock
     * and divider.
     */
    genclk_config_defaults(&cfg->ref_cfg, 0);
    genclk_config_set_source(&cfg->ref_cfg, refclk);
    genclk_config_set_divider(&cfg->ref_cfg, div);

    cfg->conf = 1U << AVR32_SCIF_DFLL0CONF_MODE;
    cfg->mul = mul << 16;
    /*
     * Initial step length of 4. If this is set too high, the DFLL
     * may fail to lock.
     */
    cfg->step = ((4U << AVR32_SCIF_DFLL0STEP_FSTEP)
            | (4U << AVR32_SCIF_DFLL0STEP_CSTEP));
    cfg->ssg = 0;
}

static inline void dfll_config_set_initial_tuning(struct dfll_config *cfg,
        uint16_t coarse, uint16_t fine)
{
    cfg->conf &= ~(AVR32_SCIF_DFLL0CONF_COARSE_MASK
            | AVR32_SCIF_DFLL0CONF_FINE_MASK);
    cfg->conf |= ((coarse << AVR32_SCIF_DFLL0CONF_COARSE)
            | (fine << AVR32_SCIF_DFLL0CONF_FINE));
}

/**
 * \brief Tune the DFLL configuration for a specific target frequency
 *
 * This will set the initial coarse and fine DFLL tuning to match the
 * given target frequency. In open loop mode, this will cause the DFLL
 * to run close to the specified frequency, though it may not match
 * exactly. In closed loop mode, the DFLL will automatically tune itself
 * to the target frequency regardless of the initial tuning, but this
 * function may be used to set a starting point close to the desired
 * frequency in order to reduce the startup time.
 *
 * \par Calculating the DFLL frequency
 *
 * \f{eqnarray*}{
    f_{DFLL} &=& \left[f_{min} + \left(f_{max} - f_{min}\right)
        \frac{\mathrm{COARSE}}{\mathrm{COARSE}_{max}}\right]
        \left(1 + x \frac{\mathrm{FINE}
            - \mathrm{FINE}_{half}}{\mathrm{FINE}_{max}}\right)
        = f_{coarse} \left(1 + x
        \frac{\mathrm{FINE}
            - \mathrm{FINE}_{half}}{\mathrm{FINE}_{max}}\right) \\
    \mathrm{COARSE} &=& \frac{\left(f_{DFLL} - f_{min}\right)}
        {f_{max} - f_{min}} \mathrm{COARSE}_{max} \\
    f_{coarse} &=& f_{min} + \left(f_{max} - f_{min}\right)
        \frac{\mathrm{COARSE}}{\mathrm{COARSE}_{max}} \\
    \mathrm{FINE} &=& \left(10 \frac{f_{DFLL} - f_{coarse}}
        {f_{coarse}} + \mathrm{FINE}_{half}\right) / 4
   \f}
 *
 * \param cfg The DFLL configuration to be tuned.
 * \param target_hz Target frequency in Hz.
 */
static inline void dfll_config_tune_for_target_hz(struct dfll_config *cfg,
        uint32_t target_hz)
{
    uint32_t target_khz;
    uint32_t coarse_khz;
    uint32_t delta_khz;
    uint32_t coarse;
    uint32_t fine;

    target_khz = target_hz / 1000;
    coarse = ((target_khz - SCIF_DFLL_MINFREQ_KHZ) * DFLL_COARSE_MAX)
            / (SCIF_DFLL_MAXFREQ_KHZ - SCIF_DFLL_MINFREQ_KHZ);
    coarse_khz = SCIF_DFLL_MINFREQ_KHZ + (((SCIF_DFLL_MAXFREQ_KHZ - SCIF_DFLL_MINFREQ_KHZ)
            / DFLL_COARSE_MAX) * coarse);
    delta_khz = target_khz - coarse_khz;
    fine = (((delta_khz * DFLL_FINE_MAX) * 2) / coarse_khz) * 5;
    fine += DFLL_FINE_HALF;
    fine /= 4;

    dfll_config_set_initial_tuning(cfg, coarse, fine);
}

/**
 * \brief Set system clock prescaler configuration
 *
 * This function will change the system clock prescaler configuration to
 * match the parameters.
 *
 * \note The parameters to this function are device-specific.
 *
 * \param cpu_shift The CPU clock will be divided by \f$2^{cpu\_shift}\f$
 * \param pba_shift The PBA clock will be divided by \f$2^{pba\_shift}\f$
 * \param pbb_shift The PBB clock will be divided by \f$2^{pbb\_shift}\f$
 */
void sysclk_set_prescalers(unsigned int cpu_shift,
        unsigned int pba_shift, unsigned int pbb_shift)
{
    irqflags_t flags;
    uint32_t   cpu_cksel = 0;
    uint32_t   pba_cksel = 0;
    uint32_t   pbb_cksel = 0;

//  Assert(cpu_shift <= pba_shift);
//  Assert(cpu_shift <= pbb_shift);

    if (cpu_shift > 0)
        cpu_cksel = ((cpu_shift - 1) << AVR32_PM_CPUSEL_CPUSEL)
                | (1U << AVR32_PM_CPUSEL_CPUDIV);

    if (pba_shift > 0)
        pba_cksel = ((pba_shift - 1) << AVR32_PM_PBASEL_PBSEL)
                | (1U << AVR32_PM_PBASEL_PBDIV);

    if (pbb_shift > 0)
        pbb_cksel = ((pbb_shift - 1) << AVR32_PM_PBBSEL_PBSEL)
                | (1U << AVR32_PM_PBBSEL_PBDIV);

    flags = cpu_irq_save();
    AVR32_PM.unlock = 0xaa000000 | AVR32_PM_CPUSEL;
    AVR32_PM.cpusel = cpu_cksel;
    AVR32_PM.unlock = 0xaa000000 | AVR32_PM_PBASEL;
    AVR32_PM.pbasel = pba_cksel;
    AVR32_PM.unlock = 0xaa000000 | AVR32_PM_PBBSEL;
    AVR32_PM.pbbsel = pbb_cksel;
    cpu_irq_restore(flags);
}

void pm_enable_osc0_crystal(unsigned int fosc0)
{
    AVR32_SCIF.gcctrl[AVR32_SCIF_GCLK_DFLL0_REF] = ((0 << AVR32_SCIF_GCCTRL_DIV_OFFSET)&AVR32_SCIF_GCCTRL_DIV_MASK)
        |((0 << AVR32_SCIF_GCCTRL_DIVEN_OFFSET)&AVR32_SCIF_GCCTRL_DIVEN_MASK)
        |((AVR32_SCIF_GC_USES_OSC0 << AVR32_SCIF_GCCTRL_OSCSEL_OFFSET)&AVR32_SCIF_GCCTRL_OSCSEL_MASK)
        |(AVR32_SCIF_GCCTRL_CEN_MASK);

}
#if 0
static void pm_enable_dfll(void)
{
    u_avr32_scif_dfll0conf_t  u_avr32_scif_dfll0conf = {AVR32_SCIF.dfll0conf};

    u_avr32_scif_dfll0conf.DFLL0CONF.en = 1;
    SCIF_UNLOCK(AVR32_SCIF_DFLL0CONF);
    AVR32_SCIF.dfll0conf = u_avr32_scif_dfll0conf.dfll0conf;

    while(!(AVR32_SCIF.pclksr & AVR32_SCIF_PCLKSR_DFLL0RDY_MASK));

#define COARSEMAXSTEP (((DFLL_CPU_FREQ - SCIF_DFLL_MINFREQ_HZ)*255)/(SCIF_DFLL_MAXFREQ_HZ - SCIF_DFLL_MINFREQ_HZ))
    SCIF_UNLOCK(AVR32_SCIF_DFLL0STEP);
    AVR32_SCIF.dfll0step = ((COARSEMAXSTEP << AVR32_SCIF_DFLL0STEP_CSTEP_OFFSET) & AVR32_SCIF_DFLL0STEP_CSTEP_MASK) | ((0x0000004 << AVR32_SCIF_DFLL0STEP_FSTEP_OFFSET) & AVR32_SCIF_DFLL0STEP_FSTEP_MASK);

    while(!(AVR32_SCIF.pclksr & AVR32_SCIF_PCLKSR_DFLL0RDY_MASK));

#if defined( AVR32_SCIF_DFLL0MUL )
    SCIF_UNLOCK(AVR32_SCIF_DFLL0MUL);
    AVR32_SCIF.dfll0mul = ((DFLL_CPU_FMUL << AVR32_SCIF_DFLL0MUL_FMUL_OFFSET)&AVR32_SCIF_DFLL0MUL_FMUL_MASK)
        | ((DFLL_CPU_IMUL << AVR32_SCIF_DFLL0MUL_IMUL_OFFSET)&AVR32_SCIF_DFLL0MUL_IMUL_MASK);
#else
    SCIF_UNLOCK(AVR32_SCIF_DFLL0FMUL);
    AVR32_SCIF.dfll0fmul = (DFLL_CPU_FMUL << AVR32_SCIF_DFLL0FMUL_FMUL_OFFSET)&AVR32_SCIF_DFLL0FMUL_FMUL_MASK;
#endif

    while(!(AVR32_SCIF.pclksr & AVR32_SCIF_PCLKSR_DFLL0RDY_MASK));

    // Set the DFLL0 to operate in closed-loop mode: DFLL0CONF.MODE=1
    u_avr32_scif_dfll0conf.DFLL0CONF.mode = SCIF_DFLL0_MODE_CLOSEDLOOP;
    u_avr32_scif_dfll0conf.DFLL0CONF.coarse = COARSEMAXSTEP;
    SCIF_UNLOCK(AVR32_SCIF_DFLL0CONF);
    AVR32_SCIF.dfll0conf = u_avr32_scif_dfll0conf.dfll0conf;

    // Wait for PCLKSR.DFLL0RDY is high
    while(!(AVR32_SCIF.pclksr & AVR32_SCIF_PCLKSR_DFLL0RDY_MASK));

    // Wait until the DFLL is locked on Fine value, and is ready to be selected as
    // clock source with a highly accurate output clock.
    while(!(AVR32_SCIF.pclksr & AVR32_SCIF_PCLKSR_DFLL0LOCKF_MASK));
}
#endif
/**
 * \brief Change the source of the main system clock.
 *
 * \param src The new system clock source. Must be one of the constants
 * from the <em>System Clock Sources</em> section.
 */
void sysclk_set_source(uint_fast8_t src)
{
    irqflags_t flags;
//  Assert(src <= SYSCLK_SRC_RC120M);

    flags = cpu_irq_save();
    AVR32_PM.unlock = 0xaa000000 | AVR32_PM_MCCTRL;
    AVR32_PM.mcctrl = src;
    cpu_irq_restore(flags);
}

static inline int dfll_is_fine_locked(unsigned int dfll_id)
{
    return !!(AVR32_SCIF.pclksr & (1U << AVR32_SCIF_PCLKSR_DFLL0LOCKF));
}

static inline int dfll_wait_for_fine_lock(unsigned int dfll_id)
{
    /* TODO: Add timeout mechanism */
    while (!dfll_is_fine_locked(dfll_id)) {
        /* Do nothing */
    }

    return 0;
}

void pm_switch_to_osc0(unsigned int fosc0, unsigned int startup)
{
    struct dfll_config dcfg;

    dfll_config_init_closed_loop_mode(&dcfg,
        0, 1,
        50000000 / AVR32_SCIF_RCOSC_FREQUENCY);
    dfll_enable_closed_loop(&dcfg, 0);
    sysclk_set_prescalers(1, 1, 1);
    dfll_wait_for_fine_lock(0);
    sysclk_set_source(SYSCLK_SRC_DFLL);

//  // Switch clock source to OSC0
//  pm_enable_osc0_crystal( fosc0 );
//
//  // Configure and start DFLL
//  pm_enable_dfll();
}

void Avr32InitClockTree( void )
{
    /* Enable one wait state for flash access */
    AVR32_FLASHCDW.fcr = AVR32_FLASHCDW_FWS_MASK;

    pm_switch_to_osc0( OSC0_VAL, 0 );
}
