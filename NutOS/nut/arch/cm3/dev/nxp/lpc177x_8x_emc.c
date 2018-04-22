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
 *
 *
 * Parts taken from lpc177x_8x_emc.c            2011-06-02
 *
 * file     lpc177x_8x_emc.c
 * brief    Contains all functions support for EMC firmware library
 *          on LPC177x_8x
 * version  1.0
 * date     02. June. 2011
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
 **********************************************************************/

/*
 * \verbatim
 * $Id: $
 * \endverbatim
 */

#include <sys/timer.h>
#include <arch/cm3.h>

#if defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#include <arch/cm3/nxp/lpc177x_8x_clk.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#include <arch/cm3/nxp/lpc407x_8x_clk.h>
#else
#warning "Unknown LPC familiy"
#endif

#include <arch/cm3/nxp/lpc177x_8x_emc.h>
#include <arch/cm3/nxp/lpc177x_8x_gpio.h>


/* default delays: source: practical test of rolf.meeser@nxp with the
   Embedded Artist Board. EMC Clock <= 80MHz, Command Delay strategi +0.
 */

#define DEFAULT_CLKOUTDLY 0
#define DEFAULT_FBCLKDLY  20
#define DEFAULT_CMDDLY    16

/* variables to save the original delays. Needed for later re-calibration of the
   delay lines.
 */

volatile uint32_t original_cmdclkdelay = 0;
volatile uint32_t original_fbclkdelay  = 0;
volatile uint32_t initial_calibration_value;


/*!
 * \brief Initialize EMC
 *
 * Initialize the external memory controller. GPIO pins should be correctly
 * configured when calling this function.
 *
 * \param     none
 * \return    none
 */

void Lpc177x_8x_EmcInit(void)
{
    /* uint8_t i; */

    /* Enable clock for EMC */
    SysCtlPeripheralClkEnable(CLKPWR_PCONP_PCEMC);

    LPC_SC->EMCDLYCTL   = 0x00001010;
    LPC_EMC->Control    = 0x00000001;
    LPC_EMC->Config     = 0x00000000;

    /* Pin configuration:
    * P2.14 - /EMC_CS2
    * P2.15 - /EMC_CS3
    *
    * P2.16 - /EMC_CAS
    * P2.17 - /EMC_RAS
    * P2.18 - EMC_CLK[0]
    * P2.19 - EMC_CLK[1]
    *
    * P2.20 - EMC_DYCS0
    * P2.21 - EMC_DYCS1
    * P2.22 - EMC_DYCS2
    * P2.23 - EMC_DYCS3
    *
    * P2.24 - EMC_CKE0
    * P2.25 - EMC_CKE1
    * P2.26 - EMC_CKE2
    * P2.27 - EMC_CKE3
    *
    * P2.28 - EMC_DQM0
    * P2.29 - EMC_DQM1
    * P2.30 - EMC_DQM2
    * P2.31 - EMC_DQM3
    *
    * P3.0-P3.31 - EMC_D[0-31]
    * P4.0-P4.23 - EMC_A[0-23]
    *
    * P4.24 - /EMC_OE
    * P4.25 - /EMC_WE
    *
    * P4.30 - /EMC_CS0
    * P4.31 - /EMC_CS1
    */

    /* GPIO Pin configuration should be done in the board file according to
       the hardware configuration
     */
}


/*!
 * \brief       SDRAM Delay line calibration.
 *
 * The estimated value gives a hint to correct the SDRAM timings
 *
 * \param       none
 * \return      value 0..4095 corresponding to the speed of logic.
 *              Faster logik (less delay) gives larger output values.
 */

static unsigned int sdram_calibrate (void)
{
    unsigned int i;
    unsigned int value = 0;
    unsigned int data;

    for (i = 16; i; i--) {        // get a mean value
        LPC_SC->EMCCAL = 0x4000;   // start calibration
        do {
            data = LPC_SC->EMCCAL;
        } while (!(data & 0x8000));// wait until DONE
        value += data & 0xFF;
    }
    return value;
}


/*!
 * \brief       Optimize the SDRAM delays.
 *
 * The complete range of a delay (0..31) is set, then tested by a SDRAM test.
 * Output value is the middle of the functional range.
 *
 * \param       sdram      SDRAM configuration struct
 * \param       shift      Bitshift value to point to the delay position in EMCDLYCTL
 * \param       def_value  Value to set if no positive range can be found.
 * \return      Optimum value which is set in EMCDLYCTL.
 */

static unsigned int sdram_optimize_delay(SDRAM sdram, unsigned int shift, unsigned int def_value)
{
    int i;
    int start_delay = -1;
    int stop_delay = -1;
#define DELAY_RANGE 0x1F

    for (i = 0; i <= DELAY_RANGE; i++) {
        // set delay value
        LPC_SC->EMCDLYCTL = ( LPC_SC->EMCDLYCTL & ~(DELAY_RANGE << shift) ) | (i << shift) ;
        if (Lpc177x_8x_EmcSDRAMCheck(sdram, i+10+shift) == 0) {
            if (start_delay < 0) {
                start_delay = i;
            }
            stop_delay = i;
        }
    }
    i = (start_delay + stop_delay) / 2;
    if (i < 0) {
        i = def_value;
    }

    // set new delay value
    LPC_SC->EMCDLYCTL = ( LPC_SC->EMCDLYCTL & ~(DELAY_RANGE << shift) ) | (i << shift);
    return i;
}


/*!
 * \brief       Ajust SDRAM SDRAM timing
 *
 * This function should be called regularly to re-calibrate the delay lines
 * which might change their correct delays due to thermal influences
 *
 * \param       none
 * \return      none
 */

void Lpc177x_8x_EmcSDRAMAdjustTiming(void)
{
    /* Get the current calibration values */
    unsigned int calibration_value = sdram_calibrate();

    /* calculate the new delay line settings */
    unsigned int fbclkdelay  = ((original_fbclkdelay  * calibration_value) / initial_calibration_value);
    unsigned int cmdclkdelay = ((original_cmdclkdelay * calibration_value) / initial_calibration_value);

    /* Sanity range check */
    if (fbclkdelay > 0x1F) {
        fbclkdelay = 0x1F;
    }

    if (cmdclkdelay > 0x1F) {
        cmdclkdelay = 0x1F;
    }

    /* Setup new values */
    LPC_SC->EMCDLYCTL = (DEFAULT_CLKOUTDLY << 24) | (DEFAULT_CLKOUTDLY << 16) |
                        (fbclkdelay << 8) | cmdclkdelay;
}


/*!
 * \brief       Simple SDRAM test
 *
 * This is used at startup time (before the first usage) to find the optimum
 * delay time values.
 *
 * ATT: This test overwrites the previous SDRAM contents!
 * Please note that IT IS NOT POSSIBLE to preserve these contents because
 * it might be possible that garbage address commands are output and we will
 * not know which locations are addressed during this test.
 * DO NOT USE this test if executing code from SDRAM.
 *
 * ATT: This test is far from complete. We neither check differen write sizes
 * nor do we use selected pattern to check single bit failures etc.
 *
 * \param       sdram   configuration struct of the used sdram
 * \param       offset  constant used to modify the test pattern
 * \return      0 if test is ok, 1 in case of a failure
 */

int Lpc177x_8x_EmcSDRAMCheck(SDRAM sdram, uint32_t offset)
{
    volatile uint32_t *ram_ptr;
    uint32_t data;
    uint32_t i,j;
    uint32_t blocks;

#define SDRAMTEST_BLOCKSIZE 0x100
#define SDRAMTEST_JUMPSIZE  0x10000

    blocks = sdram.size / (sizeof(uint32_t) * SDRAMTEST_JUMPSIZE);

    /* Writing */
    ram_ptr = (uint32_t *)sdram.base_addr;
    data = 0x10000 + offset;
    for (i = 0; i < blocks; i++) {
        for (j = 0; j < SDRAMTEST_BLOCKSIZE; j++) {
            *ram_ptr++ = data;
            data += j + (j << 4);
        }
        ram_ptr += SDRAMTEST_JUMPSIZE - SDRAMTEST_BLOCKSIZE;
        data += i + (i << 4);
    }

    /* Verifying */
    ram_ptr = (uint32_t *)sdram.base_addr;
    data = 0x10000 + offset;
    for (i = 0; i < blocks; i++) {
        for (j = 0; j < SDRAMTEST_BLOCKSIZE; j++) {
            if (data != *ram_ptr++) {
                return 1;
            }
            data += j + (j << 4);
        }
        ram_ptr += SDRAMTEST_JUMPSIZE - SDRAMTEST_BLOCKSIZE;
        data += i + (i << 4);
    }
    return 0;
}


/*!
 * \brief       EMC SDRAM initialisation
 *
 * This function configures the emc to a sdram connected to CS0
 * SDRAM timing parameters must be given as parameter
 *
 * \param       sdram   configuration struct of the used sdram
 * \param       dynamic_config  configuration values for dynamic_config register
 * \return      none
 */
void Lpc177x_8x_EmcSDRAMInit(SDRAM sdram, uint32_t dynamic_config)
{
    uint32_t emc_clock;
    uint32_t cpu_clock;

    emc_clock = NutArchClockGet(NUT_HWCLK_EMC);
    cpu_clock = NutArchClockGet(NUT_HWCLK_CPU);

    /* Initialize delay lines */
    LPC_SC->EMCDLYCTL          = 0x00000210;
    LPC_SC->EMCDLYCTL         |= 0x00080808;


    /* Init SDRAM controller */

    LPC_EMC->Control           = 0x00000001;
    LPC_EMC->Config            = 0x00000000;
    LPC_EMC->DynamicReadConfig = 0x00000001;
    LPC_EMC->DynamicRasCas0    = ((uint32_t)sdram.cas_latency) << 8 | (uint32_t)sdram.ras_latency;
    LPC_EMC->DynamicRP         = NS_2_CLKS(emc_clock, sdram.tRP);
    LPC_EMC->DynamicRAS        = NS_2_CLKS(emc_clock, sdram.tRAS);
    LPC_EMC->DynamicSREX       = NS_2_CLKS(emc_clock, sdram.tSREX);
    LPC_EMC->DynamicAPR        = sdram.tAPR;
    LPC_EMC->DynamicDAL        = sdram.tDAL + NS_2_CLKS(emc_clock, sdram.tRP);
    LPC_EMC->DynamicWR         = sdram.tWR;
    LPC_EMC->DynamicRC         = NS_2_CLKS(emc_clock, sdram.tRC);
    LPC_EMC->DynamicRFC        = NS_2_CLKS(emc_clock, sdram.tRFC);
    LPC_EMC->DynamicXSR        = NS_2_CLKS(emc_clock, sdram.tXSR);
    LPC_EMC->DynamicRRD        = NS_2_CLKS(emc_clock, sdram.tRRD);
    LPC_EMC->DynamicMRD        = sdram.tMRD;

    /* Configure addressing mode */
    LPC_EMC->DynamicConfig0 = dynamic_config;

    /* JEDEC General SDRAM Initialization Sequence
       DELAY to allow power and clocks to stabilize ~100 us
       NOP
     */
    LPC_EMC->DynamicControl = 0x00000183;

    wait_clocks(NS_2_CLKS(cpu_clock, 200000));

    /* PALL */
    LPC_EMC->DynamicControl = 0x00000103;
    LPC_EMC->DynamicRefresh = 0x00000002;

    wait_clocks(256); /* wait > 128 clk */

    LPC_EMC->DynamicRefresh = NS_2_CLKS(emc_clock, sdram.refresh) >> 4;

    wait_clocks(256); /* wait > 128 clk */

    /* COMM */
    LPC_EMC->DynamicControl = 0x00000083; /* Issue MODE command */

    /* Set mode register in SDRAM
       A0-A10 define the op-code written to the mode register.
       To compute the location of the A0-A10 in the SDRAM address range (MA):
       - MA0 = Byte Address (not used)
       - MA1..MA9 = Column Addresses (not used in Mode command).
       - MA10..MA21 = Row Addresses (MA10..MA19 used by Mode command).
       - MA22..MA23 = Bank Addresses (in BANK-ROW-COLUMN mode)
       ------------------------------------------------------------------------------------------
       Mode register table:
          bit 9:   Write Burst Mode: Programmed burst length(0), Single Location Access(1)   = 0
          bit 8~7: Operating Mode: Standard Operation(0) is the only thing defined           = 00
          bit 6~4: CAS latency: 001(1), 010(2), 011(3)                                       = 010
          bit 3:   Type of Burst: Sequential(0) or Interleaved(1)                            = 0
          bit 2~0: Burst length: 000(1), 001(2), 010(4), 011(8), 111(Full Page)              = 011
    */

    if (sdram.bus_width == 32) {
        (void) *((volatile uint32_t *)(sdram.base_addr | ((sdram.cas_latency << 4) | 0x02 /* burst length 4 */) <<
                                   (((dynamic_config & 0x1000) ? 0 : 2 /* number of bs */) + 2 /* bus width */ + sdram.cols )));
    } else {
        (void) *((volatile uint32_t *)(sdram.base_addr | ((sdram.cas_latency << 4) | 0x03 /* burst length 8 */) <<
                                   (((dynamic_config & 0x1000) ? 0 : 2 /* number of bs */) + 1 /* bus width */ + sdram.cols )));
    }

    wait_clocks(256); /* wait > 128 clk */

    /* NORM */
    LPC_EMC->DynamicControl = 0x0000;
    /* Reenable buffers */
    LPC_EMC->DynamicConfig0 |= _BV(19);

    wait_clocks(NS_2_CLKS(cpu_clock, 200000));

    initial_calibration_value = sdram_calibrate();
    original_cmdclkdelay = sdram_optimize_delay(sdram, 0, DEFAULT_CMDDLY);
    original_fbclkdelay  = sdram_optimize_delay(sdram, 8, DEFAULT_FBCLKDLY);
}


/*!
 * \brief     Configure Little Endian/Big Endian mode for EMC
 *
 * \param     endian_mode    Endian mode, should be:
 *                           - EMC_LITTLE_ENDIAN_MODE:   Little-endian mode
 *                           - EMC_BIG_ENDIAN_MODE   :   Big-endian mode
 *
 * \return    none
 */

void Lpc177x_8x_EmcConfigEndianMode(uint32_t endian_mode)
{
    LPC_EMC->Config = ((LPC_EMC->Config & 0x01) | endian_mode) & EMC_Config_MASK;
}


/****************** Group of Dynamic control functions************************/

/*!
 * \brief     Set the value for dynamic clock enable bit
 *
 * \param     clock_enable  clock enable mode, should be:
 *                          - 0: Clock enable of idle devices are deasserted to
 *                               save power
 *                          - EMC_DynamicControl_CE: All clock enables are driven HIGH continuously
 *
 * \return none
 */

void Lpc177x_8x_EmcDynCtrlClockEnable(uint32_t clock_enable)
{
    uint32_t mask = ~EMC_DynamicControl_CE;
    LPC_EMC->DynamicControl = ((LPC_EMC->DynamicControl & mask) | clock_enable);
}


/*!
 * \brief     Set the value for dynamic memory clock control:
 *            stops or runs continuously
 *
 * \param     clock_control  clock control mode, should be:
 *                           - 0: CLKOUT stops when all SDRAMs are idle and
 *                                during self-refresh mode
 *                           - EMC_DynamicControl_CS: CLKOUT runs continuously
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynCtrlClockControl(uint32_t clock_control)
{
    uint32_t mask = ~EMC_DynamicControl_CS;
    LPC_EMC->DynamicControl = ((LPC_EMC->DynamicControl & mask) | clock_control);
}


/*!
 * \brief     Switch the Self-refresh mode between normal and self-refresh mode
 *
 * \param     self_refresh_mode  self refresh mode, should be:
 *                               - 0: Normal mode
 *                               - EMC_DynamicControl_SR: Enter self-refresh mode
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynCtrlSelfRefresh(uint32_t self_refresh_mode)
{
    uint32_t mask = ~EMC_DynamicControl_SR;
    LPC_EMC->DynamicControl = ((LPC_EMC->DynamicControl & mask) | self_refresh_mode);
}


/*!
 * \brief     Enable/disable CLKOUT
 *
 * \param     MMC_val   Memory clock control mode, should be:
 *                      - 0: CLKOUT enabled
 *                      - EMC_DynamicControl_MMC: CLKOUT disabled
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynCtrlMMC(uint32_t mmc_val)
{
    uint32_t mask = ~EMC_DynamicControl_MMC;
    LPC_EMC->DynamicControl = ((LPC_EMC->DynamicControl & mask) | mmc_val);
}


/*!
 * \brief     Issue SDRAM command
 *
 * \param     sdram_command   Command mode, should be:
 *                            - 0x00: Issue SDRAM NORMAL operation command
 *                            - 0x01: Issue SDRAM MODE command
 *                            - 0x02: Issue SDRAM PALL (precharge all) command
 *                            - 0x03: Issue SRAM NOP (no operation) command
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynCtrlSDRAMCmd(uint32_t sdram_command)
{
    uint32_t mask = ~(uint32_t)(0x03 << 7);
    LPC_EMC->DynamicControl = ((LPC_EMC->DynamicControl & mask)| EMC_DynamicControl_I(sdram_command));
}


/*!
 * \brief     Switch between Normal operation and deep sleep power mode
 *
 * \param     power_command   Low-power SDRAM deep-sleep mode, should be:
 *                            - 0: Normal operation
 *                            - EMC_DynamicControl_DP: Enter deep-sleep mode
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynCtrlPowerDownMode(uint32_t power_command)
{
    uint32_t mask = ~EMC_DynamicControl_DP;
    LPC_EMC->DynamicControl = ((LPC_EMC->DynamicControl & mask)| power_command);
}


/*!
 * \brief     Set the value of EMC dynamic memory registers
 *
 * \param     par   EMC register that will set value, should be:
 *                  - EMC_DYN_MEM_REFRESH_TIMER: Dynamic Refresh register
 *                  - EMC_DYN_MEM_READ_CONFIG: Dynamic Read Config register
 *                  - EMC_DYN_MEM_TRP: Dynamic RP register
 *                  - EMC_DYN_MEM_TRAS: Dynamic RAS register
 *                  - EMC_DYN_MEM_TSREX: Dynamic SREX register
 *                  - EMC_DYN_MEM_TAPR: Dynamic APR register
 *                  - EMC_DYN_MEM_TDAL: Dynamic DAL register
 *                  - EMC_DYN_MEM_TWR: Dynamic WR register
 *                  - EMC_DYN_MEM_TRC: Dynamic RC register
 *                  - EMC_DYN_MEM_TRFC: Dynamic RFC register
 *                  - EMC_DYN_MEM_TXSR: Dynamic XSR register
 *                  - EMC_DYN_MEM_TRRD: Dynamic RRD register
 *                  - EMC_DYN_MEM_TMRD: Dynamic MRD register
 *
 * \return    none
 */

void Lpc177x_8x_EmcSetDynMemoryParameter(EMC_DYN_MEM_PAR par, uint32_t val)
{
    switch ( par) {
        case EMC_DYN_MEM_REFRESH_TIMER:
            LPC_EMC->DynamicRefresh = val;
            break;
        case EMC_DYN_MEM_READ_CONFIG:
            LPC_EMC->DynamicReadConfig = val;
            break;
        case EMC_DYN_MEM_TRP:
            LPC_EMC->DynamicRP = val;
            break;
        case EMC_DYN_MEM_TRAS:
            LPC_EMC->DynamicRAS = val;
            break;
        case EMC_DYN_MEM_TSREX:
            LPC_EMC->DynamicSREX = val;
            break;
        case EMC_DYN_MEM_TAPR:
            LPC_EMC->DynamicAPR = val;
            break;
        case EMC_DYN_MEM_TDAL:
            LPC_EMC->DynamicDAL = val;
            break;
        case EMC_DYN_MEM_TWR:
            LPC_EMC->DynamicWR = val;
            break;
        case EMC_DYN_MEM_TRC:
            LPC_EMC->DynamicRC = val;
            break;
        case EMC_DYN_MEM_TRFC:
            LPC_EMC->DynamicRFC = val;
            break;
        case EMC_DYN_MEM_TXSR:
            LPC_EMC->DynamicXSR = val;
            break;
        case EMC_DYN_MEM_TRRD:
            LPC_EMC->DynamicRRD = val;
            break;
        case EMC_DYN_MEM_TMRD:
            LPC_EMC->DynamicMRD = val;
            break;
    }
}


/*!
 * \brief     Set extended wait time out for accessing static memory
 *
 * \param     timeout     timeout value that will be set
 *
 * \return    none
 */

void Lpc177x_8x_EmcStaticExtendedWait(uint32_t timeout)
{
    LPC_EMC->StaticExtendedWait = timeout;
}


/*!
 * \brief     Configure the memory device
 *
 * \param     cs          number of chip select, should be from 0 to 3
 * \param     mem_dev     memory device type, should be:
 *                        - 0x00: Low-power SDRAM
 *                        - 0x01: Low-power SDRAM
 *                        - 0x02: Micron Syncflash
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynMemConfigMD(uint32_t cs, uint32_t mem_dev)
{
    uint32_t mask = ~(uint32_t)(0x03 << 3);
    switch (cs) {
        case 0:
            LPC_EMC->DynamicConfig0 = (LPC_EMC->DynamicConfig0 & mask) | mem_dev;
            break;
        case 1:
            LPC_EMC->DynamicConfig1 = (LPC_EMC->DynamicConfig1 & mask) | mem_dev;
            break;
        case 2:
            LPC_EMC->DynamicConfig2 = (LPC_EMC->DynamicConfig2 & mask) | mem_dev;
            break;
        case 3:
            LPC_EMC->DynamicConfig3 = (LPC_EMC->DynamicConfig3 & mask) | mem_dev;
            break;
    }
}


/*!
 * \brief     Map the address for the memory device
 *
 * \param     cs          number of chip select, should be from 0 to 3
 * \param     addr_mapped address where the memory will be mapped
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynMemConfigAM(uint32_t cs, uint32_t addr_mapped)
{
    uint32_t mask = ~(uint32_t)(0x3F << 7) | ~(uint32_t)(_BV(14))  ;

    switch (cs) {
        case 0:
            LPC_EMC->DynamicConfig0 = (LPC_EMC->DynamicConfig0 & mask) | addr_mapped;
            break;
        case 1:
            LPC_EMC->DynamicConfig1 = (LPC_EMC->DynamicConfig1 & mask) | addr_mapped;
            break;
        case 2:
            LPC_EMC->DynamicConfig2 = (LPC_EMC->DynamicConfig2 & mask) | addr_mapped;
            break;
        case 3:
            LPC_EMC->DynamicConfig3 = (LPC_EMC->DynamicConfig3 & mask) | addr_mapped;
            break;
    }
}


/*!
 * \brief     Enable/disable the buffer
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     buff_control  buffer control mode, should be:
 *                          0 - buffer disabled
 *                          EMC_StaticConfig_B: buffer enabled for the selected chip select
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynMemConfigB(uint32_t cs, uint32_t buff_control)
{
    uint32_t mask =  ~EMC_DynamicConfig_B;
    switch (cs) {
        case 0:
            LPC_EMC->DynamicConfig0 = (LPC_EMC->DynamicConfig0 & mask) | buff_control;
            break;
        case 1:
            LPC_EMC->DynamicConfig1 = (LPC_EMC->DynamicConfig1 & mask) | buff_control;
            break;
        case 2:
            LPC_EMC->DynamicConfig2 = (LPC_EMC->DynamicConfig2 & mask) | buff_control;
            break;
        case 3:
            LPC_EMC->DynamicConfig3 = (LPC_EMC->DynamicConfig3 & mask) | buff_control;
            break;
    }
}


/*!
 * \brief     Configure write permission: protect or not
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     permission    permission mode, should be:
 *                          0 - not protected
 *                          EMC_StaticConfig_P: write protection enabled
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynMemConfigP(uint32_t cs, uint32_t permission)
{
    uint32_t mask =  ~EMC_DynamicConfig_P;
    switch (cs) {
        case 0:
            LPC_EMC->DynamicConfig0 = (LPC_EMC->DynamicConfig0 & mask) | permission;
            break;
        case 1:
            LPC_EMC->DynamicConfig1 = (LPC_EMC->DynamicConfig1 & mask) | permission;
            break;
        case 2:
            LPC_EMC->DynamicConfig2 = (LPC_EMC->DynamicConfig2 & mask) | permission;
            break;
        case 3:
            LPC_EMC->DynamicConfig3 = (LPC_EMC->DynamicConfig3 & mask) | permission;
            break;
    }
}


/*!
 * \brief     Set value for RAS latency
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     ras_val       RAS value should be in range: 0..3
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynMemRAS(uint32_t cs, uint32_t ras_val)
{
    uint32_t mask =  ~(uint32_t)(0x03)  ;

    switch (cs) {
        case 0:
            LPC_EMC->DynamicRasCas0 = (LPC_EMC->DynamicRasCas0 & mask) | ras_val;
            break;
        case 1:
            LPC_EMC->DynamicRasCas1 = (LPC_EMC->DynamicRasCas1 & mask) | ras_val;
            break;
        case 2:
            LPC_EMC->DynamicRasCas2 = (LPC_EMC->DynamicRasCas2 & mask) | ras_val;
            break;
        case 3:
            LPC_EMC->DynamicRasCas3 = (LPC_EMC->DynamicRasCas3 & mask) | ras_val;
            break;
    }
}


/*!
 * \brief     Set value for CAS latency
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     cas_val       CAS value should be in range: 0..3
 *
 * \return    none
 */

void Lpc177x_8x_EmcDynMemCAS(uint32_t cs, uint32_t cas_val)
{
    uint32_t mask =  ~(uint32_t)(0x03 << 8)  ;
    switch (cs) {
        case 0:
            LPC_EMC->DynamicRasCas0 = (LPC_EMC->DynamicRasCas0 & mask) | cas_val;
            break;
        case 1:
            LPC_EMC->DynamicRasCas1 = (LPC_EMC->DynamicRasCas1 & mask) | cas_val;
            break;
        case 2:
            LPC_EMC->DynamicRasCas2 = (LPC_EMC->DynamicRasCas2 & mask) | cas_val;
            break;
        case 3:
            LPC_EMC->DynamicRasCas3 = (LPC_EMC->DynamicRasCas3 & mask) | cas_val;
            break;
    }
}


/*!
 * \brief     Configure the memory bus width
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     mem_width     memory width, should be:
 *                          - EMC_StaticConfig_MW_8BITS
 *                          - EMC_StaticConfig_MW_16BITS
 *                          - EMC_StaticConfig_MW_32BITS
 *
 * \return    none
 */

void Lpc177x_8x_EmcStaticMemConfigMW(uint32_t cs, uint32_t mem_width)
{
    uint32_t mask =  ~(uint32_t)(0x03)  ;
    switch (cs) {
        case 0:
            LPC_EMC->StaticConfig0 = (LPC_EMC->StaticConfig0 & mask) | mem_width;
            break;
        case 1:
            LPC_EMC->StaticConfig1 = (LPC_EMC->StaticConfig1 & mask) | mem_width;
            break;
        case 2:
            LPC_EMC->StaticConfig2 = (LPC_EMC->StaticConfig2 & mask) | mem_width;
            break;
        case 3:
            LPC_EMC->StaticConfig3 = (LPC_EMC->StaticConfig3 & mask) | mem_width;
            break;
    }
}


/*!
 * \brief     Configure the page mode
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     page_mode     page mode, should be
 *                          - 0: disable
 *                          - EMC_StaticConfig_PM: asynchronous page mode enable
 *
 * \return    none
 */

void Lpc177x_8x_EmcStaticMemConfigPM(uint32_t cs, uint32_t page_mode)
{
    uint32_t mask =  ~EMC_StaticConfig_PM;
    switch (cs) {
        case 0:
            LPC_EMC->StaticConfig0 = (LPC_EMC->StaticConfig0 & mask) | page_mode;
            break;
        case 1:
            LPC_EMC->StaticConfig1 = (LPC_EMC->StaticConfig1 & mask) | page_mode;
            break;
        case 2:
            LPC_EMC->StaticConfig2 = (LPC_EMC->StaticConfig2 & mask) | page_mode;
            break;
        case 3:
            LPC_EMC->StaticConfig3 = (LPC_EMC->StaticConfig3 & mask) | page_mode;
            break;
    }
}

/*!
 * \brief     Configure the chip select polarity
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     polarity      chip select polarity, should be:
 *                          - 0: Active LOW ship select
 *                          - EMC_StaticConfig_PC: Active HIGH chip select
 *
 * \return    none
 */

void Lpc177x_8x_EmcStaticMemConfigPC(uint32_t cs , uint32_t polarity)
{
    uint32_t mask =  ~EMC_StaticConfig_PC;
    switch (cs) {
        case 0:
            LPC_EMC->StaticConfig0 = (LPC_EMC->StaticConfig0 & mask) | polarity;
            break;
        case 1:
            LPC_EMC->StaticConfig1 = (LPC_EMC->StaticConfig1 & mask) | polarity;
            break;
        case 2:
            LPC_EMC->StaticConfig2 = (LPC_EMC->StaticConfig2 & mask) | polarity;
            break;
        case 3:
            LPC_EMC->StaticConfig3 = (LPC_EMC->StaticConfig3 & mask) | polarity;
            break;
    }
}


/*!
 * \brief     Configure the byte lane state
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     pb_val        Byte lane state, should be:
 *                          - 0: For reads all bits in BLSn[3:0] are HIGH.
 *                          - EMC_StaticConfig_PB: For reads all bits in BLSn[3:0] are LOW.
 *
 * \return    none
 */

void Lpc177x_8x_EmcStaticMemConfigPB(uint32_t cs , uint32_t pb_val)
{
    uint32_t mask =  ~EMC_StaticConfig_PB  ;
    switch (cs) {
        case 0:
            LPC_EMC->StaticConfig0 = (LPC_EMC->StaticConfig0 & mask) | pb_val;
            break;
        case 1:
            LPC_EMC->StaticConfig1 = (LPC_EMC->StaticConfig1 & mask) | pb_val;
            break;
        case 2:
            LPC_EMC->StaticConfig2 = (LPC_EMC->StaticConfig2 & mask) | pb_val;
            break;
        case 3:
            LPC_EMC->StaticConfig3 = (LPC_EMC->StaticConfig3 & mask) | pb_val;
            break;
    }
}


/*!
 * \brief     Configure the extended wait value
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     ex_wait       Extended wait mode, should be:
 *                          - 0: Extended wait disabled.
 *                          - EMC_StaticConfig_EW: Extended wait enabled.
 *
 * \return    none
 */

void Lpc177x_8x_EmcStaticMemConfigEW(uint32_t cs , uint32_t ex_wait)
{
    uint32_t mask =  ~EMC_StaticConfig_EW;
    switch (cs) {
        case 0:
            LPC_EMC->StaticConfig0 = (LPC_EMC->StaticConfig0 & mask) | ex_wait;
            break;
        case 1:
            LPC_EMC->StaticConfig1 = (LPC_EMC->StaticConfig1 & mask) | ex_wait;
            break;
        case 2:
            LPC_EMC->StaticConfig2 = (LPC_EMC->StaticConfig2 & mask) | ex_wait;
            break;
        case 3:
            LPC_EMC->StaticConfig3 = (LPC_EMC->StaticConfig3 & mask) | ex_wait;
            break;
    }
}


/*!
 * \brief     Configure the buffer enable value
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     buf_val       Buffer mode, should be:
 *                          - 0: Buffer disabled.
 *                          - EMC_StaticConfig_B: Buffer enabled.
 *
 * \return    none
 */

void Lpc177x_8x_EmcStaticMemConfigB(uint32_t cs , uint32_t buf_val)
{
    uint32_t mask =  ~EMC_StaticConfig_B;
    switch (cs) {
        case 0:
            LPC_EMC->StaticConfig0 = (LPC_EMC->StaticConfig0 & mask) | buf_val;
            break;
        case 1:
            LPC_EMC->StaticConfig1 = (LPC_EMC->StaticConfig1 & mask) | buf_val;
            break;
        case 2:
            LPC_EMC->StaticConfig2 = (LPC_EMC->StaticConfig2 & mask) | buf_val;
            break;
        case 3:
            LPC_EMC->StaticConfig3 = (LPC_EMC->StaticConfig3 & mask) | buf_val;
            break;
    }
}


/*!
 * \brief     Configure the write permission
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     permission    Permission mode, should be:
 *                          - 0: Write not protected.
 *                          - EMC_StaticConfig_P: Write protected.
 * \return    none
 */

void Lpc177x_8x_EmcStaticMemConfigpP(uint32_t cs , uint32_t permission)
{
    uint32_t mask =  ~(uint32_t)(_BV(20))  ;
    switch (cs) {
        case 0:
            LPC_EMC->StaticConfig0 = (LPC_EMC->StaticConfig0 & mask) | permission;
            break;
        case 1:
            LPC_EMC->StaticConfig1 = (LPC_EMC->StaticConfig1 & mask) | permission;
            break;
        case 2:
            LPC_EMC->StaticConfig2 = (LPC_EMC->StaticConfig2 & mask) | permission;
            break;
        case 3:
            LPC_EMC->StaticConfig3 = (LPC_EMC->StaticConfig3 & mask) | permission;
            break;
    }
}


/*!
 * \brief     Set the value of LPC_EMC static memory registers
 *
 * \param     cs            number of chip select, should be from 0 to 3
 * \param     par           Static register, should be:
 *                          - EMC_STA_MEM_WAITWEN: StaticWaitWen0 register
 *                          - EMC_STA_MEM_WAITOEN: StaticWaitOen0 register
 *                          - EMC_STA_MEM_WAITRD: StaticWaitRd0 register
 *                          - EMC_STA_MEM_WAITPAGE: StaticWaitPage0 register
 *                          - EMC_STA_MEM_WAITWR: StaticWaitWr0 register
 *                          - EMC_STA_MEM_WAITTURN: StaticWaitTurn0 register
 * \param     val           register value to set
 *
 * \return    none
 */

void Lpc177x_8x_EmcSetStaticMemoryParameter(uint32_t cs, EMC_STA_MEM_PAR par, uint32_t val)
{
    switch (cs) {
        case 0:
            switch (par) {
                case EMC_STA_MEM_WAITWEN:
                    LPC_EMC->StaticWaitWen0 = val;
                    break;
                case EMC_STA_MEM_WAITOEN:
                    LPC_EMC->StaticWaitOen0 = val;
                    break;
                case EMC_STA_MEM_WAITRD:
                    LPC_EMC->StaticWaitRd0 = val;
                    break;
                case EMC_STA_MEM_WAITPAGE:
                    LPC_EMC->StaticWaitPage0 = val;
                    break;
                case EMC_STA_MEM_WAITWR:
                    LPC_EMC->StaticWaitWr0 = val;
                    break;
                case EMC_STA_MEM_WAITTURN:
                    LPC_EMC->StaticWaitTurn0 = val;
                    break;
            }
            break;
        case 1:
            switch (par) {
                case EMC_STA_MEM_WAITWEN:
                    LPC_EMC->StaticWaitWen1 = val;
                    break;
                case EMC_STA_MEM_WAITOEN:
                    LPC_EMC->StaticWaitOen1 = val;
                    break;
                case EMC_STA_MEM_WAITRD:
                    LPC_EMC->StaticWaitRd1 = val;
                    break;
                case EMC_STA_MEM_WAITPAGE:
                    LPC_EMC->StaticWaitPage1 = val;
                    break;
                case EMC_STA_MEM_WAITWR:
                    LPC_EMC->StaticWaitWr1 = val;
                    break;
                case EMC_STA_MEM_WAITTURN:
                    LPC_EMC->StaticWaitTurn1 = val;
                    break;
            }
            break;
        case 2:
            switch (par) {
                case EMC_STA_MEM_WAITWEN:
                    LPC_EMC->StaticWaitWen2 = val;
                    break;
                case EMC_STA_MEM_WAITOEN:
                    LPC_EMC->StaticWaitOen2 = val;
                    break;
                case EMC_STA_MEM_WAITRD:
                    LPC_EMC->StaticWaitRd2 = val;
                    break;
                case EMC_STA_MEM_WAITPAGE:
                    LPC_EMC->StaticWaitPage2 = val;
                    break;
                case EMC_STA_MEM_WAITWR:
                    LPC_EMC->StaticWaitWr2 = val;
                    break;
                case EMC_STA_MEM_WAITTURN:
                    LPC_EMC->StaticWaitTurn2 = val;
                    break;
            }
            break;
        case 3:
            switch (par) {
                case EMC_STA_MEM_WAITWEN:
                    LPC_EMC->StaticWaitWen3 = val;
                    break;
                case EMC_STA_MEM_WAITOEN:
                    LPC_EMC->StaticWaitOen3 = val;
                    break;
                case EMC_STA_MEM_WAITRD:
                    LPC_EMC->StaticWaitRd3 = val;
                    break;
                case EMC_STA_MEM_WAITPAGE:
                    LPC_EMC->StaticWaitPage3 = val;
                    break;
                case EMC_STA_MEM_WAITWR:
                    LPC_EMC->StaticWaitWr3 = val;
                    break;
                case EMC_STA_MEM_WAITTURN:
                    LPC_EMC->StaticWaitTurn3 = val;
                    break;
            }
            break;
    }
}
