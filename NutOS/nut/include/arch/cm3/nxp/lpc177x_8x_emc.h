#ifndef _LPC177X_8X_EMC_H_
#define _LPC177X_8X_EMC_H_

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
 * Parts taken from lpc177x_8x_emc.h            2011-06-02
 *
 * file     lpc177x_8x_emc.h
 * brief    Contains all macro definitions and function prototypes
 *          support for EMC firmware library on LPC177x_8x
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

#include <inttypes.h>
#include <dev/sdram.h>

/*----------------------------------------------------------------------------*
  External memory controller settings and functions
 *----------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------*
  EMC Control Register (EMCControl)
 *----------------------------------------------------------------------------*/

#define EMC_Control_MASK          ((uint32_t) 0x07)    /* Control register mask */
#define EMC_Control_E             ((uint32_t)(1<<0))   /* Control register EMC: Enable control. */
#define EMC_Control_M             ((uint32_t)(1<<1))   /* Control register EMC: Address mirror control. */
#define EMC_Control_L             ((uint32_t)(1<<2))   /* Control register EMC: Low-power mode control. */


/*----------------------------------------------------------------------------*
  EMC Status Register (EMCStatus)
 *----------------------------------------------------------------------------*/

#define EMC_Status_MASK           ((uint32_t) 0x07)    /* Status register mask */
#define EMC_Status_B              ((uint32_t)(1<<0))   /* Status register EMC: Busy. */
#define EMC_Status_S              ((uint32_t)(1<<1))   /* Status register EMC: Write buffer status. */
#define EMC_Status_SA             ((uint32_t)(1<<2))   /* Status register EMC: Self-refresh acknowledge.. */


/*----------------------------------------------------------------------------*
  EMC Configuration register (EMCConfig)
 *----------------------------------------------------------------------------*/

#define EMC_Config_Endian_Mode    ((uint32_t)(1<<0))   /* EMC Configuration register : Enable control. */
#define EMC_Config_CCLK           ((uint32_t)(1<<8))   /* EMC Configuration register: CCLK. */
#define EMC_Config_MASK           ((uint32_t)(0x101))  /* EMC Configuration register mask */


/*----------------------------------------------------------------------------*
  Dynamic Memory Control register (EMCDynamicControl)
 *----------------------------------------------------------------------------*/

#define EMC_DynamicControl_CE     ((uint32_t)(1<<0))  /* Dynamic Memory Control register EMC: Dynamic memory clock enable. */
#define EMC_DynamicControl_CS     ((uint32_t)(1<<1))  /* Dynamic Memory Control register EMC: Dynamic memory clock control */
#define EMC_DynamicControl_SR     ((uint32_t)(1<<2))  /* Dynamic Memory Control register EMC: Self-refresh request, EMCSREFREQ*/
#define EMC_DynamicControl_MMC    ((uint32_t)(1<<5))  /* Dynamic Memory Control register EMC: Memory clock control (MMC)*/
#define EMC_DynamicControl_I(n)   ((uint32_t)(n<<7))  /* Dynamic Memory Control register EMC: SDRAM initialization*/
#define EMC_DynamicControl_DP     ((uint32_t)(1<<13)) /* Dynamic Memory Control register EMC: Low-power SDRAM deep-sleep mode (DP)*/


/*----------------------------------------------------------------------------*
  Dynamic Memory Refresh Timer register (EMCDynamicRefresh)
 *----------------------------------------------------------------------------*/

#define EMC_DynamicRefresh_REFRESH(n)   ((uint32_t ) (n & 0x3ff)) /* Dynamic Memory Refresh Timer register EMC: Refresh timer (REFRESH) */


/*----------------------------------------------------------------------------*
  Dynamic Memory Read Configuration register (EMCDynamicReadConfig)
 *----------------------------------------------------------------------------*/

#define EMC_DynamicReadConfig_RD(n)     ((uint32_t )(n & 0x03))  /* EMCDynamicReadConfig register EMC:Read data strategy (RD) */


/*----------------------------------------------------------------------------*
  Dynamic Memory Percentage Command Period register (EMCDynamictRP)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictRP_tRP(n)     ((uint32_t )(n & 0x0f))  /* EMCDynamictRP register EMC: Precharge command period (tRP). */


/*----------------------------------------------------------------------------*
  Dynamic Memory Active to Precharge Command Period register (EMCDynamictRAS)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictRP_tRAS(n)    ((uint32_t )(n & 0x0f))  /* EMCDynamictRAS register EMC: Active to precharge command period (tRAS) */


/*----------------------------------------------------------------------------*
  Dynamic Memory Last Data Out to Active Time register (EMCDynamictAPR)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictAPR_tAPR(n)   ((uint32_t )(n & 0x0f))  /* EMCDynamictAPR register EMC: Last-data-out to active command time (tAPR) */


/*----------------------------------------------------------------------------*
  Dynamic Memory Data-in to Active Command Time register (EMCDynamictDAL)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictDAL_tDAL(n)   ((uint32_t )(n & 0x0f))  /* EMCDynamictDAL register EMC: Data-in to active command (tDAL)*/


/*----------------------------------------------------------------------------*
  Dynamic Memory Write Recovery Time register (EMCDynamictWR)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictWR_tWR(n)     ((uint32_t )(n & 0x0f))  /* EMCDynamictWR register EMC: Write recovery time (tWR)*/


/*----------------------------------------------------------------------------*
  Dynamic Memory Active to Active Command Period register (EMCDynamictRC)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictRC_tRC(n)     ((uint32_t )(n & 0x1f))  /* EMCDynamictRC register EMC: Active to active command period (tRC)*/


/*----------------------------------------------------------------------------*
  Dynamic Memory Auto-refresh Period register (EMCDynamictRFC)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictRFC_tRFC(n)   ((uint32_t )(n & 0x1f))  /* EMCDynamictRFC register EMC: Auto-refresh period and auto-refresh to active command period (tRFC)*/


/*----------------------------------------------------------------------------*
  Dynamic Memory Exit Self-refresh register (EMCDynamictXSR)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictXSR_tXSR(n)   ((uint32_t )(n & 0x1f))  /* EMCDynamictXSR register EMC: Exit self-refresh to active command time (tXSR)*/


/*----------------------------------------------------------------------------*
  Dynamic Memory Active Bank A to Active Bank B Time register (EMCDynamictRRD)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictRRD_tRRD(n)   ((uint32_t )(n & 0x0f))  /* EMCDynamictRRD register EMC: Active bank A to active bank B latency (tRRD )*/


/*----------------------------------------------------------------------------*
  Dynamic Memory Load Mode register to Active Command Time (EMCDynamictMRD)
 *----------------------------------------------------------------------------*/

#define EMC_DynamictMRD_tMRD(n)   ((uint32_t )(n & 0x1f)) /* EMCDynamictMRD register EMC: Load mode register to active command time (tMRD)*/


/*----------------------------------------------------------------------------*
  Static Memory Extended Wait Register (EMCStaticExtendedWait)
 *----------------------------------------------------------------------------*/

#define EMC_StaticExtendedWait_EXTENDEDWAIT(n)   ((uint32_t )(n & 0x3ff)) /* StaticExtendedWait register EMC: External wait time out. */


/*----------------------------------------------------------------------------*
  Dynamic Memory Configuration registers (EMCDynamicConfig0-3)
 *----------------------------------------------------------------------------*/

#define EMC_DynamicConfig_MD(n)   ((uint32_t )(n << 3))   /* DynamicConfig register EMC: Memory device (MD). */
#define EMC_DynamicConfig_AM1(n)  ((uint32_t )(n << 7))   /* DynamicConfig register EMC: Address mapping (AM) */
#define EMC_DynamicConfig_AM2(n)  ((uint32_t )(1 << 14))  /* DynamicConfig register EMC: Address mapping (AM) */
#define EMC_DynamicConfig_B       ((uint32_t )(1 << 19))  /* DynamicConfig register EMC: Buffer enable */
#define EMC_DynamicConfig_P       ((uint32_t )(1 << 20))  /* DynamicConfig register EMC: Write protect (P) */


/*----------------------------------------------------------------------------*
  Dynamic Memory RAS & CAS Delay registers (EMCDynamicRASCAS0-3)
 *----------------------------------------------------------------------------*/

#define EMC_DynamicConfig_RAS(n)  ((uint32_t )(n & 0x03)) /* DynamicRASCAS register EMC: RAS latency (active to read/write delay) (RAS). */
#define EMC_DynamicConfig_CAS(n)  ((uint32_t )(n << 8))   /* DynamicRASCAS register EMC: CAS latency (CAS)*/


/*----------------------------------------------------------------------------*
  Static Memory Configuration registers (EMCStaticConfig0-3)
 *----------------------------------------------------------------------------*/

#define EMC_StaticConfig_MW(n)        ((uint32_t )(n & 0x03))  /* StaticConfig register EMC: Memory width (MW). */
#define EMC_StaticConfig_MW_8BITS     (EMC_StaticConfig_MW(0)) /* StaticConfig register EMC: Memory width 8bit . */
#define EMC_StaticConfig_MW_16BITS    (EMC_StaticConfig_MW(1)) /* StaticConfig register EMC: Memory width 16bit . */
#define EMC_StaticConfig_MW_32BITS    (EMC_StaticConfig_MW(2)) /* StaticConfig register EMC: Memory width 32bit . */
#define EMC_StaticConfig_PM       ((uint32_t )(1 << 3))   /* StaticConfig register EMC: Page mode (PM) */
#define EMC_StaticConfig_PC       ((uint32_t )(1 << 6))   /* StaticConfig register EMC: Chip select polarity (PC) */
#define EMC_StaticConfig_PB       ((uint32_t )(1 << 7))   /* StaticConfig register EMC: Byte lane state (PB) */
#define EMC_StaticConfig_EW       ((uint32_t )(1 << 8))   /* StaticConfig register EMC: Extended wait (EW) */
#define EMC_StaticConfig_B        ((uint32_t )(1 << 19))  /* StaticConfig register EMC: Buffer enable (B) */
#define EMC_StaticConfig_P        ((uint32_t )(1 << 20))  /* StaticConfig register EMC: Write protect (P) */


/*----------------------------------------------------------------------------*
  Static Memory Write Enable Delay registers (EMCStaticWaitWen0-3)
 *----------------------------------------------------------------------------*/

#define EMC_StaticWaitWen_WAITWEN(n)  ((uint32_t )(n & 0x0f)) /* StaticWaitWen register EMC: Wait write enable (WAITWEN). */


/*----------------------------------------------------------------------------*
  Static Memory Output Enable Delay registers (EMCStaticWaitOen0-3)
 *----------------------------------------------------------------------------*/

#define EMC_StaticWaitOen_WAITOEN(n)  ((uint32_t )(n & 0x0f)) /* StaticWaitOen register EMC: Wait output enable (WAITOEN). */


/*----------------------------------------------------------------------------*
  Static Memory Read Delay registers (EMCStaticWaitRd0-3)
 *----------------------------------------------------------------------------*/

#define EMC_StaticWaitRd_WAITRD(n)    ((uint32_t )(n & 0x1f)) /* StaticWaitRd register EMC: Non-page mode read wait
                                                                   states or asynchronous page mode read first access
                                                                   wait state (WAITRD) */



/*----------------------------------------------------------------------------*
  Static Memory Page Mode Read Delay registers (EMCStaticwaitPage0-3)
 *----------------------------------------------------------------------------*/

#define EMC_StaticwaitPage_WAITPAGE(n)    ((uint32_t )(n & 0x1f)) /* StaticwaitPage register EMC: Asynchronous page mode
                                                                     read after the first read wait states (WAITPAGE). */


/*----------------------------------------------------------------------------*
  Static Memory Write Delay registers (EMCStaticWaitwr0-3)
 *----------------------------------------------------------------------------*/

#define EMC_StaticWaitwr_WAITWR(n)        ((uint32_t )(n & 0x1f))     /* StaticWaitwr register EMC: Write wait states (WAITWR). */


/*----------------------------------------------------------------------------*
  Static Memory Turn Round Delay registers (EMCStaticWaitTurn0-3)
 *----------------------------------------------------------------------------*/

#define EMC_StaticWaitTurn_WAITTURN(n)    ((uint32_t )(n & 0x0f)) /* StaticWaitTurn register EMC: Bus turnaround cycles (WAITTURN). */


/*----------------------------------------------------------------------------*
  Delay Control register (EMCDLYCTL)
 *----------------------------------------------------------------------------*/

#define EMC_DLYCTL_CMDDLY(n)      ((uint32_t)(n&0x1F))
#define EMC_DLYCTL_FBCLKDLY(n)    ((uint32_t)((n&0x1F)<<8))
#define EMC_DLYCTL_CLKOUT0DLY(n)  ((uint32_t)((n&0x1F)<<16))
#define EMC_DLYCTL_CLKOUT1DLY(n)  ((uint32_t)((n&0x1F)<<24))


/*----------------------------------------------------------------------------*
  EMC Calibration register (EMCCAL)
 *----------------------------------------------------------------------------*/

#define EMC_CAL_CALVALUE(n)       ((uint32_t)(n&0xFF))
#define EMC_CAL_START             ((uint32_t)(1<<14))
#define EMC_CAL_DONE              ((uint32_t)(1<<15))


/*----------------------------------------------------------------------------*
  EMC endianess modes
 *----------------------------------------------------------------------------*/

#define EMC_LITTLE_ENDIAN_MODE    ((uint32_t)(0))
#define EMC_BIG_ENDIAN_MODE       ((uint32_t)(1))


/*----------------------------------------------------------------------------*
  EMC dynamic memory registers enum
 *----------------------------------------------------------------------------*/

typedef enum
{
  EMC_DYN_MEM_REFRESH_TIMER,
  EMC_DYN_MEM_READ_CONFIG,
  EMC_DYN_MEM_TRP,
  EMC_DYN_MEM_TRAS,
  EMC_DYN_MEM_TSREX,
  EMC_DYN_MEM_TAPR,
  EMC_DYN_MEM_TDAL,
  EMC_DYN_MEM_TWR,
  EMC_DYN_MEM_TRC,
  EMC_DYN_MEM_TRFC,
  EMC_DYN_MEM_TXSR,
  EMC_DYN_MEM_TRRD,
  EMC_DYN_MEM_TMRD
} EMC_DYN_MEM_PAR;


/*----------------------------------------------------------------------------*
  EMC static memory registers enum
 *----------------------------------------------------------------------------*/

typedef enum
{
  EMC_STA_MEM_WAITWEN,
  EMC_STA_MEM_WAITOEN,
  EMC_STA_MEM_WAITRD,
  EMC_STA_MEM_WAITPAGE,
  EMC_STA_MEM_WAITWR,
  EMC_STA_MEM_WAITTURN,
} EMC_STA_MEM_PAR;


/*----------------------------------------------------------------------------*
  Public functions
 *----------------------------------------------------------------------------*/

extern void Lpc177x_8x_EmcInit(void);
extern void Lpc177x_8x_EmcSDRAMAdjustTiming(void);
extern int  Lpc177x_8x_EmcSDRAMCheck(SDRAM sdram, uint32_t offset);
extern void Lpc177x_8x_EmcSDRAMInit(SDRAM sdram, uint32_t dynamic_config);
extern void Lpc177x_8x_EmcConfigEndianMode(uint32_t endian_mode);
extern void Lpc177x_8x_EmcDynCtrlClockEnable(uint32_t clock_enable);
extern void Lpc177x_8x_EmcDynCtrlClockControl(uint32_t clock_control);
extern void Lpc177x_8x_EmcDynCtrlSelfRefresh(uint32_t self_refresh_mode);
extern void Lpc177x_8x_EmcDynCtrlMMC(uint32_t mmc_val);
extern void Lpc177x_8x_EmcDynCtrlSDRAMCmd(uint32_t sdram_command);
extern void Lpc177x_8x_EmcDynCtrlPowerDownMode(uint32_t power_command);
extern void Lpc177x_8x_EmcSetDynMemoryParameter(EMC_DYN_MEM_PAR par, uint32_t val);
extern void Lpc177x_8x_EmcStaticExtendedWait(uint32_t Extended_wait_time_out);
extern void Lpc177x_8x_EmcDynMemConfigMD(uint32_t cs, uint32_t mem_dev);
extern void Lpc177x_8x_EmcDynMemConfigAM(uint32_t cs, uint32_t addr_mapped);
extern void Lpc177x_8x_EmcDynMemConfigB(uint32_t cs, uint32_t buff_control);
extern void Lpc177x_8x_EmcDynMemConfigP(uint32_t cs, uint32_t permission);
extern void Lpc177x_8x_EmcDynMemRAS(uint32_t cs, uint32_t ras_val);
extern void Lpc177x_8x_EmcDynMemCAS(uint32_t cs, uint32_t cas_val);
extern void Lpc177x_8x_EmcStaticMemConfigMW(uint32_t cs, uint32_t mem_width);
extern void Lpc177x_8x_EmcStaticMemConfigPM(uint32_t cs, uint32_t page_mode);
extern void Lpc177x_8x_EmcStaticMemConfigPC(uint32_t cs, uint32_t polarity);
extern void Lpc177x_8x_EmcStaticMemConfigPB(uint32_t cs, uint32_t pb_val);
extern void Lpc177x_8x_EmcStaticMemConfigEW(uint32_t cs, uint32_t ex_wait);
extern void Lpc177x_8x_EmcStaticMemConfigB(uint32_t cs, uint32_t buf_val);
extern void Lpc177x_8x_EmcStaticMemConfigP(uint32_t cs, uint32_t permission);
extern void Lpc177x_8x_EmcSetStaticMemoryParameter(uint32_t cs, EMC_STA_MEM_PAR par, uint32_t val);

#endif /* _LPC177X_8X_EMC_H_ */




