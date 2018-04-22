#ifndef _DEV_SPIBUS_LPC17XX_SSP_H_
#define _DEV_SPIBUS_LPC17XX_SSP_H_
/*
 * Copyright (C) 2013 Simon Budig <simon@budig.de>
 *
 * placed in the public domain
 */


#include <cfg/arch.h>
#if defined(MCU_LPC176x)
#include <arch/cm3/nxp/lpc176x.h>
#elif defined(MCU_LPC177x_8x)
#include <arch/cm3/nxp/lpc177x_8x.h>
#elif defined(MCU_LPC407x_8x)
#include <arch/cm3/nxp/lpc407x_8x.h>
#else
#warning "Unknown LPC familiy"
#endif

/*!
 * \file dev/spibus_lpc17xx_ssp.h
 * \brief LPC17xx specific SSP bus
 *
 * \verbatim
 * $Id:
 * \endverbatim
 */

#include <dev/spibus.h>

extern NUTSPIBUS spiBus0Lpc17xxSsp;
extern NUTSPIBUS spiBus1Lpc17xxSsp;
#if defined(LPC_SSP2_BASE)
extern NUTSPIBUS spiBus2Lpc17xxSsp;
#endif
#endif
