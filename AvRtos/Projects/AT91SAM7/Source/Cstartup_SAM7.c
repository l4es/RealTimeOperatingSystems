/***************************************************************************
* $Source: D:/Entwicklung/CVS/NetworkAnalyser/Source/Cstartup_SAM7.c,v $
* $Revision: 1.1 $
* $Author: Harald $
* $Date: 2008/10/15 16:28:58 $
*
* Module: Main
*
* Copyright (c) 2008, Harald Baumeister, Döggingen
* All rights reserved.
*
****************************************************************************/
//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : Cstartup_SAM7.c
//* Object              : Low level initializations written in C for Tools
//* Creation            : 12/Jun/04
//* 1.2   28/Feb/05 JPP : LIB change AT91C_WDTC_WDDIS & PLL
//* 1.3   21/Mar/05 JPP : Change PLL Wait time
//* 1.4   21/Aug/05 JPP : Change MC_FMR Setting
//* 1.5   29/Aug/05 JPP : Change PLL error
//* 1.6   13/oct/05 JPP : Change comment
//*----------------------------------------------------------------------------

/*****************************************************************************
 * 1. includes
 *****************************************************************************/
/*****************************************************************************
 * foreign header files
 *****************************************************************************/
#include "AT91SAM7S256.h"

/*****************************************************************************
 * own header files
 *****************************************************************************/
/*****************************************************************************
 * 2. file local constants/definitions
 *****************************************************************************/
/*****************************************************************************
 * 3. global variables
 *****************************************************************************/
unsigned int AT91F_DataAbortPC;         /* Program counter for data abort exceptions */
unsigned int AT91F_PrefetchAbortPC;     /* Program counter for prefetch abort exceptions */

/*****************************************************************************
 * 4. file local typedefs
 *****************************************************************************/
/*****************************************************************************
 * 5. file local variables
 *****************************************************************************/
/*****************************************************************************
 * 6. file local macros
 *****************************************************************************/
/*****************************************************************************
 * 7. file local function prototypes
 *****************************************************************************/
/*****************************************************************************
 * 8. file local functions
 *****************************************************************************/
/*****************************************************************************
 * 9. exported functions
 *****************************************************************************/
//  The following functions must be write in ARM mode this function called
// directly by exception vector
void AT91F_DataAbort_Handler(void) __attribute__ ((section (".init")));
void AT91F_PrefetchAbort_Handler(void) __attribute__ ((section (".init")));
void AT91F_Spurious_handler(void) __attribute__ ((section (".init")));
void AT91F_Default_IRQ_handler(void) __attribute__ ((section (".init")));
void AT91F_Default_FIQ_handler(void) __attribute__ ((section (".init")));
void AT91F_LowLevelInit( void ) __attribute__ ((section (".init")));

//*----------------------------------------------------------------------------
//* \fn    AT91F_Default_IRQ_handler
//* \brief
//*----------------------------------------------------------------------------
void AT91F_Default_IRQ_handler( void )
{
    while( 1 );
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_Default_FIQ_handler
//* \brief
//*----------------------------------------------------------------------------
void AT91F_Default_FIQ_handler( void )
{
    while( 1 );
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_DataAbort_Handler
//* \brief
//*----------------------------------------------------------------------------
void AT91F_DataAbort_Handler( void )
{
    while( 1 );
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_Default_IRQ_handler
//* \brief
//*----------------------------------------------------------------------------
void AT91F_PrefetchAbort_Handler( void )
{
    while( 1 );
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_Spurious_handler
//* \brief
//*----------------------------------------------------------------------------
void AT91F_Spurious_handler( void )
{
   while( 1 );
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_LowLevelInit
//* \brief This function performs very low level HW initialization
//*        this function can use a Stack, depending the compilation
//*        optimization mode
//*----------------------------------------------------------------------------
void AT91F_LowLevelInit( void )
{
    unsigned char i;

    ///////////////////////////////////////////////////////////////////////////
    // EFC Init
    ///////////////////////////////////////////////////////////////////////////
    AT91C_BASE_MC->MC_FMR = AT91C_MC_FWS_1FWS ;

    ///////////////////////////////////////////////////////////////////////////
    // Init PMC Step 1. Enable Main Oscillator
    // Main Oscillator startup time is board specific:
    // Main Oscillator Startup Time worst case (3MHz) corresponds to 15ms
    // (0x40 for AT91C_CKGR_OSCOUNT field)
    ///////////////////////////////////////////////////////////////////////////
    AT91C_BASE_PMC->PMC_MOR = ((( AT91C_CKGR_OSCOUNT & (0x40 <<8)) | AT91C_CKGR_MOSCEN ));
    // Wait Main Oscillator stabilization
    while(!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS));

    ///////////////////////////////////////////////////////////////////////////
    // Init PMC Step 2.
    // Set PLL to 100MHz and UDP Clock to 50MHz
    // PLL Startup time depends on PLL RC filter: worst case is choosen
    ///////////////////////////////////////////////////////////////////////////
    AT91C_BASE_PMC->PMC_PLLR = AT91C_CKGR_USBDIV_1 |            // USB divider is PLL clock output divided by 2
                               (16 << 8)           |            // Set slow clock periodes to 16 after that the Lock bit will be set
                               (AT91C_CKGR_MUL & (4 << 16)) |   // Multiplier pll input clock with 4
                               (AT91C_CKGR_DIV & 1);            // Set the PLL divider to 1
    // Wait for PLL stabilization
    while( !(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK) );
    // Wait until the master clock is established for the case we already
    // turn on the PLL
    while( !(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY) );

    ///////////////////////////////////////////////////////////////////////////
    // Init PMC Step 3.
    // Selection of Master Clock MCK equal to (Processor Clock PCK) PLL/2=50MHz
    // The PMC_MCKR register must not be programmed in a single write operation
    // (see. Product Errata Sheet)
    ///////////////////////////////////////////////////////////////////////////
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;
    // Wait until the master clock is established
    while( !(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY) );

    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK;
    // Wait until the master clock is established
    while( !(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY) );

    ///////////////////////////////////////////////////////////////////////////
    //  Disable Watchdog (write once register)
    ///////////////////////////////////////////////////////////////////////////
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    ///////////////////////////////////////////////////////////////////////////
    // Enable User Reset and set its minimal assertion to 960 us
    ///////////////////////////////////////////////////////////////////////////
	AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4<<8) | (unsigned int) (0xA5<<24);

    ///////////////////////////////////////////////////////////////////////////
    //  Init AIC: assign corresponding handler for each interrupt source
    ///////////////////////////////////////////////////////////////////////////
    AT91C_BASE_AIC->AIC_SVR[0] = (int) AT91F_Default_FIQ_handler ;

    for (i = 1; i < 31; i++)
    {
        AT91C_BASE_AIC->AIC_SVR[i] = (int) AT91F_Default_IRQ_handler ;
    }
    AT91C_BASE_AIC->AIC_SPU = (unsigned int) AT91F_Spurious_handler;

//    AT91C_BASE_MC->MC_RCR = AT91C_MC_RCB;
/*
    ///////////////////////////////////////////////////////////////////////////
    //  Copy startvector to ram and remap
    ///////////////////////////////////////////////////////////////////////////
    startVectRam = (unsigned int*)0x000200000U;
    startVectFlash = (unsigned int*)__section_begin(".intvec");

    *startVectRam = 0xA5A5A5A5U;

    if( *((uint32_t*)0x0U) != 0xA5A5A5A5U )
    {
        for( i = 0; i < 0x80U; i++ )
            *startVectRam++ = *startVectFlash++;
        HWAL_REMAP();
    }
    else
    {
        for( i = 0; i < 0x80U; i++ )
            *startVectRam++ = *startVectFlash++;
    }
*/
}

/***************************************************************************
* $Log: Cstartup_SAM7.c,v $
* Revision 1.1  2008/10/15 16:28:58  Harald
* Initial revision
*
*
****************************************************************************/
