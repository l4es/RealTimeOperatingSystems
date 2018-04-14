
#if defined (__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
extern "C" {
    extern void __libc_init_array(void);
}
#endif
#endif

#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

//*****************************************************************************
// Declaration of external SystemInit function
extern void SystemInit(void);

//*****************************************************************************
//
// Forward declaration of the default handlers. These are aliased.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions
//
//*****************************************************************************
void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

//*****************************************************************************
//
// Forward declaration of the specific IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the application
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//
//*****************************************************************************
/* Device specific interrupts */
void DMA0_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA1_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA2_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA3_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_Error_IRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved21_IRQHandler(void) ALIAS(IntDefaultHandler);
void FTFL_IRQHandler(void) ALIAS(IntDefaultHandler);
void Read_Collision_IRQHandler(void) ALIAS(IntDefaultHandler);
void LVD_LVW_IRQHandler(void) ALIAS(IntDefaultHandler);
void LLW_IRQHandler(void) ALIAS(IntDefaultHandler);
void Watchdog_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C0_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI0_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2S0_Tx_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2S0_Rx_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_LON_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_RX_TX_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_ERR_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_RX_TX_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_ERR_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_RX_TX_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_ERR_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC0_IRQHandler(void) ALIAS(IntDefaultHandler);
void CMP0_IRQHandler(void) ALIAS(IntDefaultHandler);
void CMP1_IRQHandler(void) ALIAS(IntDefaultHandler);
void FTM0_IRQHandler(void) ALIAS(IntDefaultHandler);
void FTM1_IRQHandler(void) ALIAS(IntDefaultHandler);
void CMT_IRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_IRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_Seconds_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIT0_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIT2_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIT3_IRQHandler(void) ALIAS(IntDefaultHandler);
void PDB0_IRQHandler(void) ALIAS(IntDefaultHandler);
void USB0_IRQHandler(void) ALIAS(IntDefaultHandler);
void USBDCD_IRQHandler(void) ALIAS(IntDefaultHandler);
void TSI0_IRQHandler(void) ALIAS(IntDefaultHandler);
void MCG_IRQHandler(void) ALIAS(IntDefaultHandler);
void LPTimer_IRQHandler(void) ALIAS(IntDefaultHandler);
void PORTA_IRQHandler(void) ALIAS(IntDefaultHandler);
void PORTB_IRQHandler(void) ALIAS(IntDefaultHandler);
void PORTC_IRQHandler(void) ALIAS(IntDefaultHandler);
void PORTD_IRQHandler(void) ALIAS(IntDefaultHandler);
void PORTE_IRQHandler(void) ALIAS(IntDefaultHandler);
void SWI_IRQHandler(void) ALIAS(IntDefaultHandler);

//*****************************************************************************
//
// The entry point for the application.
// main() is the entry point for Newlib based applications
//
//*****************************************************************************
extern int main(void);

//*****************************************************************************
//
// External declaration for the pointer to the stack top from the Linker Script
//
//*****************************************************************************
extern void _vStackTop(void);

//*****************************************************************************
#if defined (__cplusplus)
} // extern "C"
#endif
//*****************************************************************************
//
// The vector table.
// This relies on the linker script to place at correct location in memory.
//
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);
__attribute__ ((section(".isr_vector"))) void (* const g_pfnVectors[])(void) =
{
	// Core Level - CM4
	&_vStackTop,                       // -16 The initial stack pointer
	ResetISR,                          // -15 The reset handler
	NMI_Handler,                       // -14 The NMI handler
	HardFault_Handler,                 // -13 The hard fault handler
	MemManage_Handler,                 // -12 The MPU fault handler
	BusFault_Handler,                  // -11 The bus fault handler
	UsageFault_Handler,                // -10 The usage fault handler
	0,                                 // -9  Reserved
	0,                                 // -8  Reserved
	0,                                 // -7  Reserved
	0,                                 // -6  Reserved
	SVC_Handler,                       // -5  SVCall handler
	DebugMon_Handler,                  // -4  Debug monitor handler
	0,                                 // -3  Reserved
	PendSV_Handler,                    // -2  The PendSV handler
	SysTick_Handler,                   // -1  The SysTick handler

	/* Device specific interrupts */
	DMA0_IRQHandler           ,         // 0   DMA channel 0 transfer complete interrupt */
	DMA1_IRQHandler           ,         // 1   DMA channel 1 transfer complete interrupt */
	DMA2_IRQHandler           ,         // 2   DMA channel 2 transfer complete interrupt */
	DMA3_IRQHandler           ,         // 3   DMA channel 3 transfer complete interrupt */
	DMA_Error_IRQHandler      ,         // 4   DMA error interrupt */
	Reserved21_IRQHandler     ,         // 5   Reserved interrupt 21 */
	FTFL_IRQHandler           ,         // 6   FTFL interrupt */
	Read_Collision_IRQHandler ,         // 7   Read collision interrupt */
	LVD_LVW_IRQHandler        ,         // 8   Low Voltage Detect, Low Voltage Warning */
	LLW_IRQHandler            ,         // 9   Low Leakage Wakeup */
	Watchdog_IRQHandler       ,         // 10  WDOG interrupt */
	I2C0_IRQHandler           ,         // 11  I2C0 interrupt */
	SPI0_IRQHandler           ,         // 12  SPI0 interrupt */
	I2S0_Tx_IRQHandler        ,         // 13  I2S0 transmit interrupt */
	I2S0_Rx_IRQHandler        ,         // 14  I2S0 receive interrupt */
	UART0_LON_IRQHandler      ,         // 15  UART0 LON interrupt */
	UART0_RX_TX_IRQHandler    ,         // 16  UART0 receive/transmit interrupt */
	UART0_ERR_IRQHandler      ,         // 17  UART0 error interrupt */
	UART1_RX_TX_IRQHandler    ,         // 18  UART1 receive/transmit interrupt */
	UART1_ERR_IRQHandler      ,         // 19  UART1 error interrupt */
	UART2_RX_TX_IRQHandler    ,         // 20  UART2 receive/transmit interrupt */
	UART2_ERR_IRQHandler      ,         // 21  UART2 error interrupt */
	ADC0_IRQHandler           ,         // 22  ADC0 interrupt */
	CMP0_IRQHandler           ,         // 23  CMP0 interrupt */
	CMP1_IRQHandler           ,         // 24  CMP1 interrupt */
	FTM0_IRQHandler           ,         // 25  FTM0 fault, overflow and channels interrupt */
	FTM1_IRQHandler           ,         // 26  FTM1 fault, overflow and channels interrupt */
	CMT_IRQHandler            ,         // 27  CMT interrupt */
	RTC_IRQHandler            ,         // 28  RTC interrupt */
	RTC_Seconds_IRQHandler    ,         // 29  RTC seconds interrupt */
	PIT0_IRQHandler           ,         // 30  PIT timer channel 0 interrupt */
	PIT1_IRQHandler           ,         // 31  PIT timer channel 1 interrupt */
	PIT2_IRQHandler           ,         // 32  PIT timer channel 2 interrupt */
	PIT3_IRQHandler           ,         // 33  PIT timer channel 3 interrupt */
	PDB0_IRQHandler           ,         // 34  PDB0 interrupt */
	USB0_IRQHandler           ,         // 35  USB0 interrupt */
	USBDCD_IRQHandler         ,         // 36  USBDCD interrupt */
	TSI0_IRQHandler           ,         // 37  TSI0 interrupt */
	MCG_IRQHandler            ,         // 38  MCG interrupt */
	LPTimer_IRQHandler        ,         // 39  LPTimer interrupt */
	PORTA_IRQHandler          ,         // 40  Port A interrupt */
	PORTB_IRQHandler          ,         // 41  Port B interrupt */
	PORTC_IRQHandler          ,         // 42  Port C interrupt */
	PORTD_IRQHandler          ,         // 43  Port D interrupt */
	PORTE_IRQHandler          ,         // 44  Port E interrupt */
	SWI_IRQHandler            ,         // 45	 Software interrupt */

}; /* End of g_pfnVectors */


__attribute__ ((section(".flash_security"))) const unsigned char flash_security[] =
{
/* Flash Configuration
   16-byte flash configuration field that stores default protection settings (loaded on reset)
   and security information that allows the MCU to restrict acces to the FTFL module.
   Backdoor Comparison Key */
   0xFF,  /* BackDoorK0 */
   0xFF,  /* BackDoorK1 */
   0xFF,  /* BackDoorK2 */
   0xFF,  /* BackDoorK3 */
   0xFF,  /* BackDoorK4 */
   0xFF,  /* BackDoorK5 */
   0xFF,  /* BackDoorK6 */
   0xFF,  /* BackDoorK7 */

/* Program flash protection bytes (FPROT)
   Each program flash region can be protected from program and erase operation by setting the associated PROT bit.
   Each bit protects a 1/32 region of the program flash memory. */
   0xFF,  /* FPROT0 1/32 - 8/32 region */
   0xFF,  /* FPROT1 9/32 - 16/32 region */
   0xFF,  /* FPROT2 17/32 - 24/32 region */
   0xFF,  /* FPROT3 25/32 - 32/32 region */

/* Flash security byte (FSEC) */
   0xFE,

/* Flash nonvolatile option byte (FOPT) */
   0xFF,

/* EEPROM protection byte (FEPROT) */
   0xFF,

/* Data flash protection byte (FDPROT) */
   0xFF,

};

//*****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len)
{
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int *pulSrc = (unsigned int*) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors")))
void bss_init(unsigned int start, unsigned int len)
{
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
//*****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;


//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void ResetISR(void)
{
    //
    // Copy the data sections from flash to SRAM.
    //
    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
    while (SectionTableAddr < &__data_section_table_end)
    {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        data_init(LoadAddr, ExeAddr, SectionLen);
    }
    // At this point, SectionTableAddr = &__bss_section_table;
    // Zero fill the bss segment
    while (SectionTableAddr < &__bss_section_table_end)
    {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        bss_init(ExeAddr, SectionLen);
    }

    SystemInit();

#if defined (__cplusplus)
    //
    // Call C++ library initialisation
    //
    __libc_init_array();
#endif

    main();

    //
    // main() shouldn't return, but if it does, we'll just enter an infinite loop
    //
    while (1) {
        ;
    }
}

//*****************************************************************************
// Default exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void NMI_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void HardFault_Handler(void)
{
   // Configurable Fault Status Register
   // Consists of MMSR, BFSR and UFSR
   unsigned int _CFSR = *((unsigned int*)0xE000ED28);
   // Hard Fault Status Register
   unsigned int _HFSR = *((unsigned int*)0xE000ED2C);
   // Debug Fault Status Register
   unsigned int _DFSR = *((unsigned int*)0xE000ED30);
   // Auxiliary Fault Status Register
   unsigned int _AFSR = *((unsigned int*)0xE000ED3C);
   // Read the Fault Address Registers. These may not contain valid values.
   // Check BFARVALID/MMARVALID to see if they are valid values
   // MemManage Fault Address Register
   unsigned int _MMAR = *((unsigned int*)0xE000ED34);
   // Bus Fault Address Register
   unsigned int _BFAR = *((unsigned int*)0xE000ED38);

   (void)_CFSR;
   (void)_HFSR;
   (void)_DFSR;
   (void)_AFSR;
   (void)_MMAR;
   (void)_BFAR;

   while(1) {}
}

__attribute__ ((section(".after_vectors")))
void MemManage_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void BusFault_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void UsageFault_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void SVC_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void DebugMon_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void PendSV_Handler(void)
{ while(1) {}
}

__attribute__ ((section(".after_vectors")))
void SysTick_Handler(void)
{ while(1) {}
}

//*****************************************************************************
//
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void IntDefaultHandler(void)
{ while(1) {}
}



