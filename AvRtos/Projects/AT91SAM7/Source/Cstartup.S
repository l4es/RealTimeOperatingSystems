
#include "AT91SAM7S256_inc.h"

/* Stack Sizes */
.set  TOP_OF_MEMORY,  (AT91C_ISRAM + AT91C_ISRAM_SIZE)

/* Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs (program status registers) 	 */
.set  MODE_BITS,    0x1F               /* Bit mask for mode bits in CPSR */
.set  ARM_MODE_USR, 0x10            	/* Normal User Mode 											 */
.set  ARM_MODE_FIQ, 0x11            	/* FIQ Processing Fast Interrupts Mode 							 */
.set  ARM_MODE_IRQ, 0x12            	/* IRQ Processing Standard Interrupts Mode 						 */
.set  ARM_MODE_SVC, 0x13            	/* Supervisor Processing Software Interrupts Mode 				 */
.set  ARM_MODE_ABT, 0x17            	/* Abort Processing memory Faults Mode 							 */
.set  ARM_MODE_UND, 0x1B            	/* Undefined Processing Undefined Instructions Mode 			 */
.set  ARM_MODE_SYS, 0x1F            	/* System Running Priviledged Operating System Tasks  Mode		 */
.set  I_BIT, 0x80               		/* when I bit is set, IRQ is disabled (program status registers) */
.set  F_BIT, 0x40               		/* when F bit is set, FIQ is disabled (program status registers) */

/******************************************************************************
 * Init Vectors
 ******************************************************************************/
	 .text			/* all assembler code that follows will go into .text section	 */
	 .arm
	 .align			/* align section on 32-bit boundary								 */
	 .section .vectors, "ax"
	 .global SWI_Handler

__vector:
	b _startup
__undef_handler:
	b __undef_handler
__swi_handler:
	b SWI_Handler
__prefetch_handler:
	b __prefetch_handler
__data_abort_handler:
	b __data_abort_handler
__reserved_handler:
	b __reserved_handler
__irq_handler:
    LDR PC, [PC, #-0xF20]
__fiq_handler:
    LDR PC, [PC, #-0xF20]

/******************************************************************************
 * Reset Handler
 ******************************************************************************/
	 .text			/* all assembler code that follows will go into .text section	 */
	 .align			/* align section on 32-bit boundary								 */
	 .section .init, "ax"
	 .global _startup
	 .extern AT91F_LowLevelInit

_startup:
			ldr     r13,=TOP_OF_MEMORY          /* Temporary stack in internal RAM for Low Level Init execution*/
			ldr     r0,=AT91F_LowLevelInit
			mov     lr, pc
			bx      r0                          /* Branch on C function (with interworking)*/

			mrs     r0,cpsr                             // Original PSR value
			bic     r0,r0,#MODE_BITS                    // Clear the mode bits
			orr     r0,r0,#ARM_MODE_SVC                 // Set SVC mode bits
			msr     cpsr_c,r0                           // Change the mode
			ldr     sp,=__stack_svc_end                 // End of SVC_STACK

			bic     r0,r0,#MODE_BITS                    // Clear the mode bits
			orr     r0,r0,#ARM_MODE_UND                 // Set UND mode bits
			msr     cpsr_c,r0                           // Change the mode
			ldr     sp,=__stack_und_end                 // End of UND_STACK

			bic     r0,r0,#MODE_BITS                    // Clear the mode bits
			orr     r0,r0,#ARM_MODE_ABT                 // Set ABT mode bits
			msr     cpsr_c,r0                           // Change the mode
			ldr     sp,=__stack_abt_end                 // End of ABT_STACK

			bic     r0,r0,#MODE_BITS                    // Clear the mode bits
			orr     r0,r0,#ARM_MODE_FIQ                 // Set FIQ mode bits
			msr     cpsr_c,r0                           // Change the mode
			ldr     sp,=__stack_fiq_end                 // End of FIQ_STACK
			//- Init the FIQ register
            ldr     r8, =AT91C_BASE_AIC

			bic     r0,r0,#MODE_BITS                    // Clear the mode bits
			orr     r0,r0,#ARM_MODE_IRQ                 // Set IRQ mode bits
			msr     cpsr_c,r0                           // Change the mode
			ldr     sp,=__stack_irq_end                 // End of IRQ_STACK

			bic     r0,r0,#MODE_BITS                    // Clear the mode bits
			orr     r0,r0,#ARM_MODE_SYS                 // Set System mode bits
			msr     cpsr_c,r0                           // Change the mode
			ldr     sp,=__stack_sys_end                 // End of CSTACK

// Relocate .data section (Copy from ROM to RAM)
	        LDR     R1, =_etext
	        LDR     R2, =__data_start
	        LDR     R3, =_edata
LoopRel:    CMP     R2, R3
            LDRLO   R0, [R1], #4
            STRLO   R0, [R2], #4
            BLO     LoopRel

// Clear .bss section (Zero init)
            MOV     R0, #0
            LDR     R1, =__bss_start
            LDR     R2, =__bss_end
LoopZI:     CMP     R1, R2
            STRLO   R0, [R1], #4
            BLO     LoopZI

// Add more initialization here
            msr     CPSR_c,#F_BIT | I_BIT | ARM_MODE_SVC

// Enter the C code
			mov   r0,#0            // no arguments (argc = 0)
			mov   r1,r0
			mov   r2,r0
			mov   fp,r0            // null frame pointer
			mov   r7,r0            // null frame pointer for thumb
			ldr   r10,=main
			adr   lr, __main_exit
			bx    r10              // enter main()

__main_exit: B       __main_exit

.end
