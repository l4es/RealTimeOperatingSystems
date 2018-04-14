/*
 int.h - definitions for interrupt management
  
 Author:        Paul Barker
 Part of:       COS
 Created:       15/04/04
 Last Modified: 02/09/04

 This stuff is taken from GeekOS, with modifications
*/

/*
02/09/04:	Changed name from idt.h
*/

#ifndef _COS_INT_H_
#define _COS_INT_H_

// We'll handle all possible interrupts.
#define IDT_NENTRIES 256

// Exceptions range from 0-19
#define IDT_FIRST_EXCEPTION 0
#define IDT_NUM_EXCEPTIONS 20

// External IRQs range from 32-47
#define IDT_FIRST_EXTERNAL 32
#define IDT_NUM_EXTERNAL 16

//
// This struct reflects the contents of the stack when
// a C interrupt handler function is called.
// It must be kept up to date with the code in "lowlevel.asm".
//
typedef struct interrupt_state {
    // The register contents at the time of the exception.
    // We save these explicitly.
    register_t gs;
    register_t fs;
    register_t es;
    register_t ds;
    register_t ebp;
    register_t edi;
    register_t esi;
    register_t edx;
    register_t ecx;
    register_t ebx;
    register_t eax;

    // We explicitly push the interrupt number.
    // This makes it easy for the handler function to determine
    // which interrupt occurred.
    register_t intNum;

    // This may be pushed by the processor; if not, we push
    // a dummy error code, so the stack layout is the same
    // for every type of interrupt.
    register_t errorCode;

    // These are always pushed on the stack by the processor.
    register_t eip;
    register_t cs;
    register_t eflags;
}
interrupt_state_t;

typedef struct interrupt_gate {
	word_t offsetLow;
	word_t segmentSelector;
	dword_t reserved : 5;
	dword_t signature : 8;
	dword_t dpl : 2;
	dword_t present : 1;
	word_t offsetHigh;
}
interrupt_gate_t;

typedef union idt_entry {
	interrupt_gate_t ig;
	// In theory we could have members for trap gates
	// and task gates if we wanted.
}
idt_entry_t;

typedef void (*interrupt_handler_t)(interrupt_state_t* state);

// global functions
void install_handler(u8_t i, interrupt_handler_t hand);
void enable_irq(u8_t irq, bool_t enable);

#define irq2int(xx) (xx + IDT_FIRST_EXTERNAL)

// this is used in a few places
#define MASTER_PIC_PORT 0x20

#endif // !_COS_GDT_H_

