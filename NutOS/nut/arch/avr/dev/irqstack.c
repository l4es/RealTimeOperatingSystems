/*
 * Copyright (C) 2004 by egnite Software GmbH. All rights reserved.
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
 */

/*!
 * \file arch/avr/dev/irqstack.c
 * \brief AVR support for dedicated interrupt stack.
 *
 * \verbatim
 * $Id: irqstack.c 4937 2013-01-22 11:38:42Z haraldkipp $
 * \endverbatim
 */

#include <dev/irqreg.h>

#ifdef IRQSTACK_SIZE

static struct
{
    uint8_t _nesting;
    uint8_t _temp;
    uint8_t _SPL;
    uint8_t _SPH;
    uint8_t _stack[IRQSTACK_SIZE];
} _irq_stack;


void _irq_interrupt (IRQ_HANDLER * irh) NUT_NAKED_FUNC;
void _irq_interrupt (IRQ_HANDLER * irh)
{
    asm volatile ("sts %1, r24" "\n\t"
                  "lds r24, %2" "\n\t"      /* load _irq_nesting in r24 */
                  "tst r24" "\n\t"              /* test for zero */
                  "brne L_%=" "\n\t"            /* jump to no_switch1 if not zero */
                  "in r24, __SP_L__" "\n\t"         /* load SP_L into r24 */
                  "sts %3, r24" "\n\t"          /* and save it to _irq_SPL */
                  "in r24, __SP_H__" "\n\t"         /* load SP_H into r24 */
                  "sts %4, r24" "\n\t"          /* and save it to _irq_SPH */
                  "ldi r24, lo8(%0)" "\n\t"         /* load lo addr of begin of irqstack to r24 */
                  "out __SP_L__, r24" "\n\t"            /* write it to SP_L */
                  "ldi r24, hi8(%0)" "\n\t"         /* load hi addr of begin of irqstack to r24 */
                  "out __SP_H__, r24" "\n\t"            /* write it to SP_H */
                  "lds r24, %2" "\n\t"      /* load _irq_nesting to r24 */
                  "L_%=:" "\n\t"                /* jump label */
                  "inc r24" "\n\t"              /* increment r24 */
                  "sts %2, r24" "\n\t"      /* save it back to _irq_nesting */
                  ::
                  "i" (_irq_stack._stack + IRQSTACK_SIZE - 1),
                  "i" (&_irq_stack._temp),
                  "i" (&_irq_stack._nesting),
                  "i" (&_irq_stack._SPL),
                  "i" (&_irq_stack._SPH));
    asm volatile ("push __tmp_reg__" "\n\t"
                  "push __zero_reg__" "\n\t"
                  "clr __zero_reg__" "\n\t"
                  "push r18" "\n\t"
                  "push r19" "\n\t"
                  "push r20" "\n\t"
                  "push r21" "\n\t"
                  "push r22" "\n\t"
                  "push r23" "\n\t"
                  "push r26" "\n\t"
                  "push r27" "\n\t"
                  "push r30" "\n\t"
                  "push r31" "\n\t"::);
    asm volatile ("lds r24, %0" "\n\t"
                  "call CallHandler" "\n\t"::
                  "i" (&_irq_stack._temp));
    asm volatile ("pop r31" "\n\t"
                  "pop r30" "\n\t"
                  "pop r27" "\n\t"
                  "pop r26" "\n\t"
                  "pop r23" "\n\t"
                  "pop r22" "\n\t"
                  "pop r21" "\n\t"
                  "pop r20" "\n\t"
                  "pop r19" "\n\t"
                  "pop r18" "\n\t"
                  "pop __zero_reg__" "\n\t"
                  "pop __tmp_reg__" "\n\t"::);
    asm volatile ("cli" "\n\t"                  /* disable interrupts */
                  "lds r24, %0" "\n\t"      /* load _irq_nesting in r24 */
                  "dec r24" "\n\t"              /* decrement r24 */
                  "sts %0, r24" "\n\t"      /* save it back to _irq_nesting */
                  "brne L_%=" "\n\t"            /* jump to no_switch2 if not zero */
                  "lds r24, %1" "\n\t"          /* load _irq_SPL to r24 */
                  "out __SP_L__, r24" "\n\t"            /* write it to SP_L */
                  "lds r24, %2" "\n\t"          /* load _irq_SPH to r24 */
                  "out __SP_H__, r24" "\n\t"            /* write it to SP_H */
                  "L_%=:" "\n\t"::              /* jump label */
                  "i" (&_irq_stack._nesting),
                  "i" (&_irq_stack._SPL),
                  "i" (&_irq_stack._SPH));
    _irq_epilog
    asm volatile ("reti"::);                /* will enable interrupts */
}


#endif /* #ifdef IRQSTACK_SIZE */
