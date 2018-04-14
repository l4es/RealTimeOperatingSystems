/*
 * Copyright (C) 2004-2005, Dataedge
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
 * 3. Neither the name of the author may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information send an email to luca.bassanello@tin.it
 *
 */

/*! \brief Globaly disable interrupts
 *
 *  Calling this function will globaly disable interrupts and return the interrupts mask register before disabling
 *  interrupts.
 *
 *  \retval Interrupts mask
 */
__inline__ unsigned char INT_Disable(void) {

    unsigned char imask;

    asm("stc ccr,r0l":::"r0");              // Get CCR value
    asm("and.b #0x80,r0l");                 // Mask value
    asm("rotl.b r0l");
    asm("mov.b r0l, %0l":"=g"(imask));      // Assign to imask
    asm("orc.b #0x80,ccr");                 // Disable Interrupt
    return(imask);
}

/*! \brief Globaly enable interrupts
 *
 *  Calling this function will globaly enable interrupts. 
 */
__inline__ void INT_Enable(void) { asm("andc.b #0x7F,ccr"); }

/*! \brief Globaly restore interrupts
 *
 *  Calling this function will restore the interrupts state as were before calling the disable function.
 *  \param flags interrupts state before disabling them
 */
__inline__ void INT_Restore(unsigned char imask) {

    if (imask==0) asm("andc.b #0x7f,ccr");  // Enable interrupts
    else asm("orc.b #0x80, ccr");           // Disable interrupts
}
