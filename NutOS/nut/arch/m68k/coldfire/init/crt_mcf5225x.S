/*
 * Copyright 2012 by Embedded Technologies s.r.o
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
 */

//------------------------------------------------------------------------------
// Peripheral vectors
//------------------------------------------------------------------------------
.section .vectors,"ax",%progbits

vector40:   .long   IH_DEFAULT
vector41:   .long   IH_DEFAULT
vector42:   .long   IH_DEFAULT
vector43:   .long   IH_DEFAULT
vector44:   .long   IH_DEFAULT
vector45:   .long   IH_DEFAULT
vector46:   .long   IH_DEFAULT
vector47:   .long   IH_DEFAULT
vector48:   .long   IH_DEFAULT
vector49:   .long   IH_DEFAULT
vector4A:   .long   IH_DEFAULT
vector4B:   .long   IH_DEFAULT
vector4C:   .long   IH_DEFAULT
vector4D:   .long   IH_UART0
vector4E:   .long   IH_UART1
vector4F:   .long   IH_UART2
vector50:   .long   IH_DEFAULT
vector51:   .long   IH_I2C0
vector52:   .long   IH_DEFAULT
vector53:   .long   IH_DEFAULT
vector54:   .long   IH_DEFAULT
vector55:   .long   IH_DEFAULT
vector56:   .long   IH_DEFAULT
vector57:   .long   IH_DEFAULT
vector58:   .long   IH_DEFAULT
vector59:   .long   IH_DEFAULT
vector5A:   .long   IH_DEFAULT
vector5B:   .long   IH_DEFAULT
vector5C:   .long   IH_DEFAULT
vector5D:   .long   IH_DEFAULT
vector5E:   .long   IH_DEFAULT
vector5F:   .long   IH_DEFAULT
vector60:   .long   IH_DEFAULT
vector61:   .long   IH_DEFAULT
vector62:   .long   IH_DEFAULT
vector63:   .long   IH_DEFAULT
vector64:   .long   IH_DEFAULT
vector65:   .long   IH_DEFAULT
vector66:   .long   IH_DEFAULT
vector67:   .long   IH_DEFAULT
vector68:   .long   IH_DEFAULT
vector69:   .long   IH_DEFAULT
vector6A:   .long   IH_DEFAULT
vector6B:   .long   IH_DEFAULT
vector6C:   .long   IH_DEFAULT
vector6D:   .long   IH_DEFAULT
vector6E:   .long   IH_DEFAULT
vector6F:   .long   IH_DEFAULT
vector70:   .long   IH_DEFAULT
vector71:   .long   IH_DEFAULT
vector72:   .long   IH_DEFAULT
vector73:   .long   IH_DEFAULT
vector74:   .long   IH_DEFAULT
vector75:   .long   IH_DEFAULT
vector76:   .long   IH_DEFAULT
vector77:   .long   IH_PIT0
vector78:   .long   IH_PIT1
vector79:   .long   IH_DEFAULT
vector7A:   .long   IH_DEFAULT
vector7B:   .long   IH_DEFAULT
vector7C:   .long   IH_DEFAULT
vector7D:   .long   IH_DEFAULT
vector7E:   .long   IH_I2C1
vector7F:   .long   IH_DEFAULT
vector80:   .long   IH_DEFAULT
vector81:   .long   IH_DEFAULT
vector82:   .long   IH_DEFAULT
vector83:   .long   IH_DEFAULT
vector84:   .long   IH_DEFAULT
vector85:   .long   IH_DEFAULT
vector86:   .long   IH_DEFAULT
vector87:   .long   IH_DEFAULT
vector88:   .long   IH_DEFAULT
vector89:   .long   IH_DEFAULT
vector8A:   .long   IH_DEFAULT
vector8B:   .long   IH_DEFAULT
vector8C:   .long   IH_DEFAULT
vector8D:   .long   IH_DEFAULT
vector8E:   .long   IH_DEFAULT
vector8F:   .long   IH_DEFAULT
vector90:   .long   IH_DEFAULT
vector91:   .long   IH_DEFAULT
vector92:   .long   IH_DEFAULT
vector93:   .long   IH_DEFAULT
vector94:   .long   IH_DEFAULT
vector95:   .long   IH_DEFAULT
vector96:   .long   IH_DEFAULT
vector97:   .long   IH_DEFAULT
vector98:   .long   IH_DEFAULT
vector99:   .long   IH_DEFAULT
vector9A:   .long   IH_DEFAULT
vector9B:   .long   IH_DEFAULT
vector9C:   .long   IH_DEFAULT
vector9D:   .long   IH_DEFAULT
vector9E:   .long   IH_DEFAULT
vector9F:   .long   IH_DEFAULT
vectorA0:   .long   IH_DEFAULT
vectorA1:   .long   IH_DEFAULT
vectorA2:   .long   IH_DEFAULT
vectorA3:   .long   IH_DEFAULT
vectorA4:   .long   IH_DEFAULT
vectorA5:   .long   IH_DEFAULT
vectorA6:   .long   IH_DEFAULT
vectorA7:   .long   IH_DEFAULT
vectorA8:   .long   IH_DEFAULT
vectorA9:   .long   IH_DEFAULT
vectorAA:   .long   IH_DEFAULT
vectorAB:   .long   IH_DEFAULT
vectorAC:   .long   IH_DEFAULT
vectorAD:   .long   IH_DEFAULT
vectorAE:   .long   IH_DEFAULT
vectorAF:   .long   IH_DEFAULT
vectorB0:   .long   IH_DEFAULT
vectorB1:   .long   IH_DEFAULT
vectorB2:   .long   IH_DEFAULT
vectorB3:   .long   IH_DEFAULT
vectorB4:   .long   IH_DEFAULT
vectorB5:   .long   IH_DEFAULT
vectorB6:   .long   IH_DEFAULT
vectorB7:   .long   IH_DEFAULT
vectorB8:   .long   IH_DEFAULT
vectorB9:   .long   IH_DEFAULT
vectorBA:   .long   IH_DEFAULT
vectorBB:   .long   IH_DEFAULT
vectorBC:   .long   IH_DEFAULT
vectorBD:   .long   IH_DEFAULT
vectorBE:   .long   IH_DEFAULT
vectorBF:   .long   IH_DEFAULT

//------------------------------------------------------------------------------
// Section 0 - Basic CPU initialization (family dependant code)
//           - The code starts here after reset
//           - After this section, C stack memory must be initialized
//------------------------------------------------------------------------------
	.section .init0,"ax",%progbits

    /* Initialize SCM (System Control Module) */
    move.l	#__rambar+0x221,%d0		/* data + code + enable peripheral access */
    movec.l	%d0,%RAMBAR
    move.l	#__flashbar+0x21,%d0	/* data + code */
    movec.l	%d0,%FLASHBAR
	move.l	#__ipsbar+0x1,%d0
    move.l	%d0,0x40000000

    /* Enable PST[3:0] signals  .. required for program trace support */
    move.l	#0xf,%d0
    move.b	%d0,__ipsbar+0x100074

//------------------------------------------------------------------------------
// Section 12 - CPU and Peripherals Initialization (family dependant code)
//------------------------------------------------------------------------------
    .section .init12,"ax",%progbits
    jsr InitIntramAccess
    jsr InitClock

