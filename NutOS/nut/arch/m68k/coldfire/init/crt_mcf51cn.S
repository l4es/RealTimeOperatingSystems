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

vector40:   .long   IH_DEFAULT                  // 0x100 IRQ_pin
vector41:   .long   IH_DEFAULT                  // 0x104 Low_voltage_detect
vector42:   .long   IH_DEFAULT                  // 0x108 MCG_lock
vector43:   .long   IH_DEFAULT                  // 0x10C TPM1_ch0
vector44:   .long   IH_DEFAULT                  // 0x110 TPM1_ch1
vector45:   .long   IH_DEFAULT                  // 0x114 TPM1_ch2
vector46:   .long   IH_DEFAULT                  // 0x118 TPM1_ovfl
vector47:   .long   IH_DEFAULT                  // 0x11C MTIM1_ovfl
vector48:   .long   IH_DEFAULT                  // 0x120 TPM2_ch0
vector49:   .long   IH_DEFAULT                  // 0x124 TPM2_ch1
vector4A:   .long   IH_DEFAULT                  // 0x128 TPM2_ch2
vector4B:   .long   IH_DEFAULT                  // 0x12C TPM2_ovfl
vector4C:   .long   IH_DEFAULT                  // 0x130 SPI1
vector4D:   .long   IH_DEFAULT                  // 0x134 SPI2
vector4E:   .long   IH_DEFAULT                  // 0x138 MTIM2_ovfl
vector4F:   .long   IH_DEFAULT                  // 0x13C SCI1_err
vector50:   .long   IH_DEFAULT                  // 0x140 SCI1_rx
vector51:   .long   IH_DEFAULT                  // 0x144 SCI1_tx
vector52:   .long   IH_DEFAULT                  // 0x148 SCI2_err
vector53:   .long   IH_DEFAULT                  // 0x14C SCI2_rx
vector54:   .long   IH_DEFAULT                  // 0x150 SCI2_tx
vector55:   .long   IH_DEFAULT                  // 0x154 SCI3_OR1
vector56:   .long   IH_DEFAULT                  // 0x158 FEC TXF
vector57:   .long   IH_DEFAULT                  // 0x15C FEC RXF
vector58:   .long   IH_DEFAULT                  // 0x160 FEC Other
vector59:   .long   IH_DEFAULT                  // 0x164 FEC HBERR
vector5A:   .long   IH_DEFAULT                  // 0x168 FEC BABR
vector5B:   .long   IH_DEFAULT                  // 0x16C FEC BABT
vector5C:   .long   IH_DEFAULT                  // 0x170 FEC GRA
vector5D:   .long   IH_DEFAULT                  // 0x174 FEC TXB
vector5E:   .long   IH_DEFAULT                  // 0x178 FEC RXB
vector5F:   .long   IH_DEFAULT                  // 0x17C FEC MII
vector60:   .long   IH_DEFAULT                  // 0x180 FEC EBERR
vector61:   .long   IH_DEFAULT                  // 0x184 FEC LC
vector62:   .long   IH_DEFAULT                  // 0x188 FEC RL
vector63:   .long   IH_DEFAULT                  // 0x18C FEC UN
vector64:   .long   IH_DEFAULT                  // 0x190 SCI3_err
vector65:   .long   IH_DEFAULT                  // 0x194 SCI3_rx
vector66:   .long   IH_DEFAULT                  // 0x198 SCI3_tx
vector67:   .long   IH_DEFAULT                  // 0x19C Level 7
vector68:   .long   IH_DEFAULT                  // 0x1A0 Level 6
vector69:   .long   IH_DEFAULT                  // 0x1A4 Level 5
vector6A:   .long   IH_DEFAULT                  // 0x1A8 Level 4
vector6B:   .long   IH_DEFAULT                  // 0x1AC Level 3
vector6C:   .long   IH_DEFAULT                  // 0x1B0 Level 2
vector6D:   .long   IH_DEFAULT                  // 0x1B4 Level 1
vector6E:   .long   IH_DEFAULT                  // 0x1B8 IIC1
vector6F:   .long   IH_DEFAULT                  // 0x1BC IIC2
vector70:   .long   IH_DEFAULT                  // 0x1C0 ADC1
vector71:   .long   IH_DEFAULT                  // 0x1C4 KBI1
vector72:   .long   IH_DEFAULT                  // 0x1C8 RTC

//------------------------------------------------------------------------------
// Section 0 - Basic CPU initialization (family dependant code)
//           - The code starts here after reset
//           - After this section, C stack memory must be initialized
//------------------------------------------------------------------------------
	.section .init0,"ax",%progbits

//------------------------------------------------------------------------------
// Section 12 - CPU and Peripherals Initialization (family dependant code)
//------------------------------------------------------------------------------
    .section .init12,"ax",%progbits
    jsr InitClock

