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
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * additional information see http://www.ethernut.de/
 */

#ifndef MCF5225X_INTC_H_
#define MCF5225X_INTC_H_

/* INTC Registers */
#define MCF_INTC_IPRH(x)                     (*(volatile uint32_t*)(0x40000C00 + ((x) * 0x100)))
#define MCF_INTC_IPRL(x)                     (*(volatile uint32_t*)(0x40000C04 + ((x) * 0x100)))
#define MCF_INTC_IMRH(x)                     (*(volatile uint32_t*)(0x40000C08 + ((x) * 0x100)))
#define MCF_INTC_IMRL(x)                     (*(volatile uint32_t*)(0x40000C0C + ((x) * 0x100)))
#define MCF_INTC_INTFRCH(x)                  (*(volatile uint32_t*)(0x40000C10 + ((x) * 0x100)))
#define MCF_INTC_INTFRCL(x)                  (*(volatile uint32_t*)(0x40000C14 + ((x) * 0x100)))
#define MCF_INTC_IRLR(x)                     (*(volatile uint8_t *)(0x40000C18 + ((x) * 0x100)))
#define MCF_INTC_IACKLPR(x)                  (*(volatile uint8_t *)(0x40000C19 + ((x) * 0x100)))
#define MCF_INTC_ICR01(x)                    (*(volatile uint8_t *)(0x40000C41 + ((x) * 0x100)))
#define MCF_INTC_ICR02(x)                    (*(volatile uint8_t *)(0x40000C42 + ((x) * 0x100)))
#define MCF_INTC_ICR03(x)                    (*(volatile uint8_t *)(0x40000C43 + ((x) * 0x100)))
#define MCF_INTC_ICR04(x)                    (*(volatile uint8_t *)(0x40000C44 + ((x) * 0x100)))
#define MCF_INTC_ICR05(x)                    (*(volatile uint8_t *)(0x40000C45 + ((x) * 0x100)))
#define MCF_INTC_ICR06(x)                    (*(volatile uint8_t *)(0x40000C46 + ((x) * 0x100)))
#define MCF_INTC_ICR07(x)                    (*(volatile uint8_t *)(0x40000C47 + ((x) * 0x100)))
#define MCF_INTC_ICR08(x)                    (*(volatile uint8_t *)(0x40000C48 + ((x) * 0x100)))
#define MCF_INTC_ICR09(x)                    (*(volatile uint8_t *)(0x40000C49 + ((x) * 0x100)))
#define MCF_INTC_ICR10(x)                    (*(volatile uint8_t *)(0x40000C4A + ((x) * 0x100)))
#define MCF_INTC_ICR11(x)                    (*(volatile uint8_t *)(0x40000C4B + ((x) * 0x100)))
#define MCF_INTC_ICR12(x)                    (*(volatile uint8_t *)(0x40000C4C + ((x) * 0x100)))
#define MCF_INTC_ICR13(x)                    (*(volatile uint8_t *)(0x40000C4D + ((x) * 0x100)))
#define MCF_INTC_ICR14(x)                    (*(volatile uint8_t *)(0x40000C4E + ((x) * 0x100)))
#define MCF_INTC_ICR15(x)                    (*(volatile uint8_t *)(0x40000C4F + ((x) * 0x100)))
#define MCF_INTC_ICR16(x)                    (*(volatile uint8_t *)(0x40000C50 + ((x) * 0x100)))
#define MCF_INTC_ICR17(x)                    (*(volatile uint8_t *)(0x40000C51 + ((x) * 0x100)))
#define MCF_INTC_ICR18(x)                    (*(volatile uint8_t *)(0x40000C52 + ((x) * 0x100)))
#define MCF_INTC_ICR19(x)                    (*(volatile uint8_t *)(0x40000C53 + ((x) * 0x100)))
#define MCF_INTC_ICR20(x)                    (*(volatile uint8_t *)(0x40000C54 + ((x) * 0x100)))
#define MCF_INTC_ICR21(x)                    (*(volatile uint8_t *)(0x40000C55 + ((x) * 0x100)))
#define MCF_INTC_ICR22(x)                    (*(volatile uint8_t *)(0x40000C56 + ((x) * 0x100)))
#define MCF_INTC_ICR23(x)                    (*(volatile uint8_t *)(0x40000C57 + ((x) * 0x100)))
#define MCF_INTC_ICR24(x)                    (*(volatile uint8_t *)(0x40000C58 + ((x) * 0x100)))
#define MCF_INTC_ICR25(x)                    (*(volatile uint8_t *)(0x40000C59 + ((x) * 0x100)))
#define MCF_INTC_ICR26(x)                    (*(volatile uint8_t *)(0x40000C5A + ((x) * 0x100)))
#define MCF_INTC_ICR27(x)                    (*(volatile uint8_t *)(0x40000C5B + ((x) * 0x100)))
#define MCF_INTC_ICR28(x)                    (*(volatile uint8_t *)(0x40000C5C + ((x) * 0x100)))
#define MCF_INTC_ICR29(x)                    (*(volatile uint8_t *)(0x40000C5D + ((x) * 0x100)))
#define MCF_INTC_ICR30(x)                    (*(volatile uint8_t *)(0x40000C5E + ((x) * 0x100)))
#define MCF_INTC_ICR31(x)                    (*(volatile uint8_t *)(0x40000C5F + ((x) * 0x100)))
#define MCF_INTC_ICR32(x)                    (*(volatile uint8_t *)(0x40000C60 + ((x) * 0x100)))
#define MCF_INTC_ICR33(x)                    (*(volatile uint8_t *)(0x40000C61 + ((x) * 0x100)))
#define MCF_INTC_ICR34(x)                    (*(volatile uint8_t *)(0x40000C62 + ((x) * 0x100)))
#define MCF_INTC_ICR35(x)                    (*(volatile uint8_t *)(0x40000C63 + ((x) * 0x100)))
#define MCF_INTC_ICR36(x)                    (*(volatile uint8_t *)(0x40000C64 + ((x) * 0x100)))
#define MCF_INTC_ICR37(x)                    (*(volatile uint8_t *)(0x40000C65 + ((x) * 0x100)))
#define MCF_INTC_ICR38(x)                    (*(volatile uint8_t *)(0x40000C66 + ((x) * 0x100)))
#define MCF_INTC_ICR39(x)                    (*(volatile uint8_t *)(0x40000C67 + ((x) * 0x100)))
#define MCF_INTC_ICR40(x)                    (*(volatile uint8_t *)(0x40000C68 + ((x) * 0x100)))
#define MCF_INTC_ICR41(x)                    (*(volatile uint8_t *)(0x40000C69 + ((x) * 0x100)))
#define MCF_INTC_ICR42(x)                    (*(volatile uint8_t *)(0x40000C6A + ((x) * 0x100)))
#define MCF_INTC_ICR43(x)                    (*(volatile uint8_t *)(0x40000C6B + ((x) * 0x100)))
#define MCF_INTC_ICR44(x)                    (*(volatile uint8_t *)(0x40000C6C + ((x) * 0x100)))
#define MCF_INTC_ICR45(x)                    (*(volatile uint8_t *)(0x40000C6D + ((x) * 0x100)))
#define MCF_INTC_ICR46(x)                    (*(volatile uint8_t *)(0x40000C6E + ((x) * 0x100)))
#define MCF_INTC_ICR47(x)                    (*(volatile uint8_t *)(0x40000C6F + ((x) * 0x100)))
#define MCF_INTC_ICR48(x)                    (*(volatile uint8_t *)(0x40000C70 + ((x) * 0x100)))
#define MCF_INTC_ICR49(x)                    (*(volatile uint8_t *)(0x40000C71 + ((x) * 0x100)))
#define MCF_INTC_ICR50(x)                    (*(volatile uint8_t *)(0x40000C72 + ((x) * 0x100)))
#define MCF_INTC_ICR51(x)                    (*(volatile uint8_t *)(0x40000C73 + ((x) * 0x100)))
#define MCF_INTC_ICR52(x)                    (*(volatile uint8_t *)(0x40000C74 + ((x) * 0x100)))
#define MCF_INTC_ICR53(x)                    (*(volatile uint8_t *)(0x40000C75 + ((x) * 0x100)))
#define MCF_INTC_ICR54(x)                    (*(volatile uint8_t *)(0x40000C76 + ((x) * 0x100)))
#define MCF_INTC_ICR55(x)                    (*(volatile uint8_t *)(0x40000C77 + ((x) * 0x100)))
#define MCF_INTC_ICR56(x)                    (*(volatile uint8_t *)(0x40000C78 + ((x) * 0x100)))
#define MCF_INTC_ICR57(x)                    (*(volatile uint8_t *)(0x40000C79 + ((x) * 0x100)))
#define MCF_INTC_ICR58(x)                    (*(volatile uint8_t *)(0x40000C7A + ((x) * 0x100)))
#define MCF_INTC_ICR59(x)                    (*(volatile uint8_t *)(0x40000C7B + ((x) * 0x100)))
#define MCF_INTC_ICR60(x)                    (*(volatile uint8_t *)(0x40000C7C + ((x) * 0x100)))
#define MCF_INTC_ICR61(x)                    (*(volatile uint8_t *)(0x40000C7D + ((x) * 0x100)))
#define MCF_INTC_ICR62(x)                    (*(volatile uint8_t *)(0x40000C7E + ((x) * 0x100)))
#define MCF_INTC_ICR63(x)                    (*(volatile uint8_t *)(0x40000C7F + ((x) * 0x100)))
#define MCF_INTC_SWIACK(x)                   (*(volatile uint8_t *)(0x40000CE0 + ((x) * 0x100)))
#define MCF_INTC_L1IACK(x)                   (*(volatile uint8_t *)(0x40000CE4 + ((x) * 0x100)))
#define MCF_INTC_L2IACK(x)                   (*(volatile uint8_t *)(0x40000CE8 + ((x) * 0x100)))
#define MCF_INTC_L3IACK(x)                   (*(volatile uint8_t *)(0x40000CEC + ((x) * 0x100)))
#define MCF_INTC_L4IACK(x)                   (*(volatile uint8_t *)(0x40000CF0 + ((x) * 0x100)))
#define MCF_INTC_L5IACK(x)                   (*(volatile uint8_t *)(0x40000CF4 + ((x) * 0x100)))
#define MCF_INTC_L6IACK(x)                   (*(volatile uint8_t *)(0x40000CF8 + ((x) * 0x100)))
#define MCF_INTC_L7IACK(x)                   (*(volatile uint8_t *)(0x40000CFC + ((x) * 0x100)))

/* MCF_INTC_IPRH */
#define MCF_INTC_IPRH_INT32                  0x1
#define MCF_INTC_IPRH_INT33                  0x2
#define MCF_INTC_IPRH_INT34                  0x4
#define MCF_INTC_IPRH_INT35                  0x8
#define MCF_INTC_IPRH_INT36                  0x10
#define MCF_INTC_IPRH_INT37                  0x20
#define MCF_INTC_IPRH_INT38                  0x40
#define MCF_INTC_IPRH_INT39                  0x80
#define MCF_INTC_IPRH_INT40                  0x100
#define MCF_INTC_IPRH_INT41                  0x200
#define MCF_INTC_IPRH_INT42                  0x400
#define MCF_INTC_IPRH_INT43                  0x800
#define MCF_INTC_IPRH_INT44                  0x1000
#define MCF_INTC_IPRH_INT45                  0x2000
#define MCF_INTC_IPRH_INT46                  0x4000
#define MCF_INTC_IPRH_INT47                  0x8000
#define MCF_INTC_IPRH_INT48                  0x10000
#define MCF_INTC_IPRH_INT49                  0x20000
#define MCF_INTC_IPRH_INT50                  0x40000
#define MCF_INTC_IPRH_INT51                  0x80000
#define MCF_INTC_IPRH_INT52                  0x100000
#define MCF_INTC_IPRH_INT53                  0x200000
#define MCF_INTC_IPRH_INT54                  0x400000
#define MCF_INTC_IPRH_INT55                  0x800000
#define MCF_INTC_IPRH_INT56                  0x1000000
#define MCF_INTC_IPRH_INT57                  0x2000000
#define MCF_INTC_IPRH_INT58                  0x4000000
#define MCF_INTC_IPRH_INT59                  0x8000000
#define MCF_INTC_IPRH_INT60                  0x10000000
#define MCF_INTC_IPRH_INT61                  0x20000000
#define MCF_INTC_IPRH_INT62                  0x40000000
#define MCF_INTC_IPRH_INT63                  0x80000000

/* MCF_INTC_IPRL */
#define MCF_INTC_IPRL_INT1                   0x2
#define MCF_INTC_IPRL_INT2                   0x4
#define MCF_INTC_IPRL_INT3                   0x8
#define MCF_INTC_IPRL_INT4                   0x10
#define MCF_INTC_IPRL_INT5                   0x20
#define MCF_INTC_IPRL_INT6                   0x40
#define MCF_INTC_IPRL_INT7                   0x80
#define MCF_INTC_IPRL_INT8                   0x100
#define MCF_INTC_IPRL_INT9                   0x200
#define MCF_INTC_IPRL_INT10                  0x400
#define MCF_INTC_IPRL_INT11                  0x800
#define MCF_INTC_IPRL_INT12                  0x1000
#define MCF_INTC_IPRL_INT13                  0x2000
#define MCF_INTC_IPRL_INT14                  0x4000
#define MCF_INTC_IPRL_INT15                  0x8000
#define MCF_INTC_IPRL_INT16                  0x10000
#define MCF_INTC_IPRL_INT17                  0x20000
#define MCF_INTC_IPRL_INT18                  0x40000
#define MCF_INTC_IPRL_INT19                  0x80000
#define MCF_INTC_IPRL_INT20                  0x100000
#define MCF_INTC_IPRL_INT21                  0x200000
#define MCF_INTC_IPRL_INT22                  0x400000
#define MCF_INTC_IPRL_INT23                  0x800000
#define MCF_INTC_IPRL_INT24                  0x1000000
#define MCF_INTC_IPRL_INT25                  0x2000000
#define MCF_INTC_IPRL_INT26                  0x4000000
#define MCF_INTC_IPRL_INT27                  0x8000000
#define MCF_INTC_IPRL_INT28                  0x10000000
#define MCF_INTC_IPRL_INT29                  0x20000000
#define MCF_INTC_IPRL_INT30                  0x40000000
#define MCF_INTC_IPRL_INT31                  0x80000000

/* MCF_INTC_IMRH */
#define MCF_INTC_IMRH_INT_MASK32             0x1
#define MCF_INTC_IMRH_INT_MASK33             0x2
#define MCF_INTC_IMRH_INT_MASK34             0x4
#define MCF_INTC_IMRH_INT_MASK35             0x8
#define MCF_INTC_IMRH_INT_MASK36             0x10
#define MCF_INTC_IMRH_INT_MASK37             0x20
#define MCF_INTC_IMRH_INT_MASK38             0x40
#define MCF_INTC_IMRH_INT_MASK39             0x80
#define MCF_INTC_IMRH_INT_MASK40             0x100
#define MCF_INTC_IMRH_INT_MASK41             0x200
#define MCF_INTC_IMRH_INT_MASK42             0x400
#define MCF_INTC_IMRH_INT_MASK43             0x800
#define MCF_INTC_IMRH_INT_MASK44             0x1000
#define MCF_INTC_IMRH_INT_MASK45             0x2000
#define MCF_INTC_IMRH_INT_MASK46             0x4000
#define MCF_INTC_IMRH_INT_MASK47             0x8000
#define MCF_INTC_IMRH_INT_MASK48             0x10000
#define MCF_INTC_IMRH_INT_MASK49             0x20000
#define MCF_INTC_IMRH_INT_MASK50             0x40000
#define MCF_INTC_IMRH_INT_MASK51             0x80000
#define MCF_INTC_IMRH_INT_MASK52             0x100000
#define MCF_INTC_IMRH_INT_MASK53             0x200000
#define MCF_INTC_IMRH_INT_MASK54             0x400000
#define MCF_INTC_IMRH_INT_MASK55             0x800000
#define MCF_INTC_IMRH_INT_MASK56             0x1000000
#define MCF_INTC_IMRH_INT_MASK57             0x2000000
#define MCF_INTC_IMRH_INT_MASK58             0x4000000
#define MCF_INTC_IMRH_INT_MASK59             0x8000000
#define MCF_INTC_IMRH_INT_MASK60             0x10000000
#define MCF_INTC_IMRH_INT_MASK61             0x20000000
#define MCF_INTC_IMRH_INT_MASK62             0x40000000
#define MCF_INTC_IMRH_INT_MASK63             0x80000000

/* MCF_INTC_IMRL */
#define MCF_INTC_IMRL_MASKALL                0x1
#define MCF_INTC_IMRL_INT_MASK1              0x2
#define MCF_INTC_IMRL_INT_MASK2              0x4
#define MCF_INTC_IMRL_INT_MASK3              0x8
#define MCF_INTC_IMRL_INT_MASK4              0x10
#define MCF_INTC_IMRL_INT_MASK5              0x20
#define MCF_INTC_IMRL_INT_MASK6              0x40
#define MCF_INTC_IMRL_INT_MASK7              0x80
#define MCF_INTC_IMRL_INT_MASK8              0x100
#define MCF_INTC_IMRL_INT_MASK9              0x200
#define MCF_INTC_IMRL_INT_MASK10             0x400
#define MCF_INTC_IMRL_INT_MASK11             0x800
#define MCF_INTC_IMRL_INT_MASK12             0x1000
#define MCF_INTC_IMRL_INT_MASK13             0x2000
#define MCF_INTC_IMRL_INT_MASK14             0x4000
#define MCF_INTC_IMRL_INT_MASK15             0x8000
#define MCF_INTC_IMRL_INT_MASK16             0x10000
#define MCF_INTC_IMRL_INT_MASK17             0x20000
#define MCF_INTC_IMRL_INT_MASK18             0x40000
#define MCF_INTC_IMRL_INT_MASK19             0x80000
#define MCF_INTC_IMRL_INT_MASK20             0x100000
#define MCF_INTC_IMRL_INT_MASK21             0x200000
#define MCF_INTC_IMRL_INT_MASK22             0x400000
#define MCF_INTC_IMRL_INT_MASK23             0x800000
#define MCF_INTC_IMRL_INT_MASK24             0x1000000
#define MCF_INTC_IMRL_INT_MASK25             0x2000000
#define MCF_INTC_IMRL_INT_MASK26             0x4000000
#define MCF_INTC_IMRL_INT_MASK27             0x8000000
#define MCF_INTC_IMRL_INT_MASK28             0x10000000
#define MCF_INTC_IMRL_INT_MASK29             0x20000000
#define MCF_INTC_IMRL_INT_MASK30             0x40000000
#define MCF_INTC_IMRL_INT_MASK31             0x80000000

/* MCF_INTC_INTFRCH */
#define MCF_INTC_INTFRCH_INTFRC32            0x1
#define MCF_INTC_INTFRCH_INTFRC33            0x2
#define MCF_INTC_INTFRCH_INTFRC34            0x4
#define MCF_INTC_INTFRCH_INTFRC35            0x8
#define MCF_INTC_INTFRCH_INTFRC36            0x10
#define MCF_INTC_INTFRCH_INTFRC37            0x20
#define MCF_INTC_INTFRCH_INTFRC38            0x40
#define MCF_INTC_INTFRCH_INTFRC39            0x80
#define MCF_INTC_INTFRCH_INTFRC40            0x100
#define MCF_INTC_INTFRCH_INTFRC41            0x200
#define MCF_INTC_INTFRCH_INTFRC42            0x400
#define MCF_INTC_INTFRCH_INTFRC43            0x800
#define MCF_INTC_INTFRCH_INTFRC44            0x1000
#define MCF_INTC_INTFRCH_INTFRC45            0x2000
#define MCF_INTC_INTFRCH_INTFRC46            0x4000
#define MCF_INTC_INTFRCH_INTFRC47            0x8000
#define MCF_INTC_INTFRCH_INTFRC48            0x10000
#define MCF_INTC_INTFRCH_INTFRC49            0x20000
#define MCF_INTC_INTFRCH_INTFRC50            0x40000
#define MCF_INTC_INTFRCH_INTFRC51            0x80000
#define MCF_INTC_INTFRCH_INTFRC52            0x100000
#define MCF_INTC_INTFRCH_INTFRC53            0x200000
#define MCF_INTC_INTFRCH_INTFRC54            0x400000
#define MCF_INTC_INTFRCH_INTFRC55            0x800000
#define MCF_INTC_INTFRCH_INTFRC56            0x1000000
#define MCF_INTC_INTFRCH_INTFRC57            0x2000000
#define MCF_INTC_INTFRCH_INTFRC58            0x4000000
#define MCF_INTC_INTFRCH_INTFRC59            0x8000000
#define MCF_INTC_INTFRCH_INTFRC60            0x10000000
#define MCF_INTC_INTFRCH_INTFRC61            0x20000000
#define MCF_INTC_INTFRCH_INTFRC62            0x40000000
#define MCF_INTC_INTFRCH_INTFRC63            0x80000000

/* MCF_INTC_INTFRCL */
#define MCF_INTC_INTFRCL_INTFRC1             0x2
#define MCF_INTC_INTFRCL_INTFRC2             0x4
#define MCF_INTC_INTFRCL_INTFRC3             0x8
#define MCF_INTC_INTFRCL_INTFRC4             0x10
#define MCF_INTC_INTFRCL_INTFRC5             0x20
#define MCF_INTC_INTFRCL_INTFRC6             0x40
#define MCF_INTC_INTFRCL_INTFRC7             0x80
#define MCF_INTC_INTFRCL_INTFRC8             0x100
#define MCF_INTC_INTFRCL_INTFRC9             0x200
#define MCF_INTC_INTFRCL_INTFRC10            0x400
#define MCF_INTC_INTFRCL_INTFRC11            0x800
#define MCF_INTC_INTFRCL_INTFRC12            0x1000
#define MCF_INTC_INTFRCL_INTFRC13            0x2000
#define MCF_INTC_INTFRCL_INTFRC14            0x4000
#define MCF_INTC_INTFRCL_INTFRC15            0x8000
#define MCF_INTC_INTFRCL_INTFRC16            0x10000
#define MCF_INTC_INTFRCL_INTFRC17            0x20000
#define MCF_INTC_INTFRCL_INTFRC18            0x40000
#define MCF_INTC_INTFRCL_INTFRC19            0x80000
#define MCF_INTC_INTFRCL_INTFRC20            0x100000
#define MCF_INTC_INTFRCL_INTFRC21            0x200000
#define MCF_INTC_INTFRCL_INTFRC22            0x400000
#define MCF_INTC_INTFRCL_INTFRC23            0x800000
#define MCF_INTC_INTFRCL_INTFRC24            0x1000000
#define MCF_INTC_INTFRCL_INTFRC25            0x2000000
#define MCF_INTC_INTFRCL_INTFRC26            0x4000000
#define MCF_INTC_INTFRCL_INTFRC27            0x8000000
#define MCF_INTC_INTFRCL_INTFRC28            0x10000000
#define MCF_INTC_INTFRCL_INTFRC29            0x20000000
#define MCF_INTC_INTFRCL_INTFRC30            0x40000000
#define MCF_INTC_INTFRCL_INTFRC31            0x80000000

/* MCF_INTC_IRLR */
#define MCF_INTC_IRLR_IRQ(x)                 (((x) & 0x7F) << 0x1)

/* MCF_INTC_IACKLPR */
#define MCF_INTC_IACKLPR_PRI(x)              (((x) & 0xF) << 0)
#define MCF_INTC_IACKLPR_LEVEL(x)            (((x) & 0x7) << 0x4)

/* MCF_INTC_ICR */
#define MCF_INTC_ICR_IP(x)                   (((x) & 0x7) << 0)
#define MCF_INTC_ICR_IL(x)                   (((x) & 0x7) << 0x3)

/* MCF_INTC_SWIACK */
#define MCF_INTC_SWIACK_VECTOR(x)            (((x) & 0xFF) << 0)

/* MCF_INTC_LIACK */
#define MCF_INTC_LIACK_VECTOR(x)             (((x) & 0xFF) << 0)

#endif /* MCF5225X_INTC_H_ */
