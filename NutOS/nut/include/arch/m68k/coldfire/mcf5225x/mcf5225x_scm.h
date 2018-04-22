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

#ifndef MCF5225X_SCM_H_
#define MCF5225X_SCM_H_

/* SCM Registers */
#define MCF_SCM_IPSBAR                       (*(volatile uint32_t*)(0x40000000))
#define MCF_SCM_RAMBAR                       (*(volatile uint32_t*)(0x40000008))
#define MCF_SCM_PPMRH                        (*(volatile uint32_t*)(0x4000000C))
#define MCF_SCM_CRSR                         (*(volatile uint8_t *)(0x40000010))
#define MCF_SCM_CWCR                         (*(volatile uint8_t *)(0x40000011))
#define MCF_SCM_CWSR                         (*(volatile uint8_t *)(0x40000013))
#define MCF_SCM_DMAREQC                      (*(volatile uint32_t*)(0x40000014))
#define MCF_SCM_PPMRL                        (*(volatile uint32_t*)(0x40000018))
#define MCF_SCM_MPARK                        (*(volatile uint32_t*)(0x4000001C))
#define MCF_SCM_MPR                          (*(volatile uint8_t *)(0x40000020))
#define MCF_SCM_PPMRS                        (*(volatile uint8_t *)(0x40000021))
#define MCF_SCM_PPMRC                        (*(volatile uint8_t *)(0x40000022))
#define MCF_SCM_IPSBMT                       (*(volatile uint8_t *)(0x40000023))
#define MCF_SCM_PACR0                        (*(volatile uint8_t *)(0x40000024))
#define MCF_SCM_PACR1                        (*(volatile uint8_t *)(0x40000025))
#define MCF_SCM_PACR2                        (*(volatile uint8_t *)(0x40000026))
#define MCF_SCM_PACR3                        (*(volatile uint8_t *)(0x40000027))
#define MCF_SCM_PACR4                        (*(volatile uint8_t *)(0x40000028))
#define MCF_SCM_PACR5                        (*(volatile uint8_t *)(0x40000029))
#define MCF_SCM_PACR6                        (*(volatile uint8_t *)(0x4000002A))
#define MCF_SCM_PACR7                        (*(volatile uint8_t *)(0x4000002B))
#define MCF_SCM_PACR8                        (*(volatile uint8_t *)(0x4000002C))
#define MCF_SCM_PACR10                       (*(volatile uint8_t *)(0x4000002E))
#define MCF_SCM_GPACR0                       (*(volatile uint8_t *)(0x40000030))
#define MCF_SCM_GPACR1                       (*(volatile uint8_t *)(0x40000031))
#define MCF_SCM_PACR(x)                      (*(volatile uint8_t *)(0x40000024 + ((x) * 0x1)))
#define MCF_SCM_GPACR(x)                     (*(volatile uint8_t *)(0x40000030 + ((x) * 0x1)))

/* MCF_SCM_IPSBAR */
#define MCF_SCM_IPSBAR_V                     0x1
#define MCF_SCM_IPSBAR_BA(x)                 ((x) & 0xC0000000)

/* MCF_SCM_RAMBAR */
#define MCF_SCM_RAMBAR_BDE                   0x200
#define MCF_SCM_RAMBAR_BA(x)                 ((x) & 0xFFFF0000)

/* MCF_SCM_PPMRH */
#define MCF_SCM_PPMRH_CDGPIO                 0x1
#define MCF_SCM_PPMRH_CDEPORT                0x2
#define MCF_SCM_PPMRH_CDPIT0                 0x8
#define MCF_SCM_PPMRH_CDPIT1                 0x10
#define MCF_SCM_PPMRH_CDADC                  0x80
#define MCF_SCM_PPMRH_CDGPT                  0x100
#define MCF_SCM_PPMRH_CDPWM                  0x200
#define MCF_SCM_PPMRH_CDCAN                  0x400
#define MCF_SCM_PPMRH_CDCFM                  0x800

/* MCF_SCM_CRSR */
#define MCF_SCM_CRSR_EXT                     0x80

/* MCF_SCM_CWCR */
#define MCF_SCM_CWCR_CWTIF                   0x1
#define MCF_SCM_CWCR_CWTAVAL                 0x2
#define MCF_SCM_CWCR_CWTA                    0x4
#define MCF_SCM_CWCR_CWT(x)                  (((x) & 0x7) << 0x3)
#define MCF_SCM_CWCR_CWT_2_9                 0
#define MCF_SCM_CWCR_CWT_2_11                0x8
#define MCF_SCM_CWCR_CWT_2_13                0x10
#define MCF_SCM_CWCR_CWT_2_15                0x18
#define MCF_SCM_CWCR_CWT_2_19                0x20
#define MCF_SCM_CWCR_CWT_2_23                0x28
#define MCF_SCM_CWCR_CWT_2_27                0x30
#define MCF_SCM_CWCR_CWT_2_31                0x38
#define MCF_SCM_CWCR_CWRI                    0x40
#define MCF_SCM_CWCR_CWE                     0x80

/* MCF_SCM_CWSR */
#define MCF_SCM_CWSR_CWSR(x)                 (((x) & 0xFF) << 0)

/* MCF_SCM_DMAREQC */
#define MCF_SCM_DMAREQC_DMAC0(x)             (((x) & 0xF) << 0)
#define MCF_SCM_DMAREQC_DMAC1(x)             (((x) & 0xF) << 0x4)
#define MCF_SCM_DMAREQC_DMAC2(x)             (((x) & 0xF) << 0x8)
#define MCF_SCM_DMAREQC_DMAC3(x)             (((x) & 0xF) << 0xC)

/* MCF_SCM_PPMRL */
#define MCF_SCM_PPMRL_CDG                    0x2
#define MCF_SCM_PPMRL_CDMINIBUS              0x8
#define MCF_SCM_PPMRL_CDDMA                  0x10
#define MCF_SCM_PPMRL_CDUART0                0x20
#define MCF_SCM_PPMRL_CDUART1                0x40
#define MCF_SCM_PPMRL_CDUART2                0x80
#define MCF_SCM_PPMRL_CDI2C0                 0x200
#define MCF_SCM_PPMRL_CDQSPI                 0x400
#define MCF_SCM_PPMRL_CDI2C1                 0x800
#define MCF_SCM_PPMRL_CDRTC                  0x1000
#define MCF_SCM_PPMRL_CDDTIM0                0x2000
#define MCF_SCM_PPMRL_CDDTIM1                0x4000
#define MCF_SCM_PPMRL_CDDTIM2                0x8000
#define MCF_SCM_PPMRL_CDDTIM3                0x10000
#define MCF_SCM_PPMRL_CDINTC0                0x20000
#define MCF_SCM_PPMRL_CDINTC1                0x40000
#define MCF_SCM_PPMRL_CDFEC                  0x200000

/* MCF_SCM_MPARK */
#define MCF_SCM_MPARK_LCKOUT_TIME(x)         (((x) & 0xF) << 0x8)
#define MCF_SCM_MPARK_PRKLAST                0x1000
#define MCF_SCM_MPARK_TIMEOUT                0x2000
#define MCF_SCM_MPARK_FIXED                  0x4000
#define MCF_SCM_MPARK_M1_PRTY(x)             (((x) & 0x3) << 0x10)
#define MCF_SCM_MPARK_M0_PRTY(x)             (((x) & 0x3) << 0x12)
#define MCF_SCM_MPARK_M2_PRTY(x)             (((x) & 0x3) << 0x14)
#define MCF_SCM_MPARK_M3_PRTY(x)             (((x) & 0x3) << 0x16)
#define MCF_SCM_MPARK_BCR24BIT               0x1000000
#define MCF_SCM_MPARK_M2_P_EN                0x2000000

/* MCF_SCM_MPR */
#define MCF_SCM_MPR_MPR(x)                   (((x) & 0xF) << 0)

/* MCF_SCM_PPMRS */
#define MCF_SCM_PPMRS_PPMRS(x)               (((x) & 0x7F) << 0)
#define MCF_SCM_PPMRS_DISABLE_ALL            0x40
#define MCF_SCM_PPMRS_DISABLE_CFM            0x2B
#define MCF_SCM_PPMRC_DISABLE_CAN            0x2A
#define MCF_SCM_PPMRS_DISABLE_PWM            0x29
#define MCF_SCM_PPMRS_DISABLE_GPT            0x28
#define MCF_SCM_PPMRS_DISABLE_ADC            0x27
#define MCF_SCM_PPMRS_DISABLE_PIT1           0x24
#define MCF_SCM_PPMRS_DISABLE_PIT0           0x23
#define MCF_SCM_PPMRS_DISABLE_EPORT          0x21
#define MCF_SCM_PPMRS_DISABLE_PORTS          0x20
#define MCF_SCM_PPMRS_DISABLE_FEC            0x15
#define MCF_SCM_PPMRS_DISABLE_INTC1          0x12
#define MCF_SCM_PPMRS_DISABLE_INTC0          0x11
#define MCF_SCM_PPMRS_DISABLE_DTIM3          0x10
#define MCF_SCM_PPMRS_DISABLE_DTIM2          0xF
#define MCF_SCM_PPMRS_DISABLE_DTIM1          0xE
#define MCF_SCM_PPMRS_DISABLE_DTIM0          0xD
#define MCF_SCM_PPMRS_DISABLE_I2C1           0xB
#define MCF_SCM_PPMRS_DISABLE_QSPI           0xA
#define MCF_SCM_PPMRS_DISABLE_I2C0           0x9
#define MCF_SCM_PPMRS_DISABLE_UART2          0x7
#define MCF_SCM_PPMRS_DISABLE_UART1          0x6
#define MCF_SCM_PPMRS_DISABLE_UART0          0x5
#define MCF_SCM_PPMRS_DISABLE_DMA            0x4
#define MCF_SCM_PPMRS_DISABLE_MINIBUS        0x3
#define MCF_SCM_PPMRS_SET_CDG                0x1

/* MCF_SCM_PPMRC */
#define MCF_SCM_PPMRC_PPMRC(x)               (((x) & 0x7F) << 0)
#define MCF_SCM_PPMRC_ENABLE_ALL             0x40
#define MCF_SCM_PPMRC_ENABLE_CFM             0x2B
#define MCF_SCM_PPMRC_ENABLE_CAN             0x2A
#define MCF_SCM_PPMRC_ENABLE_PWM             0x29
#define MCF_SCM_PPMRC_ENABLE_GPT             0x28
#define MCF_SCM_PPMRC_ENABLE_ADC             0x27
#define MCF_SCM_PPMRC_ENABLE_PIT1            0x24
#define MCF_SCM_PPMRC_ENABLE_PIT0            0x23
#define MCF_SCM_PPMRC_ENABLE_EPORT           0x21
#define MCF_SCM_PPMRC_ENABLE_PORTS           0x20
#define MCF_SCM_PPMRC_ENABLE_FEC             0x15
#define MCF_SCM_PPMRC_ENABLE_INTC1           0x12
#define MCF_SCM_PPMRC_ENABLE_INTC0           0x11
#define MCF_SCM_PPMRC_ENABLE_DTIM3           0x10
#define MCF_SCM_PPMRC_ENABLE_DTIM2           0xF
#define MCF_SCM_PPMRC_ENABLE_DTIM1           0xE
#define MCF_SCM_PPMRC_ENABLE_DTIM0           0xD
#define MCF_SCM_PPMRC_ENABLE_I2C1            0xB
#define MCF_SCM_PPMRC_ENABLE_QSPI            0xA
#define MCF_SCM_PPMRC_ENABLE_I2C0            0x9
#define MCF_SCM_PPMRC_ENABLE_UART2           0x7
#define MCF_SCM_PPMRC_ENABLE_UART1           0x6
#define MCF_SCM_PPMRC_ENABLE_UART0           0x5
#define MCF_SCM_PPMRC_ENABLE_DMA             0x4
#define MCF_SCM_PPMRC_ENABLE_MINIBUS         0x3
#define MCF_SCM_PPMRC_CLEAR_CDG              0x1

/* MCF_SCM_IPSBMT */
#define MCF_SCM_IPSBMT_BMT(x)                (((x) & 0x7) << 0)
#define MCF_SCM_IPSBMT_BMT_CYCLES_1024       0
#define MCF_SCM_IPSBMT_BMT_CYCLES_512        0x1
#define MCF_SCM_IPSBMT_BMT_CYCLES_256        0x2
#define MCF_SCM_IPSBMT_BMT_CYCLES_128        0x3
#define MCF_SCM_IPSBMT_BMT_CYCLES_64         0x4
#define MCF_SCM_IPSBMT_BMT_CYCLES_32         0x5
#define MCF_SCM_IPSBMT_BMT_CYCLES_16         0x6
#define MCF_SCM_IPSBMT_BMT_CYCLES_8          0x7
#define MCF_SCM_IPSBMT_BME                   0x8

/* MCF_SCM_PACR */
#define MCF_SCM_PACR_ACCESS_CTRL0(x)         (((x) & 0x7) << 0)
#define MCF_SCM_PACR_LOCK0                   0x8
#define MCF_SCM_PACR_ACCESS_CTRL1(x)         (((x) & 0x7) << 0x4)
#define MCF_SCM_PACR_LOCK1                   0x80

/* MCF_SCM_PACR10 */
#define MCF_SCM_PACR10_ACCESS_CTRL0(x)       (((x) & 0x7) << 0)
#define MCF_SCM_PACR10_LOCK0                 0x8
#define MCF_SCM_PACR10_ACCESS_CTRL1(x)       (((x) & 0x7) << 0x4)
#define MCF_SCM_PACR10_LOCK1                 0x80

/* MCF_SCM_GPACR */
#define MCF_SCM_GPACR_ACCESS_CTRL(x)         (((x) & 0xF) << 0)
#define MCF_SCM_GPACR_LOCK                   0x80

#endif /* MCF5225X_SCM_H_ */
