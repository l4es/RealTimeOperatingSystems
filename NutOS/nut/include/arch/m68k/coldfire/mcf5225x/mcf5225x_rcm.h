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

#ifndef MCF5225X_RCM_H_
#define MCF5225X_RCM_H_

/* Register read/write macros */
#define MCF_RCM_RCR                          (*(volatile uint8_t *)(0x40110000))
#define MCF_RCM_RSR                          (*(volatile uint8_t *)(0x40110001))

/* MCF_RCM_RCR */
#define MCF_RCM_RCR_LVDE                     0x1
#define MCF_RCM_RCR_LVDRE                    0x4
#define MCF_RCM_RCR_LVDIE                    0x8
#define MCF_RCM_RCR_LVDF                     0x10
#define MCF_RCM_RCR_FRCRSTOUT                0x40
#define MCF_RCM_RCR_SOFTRST                  0x80

/* MCF_RCM_RSR */
#define MCF_RCM_RSR_LOL                      0x1
#define MCF_RCM_RSR_LOC                      0x2
#define MCF_RCM_RSR_EXT                      0x4
#define MCF_RCM_RSR_POR                      0x8
#define MCF_RCM_RSR_WDR                      0x10
#define MCF_RCM_RSR_SOFT                     0x20
#define MCF_RCM_RSR_LVD                      0x40
#define MCF_RCM_RSR_BWD                      0x80

#endif /* MCF5225X_RCM_H_ */
