/*
 * core_cm3.h
 *
 *  Created on: 15.01.2016
 *      Author: radkohl
 */

#ifndef ARM_CORTEX_M3_CORTEX_M3_H_
#define ARM_CORTEX_M3_CORTEX_M3_H_

#define NVIC_PRIO_BITS          3       /*!< Number of Bits used for Priority Levels */

#define ResetIRQn             -15   /*!< Reset Vector, invoked on Power up and warm reset */
#define NonMaskableIntIRQn    -14   /*!< Non maskable Interrupt, cannot be stopped or preempted */
#define HardFaultIRQn         -13   /*!< Hard Fault, all classes of Fault               */
#define MemoryManagementIRQn  -12   /*!< Memory Management, MPU mismatch, including Access Violation and No Match */
#define BusFaultIRQn          -11   /*!< Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault */
#define UsageFaultIRQn        -10   /*!< Usage Fault, i.e. Undef Instruction, Illegal State Transition */
#define SVCallIRQn             -5   /*!< System Service Call via SVC instruction         */
#define DebugMonitorIRQn       -4   /*!< Debug Monitor                                   */
#define PendSVIRQn             -2   /*!< Pendable request for system service             */
#define SysTickIRQn            -1   /*!< System Tick Timer                               */

/* SysTick Reload Register Definitions */
#define SysTick_CTRL       (*((volatile uint32_t*)0xE000E010UL))
#define SysTick_LOAD       (*((volatile uint32_t*)0xE000E014UL))
#define SysTick_VAL        (*((volatile uint32_t*)0xE000E018UL))

#define SysTick_LOAD_RELOAD            (0xFFFFFFUL)        /*!< SysTick LOAD: RELOAD Mask */
#define SysTick_CTRL_CLKSOURCE         (1UL << 2)            /*!< SysTick CTRL: CLKSOURCE Mask */
#define SysTick_CTRL_TICKINT           (1UL << 1)              /*!< SysTick CTRL: TICKINT Mask */
#define SysTick_CTRL_ENABLE            (1UL << 0)               /*!< SysTick CTRL: ENABLE Mask */


#define SCB_SHP            (((volatile uint32_t*)0xE000E018UL))

static inline uint32_t SysTickConfig( uint32_t ticks )
{
   if( (ticks - 1) > SysTick_LOAD_RELOAD )
      return (1);

   SysTick_LOAD = ticks - 1; /* set reload register */
   SysTick_VAL = 0; /* Load the SysTick Counter Value */
   SysTick_CTRL = SysTick_CTRL_CLKSOURCE | SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE;
   return (0);
}

static inline void NVICSetPriority( uint32_t IRQn, uint32_t priority )
{
   if( IRQn < 0 )
      SCB_SHP[(IRQn & 0xF) - 4] = ((priority << (8 - NVIC_PRIO_BITS)) & 0xff);
}

#endif /* ARM_CORTEX_M3_CORTEX_M3_H_ */
