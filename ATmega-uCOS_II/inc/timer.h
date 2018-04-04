#ifndef __TIMER_H__
#define __TIMER_H__

#define  TIMER0_STOP();				{TCCR0 = 0x00;}
#define  TIMER0_START();			{TCCR0 = 0x07;} 	// 1024分频

#define  TIMER1_STOP();				{TCCR1B = 0x00;}
#define  TIMER1_START();			{TCCR1B = 0x07;}	// 外部T1引脚为时钟源，上升沿驱动

#define  TIMER2_STOP();				{TCCR2 = 0x00;}
#define  TIMER2_START();			{TCCR2 = 0x07;}		// 外部T2引脚为时钟源，上升沿驱动

#define  TIMER3_STOP();				{TCCR3B = 0x00;}
#define  TIMER3_START();			{TCCR3B = 0x02;}	// 8分频，采集时未用T3，T3用作了TICK（1024分频）

#define TIMER0_IS_OFF()				(TCCR0 == 0x00)
#define TIMER1_IS_OFF()				(TCCR1B == 0x00)
#define TIMER2_IS_OFF()				(TCCR2 == 0x00)
#define TIMER3_IS_OFF()				(TCCR3B == 0x00)

#define TIMER0_IS_ON()				(TCCR0 != 0x00)
#define TIMER1_IS_ON()				(TCCR1B != 0x00)
#define TIMER2_IS_ON()				(TCCR2 != 0x00)
#define TIMER3_IS_ON()				(TCCR3B != 0x00)

// Timer0 用于内部时钟,每次溢出间隔时间为20ms（1024分频）
#define TIMER0_INIT_VALUE			0x64
#define TIMER0_CNT_RESET();			{TCNT0 = TIMER0_INIT_VALUE;}
// 计数初值清零
#define TIMER1_CNT_RESET();			{TCNT1 = 0x00;}
// 计数初值清零
#define TIMER2_CNT_RESET();			{TCNT2 = 0x00;}
// 计数初值清零
#define TIMER3_CNT_RESET();			{TCNT3 = 0x00;}

// 使能Timer0 Timer1 Timer2 的溢出中断
#define TIMER012_TOIE_ENABLE();		{TIMSK = _BV(TOIE0) | _BV(TOIE1) | _BV(TOIE2);}
// 禁止Timer0 Timer1 Timer2 的溢出中断
#define TIMER012_TOIE_DISABLE();	{TIMSK = 0x00;}

void timers_init(void);
void timer0_init(void);
void timer1_init(void);
void timer2_init(void);

void timers_stop(void);
void timers_start(void);
void timers_ovf_times_reset(void);
void timers_cnt_reset(void);

ISR(TIMER0_OVF_vect);
ISR(TIMER1_OVF_vect);
ISR(TIMER2_OVF_vect);

#endif
