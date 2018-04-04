#include "config.h"

void timers_init(void)
{
	timer0_init();						//定时器0初始化
	timer1_init();						//定时器1初始化
	timer2_init();						//定时器2初始化

	TIMER012_TOIE_ENABLE();				//定时器012溢出中断使能
}

// TIMER0 initialize - prescale:1024 负责读取数据的时间间隔控制
// WGM: Normal
// desired value: 20mSec
// actual value: 19.968mSec (0.2%)
void timer0_init(void)
{
	TCCR0 = 0x00; 						//stop
	ASSR  = 0x00; 						//set async mode
	TCNT0 = TIMER0_INIT_VALUE; 			//set count 0x64
	OCR0  = 0x00;

	//TCCR0 = 0x07; 					//start timer
}

// TIMER1 initialize - prescale:Rising edge 对外部时钟的上升沿进行计数 测量转矩
// WGM: 0) Normal, TOP=0xFFFF
// desired value: Hz
// actual value: Out of range
void timer1_init(void)
{
	TCCR1B = 0x00;						//stop
	TCNT1  = 0x00 						/*INVALID SETTING*/; //setup
	OCR1AH = 0x00 						/*INVALID SETTING*/;
	OCR1AL = 0x00 						/*INVALID SETTING*/;
	OCR1BH = 0x00 						/*INVALID SETTING*/;
	OCR1BL = 0x00 						/*INVALID SETTING*/;
	OCR1CH = 0x00 						/*INVALID SETTING*/;
	OCR1CL = 0x00 						/*INVALID SETTING*/;
	ICR1H  = 0x00 						/*INVALID SETTING*/;
	ICR1L  = 0x00 						/*INVALID SETTING*/;
	TCCR1A = 0x00;
	TCCR1C = 0x00;

	/*TCCR1B = 0x07; //start Timer*/
}

void timers_cnt_reset(void)
{
	TIMER0_CNT_RESET();							// Timer0用于内部时钟,每次溢出间隔时间为20ms
	TIMER1_CNT_RESET();							// 计数初值清零
	TIMER2_CNT_RESET();							// 计数初值清零
}

void timers_stop(void)
{
	TIMER0_STOP();								// 停止计数器0  负责对整个数据读取间隔控制
	TIMER1_STOP();								// 停止计数器1	负责转矩测量
	TIMER2_STOP();								// 停止计数器2  负责流速测量
}	

void timers_start(void)
{
	TIMER2_START();								// 开启计数器
	TIMER1_START();								// 开启计数器
	TIMER0_START();								// 开启计数器
}

void timers_ovf_times_reset(void)
{
	timer0_ovf_times = 0;						// Timer0溢出次数清零
	timer1_ovf_times = 0;						// Timer1溢出次数清零
	timer2_ovf_times = 0;						// Timer2溢出次数清零
}
// TIMER2 initialize - prescale:Rising edge  对外部时钟的上升沿进行计数 测量流速
// WGM: Normal
// desired value: 1Hz
// actual value: Out of range
void timer2_init(void)
{
	TCCR2 = 0x00; 						//stop
	TCNT2 = 0x00; 						//setup
	OCR2  = 0x00;

	/*TCCR2 = 0x07; 					//外部时钟源*/
}

ISR(TIMER0_OVF_vect)
{
	TIMER0_CNT_RESET();					// TIMER0初始值重新加载
	timer0_ovf_times ++;				// Timer0溢出次数加一
	adc_convert_start();				// 启动AD转换, 测量一组电流电压

	if(timer0_ovf_times >= timer0_max_ovf_timers)
	{
										// 定时时间到, 停止计时, 进行相应处理
		measure_stop();
		time_is_out_flag = TRUE;
	}
}

ISR(TIMER1_OVF_vect)
{
	timer1_ovf_times++;					// Timer1溢出次数加一
}

ISR(TIMER2_OVF_vect)
{
	timer2_ovf_times++;					// Timer2溢出次数加一
}

