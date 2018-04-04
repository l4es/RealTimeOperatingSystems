#include "config.h"

//==========================================================================================
//以下为ucos所用的定时器中断，分别为T0和T3，任何一个都可以实现。
//如果要进行更换，则要改去os_cpu_a.S文件中的相应部分。
//T0：TIMER0_COMP_vect 而T3:TIMER3_COMPA_vect
//OSC = 8000000Hz
/*利用atmega128的8位定时器Timer0的输出比较匹配单元产生定时中断,程序中设定为200Hz(5ms)*/

void timer0_init_ucos(void)
{
   TCNT0  = 0x00;   		/*清零Timer0计数器计数方式为TCNT0~OCR0		*/
   OCR0   = (1000/OS_TICKS_PER_SEC)*F_CPU_M;   		 /*产生一次匹配中断 TOP为OCR0 				*/
   TCCR0  = _BV(WGM01)|_BV(CS02)|_BV(CS01)|_BV(CS00);   /*工作于输出比较匹配CTC模式,不连接OC0端口,时钟1024预分频*/
   TIFR  |= 0x02;   		/*清除输出比较匹配中断标志位				*/
   TIMSK |= 0x02;   		/*使能输出比较匹配中断						*/
   sei();              		/*使能全局中断								*/
   TCCR0 |= _BV(FOC0);   	/*启动输出比较匹配							*/
}

//TIMER3 initialize - prescale:1024
// WGM: 4) CTC, TOP=OCRnA
// desired value: 200Hz
// actual value: 200.321Hz (0.2%)
//OSC = 8000000Hz
/*利用atmega128的8位定时器Timer3的输出比较匹配A单元产生定时中断,程序中设定为200Hz(5ms)*/
void timer3_init_ucos(void)
{
	TCCR3B = 0x00; 				//stop
	TCNT3H = 0x00; 				//setup
	TCNT3L = 0x00;
	OCR3AH = 0x00;
	OCR3AL = 0x28;
	TCCR3A = 0x00; 				//OC3C输出未连接
	TCCR3B = _BV(WGM32)|_BV(CS32)|_BV(CS30); //start Timer CTC 1024分频
	ETIFR |= _BV(OCF3A);   		/*清除输出比较匹配中断标志位，写1清0		*/
    ETIMSK|= _BV(OCIE3A);   	/*使能输出比较匹配中断						*/
    sei();              		/*使能全局中断								*/
}

void port_init(void)
{
	PORTC = 0xFF;				//PC4-5为KEY1,KEY2,PC0-3为LED1-4，全部上拉电阻
	DDRC  = 0x0F;				//PC4-5为输入，PC0-3为输出
	PORTE = 0xC0;				//PE7-6分别为外部中断7，6，用来测量流速，低频
	DDRE  = 0x00;				//外部中断采用下降沿触发，只使用了外部中断7
}

void board_init(void)
{
	port_init();
	timer3_init_ucos();
	uart_init();
	i2c_init();
	pcf8563_init();
	adc_init();
	key_value_queue_init();
}