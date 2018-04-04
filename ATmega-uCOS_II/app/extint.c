#include "config.h"

void ExtInt_init(void)
{
	EICRA = 0x00; 						//extended ext ints
	EICRB = _BV(ISC61) | _BV(ISC71); 	//设定外部中断6、7为下降沿触发
	//EXTINT_ENABLE();					//使能外部中断6、7，用来测流速
	EXTINT_DISABLE();
}

ISR(INT6_vect)
{

}

ISR(INT7_vect)
{
	measure_time_count_read();
	REVERSE(BITNUM_3);
}
