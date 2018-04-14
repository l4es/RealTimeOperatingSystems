/*
 * testcode.c
 *
 * Created: 10/4/2013 8:23:52 AM
 *  Author: dcstanc
 */ 

#include "ardio.h"
#include <avr/io.h>
#include <util/delay.h>

void delay(int ms)
{
	int i;
	
	for(i=0; i<ms; i++)
		_delay_ms(1);	
}

int pwmval=0;
int pwmval2=255;
int ctr=0;

/*
ISR(TIMER0_COMPA_vect)
{
	if(!(ctr%5))
	{
		pwmval=(pwmval+1)%255;
		OCR0A=pwmval;
	}		
		ctr++;
}

void makePWM()
{
	TCNT0=0;
	OCR0A=0;
	TCCR0A=0b10000001;
	TIMSK0|=0b10;
	DDRD|=(1<<DDD6);
}

void startPWM()
{
	TCCR0B=0b00000011;
	sei();
}*/
	
int main()
{
	// Do a simple blinky
/*	makeOutputPins(DIRB, PIN5M);
	makeOutputPins(DIRD, PIN5M);
	makeInputPins(DIRD, PIN4M);
*/	
	pinMode(13, OUTPUT);
	pinMode(12, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(4, INPUT);
	
	int myDelay=500;
	int ctr=0;
	
//	pwmval=128;
//	makePWM();
//	startPWM();

//	setupPWM16(PIN9TCNT, PIN9TCCRA, PIN9MSK, PIN9DDR, PIN9PIN, PIN9CR);	
//	setDutyCycle16(PIN9OCR, 255, PIN9SV);
//	startPWM16(PIN9TCCRB);
	
	while(1)
	{
		 ctr++;

		if(!(ctr%2))
		{
			pwmval=(pwmval+103)%1024;
			pwmval2=pwmval2-25;
			
			if(pwmval2<0)
				pwmval2=255;
			
	//		setDutyCycle16(PIN9OCR, pwmval, PIN9SV);					
			analogWrite(9, pwmval);
			analogWrite(6, pwmval2);
		}
		  			
		  myDelay=analogRead(0);
		  digitalWrite(13, HIGH);
		  		  
		  if(digitalRead(4))
			digitalWrite(5, HIGH);
		  else
			digitalWrite(5, LOW);
		  
		  delay(myDelay);
		  
		  digitalWrite(13, LOW);

		  if(digitalRead(4))
			digitalWrite(5, HIGH);
		  else
			digitalWrite(5, LOW);
			
		delay(myDelay); 
/*		setOutput(OUTB, BIT5, HIGH);
		
		if(getInput(IND, BIT4))
			setOutput(OUTD, BIT5, HIGH);
		else
			setOutput(OUTD, BIT5, LOW);
			
		_delay_ms(250);
		setOutput(OUTB, BIT5, LOW);
		
		if(getInput(IND, BIT4))
			setOutput(OUTD, BIT5, HIGH);
		else
			setOutput(OUTD, BIT5, LOW);
		
		//setOutput(OUTB, BIT5, LOW);
		_delay_ms(250);		*/

	}
}
