#include <avr/interrupt.h>

unsigned int count=0;
unsigned char flag=1;

ISR(TIMER2_OVF_vect)
{
  if(count==500)
  {
    count=0;
    
    if(flag)
      digitalWrite(6, HIGH);
    else
      digitalWrite(6, LOW);
    
    flag=!flag;
  }
  else
    count++;
}

void setup()
{
  pinMode(6, OUTPUT);
  TCCR2A=0b00000001;
  TCNT2=0;
  TIMSK2|=0b1;
  TCCR2B=0b00000011;
}

void loop()
{
  // Empty
}

