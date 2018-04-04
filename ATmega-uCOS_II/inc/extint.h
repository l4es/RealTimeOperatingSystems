#ifndef __EXTINT_H_
#define __EXTINT_H_

//#define EXTINT_ENABLE();	{EIMSK = _BV(INT6) | _BV(INT7);}
#define EXTINT_ENABLE();	{EIMSK = _BV(INT7);}
#define EXTINT_DISABLE();	{EIMSK = 0x00;}

extern void ExtInt_init(void);
extern ISR(INT6_vect);
extern ISR(INT7_vect);

#endif
