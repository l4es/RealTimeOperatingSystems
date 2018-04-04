#ifndef __BOARD_H__
#define __BOARD_H__

#define LED1				0
#define LED2				1
#define LED3				2
#define LED4				3

//---------端口控制，H——输出高电平，L——输出低电平，R——输出并取反------------
#define GPIO_OUT_H(io, x);	{DDR##io |= (1 << (x & 0x07)); PORT##io |=  (1 << (x & 0x07));}
#define GPIO_OUT_L(io, x);	{DDR##io |= (1 << (x & 0x07)); PORT##io &= ~(1 << (x & 0x07));}
#define GPIO_OUT_R(io, x);	{DDR##io |= (1 << (x & 0x07)); PORT##io ^=  (1 << (x & 0x07));}

void timer0_init_ucos(void);
void timer3_init_ucos(void);
void port_init(void);
void board_init(void);

#endif