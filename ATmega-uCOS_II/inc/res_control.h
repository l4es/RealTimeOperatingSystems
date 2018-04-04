#ifndef __RESCTRL_H__
#define __RESCTRL_H__

#define		RES_1		0x10
#define		RES_2		0x20
#define		RES_3		0x02
#define		RES_4		0x04
#define		RES_5		0x08
#define		RES_6		0x01

#define		RES_PORT	PORTA
#define		RES_DDR		DDRA
#define		RES_PIN		PINA

#define		EnableRes(bit);		{RES_PORT |= (bit);}
#define		DisableRes(bit);	{RES_PORT &= ~(bit);}
#define		ResIsEnable(bit);	{((RES_PORT & (bit)) != 0);}

extern float resistance_table[6];
extern float rev_restable[6];
extern void  res_ctrl_init(void);

extern void  res_set(uint8 mask, uint8 value);

#endif
