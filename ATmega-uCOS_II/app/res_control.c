#include "config.h"

float resistance_table[6] = 
{ 
	330.0, 
	330.0, 
	470.0, 
	470.0, 
	470.0, 
	150.0 
};

float rev_restable[6] = 
{ 
	0.0030303, 
	0.0030303, 
	0.0021277,
	0.0021277,
	0.0021277,
	0.0066667 
};

void res_ctrl_init(void)
{
	RES_PORT = 0x3F;
	RES_DDR	= 0x3F;
}

void res_set(uint8 mask, uint8 value)
{
	RES_PORT &= ~(0x3F & mask);
	RES_PORT |= ((0x3F & mask) & value);
}
