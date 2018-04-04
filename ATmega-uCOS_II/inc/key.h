#ifndef __KEY_H_
#define __KEY_H_

#define		KEY_PORT	PORTC
#define		KEY_DDR		DDRC
#define		KEY_PIN		PINC

#define		KEY_1		0x10
#define		KEY_2		0x20
#define		KEY_NONE	0x00

#define KEYVALUE_SIZE		0x08
#define KEYVALUE_SIZE_MASK	0x07

typedef struct{
	bool IsUp;
	unsigned char rear;
	unsigned char value[KEYVALUE_SIZE];
} KEYVALUEQUEUE;

extern void key_value_queue_init(void);
extern unsigned char key_read(void);

#endif
