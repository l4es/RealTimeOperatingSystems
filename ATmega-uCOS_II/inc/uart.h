#ifndef __UART_H__
#define __UART_H__

#define RXB8 				1
#define TXB8 				0
#define UPE  				2
#define OVR	 				3	
#define FE	 				4
#define UDRE 				5
#define RXC	 				7
#define RXD1           	 	2
#define TXD1           	 	3

#define	FRAMING_ERROR  		(1<<FE)
#define	PARITY_ERROR   		(1<<UPE)
#define	DATA_OVERRUN   		(1<<OVR)
#define	RX_COMPLETE    		(1<<RXC)

#define REV_EN()    		UCSR1B |= _BV(RXCIE1);
#define REV_DIS()   		UCSR1B &= ~_BV(RXCIE1);

#define TXC_EN()    		UCSR1B |= _BV(TXCIE1);
#define TXC_DIS()   		UCSR1B &= ~_BV(TXCIE1);

#define TBUF_SIZE   		32
#define BIT_MASK    		0x1F

#define TXBUF_IS_FULL		1
#define TXBUF_IS_EMPTY		2

typedef struct
{
	uint8 head;
	uint8 tail;
	uint8 buf[TBUF_SIZE];
	uint8 disable;
}SiocirQueue;

extern SiocirQueue tx_buf;

void uart_init(void);
void uart_putchar(unsigned char c);
int  uart_getchar(void);
void uart_putstring(unsigned char *str);
void kprintf(uint8 *fmt,...);

void uart_putenter(void);
void com_putchar(uint8 x);
void com_putstring(uint8 *p,uint8 len);
void com_putcommand(Cmd *pCmd);
void com_putdata(Data *pData);
void com_put_ackcommand(void);
void com_printf(const int8* fmt, ...);
void com_putenter(void);
void uart_putnstring(uint8 *p, uint8 len);

#endif
