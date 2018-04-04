// AVR application builder : 2010-9-2 上午 12:37:32
// Target : M128
// Crystal: 8.0000Mhz

#include "config.h"

SiocirQueue tx_buf;

static void tx_buf_init(void)							//发送缓冲区初始化
{
	tx_buf.head 	 = 0;
	tx_buf.tail  	 = 0;
	tx_buf.disable   = DISABLE;							//缓冲区不可用
}

static bool tx_buf_status(void)							//空：2，满：1，其他：0
{
	if(tx_buf.tail == tx_buf.head)
		return TXBUF_IS_EMPTY;							//检测发送缓冲区是否为空，2为空，0为非空
	
	if(((tx_buf.tail + 1) & BIT_MASK) == tx_buf.head)
		return TXBUF_IS_FULL;							//检测发送缓冲区是否已满，1为满，0为非满
	
	else
		return 0;
}

// UART1 initialize
// desired baud rate: 9600
// actual: baud rate: 9615 (0.2%)
static void uart_set_baudrate(uint32 baudrate)
{
	uint32 tmp;
	tmp    = (F_CPU/baudrate) / 16 - 1;					//F_CPU was defined in Makefile

	UBRR1L = (uint8)tmp;						//set baud rate hi
	UBRR1H = (uint8)(tmp>>8);					//set baud rate lo
}

void uart_init(void)
{
	DDRE   |=  (1<<TXD1);
	DDRE   &= ~(1<<RXD1);
	UCSR1B  = 0x00; 									//disable while setting baud rate
	UCSR1A  = 0x00;
	UCSR1C |= _BV(UCSZ11) | _BV(UCSZ10);				//8位数据，波特率9600，8N1,无检验
	UCSR1B |= _BV(RXEN1)  | _BV(TXEN1) | _BV(TXCIE1) | _BV(RXCIE1);//使能发送，接收，发送中断，接收中断  
	
	uart_set_baudrate(SYS_BAUDRATE);					//设置波特率，9600
	tx_buf_init();										//清空接收缓冲区
}

//向UART写一字节
void uart_putchar(unsigned char c)
{
    if(c == '\n')
	{
        uart_putchar(0x0D);								//这里存在一定的问题，就是如果
	}													//发送0A时会先发送0D然后再发送0A，
	//else
    {	loop_until_bit_is_set(UCSR1A, UDRE1);				//相当于发送了两个字节？？？？
		UDR1 = c;
	}
}

//从UART读一字节
int uart_getchar(void)
{
    loop_until_bit_is_set(UCSR1A, RXC1);
    return UDR1;
}

void uart_putenter(void)
{
	uart_putchar(0x0A);
	uart_putchar(0x0D);
}

void uart_putstring(unsigned char *str)							//基于简单方式的发送字符串函数
{
	unsigned char *pbuf = (unsigned char *)str;
	
	if(pbuf != NULL)
	{
		while(*pbuf)
		{
			uart_putchar(*pbuf++);
		}
	}
}

void kprintf(uint8 *fmt,...)
{
	va_list ap;
	uint8 string[64];
	va_start(ap, fmt);
	vsprintf(string, fmt, ap);
	uart_putstring(string);
	va_end(ap);
}
void uart_putnstring(uint8 *p, uint8 len) 		//基于简单方式的发送指定长度字符串函数
{
	uint8 *pbuf = p;
	
	if (!len)
		return;

	for(; len > 0; len--)
	{
		uart_putchar(*pbuf++);
	}
}

void com_printf(const int8* fmt, ...)  
{  
    const int8* s;  
    int d;  
    int8 buf[32]; 
	
    va_list ap;  
    va_start(ap, fmt);   								// 将ap指向fmt（即可变参数的第一个?下一个？）  
    while (*fmt)  
    {  
        if (*fmt != '%')  
        {  
            uart_putchar(*fmt++);   					// 正常发送  
            continue;     
        } 
		if(*fmt == '%')
		{
			fmt++;
		}
        switch (*fmt) 								// next %  
        {  
			case 's':  
				s = va_arg(ap, const uint8*); 			// 将ap指向者转成uint8*型，并返回之  
				for (; *s; s++)  
				{
					uart_putchar(*s);  
				}
				break;  
				
			case 'x':  
				d = va_arg(ap,int);     				// 将ap指向者转成int型，并返回之  
				itoa(d, buf, 16);         				// 将整型d以16进制转到buf中  
				for (s = buf; *s; s++) 
				{
					uart_putchar(*s); 
				}
				break;
				
			case 'd':  
				d = va_arg(ap,int);  
				itoa(d, buf, 10);         				// 将整型d以10进制转到buf中  
				for (s = buf; *s; s++)  
                {
					uart_putchar(*s);  
				}
				break; 
				
			default:  
				uart_putchar(*fmt);  
				break;  
		}  
        fmt++;  
    }  
    va_end(ap);  
} 

void com_putchar(uint8 x)						
{
	if(tx_buf_status() == TXBUF_IS_FULL)				//若发送缓冲区已满，直接返回
		return;											//是直接返回还是在这里等待？？？？？？

	TXC_DIS();											//关闭发送完成中断
	if(tx_buf.disable == DISABLE)						//如果发送缓冲区是不可用的
	{
		while(!(UCSR1A & (1<<UDRE1))); 					//如果接收数据寄存器不为空则等待
		UDR1 = x;										//把当前要发送的数据先发出去
		tx_buf.disable = ENABLE;						//置发送缓冲区可用
	}
	else
	{
		tx_buf.buf[tx_buf.tail] = x;					//将发送的数据放到缓冲区尾部
		tx_buf.tail = (tx_buf.tail + 1) & BIT_MASK;		//移动缓冲区尾部指针
	}
	TXC_EN();											//打开发送完成中断
}

void com_putstring(uint8 *p, uint8 len) 
{
	uint8 i;
	if (!len)
		return;

	for(i = 0; i < len; i++)
	{
		com_putchar(p[i]);
	}
}

void com_putenter(void)
{
	com_putchar('\n');
	com_putchar('\r');
}

void com_putcommand(Cmd *pCmd)
{
	com_putchar(FRAME_HEAD);
	com_putstring((uint8*)pCmd, pCmd->para_size + 2);
}

void com_putdata(Data *pData)
{
	com_putchar(FRAME_HEAD);
	com_putstring((uint8*)pData, DATA_LENGTH);
}

void com_put_ackcommand(void)
{
	Cmd cmd;
	cmd.cmd_code 	= 0x01;
	cmd.para_size 	= 0;
	com_putcommand(&cmd);
}


ISR(USART1_RX_vect)									//接收完成中断服务子程序
{
	uint8 status = UCSR1A;					//判断状态
	uint8 data   = UDR1;					//读取接收数据

	if((status & (FRAMING_ERROR | DATA_OVERRUN)) == 0)
	{
		cmd_rcvbuf_add_char(data);
	}
}

ISR(USART1_TX_vect)									//发送完成中断服务子程序
{
	if (tx_buf_status() != TXBUF_IS_EMPTY)			//判断发送缓冲区是否为空，不为空则发送缓冲区数据
	{
		UDR1 = tx_buf.buf[tx_buf.head];				//从发送缓冲区的头部开始发送
		tx_buf.head = (tx_buf.head+1) & BIT_MASK;	//调整头部位置
	}
	else 											//空则置disable=1
	{
		tx_buf.disable = DISABLE;					//置位缓冲区不可用
	}
}
//*/