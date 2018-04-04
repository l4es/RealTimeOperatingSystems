/************************************************
文件：pcf8563.h
用途：pcf8563函数头文件
注意：系统时钟8M
************************************************/
#include "config.h"

uint8 syserr;
uint8 *week_list[7]=
{
	(uint8 *)"Sun",
	(uint8 *)"Mon",
	(uint8 *)"Tue",
	(uint8 *)"Wed",
	(uint8 *)"Thu",
	(uint8 *)"Fri",
	(uint8 *)"Sat"
};

uint8 write_buff[8];
uint8 read_buff[8];
uint8 time_tmp[] = "00:00:00 ";
uint8 data_tmp[] = "2008-01-01 ";
uint8 *week_tmp;

uint8 time[7]=
{
	0x50/*秒*/,
	0x59/*分*/,
	0x23/*时*/,
	0x31/*天*/,
	0x02/*星期*/,
	0x12/*月/世纪*/,
	0x10/*年*/
};
/*************************************************************************
** 函数名称: clear(uint8 *p,uint8 num)
** 功能描述: 清除指定区域指定长度的数据
** 输　入: uint8 *p :起始地址   uint8 num :零数据长度
**************************************************************************/
void clear(uint8 *p,uint8 num)
{
	uint8 *pbuf = (uint8 *)p;
		
	for(; num > 0; num--)
	{
		*pbuf=0;
		pbuf++;
	}
}
/*************************************************************************
** 函数名称: pcf8536_write(uint8 addr,uint8 data)
** 功能描述: 向pcf8563指定地址写入一条数据
** 输　入: uint8 addr    ：高八位为器件地址，低八位为内部寄存器地址
uint8 data  ：需要写入的数据
**************************************************************************/
void pcf8536_write(uint8 addr,uint8 data)
{
	i2c_start();
	
	if(i2c_write_byte(WR_ADDR_CMD)==SLA_W)
	{
		i2c_write_byte(addr);
		i2c_write_byte(data);
	}
	else 
	{
		syserr=ERR_SLA_W;
	}
	
	i2c_stop();
}
/*************************************************************************
** 函数名称: pcf8536_write_p(uint8 addr,uint8 *p,uint8 num)
** 功能描述: 向pcf8563地址连续的寄存器写入系列数据
** 输　入: uint8 addr    ：高八位为器件地址，低八位为内部寄存器地址
uint8 *p    ：需要写入的数据域的起始地址
uint8 num   ：写入数据的个数
** 说明：写入数据区域为地址连续的寄存器
**************************************************************************/
void pcf8536_write_p(uint8 addr,uint8 *p,uint8 num)
{
	uint8 *pbuf = p;
	
	i2c_start();
	if(i2c_write_byte(WR_ADDR_CMD)==SLA_W)
	{
		i2c_write_byte(addr);
		for(;num>0;num--)
		{
			i2c_write_byte(*pbuf);
			pbuf++;
			_NOP();
		}
	}
	else 
	{
		syserr=ERR_SLA_W;
	}
	i2c_stop();
}
/*************************************************************************
** 函数名称: pcf8536_read(uint8 addr,uint8 *p,uint8 num)
** 功能描述: 读pcf8563
** 输　入: uint8 addr    ：高八位为器件地址，低八位为内部寄存器地址
uint8 *p    ：读出的数据存放地址的起始地址
uint8 num   ：读出数据的个数
**************************************************************************/
void pcf8536_read(uint8 addr,uint8 *p,uint8 num)
{
	uint8 *pbuf = p;

	i2c_start();
	if(i2c_write_byte(WR_ADDR_CMD)==SLA_W)
	{
		i2c_write_byte(addr);
	}
	else 
	{
		syserr=ERR_SLA_W;
	}

	i2c_start();
	if(i2c_write_byte(RD_ADDR_CMD)==SLA_W)
	{
		i2c_write_byte(addr);
	}
	else 
	{
		syserr=ERR_SLA_W;
	}

	for(; num > 0; num--)
	{
		*pbuf = i2c_read_byte();
		pbuf++;
	}
}
/*************************************************************************
** 函数名称: pcf8563_init(void)
** 功能描述: pcf8563初始化
** 输　入: 
** 输出	 : 
** 全局变量: 
** 调用模块: 
** 说明：
** 注意：
**************************************************************************/
void pcf8563_init(void)
{
	clear(write_buff,8);
	clear(read_buff,8);
	pcf8563_stop();
	pcf8563_time_set((uint8 *)time);
	pcf8563_start();
}
/*************************************************************************
** 函数名称: pcf8563_time_sort(void)
** 功能描述: 刷新时间寄存器及相关数组内容
** 输　入: 
** 输出	 : 
** 全局变量: 
** 调用模块: 
** 说明：
** 注意：
**************************************************************************/
void pcf8563_time_sort(void)
{
	time[0]=time[0]&0x7F;	//S
	time[1]=time[1]&0x7F;	//M
	time[2]=time[2]&0x3F;	//H
	time[3]=time[3]&0x3F;	//D
	time[4]=time[4]&0x07;	//W
	time[5]=time[5]&0x1F;	//M
	time[6]=time[6]&0xFF;	//Y	
	
	time_tmp[0]=(time[2]>>4)+0x30;
	time_tmp[1]=(time[2]&0x0F)+0x30; 	//H
	time_tmp[3]=(time[1]>>4)+0x30;
	time_tmp[4]=(time[1]&0x0F)+0x30;	//M
	time_tmp[6]=(time[0]>>4)+0x30;
	time_tmp[7]=(time[0]&0x0F)+0x30;	//S
	
	data_tmp[2]=(time[6]>>4)+0x30;	
	data_tmp[3]=(time[6]&0x0F)+0x30;	//Y
	data_tmp[5]=(time[5]>>4)+0x30;
	data_tmp[6]=(time[5]&0x0F)+0x30;	//M
	data_tmp[8]=(time[3]>>4)+0x30;
	data_tmp[9]=(time[3]&0x0F)+0x30;	//D
	
	week_tmp=week_list[time[4]&0x0F];	//W
}

void pcf8563_start(void)
{
	pcf8536_write(PCF8563_CSR1, PCF8563_START);
}

void pcf8563_stop(void)
{
	pcf8536_write(PCF8563_CSR1, PCF8563_STOP);
}

void pcf8563_time_set(uint8 *time_list)
{
	pcf8536_write_p(PCF8563_SEC, time_list, 7);		//pcf8563写或坊时具有地址自增功能
}

void pcf8563_time_get(uint8 *time_list)
{
	pcf8536_read(PCF8563_SEC, time_list, 7);
}

#if PCF8563_TEST == 1
void pcf8563_test(void)
{	
	uint8 temp = time[0];
	pcf8563_time_get(time);
	
	pcf8563_time_sort();
	
	if(temp != time[0])
	{
		uart_putnstring((uint8 *)data_tmp, 11);
		uart_putnstring((uint8 *)time_tmp, 9);
		uart_putstring((uint8 *)week_tmp);
		uart_putenter();
	}
	#ifndef USE_UCOS
	//OSTimeDly(OS_TICKS_PER_SEC/2); 
	#endif
	
	#ifndef USE_UCOS
	//_delay_ms(1000);
	#endif
}
#endif