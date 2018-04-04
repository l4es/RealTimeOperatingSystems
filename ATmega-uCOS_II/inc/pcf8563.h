/************************************************
文件：pcf8563.h
用途：pcf8563函数头文件
注意：系统时钟8M
************************************************/
#ifndef __PCF8563_H__
#define __PCF8563_H__

//器件地址格式： 1 0 1 0   0 0 A0 R/W	
//				 1 0 1 0   0 0  1  0 -- A2	写命令
//				 1 0 1 0   0 0  1  1 -- A3  读命令 	
#define WR_ADDR_CMD 			0xA2				//写地址命令
#define RD_ADDR_CMD 			0xA3				//读地址命令

//---------------返回状态码-------------------
#define SLA_W 					0x18				//SLA+W已发送，并接收到ACK
#define ERR_SLA_W 				0x01				//接收错误

//-------------PCF8563内部寄存器地址----------
#define PCF8563_CSR1			0x00				//控制状态寄存器1
#define PCF8563_CSR2			0x01				//控制状态寄存器2
#define PCF8563_CLKOUT			0x0D				//CLKOUT频率寄存器
#define PCF8563_TCR				0x0E				//定时器控制寄存器
#define PCF8563_RTDR			0x0F				//定时器倒计数值寄存器

#define PCF8563_SEC				0x02				//秒寄存器
#define PCF8563_MIN				0x03				//分寄存器
#define PCF8563_HOU				0x04				//时寄存器
#define PCF8563_DAY				0x05				//日寄存器
#define PCF8563_WEK				0x06				//星期寄存器
#define PCF8563_MON				0x07				//月、世纪寄存器
#define PCF8563_YEA				0x08				//年寄存器
#define PCF8563_ALM_MIN			0x09				//分钟报警
#define PCF8563_ALM_HOU			0x0A				//小时报警
#define PCF8563_AIM_DAY			0x0B				//日报警
#define PCF8563_ALM_WEK			0x0C				//星期报警

//-------------PCF8563简单命令-----------------
#define PCF8563_STOP			0x20
#define PCF8563_START			0x00

//----------------函数声明---------------------
void clear(uint8 *p, uint8 num);
void pcf8563_init(void);
void updata_time(void);
void pcf8563_cmd(uint8 ctrl_cmd);
void pcf8536_write_p(uint8 addr,uint8 *p,uint8 num);
void pcf8536_write(uint8 addr,uint8 data);
void pcf8563_start(void);
void pcf8563_stop(void);
void pcf8563_time_set(uint8 *time_list);
void pcf8563_time_get(uint8 *time_list);

#if PCF8563_TEST == 1
void pcf8563_test(void);
#endif
//----------------全局变量区-------------------
extern uint8 write_buff[8];
extern uint8 read_buff[8];
extern uint8 time_tmp[];
extern uint8 data_tmp[];
extern uint8 *week_tmp;
extern uint8 time[7];

#endif
