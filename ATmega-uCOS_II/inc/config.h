#ifndef __CONFIG_H__
#define __CONFIG_H__

//---------------标准头文件---------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <math.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdarg.h>  
#include <stdlib.h>    
#include <stdio.h>

//-------------常用宏(不需要修改)----------
#define ALL_IN_ROM					//将OSMapTbl数组放到ROM中

typedef unsigned char 				bool;
#define FALSE						0x00
#define TRUE						0x01
#define ENABLE						0x00
#define DISABLE						0x01

#define BITNUM_0					0x01
#define BITNUM_1					0x02
#define BITNUM_2					0x04
#define BITNUM_3					0x08

#define REVERSE(BITNUM) 			PORTC = ((~(BITNUM)) & PORTC) | ((~PORTC) & (BITNUM))

#define set_bit(reg,bit)			reg |=  _BV(bit)
#define clr_bit(reg,bit)			reg &= ~_BV(bit)

#define max(a,b) 					((a)>(b)?(a):(b))
#define min(a,b) 					((a)<(b)?(a):(b))

#define _NOP() 						asm("nop")
#define F_CPU_M  					(F_CPU/1000000)

//-------------常用宏(可修改)-------------
#define SYS_BAUDRATE 				9600
#define FRAME_HEAD					0xAA

//-------------测试宏（根据情况定义）-----
#define USE_UCOS					1
#define PCF8563_TEST				1
#define ADC_TEST					1
#define TEST						1

//------------任务堆栈大小----------------
#define OS_USER_TASK_STK_SIZE 		96

typedef unsigned char  uint8;       /* defined for unsigned 8-bits integer variable 	  	*/
typedef signed   char  int8;        /* defined for signed 8-bits integer variable		  	*/
typedef unsigned short uint16;      /* defined for unsigned 16-bits integer variable 	 	*/
typedef signed   short int16;       /* defined for signed 16-bits integer variable 		 	*/
typedef unsigned int   uint32;      /* defined for unsigned 32-bits integer variable 	 	*/
typedef signed   int   int32;       /* defined for signed 32-bits integer variable 		 	*/
typedef float          fp32;        /* single precision floating point variable (32bits)  	*/
typedef double         fp64;        /* double precision floating point variable (64bits)  	*/

//-------------自定义头文件-------------
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "app_cfg.h"
#include "cmd_queue.h"
#include "data_queue.h"
#include "board.h"
#include "uart.h"
#include "res_control.h"
#include "timer.h"
#include "adc.h"
#include "measure.h"
#include "extint.h"
#include "i2c.h"
#include "pcf8563.h"
#include "key.h"

#endif