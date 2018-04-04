#include "config.h"

OS_STK Task1Stk[OS_USER_TASK_STK_SIZE] = {0};
OS_STK Task2Stk[OS_USER_TASK_STK_SIZE] = {0};
OS_STK Task3Stk[OS_USER_TASK_STK_SIZE] = {0};
OS_STK Task4Stk[OS_USER_TASK_STK_SIZE] = {0};
OS_STK Task5Stk[OS_USER_TASK_STK_SIZE] = {0};

OS_EVENT *sem;
uint8  err = 0;

void Task1(void *pdata)
{
	pdata=pdata;
	char *str1 = "This is Test1";
	
	while(1)
	{
		//uart_putstring((char *)str1);
		//uart_putenter();
		GPIO_OUT_R(C, LED1);
		OSTimeDly(OS_TICKS_PER_SEC);
	}
}

void Task2(void *pdata)
{
	pdata=pdata;
	char *str2 = "This is Test2";
	
	while(1)
	{
		//uart_putstring((char *)str2);
		//uart_putenter();
		GPIO_OUT_R(C, LED2);
		OSTimeDly(OS_TICKS_PER_SEC+50);
	}
}

void Task3(void *pdata)
{
	pdata=pdata;
	char *str3 = "This is Test3";
 
	while(1)
	{      
		//uart_putstring((char *)str3);	
		//uart_putenter();
		GPIO_OUT_R(C, LED3);
		OSTimeDly(OS_TICKS_PER_SEC);  
	}
}

void Task4(void *pdata)
{
	pdata=pdata;
	char *str4 = "This is Test4";
	
	while(1)
	{      
		//com_printf("%s", str4);
		//uart_putenter();
		OSSemPend(sem, 0, &err);
		pcf8563_test();
		GPIO_OUT_R(C, LED4);
		OSTimeDly(OS_TICKS_PER_SEC/5); 
	}
}

void test(void *pdata)
{
	pdata=pdata;
	
	#ifdef TEST
	uint8 key_value = 1;
	#endif
	
	#ifndef ADC_TEST
	adc_convert_start();
	adc_mean_num_set(24);
	#endif
	
	while(1)
	{ 
		#ifdef TEST
		//pcf8563_test();
		//adc_test();
		key_value = key_read();
		if(key_value!=0)
		{
			OSSemPost(sem);
			//uart_putchar(key_value);
		}
		#endif
		
		#ifdef TEST
		OSTimeDly(OS_TICKS_PER_SEC/8);
		#endif
	}
}

void tasks_create(void)
{
	board_init();
	
	OSTaskCreate(Task1,0,&Task1Stk[OS_USER_TASK_STK_SIZE-1],1);
	OSTaskCreate(Task2,0,&Task2Stk[OS_USER_TASK_STK_SIZE-1],2);
	OSTaskCreate(Task3,0,&Task3Stk[OS_USER_TASK_STK_SIZE-1],5);
	OSTaskCreate(Task4,0,&Task4Stk[OS_USER_TASK_STK_SIZE-1],4); 
	OSTaskCreate(test, 0,&Task5Stk[OS_USER_TASK_STK_SIZE-1],3); 

	sem = OSSemCreate(0);
}

int main(void)
{
	OSInit();
	
	tasks_create();
	
	OSStart();

	return 0;
}

