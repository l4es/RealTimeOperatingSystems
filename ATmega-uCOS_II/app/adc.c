#include "config.h"
//注：在ADC转化时应该先对ADC硬件进行初始化，然后选择通道，设置
//最后 再启动ADC转化，本程序中，ADC只能进行单次转化，而非连续转化。
//另外，ADC队列操作时，一定要先加入电流值 ，再加入电压值，记住不要忘记
//调整队列元素的指针。
//-----------------------
// 定义变量
AdcStatusFlag adc_status;							// ADC状态结构体变量

unsigned int  adc_result;							// 保存了转换结果
unsigned char adc_mean_num = ADC_MAX_SIZE;			// 在滑动求平地均的算法中, 时间窗的大小,(即对最近的多少个值求平均)
AdcQueue 	  adc_queue;							// 类型于环形队列, 保存最近的转换结果

//-----------------------
void adc_init(void)
{
	ADC_OFF();										// 关闭ADC
	ADMUX = _BV(REFS0)|_BV(MUX1)|_BV(MUX0); 		// 外部参考电源VACC，选择ADC3（空）
	ACSR  = _BV(ACD);								// 禁止ACD并关闭其电源
	ADC_ON();										// 打开ADC,使能ADC中断，并对其进行2分频
	adc_queue_init();
}

static void adc_switch_channel(bool channel)		// 切换AD转换通道
{
	switch(channel)
	{
		case CURRENT:
			ADMUX = _BV(REFS0) | ADC_CURRENT; 		// ADC1 电流
			break;
			
		case VOLTAGE:
			ADMUX = _BV(REFS0) | ADC_VOLTAGE; 		// ADC2 电压
			break;
			
		default:
			break;
	}
}
// AD中断服务程序，缺点是电压，电流各转换两次，并丢掉第一次采集值

ISR(ADC_vect)										
{
	if(adc_status.who_is_convert == CURRENT)		// 如果电流正在转换
	{
		if(adc_status.first_time == TRUE)			// 并且第一次开始转换
		{
			adc_status.first_time = FALSE;			// 置第一次转换标记为F
			START_ADC();							// 启动AD转换
		}
		else
		{
			adc_result = ADC;						// 保存电流转换值
			adc_queue_add_current(adc_result);		// 将电流值加入到ADC队列中

			adc_status.first_time = TRUE;			// 重新置第一次转换开关为T
			adc_status.who_is_convert = VOLTAGE;	// 因电流转换已完成，置其标记为F

			adc_switch_channel(VOLTAGE);			// 切换通道到电压转换
			START_ADC();							// 启动AD转换
		}
	}
	else if(adc_status.who_is_convert == VOLTAGE)	// 如果当前电压正在转换
	{
		if(adc_status.first_time == TRUE)			// 是第一次转换
		{	
			adc_status.first_time = FALSE;			// 置标记为F
			START_ADC();							// 启动AD转换（转换电压）
		}
		else										// 如果不是第一次
		{
			adc_result = ADC;						// 则读取电压值
			adc_queue_add_voltage(adc_result);		// 将电压值加入ADC队列中
		}
	}
}

//---------------ADC队列函数部分------------------
void adc_queue_init(void)
{
	uint8 *p = (uint8 *)&adc_queue;
	memset(p, 0x00, sizeof(AdcQueue));		// 清空ADC队列缓冲区
}

void adc_convert_start(void)
{
	memset(&adc_status, 0, sizeof(AdcStatusFlag));
	adc_status.who_is_convert 	= CURRENT;			// 置位电流转换，先电流后电压顺序
	adc_status.first_time 		= TRUE;				// 第一次转换
	adc_switch_channel(CURRENT);					// 切换到电流转换

	START_ADC();									// 启动一次转换（没有采用连续转换）
}

void adc_queue_add_current(short int data)
{
	adc_queue.data[adc_queue.tail].current = data;	// 在队列尾部添加电流数据
}

void adc_queue_add_voltage(short int data)			
{
	adc_queue.data[adc_queue.tail].voltage = data;	// 在队列尾部添加电压数据
	//adc_queue.data[adc_queue.tail].wPower = data * adc_queue.data[adc_queue.tail].current;

	ADCQUEUE_TAIL_ADD();							// 调整尾部下标

	adc_queue.count++; 								// 数据个数加1
	adc_queue.count = min(adc_queue.count, ADC_MAX_SIZE);  //ADC_MAX_SIZE=64
}

void adc_mean_num_set(unsigned char num)			//设置的范围是1到64
{
	adc_mean_num = min(num, ADC_MAX_SIZE);
	adc_mean_num = max(num, 1);
}

void adc_queue_get_data(float* pcurrent, float* pvoltage, float* ppower)
{
	unsigned char i;
	unsigned char n;
	unsigned char tail;
	short int res_current 	= 0;
	short int res_voltage 	= 0;
	short int temp_current 	= 0;
	short int temp_voltage 	= 0;
	float 	  resPower 		= 0.0f;

	n = min(adc_mean_num, adc_queue.count);			// 取当前队列中数据个数与设置值的最小值
	if(0 == n)
		return;

	tail = adc_queue.tail;							// 获取队列尾部下标
	tail = (tail-1) & ADC_MAX_SIZE_MASK;			// 减1之后,指向最后加入的一个数据
													// 减1是因为下标是从0开始的
	for(i = 0; i < n; i++)
	{
		temp_current = adc_queue.data[tail].current;
		temp_voltage = adc_queue.data[tail].voltage;
		res_current += temp_current;
		res_voltage += temp_voltage;

		resPower += (5.0f * temp_current * measure_para.current_ratio / 1024.0f + measure_para.current_offset) * 
			(5.0f * temp_voltage * measure_para.voltage_ratio / 1024.0f + measure_para.voltage_offset);

		tail = (tail-1) & ADC_MAX_SIZE_MASK;
	}

	*pcurrent = 5.0f * res_current * measure_para.current_ratio / n / 1024.0f + measure_para.current_offset;
	*pvoltage = 5.0f * res_voltage * measure_para.voltage_ratio / n / 1024.0f + measure_para.voltage_offset;
	*ppower = resPower / n;
}
/*
ISR(ADC_vect)
{
	adc_result = ADC;
	adc_queue_add_voltage(adc_result);
}
*/
#ifdef ADC_TEST
void adc_test(void)
{
	unsigned char i;
	unsigned char n;
	unsigned char tail;
	short int res_voltage 	= 0;
	short int temp_voltage 	= 0;
	uint16 temp_adc = 0;
	float pvoltage = 0;

	adc_switch_channel(VOLTAGE);
	for(i = 0; i < 50; i++)
	{
		START_ADC();
		temp_adc +=  adc_result;
	}
	
	n = min(adc_mean_num, adc_queue.count);			// 取当前队列中数据个数与设置值的最小值
	if(0 == n)
		return;

	tail = adc_queue.tail;							// 获取队列尾部下标
	tail = (tail-1) & ADC_MAX_SIZE_MASK;			// 减1之后,指向最后加入的一个数据
													// 减1是因为下标是从0开始的
	for(i = 0; i < n; i++)
	{
		temp_voltage = adc_queue.data[tail].voltage;
		res_voltage += temp_voltage;

		tail = (tail-1) & ADC_MAX_SIZE_MASK;
	}
	pvoltage = 5.0f * res_voltage / n / 1024.0f;
	uart_putnstring((uint8 *)&pvoltage, 4);
	OSTimeDly(OS_TICKS_PER_SEC/2);
}
#endif
