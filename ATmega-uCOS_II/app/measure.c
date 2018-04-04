#include "config.h"

//---------------全局变量声明区--------------------
uint8 timer0_ovf_times = 0x00;					// Timer0溢出次数
uint8 timer1_ovf_times = 0x00;					// Timer1溢出次数
uint8 timer2_ovf_times = 0x00;					// Timer2溢出次数
uint8 timer0_max_ovf_timers = TIME0_MAX_OVF_TIMES;		// Timer0最大溢出次数,用于确定定时周期
bool time_is_out_flag = FALSE;					// 定时时间到标志
bool continue_measure_flag = FALSE;				// 继续测量标志

uint32 timer1_count = 0;						// T1计数值
uint32 timer2_count = 0;						// T2计数值

MeasurePara measure_para;						// 测量参数结构体
MeasurePara measure_para_eeprom 	__attribute__((section(".eeprom")));
MeasureFrequence measure_frequence;

//----------------函数实现------------------------
static void measure_para_init(void)					// 测量参数初始化
{
	measure_para.current_ratio 			= 1.0f;
	measure_para.current_offset 		= 0.0f;
	measure_para.voltage_ratio 			= 1.0f;
	measure_para.voltage_offset 		= 0.0f;
	measure_para.flow_speed_ratio 		= 1.0f;
	measure_para.flow_speed_offset 		= 0.0f;
	measure_para.torque_ratio 			= 1.0f;
	measure_para.torque_offset 			= 0.0f;
	measure_para.rotate_speed_ratio 	= 1.0f;
	measure_para.rotate_speed_offset	= 0.0f;

	measure_init();
}

void measure_para_write_to_eeprom(void)			//将默认测量参数保存到EEPROM中
{
	float value[MEASURE_PARA_SIZE] = {1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f};
	eeprom_write_block(value, &measure_para_eeprom, MEASURE_PARA_SIZE*sizeof(float));
}

void measure_para_read_from_eeprom(void)		//将测量参数从EEPROM中读取到measure_para中
{
	eeprom_read_block(&measure_para, &measure_para_eeprom, MEASURE_PARA_SIZE*sizeof(float));
}

void measure_init(void)
{
	memset(&measure_frequence, 0x00, sizeof(MeasureFrequence));
}

void measure_start(void)						// 启动测量
{
	timers_stop();
	timers_ovf_times_reset();				
	timers_cnt_reset();							// Timer0用于内部时钟,每次溢出间隔时间为20ms
	measure_init();
	TIMER012_TOIE_ENABLE();						// 使能Timer0 Timer1 Timer2 的溢出中断
	ADC_ON();									// 开启AD
	EXTINT_ENABLE();							// 开启外部中断
	adc_queue.count = 0x00;						// 计数清零
	adc_convert_start();						// 启动AD转换, 测量一组电流电压
	timers_start();
}

void measure_stop(void)
{
	timers_stop();
	TIMER012_TOIE_DISABLE();					// 禁止Timer0 Timer1 Timer2 的溢出中断
	ADC_OFF();									// 关闭AD
	EXTINT_DISABLE();							// 关闭外部中断
}

void measure_time_count_read(void)				// 流速测量
{
	measure_frequence.num++;					// 计数加1

	if(1 == measure_frequence.num)				// 如果是第一次
	{
		measure_frequence.start_time = TCNT0;	// 记录下定时器计数值以起始时间
		measure_frequence.timer0_start_ovf_times = timer0_ovf_times;//记录溢出次数
	}
	else
	{
		measure_frequence.end_time = TCNT0;		// 如果不是第一次，则再次记录下T0当前计数值
		measure_frequence.timer0_end_ovf_times = timer0_ovf_times;	//记录溢出次数？？？
	}
}

float measure_data_get(void)
{
	uint16 nFlowTimeCount;
	if(measure_frequence.num <= 1)
		return 0.0f;

	nFlowTimeCount  = measure_frequence.end_time  +((uint16)measure_frequence.timer0_end_ovf_times*(uint16)(256-TIMER0_INIT_VALUE));	// 获取最后一个脉冲的时间
	nFlowTimeCount -= measure_frequence.start_time+((uint16)measure_frequence.timer0_start_ovf_times*(uint16)(256-TIMER0_INIT_VALUE));// 减去第一个脉冲的时间

	if(!nFlowTimeCount)							// 如果为零, 几乎不可能
		return 0.0f;

	return (7800.0f * measure_para.flow_speed_ratio * (measure_frequence.num-1) / nFlowTimeCount
		+ measure_para.flow_speed_offset);
}

void measure_data_read(void)
{
	timer1_count = /*timer1_ovf_times * 0xFFFF +*/ TCNT1;
	timer2_count = ((uint16)timer2_ovf_times << 8) + TCNT2;

	// 读取数据
	data_buf.data.fTorque = 
		timer1_count * 40.0f / timer0_max_ovf_timers * measure_para.torque_ratio + measure_para.torque_offset;
	data_buf.data.fRotateSpeed = 
		timer2_count * 40.0f / timer0_max_ovf_timers * measure_para.rotate_speed_ratio + measure_para.rotate_speed_offset;

	adc_queue_get_data(&data_buf.data.fCurrent, &data_buf.data.fVoltage, &data_buf.data.fPower );
	data_buf.data.fFlowSpeed = measure_data_get();

	// 把数据缓冲区中所有数据放到数据发送队列中
	data_queue_add_data(&data_buf.data);		// 添加到数据发送队列中

	if(continue_measure_flag)					// 如果还需要继续测量		
		measure_start();						// 重新开始测量
}

void timer0_max_ovf_timers_set(uint8 num)		// 设置定时器0最大溢出数值（>1）
{
	timer0_max_ovf_timers = max(num, 1);
}

void measure_para_set(uint8 num, float value)	// 测量参数设置
{
	measure_para.value[num] = value;
}

float measure_para_read(uint8 num)				// 读取测量参数
{
	return measure_para.value[num];
}
