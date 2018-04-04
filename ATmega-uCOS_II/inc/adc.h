#ifndef __ADC_H_
#define __ADC_H_

//-----------------------
// ADC数据队列的最大长度
#define ADC_MAX_SIZE			64
#define ADC_MAX_SIZE_MASK		0x3F

#define ADC0 					0
#define ADC1 					1
#define ADC2 					2
#define ADC3 					3
#define ADC4 					4
#define ADC5 					5
#define ADC6 					6
#define ADC7 					7

#define ADC_CURRENT				ADC1
#define ADC_VOLTAGE				ADC2

#define CURRENT					1
#define VOLTAGE					2
//-----------------------

#define	START_ADC();			{ADCSRA |= _BV(ADSC);} // 启动ADC测量
#define	ADC_ON();				{ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADPS0);} //2分频
#define	ADC_OFF();				{ADCSRA = 0x00;}

#define ADCQUEUE_TAIL_ADD();	{adc_queue.tail = (adc_queue.tail + 1) & ADC_MAX_SIZE_MASK;}

// 标识ADC转换的状态
typedef struct
{
	bool who_is_convert;				// 标识正在进行电流转换(current), 还是电压转换(voltage)
	bool first_time;					// 标识此次转换结果是否是切换通道后的第一次转换结果
}AdcStatusFlag;

typedef struct
{
	int16 current;						//电流
	int16 voltage;						//电压
	//long int wPower;
}Pair;

typedef struct
{
	uint8 tail;							//尾标记
	uint8 count;						//数量
	Pair data[ADC_MAX_SIZE];			//存放的数据
} AdcQueue;

//-----------------------
extern AdcStatusFlag adc_status;
extern uint8 adc_mean_num;				// 在滑动求平地均的算法中, 时间窗的大小,(即对最近的多少个值求平均)
extern AdcQueue adc_queue;				// 类型于环形队列, 保存最近的转换结果

//-----------------------
void adc_init(void);
//void adc_switch_channel(bool channel);
ISR(ADC_vect);

//-----------------------
void adc_queue_init(void);
void adc_convert_start(void);
void adc_queue_add_current(int16 data);
void adc_queue_add_voltage(int16 data);
void adc_queue_get_data(float* pcurrent, 
						float* pvoltage,
						float* ppower);
void adc_mean_num_set(uint8 num);

#ifdef ADC_TEST
void adc_test(void);
#endif

#endif
