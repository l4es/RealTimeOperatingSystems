#ifndef __MEASURE_H
#define __MEASURE_H

#define MEASURE_PARA_SIZE			10
#define TIME0_MAX_OVF_TIMES			20			// timer0最大溢出次数，用于确定定时周期

typedef struct
{
	union
	{
		struct
		{
			float current_ratio;				// 电流增益
			float current_offset;				// 电流偏移量
			float voltage_ratio;				// 电压增益
			float voltage_offset;				// 电压偏移量
			float flow_speed_ratio;				// 流速增益
			float flow_speed_offset;			// 流速偏移量
			float torque_ratio;					// 转矩增益
			float torque_offset;				// 转矩偏移量
			float rotate_speed_ratio;			// 转速增益
			float rotate_speed_offset;			// 转速偏移量
		};
		float value[MEASURE_PARA_SIZE];
	};
}MeasurePara;

typedef struct
{
	uint8 num;								// 数值
	uint8 start_time;						// 启动时间计数值
	uint8 timer0_start_ovf_times;			// 定时器0起始溢出次数
	uint8 end_time;							// 结束时间计数值
	uint8 timer0_end_ovf_times;				// 定时器0结束溢出次数
}MeasureFrequence;

extern MeasurePara measure_para;
extern MeasurePara measure_para_eeprom  __attribute__((section(".eeprom")));
extern MeasureFrequence measure_frequence;

extern uint8 timer0_ovf_times;				// Timer0溢出次数
extern uint8 timer1_ovf_times;				// Timer1溢出次数
extern uint8 timer2_ovf_times;				// Timer2溢出次数
extern uint8 timer0_max_ovf_timers;			// Timer0最大溢出次数,用于确定定时周期

extern bool time_is_out_flag;				// 定时时间到标志
extern bool continue_measure_flag;

//void  measure_para_config_init(void);
void  measure_para_write_to_eeprom(void);
void  measure_para_read_from_eeprom(void);
void  measure_init(void);
void  measure_start(void);
void  measure_stop(void);
void  measure_time_count_read(void);
float measure_data_get(void);
void  measure_data_read(void);
void  timer0_max_ovf_timers_set(uint8 num);
void  measure_para_set(uint8 num, float fValue);
float measure_para_read(uint8 num);

#endif
