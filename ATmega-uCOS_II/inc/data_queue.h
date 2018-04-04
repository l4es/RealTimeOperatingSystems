#ifndef __DATAQUEUE_H_
#define __DATAQUEUE_H_

#define DATA_MAX_NUM				4
#define DATA_MAX_NUM_MASK			0x03
#define DATA_MAX_SIZE				24
#define DATA_LENGTH					(DATA_MAX_SIZE+3)		//起始结束标记+数据长度=2+1=3

#define	DATA_CODE					0x55
#define	DATA_END					0x0A

#define DATAQUEUE_STATUS_FULL		1
#define DATAQUEUE_STATUS_EMPTY		2
#define DATAQUEUE_STATUS_HASDATA	0

#define DATAQUEUE_TAIL_ADD();		{data_queue.tail=(data_queue.tail+1)&DATA_MAX_NUM_MASK;} //在尾部添加一帧
#define DATAQUEUE_HEAD_ADD();	 	{data_queue.head=(data_queue.head+1)&DATA_MAX_NUM_MASK;} //作用是去掉头部一帧

//-----------------数据结构体-----------------------
typedef struct
{
	uint8 data_code;					//数据或命令代码，0x55表示数据，其它为命令
	uint8 data_size;					//数据长度
	union
	{
		uint8 cDatas[DATA_MAX_SIZE];	//数据缓冲区
		float fDatas[DATA_MAX_SIZE/4];			//存在浮点数个数，即以下6个变量，本程序未用到
		struct									//以下结构体也未用到~！！
		{
			float fCurrent;						//电流
			float fVoltage;						//电压
			float fPower;						//功率
			float fFlowSpeed;					//流速
			float fTorque;						//转矩
			float fRotateSpeed;					//转速
		};
	};
	uint8 data_end;						//数据尾
}Data;

typedef struct
{
	uint8 status;						//状态
	Data data;									//一帧数据
}DataBuf;

typedef struct
{
	uint8 head;							//数据队列头
	uint8 tail;							//数据队列尾
	Data data[DATA_MAX_NUM];					//队列元素缓冲区，共DATA_MAX_NUM帧数据
}DataQueue;

extern DataQueue	data_queue;
extern DataBuf		data_buf;

void  data_buf_init(void);
void  data_queue_init(void);
void  data_queue_add_data(Data* pData);
void  data_queue_remove_data(void);
Data* data_queue_headptr_get(void);
bool  data_queue_status(void);

#endif
