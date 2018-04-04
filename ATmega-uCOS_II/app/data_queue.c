#include "config.h"

// ---------------定义变量----------------------
DataQueue		data_queue;
DataBuf			data_buf;

void data_buf_init(void)							//数据缓冲区初始化（只有一帧数据）
{
	memset(&data_buf, 0x00, sizeof(DataBuf));

	data_buf.data.data_code = DATA_CODE;			//DATA_CODE=0x55
	data_buf.data.data_size = DATA_MAX_SIZE;		//DATA_MAX_SIZE=24,取出6个数据（4字节表示一个数据）
	data_buf.data.data_end  = DATA_END;				//DATA_END=0x0A('\n')
}

void data_queue_init(void)							//数据队列初始化（四帧数据）
{
	memset(&data_queue, 0x00, sizeof(DataQueue));
}

void data_queue_add_data(Data* pData)				//往数据队列中添加一帧数据
{
	uint8 i;

	if(data_queue_status() == DATAQUEUE_STATUS_FULL)//如果数据队列已满，则返回
		return;

	for(i = 0; i < DATA_MAX_SIZE; i++)				//此处是为了与LabVIEW配合，因为设置的终止符为0x0A
	{
		if(pData->cDatas[i] == 0x0A )
			pData->cDatas[i] = 0x0B;
	}
	memcpy(&(data_queue.data[data_queue.tail]), pData, sizeof(Data));//将一帧pData拷到数据队列的尾部

	data_buf_init();								// 清空数据缓存
	DATAQUEUE_TAIL_ADD();							// 数据个数加1
}

void data_queue_remove_data(void)					//从数据队列中去掉一帧数据
{
	if(data_queue_status() == DATAQUEUE_STATUS_EMPTY)
		return;

	DATAQUEUE_HEAD_ADD();							//头部向后移动，作用等同于丢掉一帧数据
}

Data* data_queue_headptr_get(void)					//获得数据队列头指针
{
	if(data_queue_status() == DATAQUEUE_STATUS_EMPTY)
		return NULL;

	return (&data_queue.data[data_queue.head]);
}

bool data_queue_status(void)
{
	if(data_queue.tail == data_queue.head)
		return DATAQUEUE_STATUS_EMPTY;
		
	if(data_queue.head == (data_queue.tail + 1) % DATA_MAX_NUM)
		return DATAQUEUE_STATUS_FULL;
		
	else
		return DATAQUEUE_STATUS_HASDATA;
}
