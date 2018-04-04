#include "config.h"

/*********************************************************************************************************
** 函数名称: QueueCreate
** 功能描述: 初始化数据队列
** 输　入: data_buf 	：为队列分配的存储空间地址
**         buf_len		：为队列分配的存储空间大小（字节）
**         ReadEmpty	：为队列读空时处理程序
**         WriteFull	：为队列写满时处理程序
** 输　出: NOT_OK	:参数错误
**         QUEUE_OK	:成功
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint8 queue_create(void      *data_buf,			/*为队列分配的缓冲区存储空间地址				*/
                  uint32    buf_len,			/*为队列分配的缓冲区存储空间大小（字节）		*/
                  ReadEmpty readempty,
                  WriteFull writefull)
{
    DataQueue *queue;
    
    if(data_buf != NULL && buf_len >= (sizeof(DataQueue)))        	/* 判断参数是否有效 		*/
    {
        queue = (DataQueue *)data_buf;			//获取数据缓冲区地址

        OS_ENTER_CRITICAL();
																	/* 初始化结构体数据 		*/
        queue->data_len = (buf_len - (uint32)(((DataQueue *)0)->data_buf)) / sizeof(uint8); /* 计算队列可以存储的数据数目 */
        queue->end = queue->data_buf + queue->data_len;             /* 计算数据缓冲的结束地址 	*/
        queue->out = queue->data_buf;								/* 队列数据读取指针 		*/
        queue->in  = queue->data_buf;								/* 队列数据写入指针 		*/
        queue->data_num  = 0;										/* 当前队列数据元素个数 	*/
        queue->readempty = readempty;								/* 回调函数					*/
        queue->writefull = writefull;

        OS_EXIT_CRITICAL();

        return QUEUE_OK;
    }
    else
    {
        return NOT_OK;
    }
}


/*********************************************************************************************************
** 函数名称: QueueRead
** 功能描述: 获取队列中的数据，读取一字节
** 输　入: ret			:存储返回的消息的地址
**         data_buf		:指向队列的指针
** 输　出: NOT_OK     	：参数错误
**         QUEUE_OK    	：收到消息
**         QUEUE_EMPTY 	：无消息
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint8 queue_read(uint8 *ret, void *data_buf)					/* 读取队列中的数据 */
{
    uint8 err;
    DataQueue *queue;

    err = NOT_OK;
    if (data_buf != NULL)                                       /* 队列是否有效 	*/
    {                                                           /* 有效 			*/
        queue = (DataQueue *)data_buf;							/* 获取队列头地址	*/			
        
        OS_ENTER_CRITICAL();
        
        if (queue->data_num > 0)                                /* 队列是否为空 	*/
        {                                                       /* 不空         	*/
            *ret = queue->out[0];                               /* 数据出队到ret   	*/
            queue->out++;                                       /* 调整出队指针 	*/
            if (queue->out >= queue->end)
            {
                queue->out = queue->data_buf;					/* 如果出队指针大于队列尾，将出队指针指向队列头 */
            }
            queue->data_num--;                                  /* 队列数据减少      */
            err = QUEUE_OK;
        }
        else
        {                                                       /* 如果队列空        */
            err = QUEUE_EMPTY;
            if (queue->readempty != NULL)                       /* 调用用户处理函数  */
            {
                err = queue->readempty(ret, queue);
            }
        }
        OS_EXIT_CRITICAL();
    }
    return err;
}

/*********************************************************************************************************
** 函数名称: QueueWrite
** 功能描述: FIFO方式发送数据,一次发送一个字节
** 输　入: data_buf 	:指向队列的指针
**         data			:消息数据
** 输　出: NOT_OK   	:参数错误
**         QUEUE_FULL	:队列满
**         QUEUE_OK  	:发送成功
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#ifndef EN_QUEUE_WRITE
#define EN_QUEUE_WRITE      0
#endif

#if EN_QUEUE_WRITE > 0

uint8 queue_write(void *data_buf, uint8 data)						/* 往队列中写数据	*/
{
    uint8 err;
    DataQueue *queue;

    err = NOT_OK;
    if (data_buf != NULL)                                           /* 队列是否有效 	*/
    {
        queue = (DataQueue *)data_buf;								/* 获取队列首地址	*/
        
        OS_ENTER_CRITICAL();
        
        if (queue->data_num < queue->data_len)                      /* 如果队列不满  	*/
        {                                                           /* 不满        		*/
            queue->in[0] = data;                                    /* 数据入队    		*/
            queue->in++;                                           	/* 调整入队指针		*/
            if (queue->in >= queue->end)							/* 如果入队指针大于出队指针 */
            {
                queue->in = queue->data_buf;						/* 将入队指针指向队列头 	*/
            }
            queue->data_num++;                                      /* 数据增加    		*/
            err = QUEUE_OK;
        }
        else
        {                                                          	/* 满           	*/
            err = QUEUE_FULL;
            if (queue->writefull != NULL)                           /* 调用用户处理函数 */
            {
                err = queue->writefull(queue, data, Q_WRITE_MODE);
            }
        }
        OS_EXIT_CRITICAL();
    }
    return err;
}
#endif

/*********************************************************************************************************
** 函数名称: QueueWriteFront
** 功能描述: LIFO方式发送数据
** 输　入: data_buf		:指向队列的指针
**         data			:消息数据
** 输　出: QUEUE_FULL	:队列满
**         QUEUE_OK		:发送成功
				|-------|高地址
				|		|	^
		   end->|-------|	|
				|		|	|
				|		|	|
			in->|-------|	|
				|		|	|
				|		|	|
				|		|	|
				|		|	|
		   out->|-------|	|
				|		|	|
				|		|   |
	  data_buf->|-------|低地址
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#ifndef EN_QUEUE_WRITE_FRONT
#define EN_QUEUE_WRITE_FRONT    0
#endif

#if EN_QUEUE_WRITE_FRONT > 0

uint8 queue_write_front(void *data_buf, uint8 data)						/* 将数据插入到队列首 	*/
{
    uint8 err;
    DataQueue *queue;

    err = NOT_OK;
    if (data_buf != NULL)                                        		/* 队列是否有效 		*/
    {
        queue = (DataQueue *)data_buf;									/* 获取队列首地址		*/
        
        OS_ENTER_CRITICAL();
        
        if (queue->data_num < queue->data_len)                          /* 如果队列不满  		*/
        {                                                               /* 不满 				*/
            queue->out--;                                               /* 调整出队指针 		*/
            if (queue->out < queue->data_buf)							/* 如果出队指针小于队列首地址 	*/
            {
                queue->out = queue->end - 1;							/* 将出队指针放到队尾之前的地址 */
            }
            queue->out[0] = data;                                       /* 数据入队     		*/
            queue->data_num++;                                          /* 数据数目增加 		*/
            err = QUEUE_OK;
        }
        else
        {                                                               /* 满           		*/
            err = QUEUE_FULL;
            if (queue->writefull != NULL)                               /* 调用用户处理函数 	*/
            {
                err = queue->writefull(queue, data, Q_WRITE_FRONT_MODE);
            }
        }
        OS_EXIT_CRITICAL();
    }
    return err;
}

#endif

/*********************************************************************************************************
** 函数名称: Queuedata_num
** 功能描述: 取得当前队列中数据个数
** 输　入: data_buf	:指向队列的指针
** 输　出: 消息数
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#ifndef EN_QUEUE_NDATA
#define EN_QUEUE_NDATA    0
#endif

#if EN_QUEUE_NDATA > 0

uint16 queue_data_num(void *data_buf)
{
    uint16 temp;
    
    temp = 0;                                                   /* 队列无效返回0 */
    if (data_buf != NULL)
    {
        OS_ENTER_CRITICAL();
        temp = ((DataQueue *)data_buf)->data_num;
        OS_EXIT_CRITICAL();
    }
    return temp;
}

#endif

/*********************************************************************************************************
** 函数名称: QueueSize
** 功能描述: 取得队列总容量
** 输　入: data_buf:指向队列的指针
** 输　出: 队列总容量
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#ifndef EN_QUEUE_SIZE
#define EN_QUEUE_SIZE    0
#endif

#if EN_QUEUE_SIZE > 0

uint16 queue_size(void *data_buf)
{
    uint16 temp;
    
    temp = 0;                                                   /* 队列无效返回0 */
    if (data_buf != NULL)
    {
        OS_ENTER_CRITICAL();
        temp = ((DataQueue *)data_buf)->data_len;
        OS_EXIT_CRITICAL();
    }
    return temp;
}

#endif

/*********************************************************************************************************
** 函数名称: OSQFlush
** 功能描述: 清空队列
** 输　入: data_buf:指向队列的指针
** 输　出: 无
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#ifndef EN_QUEUE_FLUSH
#define EN_QUEUE_FLUSH    0
#endif

#if EN_QUEUE_FLUSH > 0

void queue_flush(void *data_buf)
{
    DataQueue *queue;
    
    if (data_buf != NULL)                                           /* 队列是否有效 */
    {                                                               /* 有效         */
        queue = (DataQueue *)data_buf;
        OS_ENTER_CRITICAL();
        queue->out = queue->data_buf;
        queue->in  = queue->data_buf;
        queue->data_num = 0;                                         /* 数据数目为0 */
        OS_EXIT_CRITICAL();
    }
}

#endif
