#include "config.h"

// 定义变量
CmdQueue		cmd_queue;
CmdRcvBuf		cmd_rcvbuf;

void cmd_rcvbuf_init(void)								//命令接收缓冲区初始化
{
	memset(&cmd_rcvbuf, 0x00, sizeof(CmdRcvBuf));
}

void cmd_rcvbuf_add_char(uint8 c)				//往命令接收缓冲区添加一个字符
{
	if(CMD_RCV_BUF_STATUS_EMPTY == cmd_rcvbuf.status)	//如果命令接收缓冲区是空的
	{
		if(c == FRAME_BEGIN_FLAG)						//如果接收到的字符为帧起始标记0XAA
		{
			cmd_rcvbuf.status = CMD_RCV_BUF_STATUS_RCVING;//那么就标记命令缓冲区当前状态为正在接收状态
			cmd_rcvbuf.ptr 	  = 0;						//将其指针设置为0
		}

		return;											//返回，此时再接收到数据就直接执行余下部分，真正保存接收命令
	}

	((uint8 *)&cmd_rcvbuf.cmd)[cmd_rcvbuf.ptr++] = c;//将接收到的字符放到接收缓冲区中
	
	if((cmd_rcvbuf.ptr >= 2) && (cmd_rcvbuf.ptr == cmd_rcvbuf.cmd.para_size + 2))//前两个字符为命令与数据标记和参数长度，故要大于2
	{
		cmd_rcvbuf.status = CMD_RCV_BUF_STATUS_READY;	//每帧命令的长度也是确定的，这里作一个判断
	}

	if(cmd_rcvbuf.status == CMD_RCV_BUF_STATUS_READY)	//如果一帧命令已经接收完成，即命令接收缓冲区已就绪
	{
		cmd_queue_add_cmd(&cmd_rcvbuf.cmd);				//则将当前命令添加到命令接收队列缓冲区中
		cmd_rcvbuf.status = CMD_RCV_BUF_STATUS_EMPTY;	//并置相应的命令缓冲区状态为空
		cmd_rcvbuf.ptr = 0;								//读取指针为0
	}
}

void cmd_queue_init(void)								//命令缓冲队列初始化
{
	memset(&cmd_queue, 0x00, sizeof(CmdQueue));
}

void cmd_queue_add_cmd(Cmd* pCmd)						//往命令缓冲队列添加一帧命令
{
	if(cmd_queue_status() == CMD_QUEUE_STATUS_FULL)		//如果命令缓冲队列已满则直接返回
		return;

	memcpy(&(cmd_queue.cmd[cmd_queue.tail]), pCmd, sizeof(Cmd));//把pCmd指向的命令复制到缓冲队列的尾部
	CMDQUEUE_TAIL_ADD();								//调整尾部指针
}

void cmd_queue_remove_cmd(void)							//从命令缓冲队列（头）丢掉一帧命令
{
	if(cmd_queue_status() == CMD_QUEUE_STATUS_EMPTY)
		return;

	CMDQUEUE_HEAD_ADD();								//一切数据均是从尾部添加，从头部删除（从头部读取）
}

Cmd* cmd_queue_headprt_get(void)						//获取命令缓冲队列的头指针
{
	if(cmd_queue_status() == CMD_QUEUE_STATUS_EMPTY)
		return NULL;

	return (&cmd_queue.cmd[cmd_queue.head]);
}

bool cmd_queue_status(void)								//返回当前命令缓冲队列的状态，空，满或者有数据
{
	if(cmd_queue.tail == cmd_queue.head)
		return CMD_QUEUE_STATUS_EMPTY;
	
	if(cmd_queue.head == (cmd_queue.tail + 1) % CMD_MAX_NUM)
		return CMD_QUEUE_STATUS_FULL;
	
	else
		return CMD_QUEUE_STATUS_HASDATA;
}
