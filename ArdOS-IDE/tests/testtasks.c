/*
 * testtasks.c
 *
 * Created: 14/4/2013 9:36:35 AM
 *  Author: dcstanc
 *
 * Code to test task handling
 *
 */ 

#include <stdlib.h>
#if OS_DEBUG == 1
#include <stdio.h>
#endif

#include "task.h"

tTCB tasklist[OSMAX_TASKS];
tQueue q;
unsigned char qbuf[OSMAX_TASKS];

void initTaskList(tTCB *list)
{
	int i;
	for(i=0; i<OSMAX_TASKS; i++)
	{
		list[i].pid=i;
		list[i].prio=rand() % 255;
	}
}

int main()
{
	int i;
	unsigned char op, p, k, ch;
	initTaskList(tasklist);
	
	srand((unsigned int) time(NULL));
	initQ(qbuf, OSMAX_TASKS, &q);
	
	for(i=0; i<100; i++)
	{
		// Choose random operation
		op=rand() % 2;
		
		if(op)	
		{
			k=procDeq(&q);
			if(k!=255)
				printf("Dequed (%d, %d): ", k, tasklist[k].prio);
			else
				printf("Dequed (%d, -): ", k);
		}
		else
		{
			p=rand() % OSMAX_TASKS;
			procEnq(p, tasklist, &q);
			printf("Enqued (%d %d): ", p, tasklist[p].prio);
		}
		printProcQ(&q, tasklist);
		printf("\n");
	}
	ch=getchar();
}
