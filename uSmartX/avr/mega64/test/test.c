
#include <usmartx.h>
#include <mailbox.h>
#include <mballoc.h>

TSK_CREATE(TSK1_tcb);
TSK_CREATE(TSK3_tcb);
TSK_CREATE(TSK2_tcb);

TMR_CREATE(TMR1);


MEM_HEAP_CREATE(sys_mem, 10, MEM_MB_16_BYTE);

STATUS TSK1(STATUS evt) {
	static a = 0; 
	uint8 i, buff[10];
	
	for(i = 0; i < 10; i++) {
		buff[i] = a++;
	}
			
	return SYS_OK;
}

STATUS TSK2(STATUS evt) {
	static uint8 i[] = {11,12,13,14,15,16,17,18,19,20};		
	return SYS_OK;
}

STATUS TSK3(STATUS evt) {
	uint8 buff[10];
	TMR_Stop(&TMR1);
	return SYS_OK;
	
}

task_entry_t tsk_tbl[] = {	{&TSK1, &TSK1_tcb, 1},
								{&TSK2, &TSK2_tcb, 2},
								{&TSK3, &TSK3_tcb, 3},
								{0, 0}
							};
													

int main(void) {
	void *p1, *p2, *p3;
			
	uSMARTX_Init(tsk_tbl);
	
	p1 = MEM_Alloc(&sys_mem, 15);
	p2 = MEM_Alloc(&sys_mem, 64);
	p3 = MEM_Alloc(&sys_mem, 100);
	MEM_Free(&sys_mem, p1);
	p3 = MEM_Alloc(&sys_mem, 100);
	MEM_Free(&sys_mem, p2);
	p3 = MEM_Alloc(&sys_mem, 100);
	while(1) {
		uSMARTX_Tick();
		if(uSMARTX_Scheduler() == SYS_ERROR)
			break;
	}
	
	while(1);
}