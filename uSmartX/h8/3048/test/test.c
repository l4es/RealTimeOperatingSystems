#include <usmartx.h>
#include <mailbox.h>
#include <mballoc.h>

TSK_CREATE(TSK1_tcb);
TSK_CREATE(TSK3_tcb);
TSK_CREATE(TSK2_tcb);

TMR_CREATE(DEMO_TMR);
MBX_CREATE(EVENT_MSG, 5, sizeof(uint8));

STATUS TSK1(STATUS evt) {
	/* DEMO */
	TSK_Sleep( TSK_Self(), 100);
	return SYS_OK;
}

STATUS TSK2(STATUS evt) {
	static uint8 ucTest;
	/* DEMO */
	if( MBX_Pend(&EVENT_MSG, &ucTest, 500) == SYS_OK ) {
	
		switch( ucTest ) {
			case 100:
				/* Handle event 100 here */
				break;
			default:
				break;	
		}	
		
	}
	return SYS_OK;
}

STATUS TSK3(STATUS evt) {
	TSK_Sleep( TSK_Self(), 30);
	return SYS_OK;
	
}

task_entry_t tsk_tbl[] = {	{&TSK1, &TSK1_tcb, 1, "TASK1"},
							{&TSK2, &TSK2_tcb, 2, "TASK2"},
							{&TSK3, &TSK3_tcb, 3, "TASK3"},
							{0, 0}
							};
													

STATUS TimeoutHandler(uint8 Event, void *pArg1, void *pArg2) {	
	MBX_Post(pArg1, &Event, 0);
	return SYS_OK;
}

int main(void) {
	
	MBX_Init(&EVENT_MSG);
	TMR_Start(&DEMO_TMR, 5000, &TimeoutHandler, 100, &EVENT_MSG, 0, TMR_PERIODIC);
		
	uSMARTX_Init(tsk_tbl);	
	while(1) {
		uSMARTX_Tick();
		if(uSMARTX_Scheduler() == SYS_ERROR)
			break;
	}
	
	while(1);
}
