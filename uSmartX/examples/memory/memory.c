#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>

#include <string.h>

#include <usmartx.h>
#include <cycbuff.h>

/*	This example shows the usage of the uSmartX fifo buffers. The application is the classic
 *	producer/consumer example. The producer task puts data into the buffer. When the buffer
 *	reaches the trigger level the trigger callback function is executed telling the consumer task
 *	to start fetching data from the buffer. The consumer task retrives one byte of data and waits
 *	the timeout trigger function to trigger, telling the consumer task to retrieve the next byte.
 *
 *	The example project is setup for an ATmega64 devie running at 16 Mhz.
 *	
 */
//--------------------------------------------------------------------------------
SIGNAL(SIG_OUTPUT_COMPARE0) {
	uSMARTX_Tick();		
}
//--------------------------------------------------------------------------------
TSK_CREATE(TSK_PRODUCER);
TSK_CREATE(TSK_CONSUMER);

BUF_CREATE(BUF, 10 /* buffer size */, 2/*data length */); 

//--------------------------------------------------------------------------------
/*	These are the two buffer associated trigger functions. The first one is called when the
 *	buffer fills up to a certain level, the second one fires after the buffer contains data
 *	that is older than its timeout.
 */
STATUS TriggerFxn(uint8 ucEvent, void *pArg1, void *pArg2) {
	TSK_Resume(&TSK_CONSUMER);
	return SYS_OK;
}

STATUS TimeoutFxn(uint8 ucEvent, void *pArg1, void *pArg2) {	
	TSK_Resume(&TSK_CONSUMER);
	return SYS_OK;
}
//--------------------------------------------------------------------------------
/*	The producer task puts data into the buffer if there is enought free space in it, otherwise
 *	the task block itself and wait the consumer task to unblock it.
 */
STATUS TSK_Producer(STATUS ucEvent) {
	static uint16 unData = 0;
	
	unData++;
	while( BUF_Put(&BUF, &unData) == SYS_OK )
		asm(" nop");
	
	TSK_Sleep(TSK_Self(), 100);
	return SYS_OK;	
}
//--------------------------------------------------------------------------------
/* The consumer task gets one element from the buffer and returns.
 */
STATUS TSK_Consumer(STATUS ucEvent) {
	uint16 unData;
	
	if( BUF_Get(&BUF, &unData) == SYS_OK ) {
		TSK_Suspend(&TSK_CONSUMER);
	}
	return SYS_OK;
}
//--------------------------------------------------------------------------------
/*	Task initialisation table must be properly initialized. In this example
 *	all task have equal priority and will be scheduled in round robin fashion.
 */
task_entry_t uSmartXTaskTable[] = {
	{&TSK_Producer,		&TSK_PRODUCER, 	1, "PRODUCER"},
	{&TSK_Consumer,		&TSK_CONSUMER, 	1, "CONSUMER"},
	{0,0,0,""}
};
//--------------------------------------------------------------------------------
int main(void) {	
	
	/* Init System Timer to fire interrupts at 1ms - timer tick */
	OCR0		= 125;	  		 			// set output compare match register, timer tick 1ms
	TCCR0		= (1 << WGM01) + (1 << CS02) + (1 << CS00);	//clear on compare match mode, Fclk/128 
	TIMSK		|= (1 << OCIE0); // enable output compare 2 interrupt
	TCNT0		= 0x00;	 		 // clear timer2 register
						
	uSMARTX_Init(uSmartXTaskTable);
	
	/*	Append timeout trigger function. The trigger function will be executed after
	 *	there is no write/read access if the buffer contains some unread data.
	 */
	BUF_AppendToutFxn(&BUF, 10 /*10 ms*/, &TimeoutFxn);
	
	/*	Append level trigger function. The trigger function will be executed after
	 *	the buffer fills to a level of 4. The function will be re-triggered after the
	 *	buffer is completely emptied out.
	 */
	BUF_AppendTrgFxn(&BUF, 4, &TriggerFxn);
	
	/* Enable interrupts. */
	INT_Enable();
	
	while(1) {	
		if(uSMARTX_Scheduler() == SYS_ERROR)
			break;
	}

	/* Handle error here. */

	return 0;
}