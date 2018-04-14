#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>

#include <string.h>

#include <usmartx.h>
#include <mailbox.h>

/*	This example project shows how to setup the timer functionality of the uSmartX kernel. Prior using 
 *	system calls which are time dependant, e.g. MBX_Pend() or TSK_Sleep(), the system timer tick must be setup. The timer
 *	tick processing is done by the uSmartX_Tick() system call. On each call of the function all currently active 
 *	timers are evaluated. Plase note that the uSmartX kernel internaly starts some timers if they are needed. For
 *	example when the TSK_Sleep() is called, the kernel starts a timer, which is associated with the task we want to
 *  sleep.
 *	Normally the uSmartX_Tick() system call is being called from a periodic interrupt routine. The example below
 *	calls the function each elapsed mili-second, thus achieving an 1ms timer tick.
 *
 *	This example also shows how to use MBX_xxx system calls.
 *
 *	The example project is setup for an ATmega64 devie running at 16 Mhz.
 *	
 */
//--------------------------------------------------------------------------------
SIGNAL(SIG_OUTPUT_COMPARE0) {
	uSMARTX_Tick();		
}
//--------------------------------------------------------------------------------
TSK_CREATE(TSK_LED);
TSK_CREATE(TSK_KESYSCAN);
TSK_CREATE(TSK_SYSTEM);

MBX_CREATE(SYS_EVENT_MBX, 5, 1);

TMR_CREATE(KEY_TIMER);
//--------------------------------------------------------------------------------
/*	This is a timer callback function and its executed 1 second after the key has been pressed.
 *	The key ID is passed thrue the ucEvent argument.
 */
STATUS KeyFxn(uint8 ucEvent, void *pArg1, void *pArg2) {
	uint8 ucSysEvent = 2;
	
	/* This function is called from the uSmartX_Tick() context. In other words this function
	 * is called from the interrupt service routine, thus we must call the MBX_Post() system call
	 * with timeout parameter 0.
	 */
	MBX_Post(&SYS_EVENT_MBX, &ucSysEvent, 0);
	return SYS_OK;
}
//--------------------------------------------------------------------------------
/*	This task takes care of the system hear beat led. It simply toggles a led and after
 * 	that it sleeps for 500ms.
 */
STATUS TSK_LedHandler(STATUS ucEvent) {
	static uint8 bBlink = 1;
	
	if( bBlink )
		PORTA |= (1 << PORTA0);
	else
		PORTA &= ~(1 << PORTA0);
		
	bBlink ^= 1;
	
	/*	We want this task to sleep, thus calling the function with the self handler. Self handler is returned
	 * 	by the TSK_Self() system call.
	 */
	TSK_Sleep(TSK_Self(), 500);
	return SYS_OK;	
}
//--------------------------------------------------------------------------------

STATUS TSK_KeyScan(STATUS ucEvent) {
	static uint8 ucKeyPressCount = 0;
	uint8 ucSysEvent;
			
	if( ucKeyPressCount == 5 ) {
		ucKeyPressCount++;
		ucSysEvent = 1;
		
		/* After the key has been pressed post a message to the system task
		 * signalling the key event. Note that we pass the pointer to the message
		 * to the system call. The pointer must be global or static type as shown in this example.
		 */
		MBX_Post(&SYS_EVENT_MBX, &ucSysEvent, 0);
		
		/* Start a key timer. The timer will elapse after 1s. The KeyFxn() callback function will be executed
		 * receiving the value 1 as the entry argument. The timer will run in one-shot mode.
		 */	
		TMR_Start(&KEY_TIMER, 1000, &KeyFxn, 1, 0, 0, TMR_ONE_SHOT);
	}
	else if( PINF & (1 << PORTF3) ) {
		ucKeyPressCount = 0;
	}
	else if( !(PINF & (1 << PORTF3)) ) {
		if( ucKeyPressCount < 5 ) {
			ucKeyPressCount++;
		}			
	}
	
	/* Key scanning period is 50 ms. */
	TSK_Sleep(TSK_Self(), 50);		
	return SYS_OK;
}
//--------------------------------------------------------------------------------
/*	The system task processes events from other tasks and timers and it also does some
 *	periodic processing.
 */
STATUS TSK_System(STATUS ucEvent) {
	uint8 ucSysEvent;
	static uint32 ulPeriodicAnchor = 0;
	
	if( MBX_Pend(&SYS_EVENT_MBX, &ucSysEvent, 100) == SYS_OK ) {
		
		switch(ucSysEvent) {
			case 1:
				/* Key has been pressed. Process the event here. */
				break;
			
			case 2:
				/* One second after key has been pressed has elapsed. Process the event here. */
				break;
				
			default:
				break;	
		}
	}
	
	/* This is an example how periodic processing with the TMR_GetTicks() system call and a timer anchor.
	 * In this particular example the watchdog is kickeg every 5s.
	 */
	if(TMR_GetTicks() - ulPeriodicAnchor > 5000 ) {
		/* Kick watchdog. */
		//KickWdog();
		
		/* Re-trigger periodic anchor. */
		ulPeriodicAnchor = TMR_GetTicks();
	}	
	return SYS_OK;
}
//--------------------------------------------------------------------------------
/*	Task initialisation table must be properly initialized. In this example
 *	all task have equal priority and will be scheduled in round robin fashion.
 */
task_entry_t uSmartXTaskTable[] = {
	{&TSK_LedHandler,	&TSK_LED, 		1, "LED"},
	{&TSK_KeyScan, 		&TSK_KESYSCAN, 	1, "KEYSCAN"},
	{&TSK_System, 		&TSK_SYSTEM, 	1, "SYSTEM"},
	{0,0,0,""}
};
//--------------------------------------------------------------------------------
int main(void) {
	
	/* Init System Timer to fire interrupts at 1ms */
	OCR0		= 125;	  		 			// set output compare match register, timer tick 1ms
	TCCR0		= (1 << WGM01) + (1 << CS02) + (1 << CS00);	//clear on compare match mode, Fclk/128 
	TIMSK		|= (1 << OCIE0); // enable output compare 2 interrupt
	TCNT0		= 0x00;	 		 // clear timer2 register
			
	MBX_Init(&SYS_EVENT_MBX);
		
	uSMARTX_Init(uSmartXTaskTable);
	
	/* Enable interrupts. */
	INT_Enable();
	
	while(1) {	
		if(uSMARTX_Scheduler() == SYS_ERROR)
			break;
	}

	/* Handle error here. */

	return 0;
}