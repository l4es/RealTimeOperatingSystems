
#include <stdio.h>
#include "AvRtos.h"
#include "MK20D5.h"

//----------------------------------------------------------------------------
/* Test application stack size definitions */
#define STACK_SIZE      128

/* Test application queue message size definitions */
#define QUEUE_MESSAGE_SIZE    sizeof(tMessageQueue)

//----------------------------------------------------------------------------
/* Test application message queue type definitions */
typedef struct messageQueue
{
   uint8_t command;
   uint8_t value;

} tMessageQueue;

//----------------------------------------------------------------------------
/* Test application stack declaration */
static tStack pingSemaphoreTaskStack[STACK_SIZE];
static tStack pongSemaphoreTaskStack[STACK_SIZE];
static tStack led1TaskStack[STACK_SIZE];
static tStack led2TaskStack[STACK_SIZE];
static tStack led3TaskStack[STACK_SIZE];

/* Test application semaphores */
static tSemaphoreHandle interruptSemaphore;
static tSemaphoreHandle pingSemaphore;
static tSemaphoreHandle pongSemaphore;

/* Test application semaphores */
static tQueueHandle pingQueue;
static tQueueHandle pongQueue;

/* Test application semaphores */
static uint8_t pingQueueBuffer[QUEUE_MESSAGE_SIZE];
static uint8_t pongQueueBuffer[QUEUE_MESSAGE_SIZE];

void pingSemaphoreTask( void )
{
   while( 1 )
   {
      /* Get the ping semaphore. Wait for ever to get the semaphore. */
      semaphoreGet( pingSemaphore, KERNEL_WAIT_FOREVER );

      /* Wait for 100 ms */
      taskSleep( 100 / KERNEL_TICK_MS );

      /* Set the pong semaphore. Pong task can do his work and go on. */
      semaphoreSet( pongSemaphore );

      /* Wait for 100 ms */
      taskSleep( 100 / KERNEL_TICK_MS );
   }
}

void pongSemaphoreTask( void )
{
   while( 1 )
   {
      /* Get the pong semaphore. This semaphore will be set from ping task. */
      semaphoreGet( pongSemaphore, KERNEL_WAIT_FOREVER );

      /* Wait for 100 ms */
      taskSleep( 100 / KERNEL_TICK_MS );

      /* Set the ping semaphore. Ping task can do his work and go on. */
      semaphoreSet( pingSemaphore );

      /* Wait for 100 ms */
      taskSleep( 100 / KERNEL_TICK_MS );
   }
}

void pingQueueTask( void )
{
   /* Local queue message declaration */
   tMessageQueue message;

   /* pongQueueTask waits for a message from us.
    * Build up a message and send it to the queue. */
   message.command = 1;
   message.value = 1;
   queuePut( pongQueue, &message, KERNEL_WAIT_FOREVER );

   while( 1 )
   {
      /* Get the message from ping queue. Wait for ever to get the message. */
      queueGet( pingQueue, &message, KERNEL_WAIT_FOREVER );

      /* Increment the value count. */
      message.value++;

      /* Wait for 100 ms */
      taskSleep( 100 / KERNEL_TICK_MS );

      /* Send the message back to pong queue. */
      queuePut( pongQueue, &message, KERNEL_WAIT_FOREVER );

      /* Wait for 100 ms */
      taskSleep( 100 / KERNEL_TICK_MS );
   }
}

void pongQueueTask( void )
{
   /* Local queue message declaration */
   tMessageQueue message;

   while( 1 )
   {
      /* Get the message from pong queue. Wait for ever to get the message. */
      queueGet( pongQueue, &message, KERNEL_WAIT_FOREVER );

      /* Increment the value count. */
      message.value++;

      /* Wait for 100 ms */
      taskSleep( 100 / KERNEL_TICK_MS );

      /* Send the message back to ping queue. */
      queuePut( pingQueue, &message, KERNEL_WAIT_FOREVER );

      /* Wait for 100 ms */
      taskSleep( 100 / KERNEL_TICK_MS );
   }
}

void led1Task( void )
{
   while( 1 )
   {
      /* Wait for 100 ms */
      taskSleep( 1000 / KERNEL_TICK_MS );

      PTA->PTOR |= (1<<2);
   }
}

void led2Task( void )
{
   while( 1 )
   {
      /* Wait for 100 ms */
      taskSleep( 500 / KERNEL_TICK_MS );

      PTC->PTOR |= (1<<3);
   }
}

void led3Task( void )
{

   while( 1 )
   {
      /* Wait for 100 ms */
      taskSleep( 2000 / KERNEL_TICK_MS );

      PTD->PTOR |= (1<<4);
   }
}

int main( void )
{
   /*
    * Port C3 = RED LED
    * Port D4 = Green LED
    * Port A2 = Blue LED
    * */
   SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK);
   PORTC->PCR[3] = PORT_PCR_MUX(1);
   PTC->PSOR |= (1<<3);
   PTC->PDDR |= (1<<3);

   PORTD->PCR[4] = PORT_PCR_MUX(1);
   PTD->PSOR |= (1<<4);
   PTD->PDDR |= (1<<4);

   PORTA->PCR[2] = PORT_PCR_MUX(1);
   PTA->PSOR |= (1<<2);
   PTA->PDDR |= (1<<2);

   /* Create the needed tasks */
   kernelCreateTask( pingSemaphoreTask, "pingSemaphore", pingSemaphoreTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   kernelCreateTask( pongSemaphoreTask, "pongSemaphore", pongSemaphoreTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   kernelCreateTask( led1Task, "led1", led1TaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );
   kernelCreateTask( led2Task, "led2", led2TaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );
   kernelCreateTask( led3Task, "led3", led3TaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );

   /* Create the needed semaphores.
    * Set the pingSemaphore initialy to 1, that ping task can get it the first time. */
   interruptSemaphore = semaphoreCreate( 0 );
   pingSemaphore = semaphoreCreate( 1 );
   pongSemaphore = semaphoreCreate( 0 );

   /* Create the needed queues. */
   pingQueue = queueCreate( pingQueueBuffer, QUEUE_MESSAGE_SIZE, 1 );
   pongQueue = queueCreate( pongQueueBuffer, QUEUE_MESSAGE_SIZE, 1 );

   /* Finally start the scheduler */
   kernelStartScheduler();

   /* We should never get here. */
   return 0;
}
