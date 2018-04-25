#include <stdlib.h>
#include <avr/io.h>

#include "AvRtos.h"

//----------------------------------------------------------------------------
/* Test application stack size definitions */
#define STACK_SIZE      150

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
static uint8_t interruptTaskStack[STACK_SIZE];
static uint8_t pingSemaphoreTaskStack[STACK_SIZE];
static uint8_t pongSemaphoreTaskStack[STACK_SIZE];
static uint8_t pingQueueTaskStack[STACK_SIZE];
static uint8_t pongQueueTaskStack[STACK_SIZE];

/* Test application mutex */
static tMutexHandle mutex;

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

//----------------------------------------------------------------------------
ISR(TIMER2_COMP_vect)
{
   /*
    * Don something to handle the interrupt
    */

   /* If there are tasks that waits for this semaphore,
    * after this interrupt service routine a context switch will be
    * performed. */
   semaphoreSet( interruptSemaphore );

   /*
    * Don something to handle the interrupt
    */
}

void setupTimerInterrupt( void )
{
   /* Setup compare match value. Interrupts are disabled
    before this is called so we need not worry here. */
   OCR2 = 0xf0;

   /* Setup clock source and compare match behavior. */
   TCCR2 = 0x08 | 0x05;
}

void interruptTask( void )
{
   /* Enable the interrupt. */
   TIMSK |= (1<<OCIE2);

   while( 1 )
   {
      /* Get the ping semaphore. Wait for ever to get the semaphore. */
      semaphoreGet( interruptSemaphore, KERNEL_WAIT_FOREVER );

      /* Wait for 100 ms */
      taskSleep( 100 / KERNEL_TICK_MS );
   }
}

void pingSemaphoreTask( void )
{
   while( 1 )
   {
      /* Get owner of the mutex */
      mutexGet( mutex, KERNEL_WAIT_FOREVER );

      /* Wait for 200 ms */
      taskSleep( 200 / KERNEL_TICK_MS );

      /* Release the mutex */
      mutexGive( mutex );

      /* Wait for 200 ms */
      taskSleep( 200 / KERNEL_TICK_MS );

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

      /* Get owner of the mutex */
      mutexGet( mutex, KERNEL_WAIT_FOREVER );

      /* Wait for 200 ms */
      taskSleep( 200 / KERNEL_TICK_MS );

      /* Release the mutex */
      mutexGive( mutex );

      /* Wait for 200 ms */
      taskSleep( 200 / KERNEL_TICK_MS );

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

int main( void )
{
   /* Disable global interrupts */
   cli();

   /* Setup timer interrupt. */
   setupTimerInterrupt();

   /* Create the needed tasks */
   taskCreate( interruptTask, "interrupt", interruptTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   taskCreate( pingSemaphoreTask, "pingSemaphore", pingSemaphoreTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   taskCreate( pongSemaphoreTask, "pongSemaphore", pongSemaphoreTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   taskCreate( pingQueueTask, "pingQueue", pingQueueTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );
   taskCreate( pongQueueTask, "pongQueue", pongQueueTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );

   /* Create the needed mutex. */
   mutex = mutexCreate();

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