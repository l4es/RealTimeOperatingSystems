#include "board.h"
#include "AvRtos.h"
#include <stdio.h>

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
static tStack led1TaskStack[STACK_SIZE];
static tStack led2TaskStack[STACK_SIZE];
static tStack pingSemaphoreTaskStack[STACK_SIZE];
static tStack pongSemaphoreTaskStack[STACK_SIZE];
static tStack pingQueueTaskStack[STACK_SIZE];
static tStack pongQueueTaskStack[STACK_SIZE];

/* Test application mutex */
static tMutexHandle mutex;

/* Test application semaphores */
static tSemaphoreHandle pingSemaphore;
static tSemaphoreHandle pongSemaphore;

/* Test application semaphores */
static tQueueHandle pingQueue;
static tQueueHandle pongQueue;

/* Test application semaphores */
static uint8_t pingQueueBuffer[QUEUE_MESSAGE_SIZE];
static uint8_t pongQueueBuffer[QUEUE_MESSAGE_SIZE];

//----------------------------------------------------------------------------
void led1Task( void )
{

   uint8_t led;

   while( 1 )
   {
      /* Wait for 100 ms */
      taskSleep( 1000 / KERNEL_TICK_MS );

      led = !led;
      Board_LED_Set( 0, led );
   }
}

void led2Task( void )
{

   uint8_t led;

   while( 1 )
   {
      /* Wait for 100 ms */
      taskSleep( 500 / KERNEL_TICK_MS );

      led = !led;
      Board_LED_Set( 1, led );
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

   SystemCoreClockUpdate();
   Board_Init();
   Board_LED_Set( 0, 0 );
   Board_LED_Set( 1, 1 );

   /* The sysTick counter only has 24 bits of precision, so it will
    overflow quickly with a fast core clock. You can alter the
    sysTick divider to generate slower sysTick clock rates. */
   Chip_Clock_SetSysTickClockDiv( 1 );

   /* Create the needed tasks */
   taskCreate( led1Task, "led1", led1TaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );
   taskCreate( led2Task, "led2", led2TaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );
   taskCreate( pingSemaphoreTask, "pingSemaphore", pingSemaphoreTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   taskCreate( pongSemaphoreTask, "pongSemaphore", pongSemaphoreTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   taskCreate( pingQueueTask, "pingQueue", pingQueueTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );
   taskCreate( pongQueueTask, "pongQueue", pongQueueTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );

   /* Create the needed mutex. */
   mutex = mutexCreate();

   /* Create the needed semaphores.
    * Set the pingSemaphore initialy to 1, that ping task can get it the first time. */
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
