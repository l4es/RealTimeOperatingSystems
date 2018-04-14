#include <stdlib.h>
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
static tStack ledTaskStack[STACK_SIZE];
static tStack pingSemaphoreTaskStack[STACK_SIZE];
static tStack pongSemaphoreTaskStack[STACK_SIZE];
static tStack pingQueueTaskStack[STACK_SIZE];
static tStack pongQueueTaskStack[STACK_SIZE];

/* Test application mutex */
static tMutexHandle mutex;

/* Test application event */
static tEventHandle event;

/* Test application semaphores */
static tSemaphoreHandle pingSemaphore;
static tSemaphoreHandle pongSemaphore;

/* Test application semaphores */
static tQueueHandle pingQueue;
static tQueueHandle pongQueue;

/* Test application semaphores */
static uint8_t pingQueueBuffer[QUEUE_MESSAGE_SIZE];
static uint8_t pongQueueBuffer[QUEUE_MESSAGE_SIZE];

void ledTask( void )
{
   uint8_t led = 0;

   while( 1 )
   {
      /* Wait for 100 ms */
      taskSleep( 1000 / KERNEL_TICK_MS );

      if(led)
         AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA8;
      else
         AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA8;
      led = !led;
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
   /* Set second event flag */
   eventSet( event, 2, EVENT_OPTION_OR );

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

   /* Set first event flag */
   eventSet( event, 1, EVENT_OPTION_OR );

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
      /* Get event flags */
      eventRetrieve( event, 3, 0, EVENT_OPTION_AND_CONSUME, KERNEL_WAIT_FOREVER );

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
   /* Enable clock on PIOA */
   AT91C_BASE_PMC->PMC_PCER = ( 1 << AT91C_ID_PIOA );

   /* Configure all IO-Lines to a default state */
   /* Disable all Pull-Up restistor on the IO-Line. After reset all pull-ups are enabled. */
   AT91C_BASE_PIOA->PIO_PPUDR = 0xFFFFFFFF;
   /* Set all io-lines to input. After reset all io-lines are normaly inputs. */
   AT91C_BASE_PIOA->PIO_ODR = 0xFFFFFFFF;
   /* Give the controll of all io-lines to the PIO controller. */
   AT91C_BASE_PIOA->PIO_PER = 0xFFFFFFFF;

   /* Initialize led ports */
   AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA8;
   /* Enable output writing */
   AT91C_BASE_PIOA->PIO_OWER = AT91C_PIO_PA8;
   /* Enable output */
   AT91C_BASE_PIOA->PIO_OER =  AT91C_PIO_PA8;

   /* Create the needed tasks */
   taskCreate( ledTask, "led", ledTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   taskCreate( pingSemaphoreTask, "pingSemaphore", pingSemaphoreTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   taskCreate( pongSemaphoreTask, "pongSemaphore", pongSemaphoreTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 1 );
   taskCreate( pingQueueTask, "pingQueue", pingQueueTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );
   taskCreate( pongQueueTask, "pongQueue", pongQueueTaskStack, STACK_SIZE, KERNEL_IDLE_TASK_PRIORITY + 2 );

   /* Create the needed mutex. */
   mutex = mutexCreate();

   /* Create the needed event. */
   event = eventCreate();

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
