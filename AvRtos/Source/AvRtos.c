/***************************************************************************
 * $Source$: AvRtos.c
 * $Rev$: 2.0.0
 * $Author$: Harald
 * $Date$: 2016/01/12
 *
 * Module: AvRtos
 *
 * Copyright (c) 2016, Harald Baumeister, Döggingen
 * All rights reserved.
 *
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation. Either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY. Without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 ****************************************************************************/

/*****************************************************************************
 * 1. includes
 *****************************************************************************/
#include "AvRtos.h"

/*****************************************************************************
 * 2. file local constants/definitions
 *****************************************************************************/

/*****************************************************************************
 * 3. global variables
 *****************************************************************************/
void* kernelRunningTask = 0;      /**< Pointer to currently running task */
uint8_t kernelCurrentContext = 0; /**< If current context != 0 we are running from interrupt */

/*****************************************************************************
 * 4. file local typedefs
 *****************************************************************************/
/**
 * Type definition for the task states
 */
typedef enum _TaskState
{
   TERMINATED = 0, READY = 1, WAIT = 2, SUSPENDED = 3,

} tTaskState;

/**
 * Type definition for the task control block
 */
typedef struct _Task
{
   tStack* stackPointer;   /**< Pointer to the current task stack. THIS MUST BE THE FIRST ENTRY IN THIS STRUCTURE */
   tTaskState state;       /**< Current task state */
   uint8_t priority;       /**< Task priority */
   uint16_t ticksToWait;   /**< Timeout value that will be counted down at each system tick */
   char* taskName;         /**< Pointer to the task name */
   struct _Task* next;     /**< Pointer to the next task in the linked list */

} tTask;

/**
 * Type definition for general system elements like semaphores and queues 
 */
typedef struct _Element
{
   tTask* waitingQ;        /**< Linked list of tasks that waits for this element */
   struct _Element* next;  /**< Pointer to the element in the linked list */

} tElement;

#if KERNEL_CONFIG_USE_SEMAPHORES == 1
/**
 * Type definition for a semaphore
 */
typedef struct _Semaphore
{
   uint8_t count;          /**< Semaphore count value */
   tElement element;       /**< Element to handle linked list */

} tSemaphore;
#endif

#if KERNEL_CONFIG_USE_EVENT == 1
/**
 * Type definition for a event
 */
typedef struct _Event
{
   uint32_t events;        /**< Event flag register */
   tElement element;       /**< Element to handle linked list */

} tEvent;
#endif

#if KERNEL_CONFIG_USE_MUTEXES == 1
/**
 * Type definition for a mutex
 */
typedef struct _Mutex
{
   tTaskHandle owner;      /**< Handle to the task that owns the mutex */
   tElement element;       /**< Element to handle linked list */

} tMutex;
#endif

#if KERNEL_CONFIG_USE_QUEUES == 1
/**
 * Type definition for a queue 
 */
typedef struct _Queue
{
   uint8_t* queueBuffer;   /**< Pointer to the queue buffer */
   uint8_t  msgSize;       /**< Size of one message */
   uint8_t  msgCount;      /**< Number of messages currently put to the queue */
   uint8_t  msgMax;        /**< Number of maximal messages that can be put to the queue */
   uint8_t  insertPointer; /**< Pointer to handle message insertion */
   uint8_t  removePointer; /**< Pointer to handle removing messages */
   tElement getElement;    /**< Element to handle the queue task list */
   tElement putElement;    /**< Element to handle the queue task list */

} tQueue;
#endif

/*****************************************************************************
 * 5. file local variables
 *****************************************************************************/
static uint8_t kernelRunning = 0;         /**< Flag to identify if scheduler is running */
static uint32_t kernelTicks = 0;          /**< General scheduler tick counter */
static tTask* kernelIdleTask = 0;         /**< Pointer to the kernel idle task */
static tTask* kernelTaskReadyQ = 0;       /**< Head of the linked list of tasks in the READY state */
static tTask* kernelTaskWaitQ = 0;        /**< Head of the linked list of tasks in the WAIT */
static tElement* kernelTimeoutQ = 0;      /**< Head of elements that wait for a timeout */

/* Memory allocation of system task control blocks + one for system idle task */
static uint8_t kernelNumberOfTasks = 0;
static tTask kernelTasks[KERNEL_NUMBER_OF_TASKS + 1];

#if KERNEL_CONFIG_USE_SEMAPHORES == 1
/* Memory allocation of system semaphores */
static uint8_t kernelNumberOfSemaphores = 0;
static tSemaphore kernelSemaphores[KERNEL_NUMBER_OF_SEMAPHORES];
#endif

#if KERNEL_CONFIG_USE_MUTEXES == 1
/* Memory allocation of system mutexes */
static uint8_t kernelNumberOfMutexes = 0;
static tMutex kernelMutexes[KERNEL_NUMBER_OF_MUTEXES];
#endif

#if KERNEL_CONFIG_USE_EVENT == 1
/* Memory allocation of system events */
static uint8_t kernelNumberOfEvents = 0;
static tEvent kernelEvents[KERNEL_NUMBER_OF_EVENTS];
#endif

#if KERNEL_CONFIG_USE_QUEUES == 1
/* Memory allocation of system queues */
static uint8_t kernelNumberOfQueues = 0;
static tQueue kernelQueues[KERNEL_NUMBER_OF_QUEUES];
#endif

/* Kernel idle task stack */
static tStack kernelIdelTaskStack[KERNEL_IDLE_TASK_STACK_SIZE];

/*****************************************************************************
 * 6. file local macros
 *****************************************************************************/

/*****************************************************************************
 * 7. file local function prototypes
 *****************************************************************************/
/**
 * Linked list handling function. This function will put the given task in
 * priority order to the list. The highest priority task is the first entry.
 * Task with the same priority will be linked in FIFO order.
 */
static void _kernelEnqueueTask( tTask** queue, tTask* task );

/**
 * Linked list handling function.
 * Remove the given task from the linked list.
 */
static void _kernelDequeueTask( tTask** queue, tTask* task );

/**
 * Linked list handling function.
 * Get and remove the highest priority task from the linked list.
 * This is always the first task in the list.
 */
static tTask* _kernelDequeueHighestPrioTask( tTask** queue );

/**
 * Linked list handling function for system elements like semaphores and queues.
 * This function will put the given element to the end of the list.
 */
static void _kernelEnqueueElement( tElement** queue, tElement* element );

/**
 * Linked list handling function for system elements like semaphores and queues.
 * This function will get the given element from the list.
 */
static void _kernelDequeueElement( tElement** queue, tElement* element );

/**
 * This function counts down the timeout value of every task in the linked list.
 * If the timeout was reached, the task will be put to the running queue again.
 */
static void _kernelProcessTaskTimeout( tTask** queue );

/**
 * Idle task. This task will be executed when no other task is ready to run.
 */
static void _kernelIdelTask( void );

/**
 * Main scheduling algorithms. This function will be called at each system tick
 * and performs a context switch if necessary.
 */
void _kernelScheduleTick( void );

/**
 * If a system function is called within a interrupt service routine,
 * this function checks is a context switch is needed.
 * If so, the execution of another task take place after the interrupt.
 */
void _kernelSwitchIsrContext( void );

/**
 * Function to switch to an other task context.
 * Some CPUs needs this as a naked function, so the
 * __ATTRIBUTE__ is defined in the hardware specific pat of the RTOS.
 */
void _kernelSwitchContext( void ) __ATTRIBUTE__;

/*****************************************************************************
 * 8. file local functions
 *****************************************************************************/
//----------------------------------------------------------------------------
static void _kernelEnqueueTask( tTask** queue, tTask* task )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   uint8_t i;
   tTask* prev;
   tTask* next;

   /************************************************************************
    * function code
    *************************************************************************/

   if( (queue != 0) && (task != 0) )
   {
      prev = next = *queue;
      for( i = 0; i < kernelNumberOfTasks; i++ )
      {
         /* Check if this is the first task or if we should insert at front of queue */
         if( (next == 0) || (task->priority > next->priority) )
         {
            /* Make this TCB the new listhead */
            if( next == *queue )
            {
               *queue = task;
               task->next = next;
            }
            /* Insert between two TCBs or at the tail */
            else
            {
               task->next = next;
               prev->next = task;
            }

            /* Quit the loop, we've finished inserting */
            break;
         }
         else
         {
            /* Not inserting here, try the next one */
            prev = next;
            next = next->next;
         }
      }
   }
}

//----------------------------------------------------------------------------
static void _kernelDequeueTask( tTask** queue, tTask* task )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   uint8_t i;
   tTask* prev;
   tTask* next;

   /************************************************************************
    * function code
    *************************************************************************/

   if( (queue != 0) && (task != 0) )
   {
      prev = next = *queue;
      for( i = 0; i < kernelNumberOfTasks; i++ )
      {
         /* Is this entry the one we're looking for? */
         if( next == task )
         {
            if( next == *queue )
            {
               /* We're removing the list head */
               *queue = next->next;
            }
            else
            {
               /* We're removing a mid or tail TCB */
               prev->next = next->next;
            }
            next->next = 0;
            break;
         }
         else if ( next == 0 )
         {
            break;
         }

         /* Move on to the next in the list */
         prev = next;
         next = next->next;
      }
   }
}

//----------------------------------------------------------------------------
static tTask* _kernelDequeueHighestPrioTask( tTask** queue )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask* result = 0;

   /************************************************************************
    * function code
    *************************************************************************/

   /* Remove and return the listhead */
   if( (queue != 0) && (*queue != 0) )
   {
      result = *queue;
      *queue = result->next;
      result->next = 0;
   }

   return (result);
}

//----------------------------------------------------------------------------
static void _kernelEnqueueElement( tElement** queue, tElement* element )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tElement* next;

   /************************************************************************
    * function code
    *************************************************************************/

   if( (queue != 0) && (element != 0) )
   {
      next = *queue;
      while( (next) && (next != element) )
         next = next->next;

      if( next != element )
      {
         element->next = *queue;
         *queue = element;
      }
   }
}

//----------------------------------------------------------------------------
static void _kernelDequeueElement( tElement** queue, tElement* element )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tElement* prev;
   tElement* next;

   /************************************************************************
    * function code
    *************************************************************************/

   prev = next = kernelTimeoutQ;
   while( next )
   {
      /* Is this entry the one we're looking for? */
      if( next == element )
      {
         if( next == kernelTimeoutQ )
            /* We're removing the list head */
            kernelTimeoutQ = next->next;
         else
         {
            /* We're removing a mid or tail TCB */
            prev->next = next->next;
         }
         next->next = 0;
         break;
      }

      /* Move on to the next in the list */
      prev = next;
      next = next->next;
   }
}

//----------------------------------------------------------------------------
static void _kernelProcessTaskTimeout( tTask** queue )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask* nextT;
   tTask* task;

   /************************************************************************
    * function code
    *************************************************************************/

   if( queue == 0 )
      return;

   /* Process the waitQ */
   nextT = *queue;
   while( nextT )
   {
      if( nextT->ticksToWait < KERNEL_WAIT_FOREVER )
      {
         nextT->ticksToWait--;
         if( nextT->ticksToWait == 0 )
         {
            /* Copy pointer for queue manipulation */
            task = nextT;
            nextT = nextT->next;

            /* Set new task state */
            task->state = READY;

            /* Remove this task from waitQ */
            _kernelDequeueTask( queue, task );

            /* Insert this task to readyQ */
            _kernelEnqueueTask( &kernelTaskReadyQ, task );
         }
         else
         {
            nextT = nextT->next;
         }
      }
      else
      {
         nextT = nextT->next;
      }
   }
}

//----------------------------------------------------------------------------
void _kernelSwitchIsrContext(void)
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask* task;

   /************************************************************************
    * function code
    *************************************************************************/

   /* Check if the next task in the readyQ has a higher or equal priority to the
    * currently running task. If so switch to it. */
   if( (kernelTaskReadyQ) && (kernelTaskReadyQ->priority >= ((tTask*)kernelRunningTask)->priority) )
   {
      /* Get task to switch to */
      task = _kernelDequeueHighestPrioTask( &kernelTaskReadyQ );

      /* Add the current task to the readyQ */
      _kernelEnqueueTask( &kernelTaskReadyQ, kernelRunningTask );

      /* Switch to the new task */
      kernelRunningTask = task;
   }
}

//----------------------------------------------------------------------------
void _kernelSwitchContext( void )
{
   /************************************************************************
    * local variables
    *************************************************************************/

   /************************************************************************
    * function code
    *************************************************************************/

   CONTEXT_SWITCH_SAVE_CONTEXT();

   /* Get next task to switch to */
   kernelRunningTask = _kernelDequeueHighestPrioTask( &kernelTaskReadyQ );

   /* If there is no task to run, switch to idle task */
   if( kernelRunningTask == 0 )
      kernelRunningTask = kernelIdleTask;

   CONTEXT_SWITCH_RESTORE_CONTEXT();
   CONTEXT_SWITCH_RETURN();
}

//----------------------------------------------------------------------------
static void _kernelIdelTask( void )
{
   while( 1 )
   {
      KERNEL_IDLE_HOOK_FUNCTION();
   }
}

//----------------------------------------------------------------------------
void _kernelScheduleTick( void )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tElement* nextS;

   /************************************************************************
    * function code
    *************************************************************************/

   /* Process the waitQ */
   _kernelProcessTaskTimeout( &kernelTaskWaitQ );

   /* Process the timeoutQ */
   nextS = kernelTimeoutQ;
   while( nextS )
   {
      /* Process all the tasks in this timeout waitQ */
      _kernelProcessTaskTimeout( &nextS->waitingQ );

      /* Move on to the next in the list */
      nextS = nextS->next;
   }
}

/*****************************************************************************
 * 9. exported functions
 *****************************************************************************/
//----------------------------------------------------------------------------
uint8_t kernelStartScheduler( void )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   ENTER_CRITICAL();

   /* Configure system tick interrupt */
   kernelSetupTimerInterrupt();

   /* Setup Processor dependent hardware */
   kernelSetupHardware();

   /* Initialize idle task */
   kernelIdleTask = (tTask*) taskCreate( _kernelIdelTask, "IDLE", kernelIdelTaskStack, KERNEL_IDLE_TASK_STACK_SIZE, 0 );

   kernelRunning = 1;
   kernelTicks = 0;

   /* Get next task to switch to */
   kernelRunningTask = _kernelDequeueHighestPrioTask( &kernelTaskReadyQ );

   /* Simulate a function call end as generated by the compiler.  We will now
    jump to the start of the task the context of which we have just restored. */
   kernelStartFirstTask();

   /* Should not get here. */
   return 1;
}

//----------------------------------------------------------------------------
tTaskHandle taskCreate( tKernelTaskFunction taskFunction, char* taskName, tStack* stack, uint16_t stackSize, uint8_t priority )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask  *task = 0;
   tStack *sp;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   /* Check if we can create this task */
   if( kernelNumberOfTasks >= (KERNEL_NUMBER_OF_TASKS + 1) )
      return task;

   ENTER_CRITICAL();

   /* Setup basic stack */
   sp = kernelSetupTaskStack( taskFunction, (tStack*)stack, stackSize );

   /* Get and initialize the task control block */
   task = &kernelTasks[kernelNumberOfTasks++];
   task->state = READY;
   task->priority = priority;
   task->stackPointer = sp;
   task->taskName = taskName;
   task->ticksToWait = KERNEL_WAIT_FOREVER;

   /* Insert task to readyQ */
   _kernelEnqueueTask( &kernelTaskReadyQ, task );

   EXIT_CRITICAL();

   return task;
}

//----------------------------------------------------------------------------
void taskSleep( uint16_t ticks )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask *task;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   if( ticks == 0 )
      return;

   ENTER_CRITICAL();

   /* Set new task state */
   task = kernelRunningTask;
   task->state = WAIT;
   task->ticksToWait = ticks;

   /* Remove task from readyQ */
   _kernelDequeueTask( &kernelTaskReadyQ, task );

   /* Insert the task to waitQ */
   _kernelEnqueueTask( &kernelTaskWaitQ, task );

   /* Switch the context */
   if( kernelRunning )
      RESCHEDULE();

   EXIT_CRITICAL();
}

//----------------------------------------------------------------------------
void taskSuspend( tTaskHandle taskToSuspend )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask* task;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   ENTER_CRITICAL();

   task = (tTask*) taskToSuspend;
   if( task )
   {
      /* Set new task state */
      task->state = SUSPENDED;

      /* Remove task from readyQ */
      _kernelDequeueTask( &kernelTaskReadyQ, task );
   }

   EXIT_CRITICAL();
}

//----------------------------------------------------------------------------
void taskResume( tTaskHandle taskToResume )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask* task;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   ENTER_CRITICAL();

   task = (tTask*) taskToResume;
   if( task )
   {
      /* Set new task state */
      task->state = READY;

      /* Insert this task to readyQ */
      if( task != kernelRunningTask )
         _kernelEnqueueTask( &kernelTaskReadyQ, task );
   }

   EXIT_CRITICAL();
}

#if KERNEL_CONFIG_USE_SEMAPHORES == 1
//----------------------------------------------------------------------------
tSemaphoreHandle semaphoreCreate( uint8_t initValue )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tSemaphore* sem;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   /* Check if we can create this semaphore */
   if( kernelNumberOfSemaphores >= KERNEL_NUMBER_OF_SEMAPHORES )
      return 0;

   ENTER_CRITICAL();
   sem = &kernelSemaphores[kernelNumberOfSemaphores];
   sem->count = initValue;
   sem->element.waitingQ = 0;
   sem->element.next = 0;
   kernelNumberOfSemaphores++;
   EXIT_CRITICAL();

   return (tSemaphoreHandle) sem;
}

//----------------------------------------------------------------------------
uint8_t semaphoreGet( tSemaphoreHandle semaphore, uint16_t timeoutTicks )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tSemaphore* sem;
   uint8_t error = KERNEL_NO_ERROR;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   if( semaphore == 0 )
      return KERNEL_PARAMETER_ERROR;

   ENTER_CRITICAL();

   sem = (tSemaphore*) semaphore;
   if( sem->count == 0 )
   {
      /* Check if a timeout value was set.
       * If not we will return immediately with timeout error. */
      if( timeoutTicks )
      {
         /* Insert the task to waitQ of this semaphore */
         _kernelEnqueueTask( &(sem->element.waitingQ), kernelRunningTask );

         /* Set new task state */
         ((tTask*)kernelRunningTask)->state = WAIT;
         ((tTask*)kernelRunningTask)->ticksToWait = timeoutTicks;

         /* Place this semaphore to the timeoutQ.
          * The timeout will be handled and checked at each system tick. */
         _kernelEnqueueElement( &kernelTimeoutQ, &(sem->element) );

         /* Switch the context */
         if( kernelRunning )
            RESCHEDULE();

         /* At this point we have to disable interrupts again, because _kernelSwitchContext
          * has enabled the global interrupt. */
         ENTER_CRITICAL();

         /* We reach this section only if the semaphore was set by an other task
          * or if there was an timeout. If there was an timeout the semaphore is always 0 */
         if( ((tTask*)kernelRunningTask)->ticksToWait == 0 )
         {
            error = KERNEL_TIMEOUT_ERROR;
         }
         else
         {
            /* Decrement the semaphore count */
            sem->count--;
         }
      }
      else
      {
         error = KERNEL_TIMEOUT_ERROR;
      }
   }
   else
   {
      /* Decrement the semaphore count */
      sem->count--;
   }

   EXIT_CRITICAL();

   return error;
}

//----------------------------------------------------------------------------
void semaphoreSet( tSemaphoreHandle semaphore )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask *task = 0;
   tSemaphore* sem;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   if( semaphore == 0 )
      return;

   ENTER_CRITICAL();

   sem = (tSemaphore*) semaphore;

   /* Increment the semaphore */
   sem->count++;

   /* Remove the semaphore from the timeoutQ */
   _kernelDequeueElement( &kernelTimeoutQ, &sem->element );

   /* Check if there are tasks that wait for this semaphore */
   if( sem->element.waitingQ )
   {
      /* Remove task from semaphore waitQ */
      task = _kernelDequeueHighestPrioTask( &(sem->element.waitingQ) );

      /* Set new task state */
      task->state = READY;

      /* Insert this task to readyQ */
      _kernelEnqueueTask( &kernelTaskReadyQ, task );
   }

   /* If this function was called from an interrupt service routine and
    * there is a task that waits for this queue, switch directly to it. */
   if( (kernelCurrentContext) && (task) )
   {
      _kernelSwitchIsrContext();
   }

   EXIT_CRITICAL();
}
#endif

#if KERNEL_CONFIG_USE_MUTEXES == 1
//----------------------------------------------------------------------------
tMutexHandle mutexCreate( void )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tMutex* mutex;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   /* Check if we can create this semaphore */
   if( kernelNumberOfMutexes >= KERNEL_NUMBER_OF_MUTEXES )
      return 0;

   ENTER_CRITICAL();
   mutex = &kernelMutexes[kernelNumberOfMutexes];
   mutex->owner = 0;
   mutex->element.waitingQ = 0;
   mutex->element.next = 0;
   kernelNumberOfMutexes++;
   EXIT_CRITICAL();

   return (tMutexHandle) mutex;
}

//----------------------------------------------------------------------------
uint8_t mutexGet( tMutexHandle mutex, uint16_t timeoutTicks )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tMutex* mu;
   uint8_t error = KERNEL_NO_ERROR;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   if( mutex == 0 )
      return KERNEL_PARAMETER_ERROR;

   if( kernelCurrentContext )
      return KERNEL_CONTEXT_ERROR;

   ENTER_CRITICAL();

   mu = (tMutex*) mutex;
   if(( mu->owner != kernelRunningTask ) && ( mu->owner != 0 ))
   {
      /* Check if a timeout value was set.
       * If not we will return immediately with timeout error. */
      if( timeoutTicks )
      {
         /* Insert the task to waitQ of this semaphore */
         _kernelEnqueueTask( &(mu->element.waitingQ), kernelRunningTask );

         /* Set new task state */
         ((tTask*)kernelRunningTask)->state = WAIT;
         ((tTask*)kernelRunningTask)->ticksToWait = timeoutTicks;

         /* Place this mutex to the timeoutQ.
          * The timeout will be handled and checked at each system tick. */
         _kernelEnqueueElement( &kernelTimeoutQ, &(mu->element) );

         /* Switch the context */
         if( kernelRunning )
            RESCHEDULE();

         /* At this point we have to disable interrupts again, because _kernelSwitchContext
          * has enabled the global interrupt. */
         ENTER_CRITICAL();

         /* We reach this section only if the mutex was given back by an other task
          * or if there was an timeout. */
         if( ((tTask*)kernelRunningTask)->ticksToWait == 0 )
         {
            error = KERNEL_TIMEOUT_ERROR;
         }
         else
         {
            /* Become the owner of the mutex */
            mu->owner = kernelRunningTask;
         }
      }
      else
      {
         error = KERNEL_TIMEOUT_ERROR;
      }
   }
   else
   {
      /* Become the owner of the mutex */
      mu->owner = kernelRunningTask;
   }

   EXIT_CRITICAL();

   return error;
}

//----------------------------------------------------------------------------
uint8_t mutexGive( tMutexHandle mutex )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask *task = 0;
   tMutex* mu;
   uint8_t error = KERNEL_NO_ERROR;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   if( mutex == 0 )
      return KERNEL_PARAMETER_ERROR;

   if( kernelCurrentContext )
      return KERNEL_CONTEXT_ERROR;

   ENTER_CRITICAL();

   mu = (tMutex*) mutex;

   /* We can only give back the mutex if we are the owner */
   if( mu->owner == kernelRunningTask )
   {
      /* We are the owner. Give mutex back */
      mu->owner = 0;

      /* Remove the mutex from the timeoutQ */
      _kernelDequeueElement( &kernelTimeoutQ, &mu->element );

      /* Check if there are tasks that wait for this mutex */
      if( mu->element.waitingQ )
      {
         /* Remove task from mutex waitQ */
         task = _kernelDequeueHighestPrioTask( &(mu->element.waitingQ) );

         /* Set new task state */
         task->state = READY;

         /* Insert this task to readyQ */
         _kernelEnqueueTask( &kernelTaskReadyQ, task );
      }
   }
   else
   {
      error = KERNEL_MUTEX_OWNER_ERROR;
   }

   EXIT_CRITICAL();

   return error;
}
#endif

#if KERNEL_CONFIG_USE_EVENT == 1
//----------------------------------------------------------------------------
tEventHandle eventCreate( void )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tEvent* event;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   /* Check if we can create this semaphore */
   if( kernelNumberOfEvents >= KERNEL_NUMBER_OF_EVENTS )
      return 0;

   ENTER_CRITICAL();
   event = &kernelEvents[kernelNumberOfEvents];
   event->events = 0;
   event->element.waitingQ = 0;
   event->element.next = 0;
   kernelNumberOfEvents++;
   EXIT_CRITICAL();

   return (tEventHandle) event;
}

//----------------------------------------------------------------------------
uint8_t eventRetrieve( tEventHandle event, uint32_t reqEvents, uint32_t* retEvents, tEventOption option, uint16_t timeoutTicks )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tEvent* ev;
   uint32_t compare = 0;
   uint8_t error = KERNEL_NO_ERROR;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   if( event == 0 )
      return KERNEL_PARAMETER_ERROR;

   ENTER_CRITICAL();

   ev = (tEvent*) event;

   while( 1 )
   {
      /* Isolate common event flags.  */
      compare = ev->events & reqEvents;

      /* Determine if all of the events must be present.  */
      if(( option == EVENT_OPTION_AND ) || ( option == EVENT_OPTION_AND_CONSUME ))
      {
         /* Yes, all events must be present. */
         /* See if the value is the same as the requested value.  */
         if( compare != reqEvents )
         {
            /* No, value is not the same. Clear compare value */
            compare = 0U;
         }
      }

      /* Determine if the requested combination of event flags are present.  */
      if( compare != 0U )
      {
         /* Copy the current event flags into the appropriate destination.  */
         if( retEvents != 0 )
            *retEvents = ev->events;

         /* Clear the requested event falgs. */
         if(( option == EVENT_OPTION_AND_CONSUME ) || ( option == EVENT_OPTION_OR_CONSUME ))
            ev->events = ev->events & ~reqEvents;

         break;
      }
      else
      {
         /* Check if a timeout value was set.
          * If not we will return immediately with timeout error. */
         if( timeoutTicks )
         {
            /* Insert the task to waitQ of this event */
            _kernelEnqueueTask( &(ev->element.waitingQ), kernelRunningTask );

            /* Set new task state */
            ((tTask*)kernelRunningTask)->state = WAIT;
            ((tTask*)kernelRunningTask)->ticksToWait = timeoutTicks;

            /* Place this event to the timeoutQ.
             * The timeout will be handled and checked at each system tick. */
            _kernelEnqueueElement( &kernelTimeoutQ, &(ev->element) );

            /* Switch the context */
            if( kernelRunning )
               RESCHEDULE();

            /* At this point we have to disable interrupts again, because _kernelSwitchContext
             * has enabled the global interrupt. */
            ENTER_CRITICAL();

            /* We reach this section only if the mutex was given back by an other task
             * or if there was an timeout. */
            if( ((tTask*)kernelRunningTask)->ticksToWait == 0 )
            {
               error = KERNEL_TIMEOUT_ERROR;
               break;
            }
         }
         else
         {
            error = KERNEL_TIMEOUT_ERROR;
            break;
         }
      }
   }

   EXIT_CRITICAL();

   return error;
}

//----------------------------------------------------------------------------
uint8_t eventSet( tEventHandle event, uint32_t eventFalgs, tEventOption option )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tTask *task = 0;
   tEvent* ev;
   uint8_t error = KERNEL_NO_ERROR;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   if( event == 0 )
      return KERNEL_PARAMETER_ERROR;

   ENTER_CRITICAL();

   ev = (tEvent*) event;

   /* Perform the specified operation on the current event flags in the group. */
   if(( option == EVENT_OPTION_AND ) || ( option == EVENT_OPTION_AND_CONSUME ))
   {
      /* AND the specified events with the current events.  */
      ev->events = ev->events & eventFalgs;
   }
   else
   {
      /* OR the specified events with the current events.  */
      ev->events = ev->events | eventFalgs;
   }

   /* Remove the semaphore from the timeoutQ */
   _kernelDequeueElement( &kernelTimeoutQ, &ev->element );

   /* Check if there are tasks that wait for this event */
   while( ev->element.waitingQ )
   {
      /* Remove task from semaphore waitQ */
      task = _kernelDequeueHighestPrioTask( &(ev->element.waitingQ) );

      /* Set new task state */
      task->state = READY;

      /* Insert this task to readyQ */
      _kernelEnqueueTask( &kernelTaskReadyQ, task );
   }

   /* If this function was called from an interrupt service routine and
    * there is a task that waits for this queue, switch directly to it. */
   if( (kernelCurrentContext) && (task) )
   {
      _kernelSwitchIsrContext();
   }

   EXIT_CRITICAL();

   return error;
}

#endif

#if KERNEL_CONFIG_USE_QUEUES == 1
//----------------------------------------------------------------------------
tQueueHandle queueCreate( uint8_t *buff, uint8_t size, uint8_t num_msgs )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tQueue* que;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   /* Check if we can create this semaphore */
   if( kernelNumberOfQueues >= KERNEL_NUMBER_OF_QUEUES )
      return 0;

   ENTER_CRITICAL();
   que = &kernelQueues[kernelNumberOfQueues];
   que->queueBuffer = buff;
   que->msgCount = 0;
   que->insertPointer = 0;
   que->removePointer = 0;
   que->msgSize = size;
   que->msgMax = num_msgs;
   que->putElement.waitingQ = 0;
   que->putElement.next = 0;
   que->getElement.waitingQ = 0;
   que->getElement.next = 0;
   kernelNumberOfQueues++;
   EXIT_CRITICAL();

   return (tQueueHandle) que;
}

//----------------------------------------------------------------------------
uint8_t queueGet( tQueueHandle queue, void* msgBuffer, uint16_t timeoutTicks )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tQueue* que;
   tTask *task = 0;
   uint8_t error = KERNEL_NO_ERROR;
   uint8_t returnNext = 0;
   uint8_t* source;
   uint8_t* dest;
   uint8_t i;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   if(( queue == 0 ) || ( msgBuffer == 0 ))
      return KERNEL_PARAMETER_ERROR;

   ENTER_CRITICAL();

   que = (tQueue*) queue;
   if( que->msgCount == 0 )
   {
      /* Check if a timeout value was set.
       * If not we will return immediately with timeout error. */
      if( timeoutTicks )
      {
         /* Insert the task to waitQ of this queue */
         _kernelEnqueueTask( &(que->getElement.waitingQ), kernelRunningTask );

         /* Set new task state */
         ((tTask*)kernelRunningTask)->state = WAIT;
         ((tTask*)kernelRunningTask)->ticksToWait = timeoutTicks;

         /* Place this semaphore to the timeoutQ.
          * The timeout will be handled and checked at each system tick. */
         _kernelEnqueueElement( &kernelTimeoutQ, &(que->getElement) );

         /* Switch the context */
         if( kernelRunning )
            RESCHEDULE();

         /* At this point we have to disable interrupts again, because _kernelSwitchContext
          * has enabled the global interrupt. */
         ENTER_CRITICAL();

         /* We reach this section only if the queue was set by an other task
          * or if there was an timeout. */
         if( ((tTask*)kernelRunningTask)->ticksToWait == 0 )
         {
            error = KERNEL_TIMEOUT_ERROR;
         }
         else
         {
            /* Receive the queue message and return it */
            returnNext = 1;
         }
      }
      else
      {
         error = KERNEL_TIMEOUT_ERROR;
      }
   }
   else
   {
      /* Receive the queue message and return it */
      returnNext = 1;
   }

   if( returnNext )
   {
      /* There is space in the queue, copy it in */
      source = que->queueBuffer + que->removePointer;
      dest = (uint8_t*)msgBuffer;
      for( i = 0; i < que->msgSize; i++ )
         *dest++ = *source++;

      que->removePointer += que->msgSize;
      que->msgCount--;

      /* Check if the insert index should now wrap to the beginning */
      if( que->removePointer >= (que->msgSize * que->msgMax) )
         que->removePointer = 0;

      /* Check if there are tasks that wait for this queue */
      while( que->putElement.waitingQ )
      {
         /* Remove task from queue waitQ */
         task = _kernelDequeueHighestPrioTask( &(que->putElement.waitingQ) );

         /* Set new task state */
         task->state = READY;

         /* Insert this task to readyQ */
         _kernelEnqueueTask( &kernelTaskReadyQ, task );
      }

      /* If this function was called from an interrupt service routine and
       * there is a task that waits for this queue, switch directly to it. */
      if( (kernelCurrentContext) && (task) )
      {
         _kernelSwitchIsrContext();
      }
   }

   EXIT_CRITICAL();

   return error;
}

//----------------------------------------------------------------------------
uint8_t queuePut( tQueueHandle queue, void* msgBuffer, uint16_t timeoutTicks )
{
   /************************************************************************
    * local variables
    *************************************************************************/
   tQueue* que;
   tTask *task = 0;
   uint8_t error = KERNEL_NO_ERROR;
   uint8_t putNext = 0;
   uint8_t* source;
   uint8_t* dest;
   uint8_t i;
   CRITICAL_SECTION;

   /************************************************************************
    * function code
    *************************************************************************/

   if(( queue == 0 ) || ( msgBuffer == 0 ))
      return KERNEL_PARAMETER_ERROR;

   ENTER_CRITICAL();

   que = (tQueue*) queue;
   if( que->msgCount == que->msgMax )
   {
      /* Check if a timeout value was set.
       * If not we will return immediately with timeout error. */
      if( timeoutTicks )
      {
         /* Insert the task to waitQ of this queue */
         _kernelEnqueueTask( &(que->putElement.waitingQ), kernelRunningTask );

         /* Set new task state */
         ((tTask*)kernelRunningTask)->state = WAIT;
         ((tTask*)kernelRunningTask)->ticksToWait = timeoutTicks;

         /* Place this semaphore to the timeoutQ.
          * The timeout will be handled and checked at each system tick. */
         _kernelEnqueueElement( &kernelTimeoutQ, &(que->putElement) );

         /* Switch the context */
         if( kernelRunning )
            RESCHEDULE();

         /* At this point we have to disable interrupts again, because _kernelSwitchContext
          * has enabled the global interrupt. */
         ENTER_CRITICAL();

         /* We reach this section only if the queue was set by an other task
          * or if there was an timeout. */
         if( ((tTask*)kernelRunningTask)->ticksToWait == 0 )
         {
            error = KERNEL_TIMEOUT_ERROR;
         }
         else
         {
            /* Put the queue message to the queue */
            putNext = 1;
         }
      }
      else
      {
         error = KERNEL_TIMEOUT_ERROR;
      }
   }
   else
   {
      /* Put the queue message to the queue */
      putNext = 1;
   }

   if( putNext )
   {
      /* Copy message to destination buffer */
      dest = que->queueBuffer + que->insertPointer;
      source = (uint8_t*)msgBuffer;
      for( i = 0; i < que->msgSize; i++ )
         *dest++ = *source++;

      que->insertPointer += que->msgSize;
      que->msgCount++;

      /* Check if the insert index should now wrap to the beginning */
      if( que->insertPointer >= (que->msgSize * que->msgMax) )
         que->insertPointer = 0;

      /* Check if there are tasks that wait for this queue */
      while( que->getElement.waitingQ )
      {
         /* Remove task from waitQ */
         task = _kernelDequeueHighestPrioTask( &(que->getElement.waitingQ) );

         /* Set new task state */
         task->state = READY;

         /* Insert this task to readyQ */
         _kernelEnqueueTask( &kernelTaskReadyQ, task );
      }

      /* If this function was called from an interrupt service routine and
       * there is a task that waits for this queue, switch directly to it. */
      if( (kernelCurrentContext) && (task) )
      {
         _kernelSwitchIsrContext();
      }
   }

   EXIT_CRITICAL();

   return error;
}
#endif
