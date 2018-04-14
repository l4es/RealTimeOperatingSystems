/****************************************************************************************/
/* qtOS kernel                                                                          */
/* (C) Lluís Ribas-Xirgo, 2013. Univ. Autònoma de Barcelona                             */
/****************************************************************************************/

#ifndef _qtOS_basic_kernel_c_
#define _qtOS_basic_kernel_c_

#include "qtos_basic.h"

char *qtOS_err_tbl[] = {
  /* 00 */ "No error",
  /* 01 */ "Cannot allocate more tasks!",
  /* 02 */ "Last active process deallocated!", /* Stops execution without error */
  /* 03 */ "All tasks are blocked!",
  /* 04 */ "There are no processes to schedule!",  /* Stops execution without error */
  /* 05 */ "Failed to execute task init()!",
  /* 06 */ "Undefined error",
  /* 07 */ "Unknown process name in fork()!",
  /* 08 */ "Unknown process name in call()!",
  /* 09 */ "Cannot fork an already active task!",
  /* 10 */ "Cannot call an already active task!",
  /* 11 */ "Cannot call a second task!",
  /* 12 */ "Undefined error",
  /* 13 */ "Undefined error",
  /* 14 */ "Undefined error",
  /* 15 */ "Software error, join() called from some non-running process!",
  /* 16 */ "Software error, join() called from some non-active process!",
  /* 17 */ "Software error, no process to dispatch!",
  /* 18 */ "Software error, qtOS cannot return!",
  /* 19 */ NULL
}; /* qtOS_err_tbl */

qtOS_control_block_t qtOS;

int qtOS_init( )
{
  qtOS.error = 0;
  qtOS.tasks.length = 0;
  qtOS.processes.length = 0;
  qtOS.policy = qtOS_PREEMPTIVE;
  printf( "qtOS ready.\n" );
  return 0;
} /* qtOS_init */

int qtOS_set_policy_to( qtOS_policy_t policy )
{
  qtOS.policy = policy;
  return 0;
} /* qtOS_sst_policy_to */

int qtOS_new_task( char name[], int priority, int (* init)(), int (* step)() )
{
  int failure, index, backup;

  failure = 0;
  index = qtOS.tasks.length;
  if( index < qtOS_MAXPROC ) {
    qtOS.tasks.task[index].name = (char *)strdup( name );
    qtOS.tasks.task[index].status = qtOS_IDLE;
    qtOS.tasks.task[index].parent = -1;
    qtOS.tasks.task[index].caller = -1;
    qtOS.tasks.task[index].callee = -1;
    qtOS.tasks.task[index].priority = priority;
    qtOS.tasks.task[index].xpri = priority;
    qtOS.tasks.task[index].baton = 0;
    qtOS.tasks.task[index].init = init;
    qtOS.tasks.task[index].step = step;
    backup = qtOS.processes.PID[0]; /* to make qtOS_self_*() work... */
    qtOS.processes.PID[0] = index;  /* ... during init() execution.  */
    failure = qtOS.tasks.task[index].init();
    qtOS.processes.PID[0] = backup;
    if( failure ) {
      qtOS.error = 5; /* Failed to execute task init()! */
    } else {
      qtOS.tasks.length = index + 1;
    } /* if */
  } else {
    qtOS.error = 1; /* Cannot allocate more tasks! */
    failure = -1;
  } /* if */
  return failure;
} /* qtOS_new_task */

int qtOS_self_PID()
{
  return qtOS.processes.PID[0];
} /* qtOS_self_PID */

char *qtOS_self_name()
{
  int itask = qtOS_self_PID();

  if( itask < 0 ) {
    return strdup( "<qtOS>" );
  } else {
    return qtOS.tasks.task[itask].name;
  } /* if */
} /* qtOS_self_name */

int qtOS_can_work( )
{
  int can;
  can = -1;
  if( qtOS.error != 0 || qtOS.processes.length == 0 ) { can = 0; }
  return can;
} /* qtOS_can_work */

int qtOS_lookup( char name[] )
{
  int index, taskq;

  index = 0;
  taskq = qtOS.tasks.length;
  while( index < taskq && strcmp( qtOS.tasks.task[index].name, name ) ) {
    index = index + 1;
  } /* while */
  if( index == taskq ) { index = -1; }
  return index;
} /* qtOS_lookup */

int qtOS_call( char name[] )
{
  int index, failure, curr;

  failure = 0;
  if( qtOS.processes.length > 0 ) {
    curr = qtOS.processes.PID[0];
    if( qtOS.tasks.task[curr].status == qtOS_RUNNING ) {
      qtOS.tasks.task[curr].status = qtOS_BLOCKED;
      qtOS.tasks.task[curr].baton = 0;
    } else {
      if( qtOS.tasks.task[curr].status == qtOS_BLOCKED && qtOS.tasks.task[curr].callee >= 0 ) {
        qtOS.error = 11; /* Cannot call a second task! */
        failure = -1;
      } else {
        curr = -1; /* There is no process running, must be qtOS */
      } /* if */
    } /* if */
  } else {
    curr = -1; /* There is no process running, must be qtOS */
  } /* if */
  if( failure == 0 ) {
    index = qtOS_lookup( name );
    if( index < 0 ) {
      qtOS.error = 8; /* Unknown process name in call()! */
      failure = -1;
    } /* if */
  } /* if */
  if( failure == 0 ) {
    if( qtOS.tasks.task[index].status == qtOS_IDLE ) {
      qtOS.tasks.task[index].status = qtOS_READY;
      /* Mark current process / qtOS as the caller process */
      qtOS.tasks.task[index].caller = curr;
      /* If caller has higher priority than callee, transmit priority */
      if( curr >= 0 ) { /* caller must be a process, not the qtOS */
        /* Store callee PID into caller PCB */
        qtOS.tasks.task[curr].callee = index;
        /* Execution priority inheritance */
        if( qtOS.tasks.task[index].xpri > qtOS.tasks.task[curr].xpri ) {
          qtOS.tasks.task[index].xpri = qtOS.tasks.task[curr].xpri;
        } /* if */
      } /* if */
      /* Add new process to active processes' list */
      qtOS.processes.PID[qtOS.processes.length] = index;
      qtOS.processes.length = qtOS.processes.length + 1;
      failure = 0;
    } else { /* The task is active and cannot have the same instance running twice */
      qtOS.error = 10; /* Cannot call an already active task! */
      failure = -1;
    } /* if */
  } /* if */
  return failure;
} /* qtOS_call */

int qtOS_return( )
{
  int index, top;
  int failure;

  if( qtOS.processes.length > 0 ) {
    index = qtOS.processes.PID[0];
    if( qtOS.tasks.task[index].status == qtOS_RUNNING ) {
      qtOS.tasks.task[index].status = qtOS_IDLE;
      qtOS.tasks.task[index].xpri = qtOS.tasks.task[index].priority;
      qtOS.tasks.task[index].baton = 0;
      if( qtOS.tasks.task[index].caller >= 0 ) {
        qtOS.tasks.task[qtOS.tasks.task[index].caller].status = qtOS_READY;
        qtOS.tasks.task[index].caller = -1;
      } /* if */
      index = 0;
      top = qtOS.processes.length - 1;
      while( index < top ) {
        qtOS.processes.PID[index] = qtOS.processes.PID[index + 1];
        index = index + 1;
      } /* while */
      failure = 0;
      if( index > 0 ) {
        qtOS.processes.length = top;
      } else { /* Last active process deallocated! */
        qtOS.processes.length = 0;
        qtOS.error = 0;
        failure = 0;
      } /* if */
    } else {
      qtOS.error = 18; /* Software error, qtOS cannot return */
      failure = -1;
    } /* if */
  } else {
    qtOS.error = 18; /* Software error, qtOS cannot return */
    failure = -1;
  } /* if */
  return failure;
} /* qtOS_return */


int qtOS_fork( char name[] )
{
  int index, failure, curr;

  failure = 0;
  if( qtOS.processes.length > 0 ) {
    curr = qtOS.processes.PID[0];
    if( qtOS.tasks.task[curr].status != qtOS_RUNNING ) {
      curr = -1; /* There is no process running, must be qtOS */
    } /* if */
  } else {
    curr = -1; /* There is no process running, must be qtOS */
  } /* if */
  index = qtOS_lookup( name );
  if( index < 0 ) {
    qtOS.error = 7; /* Unknown process name in fork()! */
    failure = -1;
  } else {
    if( qtOS.tasks.task[index].status == qtOS_IDLE ) {
      qtOS.tasks.task[index].status = qtOS_READY;
      /* Mark current process / qtOS as the parent process */
      qtOS.tasks.task[index].parent = curr;
      /* Add new process to active processes' list */
      qtOS.processes.PID[qtOS.processes.length] = index;
      qtOS.processes.length = qtOS.processes.length + 1;
      failure = 0;
    } else { /* The task is active and cannot have the same instance running twice */
      qtOS.error = 9; /* Cannot fork an already active task! */
      failure = -1;
    } /* if */
  } /* if */
  return failure;
} /* qtOS_fork */

int qtOS_join( )
{
  int index, top;
  int failure;

  if( qtOS.processes.length > 0 ) {
    index = qtOS.processes.PID[0];
    if( qtOS.tasks.task[index].status == qtOS_RUNNING ) {
      qtOS.tasks.task[index].status = qtOS_IDLE;
      qtOS.tasks.task[index].xpri = qtOS.tasks.task[index].priority;
      qtOS.tasks.task[index].baton = 0;
      if( qtOS.tasks.task[index].parent >= 0 ) {
        /* Should the son send some signal to the parent, must be coded here */
        qtOS.tasks.task[index].parent = -1;
      } /* if */
      index = 0;
      top = qtOS.processes.length - 1;
      while( index < top ) {
        qtOS.processes.PID[index] = qtOS.processes.PID[index + 1];
        index = index + 1;
      } /* while */
      failure = 0;
      if( index > 0 ) {
        qtOS.processes.length = top;
      } else { /* Last active process deallocated! */
        qtOS.processes.length = 0;
        qtOS.error = 0;
        failure = 0;
      } /* if */
    } else {
      qtOS.error = 15; /* Software error, join() called from some non-running process */
      failure = -1;
    } /* if */
  } else {
    qtOS.error = 16; /* Software error, join() called from some non-active process */
    failure = -1;
  } /* if */
  return failure;
} /* qtOS_join */

int qtOS_cmp( int itask, int jtask )
{
  int istat, jstat, sign;

  istat = 0;
  switch( qtOS.tasks.task[itask].status ) {
    case qtOS_BLOCKED: istat = istat + 1;
    case qtOS_READY:   istat = istat + 1;
    default:           ;
  } /* switch */
  jstat = 0;
  switch( qtOS.tasks.task[jtask].status ) {
    case qtOS_BLOCKED: jstat = jstat + 1;
    case qtOS_READY:   jstat = jstat + 1;
    default:           ;
  } /* switch */
  sign = istat - jstat;
  if( sign == 0 ) {
    sign = qtOS.tasks.task[itask].xpri - qtOS.tasks.task[jtask].xpri;
  } /* if */
  if( sign == 0 ) {
    sign = qtOS.tasks.task[itask].baton - qtOS.tasks.task[jtask].baton;
  } /* if */
  return sign;
} /* qtOS_cmp */

int qtOS_schedule( )
{
  int max, i, j, itask, jtask, found, same, failure;

  max = qtOS.processes.length;
  if( max > 0 ) {
    itask = qtOS.processes.PID[0];
    if( qtOS.tasks.task[itask].status == qtOS_RUNNING ) {
      qtOS.tasks.task[itask].baton = 1;
      if( qtOS.policy == qtOS_PREEMPTIVE ) {
        qtOS.tasks.task[itask].status = qtOS_READY;
      } /* if */
    } /* if */
  } /* if */
  /* Sort active processes */
  i = 1;
  while( i < max ) { /* linear insertion sort */
    itask = qtOS.processes.PID[i];
    j = i - 1;
    found = 0;
    while( j >= 0 && found==0 ) {
      if( qtOS_cmp( itask, qtOS.processes.PID[j] ) < 0 ) {
        qtOS.processes.PID[j+1] = qtOS.processes.PID[j];
        j = j - 1;
      } else {
        found = 1;
      } /* if */
    } /* while */
    if( found > 0 ) {
      qtOS.processes.PID[j+1] = itask;
    } else {
      qtOS.processes.PID[0] = itask;
    } /* if */
    i = i + 1;
  } /* while */
  if( max > 0 ) {
    itask = qtOS.processes.PID[0];
    if( qtOS.tasks.task[itask].status == qtOS_BLOCKED ) {
      qtOS.error = 3;  /* All tasks are blocked! */
      failure = -1;
    } else {
      qtOS.tasks.task[itask].status = qtOS_RUNNING;
      j = 1;     /* look for task with baton within same priority block ... */
      found = 0; /* ... than the first one. */
      same = 1;
      while( j < max && same > 0 && found == 0 ) {
        jtask = qtOS.processes.PID[j];
        if( qtOS.tasks.task[jtask].status == qtOS_READY &&
            qtOS.tasks.task[itask].xpri == qtOS.tasks.task[jtask].xpri
           ) {
          if( qtOS.tasks.task[jtask].baton > 0 ) {
            qtOS.tasks.task[jtask].baton = 0;
            found = 1;
          } else {
            j = j + 1;
          } /* if */
        } else {
          same = 0; /* end of block of processes with same priority */
        } /* if */
      } /* while */
      failure = 0;
    } /* if */
  } else { /* There are no processes to schedule! */
    qtOS.error = 0;
    failure = 0;
  } /* if */
  return failure;
} /* qtOS_schedule */

int qtOS_dispatch( )
{
  int index, failure;

  index = qtOS.processes.PID[0];
  if( qtOS.tasks.task[index].status == qtOS_RUNNING ) {
    failure = qtOS.tasks.task[index].step();
  } else {
    qtOS.error = 17;  /* Software error, no process to dispatch! */
     failure = -1;
  } /* if */
  return failure;
} /* qtOS_dispatch */

int qtOS_stop( )
{
  if( qtOS.error != 0 ) {
    printf( "\nqtOS stopped with error: %s\n", qtOS_err_tbl[qtOS.error] );
  } else {
    printf( "\nqtOS stopped.\n" );
  } /* if */
  return 0;
} /* qtOS_stop */

int qtOS_run( )
{
  int failure;
  printf( "qtOS started.\n" );
  while( qtOS_can_work() ) {
    failure = qtOS_schedule();
    if( failure == 0 ) failure = qtOS_dispatch();
  } /* while */
  qtOS_stop();
  return failure;
} /* qtOS_run */

#endif
