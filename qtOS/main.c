/****************************************************************************************/
/* Examples of qtOS_basic:                                                              */
/* - Seat belt alarm controller                                                         */
/* - Take-a-number system                                                               */
/* - Simple shell                                                                       */
/* (C) Lluís Ribas-Xirgo, 2013. Univ. Autònoma de Barcelona                             */
/****************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <conio.h>
#include <time.h>
#include "qtos_basic.h"

/* Seat belt alarm controller ***********************************************************/

/* Shared memory: */

int BeltExit; /* This is a counter that is non-zero to mean STOP:
                 When all tasks are active it remains to zero until
                 a stopping signal has been activated.
                 Then it is set to a value that is equal
                 to the number of involved processes (4, in this case).
                 When any of these processes runs its step,
                 if the counter is greater than zero,
                 it decrements it by 1 and becomes inactive by either
                 qtOS_join() or qtOS_return().
               */
int BeltOn, KeyOn, KeyOff, StartTimer, End5, End10, Alarm;

/* Task functions: */

enum { BELT_IDLE, BELT_RUNNING } belt_state;

int belt_init()
{
  belt_state = BELT_IDLE;
  BeltExit = 0;
  return 0;
} /* belt_init */

int belt_step()
{
  int failure;

  if( belt_state == BELT_IDLE ) {
    failure = qtOS_fork( "alarm" );
    if( failure == 0 ) failure = qtOS_fork( "timer" );
    if( failure == 0 ) failure = qtOS_fork( "carmon" );
    belt_state = BELT_RUNNING;
  } else {
    if( BeltExit > 0 ) {
      BeltExit = BeltExit - 1;
      belt_state = BELT_IDLE;
      qtOS_return();
    } /* if */
  } /* if */
  return failure;
} /* belt_step */

enum { ALARM_INIT, ALARM_START, ALARM_WAIT, ALARM_BEEP } alarm_state;

int alarm_init()
{
  StartTimer = 0;
  Alarm = 0;
  alarm_state = ALARM_INIT;
  return 0;
} /* alarm_init */

int alarm_step()
{
  int failure;

  failure = 0;
  switch( alarm_state ) {
    case ALARM_INIT: {
      if( KeyOn && !BeltOn ) {
        alarm_state = ALARM_START;
        StartTimer = 1;
        alarm_state = ALARM_WAIT;
      } /* if */
      Alarm = 0;
      break;
    } /* case */
    case ALARM_WAIT: {
      if( KeyOff || BeltOn || End10 ) {
        /* Former condition has been added "|| End10" to avoid
           that concurrent reception of End5 and En10 causes
           alarm to beep forever.
        */
        alarm_state = ALARM_INIT;
      } else {
        if( End5 ) {
          alarm_state = ALARM_BEEP; printf( "alarm: beeping\n" );
        } /* if */
      } /* if */
      Alarm = 0;
      break;
    } /* case */
    case ALARM_BEEP: {
      if( KeyOff || BeltOn || End10 ) {
        alarm_state = ALARM_INIT;  printf( "alarm: off\n" );
      } /* if */
      Alarm = 1;
      break;
    } /* case */
    default: {
      failure = -1;
    } /* default */
  } /* switch */
  /* Input events' consumption: */
  KeyOn  = 0;
  KeyOff = 0;
  End5   = 0;
  End10  = 0;
  if( BeltExit > 0 ) {
    BeltExit = BeltExit - 1;
    alarm_state = ALARM_INIT;
    failure = qtOS_join();
  } /* if */
  return failure;
} /* alarm_step */

enum { TIMER_OFF, TIMER_TO_5, TIMER_TO_10 } timer_state;
clock_t time0;

int timer_init()
{
  StartTimer = 0;
  End5 = 0;
  End10 = 0;
  time0 = clock();
  alarm_state = TIMER_OFF;
  return 0;
} /* timer_init */

int timer_step()
{
  double elapsed;

  if( StartTimer == 1 ) {
    timer_state = TIMER_TO_5;
    time0 = clock();
  } /* if */
  if( timer_state == TIMER_TO_5 ) {
    elapsed = (double)(clock()-time0)/CLOCKS_PER_SEC;
    if( elapsed >= 5.0 ) {
      End5 = 1; printf("timer: emit End5\n");
      timer_state = TIMER_TO_10;
    } /* if */
  } /* if */
  if( timer_state == TIMER_TO_10 ) {
    elapsed = (double)(clock()-time0)/CLOCKS_PER_SEC;
    if( elapsed >= 10.0 ) {
      End10 = 1; printf("timer: emit End10\n");
      timer_state = TIMER_OFF;
    } /* if */
  } /* if */
  /* Input events' consumption: */
  StartTimer = 0;
  if( BeltExit > 0 ) {
    BeltExit = BeltExit - 1;
    timer_state = TIMER_OFF;
    return qtOS_join();
  } else {
    return 0;
  } /* if */
} /* timer_step */

int carmon_init()
{
  BeltExit = 0;
  BeltOn = 0;
  KeyOn = 0;
  KeyOff = 0;
  return 0;
} /* carmon_init */

int carmon_step()
{
  int  failure;
  char c;

  failure = 0;
  fflush( stdin );
  printf( "carmon: Type in ...\n" );
  printf( "        [B] to toggle seat belt,\n" );
  printf( "        [I] to start engine (KeyOn),\n" );
  printf( "        [S] to stop engine (KeyOff), and\n" );
  printf( "        [Q] to quit simulation.\n" );
  printf( "        or else to continue.\n" );
  c = getch();
  printf( "carmon: " );
  switch( c ) {
    case 'B':
    case 'b':
      if( BeltOn ) {
        BeltOn = 0;
        printf( "BeltOn'" );
      } else {
        BeltOn = 1;
        printf( "BeltOn" );
      } /* if */
      break;
    case 'I':
    case 'i': KeyOn = 1; KeyOff = 0; printf( "KeyOn" ); break;
    case 'S':
    case 's': KeyOff = 1; KeyOn = 0; printf( "KeyOff" );break;
    case 'Q':
    case 'q': BeltExit = 4; printf( "Exit!" ); break;
    default:  printf( "<nothing>" );
  } /* switch */
  putchar( '\n' );
  if( BeltExit > 0 ) {
    BeltExit = BeltExit - 1;
    failure = qtOS_join();
  } /* if */
  return failure;
} /* carmon_step */

/* **************************************************************************************/

/* Take-a-number system *****************************************************************/
/* The idea is that there is a task for every possible element in the queue,
   and that this task is a representative (i.e. an agent) of the customer in the system.
   Agents share the same behavior: take a number and wait their turn to be served.
   There is an observer that detects new clients entering and the first ones exiting
   the system.
*/

#define QUEUE_CAPACITY 8

/* Shared memory: */

char *ClientNames[QUEUE_CAPACITY] = { "01", "02", "03", "04", "05", "06", "07", "08" };
int NowServing, NextInQueue, Quantity;

/* Task functions: */

enum { TURNS_IDLE, TURNS_RUNNING } turns_state;

int turns_init()
{
  turns_state = TURNS_IDLE;
  return 0;
} /* turns_init */

int turns_step()
{
  int failure;

  failure = 0;
  if( turns_state == TURNS_IDLE ) {
    failure = qtOS_call( "observer" );
    turns_state = TURNS_RUNNING;
  } else {
    turns_state = TURNS_IDLE;
    qtOS_return();
  } /* if */
  return failure;
} /* turns_step */

enum { CLIENT_GET_TICKET, CLIENT_GET_OUT } client_state[QUEUE_CAPACITY];

int client_search( char *name )
{
  int i, found;

  i = 0;
  found = 0;
  while( i < QUEUE_CAPACITY && found == 0 ) {
    if( strcmp( name, ClientNames[i] ) ) {
      i = i + 1;
    } else {
      found = 1;
    } /* if */
  } /* while */
  if( found == 0 ) i = -1;
  return i;
} /* client_search */

int client_init()
{
  int i;

  i = client_search( qtOS_self_name() );
  if( i >= 0 ) {
    client_state[i] = CLIENT_GET_TICKET;
    i = 0;
  } /* if */
  return i;
} /* client_init */

int client_step()
{
  int  failure;
  int i;

  failure = 0;
  i = client_search( qtOS_self_name() );
  if( i < 0 ) return -1;
  switch( client_state[i] ) {
    case CLIENT_GET_TICKET: {
      printf( "client: ticket no. %s\n", qtOS_self_name() );
      client_state[i] = CLIENT_GET_OUT;
      failure = qtOS_return();
      break;
    }
    case CLIENT_GET_OUT: {
      printf( "client: ticket no. %s, dropped\n", qtOS_self_name() );
      client_state[i] = CLIENT_GET_TICKET;
      failure = qtOS_fork( "observer" );
      failure = qtOS_join();
      break;
    }
  } /* switch */
  return failure;
} /* client_step */

enum { O_READING, O_GIVE_TICKET, O_PUSH, O_TAKE_TICKET, O_POP, O_STOP } observer_state;

int observer_init()
{
  NowServing = 0;
  NextInQueue = 0;
  Quantity = 0;
  observer_state = O_READING;
  return 0;
} /* observer_init */

int observer_step()
{
  int  failure;
  char c;

  failure = 0;
  switch( observer_state ) {
    case O_READING: {
      fflush( stdin );
      printf( "observer: Type in ...\n" );
      printf( "          [T] to [t]ake a number,\n" );
      printf( "          [G] to [g]o away, and\n" );
      printf( "          [Q] to quit simulation.\n" );
      printf( "          else to go on.\n" );
      c = getch();
      switch( c ) {
        case 'T':
        case 't': {
          observer_state = O_GIVE_TICKET;
          break;
        } /* client takes a number */
        case 'G':
        case 'g': {
          observer_state = O_TAKE_TICKET;
          break;
        } /* client gives a number back */
        case 'Q':
        case 'q': {
          observer_state = O_STOP;
        } /* quit simulation */
      } /* switch */
      break;
    }
    case O_GIVE_TICKET: {
      if( Quantity < QUEUE_CAPACITY ) {
        failure = qtOS_call( ClientNames[NextInQueue] );
        if( failure ) observer_state = O_STOP; else observer_state = O_PUSH;
      } else {
        printf( "observer: Queue capacity (%d) exceeded!\n", QUEUE_CAPACITY );
        observer_state = O_READING;
      } /* if */
      break;
    }
    case O_PUSH: {
      failure = qtOS_fork( ClientNames[NextInQueue] );
      if( failure ) {
        printf( "observer: Error processing %s entrance!\n", ClientNames[NextInQueue] );
        observer_state = O_STOP;
      } else {
        NextInQueue = (NextInQueue + 1) % QUEUE_CAPACITY;
        Quantity = Quantity + 1;
        observer_state = O_READING;
      } /* if */
      break;
    }
    case O_TAKE_TICKET: {
      if( Quantity > 0 ) {
        failure = qtOS_join();
        if( failure ) {
          printf( "observer: Error processing %s exit!\n", ClientNames[NowServing] );
          observer_state = O_STOP;
        } else {
          observer_state = O_POP;
        } /* if */
      } else {
        printf( "observer: Queue underflow, there are no clients waiting!\n" );
        observer_state = O_READING;
      } /* if */
      break;
    }
    case O_POP: {
      NowServing = (NowServing + 1) % QUEUE_CAPACITY;
      Quantity = Quantity - 1;
      observer_state = O_READING;
      break;
    }
    case O_STOP: {
      if( Quantity > 0 ) {
        printf( "observer: Cannot stop while there are clients!\n" );
      } else {
        NowServing = 0;
        NextInQueue = 0;
        failure = qtOS_return();
      } /* if */
      observer_state = O_READING;
    }
  } /* switch */
  return failure;
} /* observer_step */

/* **************************************************************************************/

/* qtOS Shell ***************************************************************************/

typedef struct qtOS_shell_s {
  char input[BUFSIZ];
} qtOS_shell_t;

qtOS_shell_t qtOS_shell;

int qtOS_shell_init()
{
  qtOS_shell.input[0] = '\0';
  printf( "qtOS> Ready.\n" );
  return 0;
} /* qtOS_shell_init */

int qtOS_shell_step( )
{
  int failure;
  int i;

  failure = 0;
  printf( "qtOS> " );
  scanf( "%s", qtOS_shell.input );
  i = 0;
  while( i<BUFSIZ && isspace( qtOS_shell.input[i] ) ) {
    i = i + 1;
  } /* while */
  switch( qtOS_shell.input[i] ) {
    case 'A':
    case 'a': qtOS_call( "belt" ); break;
    case 'T':
    case 't': qtOS_call( "turns" ); break;
    case 'H':
    case 'h': printf( "qtOS: Commands are [A]larm, [T]urns, [H]elp, and [Q]uit\n" ); break;
    case 'Q':
    case 'q': printf( "qtOS: Bye.\n" ); failure = qtOS_join(); break;
    default : printf( "qtOS: Unknown command.\n" );
  } /* switch */
  return failure;
} /* qtOS_shell_step */

/* **************************************************************************************/

int main()
{
    int failure = 0;
    int i;

    printf( "qtOS v0.1 running tasks step by step\n\n" );
    qtOS_init();
    qtOS_new_task( "shell",    4, qtOS_shell_init, qtOS_shell_step );
    /* Seat belt alarm controller */
    qtOS_new_task( "belt",     3, belt_init, belt_step );
    qtOS_new_task( "alarm",    2, alarm_init, alarm_step );
    qtOS_new_task( "timer",    2, timer_init, timer_step );
    qtOS_new_task( "carmon",   2, carmon_init, carmon_step );
    /* Take-a-number example */
    qtOS_new_task( "turns",    3, turns_init, turns_step );
    qtOS_new_task( "observer", 1, observer_init, observer_step );
    for( i = 0; i < QUEUE_CAPACITY; i = i + 1 ) {
      qtOS_new_task( ClientNames[i], 2, client_init, client_step );
    } /* for */
    qtOS_fork( "shell" );
    failure = qtOS_run();
    return failure;
} /* main */
