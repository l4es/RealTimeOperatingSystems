/****************************************************************************************/
/* qtOS kernel                                                                          */
/* (C) Lluís Ribas-Xirgo, 2013. Univ. Autònoma de Barcelona                             */
/****************************************************************************************/

#ifndef _qtOS_basic_kernel_h_
#define _qtOS_basic_kernel_h_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

typedef enum qtOS_process_status_e {
  qtOS_IDLE,
  qtOS_READY,
  qtOS_RUNNING,
  qtOS_BLOCKED
} qtOS_process_status_t;

typedef struct qtOS_PCB_s {
  char *name;
  qtOS_process_status_t status;
  int priority;
  int xpri;
  int baton;
  int parent;
  int caller;
  int callee;
  int (* init)();
  int (* step)();
} qtOS_PCB_t;

#define qtOS_MAXPROC 16

typedef struct qtOS_task_list_s {
  qtOS_PCB_t task[ qtOS_MAXPROC ];
  int        length;
} qtOS_task_list_t;

typedef struct qtOS_PID_list_s {
  int PID[ qtOS_MAXPROC ];
  int length;
} qtOS_PID_list_t;

typedef enum qtOS_policy_e {
  qtOS_NON_PREEMPTIVE, /* default */
  qtOS_PREEMPTIVE
} qtOS_policy_t;

typedef struct qtOS_control_block_s {
  qtOS_task_list_t tasks;
  qtOS_PID_list_t  processes; /* indices of the previous list */
  qtOS_policy_t    policy;
  int              error;
} qtOS_control_block_t;

int qtOS_init();

int qtOS_set_policy_to();

int qtOS_new_task( char name[], int priority, int (* init)(), int (* step)() );

int qtOS_self_PID();

char *qtOS_self_name();

int qtOS_call( char name[] );

int qtOS_return();

int qtOS_fork( char name[] );

int qtOS_join();

int qtOS_run();

int qtOS_stop();

#endif
