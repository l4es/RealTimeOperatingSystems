/*
 * mutex.h
 *
 * Created: 8/5/2013 8:47:53 AM
 *  Author: dcstanc
 */


#ifndef MUTEX_H_
#define MUTEX_H_
#include "kernel.h"

#if OSUSE_MUTEXES==1
typedef struct tmut
{
	unsigned char val;
	unsigned char procList[OSMAX_TASKS];
	tQueue blocked;
} OSMutex;

void OSCreateMutex(OSMutex *mutex);
void OSTakeMutex(OSMutex *mutex);
void OSGiveMutex(OSMutex *mutex);

#endif

#if OSUSE_MUTEXES==1 && OSUSE_CONDITIONALS==1
typedef struct cond
{
	unsigned char blockedProcess, pendingWake;
}  OSCond;

void OSCreateConditional(OSCond *cond);
void OSWait(OSCond *cond, OSMutex *mutex);
void OSSignal(OSCond *cond);

#endif

#endif /* MUTEX_H_ */
