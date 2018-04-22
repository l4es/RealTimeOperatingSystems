//*****************************************************************************
//
// sysctl.h - Prototypes for the system control driver.
//
//*****************************************************************************

#ifndef __SYSCTL_H__
#define __SYSCTL_H__

#if defined(SAM3U)
extern void SysCtlClockSet(unsigned long ulConfig);
#endif

#endif // __SYSCTL_H__
