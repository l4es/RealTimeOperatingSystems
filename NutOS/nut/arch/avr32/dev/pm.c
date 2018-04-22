
#include <cfg/clock.h>
#include <cfg/arch.h>

#if defined ( MCU_AVR32UC3L064 )
#include "pm_uc3l.c"
#else
#include "pm_uc3a.c"
#endif

