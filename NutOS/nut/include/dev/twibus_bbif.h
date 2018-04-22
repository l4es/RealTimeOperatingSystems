#ifndef _DEV_TWIBUS_BBIF_H_
#define _DEV_TWIBUS_BBIF_H_

#include <sys/types.h>

typedef struct _NUTTWIICB NUTTWIICB;
/*
 * Runtime Data container.
 * This is installed in heap at initializaton
 * of a bus.
 */
struct _NUTTWIICB {
    /********** Master mode *********/

    /*! \brief Bus last error condition.
     */
    volatile int_fast8_t tw_mm_error;
};

extern NUTTWIBUS TwBbifBus;

#ifndef DEF_TWIBUS
#define DEF_TWIBUS TwBbifBus
#endif

#endif /* _DEV_TWIBUS_BBIF_H_ */
