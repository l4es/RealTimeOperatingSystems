#ifndef _DEV_TWIBUS_GPIO_H_
#define _DEV_TWIBUS_GPIO_H_

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
};
extern NUTTWIBUS TwGpioBus;

#ifndef DEF_TWIBUS
#define DEF_TWIBUS TwGpioBus
#endif

#endif /* _DEV_TWIBUS_GPIO_H_ */
