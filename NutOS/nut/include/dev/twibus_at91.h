#ifndef _DEV_TWIBUS_AT91_H_
#define _DEV_TWIBUS_AT91_H_

#include <sys/types.h>
#include <cfg/arch.h>

typedef struct _NUTTWIICB NUTTWIICB;
/*
 * Runtime Data container.
 * This is installed in heap at initializaton
 * of a bus.
 */
struct _NUTTWIICB {
    /********** Master mode *********/

    /*! \brief Bus slave address.
     */
    volatile uint_fast16_t tw_mm_sla;

    /*! \brief Bus current error condition.
     */
    volatile int_fast8_t tw_mm_err;

    /*! \brief Bus last error condition.
     */
    volatile int_fast8_t tw_mm_error;

    /*! \brief Bus transmission data buffer pointer.
     */
    uint8_t *tw_mm_buf;

    /*! \brief Bus transmission data block length.
     */
    volatile uint_fast16_t tw_mm_len;

    /*! \brief Bus transmissinn position.
     */
    volatile uint_fast16_t tw_mm_idx;

    /*! \brief Transmission Ongoing Mutex.
     */
    HANDLE tw_mm_mtx;
};

extern NUTTWIBUS At91TwiBus;

#ifndef DEF_TWIBUS
#define DEF_TWIBUS At91TwiBus
#endif

#endif /* _DEV_TWIBUS_AT91_H_ */
