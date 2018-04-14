
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

#if defined(NO_BOARD_LIB)
#include "chip.h"
const uint32_t OscRateIn = 12000000;
const uint32_t RTCOscRateIn = 32768;
#else
#include "board.h"
#endif

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set up and initialize hardware prior to call to main */
void SystemInit(void)
{
#if defined(NO_BOARD_LIB)
   /* Chip specific SystemInit */
   Chip_SystemInit();
#else
   /* Board specific SystemInit */
   Board_SystemInit();
#endif
}
