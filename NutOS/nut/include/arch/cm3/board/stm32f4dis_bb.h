#ifndef DEV_UART
#define DEV_UART devUsartStm32_6
#include <dev/usartstm32.h>
#endif

#ifndef DEV_UART_NAME
#define DEV_UART_NAME devUsartStm32_6.dev_name
#endif

/* Ethernet interface */

#include <dev/stm32_emac.h>
#ifndef DEV_ETHER_NAME
#define DEV_ETHER_NAME  "eth0"
#endif

#include <arch/cm3/board/f4_discovery.h>
