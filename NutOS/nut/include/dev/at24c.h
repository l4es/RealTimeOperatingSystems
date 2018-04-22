#ifndef _DEV_AT24C_H_
#define _DEV_AT24C_H_

#include <sys/types.h>
#include <stdint.h>
#include <cfg/eeprom.h>

#ifndef NUT_CONFIG_AT24_ADR
#define NUT_CONFIG_AT24_ADR 0x50
#endif

struct at24c{
   uint8_t  SlaveAddress;   /*< Slave address on the bus */
   uint8_t  IAddrW;         /*< With of slaves address register in bytes */
   uint8_t  PageSize;       /*< Size of pages that can be written at once */
   uint32_t EepromSize;     /*< Size of the eeprom memory in total */
   uint8_t  Timeout;        /*< Timeout of internal programming in ms*/
#ifdef AT24C_BLOCK_ADDR
   uint8_t  BlInSla;        /*< Block in slave address (lower 3 bits used
                                for internal high byte of address) */
#endif
   HANDLE   ee_mutex;       /*< Mutex to avoid concurrent access to same EEPROM */
//   unsigned int EepromName;
};

extern int At24cWrite( struct at24c *at24cs, uint8_t *buffer, uint16_t len, uint16_t addr);
extern int At24cRead( struct at24c *at24cs, uint8_t *buffer, uint16_t len, uint16_t addr );

#endif
