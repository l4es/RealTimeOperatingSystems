/************************************************
文件：TWI.h
用途：TWI头文件
************************************************/
#ifndef __TWI_H__
#define __TWI_H__

void i2c_init(void);
void i2c_start(void);
uint8 i2c_write_byte(uint8 data);
uint8 i2c_write_string(uint8 *data_buf, uint8 len);
uint8 i2c_read_byte(void);
void i2c_read_string(uint8 *receive, uint8 len);
void i2c_stop(void);

#endif
