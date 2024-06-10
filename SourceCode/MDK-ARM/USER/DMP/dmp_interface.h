#ifndef DMP_INTERFACE_H
#define DMP_INTERFACE_H

#include "main.h"
#include "cmsis_os.h"


void my_get_ms(unsigned long *tick);
void my_delay_ms(uint32_t ms);
uint8_t hal_i2c_write(uint8_t addr,uint8_t reg, uint8_t len, uint8_t *dat);
uint8_t hal_i2c_read(uint8_t addr,uint8_t reg, uint8_t len, uint8_t *dat);

#endif
