#include "dmp_interface.h"
#include "i2c.h"
#include "main.h"


void my_get_ms(unsigned long *tick)
{
    if(!tick)
        return;
    *tick = xTaskGetTickCount();	
}

void my_delay_ms(uint32_t ms)
{
    osDelay(ms);
}

uint8_t hal_i2c_write(uint8_t addr,uint8_t reg, uint8_t len, uint8_t *dat)
{
    return HAL_I2C_Mem_Write(&hi2c4,addr<<1,reg,I2C_MEMADD_SIZE_8BIT,dat,len,1000);
}

uint8_t hal_i2c_read(uint8_t addr,uint8_t reg, uint8_t len, uint8_t *dat)
{
    return HAL_I2C_Mem_Read(&hi2c4,addr<<1,reg,I2C_MEMADD_SIZE_8BIT,dat,len,1000);
}
