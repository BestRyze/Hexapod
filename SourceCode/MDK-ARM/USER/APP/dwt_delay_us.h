#ifndef DWT_DELAY_US_H
#define DWT_DELAY_US_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/**
 * @brief  Initializes DWT_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 *         1: DWT counter Error
 *         0: DWT counter works
 */
uint32_t DWT_Delay_Init(void);

/**
 * @brief  This function provides a delay (in microseconds), 自定义的微秒级延时函数，可以兼容实时操作系统
 * @param  microseconds: delay in microseconds
 */
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)   
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

#ifdef __cplusplus
}
#endif

#endif
