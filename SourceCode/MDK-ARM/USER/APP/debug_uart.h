#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "main.h"
#include "stdio.h"

/***********改串口号的话要改这里****************/
#define HAL_DEBUG_UART_Init MX_UART8_Init
#define DEBUG_UART UART8
#define DEBUG_UART_h huart8  
/************************************************/

#define DEBUG_STR_LEN 0x1000 //缓冲区长度

#define APP_PRINT(fn, ...) Debug_UART_print(debug_str,sprintf((char *)debug_str, fn , ##__VA_ARGS__));
extern uint8_t debug_str[DEBUG_STR_LEN];

#ifdef __cplusplus
extern "C"{
#endif
void Debug_UART_print(uint8_t *str, int bytes); //自定义的串口打印，配合宏APP_PRINT使用或者直接用都行

void Debug_UART_Init(void); //调用APP_PRINT前记得初始化串口

void Debug_UART_Callback(UART_HandleTypeDef *huart); //给中断回调函数调用
#ifdef __cplusplus	
}
#endif
#endif
