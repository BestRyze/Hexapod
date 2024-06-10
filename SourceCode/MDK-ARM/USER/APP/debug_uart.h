#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "main.h"
#include "stdio.h"

/***********�Ĵ��ںŵĻ�Ҫ������****************/
#define HAL_DEBUG_UART_Init MX_UART8_Init
#define DEBUG_UART UART8
#define DEBUG_UART_h huart8  
/************************************************/

#define DEBUG_STR_LEN 0x1000 //����������

#define APP_PRINT(fn, ...) Debug_UART_print(debug_str,sprintf((char *)debug_str, fn , ##__VA_ARGS__));
extern uint8_t debug_str[DEBUG_STR_LEN];

#ifdef __cplusplus
extern "C"{
#endif
void Debug_UART_print(uint8_t *str, int bytes); //�Զ���Ĵ��ڴ�ӡ����Ϻ�APP_PRINTʹ�û���ֱ���ö���

void Debug_UART_Init(void); //����APP_PRINTǰ�ǵó�ʼ������

void Debug_UART_Callback(UART_HandleTypeDef *huart); //���жϻص���������
#ifdef __cplusplus	
}
#endif
#endif
