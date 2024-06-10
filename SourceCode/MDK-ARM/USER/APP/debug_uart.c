#include "debug_uart.h"
#include "main.h"
#include "usart.h"

uint8_t debug_str[DEBUG_STR_LEN];

/**
 * @brief 串口初始化并开始接收
 * @param 无
 * @return 无
 */

static uint8_t msg;
void Debug_UART_Init(void)
{
	HAL_DEBUG_UART_Init();
	HAL_UART_Receive_IT(&DEBUG_UART_h, &msg, 1); // 串口开始接收
}

/**
 * @brief 串口中断回调函数
 * @param p_args
 */
void Debug_UART_Callback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(&DEBUG_UART_h, &msg, 1, 1000); // 数据原封不动发回去
	HAL_UART_Receive_IT(&DEBUG_UART_h, &msg, 1);	 // 串口开始接收
}

/**
 * @brief 因为直接使用printf需要修改C开发环境，需要分配堆，且需要/n或者手动fflush或者等缓冲区满了才会打印，不方便移植，故这里写了个自定义的print
 * @param str 字符串
 * @param bytes 字符串长度
 */
void Debug_UART_print(uint8_t *str, int bytes)
{
	// HAL_UART_Transmit(&DEBUG_UART_h, str, bytes, 0xffff);
	HAL_UART_Transmit_DMA(&DEBUG_UART_h, str, bytes);
}

// 重定向fputc
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&DEBUG_UART_h, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}
