#include "debug_uart.h"
#include "main.h"
#include "usart.h"

uint8_t debug_str[DEBUG_STR_LEN];

/**
 * @brief ���ڳ�ʼ������ʼ����
 * @param ��
 * @return ��
 */

static uint8_t msg;
void Debug_UART_Init(void)
{
	HAL_DEBUG_UART_Init();
	HAL_UART_Receive_IT(&DEBUG_UART_h, &msg, 1); // ���ڿ�ʼ����
}

/**
 * @brief �����жϻص�����
 * @param p_args
 */
void Debug_UART_Callback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(&DEBUG_UART_h, &msg, 1, 1000); // ����ԭ�ⲻ������ȥ
	HAL_UART_Receive_IT(&DEBUG_UART_h, &msg, 1);	 // ���ڿ�ʼ����
}

/**
 * @brief ��Ϊֱ��ʹ��printf��Ҫ�޸�C������������Ҫ����ѣ�����Ҫ/n�����ֶ�fflush���ߵȻ��������˲Ż��ӡ����������ֲ��������д�˸��Զ����print
 * @param str �ַ���
 * @param bytes �ַ�������
 */
void Debug_UART_print(uint8_t *str, int bytes)
{
	// HAL_UART_Transmit(&DEBUG_UART_h, str, bytes, 0xffff);
	HAL_UART_Transmit_DMA(&DEBUG_UART_h, str, bytes);
}

// �ض���fputc
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
