#include "led_task.h"
#include "cmsis_os.h"
#include "debug_uart.h"
#include "remote.h"
#include "usart.h"




extern "C"{
void LED_Task(void const * argument)
{
	osDelay(100);
	osDelay(100);
	while(1)
	{
		if((remote_hock++)>3)
		{
			HAL_UART_DeInit(&REMOTE_UART_h); //»Ù“£øÿ∆˜¿Îœﬂ‘Ú≥¢ ‘÷ÿ∆Ù
			Remote_Init();
		}
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		osDelay(100);
	}
}
}
