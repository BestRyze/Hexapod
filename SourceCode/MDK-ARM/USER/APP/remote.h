#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"
#include "dma.h"

/***********�Ĵ��ںŵĻ�Ҫ������****************/
#define HAL_REMOTE_UART_Init MX_UART7_Init
#define REMOTE_UART_Callback HAL_UART_RxCpltCallback
#define REMOTE_UART UART7
#define REMOTE_UART_h huart7
#define HAL_REMOTE_DMA_Init MX_DMA_Init
/************************************************/



#define REMOTE_DATA_LEN 18  //һ֡18�ֽ�
#ifdef __cplusplus
extern "C"{
#endif
typedef struct
{
	int16_t right_HRZC;  //�ұ�ˮƽҡ��
	int16_t right_VETC;	//�ұߴ�ֱҡ��
	int16_t left_HRZC;		//���ˮƽҡ��
	int16_t left_VETC;		//��ߴ�ֱҡ��
	uint8_t S1;						//
	uint8_t S2;						//
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;
	uint8_t mouse_r;
	uint16_t blank; //ռ�գ���ʱ��ʹ��
	int16_t thumb_wheel;   //����
}RC_remote_data_t;



void Remote_Init(void);
RC_remote_data_t Remote_read_data(void);
void Remote_UART_Callback(UART_HandleTypeDef *huart);

extern uint32_t remote_hock;

#ifdef __cplusplus
}
#endif
	
#endif
