#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"
#include "dma.h"

/***********改串口号的话要改这里****************/
#define HAL_REMOTE_UART_Init MX_UART7_Init
#define REMOTE_UART_Callback HAL_UART_RxCpltCallback
#define REMOTE_UART UART7
#define REMOTE_UART_h huart7
#define HAL_REMOTE_DMA_Init MX_DMA_Init
/************************************************/



#define REMOTE_DATA_LEN 18  //一帧18字节
#ifdef __cplusplus
extern "C"{
#endif
typedef struct
{
	int16_t right_HRZC;  //右边水平摇杆
	int16_t right_VETC;	//右边垂直摇杆
	int16_t left_HRZC;		//左边水平摇杆
	int16_t left_VETC;		//左边垂直摇杆
	uint8_t S1;						//
	uint8_t S2;						//
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;
	uint8_t mouse_r;
	uint16_t blank; //占空，暂时不使用
	int16_t thumb_wheel;   //拨轮
}RC_remote_data_t;



void Remote_Init(void);
RC_remote_data_t Remote_read_data(void);
void Remote_UART_Callback(UART_HandleTypeDef *huart);

extern uint32_t remote_hock;

#ifdef __cplusplus
}
#endif
	
#endif
