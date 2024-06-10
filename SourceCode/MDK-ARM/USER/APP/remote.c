#include "remote.h"
#include "usart.h"
#include "debug_uart.h"

RC_remote_data_t rc_remote_data;
volatile uint8_t RC_remote_buffer[REMOTE_DATA_LEN];
uint32_t remote_hock;
uint8_t msg;

//初始化
void Remote_Init()
{
	HAL_REMOTE_UART_Init();  //初始化串口
	//HAL_REMOTE_DMA_Init();   //初始化DMA
	//HAL_UART_Receive_DMA(&REMOTE_UART_h,(uint8_t *)&rc_remote_data,REMOTE_DATA_LEN); //开始DMA传输
	HAL_UART_Receive_IT(&REMOTE_UART_h,(uint8_t*)RC_remote_buffer,REMOTE_DATA_LEN);
}



//读遥控数据
RC_remote_data_t Remote_read_data()
{
	//因为数据可能在处理的过程中被DMA覆盖，故需要拷贝一份出来
	static RC_remote_data_t remote_data_buffer;
	static RC_remote_data_t zero_data; //if remote data is vaild ,return zero data
	remote_data_buffer = rc_remote_data;
	//数据预处理
	remote_data_buffer.left_HRZC -=1024;
	remote_data_buffer.left_VETC -=1024;
	remote_data_buffer.right_HRZC -=1024;
	remote_data_buffer.right_VETC -=1024;
	remote_data_buffer.thumb_wheel -=1024;
	//check if the data is vaild
	if(remote_data_buffer.left_HRZC>800 || remote_data_buffer.left_HRZC<-800||
		 remote_data_buffer.left_VETC>800 || remote_data_buffer.left_VETC<-800||
		 remote_data_buffer.right_HRZC>800 || remote_data_buffer.right_HRZC<-800||
		 remote_data_buffer.right_VETC>800 || remote_data_buffer.right_VETC<-800||
		 remote_data_buffer.thumb_wheel>800 || remote_data_buffer.thumb_wheel<-800||
		 remote_data_buffer.S1==0 || remote_data_buffer.S1==0||
		 (remote_data_buffer.mouse_l !=0 && remote_data_buffer.mouse_l !=1)||
		 (remote_data_buffer.mouse_r !=0 && remote_data_buffer.mouse_r !=1)
	  )
	{
		//restart remote uart
		HAL_UART_DeInit(&REMOTE_UART_h);
		Remote_Init();
		//__HAL_DMA_SET_COUNTER(REMOTE_UART_h.hdmarx,REMOTE_DATA_LEN);
		return zero_data;
	}
	//死区处理
	if(remote_data_buffer.left_HRZC>-25&&remote_data_buffer.left_HRZC<25)
		remote_data_buffer.left_HRZC = 0;
	if(remote_data_buffer.left_VETC>-25&&remote_data_buffer.left_VETC<25)
		remote_data_buffer.left_VETC = 0;
	if(remote_data_buffer.right_HRZC>-25&&remote_data_buffer.right_HRZC<25)
		remote_data_buffer.right_HRZC = 0;
	if(remote_data_buffer.right_VETC>-25&&remote_data_buffer.right_VETC<25)
		remote_data_buffer.right_VETC = 0;
	if(remote_data_buffer.thumb_wheel>-25&&remote_data_buffer.thumb_wheel<25)
		remote_data_buffer.thumb_wheel = 0;
	
	return remote_data_buffer;
}



void Remote_UART_Callback(UART_HandleTypeDef *huart)
{
	//数据处理
	rc_remote_data.right_HRZC = ((int16_t)RC_remote_buffer[0] | ((int16_t)RC_remote_buffer[1] << 8)) & 0x07ff;		   //!< Channel 0
	rc_remote_data.right_VETC = (((int16_t)RC_remote_buffer[1] >> 3) | ((int16_t)RC_remote_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	rc_remote_data.left_HRZC = (((int16_t)RC_remote_buffer[2] >> 6) | ((int16_t)RC_remote_buffer[3] << 2) |		   //!< Channel 2
						                  ((int16_t)RC_remote_buffer[4] << 10)) & 0x07ff;
	rc_remote_data.left_VETC = ((int16_t)(RC_remote_buffer[4] >> 1) | ((int16_t)RC_remote_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	
	if(rc_remote_data.left_HRZC<1024)rc_remote_data.left_HRZC+=520; //软件校准偏差
	rc_remote_data.S1 = ((RC_remote_buffer[5] >> 4) & 0x0003);							   //!< Switch left
	rc_remote_data.S2 = ((RC_remote_buffer[5] >> 4) & 0x000C) >> 2;						   //!< Switch right
	rc_remote_data.mouse_x = RC_remote_buffer[6] | (RC_remote_buffer[7] << 8);					   //!< Mouse X axis
	rc_remote_data.mouse_y = RC_remote_buffer[8] | (RC_remote_buffer[9] << 8);					   //!< Mouse Y axis
	rc_remote_data.mouse_z = RC_remote_buffer[10] | (RC_remote_buffer[11] << 8);				   //!< Mouse Z axis
	rc_remote_data.mouse_l = RC_remote_buffer[12];										   //!< Mouse Left Is Press ?
	rc_remote_data.mouse_r = RC_remote_buffer[13];										   //!< Mouse Right Is Press ?
	rc_remote_data.thumb_wheel = RC_remote_buffer[16] | (RC_remote_buffer[17] << 8);				   //NULL

	HAL_UART_Receive_IT(&REMOTE_UART_h,(uint8_t *)RC_remote_buffer,REMOTE_DATA_LEN);
	remote_hock=0;  //接收到数据后归零
}

