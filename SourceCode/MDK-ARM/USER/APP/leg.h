#ifndef LEG_H
#define LEG_H

#include "Servo.h"
#include "usart.h"
#include "my_math.h"

class Leg
{
private:
	uint8_t send_buffer[100]; //���ͻ���
	Servo servos[3];
	Thetas theta;
	Servo_Broad_Cast servo_broad_cast;
	UART_HandleTypeDef *huart;
	void TX_Enable();
	void RX_Enable();
	void TX_Unable();
public:
	Leg(UART_HandleTypeDef *huart); // ���캯��
	Leg(){};						// �޲ι���
	void set_thetas(Thetas thetas); // ���û�е�ȵĽǶ�
	void set_time(uint16_t tims);	// ���û�е���ƶ���ʱ��
	void move_DMA();					// ��е���ƶ�����
	void move_UART();
	void move_wait();				// ���û�е�ȽǶȣ�����Ҫ�ȴ���ʼ������ƶ�
	void move_start();				// �û�е�ȿ�ʼ�˶�
	void load();					// �ϵ�
	void unload();					// ����
	void read_angle(uint32_t id);	// ��ȡ����Ƕ�
};



#endif
