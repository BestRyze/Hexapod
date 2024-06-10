#ifndef ARM_H
#define ARM_H

#include "Servo.h"
#include "usart.h"
#include "my_math.h"
#include "bsp.h"

#define ARM_LINK_LEN1 0.0f
#define ARM_LINK_LEN2 160.f
#define ARM_LINK_LEN3 200.f

class Arm
{
private:
	uint8_t send_buffer[100]; //���ͻ���
	Servo servos[3],grip_pawl_servo;
	Thetas theta;
	Servo_Broad_Cast servo_broad_cast;
	UART_HandleTypeDef *huart;
	Position3 end_pos;
	void TX_Enable();
	void TX_Unable();
public:
	Arm(UART_HandleTypeDef *huart); // ���캯��
	Arm(){};						// �޲ι���
	void set_thetas(Thetas thetas); // ���û�е�۵ĽǶ�
    void set_grip_theta(float theta); //���ü�צ�Ƕ�
	void set_time(uint16_t tims);	// ���û�е���ƶ���ʱ��
	void move_DMA();					// ��е���ƶ�����
	void move_UART();
	bool set_pos(Position3 end_pos);					//����ĩ��λ��				
	Position3 get_pos();
	bool ikine();					//���˶�����
};

#endif
