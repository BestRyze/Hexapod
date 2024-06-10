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
	uint8_t send_buffer[100]; //发送缓存
	Servo servos[3],grip_pawl_servo;
	Thetas theta;
	Servo_Broad_Cast servo_broad_cast;
	UART_HandleTypeDef *huart;
	Position3 end_pos;
	void TX_Enable();
	void TX_Unable();
public:
	Arm(UART_HandleTypeDef *huart); // 构造函数
	Arm(){};						// 无参构造
	void set_thetas(Thetas thetas); // 设置机械臂的角度
    void set_grip_theta(float theta); //设置夹爪角度
	void set_time(uint16_t tims);	// 设置机械臂移动的时间
	void move_DMA();					// 机械腿移动命令
	void move_UART();
	bool set_pos(Position3 end_pos);					//设置末端位置				
	Position3 get_pos();
	bool ikine();					//逆运动解算
};

#endif
