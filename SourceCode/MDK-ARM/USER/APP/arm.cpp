#include "arm.h"

Arm::Arm(UART_HandleTypeDef *huart)
{
	this->huart = huart;
	servos[0] = Servo(1);
	servos[1] = Servo(2);
	servos[2] = Servo(3);
	grip_pawl_servo = Servo(4);
	this->end_pos.x = 0;
	this->end_pos.y = 150;
	this->end_pos.z = 200;
}

void Arm::TX_Enable()
{
    __ARM_TXEN();
}

void Arm::TX_Unable()
{
    __ARM_TXUEN();
}

void Arm::move_DMA()
{
	this->ikine();
    this->TX_Enable();
	this->servos[0].move(this->send_buffer);
	this->servos[1].move(this->send_buffer + SERVO_MOVE_TIME_WRITE_LEN + 3);
	this->servos[2].move(this->send_buffer + (SERVO_MOVE_TIME_WRITE_LEN + 3) * 2);
    this->grip_pawl_servo.move(this->send_buffer + (SERVO_MOVE_TIME_WRITE_LEN + 3) * 3);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, (SERVO_MOVE_TIME_WRITE_LEN + 3) * 4);
}

void Arm::move_UART()
{
    this->TX_Enable();
	this->servos[0].move(this->send_buffer);
	this->servos[1].move(this->send_buffer + SERVO_MOVE_TIME_WRITE_LEN + 3);
	this->servos[2].move(this->send_buffer + (SERVO_MOVE_TIME_WRITE_LEN + 3) * 2);
    this->grip_pawl_servo.move(this->send_buffer + (SERVO_MOVE_TIME_WRITE_LEN + 3) * 3);
	HAL_UART_Transmit(this->huart, this->send_buffer, (SERVO_MOVE_TIME_WRITE_LEN + 3) * 4,1000);
}

void Arm::set_thetas(Thetas theta)
{
	this->theta.angle[0] = theta.angle[0];
	this->theta.angle[1] = theta.angle[1];
	this->theta.angle[2] = theta.angle[2];
	this->servos[0].set_angle(theta.angle[0]);
	this->servos[1].set_angle(theta.angle[1]);
	this->servos[2].set_angle(theta.angle[2]);   
}

void Arm::set_grip_theta(float theta)
{
    this->grip_pawl_servo.set_angle(theta);
}

void Arm::set_time(uint16_t move_time)
{
    servos[0].set_time(move_time);
	servos[1].set_time(move_time);
	servos[2].set_time(move_time);
    grip_pawl_servo.set_time(move_time);
}

bool Arm::set_pos(Position3 end_pos)
{
	this->end_pos = end_pos;
	return this->ikine();
}

Position3 Arm::get_pos()
{
	return this->end_pos;
}


bool Arm::ikine()
{
	static Position3 pos1;
    static float R, Lr, alpha_r, alpha1, alpha2;
    pos1 = this->end_pos;
    R = sqrt(pow(pos1.x, 2) + pow(pos1.y, 2));
    Lr = sqrt(pow(pos1.z, 2) + pow((R - ARM_LINK_LEN1), 2));
    alpha_r = atan(-pos1.z / (R - ARM_LINK_LEN1));
    alpha1 = acos((pow(ARM_LINK_LEN2, 2) + pow(Lr, 2) - pow(ARM_LINK_LEN3, 2)) / (2 * Lr * ARM_LINK_LEN2));
    alpha2 = acos((pow(Lr, 2) + pow(ARM_LINK_LEN3, 2) - pow(ARM_LINK_LEN2, 2)) / (2 * Lr * ARM_LINK_LEN3));
    Thetas thetas(atan2(pos1.y, pos1.x), alpha1 - alpha_r, -(alpha1 + alpha2));
	thetas.angle[0] -= PI/2;
	thetas.angle[1] = PI/2 - thetas.angle[1];
	thetas.angle[2] = -thetas.angle[2];
    //value_limit(thetas.angle[1], MIN_JOINT2_RAD, MAX_JOINT2_RAD);
    //value_limit(thetas.angle[2], MIN_JOINT3_RAD, MAX_JOINT3_RAD);
	if(isnan(thetas.angle[0])||isnan(thetas.angle[1])||isnan(thetas.angle[2])) //若无解则不设置舵机的角度，并返回flase
		return false;
		
	set_thetas(thetas);
	return true;
    //this->theta = thetas;
}

