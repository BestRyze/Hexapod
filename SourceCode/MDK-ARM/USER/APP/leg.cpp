#include "leg.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include "bsp.h"


uint8_t receive_buffer[100];

Leg::Leg(UART_HandleTypeDef *huart)
{
	this->huart = huart;
	servos[0] = Servo(1);
	servos[1] = Servo(2);
	servos[2] = Servo(3);
}

void Leg::set_thetas(Thetas theta)
{
	this->theta.angle[0] = theta.angle[0];
	this->theta.angle[1] = theta.angle[1];
	this->theta.angle[2] = theta.angle[2];
	this->servos[0].set_angle(theta.angle[0]);
	this->servos[1].set_angle(theta.angle[1]);
	this->servos[2].set_angle(theta.angle[2]);
}

void Leg::set_time(uint16_t move_time)
{
	servos[0].set_time(move_time);
	servos[1].set_time(move_time);
	servos[2].set_time(move_time);
}

void Leg::move_DMA()
{
	this->TX_Enable();
	this->servos[0].move(this->send_buffer);
	this->servos[1].move(this->send_buffer + SERVO_MOVE_TIME_WRITE_LEN + 3);
	this->servos[2].move(this->send_buffer + (SERVO_MOVE_TIME_WRITE_LEN + 3) * 2);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, (SERVO_MOVE_TIME_WRITE_LEN + 3) * 3);
}

void Leg::move_UART()
{
	this->TX_Enable();
	this->servos[0].move(this->send_buffer);
	this->servos[1].move(this->send_buffer + SERVO_MOVE_TIME_WRITE_LEN + 3);
	this->servos[2].move(this->send_buffer + (SERVO_MOVE_TIME_WRITE_LEN + 3) * 2);
	HAL_UART_Transmit(this->huart, this->send_buffer, (SERVO_MOVE_TIME_WRITE_LEN + 3) * 3, 1000);
}

void Leg::move_wait()
{
	this->TX_Enable();
	this->servos[0].move_wait(this->send_buffer);
	this->servos[1].move_wait(this->send_buffer + SERVO_MOVE_TIME_WAIT_WRITE_LEN + 3);
	this->servos[2].move(this->send_buffer + (SERVO_MOVE_TIME_WAIT_WRITE_LEN + 3) * 2);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, (SERVO_MOVE_TIME_WAIT_WRITE_LEN + 3) * 3);
}

void Leg::move_start()
{
	this->TX_Enable();
	this->servo_broad_cast.move_start(this->send_buffer);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, SERVO_MOVE_START_LEN + 3);
}

void Leg::load()
{
	this->servo_broad_cast.load(this->send_buffer);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, SERVO_LOAD_OR_UNLOAD_WRITE_LEN + 3);
}

void Leg::unload()
{
	this->servo_broad_cast.unload(this->send_buffer);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, SERVO_LOAD_OR_UNLOAD_WRITE_LEN + 3);
}

void Leg::read_angle(uint32_t id)
{
	this->TX_Enable();
	this->servos[id - 1].read_angle(this->send_buffer);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, SERVO_POS_READ_LEN + 3); // 为了在发送完成后再开启接收，这里不使用DMA
	while (__HAL_UART_GET_FLAG(this->huart, UART_FLAG_TC))
		; // 等待串口发送完成

	this->RX_Enable(); // 使能接收
	HAL_UART_Receive(this->huart, receive_buffer, RECV_SERVO_POS_READ_LEN + 3, 1000);
	float angle = (((uint16_t)receive_buffer[5] | ((uint16_t)receive_buffer[6] << 8)) - 500) / 750 * PI;
	this->theta.angle[id - 1] = angle;
	this->servos[id - 1].set_angle(angle);

	this->TX_Enable(); // 发送完毕后使能发送
}

// 使能接收
void Leg::RX_Enable()
{
	switch ((uint32_t)(this->huart->Instance))
	{
	case (uint32_t)USART1:
		__LEG1_RXEN();
		break;
	case (uint32_t)USART2:
		__LEG2_RXEN();
		break;
	case (uint32_t)USART3:
		__LEG3_RXEN();
		break;
	case (uint32_t)UART4:
		__LEG4_RXEN();
		break;
	case (uint32_t)UART5:
		__LEG5_RXEN();
		break;
	case (uint32_t)USART6:
		__LEG6_RXEN();
		break;
	case (uint32_t)UART8:
		__ARM_RXEN();
		break;
	default:
		break;
	}
}

// 使能发送
void Leg::TX_Enable()
{
	switch ((uint32_t)(this->huart->Instance))
	{
	case (uint32_t)USART1:
		__LEG1_TXEN();
		break;
	case (uint32_t)USART2:
		__LEG2_TXEN();
		break;
	case (uint32_t)USART3:
		__LEG3_TXEN();
		break;
	case (uint32_t)UART4:
		__LEG4_TXEN();
		break;
	case (uint32_t)UART5:
		__LEG5_TXEN();
		break;
	case (uint32_t)USART6:
		__LEG6_TXEN();
		break;
	case (uint32_t)UART8:
		__ARM_TXEN();
		break;
	default:
		break;
	}
}

void Leg::TX_Unable()
{
	switch ((uint32_t)(this->huart->Instance))
	{
	case (uint32_t)USART1:
		__LEG1_TXUEN();
		break;
	case (uint32_t)USART2:
		__LEG2_TXUEN();
		break;
	case (uint32_t)USART3:
		__LEG3_TXUEN();
		break;
	case (uint32_t)UART4:
		__LEG4_TXUEN();
		break;
	case (uint32_t)UART5:
		__LEG5_TXUEN();
		break;
	case (uint32_t)USART6:
		__LEG6_TXUEN();
		break;
	case (uint32_t)UART8:
		__ARM_TXUEN();
		break;
	default:
		break;
	}
}


