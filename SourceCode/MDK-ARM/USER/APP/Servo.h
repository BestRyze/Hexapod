#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include "usart.h"

class Servo
{
private:
	uint16_t angle;		// ����Ƕ�0-1000����Ӧ-120�ȵ�120��
	uint16_t move_time; // ����ƶ����趨�Ƕȵ�ʱ�䣬��λms����Χ0-30000ms
	uint8_t id;			// ���id
public:
	Servo(){};
	Servo(uint8_t id){this->id=id;};
	void set_angle(float angle);
	void set_time(uint16_t move_time);
	void move(uint8_t* send_buffer);	   // ֱ���ƶ�
	void move_wait(uint8_t* send_buffer); // �趨����Ƕȣ�������Ҫ�ȴ���ʼָ��ſ�ʼ�ƶ�
	void read_angle(uint8_t* send_buffer);          //��ȡ����Ƕ�
};

class Servo_Broad_Cast
{
public:
	void move_start(uint8_t* send_buffer); // �����ʼ�ƶ�
	void move_stop(uint8_t* send_buffer);	// ���ֹͣ�ƶ�
	void unload(uint8_t* send_buffer);		// �������
	void load(uint8_t* send_buffer);		// ����ϵ�
};

// ����ָ����
#define SERVO_MOVE_TIME_WRITE 1
#define SERVO_MOVE_TIME_READ 2
#define SERVO_MOVE_TIME_WAIT_WRITE 7
#define SERVO_MOVE_TIME_WAIT_READ 8
#define SERVO_MOVE_START 11
#define SERVO_MOVE_STOP 12
#define SERVO_ID_WRITE 13
#define SERVO_ID_READ 14
#define SERVO_ANGLE_OFFSET_ADJUST 17
#define SERVO_ANGLE_OFFSET_WRITE 18
#define SERVO_ANGLE_OFFSET_READ 19
#define SERVO_ANGLE_LIMIT_WRITE 20
#define SERVO_ANGLE_LIMIT_READ 21
#define SERVO_VIN_LIMIT_WRITE 22
#define SERVO_VIN_LIMIT_READ 23
#define SERVO_TEMP_MAX_LIMIT_WRITE 24
#define SERVO_TEMP_MAX_LIMIT_READ 25
#define SERVO_TEMP_READ 26
#define SERVO_VIN_READ 27
#define SERVO_POS_READ 28
#define SERVO_OR_MOTOR_MODE_WRITE 29
#define SERVO_OR_MOTOR_MODE_READ 30
#define SERVO_LOAD_OR_UNLOAD_WRITE 31
#define SERVO_LOAD_OR_UNLOAD_READ 32
#define SERVO_LED_CTRL_WRITE 33
#define SERVO_LED_CTRL_READ 34
#define SERVO_LED_ERROR_WRITE 35
#define SERVO_LED_ERROR_READ 36

// ָ���
#define SERVO_MOVE_TIME_WRITE_LEN 7
#define SERVO_MOVE_TIME_READ_LEN 3
#define SERVO_MOVE_TIME_WAIT_WRITE_LEN 7
#define SERVO_MOVE_TIME_WAIT_READ_LEN 3
#define SERVO_MOVE_START_LEN 3
#define SERVO_MOVE_STOP_LEN 3
#define SERVO_ID_WRITE_LEN 4
#define SERVO_ID_READ_LEN 3
#define SERVO_ANGLE_OFFSET_ADJUST_LEN 4
#define SERVO_ANGLE_OFFSET_WRITE_LEN 3
#define SERVO_ANGLE_OFFSET_READ_LEN 3
#define SERVO_ANGLE_LIMIT_WRITE_LEN 7
#define SERVO_ANGLE_LIMIT_READ_LEN 3
#define SERVO_VIN_LIMIT_WRITE_LEN 7
#define SERVO_VIN_LIMIT_READ_LEN 3
#define SERVO_TEMP_MAX_LIMIT_WRITE_LEN 4
#define SERVO_TEMP_MAX_LIMIT_READ_LEN 3
#define SERVO_TEMP_READ_LEN 3
#define SERVO_VIN_READ_LEN 3
#define SERVO_POS_READ_LEN 3
#define SERVO_OR_MOTOR_MODE_WRITE_LEN 7
#define SERVO_OR_MOTOR_MODE_READ_LEN 3
#define SERVO_LOAD_OR_UNLOAD_WRITE_LEN 4
#define SERVO_LOAD_OR_UNLOAD_READ_LEN 3
#define SERVO_LED_CTRL_WRITE_LEN 4
#define SERVO_LED_CTRL_READ_LEN 3
#define SERVO_LED_ERROR_WRITE_LEN 4
#define SERVO_LED_ERROR_READ_LEN 3


//����ָ���
#define RECV_SERVO_MOVE_TIME_READ_LEN 7
#define RECV_SERVO_MOVE_TIME_WAIT_READ_LEN 7
#define RECV_SERVO_ID_READ_LEN 4
#define RECV_SERVO_ANGLE_OFFSET_READ_LEN 4
#define RECV_SERVO_ANGLE_LIMIT_READ_LEN 7
#define RECV_SERVO_VIN_LIMIT_READ_LEN 7
#define RECV_SERVO_TEMP_MAX_LIMIT_READ_LEN 4
#define RECV_SERVO_TEMP_READ_LEN 4
#define RECV_SERVO_VIN_READ_LEN 5
#define RECV_SERVO_POS_READ_LEN 5
#define RECV_SERVO_OR_MOTOR_MODE_READ_LEN 7
#define RECV_SERVO_LOAD_OR_UNLOAD_READ_LEN 4
#define RECV_SERVO_LED_CTRL_READ_LEN 4
#define RECV_SERVO_LED_ERROR_READ_LEN 4

#define SERVO_BROADCAST_ID 0xFE // �㲥id

#endif
