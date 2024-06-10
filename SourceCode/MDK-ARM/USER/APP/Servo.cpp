#include "Servo.h"
#include "gait_prg.h"

volatile uint8_t cmd_send_buffer[12] = {0x55, 0x55};

void Servo::set_angle(float angle)
{
	this->angle = angle / PI * 750 + 500; // 将弧度转换为舵机值
}

void Servo::set_time(uint16_t move_time)
{
	this->move_time = move_time;
}

// 计算校验和
static uint8_t check_sum(uint8_t *send_data, uint8_t data_len)
{
	static uint16_t checksum;
	checksum = send_data[2] + send_data[3] + send_data[4];
	for (int i = 0; i < data_len - 3; i++)
	{
		checksum += send_data[5 + i];
	}
	uint8_t i;
	i = uint8_t(~checksum); // 取反
	return i;
}

// 发送舵机移动指令
void Servo::move(uint8_t *send_buffer)
{
	send_buffer[0] = 0x55;
	send_buffer[1] = 0x55;
	send_buffer[2] = this->id;
	send_buffer[3] = SERVO_MOVE_TIME_WRITE_LEN;
	send_buffer[4] = SERVO_MOVE_TIME_WRITE;
	send_buffer[5] = this->angle;
	send_buffer[6] = this->angle >> 8;
	send_buffer[7] = this->move_time;
	send_buffer[8] = this->move_time >> 8;
	send_buffer[9] = check_sum(send_buffer, SERVO_MOVE_TIME_WRITE_LEN);
}

// 设定舵机角度，但是需要等待开始指令才开始移动
void Servo::move_wait(uint8_t *send_buffer)
{
	send_buffer[0] = 0x55;
	send_buffer[1] = 0x55;
	send_buffer[2] = this->id;
	send_buffer[3] = SERVO_MOVE_TIME_WAIT_WRITE;
	send_buffer[4] = SERVO_MOVE_TIME_WAIT_WRITE_LEN;
	send_buffer[5] = this->angle;
	send_buffer[6] = this->angle >> 8;
	send_buffer[7] = this->move_time;
	send_buffer[8] = this->move_time >> 8;
	send_buffer[9] = check_sum(send_buffer, SERVO_MOVE_TIME_WAIT_WRITE_LEN);
}

void Servo::read_angle(uint8_t *send_buffer)
{
	send_buffer[0] = 0x55;
	send_buffer[1] = 0x55;
	send_buffer[2] = this->id;
	send_buffer[3] = SERVO_POS_READ;
	send_buffer[4] = SERVO_POS_READ_LEN;
	send_buffer[5] = check_sum(send_buffer,SERVO_POS_READ_LEN);
}

/*************广播指令***********/
void Servo_Broad_Cast::move_start(uint8_t *send_buffer)
{
	send_buffer[0] = 0x55;
	send_buffer[1] = 0x55;
	send_buffer[2] = SERVO_BROADCAST_ID;
	send_buffer[3] = SERVO_MOVE_START;
	send_buffer[4] = SERVO_MOVE_START_LEN;
	send_buffer[5] = check_sum(send_buffer, SERVO_MOVE_START_LEN);
}

void Servo_Broad_Cast::move_stop(uint8_t *send_buffer)
{
	send_buffer[0] = 0x55;
	send_buffer[1] = 0x55;
	send_buffer[2] = SERVO_BROADCAST_ID;
	send_buffer[3] = SERVO_MOVE_STOP;
	send_buffer[4] = SERVO_MOVE_STOP_LEN;
	send_buffer[5] = check_sum(send_buffer, SERVO_MOVE_STOP_LEN);
}

void Servo_Broad_Cast::unload(uint8_t *send_buffer)
{
	send_buffer[0] = 0x55;
	send_buffer[1] = 0x55;
	send_buffer[2] = SERVO_BROADCAST_ID;
	send_buffer[3] = SERVO_LOAD_OR_UNLOAD_WRITE;
	send_buffer[4] = SERVO_LOAD_OR_UNLOAD_WRITE_LEN;
	send_buffer[5] = 0; // 0表示掉电
	send_buffer[6] = check_sum(send_buffer, SERVO_LOAD_OR_UNLOAD_WRITE_LEN);
}

void Servo_Broad_Cast::load(uint8_t *send_buffer)
{
	send_buffer[0] = 0x55;
	send_buffer[1] = 0x55;
	send_buffer[2] = SERVO_BROADCAST_ID;
	send_buffer[3] = SERVO_LOAD_OR_UNLOAD_WRITE;
	send_buffer[4] = SERVO_LOAD_OR_UNLOAD_WRITE_LEN;
	send_buffer[5] = 1; // 1表示上电
	send_buffer[6] = check_sum(send_buffer, SERVO_LOAD_OR_UNLOAD_WRITE_LEN);
}


