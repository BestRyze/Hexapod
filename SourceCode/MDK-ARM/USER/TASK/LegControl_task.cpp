#include "LegControl_task.h"
#include "gait_prg.h"
#include "cmsis_os.h"
#include "leg.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "bsp.h"
#include "remote.h"
#include "dwt_delay_us.h"
#include "mpu6050.h"

using namespace std;

// 全局变量
uint32_t LegControl_round; // 机器人回合数
Hexapod hexapod;		   // 机器人结构体

Gait_prg gait_prg;	  // 步态规划
uint32_t round_time;  // 回合时间
Thetas leg_offset[6]; // 腿部关节角偏移，用于将舵机相对机器人本体的角度换算至相对舵机本身的角度

// 函数
static void remote_deal(void);
extern "C"
{
	void LegControl_Task(void const *argument)
	{
		hexapod.Init();
		gait_prg.Init();
		osDelay(100);
		static uint32_t code_time_start, code_time_end, code_time; // 用于计算程序运行时间，保证程序隔一段时间跑一遍
		while (1)
		{
			code_time_start = xTaskGetTickCount(); // 获取当前systick时间

			remote_deal();
			if (hexapod.velocity.omega >= 0)
				LegControl_round = (++LegControl_round) % N_POINTS; // 控制回合自增长
			else
			{
				if (LegControl_round == 0)
					LegControl_round = N_POINTS - 1;
				else
					LegControl_round--;
			}
			/*步态控制*/
			gait_prg.CEN_and_pace_cal();
			gait_prg.gait_proggraming();
			/*开始移动*/
			round_time = gait_prg.get_pace_time() / N_POINTS;
			hexapod.move(round_time);
			// 计算程序运行时间
			code_time_end = xTaskGetTickCount();		 // 获取当前systick时间
			code_time = code_time_end - code_time_start; // 做差获取程序运行时间（8ms）
			if (code_time < round_time)
				osDelay(round_time - code_time); // 保证程序执行周期等于回合时间
			else
				osDelay(1); // 至少延时1ms
		}
	}
}

// 初始化腿部变量并初始化串口，使能串口发送
void Hexapod::Init(void)
{
	legs[0] = Leg(&huart1);
	legs[1] = Leg(&huart2);
	legs[2] = Leg(&huart3);
	legs[3] = Leg(&huart4);
	legs[4] = Leg(&huart5);
	legs[5] = Leg(&huart6);
	arm = Arm(&huart8);
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_USART6_UART_Init();
	MX_UART8_Init();
	leg_offset[0] = Thetas(PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[1] = Thetas(0.0f, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[2] = Thetas(-PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[3] = Thetas(3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[4] = Thetas(PI, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[5] = Thetas(-3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	mpu_pid_x.Init(MPU_X_PID_KP, MPU_X_PID_KI, MPU_X_PID_KD, CIR_OFF);
	mpu_pid_y.Init(MPU_Y_PID_KP, MPU_Y_PID_KI, MPU_Y_PID_KD, CIR_OFF);
	velocity_fof[0].set_k_filter(VELOCITY_FOF_K);
	velocity_fof[1].set_k_filter(VELOCITY_FOF_K);
	velocity_fof[2].set_k_filter(VELOCITY_FOF_K);
	body_pos_fof[0].set_k_filter(BODY_POS_FOF_K);
	body_pos_fof[1].set_k_filter(BODY_POS_FOF_K);
	body_pos_fof[2].set_k_filter(BODY_POS_FOF_K);
	body_angle_fof[0].set_k_filter(BODY_ANGLE_FOF_K);
	body_angle_fof[1].set_k_filter(BODY_ANGLE_FOF_K);
	body_angle_fof[2].set_k_filter(BODY_ANGLE_FOF_K);

	arm_end_pos.x = 0;
	arm_end_pos.y = 200;
	arm_end_pos.z = 0;
}

// 计算机器人速度
void Hexapod::velocity_cal(const RC_remote_data_t &remote_data)
{
	if (this->mode != HEXAPOD_MOVE || this->arm_sw==ARM_ON) // 若不是行走模式则速度为0
	{
		velocity.Vx = 0;
		velocity.Vy = 0;
		velocity.omega = 0;
	}
	else
	{
		velocity.Vx = velocity_fof[0].cal(0.3f * remote_data.right_HRZC);
		velocity.Vy = velocity_fof[1].cal(0.3f * remote_data.right_VETC);
		velocity.omega = velocity_fof[2].cal(-0.3f * remote_data.left_HRZC);
	}
	if(velocity.Vx>-0.0001f && velocity.Vx<0.0001f)velocity.Vx=0;
	if(velocity.Vy>-0.0001f && velocity.Vy<0.0001f)velocity.Vy=0;
	if(velocity.omega>-0.0001f && velocity.omega<0.0001f)velocity.omega=0;
	gait_prg.set_velocity(velocity);
}

void Hexapod::body_position_cal(const RC_remote_data_t &remote_data)
{
	if(arm_sw==ARM_ON)return;

	if (this->mode != HEXAPOD_BODY_ANGEL_CONTROL) // 除了姿态控制模式，其他情况下都能控制z轴高度
		body_pos.z += ROTATE_BODY_POS_SENSI * remote_data.left_VETC;
	if (this->mode == HEXAPOD_BODY_POS_CONTROL) // 若是身体位置控制模式则计算xy位置
	{
		// body_pos.y += ROTATE_BODY_POS_SENSI * remote_data.right_VETC;
		// body_pos.x += ROTATE_BODY_POS_SENSI * remote_data.right_HRZC;
		body_pos.y = HEXAPOD_MAX_Y/660.0f * remote_data.right_VETC;
		body_pos.x = -HEXAPOD_MIN_X/660.0f * remote_data.right_HRZC;
	}   
	//限制数值
	value_limit(body_pos.z, HEXAPOD_MIN_HEIGHT, HEXAPOD_MAX_HEIGHT);
	value_limit(body_pos.y, HEXAPOD_MIN_Y, HEXAPOD_MAX_Y);
	value_limit(body_pos.x, HEXAPOD_MIN_X, HEXAPOD_MAX_X);
	//一阶低通滤波
	body_pos.y = body_angle_fof[1].cal(body_pos.y);
	body_pos.x = body_angle_fof[2].cal(body_pos.x);
	gait_prg.set_body_position(body_pos);
}


void Hexapod::arm_position_cal(const RC_remote_data_t &remote_data)
{
	this->arm_end_last_pos = this->arm_end_pos;
	switch (this->arm_sw)
	{
	case ARM_ON:
		this->arm_end_pos.x += remote_data.right_HRZC*ROTATE_BODY_POS_SENSI;
		this->arm_end_pos.y += remote_data.right_VETC*ROTATE_BODY_POS_SENSI;
		this->arm_end_pos.z += remote_data.left_VETC*ROTATE_BODY_POS_SENSI;
		this->arm_grip_pawl_angle +=remote_data.left_HRZC*ROTATE_BODY_ANGLE_SENSI;
		value_limit(this->arm_grip_pawl_angle,-1.3f,0);
		this->arm.set_grip_theta(this->arm_grip_pawl_angle); 
		break;
	default:
		break;
	}
	if(!arm.set_pos(arm_end_pos)) //无解则回退
		this->arm_end_pos=this->arm_end_last_pos;
}

/*
 *@brief 检查拨轮数据，符合要求就归零
 *@param remote_data 遥控数据
 */
void Hexapod::body_pos_zero(const RC_remote_data_t &remote_data)
{
	if (remote_data.thumb_wheel > 500)
	{
		this->body_pos.zero();
	}
}

void Hexapod::mode_select(const RC_remote_data_t &remote_data)
{
	switch (remote_data.S1)
	{
	case 1:					 // 拨杆在上面
		mode = HEXAPOD_MOVE; // 移动模式
		break;
	case 2:								   // 拨杆在下面
		mode = HEXAPOD_BODY_ANGEL_CONTROL; // 机身旋转角度控制
		break;
	case 3:								 // 拨杆在中间
		mode = HEXAPOD_BODY_POS_CONTROL; // 机身位置控制
	default:
		break;
	}
	arm_sw = ARM_OFF;
	// switch (remote_data.S2)
	// {
	// case 0:
	// 	mpu_sw = MPU_ON;
	// 	break;
	// case 1:
	// 	mpu_sw = MPU_OFF;
	// 	break;
	// default:
	// 	break;
	// }
	// //暂时改成机械臂，拨杆不够用了
	// switch (remote_data.S2)
	// {
	// case 0:
	// 	arm_sw = ARM_ON;
	// 	break;
	// case 1:
	// 	arm_sw = ARM_OFF;
	// 	break;
	// default:
	// 	break;
	// }
}

static void remote_deal(void)
{
	static RC_remote_data_t remote_data;
	remote_data = Remote_read_data();
	hexapod.mode_select(remote_data);
	hexapod.velocity_cal(remote_data);
	hexapod.body_position_cal(remote_data);
	hexapod.body_pos_zero(remote_data);
	hexapod.arm_position_cal(remote_data);
}


/*
 * @brief 让机器人动起来
 * @param round_time 回合时间，单位ms
 */
void Hexapod::move(uint32_t round_time)
{
	// // 设置腿1-6的角度
	// for (int i = 0; i < 6; i++)
	// {
	// 	legs[i].set_thetas((gait_prg.actions[i].thetas[LegControl_round]) - leg_offset[i]); // 设置机械腿角度
	// 	legs[i].set_time(round_time);														// 设置机械腿移动时间
	// }
	// // 设置腿5的角度
	// if (gait_prg.actions[4].thetas[LegControl_round].angle[0] <= 0)
	// {
	// 	Thetas theta_temp;
	// 	theta_temp = (gait_prg.actions[4].thetas[LegControl_round]) - leg_offset[4];
	// 	theta_temp.angle[0] += 2 * PI;
	// 	legs[4].set_thetas(theta_temp); // 设置机械腿角度
	// }
	Thetas theta_temp;
	for (int i = 0; i < 6;i++)
	{
		theta_temp = (gait_prg.actions[i].thetas[LegControl_round]) - leg_offset[i];
		if(theta_temp.angle[0] <= -2.0f/3.0f*PI )
		{
			theta_temp.angle[0] += 2 * PI;
		}
		legs[i].set_thetas(theta_temp); // 设置机械腿角度
		legs[i].set_time(round_time);
		legs[i].move_DMA();
	}
	arm.set_time(round_time);
	arm.move_DMA();
	// legs[0].move_DMA();
	// legs[1].move_DMA();
	// legs[2].move_DMA();
	// legs[3].move_DMA();
	// legs[4].move_DMA();
	// legs[5].move_DMA();
}
