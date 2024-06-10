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

// ȫ�ֱ���
uint32_t LegControl_round; // �����˻غ���
Hexapod hexapod;		   // �����˽ṹ��

Gait_prg gait_prg;	  // ��̬�滮
uint32_t round_time;  // �غ�ʱ��
Thetas leg_offset[6]; // �Ȳ��ؽڽ�ƫ�ƣ����ڽ������Ի����˱���ĽǶȻ�������Զ������ĽǶ�

// ����
static void remote_deal(void);
extern "C"
{
	void LegControl_Task(void const *argument)
	{
		hexapod.Init();
		gait_prg.Init();
		osDelay(100);
		static uint32_t code_time_start, code_time_end, code_time; // ���ڼ����������ʱ�䣬��֤�����һ��ʱ����һ��
		while (1)
		{
			code_time_start = xTaskGetTickCount(); // ��ȡ��ǰsystickʱ��

			remote_deal();
			if (hexapod.velocity.omega >= 0)
				LegControl_round = (++LegControl_round) % N_POINTS; // ���ƻغ�������
			else
			{
				if (LegControl_round == 0)
					LegControl_round = N_POINTS - 1;
				else
					LegControl_round--;
			}
			/*��̬����*/
			gait_prg.CEN_and_pace_cal();
			gait_prg.gait_proggraming();
			/*��ʼ�ƶ�*/
			round_time = gait_prg.get_pace_time() / N_POINTS;
			hexapod.move(round_time);
			// �����������ʱ��
			code_time_end = xTaskGetTickCount();		 // ��ȡ��ǰsystickʱ��
			code_time = code_time_end - code_time_start; // �����ȡ��������ʱ�䣨8ms��
			if (code_time < round_time)
				osDelay(round_time - code_time); // ��֤����ִ�����ڵ��ڻغ�ʱ��
			else
				osDelay(1); // ������ʱ1ms
		}
	}
}

// ��ʼ���Ȳ���������ʼ�����ڣ�ʹ�ܴ��ڷ���
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

// ����������ٶ�
void Hexapod::velocity_cal(const RC_remote_data_t &remote_data)
{
	if (this->mode != HEXAPOD_MOVE || this->arm_sw==ARM_ON) // ����������ģʽ���ٶ�Ϊ0
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

	if (this->mode != HEXAPOD_BODY_ANGEL_CONTROL) // ������̬����ģʽ����������¶��ܿ���z��߶�
		body_pos.z += ROTATE_BODY_POS_SENSI * remote_data.left_VETC;
	if (this->mode == HEXAPOD_BODY_POS_CONTROL) // ��������λ�ÿ���ģʽ�����xyλ��
	{
		// body_pos.y += ROTATE_BODY_POS_SENSI * remote_data.right_VETC;
		// body_pos.x += ROTATE_BODY_POS_SENSI * remote_data.right_HRZC;
		body_pos.y = HEXAPOD_MAX_Y/660.0f * remote_data.right_VETC;
		body_pos.x = -HEXAPOD_MIN_X/660.0f * remote_data.right_HRZC;
	}   
	//������ֵ
	value_limit(body_pos.z, HEXAPOD_MIN_HEIGHT, HEXAPOD_MAX_HEIGHT);
	value_limit(body_pos.y, HEXAPOD_MIN_Y, HEXAPOD_MAX_Y);
	value_limit(body_pos.x, HEXAPOD_MIN_X, HEXAPOD_MAX_X);
	//һ�׵�ͨ�˲�
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
	if(!arm.set_pos(arm_end_pos)) //�޽������
		this->arm_end_pos=this->arm_end_last_pos;
}

/*
 *@brief ��鲦�����ݣ�����Ҫ��͹���
 *@param remote_data ң������
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
	case 1:					 // ����������
		mode = HEXAPOD_MOVE; // �ƶ�ģʽ
		break;
	case 2:								   // ����������
		mode = HEXAPOD_BODY_ANGEL_CONTROL; // ������ת�Ƕȿ���
		break;
	case 3:								 // �������м�
		mode = HEXAPOD_BODY_POS_CONTROL; // ����λ�ÿ���
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
	// //��ʱ�ĳɻ�е�ۣ����˲�������
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
 * @brief �û����˶�����
 * @param round_time �غ�ʱ�䣬��λms
 */
void Hexapod::move(uint32_t round_time)
{
	// // ������1-6�ĽǶ�
	// for (int i = 0; i < 6; i++)
	// {
	// 	legs[i].set_thetas((gait_prg.actions[i].thetas[LegControl_round]) - leg_offset[i]); // ���û�е�ȽǶ�
	// 	legs[i].set_time(round_time);														// ���û�е���ƶ�ʱ��
	// }
	// // ������5�ĽǶ�
	// if (gait_prg.actions[4].thetas[LegControl_round].angle[0] <= 0)
	// {
	// 	Thetas theta_temp;
	// 	theta_temp = (gait_prg.actions[4].thetas[LegControl_round]) - leg_offset[4];
	// 	theta_temp.angle[0] += 2 * PI;
	// 	legs[4].set_thetas(theta_temp); // ���û�е�ȽǶ�
	// }
	Thetas theta_temp;
	for (int i = 0; i < 6;i++)
	{
		theta_temp = (gait_prg.actions[i].thetas[LegControl_round]) - leg_offset[i];
		if(theta_temp.angle[0] <= -2.0f/3.0f*PI )
		{
			theta_temp.angle[0] += 2 * PI;
		}
		legs[i].set_thetas(theta_temp); // ���û�е�ȽǶ�
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
