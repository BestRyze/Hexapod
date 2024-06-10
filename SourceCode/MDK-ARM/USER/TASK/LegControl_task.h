#ifndef LEGCONTROL_TASK_H
#define LEGCONTROL_TASK_H

#include "my_math.h"
#include "leg.h"
#include "remote.h"
#include "gait_prg.h"
#include "arm.h"

#define LEG_JOINT2_OFFSET PI / 2
#define LEG_JOINT3_OFFSET -2 * PI / 9

#define HEXAPOD_MIN_HEIGHT -70.0f
#define HEXAPOD_MAX_HEIGHT 70.0f
#define HEXAPOD_MIN_X -40.0f
#define HEXAPOD_MAX_X 40.0f
#define HEXAPOD_MIN_Y -40.0f
#define HEXAPOD_MAX_Y 40.0f


#define HEXAPOD_MIN_X_ROTATE -15.0f / 180 * PI // ��X����ת�Ƕ���СΪ-15��
#define HEXAPOD_MAX_X_ROTATE 15.0f / 180 * PI  // ��X����ת�Ƕ����Ϊ 15��
#define HEXAPOD_MIN_Y_ROTATE -10.0f / 180 * PI // ��X����ת�Ƕ���СΪ-10��
#define HEXAPOD_MAX_Y_ROTATE 10.0f / 180 * PI  // ��X����ת�Ƕ����Ϊ 10��
#define HEXAPOD_MIN_Z_ROTATE -25.0f / 180 * PI // ��X����ת�Ƕ���СΪ-25��
#define HEXAPOD_MAX_Z_ROTATE 25.0f / 180 * PI  // ��X����ת�Ƕ����Ϊ 25��

/*PID*/
#define MPU_X_PID_KP 0.015f
#define MPU_X_PID_KI 0.0f
#define MPU_X_PID_KD 0.5f

#define MPU_Y_PID_KP 0.015f
#define MPU_Y_PID_KI 0.0f
#define MPU_Y_PID_KD 0.5f


/*FOFһ�׵�ͨ�˲�����*/
#define VELOCITY_FOF_K 0.3f
#define BODY_POS_FOF_K 0.1f
#define BODY_ANGLE_FOF_K 0.1f


#define ROTATE_BODY_ANGLE_SENSI 0.00002f//���ƽǶ�������
#define ROTATE_BODY_POS_SENSI 0.006f//����λ��������


typedef enum
{
    HEXAPOD_MOVE,
    HEXAPOD_BODY_ANGEL_CONTROL,
    HEXAPOD_BODY_POS_CONTROL,
} Hexapod_mode_e;

typedef enum
{
    MPU_ON,
    MPU_OFF,
}MPU_SW_e;

typedef enum
{
    ARM_ON,
    ARM_OFF,
}ARM_SW_e;

class Hexapod
{
public:
    Leg legs[6];                 // ������
    Arm arm;    //3���е��
    Velocity velocity;           // �������ٶ�
    Hexapod_mode_e mode; // ������ģʽ
    MPU_SW_e mpu_sw;    //�Ƿ��������ǿ���
    ARM_SW_e arm_sw; //�Ƿ���ƻ�е��
    Position3 body_pos;     //����λ��
    Position3 arm_end_pos;
    float arm_grip_pawl_angle;  //��צ�Ƕ�  
    Position3 arm_end_last_pos; //��һ��λ��
    PID mpu_pid_x; //x��pid
    PID mpu_pid_y; //y��pid
    First_order_filter velocity_fof[3];
    First_order_filter body_pos_fof[3];
    First_order_filter body_angle_fof[3];
    bool mpu_flag;
    void Init();          
    void velocity_cal(const RC_remote_data_t &remote_data);
    void body_position_cal(const RC_remote_data_t &remote_data);
    void mode_select(const RC_remote_data_t &remote_data);
    void body_pos_zero(const RC_remote_data_t &remote_data);
    void arm_position_cal(const RC_remote_data_t &remote_data);
    void move(uint32_t round_time);
};


#endif
