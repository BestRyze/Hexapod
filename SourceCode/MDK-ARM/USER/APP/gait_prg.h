#ifndef GAIT_PRG_H
#define GAIT_PRG_H

#include "my_math.h"
#include "main.h"

#define LEG_LEN1 53.f  // �Ȳ���һ���˳��ȣ���λmm��
#define LEG_LEN2 80.f  // �Ȳ��ڶ����˳��ȣ���λmm��
#define LEG_LEN3 144.f // �Ȳ��������˳��ȣ���λmm��

#define CHASSIS_LEN 162.2f        // ���̳��ȣ�y�᷽��
#define CHASSIS_WIDTH 161.5f      // ���̿�ȣ�x�᷽��
#define CHASSIS_FRONT_WIDTH 93.3f // ����ǰ�˿�ȣ�x�᷽��

#define N_POINTS 100                      // ���������������ż����
#define THETA_STAND_2 40.0f / 180.0f * PI // ��е��վ��ʱ��������ؽڵĽǶ�
#define THETA_STAND_3 -110.0f / 180.0f * PI

#define K_CEN 500.0f     // ����ȷ��Բ��ģ����ϵ��
#define KR_1 1           //%���ڼ��㲽����С��ϵ��
#define KR_2 1.0f        //%���ڼ��㲽����С��ϵ��
#define MAX_R_PACE 60.0f // ��󲽷��뾶
#define MAX_SPEED 0.3f * 660

#define MIN_Z_PACE 15.0f

#define MAX_JOINT2_RAD PI / 2.0f          // ��2�ؽ���󻡶�
#define MIN_JOINT2_RAD -0.1f * PI         // ��2�ؽ���С����
#define MAX_JOINT3_RAD -(1.0f / 6.0f) * PI // ��3�ؽ���󻡶�
#define MIN_JOINT3_RAD -(7.0f / 9.0f) * PI // ��3�ؽ���С����

#define K_W (1.0f/56.56854f) // 1/|B|_max

class Velocity
{
public:
    float Vx;    // x���ٶ�
    float Vy;    // y���ٶ�
    float omega; // ���ٶ�
};

typedef struct
{
    Thetas thetas[N_POINTS];
} action;

class Gait_prg
{
private:
    uint32_t pace_time;       // ��һ�����ѵ�ʱ��
    Position3 Pws[6];         // ��е��ĩ��վ��״̬���������ʼ�˵�λ��
    Position3 Pws_default[6]; // Ĭ������»�е��ĩ��վ��״̬���������ʼ�˵�λ��
    Position3 P_legs[6];      // ������е����ʼ������ڻ��������ĵ�����
    Position3 CEN;            // ��Բ�ĵ�����
    float R_pace;             // ������С����λmm��
    Position3 body_pos;       //����λ��
    Velocity velocity;       //�����ٶ�
    Position3 rotate_angle; // ������ת�Ƕ�
    float move_point();
public:
    action actions[6];
    void Init(); // ��ʼ��
    void CEN_and_pace_cal();
    void gait_proggraming();
    uint32_t get_pace_time();
    void set_height(float height);
    void set_body_position(Position3 &body_pos);
    void set_velocity(Velocity &velocity);
};

#endif
