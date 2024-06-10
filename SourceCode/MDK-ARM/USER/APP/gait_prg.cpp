#include "gait_prg.h"
#include "cmsis_os.h"
#include <cmath>
#include "remote.h"
#include "my_math.h"
#include "arm_math.h"
using namespace std;

// 全局变量
extern uint32_t LegControl_round; // 控制回合

// 函数
static Position3 fkine(Thetas thetas);
static Thetas ikine(Position3 &pos);

void Gait_prg::Init()
{
    // 计算机械腿相对于起始端的末端坐标
    Pws[0] = fkine(Thetas(PI / 4, THETA_STAND_2, THETA_STAND_3));
    Pws[1] = fkine(Thetas(0, THETA_STAND_2, THETA_STAND_3));
    Pws[2] = fkine(Thetas(-PI / 4, THETA_STAND_2, THETA_STAND_3));
    Pws[3] = fkine(Thetas(3 * PI / 4, THETA_STAND_2, THETA_STAND_3));
    Pws[4] = fkine(Thetas(PI, THETA_STAND_2, THETA_STAND_3));
    Pws[5] = fkine(Thetas(5 * PI / 4, THETA_STAND_2, THETA_STAND_3));
    // 默认站立坐标，这里copy一份
    memcpy(Pws_default, Pws, sizeof(Position3) * 6);
    // 计算各个机械腿起始端相对于机器人中心的坐标
    P_legs[0] = Position3(CHASSIS_FRONT_WIDTH / 2, CHASSIS_LEN / 2, 0);
    P_legs[1] = Position3(CHASSIS_WIDTH / 2, 0, 0);
    P_legs[2] = Position3(CHASSIS_FRONT_WIDTH / 2, -CHASSIS_LEN / 2, 0);
    P_legs[3] = Position3(-CHASSIS_FRONT_WIDTH / 2, CHASSIS_LEN / 2, 0);
    P_legs[4] = Position3(-CHASSIS_WIDTH / 2, 0, 0);
    P_legs[5] = Position3(-CHASSIS_FRONT_WIDTH / 2, -CHASSIS_LEN / 2, 0);
}

/*
 * 正运动解算
 */
static Position3 fkine(Thetas thetas)
{
    Position3 position3(cos(thetas.angle[0]) * (LEG_LEN1 + LEG_LEN3 * cos(thetas.angle[1] + thetas.angle[2]) + LEG_LEN2 * cos(thetas.angle[1])),
                        sin(thetas.angle[0]) * (LEG_LEN1 + LEG_LEN3 * cos(thetas.angle[1] + thetas.angle[2]) + LEG_LEN2 * cos(thetas.angle[1])),
                        LEG_LEN3 * sin(thetas.angle[1] + thetas.angle[2]) + LEG_LEN2 * sin(thetas.angle[1]));

    return position3;
}

/*
 * 逆运动解算
 */
static Thetas ikine(Position3 &pos)
{
    static Position3 pos1;
    static float R, Lr, alpha_r, alpha1, alpha2;
    pos1 = pos;
    R = sqrt(pow(pos1.x, 2) + pow(pos1.y, 2));
    Lr = sqrt(pow(pos1.z, 2) + pow((R - LEG_LEN1), 2));
    alpha_r = atan(-pos1.z / (R - LEG_LEN1));
    alpha1 = acos((pow(LEG_LEN2, 2) + pow(Lr, 2) - pow(LEG_LEN3, 2)) / (2 * Lr * LEG_LEN2));
    alpha2 = acos((pow(Lr, 2) + pow(LEG_LEN3, 2) - pow(LEG_LEN2, 2)) / (2 * Lr * LEG_LEN3));
    Thetas thetas(atan2(pos1.y, pos1.x), alpha1 - alpha_r, -(alpha1 + alpha2));
    value_limit(thetas.angle[1], MIN_JOINT2_RAD, MAX_JOINT2_RAD);
    value_limit(thetas.angle[2], MIN_JOINT3_RAD, MAX_JOINT3_RAD);
    return thetas;
}

float Gait_prg::move_point()
{
    float fun,m_velocity;
    m_velocity = sqrt(pow(velocity.Vx,2)+pow(velocity.Vy,2));
    fun = (body_pos.x * velocity.Vx + body_pos.y * velocity.Vy)/(m_velocity)*K_W;
    return fun;
}




/*
 *@brief 设置机器人高度
 *@param height 机器人的高度
 */
void Gait_prg::set_height(float height)
{
    for (int i = 0; i < 6; i++)
    {
        Pws[i].z = Pws_default[i].z + height;
    }
}

/*
 *@brief 设置机器人身体位置
 *@param body_pos 机器人的身体位置
 */
void Gait_prg::set_body_position(Position3 &body_pos)
{
    this->body_pos = body_pos;
    for (int i = 0; i < 6; i++)
    {
        Pws[i] = Pws_default[i] - body_pos;
    }
}

/*
 *@brief 设置机器人速度
 *@param velocity 机器人速度
 */
void Gait_prg::set_velocity(Velocity &velocity)
{
    this->velocity = velocity;
}

/*
 * 计算圆心位置和步伐大小已及步伐执行时间
 */
void Gait_prg::CEN_and_pace_cal()
{
    // 数据预处理，避免出现0
    if (velocity.Vx == 0)
        velocity.Vx += 0.001f;
    if (velocity.Vy == 0)
        velocity.Vy += 0.001f;
    if (velocity.omega == 0)
        velocity.omega += 0.001f;

    if (velocity.omega < 0)
    {
        velocity.Vx = -velocity.Vx;
        velocity.Vy = -velocity.Vy;
    }

    // 计算圆心模长
    float module_CEN = K_CEN / velocity.omega * sqrt(pow(velocity.Vx, 2) + pow(velocity.Vy, 2));
    Velocity velocity_s; // 旋转90度
    velocity_s.Vx = -velocity.Vy;
    velocity_s.Vy = velocity.Vx;
    if (velocity_s.Vx >= 0)
        CEN.x = sqrt(pow(module_CEN, 2) / (1 + pow(velocity.Vx, 2) / pow(velocity.Vy, 2)));
    else
        CEN.x = -sqrt(pow(module_CEN, 2) / (1 + pow(velocity.Vx, 2) / pow(velocity.Vy, 2)));
    // 计算步伐大小
    float module_speed = pow(abs(pow(velocity.Vx, 3) + pow(velocity.Vy, 3) + pow(velocity.omega, 3)), 1.0f / 3);
    if (module_speed > MAX_SPEED)
        module_speed = MAX_SPEED; // 限制速度
    R_pace = KR_2 * module_speed;
    // 计算步伐时间
    if (R_pace > MAX_R_PACE)
        this->pace_time = 1000 / (R_pace / MAX_R_PACE); // 若超过最大步伐大小则缩小步伐时间
    else
        this->pace_time = 1000; // 若小于最大步伐大小则固定步伐时间
    if (R_pace > MAX_R_PACE)
        R_pace = MAX_R_PACE; // 限制步伐大小
    CEN.y = -CEN.x * velocity.Vx / velocity.Vy;
}

/*
 * 步态规划
 */
void Gait_prg::gait_proggraming()
{
    Position3 Vec_CEN2leg_ends[6];    // 圆心到腿部末端的向量
    static float angle_off[6];        // 圆心与机械腿末端的夹角
    static float norm_CEN2legs[6];    // 圆心到机械腿末端的模长
    static float Rp_ratios[6];        // 各个机械腿步态规划的大小比例
    Position3 Vec_Leg_Start2CEN_s[6]; // 腿部起始端到圆心起始端的向量
    for (int i = 0; i < 6; i++)
    {
        Vec_CEN2leg_ends[i] = Pws[i] + P_legs[i] - CEN;                                         // 计算圆心到每个腿部末端的向量
        angle_off[i] = atan2(Vec_CEN2leg_ends[i].y, Vec_CEN2leg_ends[i].x);                     // 计算圆心与机械腿末端的夹角
        norm_CEN2legs[i] = sqrt(pow(Vec_CEN2leg_ends[i].x, 2) + pow(Vec_CEN2leg_ends[i].y, 2)); // 计算圆心与机械腿末端的模长
        Vec_Leg_Start2CEN_s[i] = CEN - P_legs[i];                                               // 计算腿部起始端到圆心起始端的向量
    }
    float max_norm_CEN2legs = 0;
    for (int i = 0; i < 6; i++)
        if (norm_CEN2legs[i] > max_norm_CEN2legs)
            max_norm_CEN2legs = norm_CEN2legs[i]; // 选出最大模长

    static float R_paces[6]; // 各个机械腿的步长
    for (int i = 0; i < 6; i++)
    {
        Rp_ratios[i] = norm_CEN2legs[i] / max_norm_CEN2legs; // 计算各个机械腿步态规划的大小比例
        R_paces[i] = Rp_ratios[i] * R_pace;                  // 计算各个机械腿步态的大小
    }
    float d_theta = 2 * R_paces[0] / norm_CEN2legs[0]; // 计算机械腿走一步绕圆心拐的角度，随便拿一组数据来算就行
    float step_size = d_theta / (N_POINTS / 2);

    /*********先对腿1，3，5做步态规划***********/
    static float angle_t;   // 用于计算该点的角度
    static float y_temp;    // 用于计算z轴高度的临时变量
    static Position3 point; // 用于存储末端坐标点
    for (int i = 0; i < 5; i += 2)
    {
        if (LegControl_round < N_POINTS / 2) // 0-9, 画下半圆
        {
            angle_t = angle_off[i] + d_theta / 2 - step_size * LegControl_round;  // 计算这个点的角度
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t); // 计算这个点的x轴坐标(相对于机械腿起始端)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t); // 计算这个点的y轴坐标(相对于机械腿起始端)
            point.z = Pws[i].z;                                                   // 动作前半部分贴着地面，故取站立时的z轴坐标
        }
        else // 10-19，画上半圆
        {
            angle_t = angle_off[i] - d_theta / 2 + step_size * (LegControl_round - N_POINTS / 2); // 计算这个点的角度
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t);                 // 计算这个点的x轴坐标(相对于机械腿起始端)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t);                 // 计算这个点的y轴坐标(相对于机械腿起始端)
            y_temp = -R_pace + (LegControl_round - N_POINTS / 2) * (R_pace * 4 / N_POINTS);
            // 根据圆的大小缩小z轴高度,并迁移坐标系到机械腿末端,因为站立时z轴都是一样的，所以随便用一个Pw就行
            if (R_pace > 0.5f && R_pace < MIN_Z_PACE)
                point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] * 3 + Pws[i].z;
            else
                point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] + Pws[i].z;
        }
        actions[i].thetas[LegControl_round] = ikine(point);
    }

    /*********对腿2，4，6做步态规划***********/
    for (int i = 1; i <= 5; i += 2)
    {
        if (LegControl_round < N_POINTS / 2) // 0-9, 画上半圆
        {
            angle_t = angle_off[i] - d_theta / 2 + step_size * LegControl_round;  // 计算这个点的角度
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t); // 计算这个点的x轴坐标(相对于机械腿起始端)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t); // 计算这个点的y轴坐标(相对于机械腿起始端)
            y_temp = -R_pace + LegControl_round * (R_pace * 4 / N_POINTS);
            // 根据圆的大小缩小z轴高度,并迁移坐标系到机械腿末端,因为站立时z轴都是一样的，所以随便用一个Pw就行
            if (R_pace > 0.5f && R_pace < MIN_Z_PACE)
                point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] * 3 + Pws[i].z;
            else
                point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] + Pws[i].z;
        }
        else // 10-19, 画下半圆
        {
            angle_t = angle_off[i] + d_theta / 2 - step_size * (LegControl_round - N_POINTS / 2); // 计算这个点的角度
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t);                 // 计算这个点的x轴坐标(相对于机械腿起始端)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t);                 // 计算这个点的y轴坐标(相对于机械腿起始端)
            point.z = Pws[i].z;
        }
        actions[i].thetas[LegControl_round] = ikine(point);
    }  
}

uint32_t Gait_prg::get_pace_time()
{
    return this->pace_time;
}
