#include "my_math.h"

void Position3::zero()
{
    x = 0;
    y = 0;
    z = 0;
}

Position3 operator+(const Position3 &pos1, const Position3 &pos2)
{
    Position3 pos;
    pos.x = pos1.x + pos2.x;
    pos.y = pos1.y + pos2.y;
    pos.z = pos1.z + pos2.z;
    return pos;
}

Position3 operator-(const Position3 &pos1, const Position3 &pos2)
{
    Position3 pos;
    pos.x = pos1.x - pos2.x;
    pos.y = pos1.y - pos2.y;
    pos.z = pos1.z - pos2.z;
    return pos;
}

Thetas operator+(const Thetas &theta1, const Thetas &theta2)
{
    Thetas theta;
    theta.angle[0] = theta1.angle[0] + theta2.angle[0];
    theta.angle[1] = theta1.angle[1] + theta2.angle[1];
    theta.angle[2] = theta1.angle[2] + theta2.angle[2];
    return theta;
}

Thetas operator-(const Thetas &theta1, const Thetas &theta2)
{
    Thetas theta;
    theta.angle[0] = theta1.angle[0] - theta2.angle[0];
    theta.angle[1] = theta1.angle[1] - theta2.angle[1];
    theta.angle[2] = theta1.angle[2] - theta2.angle[2];
    return theta;
}

Thetas &Thetas::operator=(const float angles[3])
{
    this->angle[0] = angles[0];
    this->angle[1] = angles[1];
    this->angle[2] = angles[2];
    return *this;
}

Thetas::Thetas(const float angles[3])
{
    this->angle[0] = angles[0];
    this->angle[1] = angles[1];
    this->angle[2] = angles[2];
}

void value_limit(float &val, float min, float max)
{
    if (val > max)
        val = max;
    if (val < min)
        val = min;
}

PID::PID(float kp, float ki, float kd, Cir_mode cicir_moder)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->cir_mode = cir_mode;
}

float PID::cal(float current_val, float set_val)
{
    this->error[2] = this->error[1];
    this->error[1] = this->error[0];
    this->current_val = current_val;
    this->set_val = set_val;
    this->error[0] = set_val - current_val;

    switch (this->cir_mode)
    {
    case CIR_OFF:
        break;
    case CIR_ON:
        if (this->error[0] > PI / 2)
            this->error[0] -= PI;
        else if (this->error[0] < -PI / 2)
            this->error[0] += PI;
        break;
    default:
        break;
    }
    this->pout = this->kp * this->error[0]; // 比例项计算
    this->iout = this->ki * this->error[0]; // 积分项计算
    // 微分项计算
    this->Derror[2] = this->Derror[1];
    this->Derror[1] = this->Derror[0];
    this->Derror[0] = (this->error[0] - this->error[1]);
    this->dout = this->kd * this->Derror[0];
    this->out = this->pout + this->iout + this->dout;
    return this->out;
}

void PID::Init(float kp, float ki, float kd, Cir_mode cir_mode)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->cir_mode = cir_mode;
}

void First_order_filter::set_k_filter(float k_filter)
{
    this->k_filter = k_filter;
}

float First_order_filter::cal(float input)
{
    this->out = this->k_filter * input + (1 - k_filter) * this->last_input;
    this->last_input = this->out;
    return this->out;
}

void Diff_Limit::set_diff(float diff)
{
    this->diff = diff;
}

void Diff_Limit::set_fre(uint32_t fre)
{
    this->fre = fre;
}

float Diff_Limit::cal(float goal_value)
{
    float temp;
    this->goal_value = goal_value;
    if (goal_value > current_value)
    {
        temp = current_value + diff / fre;
        if(temp>goal_value)
        {
            current_value = goal_value;
            return current_value;
        }
        else current_value = temp;
    }
    else
    {
        temp = current_value - diff / fre;
        if(temp<goal_value)
        {
            current_value = goal_value;
            return current_value;
        }
        else current_value=temp;
    }
    return temp;
}
