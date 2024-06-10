#ifndef MY_MATH_H
#define MY_MATH_H

#define PI 3.14159f // Բ����

#include "main.h"

class Thetas
{
public:
    float angle[3];
    Thetas(float angle_0 = 0, float angle_1 = 0, float angle_2 = 0)
    {
        this->angle[0] = angle_0;
        this->angle[1] = angle_1;
        this->angle[2] = angle_2;
    }
    Thetas &operator=(const float angles[3]);
    Thetas(const float angles[3]);
};

Thetas operator+(const Thetas &theta1, const Thetas &theta2);
Thetas operator-(const Thetas &theta1, const Thetas &theta2);

class Position3
{
public:
    float x;
    float y;
    float z;
    Position3(float x = 0, float y = 0, float z = 0)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    void zero(); // ����
};
Position3 operator+(const Position3 &pos1, const Position3 &pos2);
Position3 operator-(const Position3 &pos1, const Position3 &pos2);

typedef enum
{
    CIR_ON,
    CIR_OFF,
} Cir_mode;

class PID
{
private:
    float kp, ki, kd;
    float pout, iout, dout, out;
    float Derror[3]; // ΢���� 0���� 1��һ�� 2���ϴ�
    float error[3];  // ����� 0���� 1��һ�� 2���ϴ�
    float current_val;
    float set_val;
    Cir_mode cir_mode;

public:
    PID(float kp, float ki, float kd, Cir_mode cir_mode);
    PID(){}; // �չ���
    float cal(float current_val, float set_val);
    void Init(float kp, float ki, float kd, Cir_mode cir_mode);
};

class First_order_filter
{
private:
    float last_input; // ��һ������
    float out;        // ���
    float k_filter;   // �˲�����
public:
    First_order_filter(float k_filter = 1) { this->k_filter = k_filter; };
    float cal(float input);
    void set_k_filter(float k_filter);
};

class Diff_Limit
{
private:
    float goal_value; //Ŀ��ֵ
    float current_value; //��ǰֵ
    float diff; //����
    uint32_t fre; //����Ƶ�ʣ�Hz��
public:
    Diff_Limit(float diff=1,uint32_t fre=100){this->diff = diff; this->fre = fre;};
    void set_diff(float diff);
    void set_fre(uint32_t fre);
    float cal(float goal_value);
};

void value_limit(float &val, float min, float max);

#endif
