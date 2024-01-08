#ifndef GAIT_PRG_H
#define GAIT_PRG_H

#include "my_math.h"
#include "../main.h"

#define LEG_LEN1 40.  // �Ȳ���һ���˳��ȣ���λmm��
#define LEG_LEN2 70.  // �Ȳ��ڶ����˳��ȣ���λmm��
#define LEG_LEN3 140. // �Ȳ��������˳��ȣ���λmm��

#define CHASSIS_LEN 173.2        // ���̳��ȣ�y�᷽��
#define CHASSIS_WIDTH 200.      // ���̿�ȣ�x�᷽��
#define CHASSIS_FRONT_WIDTH 100.0 // ����ǰ�˿�ȣ�x�᷽��


#define THETA_STAND_2 40.0 / 180.0 * PI // ��е��վ��ʱ��������ؽڵĽǶ�
#define THETA_STAND_3 -110.0 / 180.0 * PI

#define K_CEN 500.0     // ����ȷ��Բ��ģ����ϵ��
#define KR_1 1           //%���ڼ��㲽����С��ϵ��
#define KR_2 1.0        //%���ڼ��㲽����С��ϵ��
#define Kz 0.3 //����̧�ȸ߶�
#define MAX_R_PACE 100.0 // ��󲽷��뾶
#define MAX_PACE_TIME 5000 //��󲽷�ʱ��ms
#define MAX_SPEED 0.3 * 660

#define N_POINTS (TPS * MAX_PACE_TIME/1000)                      // ���������������ż����

#define MIN_Z_PACE 15.0

#define MAX_JOINT2_RAD PI / 2.0f         // ��2�ؽ���󻡶�
#define MIN_JOINT2_RAD -0.1 * PI         // ��2�ؽ���С����
#define MAX_JOINT3_RAD -(1.0 / 6.0) * PI // ��3�ؽ���󻡶�
#define MIN_JOINT3_RAD -(7.0 / 9.0) * PI // ��3�ؽ���С����

#define K_W (1.0/56.56854) // 1/|B|_max

class Velocity
{
public:
    double Vx;    // x���ٶ�
    double Vy;    // y���ٶ�
    double omega; // ���ٶ�
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
    double R_pace;             // ������С����λmm��
    Position3 body_pos;       //����λ��
    Velocity velocity;       //�����ٶ�
    Position3 hexapod_rotate(Position3 &point, uint32_t index);
    Position3 rotate_angle; // ������ת�Ƕ�
    double move_point();
public:
    action actions[6];
    void Init(); // ��ʼ��
    void CEN_and_pace_cal();
    void gait_proggraming();
    uint32_t get_pace_time();
    void set_height(double height);
    void set_body_rotate_angle(Position3 &rotate_angle);
    void set_body_position(Position3 &body_pos);
    void set_velocity(Velocity &velocity);
};

#endif
