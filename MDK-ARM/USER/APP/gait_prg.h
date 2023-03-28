#ifndef GAIT_PRG_H
#define GAIT_PRG_H

#include "my_math.h"

#define LEG_LEN1 53.f  // 腿部第一连杆长度（单位mm）
#define LEG_LEN2 80.f  // 腿部第二连杆长度（单位mm）
#define LEG_LEN3 144.f // 腿部第三连杆长度（单位mm）

#define CHASSIS_LEN 162.2f        // 底盘长度（y轴方向）
#define CHASSIS_WIDTH 161.5f      // 底盘宽度（x轴方向）
#define CHASSIS_FRONT_WIDTH 93.3f // 底盘前端宽度（x轴方向）

#define N_POINTS 20                       // 半径和点的数量
#define THETA_STAND_2 40.0f / 180.0f * PI // 机械腿站立时最后两个关节的角度
#define THETA_STAND_3 -110.0f / 180.0f * PI

#define K_CEN 1 // 用于确定圆心模长的系数
#define KR_1 1  //%用于计算步伐大小的系数
#define KR_2 1  //%用于计算步伐大小的系数

class Velocity
{
public:
    float Vx;    // x轴速度
    float Vy;    // y轴速度
    float omega; // 角速度
};

class Gait_prg
{
public:
    Gait_prg(); // 初始化
    void CEN_and_pace_cal();
    void gait_proggraming();
};

typedef struct
{
    Thetas thetas[N_POINTS];
} action;

#endif
