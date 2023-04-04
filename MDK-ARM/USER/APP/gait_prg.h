#ifndef GAIT_PRG_H
#define GAIT_PRG_H

#include "my_math.h"
#include "main.h"

#define LEG_LEN1 53.f  // 腿部第一连杆长度（单位mm）
#define LEG_LEN2 80.f  // 腿部第二连杆长度（单位mm）
#define LEG_LEN3 144.f // 腿部第三连杆长度（单位mm）

#define CHASSIS_LEN 162.2f        // 底盘长度（y轴方向）
#define CHASSIS_WIDTH 161.5f      // 底盘宽度（x轴方向）
#define CHASSIS_FRONT_WIDTH 93.3f // 底盘前端宽度（x轴方向）

#define N_POINTS 40                       // 点的数量（必须是偶数）
#define THETA_STAND_2 40.0f / 180.0f * PI // 机械腿站立时最后两个关节的角度
#define THETA_STAND_3 -110.0f / 180.0f * PI

#define K_CEN 500.0f     // 用于确定圆心模长的系数
#define KR_1 1           //%用于计算步伐大小的系数
#define KR_2 1           //%用于计算步伐大小的系数
#define MAX_R_PACE 75.0f // 最大步伐半径
#define MAX_SPEED 0.2f * 660

#define MIN_Z_PACE 15.0f

class Velocity
{
public:
    float Vx;    // x轴速度
    float Vy;    // y轴速度
    float omega; // 角速度
};

typedef struct
{
    Thetas thetas[N_POINTS];
} action;

class Gait_prg
{
private:
    uint32_t pace_time;  // 走一步花费的时间
    Position3 Pws[6];    // 机械腿末端站立状态下相对于起始端的位置
    Position3 Pws_default[6]; //默认情况下机械腿末端站立状态下相对于起始端的位置
    Position3 P_legs[6]; // 各个机械腿起始端相对于机器人中心的坐标
    Position3 CEN;       // 绕圆心的坐标
    float R_pace;        // 步伐大小（单位mm）
    Position3 hexapod_rotate(Position3 &point,uint32_t index);
    Position3 rotate_angle;  //机体旋转角度
public:
    action actions[6];
    void Init(); // 初始化
    void CEN_and_pace_cal(Velocity velocity);
    void gait_proggraming();
    uint32_t get_pace_time();
    void set_height(float height);
};

#endif
