#ifndef GAIT_PRG_H
#define GAIT_PRG_H

#include "my_math.h"
#include "../main.h"

#define LEG_LEN1 40.  // 腿部第一连杆长度（单位mm）
#define LEG_LEN2 70.  // 腿部第二连杆长度（单位mm）
#define LEG_LEN3 140. // 腿部第三连杆长度（单位mm）

#define CHASSIS_LEN 173.2        // 底盘长度（y轴方向）
#define CHASSIS_WIDTH 200.      // 底盘宽度（x轴方向）
#define CHASSIS_FRONT_WIDTH 100.0 // 底盘前端宽度（x轴方向）


#define THETA_STAND_2 40.0 / 180.0 * PI // 机械腿站立时最后两个关节的角度
#define THETA_STAND_3 -110.0 / 180.0 * PI

#define K_CEN 500.0     // 用于确定圆心模长的系数
#define KR_1 1           //%用于计算步伐大小的系数
#define KR_2 1.0        //%用于计算步伐大小的系数
#define Kz 0.3 //控制抬腿高度
#define MAX_R_PACE 100.0 // 最大步伐半径
#define MAX_PACE_TIME 5000 //最大步伐时间ms
#define MAX_SPEED 0.3 * 660

#define N_POINTS (TPS * MAX_PACE_TIME/1000)                      // 点的数量（必须是偶数）

#define MIN_Z_PACE 15.0

#define MAX_JOINT2_RAD PI / 2.0f         // 第2关节最大弧度
#define MIN_JOINT2_RAD -0.1 * PI         // 第2关节最小弧度
#define MAX_JOINT3_RAD -(1.0 / 6.0) * PI // 第3关节最大弧度
#define MIN_JOINT3_RAD -(7.0 / 9.0) * PI // 第3关节最小弧度

#define K_W (1.0/56.56854) // 1/|B|_max

class Velocity
{
public:
    double Vx;    // x轴速度
    double Vy;    // y轴速度
    double omega; // 角速度
};

typedef struct
{
    Thetas thetas[N_POINTS];
} action;

class Gait_prg
{
private:
    uint32_t pace_time;       // 走一步花费的时间
    Position3 Pws[6];         // 机械腿末端站立状态下相对于起始端的位置
    Position3 Pws_default[6]; // 默认情况下机械腿末端站立状态下相对于起始端的位置
    Position3 P_legs[6];      // 各个机械腿起始端相对于机器人中心的坐标
    Position3 CEN;            // 绕圆心的坐标
    double R_pace;             // 步伐大小（单位mm）
    Position3 body_pos;       //机身位置
    Velocity velocity;       //机身速度
    Position3 hexapod_rotate(Position3 &point, uint32_t index);
    Position3 rotate_angle; // 机体旋转角度
    double move_point();
public:
    action actions[6];
    void Init(); // 初始化
    void CEN_and_pace_cal();
    void gait_proggraming();
    uint32_t get_pace_time();
    void set_height(double height);
    void set_body_rotate_angle(Position3 &rotate_angle);
    void set_body_position(Position3 &body_pos);
    void set_velocity(Velocity &velocity);
};

#endif
