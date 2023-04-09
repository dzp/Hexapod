#ifndef LEGCONTROL_TASK_H
#define LEGCONTROL_TASK_H

#include "my_math.h"
#include "leg.h"
#include "remote.h"
#include "gait_prg.h"

#define LEG_JOINT2_OFFSET PI / 2
#define LEG_JOINT3_OFFSET -2 * PI / 9

#define HEXAPOD_MIN_HEIGHT -70.0f
#define HEXAPOD_MAX_HEIGHT 70.0f
#define HEXAPOD_MIN_X -40.0f
#define HEXAPOD_MAX_X 40.0f
#define HEXAPOD_MIN_Y -40.0f
#define HEXAPOD_MAX_Y 40.0f


#define HEXAPOD_MIN_X_ROTATE -15.0f / 180 * PI // 绕X轴旋转角度最小为-15度
#define HEXAPOD_MAX_X_ROTATE 15.0f / 180 * PI  // 绕X轴旋转角度最大为 15度
#define HEXAPOD_MIN_Y_ROTATE -10.0f / 180 * PI // 绕X轴旋转角度最小为-10度
#define HEXAPOD_MAX_Y_ROTATE 10.0f / 180 * PI  // 绕X轴旋转角度最大为 10度
#define HEXAPOD_MIN_Z_ROTATE -25.0f / 180 * PI // 绕X轴旋转角度最小为-25度
#define HEXAPOD_MAX_Z_ROTATE 25.0f / 180 * PI  // 绕X轴旋转角度最大为 25度
typedef enum
{
    HEXAPOD_MOVE,
    HEXAPOD_BODY_ANGEL_CONTROL,
    HEXAPOD_BODY_POS_CONTROL,
} Hexapod_mode_e;

class Hexapod
{
public:
    Leg legs[6];                 // 六条腿
    Velocity velocity;           // 机器人速度
    Hexapod_mode_e mode; // 机器人模式
    Position3 body_pos;
    Position3 body_angle;
    void Init();          
    void velocity_cal(const RC_remote_data_t &remote_data);
    void body_position_cal(const RC_remote_data_t &remote_data);
    void body_angle_cal(const RC_remote_data_t &remote_data);
    void mode_select(const RC_remote_data_t &remote_data);
    void body_angle_and_pos_zero(const RC_remote_data_t &remote_data);
    void move(uint32_t round_time);
};

#endif
