#ifndef LEGCONTROL_TASK_H
#define LEGCONTROL_TASK_H

#define LEG_JOINT2_OFFSET PI/2
#define LEG_JOINT3_OFFSET -2 * PI / 9
#define HEXAPOD_MAX_HEIGHT 70.0f
#define HEXAPOD_MIN_HEIGHT -70.0f

#define HEXAPOD_MIN_X_ROTATE -15.0f/180*PI //绕X轴旋转角度最小为-15度
#define HEXAPOD_MAX_X_ROTATE 15.0f/180*PI  //绕X轴旋转角度最大为 15度
#define HEXAPOD_MIN_Y_ROTATE -10.0f/180*PI //绕X轴旋转角度最小为-10度
#define HEXAPOD_MAX_Y_ROTATE 10.0f/180*PI  //绕X轴旋转角度最大为 10度
#define HEXAPOD_MIN_Z_ROTATE -25.0f/180*PI //绕X轴旋转角度最小为-25度
#define HEXAPOD_MAX_Z_ROTATE 25.0f/180*PI  //绕X轴旋转角度最大为 25度
typedef enum
{
    HEXAPOD_MOVE,
    HEXAPOD_DANCE,
}Hexapod_mode_e;

#endif
