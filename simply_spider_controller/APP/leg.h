#ifndef LEG_H
#define LEG_H

#include "my_math.h"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

using namespace webots;

class Leg
{
private:
	Motor* motor[3];  //电机
	PositionSensor* pos_sensor[3]; //位置传感器
	Thetas set_theta;       //设定角度
	Thetas theta;			//当前角度
	Thetas last_set_theta;      //上一次角度
	double move_time; //移动到下一个动作需要的时间
public:
	Leg(Motor* motor_1, Motor* motor_2, Motor* motor_3, PositionSensor* pos_sensor_1=NULL, PositionSensor* pos_sensor_2=NULL, PositionSensor* pos_sensor_3=NULL); // 构造函数,获取电机id
	Leg();						// 无参构造
	void set_thetas(Thetas thetas); // 设置机械腿的角度
	void set_move_time(double move_time);	// 设置机械腿移动的时间
	void read_angle();	// 读取舵机角度
	void move(); //跑路
};

#endif
