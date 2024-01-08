#ifndef LEG_H
#define LEG_H

#include "my_math.h"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

using namespace webots;

class Leg
{
private:
	Motor* motor[3];  //���
	PositionSensor* pos_sensor[3]; //λ�ô�����
	Thetas set_theta;       //�趨�Ƕ�
	Thetas theta;			//��ǰ�Ƕ�
	Thetas last_set_theta;      //��һ�νǶ�
	double move_time; //�ƶ�����һ��������Ҫ��ʱ��
public:
	Leg(Motor* motor_1, Motor* motor_2, Motor* motor_3, PositionSensor* pos_sensor_1=NULL, PositionSensor* pos_sensor_2=NULL, PositionSensor* pos_sensor_3=NULL); // ���캯��,��ȡ���id
	Leg();						// �޲ι���
	void set_thetas(Thetas thetas); // ���û�е�ȵĽǶ�
	void set_move_time(double move_time);	// ���û�е���ƶ���ʱ��
	void read_angle();	// ��ȡ����Ƕ�
	void move(); //��·
};

#endif
