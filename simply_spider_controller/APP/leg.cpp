#include "leg.h"
#include "stdlib.h"
#include "../main.h"


uint8_t receive_buffer[100];

Leg::Leg(Motor* motor_1, Motor* motor_2, Motor* motor_3, PositionSensor* pos_sensor_1, PositionSensor* pos_sensor_2, PositionSensor* pos_sensor_3)
{
	motor[0] = motor_1;
	motor[1] = motor_2;
	motor[2] = motor_3;
	pos_sensor[0] = pos_sensor_1;
	pos_sensor[1] = pos_sensor_2;
	pos_sensor[2] = pos_sensor_3;
	move_time = 0;
	pos_sensor[0]->enable(1000/TPS);
	pos_sensor[1]->enable(1000 / TPS);
	pos_sensor[2]->enable(1000 / TPS);

}

//无参构造
Leg::Leg()
{
	motor[0] = motor[1] = motor[2] = NULL;
	pos_sensor[0] = pos_sensor[1] = pos_sensor[2] = NULL;
	move_time = 0;
}

void Leg::set_thetas(Thetas theta)
{
	//last_set_theta = set_theta;
	this->read_angle();
	last_set_theta = this->theta;
	//std::cout << this->theta.angle[1] << "\n";
	this->set_theta.angle[0] = theta.angle[0];
	this->set_theta.angle[1] = theta.angle[1];
	this->set_theta.angle[2] = theta.angle[2];
}

void Leg::set_move_time(double move_time)
{
	this->move_time = move_time;
}



void Leg::read_angle()
{
	theta.angle[0] = pos_sensor[0]->getValue();
	theta.angle[1] = pos_sensor[1]->getValue();
	theta.angle[2] = pos_sensor[2]->getValue();
	//std::cout <<"theta is " << theta.angle[0] << std::endl;
}

//debug
volatile double angle_debug;

#include <webots/Robot.hpp>
extern Robot* robot;
void Leg::move()
{
	Thetas theta_diff = set_theta - last_set_theta;
	double speed0, speed1, speed2;
	motor[0]->setPosition(set_theta.angle[0]);
	motor[1]->setPosition(set_theta.angle[1]);
	motor[2]->setPosition(set_theta.angle[2]);
	//motor[0]->setPosition(40./180.*PI);
	//motor[1]->setPosition(20.0 / 180.0 * PI);
	//motor[2]->setPosition(90.0 / 180.0 * PI);
	speed0 = abs(theta_diff.angle[0] / move_time);
	speed1 = abs(theta_diff.angle[1] / move_time);
	speed2 = abs(theta_diff.angle[2] / move_time);
	value_limit(speed0, 0, 3*PI);
	value_limit(speed1, 0, 3*PI);
	value_limit(speed2, 0, 3*PI);
	motor[0]->setVelocity(speed0);
	motor[1]->setVelocity(speed1);
	motor[2]->setVelocity(speed2);
	//motor[0]->setVelocity(2);
	//motor[1]->setVelocity(2);
	//motor[2]->setVelocity(2);
	if (motor[0] == robot->getMotor("servo4_1"))
	{
		angle_debug = set_theta.angle[0];
	}
}
