#include "leg.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "main.h"

Leg::Leg(UART_HandleTypeDef* huart)
{
	this->huart = huart;
	servos[0] = Servo(1);
	servos[1] = Servo(2);
	servos[2] = Servo(3);
}

void Leg::set_thetas(Thetas theta)
{
	this->theta.angle[0] = theta.angle[0];
	this->theta.angle[1] = theta.angle[1];
	this->theta.angle[2] = theta.angle[2];
	this->servos[0].set_angle(theta.angle[0]);
	this->servos[1].set_angle(theta.angle[1]);
	this->servos[2].set_angle(theta.angle[2]);
}

void Leg::set_time(uint16_t move_time)
{
	servos[0].set_time(move_time);
	servos[1].set_time(move_time);
	servos[2].set_time(move_time);
}

void Leg::move()
{
	this->servos[0].move(this->huart);
	this->servos[1].move(this->huart);
	this->servos[2].move(this->huart);
}

void Leg::move_wait()
{
	this->servos[0].move_wait(this->huart);
	this->servos[1].move_wait(this->huart);
	this->servos[2].move_wait(this->huart);
}


