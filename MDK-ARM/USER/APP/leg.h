#ifndef LEG_H
#define LEG_H

#include "Servo.h"
#include "usart.h"
#include "my_math.h"

class Leg : public Servo_Broad_Cast
{
private:
	Servo servos[3];
	Thetas theta;

public:
	UART_HandleTypeDef *huart;
	Leg(UART_HandleTypeDef *huart); // 构造函数
	Leg(){};//无参构造
	void set_thetas(Thetas thetas);	// 设置机械腿的角度
	void set_time(uint16_t tims);	// 设置机械腿移动的时间
	void move();					// 机械腿移动命令
	void move_wait();				//设置机械腿角度，但需要等待开始命令才移动
	void move_start();              // 让机械腿开始运动
	
};

#endif
