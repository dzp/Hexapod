// File:          hexapod_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include "APP/gait_prg.h"
#include "simply_spider_controller.h"
#include "main.h"
#include "iostream"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

uint32_t LegControl_round; // 控制回合
Gait_prg gait_prg;	  // 步态规划
Hexapod hexapod;		   // 机器人结构体
Robot* robot;

int main(int argc, char** argv)
{
    // create the Robot instance.
    robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor *motor = robot->getMotor("motorname");
    //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
    //  ds->enable(timeStep);
    gait_prg.Init();
    hexapod.Init();
    hexapod.velocity.omega = 20;
    hexapod.velocity.Vy = 0;
    hexapod.velocity.Vx = 0;
    hexapod.body_angle.x = PI / 9;
    hexapod.body_angle.y = PI / 12;
    hexapod.body_angle.z = 0;
    gait_prg.set_body_rotate_angle(hexapod.body_angle);
    //test
    //Motor* motor_1=robot->getMotor("servo1_3");
    //motor_1->setPosition(PI/4);
    //motor_1->setVelocity(1);

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1)
    {
        LegControl_round = (++LegControl_round) % N_POINTS;
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();

        gait_prg.set_velocity(hexapod.velocity);
        gait_prg.CEN_and_pace_cal();
        gait_prg.gait_proggraming();
        hexapod.update_leg_theta();
        hexapod.move(1.0 / TPS * (MAX_PACE_TIME/1000.));
        // Process sensor data here.
        //std::cout << 1.0 / N_POINTS << std::endl;

        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
    };

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}

#include <iostream>
using namespace std;

void Hexapod::Init()
{
    if (robot == NULL)
        return;
    legs[0] = Leg(robot->getMotor("servo1_1"), robot->getMotor("servo1_2"), robot->getMotor("servo1_3"),
        robot->getPositionSensor("servo1_1_sensor"), robot->getPositionSensor("servo1_2_sensor"), robot->getPositionSensor("servo1_3_sensor"));
    legs[1] = Leg(robot->getMotor("servo2_1"), robot->getMotor("servo2_2"), robot->getMotor("servo2_3"),
        robot->getPositionSensor("servo2_1_sensor"), robot->getPositionSensor("servo2_2_sensor"), robot->getPositionSensor("servo2_3_sensor"));
    legs[2] = Leg(robot->getMotor("servo3_1"), robot->getMotor("servo3_2"), robot->getMotor("servo3_3"),
        robot->getPositionSensor("servo3_1_sensor"), robot->getPositionSensor("servo3_2_sensor"), robot->getPositionSensor("servo3_3_sensor"));
    legs[3] = Leg(robot->getMotor("servo6_1"), robot->getMotor("servo6_2"), robot->getMotor("servo6_3"),
        robot->getPositionSensor("servo6_1_sensor"), robot->getPositionSensor("servo6_2_sensor"), robot->getPositionSensor("servo6_3_sensor"));
    legs[4] = Leg(robot->getMotor("servo5_1"), robot->getMotor("servo5_2"), robot->getMotor("servo5_3"),
        robot->getPositionSensor("servo5_1_sensor"), robot->getPositionSensor("servo5_2_sensor"), robot->getPositionSensor("servo5_3_sensor"));
    legs[5] = Leg(robot->getMotor("servo4_1"), robot->getMotor("servo4_2"), robot->getMotor("servo4_3"),
        robot->getPositionSensor("servo4_1_sensor"), robot->getPositionSensor("servo4_2_sensor"), robot->getPositionSensor("servo4_3_sensor"));
    leg_offset[0] = Thetas(PI / 3, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
    leg_offset[1] = Thetas(0.0f, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
    leg_offset[2] = Thetas(-PI / 3, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
    leg_offset[3] = Thetas(2 * PI / 3, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
    leg_offset[4] = Thetas(PI, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
    leg_offset[5] = Thetas(-2 * PI / 3, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
}


void Hexapod::move(double round_time)
{
    Thetas theta_temp;
    for (int i = 0;i < 6;i++)
    {
        theta_temp = (gait_prg.actions[i].thetas[LegControl_round]) - leg_offset[i];
        theta_temp.angle[1] = -theta_temp.angle[1];
        if (theta_temp.angle[0] <= -2.0f / 3.0f * PI)
        {
            theta_temp.angle[0] += 2 * PI;
        }
        legs[i].set_thetas(theta_temp);
        legs[i].set_move_time(round_time);
        legs[i].move();
    }

}

void Hexapod::update_leg_theta()
{
    for (size_t i = 0; i < 6; i++)
    {
        legs[i].read_angle();
    }
}





