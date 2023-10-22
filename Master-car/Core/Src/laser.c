//
// Created by 25041 on 2023/6/20.
//
#include "laser.h"
#include "pid.h"
#include "steering_wheel.h"
#include "locale.h"

PID_TypeDef laser_loc_pid;

Data_Trans laser_data_01;
Data_Trans laser_data_23;
float laser_loc_data[2];

void Laser_Loc(void)
{
    //float angle_deifference;
    //以左边的激光距离为准
    laser_loc_data[distance] = laser_data_01.float_16[0];
    laser_loc_data[difference] = laser_data_01.float_16[0] - laser_data_01.float_16[1];

    //angle_deifference = atan2f(laser_loc_data[difference],22.0f);

    usart_printf("%f\n",laser_loc_data[difference]);
    pid_calculate(&laser_loc_pid,laser_loc_data[difference],0);

    SetRobotVel_XY(&rrWheel, 0.f, 0.f, (float)laser_loc_pid.output, locater.angle);

}


