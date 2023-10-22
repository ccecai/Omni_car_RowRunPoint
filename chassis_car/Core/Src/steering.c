//
// Created by Yanzhenbo on 2023/4/1.
//

#include "steering.h"

float angle_front_target = 0.0f, angle_left_target = 0.0f, angle_right_target = 0.0f,angle_back_target = 0.0f;

float angle_front_base, angle_left_base, angle_right_base,angle_back_base;

float speed_front = 0.0f, speed_left = 0.0f, speed_right = 0.0f,speed_back = 0.0f;

static uint8_t ticks = 0;

void heartbeat_fresh()
{
    ticks++;
    if(ticks > 10)
        ticks = 10;

}

uint8_t heartbeat_check()
{
    if(ticks > 0)
        ticks--;
    if(ticks == 0)
        return disable;
    else
        return enable;
}