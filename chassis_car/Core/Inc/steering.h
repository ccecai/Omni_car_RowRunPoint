//
// Created by Yanzhenbo on 2023/4/1.
//

#ifndef CHASSIS_ER_STEERING_H
#define CHASSIS_ER_STEERING_H
#include "main.h"

#define enable 1
#define disable 0

extern float angle_front_target, angle_left_target, angle_right_target,angle_back_target;
extern float angle_front_base, angle_left_base, angle_right_base;
extern float speed_front, speed_left, speed_right,speed_back;
void heartbeat_fresh();
uint8_t heartbeat_check();
#endif //CHASSIS_ER_STEERING_H
