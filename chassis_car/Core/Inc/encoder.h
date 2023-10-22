//
// Created by Yanzhenbo on 2023/4/9.
//

#ifndef CHASSIS_ER_ENCODER_H
#define CHASSIS_ER_ENCODER_H
#include "main.h"
#define encoder_left 0
#define encoder_front 1
#define encoder_right 2

#define zero_left 342.0f
#define zero_front 234.68f
#define zero_right 155.78f

typedef struct
{
    float angle_now;
    float angle_last;
    int32_t round;
    float total_angle;
    uint32_t msg_cnt;
}encoder_t;

#define ENCODER_DATA_SIZE 9

extern uint8_t encoder_data_u2[ENCODER_DATA_SIZE];
extern uint8_t encoder_data_u3[ENCODER_DATA_SIZE];
extern uint8_t encoder_data_u6[ENCODER_DATA_SIZE];

extern encoder_t angle_feedback[3];//ËùµÃ½Ç¶È

uint16_t GetModbusCRC16_Tab(uint8_t *data, uint32_t len);

#endif //CHASSIS_ER_ENCODER_H
