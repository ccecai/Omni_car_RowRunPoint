//
// Created by 25041 on 2023/6/20.
//

#ifndef MASTER_ER_LASER_H
#define MASTER_ER_LASER_H
#include "main.h"

#define laser_01                0x151
#define laser_23                0x152

typedef union {
    uint32_t uint_32;
    uint8_t  uint_8[4];

    float float_16[2];
}Data_Trans;

enum {
    distance = 0,
    difference,
};

extern PID_TypeDef laser_loc_pid;
extern Data_Trans laser_data_01;
extern Data_Trans laser_data_23;
extern float laser_loc_data[2];

void Laser_Loc(void);

#endif //MASTER_ER_LASER_H
