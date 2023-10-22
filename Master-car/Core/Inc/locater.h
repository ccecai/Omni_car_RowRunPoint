//
// Created by Yanzhenbo on 2023/3/1.
//
#ifndef __LOCATER__H
#define __LOCATER__H

#include "steering_wheel.h"
#include "main.h"
#include "stm32f4xx.h"

#define X1 260
#define X2 450
#define X3 550
#define X4 650
#define X5 750
#define X6 940

#define Y0 260
#define Y1 295
#define Y2 400
#define Y3 450
#define Y4 600

#define Y_ROUTE 350

#define RR_WIDE 40

typedef struct
{
    float pos_x;
    float pos_y;

    float pos_x_last;
    float pos_y_last;

    float speed_x;
    float speed_y;

    float angle;
    float angular_speed;

    float continuousAngle;
    float lastAngle;
    float LastlastAngle;
    int circleNum;

    float pos_x_base;
    float pos_y_base;

}locater_def;

typedef union
{
    uint8_t data_8[12];
    int32_t data_32[3];
    float data_f[3];
}loc_receive_union;

typedef struct
{
    float x;
    float y;
    float angle;
}pointStruct;

extern const pointStruct ROUTE_PIONT_RING_FRONT[3];


extern const pointStruct POINT_TAKE_RING[3];

extern const pointStruct ROUTE_COLUMN[15];



extern locater_def locater;
extern PID_TypeDef PID_route_angle,PID_route_x,PID_route_y;
extern pointStruct spot[4];
extern uint8_t spotNum;

extern pointStruct startSpot[4];
extern uint8_t startSpotNum;


void locater_data_rec(uint8_t *data, locater_def *loc);

pointStruct RouteInsertLinear(int32_t counting, int32_t total,pointStruct* startingPiont, pointStruct* destination);

pointStruct RouteInsertBezier_2(int32_t counting, int32_t total,pointStruct* startingPiont, pointStruct* destination, pointStruct* crolPiont);

pointStruct RouteInsertBezier_3(int32_t counting, int32_t total,pointStruct* startingPiont, pointStruct* destination, pointStruct* crolPiont_1, pointStruct* crolPiont_2);

float distanceCal(pointStruct point, float x, float y);

void RouteFormulate(pointStruct destination,float loc_x,float loc_y,float loc_angle);

uint8_t AutoRouteMotive(pointStruct* route,uint8_t* num);

float RouteAngleOptimize(float targetAngle,float presentAngle);


#endif //__LOCATER__H
