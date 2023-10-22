//
// Created by 1 on 2023-10-16.
//

#ifndef MASTER_RR_OMNI_CLASSIS_TASK_H
#define MASTER_RR_OMNI_CLASSIS_TASK_H

#include "can_rec.h"
#include "stm32f4xx_it.h"
#include "math.h"
#include "stdio.h"

#define Pi 3.1415926535f
#define Distance 15.0f
#define M1_ID 0x201
#define M2_ID 0x202
#define M3_ID 0x203
#define M4_ID 0x204
#define X_Start 651.2f
#define Y_Start 76.0f
#define R_Decrease 70.0f
#define Speed_Increase 0.5f

extern int8_t flag;

typedef struct {
    float X_Speed; //底盘x方向速度目标值
    float Y_Speed; //y方向
    float R_Speed; //自转线速度
    float angle;
    float tan_angle;
}Target;

typedef struct {

    float v_front;
    float v_right;
    float v_left;
    float v_back;

}OmniWheel;

typedef struct {

    float v_x;
    float v_y;
    float v_r;
    float last_v_x;
    float last_v_y;
    float last_v_r;
    float Done_flag;

}FinalVelocity;

extern OmniWheel FourWheel;
extern Target FourTarget;
extern FinalVelocity Fourv;

void Classis_Slove(FinalVelocity *target,OmniWheel *Wheel);
void Get_Target(Target *target,float x,float y,float r,float angle);
void Speed_Change(Target *target,FinalVelocity *Fv);
float Run_Point(PID_TypeDef *PID_x,PID_TypeDef *PID_y,PID_TypeDef *PID_r,float x_change,float y_change,float r_change,float angle_change,int straight_flag);
void Run_RouteCube(PID_TypeDef *PID_x,PID_TypeDef *PID_y,PID_TypeDef *PID_r,float x_change,float y_change,float r_change,float angle_change);
void Disc_Get_Target(Target *target,float x,float y,float r,float angle);
float Angle_Change(float angle);
float FanAngle_Change(float angle);
float Row_Change(Target *target,float x,float y,float r,float angle);
void Revolove(float r_change);
void XYR_Speed_Charge(PID_TypeDef *PID_x,PID_TypeDef *PID_y,PID_TypeDef *PID_r);

#endif //MASTER_RR_OMNI_CLASSIS_TASK_H
