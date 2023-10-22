#ifndef __PID_H
#define __PID_H

//#include "sys.h"

#include <stdint-gcc.h>

typedef struct
{
    float target;				//目标值

    float kp;
    float ki;
    float kd;

    float measure;				//测量值
    float err;					//误差
    float last_err;      		//上次误差

    float DeadBand;
    float pout;
    float iout;
    float dout;

    float output;				//本次输出
    float last_output;			//上次输出

    float MaxOutput;			//输出限幅
    float IntegralLimit;		//积分限幅
    float IntegralSeparation;   //积分分离

    float DifferentiationLimit;

    //uint32_t thistime;
    //uint32_t lasttime;
    //uint8_t dtime;

}PID_TypeDef;

//pid全局变量



void pid_param_init(PID_TypeDef * pid,
                    float maxout,
                    float deadband,
                    float intergralLimit,
                    float integralSeparation,
                    float differentiationLimit,
                    float  target,
                    float 	kp,
                    float 	ki,
                    float 	kd);

void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd);

float pid_calculate(PID_TypeDef* pid, float measure, float target);

float pid_output_limit_calculate(PID_TypeDef* pid, float measure, float target, float max);

void pid_set_target(PID_TypeDef* pid, float target);

#endif

