#ifndef __PID_H
#define __PID_H

#include <stdint-gcc.h>

/**
  * @brief  PID参数结构体定义
  */
typedef struct {
    float target; 													//定义设定值
    float current; 													//定义当前实际值
    float err;					 										//定义偏差值
    float err_last; 												//定义上一个偏差值
    float err_llast;                                              //定义更前一个的偏差，增量式pid使用
    float Kp,Ki,Kd; 												//定义比例、积分、微分系数
    float p_out;														//定义P项输出
    float i_out;														//定义I项输出
    float d_out;														//定义D项输出
    float PID_total_out; 										//定义PID执行输出
    float integral; 												//定义积分值
    float differentiation;
/*-----------------------积分限幅---------------------------*/
    float integral_limit;										    //定义积分限幅值

/*-----------------------积分分离---------------------------*/
    float integral_separation_limit;                    //定义积分分离限制幅度，当偏差大于这个限度时，不进行积分累加

}PID_BASESTR;

/*3508双环PID*/
extern PID_BASESTR PID_SPD_FRONT;
extern PID_BASESTR PID_ANGLE_FRONT;

extern PID_BASESTR PID_SPD_LEFT;
extern PID_BASESTR PID_ANGLE_LEFT;

extern PID_BASESTR PID_SPD_RIGHT;
extern PID_BASESTR PID_ANGLE_RIGHT;

extern PID_BASESTR PID_SPD_BACK;
extern PID_BASESTR PID_ANGLE_BACK;
//以下为函数声明
void PID_Set(PID_BASESTR *PID, float kp, float ki, float kd,
                  float integral_limit, float integral_separation_limit);
float PID_realise(PID_BASESTR *PID, float target, float current, float max_output);

#endif

