#include <stdlib.h>
#include "pid.h"
#include "stm32f4xx.h"
#include "math.h"

#define ABS(x)	((x>0)? (x): -(x))


/**
 *
 * @param pid
 * @param id
 * @param maxout
 * @param deadband
 * @param intergralLimit 设置为0为不启用
 * @param integralSeparation 设置为0为不启用
 * @param differentiationLimit 设置为0为不启用
 * @param target
 * @param kp
 * @param ki
 * @param kd
 */
void pid_param_init(PID_TypeDef * pid,
                    float maxout,
                    float deadband,
                    float intergralLimit,
                    float integralSeparation,
                    float differentiationLimit,
                    float  target,
                    float 	kp,
                    float 	ki,
                    float 	kd)
{
    pid->MaxOutput = maxout;
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergralLimit;
    pid->target = target;
    pid->IntegralSeparation = integralSeparation;
    pid->DifferentiationLimit = differentiationLimit;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output = 0;
}

/**
 * @brief 中途更改参数设定
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 */
void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief pid计算
 * @param pid
 * @param measure
 * @param target
 * @return
 */
float pid_calculate(PID_TypeDef* pid, float measure, float target)
{
    //uint32_t time,lasttime;

    //pid->lasttime = pid->thistime;
    ///pid->thistime = HAL_GetTick();
    //pid->dtime = pid->thistime-pid->lasttime;

    pid->measure = measure;
    pid->target = target;

    pid->last_err = pid->err;
    pid->last_output = pid->output;

    pid->err = pid->target - pid->measure;

    //是否进入死区，误差小于pid->DeadBand则认为没有误差
    if((ABS(pid->err) > pid->DeadBand))
    {
        pid->pout =  pid->err;
        pid->dout =  pid->err - pid->last_err;

        //积分限幅
        if(ABS(pid->err) < pid->IntegralSeparation || pid->IntegralSeparation == 0)
        {
            pid->iout += pid->err;
            if(pid->IntegralLimit != 0)
            {
                //积分是否超出限制
                if(pid->iout > pid->IntegralLimit)
                    pid->iout = pid->IntegralLimit;
                else if(pid->iout < - pid->IntegralLimit)
                    pid->iout = - pid->IntegralLimit;
            }
        }
        else
            pid->iout = 0;

        if(pid->DifferentiationLimit != 0)  //开启微分限幅
        {
            if(pid->dout > pid->DifferentiationLimit)
                pid->dout = pid->DifferentiationLimit;
            else if(pid->dout < -pid->DifferentiationLimit)
                pid->dout = -pid->DifferentiationLimit;
        }

        //pid输出和
        pid->output = pid->kp * pid->pout + pid->ki * pid->iout + pid->kd * pid->dout;
        //pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波

        if(pid->output > pid->MaxOutput)
            pid->output = pid->MaxOutput;
        else if(pid->output < -(pid->MaxOutput))
            pid->output = -(pid->MaxOutput);


        return pid->output;
    }
    else
        return 0;
}

float pid_output_limit_calculate(PID_TypeDef* pid, float measure, float target, float max)
{
    float output;
    output = pid_calculate(pid, measure, target);
    if(output > max)
        output = max;
    else if(output < -max)
        output = -max;
    return output;
}

void pid_set_target(PID_TypeDef* pid, float target)
{
    pid->target = target;
}
