#include <stdlib.h>
#include "pid.h"

PID_BASESTR PID_SPD_FRONT;
PID_BASESTR PID_ANGLE_FRONT;

PID_BASESTR PID_SPD_LEFT;
PID_BASESTR PID_ANGLE_LEFT;

PID_BASESTR PID_SPD_RIGHT;
PID_BASESTR PID_ANGLE_RIGHT;

PID_BASESTR PID_SPD_BACK;
PID_BASESTR PID_ANGLE_BACK;
/**
  * @brief  PID参数设置
  * @param  *PID 要设置的目标PID结构体地址
  */
void PID_Set(PID_BASESTR *PID, float kp, float ki, float kd,
             float integral_limit, float integral_separation_limit)
{
	PID->integral_limit = integral_limit;
	PID->integral_separation_limit = integral_separation_limit;
	PID->Kp = kp;
	PID->Ki = ki;
	PID->Kd = kd;
	PID->target = 0;
	PID->current = 0;
	PID->err = 0;
	PID->err_last = 0;
    PID->err_llast = 0;
	PID->integral = 0;
    PID->differentiation = 0;
	PID->p_out = 0;
	PID->i_out = 0;
	PID->d_out = 0;
	PID->PID_total_out = 0;
}


/**
  * @brief  位置式PID输出释放
  * @param  *PID 要计算输出的目标PID结构体
  * @param  target PID目标
  * @param  current PID输入（当前值）
  * @param  max_output PID输出限幅（绝对值）
  * @retval PID输出
  */
float PID_realise(PID_BASESTR *PID, float target, float current, float max_output)
{
    //赋值
	PID->target = target;
	PID->current = current;

    //误差计算
	PID->err = PID->target - PID->current;

	if(PID->integral_separation_limit != 0)		//如果积分分离值不为0，即开启了积分分离
	{
		if(abs(PID->err) > PID->integral_separation_limit)      //当偏差超过限度时，积分项置0
		{
			PID->integral = 0;
		}
		else
		{
            //积分累计
			PID->integral += PID->err;

            //在开启了积分分离的前提下，判断是否积分限幅
			if(PID->integral_limit != 0)
			{				       
				if(PID->integral < -PID->integral_limit)
						PID->integral = -PID->integral_limit;     //负值
				else if(PID->integral >	PID->integral_limit)
						PID->integral = PID->integral_limit;      //正
			}				
		}
	}
	else																														//δ�������ַ���
	{
		PID->integral += PID->err;	
		if(PID->integral_limit != 0)     //如果开启了积分限幅
		{		
			if(PID->integral < -PID->integral_limit)
					PID->integral = -PID->integral_limit;
			if(PID->integral > PID->integral_limit)
					PID->integral = PID->integral_limit;
		}
	}

    PID->differentiation = PID->err-PID->err_last;

	PID->p_out = PID->Kp * PID->err;
	PID->i_out = PID->Ki * PID->integral;
	PID->d_out = PID->Kd * PID->differentiation;
	
	PID->PID_total_out = PID->p_out + PID->i_out + PID->d_out;

	PID->err_last = PID->err;

    if(PID->PID_total_out > max_output)		//输出限幅 正向
        PID->PID_total_out = max_output;

    else if(PID->PID_total_out < -max_output)	//输出限幅 负向
        PID->PID_total_out = -max_output;

	return PID->PID_total_out;
}

