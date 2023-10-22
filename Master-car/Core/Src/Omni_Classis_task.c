//
// Created by WangCheng on 2023-10-16.
//
#include "Omni_Classis_task.h"

int count = 1;
int circle = 0;
int fu_flag = 0;
int half_circle = 0;
/*
 * 该函数对四轮万向轮模型进行解算
 */
void Classis_Slove(FinalVelocity *target,OmniWheel *Wheel)
{
    Wheel->v_front = Speed_Increase * target->v_y + Distance * Pi * target->v_r/R_Decrease; //正常解算结果来说，Speed_Increase与R_Decrease都应为一个定值，但是在实际中不对速度进行限制或者加速会对小车进行损伤或者无法达到目标，可在.h中修改宏定义
    Wheel->v_left = -Speed_Increase * target->v_x + Distance * Pi * target->v_r/R_Decrease;
    Wheel->v_right = Speed_Increase * target->v_x + Distance * Pi * target->v_r/R_Decrease;
    Wheel->v_back = -Speed_Increase * target->v_y + Distance * Pi * target->v_r/R_Decrease;
}

/*
 * 该函数给各个速度目标值赋值
 */
void Disc_Get_Target(Target *target,float x,float y,float r,float angle)
{
    target->X_Speed = x + X_Start;
    target->Y_Speed = y + Y_Start;
    target->angle = angle;
    target->tan_angle = x/y;
}

void Get_Target(Target *target,float x,float y,float r,float angle)
{
    target->X_Speed = x;
    target->Y_Speed = y;
    target->R_Speed = r;
    target->angle = angle;
    target->tan_angle = x/y;
}

/*
 * 该函数为了使定向移动可以走直线。对过大的速度进行限速
 */
void Speed_Change(Target *target,FinalVelocity *Fv)
{

    if(fu_flag == 1)
    {
        Fourv.v_x = -Fourv.v_x;
        Fourv.v_y = -Fourv.v_y;
    }
    if(Fv->v_x > 0 && Fv->v_y < 0)
    {
        if (Fv->v_x > -Fv->v_y * target->tan_angle)
            Fv->v_x = -Fv->v_y * target->tan_angle;
        else if (Fv->v_x < -Fv->v_y * target->tan_angle)
            Fv->v_y = -Fv->v_x / target->tan_angle;
    }
    if(Fv->v_x < 0 && Fv->v_y > 0)
    {
        if (-Fv->v_x > Fv->v_y * target->tan_angle)
            Fv->v_x = -Fv->v_y * target->tan_angle;
        else if (-Fv->v_x < Fv->v_y * target->tan_angle)
            Fv->v_y = -Fv->v_x / target->tan_angle;
    }
    if(Fv->v_x > 0 && Fv->v_y > 0)
    {
        if (Fv->v_x > Fv->v_y * target->tan_angle)
            Fv->v_x = Fv->v_y * target->tan_angle;
        else if (Fv->v_x < Fv->v_y * target->tan_angle)
            Fv->v_y = Fv->v_x / target->tan_angle;
    }
    if(Fv->v_x < 0 && Fv->v_y < 0)
    {
        if (Fv->v_x < Fv->v_y * target->tan_angle)
            Fv->v_x = Fv->v_y * target->tan_angle;
        else if (Fv->v_x > Fv->v_y * target->tan_angle)
            Fv->v_y = Fv->v_x / target->tan_angle;
    }

    if(fu_flag == 1)
    {
        Fourv.v_x = -Fourv.v_x;
        Fourv.v_y = -Fourv.v_y;
        fu_flag = 0;
    }

}
/*
 * 普通的跑点函数
 * x_change与y_change即为变化的编码器值。r_change暂时无用，angle_change为变化到的角度
 */
float Run_Point(PID_TypeDef *PID_x,PID_TypeDef *PID_y,PID_TypeDef *PID_r,float x_change,float y_change,float r_change,float angle_change,int straight_flag)
{

        Row_Change(&FourTarget,x_change,y_change,r_change,angle_change);

        XYR_Speed_Charge(PID_x,PID_y,PID_r);

        if(straight_flag == 1)
        {
            Speed_Change(&FourTarget,&Fourv);
        }

        Classis_Slove(&Fourv,&FourWheel);

        usart_printf("%f,%f,%f,%f,%f,%f\n",Fourv.v_x,Fourv.v_y,locater.pos_x,locater.pos_y,locater.angle,locater.continuousAngle);

        if(((Fourv.v_x < 0.04f) && (Fourv.v_x > -0.04f)) && ((Fourv.v_y < 0.04f) && (Fourv.v_y > -0.04f)))
        {
            SendWheelData(&hcan1,M1_ID,0.0f,0);
            SendWheelData(&hcan1,M2_ID,0.0f,0);
            SendWheelData(&hcan1,M3_ID,0.0f,0);
            SendWheelData(&hcan1,M4_ID,0.0f,0);
            return 0;
        }
        else
        {
            SendWheelData(&hcan1,M1_ID,FourWheel.v_front * 100.0f,0);
            SendWheelData(&hcan1,M2_ID,FourWheel.v_left * 100.0f,0);
            SendWheelData(&hcan1,M3_ID,FourWheel.v_right * 100.0f,0);
            SendWheelData(&hcan1,M4_ID,FourWheel.v_back * 100.0f,0);
            return 1;
        }



}


/*
 * 该函数可以书写特定行走路线，目前书写为调用后跑一个正方形
 */
void Run_RouteCube(PID_TypeDef *PID_x,PID_TypeDef *PID_y,PID_TypeDef *PID_r,float x_change,float y_change,float r_change,float angle_change)
{
    static float last_x_change,last_y_change,last_r_change,last_angle_change;

    switch(count++)
    {
        case 1:
            Fourv.Done_flag = 1;
            last_x_change = x_change;
            last_y_change = y_change;
            last_r_change = r_change;
            last_angle_change = angle_change;
            break;
        case 2:
            Fourv.Done_flag = 1;
            x_change = last_x_change;
            y_change = last_y_change+500;
            last_x_change = x_change;
            last_y_change = y_change;
            last_r_change = r_change;
            last_angle_change = angle_change;
            break;
        case 3:
            Fourv.Done_flag = 1;
            x_change = last_x_change-500;
            y_change = last_y_change;
            last_x_change = x_change;
            last_y_change = y_change;
            last_r_change = r_change;
            last_angle_change = angle_change;
            break;
        case 4:
            Fourv.Done_flag = 1;
            x_change = last_x_change;
            y_change = last_y_change - 500;
            last_x_change = x_change;
            last_y_change = y_change;
            last_r_change = r_change;
            last_angle_change = angle_change;
            break;
        default:
            Fourv.Done_flag = 0;
            x_change = 0;
            y_change = 0;
            break;
    }

    while(Fourv.Done_flag)
    {
        Fourv.Done_flag = Run_Point(PID_x,PID_y,PID_r,x_change,y_change,r_change,0,0);
        usart_printf("%d,%f,%f\n",count,x_change,y_change);
    }
   
}
/*
 * 角度转换函数，将角度转换为弧度
 */
float Angle_Change(float angle)
{
    return angle*Pi/180;
}
/*
 * 将弧度转换为角度
 */
float FanAngle_Change(float angle)
{
    return angle*180/Pi;
}
/*
 * 此函数为将小车自身坐标系转换成世界坐标系，来借助码盘实现定位，该函数使得小车可以边走边转
 * 函数原理需要较多数学解算，可以自己研究
 */
float Row_Change(Target *target,float x,float y,float r,float angle)
{
    if(((locater.angle > 0) && (locater.angle < FanAngle_Change(atan2f(y,x)))) && ((x != 0) || (y != 0)))
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_cos_f32(Angle_Change(-locater.angle) + atan2f(y,x))/arm_sin_f32(Angle_Change(-locater.angle) + atan2f(y,x));
    }

    else if(((locater.angle < 90.0f) && (locater.angle > FanAngle_Change(atan2f(y,x)))) && ((x != 0) || (y != 0)) )
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = -y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_sin_f32(Angle_Change(90-locater.angle) + atan2f(y,x))/arm_cos_f32(Angle_Change(90-locater.angle) + atan2f(y,x));

    }
    else if(((locater.angle > 90.0f) && (locater.angle < FanAngle_Change(atan2f(y,x)) + 90)) && ((x != 0) || (y != 0)) )
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = -y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_cos_f32(Angle_Change(locater.angle) - atan2f(y,x))/arm_sin_f32(Angle_Change(locater.angle) - atan2f(y,x));

    }
    else if(((locater.angle <= 180.0f) && (locater.angle > FanAngle_Change(atan2f(y,x)) + 90)) && ((x != 0) || (y != 0)) )
    {
        fu_flag = 1;
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_sin_f32(Angle_Change(locater.angle - 90) - atan2f(y,x))/arm_cos_f32(Angle_Change(locater.angle - 90) - atan2f(y,x));
//        usart_printf("%f,%f,%f,%f,%f\n",target->X_Speed,target->Y_Speed,locater.pos_x,locater.pos_y,locater.angle);
//        usart_printf("%f,%f,%f,%f,%f,%f\n",target->X_Speed,target->Y_Speed, Fourv.v_x,Fourv.v_y,locater.pos_x,locater.pos_y);
    }

    else if(((locater.angle < 0) && (locater.angle > -FanAngle_Change(atan2f(x,y))))  && (angle > 0) && ((x != 0) || (y != 0)))
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_sin_f32(Angle_Change(90 + locater.angle) - atan2f(y,x))/arm_cos_f32(Angle_Change(90 + locater.angle) - atan2f(y,x));

    }
    else if(((locater.angle > -90.0f) && (locater.angle < -FanAngle_Change(atan2f(x,y)))) && ((x != 0) || (y != 0)))
    {
        target->X_Speed = -x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_sin_f32(Angle_Change(-locater.angle) - atan2f(x,y))/arm_cos_f32(Angle_Change(-locater.angle) - atan2f(x,y));

    }
    else if(((locater.angle < -90.0f) && (locater.angle > -FanAngle_Change(atan2f(x,y)) - 90)) && ((x != 0) || (y != 0)) )
    {
        target->X_Speed = -x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_cos_f32(Angle_Change(90 + locater.angle) + atan2f(x,y))/arm_sin_f32(Angle_Change(90 + locater.angle) + atan2f(x,y));

    }
    else if(((locater.angle >= -180.0f) && (locater.angle < -FanAngle_Change(atan2f(x,y)) - 90)) && ((x != 0) || (y != 0)) )
    {
        fu_flag = 1;
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = arm_cos_f32(Angle_Change(-locater.angle - 90) - atan2f(x,y))/arm_sin_f32(Angle_Change(-locater.angle - 90) - atan2f(x,y));

    }
    if((angle == 0) || ((x == 0) && (y == 0)))
    {
        target->X_Speed = x + X_Start;
        target->Y_Speed = y + Y_Start;
        target->angle = angle;
        target->tan_angle = x/y;

    }
}
/*
 * 原地自转函数
 */
void Revolove(float r_change)
{
    Get_Target(&FourTarget,0,0,r_change,0);
    Classis_Slove(&FourTarget,&FourWheel);
    SendWheelData(&hcan1,M1_ID,FourWheel.v_front * 100.0f,0);
    SendWheelData(&hcan1,M2_ID,FourWheel.v_left * 100.0f,0);
    SendWheelData(&hcan1,M3_ID,FourWheel.v_right * 100.0f,0);
    SendWheelData(&hcan1,M4_ID,FourWheel.v_back * 100.0f,0);
}

/*
 * 对x,y方向速度与自转线速度进行pid计算
 */
void XYR_Speed_Charge(PID_TypeDef *PID_x,PID_TypeDef *PID_y,PID_TypeDef *PID_r)
{
    if (fu_flag == 1) //解算出当小车转动角度较大时速度将和码盘反方向行走，故给予负值来完成到达码盘目标值的目的
    {
        Fourv.last_v_x = Fourv.v_x;
        Fourv.last_v_y = Fourv.v_y;
        Fourv.v_x = -pid_calculate(PID_x, locater.pos_x * 100.0f, FourTarget.X_Speed * 100.0f);
        Fourv.v_y = -pid_calculate(PID_y, locater.pos_y * 100.0f, FourTarget.Y_Speed * 100.0f);
    }
    else
    {
        Fourv.last_v_x = Fourv.v_x;
        Fourv.last_v_y = Fourv.v_y;
        Fourv.v_x = pid_calculate(PID_x, locater.pos_x * 100.0f, FourTarget.X_Speed * 100.0f);
        Fourv.v_y = pid_calculate(PID_y, locater.pos_y * 100.0f, FourTarget.Y_Speed * 100.0f);
    }

    Fourv.last_v_r = Fourv.v_r;
    Fourv.v_r = -pid_calculate(PID_r, locater.continuousAngle * 100.0f, FourTarget.angle * 100.0f);
    //直接进行解算小车转动从反向转转动，反应较大，故取负值


}

