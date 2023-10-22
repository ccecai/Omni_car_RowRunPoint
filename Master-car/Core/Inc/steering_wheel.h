#ifndef __STEER_WHEEL__H
#define __STEER_WHEEL__H


#include "main.h"
#include "stm32f4xx.h"
#include "math.h"

#define ABS(x)  ((x>0)? (x): -(x))

//机器人中心到轮子距离
#define DIS_CENTER2WHEEL  16.472f

#define angleBase_Front  (180.0f)
#define angleBase_Left   (-60.0f)
#define angleBase_Right  (60.0f)

#define WHEEL_FRONT_ID        (0x301)
#define WHEEL_LEFT_ID         (0x302)
#define WHEEL_RIGHT_ID        (0x303)

//“车轮”速度大小及方向
typedef struct
{
    uint32_t CAN_ID;
    float positon_angle;
	float velocity;              //车轮速度大小
	float direction_angle;        //车轮方向
	float last_direction_angle;
}WheelStructTypedef;

//三个车轮结构体信息
typedef struct
{
    WheelStructTypedef FrontWheel;
    WheelStructTypedef LeftWheel;
    WheelStructTypedef RightWheel;
}RobotWheelStruct;

/*//“机器人”速度大小与方向（由3或4个“车轮速度矢量”共同决定）结构体
typedef struct
{
//	float vel;      //机器人速度大小
//	float direction;//机器人速度方向
    float vel_x;    //机器人速度x分量
    float vel_y;    //机器人速度y分量
	float omega;    //机器人自转角速度（俯视图逆时针为正方向）
}robot_vel_s;*/

union union_float
{
    uint8_t data_8[4];
    float data_32;
};

//c文件中变量的声明（用于外部调用）

extern RobotWheelStruct rrWheel;

//c文件中函数声明
void SetRobotVel_Vector(RobotWheelStruct *robotWheel, float dir, float vel, float rotate, float robotAngle);
void SetRobotVel_XY(RobotWheelStruct *robotWheel, float vel_x, float vel_y, float rotate, float robotAngle);
void RobotLock(RobotWheelStruct *robotWheel);

void Robot2Wheel_Vector(WheelStructTypedef* wheelStruct, float dir, float vel, float rotate, float robotAngle);
void Robot2Wheel_XY(WheelStructTypedef* wheelStruct, float vel_x, float vel_y, float rotate, float robotAngle);
void AngleOptimize(WheelStructTypedef* wheelStruct);

float Rad2Angle(float radian);
void angleLimit(float *angle);
float Angle2Rad(float angle);

void SendWheelData(CAN_HandleTypeDef *_hcan, uint32_t wheel_id, float wheel_spd, float wheel_direction);

#endif
