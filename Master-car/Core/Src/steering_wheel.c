/**
 * @brief 这里是三舵轮运动学解算以及一些相关的函数
 */
#include "steering_wheel.h"

RobotWheelStruct rrWheel = {{WHEEL_FRONT_ID, angleBase_Front, 0, 0, 0},
                            {WHEEL_LEFT_ID,  angleBase_Left, 0, 0, 0},
                            {WHEEL_RIGHT_ID, angleBase_Right, 0, 0, 0}};

/**
 * @brief 设定机器人的速度，即把机器人的目标速度转换成三个轮系的速度矢量，并下发到底盘控制板上,这是最终使用的控制函数
 * @param robotVel 机器人目标速度信息结构体
 * @param robotAngle 机器人当前角度
 */
void SetRobotVel_Vector(RobotWheelStruct *robotWheel, float dir, float vel, float rotate, float robotAngle)
{
    if(ABS(vel) + ABS(rotate) > 0.00001f)
    {
        Robot2Wheel_Vector(&robotWheel->FrontWheel, dir, vel, rotate, robotAngle);
        AngleOptimize(&robotWheel->FrontWheel);

        Robot2Wheel_Vector(&robotWheel->LeftWheel, dir, vel, rotate, robotAngle);
        AngleOptimize(&robotWheel->LeftWheel);

        Robot2Wheel_Vector(&robotWheel->RightWheel, dir, vel, rotate, robotAngle);
        AngleOptimize(&robotWheel->RightWheel);
    }
    else
    {
        robotWheel->FrontWheel.velocity = 0;
        robotWheel->LeftWheel.velocity = 0;
        robotWheel->RightWheel.velocity = 0;
    }
    SendWheelData(&hcan2, robotWheel->FrontWheel.CAN_ID, robotWheel->FrontWheel.velocity, robotWheel->FrontWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->LeftWheel.CAN_ID, robotWheel->LeftWheel.velocity, robotWheel->LeftWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->RightWheel.CAN_ID, robotWheel->RightWheel.velocity, robotWheel->RightWheel.direction_angle);

}

void SetRobotVel_XY(RobotWheelStruct *robotWheel, float vel_x, float vel_y, float rotate, float robotAngle)
{
    if(ABS(vel_x) + ABS(vel_y) + ABS(rotate) > 0.00001f)
    {
        Robot2Wheel_XY(&robotWheel->FrontWheel, vel_x, vel_y, rotate, robotAngle);
        AngleOptimize(&robotWheel->FrontWheel);

        Robot2Wheel_XY(&robotWheel->LeftWheel, vel_x, vel_y, rotate, robotAngle);
        AngleOptimize(&robotWheel->LeftWheel);

        Robot2Wheel_XY(&robotWheel->RightWheel, vel_x, vel_y, rotate, robotAngle);
        AngleOptimize(&robotWheel->RightWheel);

    }
    else
    {
        robotWheel->FrontWheel.velocity = 0;
        robotWheel->LeftWheel.velocity = 0;
        robotWheel->RightWheel.velocity = 0;
    }
    SendWheelData(&hcan2, robotWheel->FrontWheel.CAN_ID, robotWheel->FrontWheel.velocity, robotWheel->FrontWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->LeftWheel.CAN_ID, robotWheel->LeftWheel.velocity, robotWheel->LeftWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->RightWheel.CAN_ID, robotWheel->RightWheel.velocity, robotWheel->RightWheel.direction_angle);


}

/**
 * @brief 把三个轮子调整到锁死状态
 */
void RobotLock(RobotWheelStruct *robotWheel)
{
    robotWheel->FrontWheel.last_direction_angle = robotWheel->FrontWheel.direction_angle;
    robotWheel->FrontWheel.direction_angle = robotWheel->FrontWheel.positon_angle + 90.0f;
    robotWheel->FrontWheel.velocity = 0;
    AngleOptimize(&robotWheel->FrontWheel);

    robotWheel->LeftWheel.last_direction_angle = robotWheel->LeftWheel.direction_angle;
    robotWheel->LeftWheel.direction_angle = robotWheel->LeftWheel.positon_angle + 90.0f;
    robotWheel->LeftWheel.velocity = 0;
    AngleOptimize(&robotWheel->LeftWheel);

    robotWheel->RightWheel.last_direction_angle = robotWheel->RightWheel.direction_angle;
    robotWheel->RightWheel.direction_angle = robotWheel->RightWheel.positon_angle + 90.0f;
    robotWheel->RightWheel.velocity = 0;
    AngleOptimize(&robotWheel->RightWheel);

    SendWheelData(&hcan2, robotWheel->FrontWheel.CAN_ID, robotWheel->FrontWheel.velocity, robotWheel->FrontWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->LeftWheel.CAN_ID, robotWheel->LeftWheel.velocity, robotWheel->LeftWheel.direction_angle);
    SendWheelData(&hcan2, robotWheel->RightWheel.CAN_ID, robotWheel->RightWheel.velocity, robotWheel->RightWheel.direction_angle);
}

void Robot2Wheel_Vector(WheelStructTypedef* wheelStruct, float dir, float vel, float rotate, float robotAngle)
{
	float vel_x;
    float vel_y; //平移速度在xy方向的分量
    float vel_rotate;               //自转带来的线速度
    float sumVel_x;
    float sumVel_y; //平移速度在xy方向的分量

    /*先计算平移速度的分量*/
    vel_x = vel * arm_cos_f32(dir);
    vel_y = vel * arm_sin_f32(dir);
    /*再计算自转带来的线速度*/
    vel_rotate = Angle2Rad(rotate) * DIS_CENTER2WHEEL;   //各轮子自转线速度大小为角速度乘以轮到中心距离（单位mm）

	/*将这两个分量叠加*/
	sumVel_x = vel_x + vel_rotate * arm_cos_f32(Angle2Rad(wheelStruct->positon_angle + robotAngle));  //这一笔用的巧，直接把一个平动先分到xy坐标上，在用自转分速度分到xy坐标上，最后再用xy正切算出分到每个轮上最终确定的方向和速度！
	sumVel_y = vel_y + vel_rotate * arm_sin_f32(Angle2Rad(wheelStruct->positon_angle + robotAngle));

    /*计算这个轮子的速度大小绝对值*/
	arm_sqrt_f32(sumVel_x * sumVel_x + sumVel_y * sumVel_y, &wheelStruct->velocity);

    /*计算这个轮子的速度方向*/
    wheelStruct->last_direction_angle = wheelStruct->direction_angle;
    wheelStruct->direction_angle = Rad2Angle(atan2f(sumVel_y, sumVel_x)) - robotAngle;
}

void Robot2Wheel_XY(WheelStructTypedef* wheelStruct, float vel_x, float vel_y, float rotate, float robotAngle)
{
    float vel_rotate;               //自转带来的线速度
    float sumVel_x;
    float sumVel_y; //平移速度在xy方向的分量

    /*再计算自转带来的线速度*/
    vel_rotate = Angle2Rad(rotate) * DIS_CENTER2WHEEL;   //各轮子自转线速度大小为角速度乘以轮到中心距离（单位mm）

    /*将这两个分量叠加*/
    sumVel_x = vel_x + vel_rotate * arm_cos_f32(Angle2Rad(wheelStruct->positon_angle + robotAngle));  //这一笔用的巧，直接把一个平动先分到xy坐标上，在用自转分速度分到xy坐标上，最后再用xy正切算出分到每个轮上最终确定的方向和速度！
    sumVel_y = vel_y + vel_rotate * arm_sin_f32(Angle2Rad(wheelStruct->positon_angle + robotAngle));

    /*计算这个轮子的速度大小绝对值*/
    arm_sqrt_f32(sumVel_x * sumVel_x + sumVel_y * sumVel_y, &wheelStruct->velocity);

    /*计算这个轮子的速度方向*/
    wheelStruct->last_direction_angle = wheelStruct->direction_angle;
    wheelStruct->direction_angle = Rad2Angle(atan2f(sumVel_y, sumVel_x)) - robotAngle;
}


void AngleOptimize(WheelStructTypedef* wheelStruct)
{
    float wheel_angle_differ;

    wheel_angle_differ = wheelStruct->direction_angle - wheelStruct->last_direction_angle;
    angleLimit(&wheel_angle_differ);

    if(ABS(wheel_angle_differ) <= 90.0f)
    {
        wheelStruct->direction_angle = wheelStruct->last_direction_angle + wheel_angle_differ;
    }
    else if(wheel_angle_differ < -90.0f)
    {
        wheelStruct->direction_angle = wheelStruct->last_direction_angle + wheel_angle_differ + 180.0f;
        wheelStruct->velocity = -wheelStruct->velocity;
    }
    else if(wheel_angle_differ > 90.0f)
    {
        wheelStruct->direction_angle = wheelStruct->last_direction_angle + wheel_angle_differ - 180.0f;
        wheelStruct->velocity = -wheelStruct->velocity;
    }
}

/**
 * @brief 将角度限制在-180~180内
 * @param angle
 */
void angleLimit(float *angle)
{
	if(*angle>180)
	{
		*angle-=360;
        angleLimit(angle);
	}
	else if(*angle<=-180)
	{
		*angle+=360;
        angleLimit(angle);
	}
}

//角度单位转换1
float Angle2Rad(float angle)
{
	return angle/180.0f*3.14159265359f;
}
//角度单位转换2
float Rad2Angle(float radian)
{
	return radian/3.14159265359f*180;
}

/**
 * @brief 向下层板子发送轮系控制信息
 * @param _hcan &hcan1 or 2
 * @param wheel_id 轮canid
 * @param wheel_spd 轮系速度
 * @param wheel_direction 轮系方向
 */
void SendWheelData(CAN_HandleTypeDef *_hcan, uint32_t wheel_id, float wheel_spd, float wheel_direction)
{
    static CAN_TxHeaderTypeDef TxHeader;    //发送报文结构体定义
    static uint8_t TxData[8];
    static uint32_t mbox;
    static union union_float union_32f;

    TxHeader.StdId = wheel_id;     // id赋值
    TxHeader.IDE = 0;              // 标准帧
    TxHeader.RTR = 0;              //
    TxHeader.DLC = 8;              // 8字节数据帧

    union_32f.data_32 = wheel_spd;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_32 = wheel_direction;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //等一个空邮箱
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //发送失败就卡住了
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
