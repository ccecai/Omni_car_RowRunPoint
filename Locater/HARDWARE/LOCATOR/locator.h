#ifndef __LOCATOR_H
#define __LOCATOR_H

#include "sys.h"

#define PI 3.14159265358979f

//定位仪轮子直径
#define LOCATOR_D 47.3f

//定位仪坐标系在机器人坐标系中的位置与角度
//#define OFFSET_X     0
//#define OFFSET_Y     0
#define OFFSET_DIS   0//定位仪原点的距机器人坐标系原点的距离，当前版本未使用
#define OFFSET_ANGLE 0.0f

//机器人的初始位置与姿态
#define ORIGINAL_X     0
#define ORIGINAL_Y     0
#define ORIGINAL_ANGLE 0

//消除数据噪声的标度，角度变化超过NOISE_FILTERDATA才认为机器人姿态角发生变化/////////////////////////////////////////
#define NOISE_FILTERDATA 1000

//陀螺仪数据标度因子注意这里的值要跟定时器频率挂钩(默认值是0.1ms读一次数据的积分标度)
//包含了二进制数据向常用单位的变换、积分效果，最终数据单位转换成°
#define GYRO_SCALE_FACTOR 0.000000005598580745f

//编码器的标度因子
//PI*LOCATOR_D/16384(一圈返回脉冲为16384)
#define ENCODER_SCALE_FACTOR 0.00766077f

typedef struct
{
   float dis_p;//定位仪坐标系中的距离
   float dis_q;
   float pos_x;//空间坐标系中的距离
   float pos_y;
   float yaw;//定位仪坐标系与空间坐标系的夹角，逆时针为正
}LocatorDataTypedef;

extern LocatorDataTypedef locator;

extern float GYRO_OFFSET;

void Locator_Init(void);
void Locator_Filter_ReadYaw(LocatorDataTypedef *locator);
void Locator_Calculate(LocatorDataTypedef *locator);
void Locator_USART_SendData(LocatorDataTypedef *locator);


#endif // !__LOCATOR_H

