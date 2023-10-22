#include "locator.h"

#include "encoder.h"
#include "xv7011.h"
#include "usart.h"
#include "tim.h"
#include "filter.h"

//数据格式变换使用到的定义
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp) + 0) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
static void AngleNormal(float *angle);
static float Angle2Rad(float angle);

LocatorDataTypedef locator = {0, 0, ORIGINAL_X, ORIGINAL_Y, ORIGINAL_ANGLE};

float GYRO_OFFSET = 0;//陀螺仪偏移量

//定位仪的初始化，包括编码器、vx7011、USART1、TIM3、窗口滤波器结构体的初始化
void Locator_Init(void)
{
    Encoder_Init_TIM2();	 //编码器1初始化
	Encoder_Init_TIM4();	 //编码器2初始化
    XV7011_Init();
    UASRT1_Init(1152000);     //串口初始化为1152000
    TIMER_TASK_TIM3_Init();  //100us读取一次陀螺仪数据,二者都需要减一后赋值
    while(locatorInitflag) {}//等待陀螺仪偏移量、窗口滤波器结构体完成初始化
}

//读出xv7011的数据，并进行滤波与积分
void Locator_Filter_ReadYaw(LocatorDataTypedef *locator)
{
    int32_t gyro;//原始数据
    float yaw;//滤波后的数据
    XV7011_ReadData32(&gyro);//读出
    yaw = WindowFilter(&Gyro_Window, gyro);//滤波
    yaw = -(yaw - GYRO_OFFSET); //取逆时针为正，消除偏移

    if (!(yaw > -NOISE_FILTERDATA && yaw < NOISE_FILTERDATA))//消除抖动，当变化量超过NOISE_FILTERDATA时才进行积分
        locator->yaw += yaw * GYRO_SCALE_FACTOR;
}


//读入编码器的数值，与此时的姿态角计算新的空间坐标系中的坐标
void Locator_Calculate(LocatorDataTypedef *locator)
{
	float angleCal=locator->yaw + OFFSET_ANGLE;//定位仪与世界坐标系的角度差
    AngleNormal(&angleCal);//将角度归一化

    //读出编码器的数值，换算成距离
    locator->dis_p = -(float)TIM2_Encoder_Read() * ENCODER_SCALE_FACTOR;
    locator->dis_q = (float)TIM4_Encoder_Read() * ENCODER_SCALE_FACTOR;

    //将定位仪坐标系中坐标的变化量变换到空间坐标系中，并完成累加
    locator->pos_x += locator->dis_p * arm_cos_f32(Angle2Rad(angleCal)) - locator->dis_q * arm_sin_f32(Angle2Rad(angleCal));
    locator->pos_y += locator->dis_p * arm_sin_f32(Angle2Rad(angleCal)) + locator->dis_q * arm_cos_f32(Angle2Rad(angleCal));
}

//使用DMA将定位仪数据从USART1发送出去
void Locator_USART_SendData(LocatorDataTypedef *locator)
{
    uint8_t i;
    uint8_t dataSum = 0;
	
	AngleNormal(&(locator->yaw));

    USART1_DMA_TX_BUFFER[0] = 'P';
    USART1_DMA_TX_BUFFER[1] = 'G';
    /*发送x坐标值*/
    USART1_DMA_TX_BUFFER[2] = BYTE0(locator->pos_x);
    USART1_DMA_TX_BUFFER[3] = BYTE1(locator->pos_x);
    USART1_DMA_TX_BUFFER[4] = BYTE2(locator->pos_x);
    USART1_DMA_TX_BUFFER[5] = BYTE3(locator->pos_x);
    /*发送y坐标值*/
    USART1_DMA_TX_BUFFER[6] = BYTE0(locator->pos_y);
    USART1_DMA_TX_BUFFER[7] = BYTE1(locator->pos_y);
    USART1_DMA_TX_BUFFER[8] = BYTE2(locator->pos_y);
    USART1_DMA_TX_BUFFER[9] = BYTE3(locator->pos_y);
    /*发送机器人姿态角*/
    USART1_DMA_TX_BUFFER[10] = BYTE0(locator->yaw);
    USART1_DMA_TX_BUFFER[11] = BYTE1(locator->yaw);
    USART1_DMA_TX_BUFFER[12] = BYTE2(locator->yaw);
    USART1_DMA_TX_BUFFER[13] = BYTE3(locator->yaw);

    dataSum = 0;
    for (i = 0; i < 14; i++)
        dataSum += USART1_DMA_TX_BUFFER[i]; //和校验
    USART1_DMA_TX_BUFFER[14] = dataSum;

    UASRT1_DMA_TX(); // DMA发送以实现程序并行，发送14个字节耗时水平至少1.2ms
}

/**
 *@brief 将角度归一化到零周期(-180°，180°]
 *@param angle：输入角度值
 *@note
 */
static void AngleNormal(float *angle)
{
	while(*angle > 180)
	{
		*angle -= 360;
	}
	while(*angle <= -180)
	{
		*angle += 360;
	}
}

/**
 *@brief 将角度转化为弧度制
 *@param angle：输入角度值
 *@note
 */
static float Angle2Rad(float angle)
{
	return angle * 0.01745329252f; //pi/180
}
