#include "sys.h"
#include "delay.h"

#include "led.h"
#include "usart.h"
#include "encoder.h"
#include "bsp_spi.h"
#include "filter.h"
#include "xv7011.h"
#include "locator.h"
#include "tim.h"

#include "arm_math.h"
#include "arm_const_structs.h"

//软件复位函数
void System_Reset(void) 
{
	__set_FAULTMASK(1); //关闭所有中断
	NVIC_SystemReset(); //进行软件复位
}


//主函数
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置
	delay_init(168);								//延时函数初始化中断优先级分组2

	//LED_Init();
	//LED_ON();
	USART3_Init(115200);

	Locator_Init();

	while (1)
	{
		if (locatorCalflag == 1) //读10次陀螺仪数据就跑一次（1ms/1次）
		{
			Locator_Calculate(&locator);
			locatorCalflag = 0;
		}
		if (locatorSendflag == 1) //10ms串口发送一次，串口DMA耗时水平不超过2.5ms，合理
		{
			Locator_USART_SendData(&locator);//串口发送数据
			UsartPrintf(USART3, "%f,%f,%f\n", locator.pos_x, locator.pos_y, locator.yaw);
			locatorSendflag = 0;
		}
		if (locatorResetflag == 1)
		{
			System_Reset();
		}

	}
}
