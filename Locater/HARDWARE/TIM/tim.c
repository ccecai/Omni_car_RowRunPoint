#include "tim.h"

#include "filter.h"
#include "xv7011.h"
#include "locator.h"

uint8_t locatorSendflag = 0;
uint8_t locatorCalflag = 0;
uint8_t locatorInitflag = 1;


void TIMER_TASK_TIM3_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = 99;						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = 83;					//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);				//根据指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //使能指定的TIM6中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;			  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);							  //初始化NVIC寄存器
	TIM_Cmd(TIM3, ENABLE);
}

void TIM3_IRQHandler(void)
{
	static u16 count;//计时变量
	static int32_t gyro;
	static int32_t sumGyro;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
	{
		if(locatorInitflag)//初始化，计算启动时INITCOUNT个数据的均值，用于消除误差
		{
			XV7011_ReadData32(&gyro);
			sumGyro += gyro;
			count++;
			if (count == INITCOUNT)
			{
				GYRO_OFFSET = (float)sumGyro / (float)INITCOUNT;//计算偏移量
				WindowFilterStructInit(&Gyro_Window, 20,(int32_t)GYRO_OFFSET);//利用得到的偏移量初始化窗口滤波器的结构体
				count = 0;
				locatorInitflag = 0;
			}
		}
		else//正常工作模式下执行的部分，进行积分，以及时间标志的控制
		{
			Locator_Filter_ReadYaw(&locator);
			count = (count + 1) % 1000;
			if (!(count % SENDCOUNT))//发送时间标志控制
			{
				locatorSendflag = 1;
			}
			if (!(count % CALCOUNT))//计算时间标志控制
			{
				locatorCalflag = 1;
			}
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除TIMx的中断待处理位:TIM 中断源
	}
}
