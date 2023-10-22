/********************************Copyright (c)**********************************\
**
**                   (c) Copyright 2021, Main, China, QD.
**                           All Rights Reserved
**
**                           By(四川大学Robocon川山甲战队)
**
**
**----------------------------------文件信息------------------------------------
** 文件名称: encoder.c
** 创建人员: RC
** 创建日期: 2021-01-13
** 文档描述:
**
**----------------------------------版本信息------------------------------------
** 版本代号: V0.1
** 版本说明: 初始版本
**
**------------------------------------------------------------------------------
********************************End of Head************************************/

#include "encoder.h"

#define ENCODER_TIM_PERIOD (u16)(0xFFFF)   //不可大于65535 因为F4的定时器是16位的

/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
#if ENCODER_TIMER_IT_ENABLE
    NVIC_InitTypeDef NVIC_InitStructure;
#endif
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器4的时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PB端口时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); 	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2); 
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	//端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	//端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
				
		
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);//使用编码器模式3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
#if ENCODER_TIMER_IT_ENABLE
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);
#endif
	TIM_SetCounter(TIM2, 0x8000);
    TIM_Cmd(TIM2, ENABLE);
}

void Encoder_Init_TIM4(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
#if ENCODER_TIMER_IT_ENABLE
    NVIC_InitTypeDef NVIC_InitStructure;
#endif
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PB端口时钟

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); 	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); 
		
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);//使用编码器模式3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
#if ENCODER_TIMER_IT_ENABLE
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);
#endif
	TIM_SetCounter(TIM4, 0x8000);
    TIM_Cmd(TIM4, ENABLE);
}

/**************************************************************************
函数功能：TIM2中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
#if ENCODER_TIMER_IT_ENABLE
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源
    }
}
#endif
/**************************************************************************
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
#if ENCODER_TIMER_IT_ENABLE
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源
    }
}
#endif

/*******************************************************************************
** 函数名称: TIM2_Encoder_Read
** 功能描述: 采集编码器计数值
** 参数说明: None
** 返回说明: 当前编码器计数值
** 创建人员: RC
** 创建日期: 2021-01-13
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
int TIM2_Encoder_Read(void)
{
    int cnt = 0;
    cnt = TIM_GetCounter(TIM2) - 0x8000;
    TIM_SetCounter(TIM2, 0x8000);
    return cnt;
}

/*******************************************************************************
** 函数名称: TIM4_Encoder_Read
** 功能描述: 采集编码器计数值
** 参数说明: None
** 返回说明: 当前编码器计数值
** 创建人员: RC
** 创建日期: 2021-01-13
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
int TIM4_Encoder_Read(void)
{
   	int cnt = 0;
	cnt = TIM_GetCounter(TIM4)-0x8000 ;
    TIM_SetCounter(TIM4, 0x8000);
    return cnt;
}
/********************************End of File************************************/
