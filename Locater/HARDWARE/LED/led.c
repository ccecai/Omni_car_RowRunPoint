#include "led.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOD的时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   //输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   //上拉输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_15); // GPIOA15 低电平
}
