#ifndef _LED_H
#define _LED_H
#include "sys.h"

//LED端口定义
#define LED_ON()  GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define LED_OFF() GPIO_ResetBits(GPIOA, GPIO_Pin_15)

void LED_Init(void);  //初始化
#endif
