#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"

//是否启用定时器的中断，1/0 Y/N
#define ENCODER_TIMER_IT_ENABLE 0

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM4(void);
int TIM2_Encoder_Read(void);
int TIM4_Encoder_Read(void);

#endif
