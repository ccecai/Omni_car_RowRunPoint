#ifndef __TIMER_h
#define __TIMER_h

#include "sys.h"

#define SENDCOUNT 50    //发送频率为500Hz
#define CALCOUNT 20     //计算频率为1kHz
#define INITCOUNT 15000 //计算传感器的偏移量所取的次数

//计时控制使用到的标志位，1为有效
extern uint8_t locatorSendflag;
extern uint8_t locatorCalflag;
extern uint8_t locatorInitflag;

void TIMER_TASK_TIM3_Init(void);

#endif
