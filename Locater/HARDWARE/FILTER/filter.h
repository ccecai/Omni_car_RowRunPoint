#ifndef __FILTER_H__
#define __FILTER_H__

#include "sys.h"

//最大窗口长度
#define WINDOW_SIZE 50

//滑动窗口滤波器
typedef struct
{
    uint8_t len;              //窗口宽度
    uint8_t index;            //当前指针
    int32_t sum;              //窗口内所有数值的和
    int32_t buf[WINDOW_SIZE]; 
} WindowFilterStruct;

extern WindowFilterStruct Gyro_Window;

void WindowFilterStructInit(WindowFilterStruct *Window, uint8_t len, int32_t offset);
float WindowFilter(WindowFilterStruct *Window, int32_t inPut);

#endif
