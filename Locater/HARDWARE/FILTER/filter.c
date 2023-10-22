#include "filter.h"

//滑动窗口滤波使用的结构体
WindowFilterStruct Gyro_Window;

//滑动窗口滤波结构体初始化
void WindowFilterStructInit(WindowFilterStruct *Window, uint8_t len, int32_t offset)
{
	uint8_t i;
	Window->len = len;
	Window->index = 0;
	Window->sum = Window->len * offset;
	for(i = 0;i < Window->len;i++)
	{
		Window->buf[i] = offset;
	}
}

//滑动窗口滤波
float WindowFilter(WindowFilterStruct *Window, int32_t inPut)
{
	Window->sum -= Window->buf[Window->index];//减去队列中的最后一项
	Window->sum += inPut;                     //加上新的输入
	Window->buf[Window->index++] = inPut;     //更新队列

	if (Window->index >= Window->len)
		Window->index = 0;

	return (float)Window->sum / (float)Window->len; //返回结果
}

