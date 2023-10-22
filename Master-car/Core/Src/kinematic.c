#include "kinematic.h"
#include "stm32f4xx_it.h"

/**
 * @brief 定位初始化
 * @param position 定位结构体
 */
void Position_Init(PositionStructure *position)
{
	position->x=0;
	position->y=0;
	position->theta=0;
	
	position->lastx=0.0f;
	position->lasty=0.0f;
}