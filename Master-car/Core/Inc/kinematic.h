#ifndef __KINEMATIC_H
#define __KINEMATIC_H
#include "main.h"

#define pi 3.14159265359f

typedef struct 
{
	float x;
	float y;
	float theta;
	
	int lastx;
	int lasty;
	
	int pulsea;
	int pulseb;
	
	int distancea;
	int distanceb;
}PositionStructure;

void Position_Init(PositionStructure *position);



#endif
