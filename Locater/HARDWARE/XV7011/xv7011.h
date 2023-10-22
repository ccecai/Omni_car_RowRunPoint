#ifndef _XV7011_H
#define _XV7011_H

#include "sys.h"
#include "delay.h"

void XV7011_Init(void);
void XV7011_ReadData32(int32_t *data);
void XV7011_ReadData16(int16_t *data);

#endif
