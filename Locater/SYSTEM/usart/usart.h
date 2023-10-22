#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
//#include "stm32f4xx_conf.h"
#include "sys.h" 

//USART1 DMA发送的长度
#define USART1_DMA_SEND_LEN     15

//USART1 DMA发送缓冲区
extern uint8_t USART1_DMA_TX_BUFFER[USART1_DMA_SEND_LEN];

extern uint8_t locatorResetflag;

void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...);

void UASRT1_Init(u32 bound);
void USART3_Init(u32 bound);
void UASRT1_DMA_TX(void);
#endif


