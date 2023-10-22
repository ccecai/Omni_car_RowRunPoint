#include "sys.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos 使用
#endif
//////////////////////////////////////////////////////////////////////////////////

uint8_t USART1_DMA_TX_BUFFER[USART1_DMA_SEND_LEN];

uint8_t UART1_Use_DMA_Tx_Flag;

uint8_t locatorResetflag = 0;

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...)
{

	unsigned char UsartPrintfBuf[296];
	va_list ap;
	unsigned char *pStr = UsartPrintfBuf;
	
	va_start(ap, fmt);
	vsnprintf((char *)UsartPrintfBuf, sizeof(UsartPrintfBuf), fmt, ap);
	va_end(ap);
	
	while(*pStr != 0)
	{
		USART_SendData(USARTx, *pStr++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
	}

}

#endif


void UASRT1_Init(u32 bound)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //使能USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // GPIOA10复用为USART1

	// DMA配置
	DMA_DeInit(DMA2_Stream7);
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
	{
	}																	   //等待DMA可配置
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;						   /* 配置DMA通道 */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));  /* 外设 */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART1_DMA_TX_BUFFER;/* 源的数据*/
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				   /* 方向 */
	DMA_InitStructure.DMA_BufferSize = USART1_DMA_SEND_LEN;				   /* 长度 */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	   /* 外设地址是否自增 */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				   /* 内存地址是否自增 */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;	   /* 目的数据带宽 */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		   /* 源数据宽度 */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;						   /* 单次传输模式/循环传输模式 */
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;					   /* DMA优先级 */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;				   /* FIFO模式/直接模式 */
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;		   /* FIFO大小 */
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;			   /* 单次传输 */
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);				 //根据指定的参数初始化VIC寄存器、
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7); //清除传输完成中断
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TE | DMA_IT_TC, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); //使能串口1的DMA发送

	// USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//初始化PA9，PA10

	// USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式
	USART_Init(USART1, &USART_InitStructure);										//初始化串口1
	USART_Cmd(USART1, ENABLE);														//使能串口1

	USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	// Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  //串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  //根据指定的参数初始化VIC寄存器、
}

void UASRT1_DMA_TX(void)
{
	/* 等待空闲 */
	if (UART1_Use_DMA_Tx_Flag == 0)
	/* 复制数据 */
	{
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
		DMA_SetCurrDataCounter(DMA2_Stream7, USART1_DMA_SEND_LEN);
		DMA_Cmd(DMA2_Stream7, ENABLE);
		UART1_Use_DMA_Tx_Flag = 1;
	}
	else
	{
	} // donothing
}

void USART3_Init(u32 bound)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //使能USART1时钟

	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3); // GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3); // GPIOA10复用为USART1

	// USART3端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; // GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			 //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //初始化PA9，PA10

	// USART3 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式
	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART3, ENABLE);

	USART_ClearFlag(USART3, USART_FLAG_TC);

	// Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		  //串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  //根据指定的参数初始化VIC寄存器、
}

void USART1_IRQHandler(void) //串口1中断服务程序
{
	u8 Res = Res;
#if SYSTEM_SUPPORT_OS //使用UCOS操作系统
	OSIntEnter();
#endif
	{
		if (USART_GetITStatus(USART1, USART_IT_TXE) == RESET)
		{
			/* 关闭发送完成中断  */
			USART_ITConfig(USART1, USART_IT_TC, DISABLE);
			/* 发送完成  */
			UART1_Use_DMA_Tx_Flag = 0;
		}
		
		if(USART_GetITStatus(USART1, USART_IT_RXNE) == RESET)
		{
			Res = USART_ReceiveData(USART1);
			if(Res=='x')
				locatorResetflag = 1;
		}
	}
#if SYSTEM_SUPPORT_OS
	OSIntExit(); //退出中断
#endif
}

void DMA2_Stream7_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
	{
		/* 清除标志位 */
		DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	}
}
