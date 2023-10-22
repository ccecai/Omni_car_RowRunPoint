/*********************************************************/
//
//
//
/*********************************************************/
#define BSP_SPI

#include "bsp_spi.h"
#include "sys.h"

void SPI_ModeSet(SPI_TypeDef *SPIx, uint8_t mode)
{
	SPI_InitTypeDef SPI_InitStructure;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;					   //主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;				   //数据大小8位
	switch (mode)
	{
	case 0:
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //时钟极性，空闲时为低
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第1个边沿有效，上升沿为采样时刻
	}
	break;
	case 1:
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //时钟极性，空闲时为低
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //第1个边沿有效，上升沿为采样时刻
	}
	break;
	case 2:
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;	 //时钟极性，空闲时为高
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第1个边沿有效，上升沿为采样时刻
	}
	break;
	case 3:
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;	 //时钟极性，空闲时为高
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //第1个边沿有效，上升沿为采样时刻
	}
	break;
	default:
	{
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //时钟极性，空闲时为低
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第1个边沿有效，上升沿为采样时刻
	}
	break;
	}
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							// NSS信号由软件产生
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // 16分频，5.25MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIx, &SPI_InitStructure);
}

void Init_SPI1(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); // PA5复用为 SPI1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); // PA6复用为 SPI1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); // PA7复用为 SPI1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   //复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //上拉

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);

	SPI_ModeSet(SPI1, 0);

	SPI_Cmd(SPI1, ENABLE);
}

uint8_t SPI1_RW_Byte(uint8_t byte)
{
	/*等待发送寄存器空*/
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPI1, byte); //发送一个字节
	/* 等待接收寄存器有效*/
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
		;
	return SPI_I2S_ReceiveData(SPI1);
}

void Init_SPI2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI1); // PB13复用为 SPI2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI1); // PB14复用为 SPI2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI1); // PB15复用为 SPI2

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   //复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
	//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_ModeSet(SPI2, 0);

	SPI_Cmd(SPI2, ENABLE);
}

uint8_t SPI2_RW_Byte(uint8_t byte)
{
	/*等待发送寄存器空*/
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPI2, byte); //发送一个字节
	/* 等待接收寄存器有效*/
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;
	return SPI_I2S_ReceiveData(SPI2);
}
