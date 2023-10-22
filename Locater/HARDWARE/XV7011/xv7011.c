#include "xv7011.h"
#include "bsp_spi.h"

/* XV7011BB Gyroscope registers addresses */
#define GYRO_DSP_CTL1            0x01
#define GYRO_DSP_CTL2            0x02
#define GYRO_DSP_CTL3            0x03
#define GYRO_STS_RD              0x04
#define GYRO_SLP_IN              0x05
#define GYRO_SLP_OUT             0x06
#define GYRO_ST_BY               0x07
#define GYRO_TEMP_RD             0x08
#define GYRO_SW_RST              0x09
#define GYRO_DAT_ACCON           0x0A
#define GYRO_OUT_CTL1            0x0B
#define GYRO_AUTO_C              0x0C
#define GYRO_DSP_RES             0x0D
#define GYRO_MEM_LOAD            0x1B
#define GYRO_TSDATA_FORMAT       0x1C
#define GYRO_IF_CTL              0x1F

#define CS_XV7011_ON()           GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define CS_XV7011_OFF()          GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define XV7011_SPI_RW(reg)       SPI1_RW_Byte(reg)
#define XV7011_Delay_ms(nms)     delay_ms(nms)

static void XV7011_WriteReg(uint8_t reg, uint8_t data);
static uint8_t XV7011_ReadREG(uint8_t reg);
static void XV7011_ReadBytes(uint8_t reg, uint8_t length, uint8_t *data);

// XV7011初始化函数
void XV7011_Init(void)
{
	//XV7011 CS GPIO初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);//CS_XV7011_OFF()

	//SPI初始化
	Init_SPI1();

	//xv7011寄存器配置工作模式
	XV7011_Delay_ms(1);
	XV7011_SPI_RW(GYRO_SW_RST); //重启
	XV7011_Delay_ms(10);

	XV7011_WriteReg(GYRO_IF_CTL, 0);	     //开启spi  关闭iic
	XV7011_WriteReg(GYRO_DSP_CTL2, 0X01);    //低通滤波  2ND ORDER(与多个XV7011读数据时的时序有关) 35HZ
	XV7011_WriteReg(GYRO_DSP_CTL3, 0X01);    //不开启0速率校准 FS/2
	//XV7011_WriteReg(GYRO_DSP_CTL3,0X41);   //开启0速率校准 FS/2
	XV7011_WriteReg(GYRO_OUT_CTL1, 0X05);    //24bit   持续输出
	XV7011_SPI_RW(GYRO_SLP_OUT);
	//XV7011_SPI_RW(GYRO_AUTO_C);

	XV7011_Delay_ms(1);
}


//向地址为reg的寄存器中写入data
static void XV7011_WriteReg(uint8_t reg, uint8_t data)
{
	CS_XV7011_ON();
	XV7011_SPI_RW(reg);
	XV7011_SPI_RW(data);
	CS_XV7011_OFF();
}

//读出地址为reg的寄存器中的数据
static uint8_t XV7011_ReadREG(uint8_t reg)
{
	uint8_t regData;
	CS_XV7011_ON();
	XV7011_SPI_RW(reg | 0x80);//将首位置为1
	regData = XV7011_SPI_RW(0xff);//读数据的时候MOSI上的数据无影响
	CS_XV7011_OFF();
	return regData;
}

//从vx7011中连续读出多字节的数据
static void XV7011_ReadBytes(uint8_t reg, uint8_t length, uint8_t *data)
{
	CS_XV7011_ON();
	uint8_t count = 0;
	XV7011_SPI_RW(reg | 0x80);//将首位置为1
	for (count = 0; count < length; count++)
	{
		data[count] = XV7011_SPI_RW(0xff);
	}
	CS_XV7011_OFF();
}

//从xv7011中读出24bit的角速度值
void XV7011_ReadData32(int32_t *data)
{
	static int16_t temp;
	uint8_t buffer[3];
	XV7011_ReadBytes(GYRO_DAT_ACCON, 3, buffer);
	//	*data = (int32_t)((buffer[0]<<16) | (buffer[1]<<8) | buffer[2]);
	temp = (int16_t)(((buffer[0]) << 8) | buffer[1]);
	*data = (int32_t)((temp << 8) | buffer[2]);
}

//从xv7011中读出16bit的角速度值
void XV7011_ReadData16(int16_t *data)
{
	uint8_t buffer[2];
	XV7011_ReadBytes(GYRO_DAT_ACCON, 2, buffer);
	*data = (int16_t)(((buffer[0]) << 8) | buffer[1]);
}











