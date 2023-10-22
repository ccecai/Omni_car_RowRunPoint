#ifndef __24L01_H__
#define __24L01_H__

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "main.h"

//控制命令的宏定义
//#define back_to_origin 0x13
//#define ring_point_left 0x11
//#define ring_point_right 0x12

//#define point_1 0x0d
//#define point_2 0x0f
//#define point_3 0x0e
//#define point_4 0x10

//#define flip 0x07
//#define reset 0x08

#define NRF_TAKE_RING 0x0b
#define NRF_ROBOT_LOCK 0x0a

#define NRF_CLAW_HEIGHT_3 0x14
#define NRF_CLAW_HEIGHT_2 0x15
#define NRF_CLAW_HEIGHT_1 0x16
#define NRF_CLAW_HEIGHT_0 0x17

#define NRF_SHOOT_RING 0x01

#define NRF_CLAW_CLOSE 0x05
#define NRF_CLAW_OPEN 0x06



#define NRF_PIONT_1 0xa1
#define NRF_PIONT_2 0xa2
#define NRF_PIONT_3 0xa3
#define NRF_PIONT_4 0xa4
#define NRF_PIONT_5 0xa5
#define NRF_PIONT_6 0xa6
#define NRF_PIONT_7 0xa7
#define NRF_PIONT_8 0xa8
#define NRF_PIONT_9 0xa9
#define NRF_PIONT_10 0xaa
#define NRF_PIONT_11 0xab
#define NRF_PIONT_12 0xac
#define NRF_PIONT_13 0xad
#define NRF_PIONT_14 0xae


#define NRF_TAKE_RING_0 0xc0
#define NRF_TAKE_RING_1 0xc1
#define NRF_TAKE_RING_2 0xc2


#define NRF_Laser_Loc   0x08
#define NRF_HAND_CTRL 0x0c

//控制状态的宏定义
#define NRF_STATUS_SPEED  0x01
#define NRF_STATUS_ANGLE  0x02
#define NRF_STATUS_AUTO 0x04
#define NRF_STATUS_VISION 0x08




//遥控器接收结构体
typedef struct
{
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint8_t command;
    uint8_t status;
    uint8_t speed;
    uint8_t height;
    int8_t yaw;
    uint16_t midValue;
}RemoteRXSturct;

extern uint16_t midValue;

// NRF24L01发送接收数据宽度定义
#define TX_ADR_WIDTH 5    // 5字节的地址宽度
#define RX_ADR_WIDTH 5    // 5字节的地址宽度


#define TX_PLOAD_WIDTH 10 // 32字节的用户数据宽度
#define RX_PLOAD_WIDTH 10    // 32字节的用户数据宽度

// NRF24L01寄存器操作命令
#define NRF_READ_REG 0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_WRITE_REG 0x20 //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD 0x61   //读RX有效数据,1~32字节
#define WR_TX_PLOAD 0xA0   //写TX有效数据,1~32字节
#define FLUSH_TX 0xE1      //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX 0xE2      //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL 0xE3   //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP 0xFF           //空操作,可以用来读状态寄存器
// SPI(NRF24L01)寄存器地址
#define CONFIG 0x00     //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
// bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA 0x01      //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR 0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW 0x03   //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR 0x04 //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH 0x05      // RF通道,bit6:0,工作通道频率;
#define RF_SETUP 0x06   // RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS 0x07     //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
// bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX 0x10     //达到最大发送次数中断
#define TX_OK 0x20      // TX发送完成中断
#define RX_OK 0x40      //接收到数据中断

#define OBSERVE_TX 0x08      //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD 0x09              //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0 0x0A      //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1 0x0B      //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2 0x0C      //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3 0x0D      //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4 0x0E      //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5 0x0F      //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR 0x10         //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0 0x11        //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1 0x12        //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2 0x13        //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3 0x14        //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4 0x15        //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5 0x16        //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17 // FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
// bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;

extern uint8_t NRFHeartBeat;


void NRF24L01_SPI_Init(void);
void NRF24L01_RX_Mode(void);                                              //配置为接收模式
void NRF24L01_TX_Mode(void);                                              //配置为发送模式
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts); //写数据区
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);  //读数据区
uint8_t NRF24L01_Read_Reg(uint8_t reg);                                   //读寄存器
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);                   //写寄存器
uint8_t NRF24L01_Check(void);                                             //检查24L01是否存在
uint8_t NRF24L01_TxPacket(uint8_t *txbuf);                                //发送一个包的数据
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);                                //接收一个包的数据

uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t byte);


#endif

