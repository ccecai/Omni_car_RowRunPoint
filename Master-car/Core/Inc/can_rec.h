#ifndef __CAN_REC_H__
#define __CAN_REC_H__
#include "can.h"


//can指令宏定义
#define COMMAND_SHOOT_SPD_ID        0x100
#define COMMAND_SHOOT_SIGAL_ID      0x101
#define COMMAND_CLAW_POS_ID         0x102

#define RESPOND_ID                  0x0FF
#define RESPOND_SHOOT_SPD           0xF0
#define RESPOND_SHOOT_SIGAL         0xF1
#define RESPOND_CLAW_POS            0xF2

#define STATE_ID                    0x0FE

/*这里是和抓取装置相关指令的宏定义*/
#define CLAW_POS_HEIHEST        -217000       //上升到最高
#define CLAW_POS_LOWEST         0             //下降到最低端
#define CLAW_POS_ROUTINE        -160000       //回到默认位置


/*全局变量的声明*/

/*函数的声明*/

void CANSendData(CAN_HandleTypeDef *_hcan, uint32_t ID, uint8_t length, uint8_t *data);

void SendShootSpeed(int speed);
void SendShootSiganl(void);
void SendClawPostion(int position);
void Set_Omni_Can(CAN_HandleTypeDef* hcan,int v1);


void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan);

#endif /* __CAN_REC_H__ */