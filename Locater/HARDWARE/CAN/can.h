#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	     
extern int cnt_round;
extern int angle_offset;
extern int angle_fab;
extern float R;
extern int target_position;


 

	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
										 							 				    
void FOC_CAN1_Mode_Init(void);												
 
u8 CAN1_Send_Msg(u32 ID,u8* msg,u8 len);						

u8 CAN1_Receive_Msg(u8 *buf);					

void Send_to_motor(int Motor_Current1,int Motor_Current2,int Motor_Current3,int Motor_Current4);
u8 Get_value_from_motor(void);
#endif

















