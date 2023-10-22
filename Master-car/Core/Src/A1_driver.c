//
// Created by 19734 on 2022/12/29.
//
#include "A1_driver.h"
#include "crc32.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"

MOTOR_send motor_s[2];
MOTOR_send motor_s_stop[2];

#define MOTOR_SEND_LENGTH 34    //发送给电机的控制命令是34个字节
char motor_send[MOTOR_SEND_LENGTH] = {0};   //发送数组
char motor_send_stop[MOTOR_SEND_LENGTH] = {0};  //发送停止命令的数组
uint32_t crc_32;

int32_t pos_base_a1[3] = {3000,600,0};                   //电机运动初始位置
int32_t pos_realtime_a1[3];                     //电机运动结束位置

int32_t pos_control_a1[3]; //限速时牵引实际位置到设定位置用的位置环输入位置
int32_t pos_set_a1[3]; //设置电机要转到的位置

//a1电机的控制状态 0：不进行位置控制 1：进行固定目标的位置控制 2：进行摄像头的对正控制(仅yaw电机使用到了)
uint8_t control_staus[3] = {0};


/**
 * @brief 给指定电机的发送报文赋值
 * @param id
 */
void modify_id_data(uint8_t id)
{
    motor_send[0] = 0xFE;
    motor_send[1] = 0xEE;
    motor_send[2] = id;     //电机ID
    motor_send[3] = 0x00;
    motor_send[4] = motor_s[id].mode;
    motor_send[5] = 0xFF;   //设为0xFF，可以避免电机反馈的数据抖动
    motor_send[12] = (int)motor_s[id].T*256;
    motor_send[13] = (int)(motor_s[id].T*256)>>8;
    motor_send[14] = motor_s[id].W*128;
    motor_send[15] = (int)(motor_s[id].W*128)>>8;
    motor_send[16] = motor_s[id].Pos;
    motor_send[17] = motor_s[id].Pos>>8;
    motor_send[18] = motor_s[id].Pos>>16;
    motor_send[19] = motor_s[id].Pos>>24;
    motor_send[20] = motor_s[id].K_P*2048;
    motor_send[21] = (int)(motor_s[id].K_P*2048)>>8;
    motor_send[22] = motor_s[id].K_W*1024;
    motor_send[23] = (int)(motor_s[id].K_W*1024)>>8;
    crc_32 = crc32_core((uint32_t*)motor_send, 7);    //计算crc_32
    //crc_32高八位放在第34个字节
    motor_send[33] = crc_32>>24;
    motor_send[32] = crc_32>>16;
    motor_send[31] = crc_32>>8;
    motor_send[30] = crc_32;
}


/**
 * @brief 控制A1电机转到指定的位置，使用位置式pd单环
 * @param id 电机id a1的id为0 1 2，一个总线上只能挂3个电机
 * @param kp 比例系数
 * @param kw 微分系数
 * @param aim_pos 转子目标位置，以当前位置为0位置，注意电机有9.1的减速比，输出轴位置需要计算
 */
void a1_pos_control(uint8_t id, float kp, float kw, int aim_pos)
{
    float position_error;
    motor_s[id].mode = 10;    //闭环伺服控制
    motor_s[id].T = 0;        //将T和W设成0，可以形成对电机的PD控制，
    motor_s[id].W = 0;        //其中K_P为比例系数，K_W为微分系数
    motor_s[id].K_W = kw;     //
    motor_s[id].K_P = kp;     //
    motor_s[id].Pos = aim_pos + pos_base_a1[id];    //设置目标位置

    modify_id_data(id);     //数据赋值

    HAL_UART_Transmit_DMA(&huart6,(uint8_t *) motor_send,MOTOR_SEND_LENGTH);    //启动一次发送
}

/**
 * @brief 给指定电机发送停转命令，用于获取电机的初始位置，因为只有给电机发送报文，电机才会反馈报文，
 *        所以在上电之初给电机发送停转命令来获取电机初始位置
 * @param id
 */
void motor_stop(uint8_t id)
{
    motor_send_stop[0] = 0xFE;
    motor_send_stop[1] = 0xEE;
    motor_send_stop[2] = id;         //电机ID
    motor_send_stop[3] = 0x00;
    motor_send_stop[4] = 0x00;      //模式为0时表示停转，当该值为0时，后面的控制参数都失去意义
    motor_send_stop[5] = 0xFF;
    crc_32 = crc32_core((uint32_t*)motor_send_stop, 7);    //计算crc_32
    //crc_32高八位放在第34个字节
    motor_send_stop[33] = crc_32 >> 24;
    motor_send_stop[32] = crc_32 >> 16;
    motor_send_stop[31] = crc_32 >> 8;
    motor_send_stop[30] = crc_32;
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *) motor_send_stop, MOTOR_SEND_LENGTH);
}

/**
 * @brief 初始化的时候执行，获取a1电机在上电后的 转子 初始位置
 */
void get_pos_base()
{
    motor_stop(A1_HEGHT_ID);
    motor_stop(A1_YAW_ID);
}

/**
 * @brief 限幅函数，有上下两个限制
 */
int limiting_amplitude(int input, int output_max, int output_min)
{
    if(input > output_max)
        input = output_max;
    else if(input < output_min)
        input = output_min;
    return input;
}

/**
 * @brief 这个是串口接收中断回调函数，使用过程中有点小问题，建议先把自己的接收函数写到f4xx_it.c里的对应中断服务函数中
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {	//判断串口号
//        HAL_UART_Receive_IT(&huart2,&vision_offset,1);  //接收偏移量
//        if(vision_offset)
//        {
//            usart_printf("%d\r\n",vision_offset);
//            xQueueOverwriteFromISR(vision_offset_getHandle,&vision_offset,0);   //向队列中发送摄像头测得的偏移量
//        }

    }
}