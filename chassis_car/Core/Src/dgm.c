//
// Created by Yanzhenbo on 2023/4/1.
//

#include "dgm.h"
#include "can_rec.h"

void DGM_SET_TARGET_VELOCITY(CAN_HandleTypeDef *_hcan, uint16_t dgm_id, float target_speed)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t mbox;         //发送使用到的can邮箱
    union_32 txdata;

    TxHeader.StdId = dgm_id<<6 | CAN_CMD_SET_VELOCITY;	         // id赋值
    TxHeader.IDE = 0;                     // 标准帧
    TxHeader.RTR = 0;                     //
    TxHeader.DLC = 4;                     // 8字节数据帧

    txdata.data_f = target_speed;

    //等一个空の邮箱呢
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //发送成功了吗？失败就卡住了捏
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, &txdata.data_8[0], &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void DGM_CAN_CMD_MOTOR_ENABLE(CAN_HandleTypeDef *_hcan, uint16_t dgm_id)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t mbox;         //发送使用到的can邮箱
    union_32 txdata;

    TxHeader.StdId = dgm_id<<6 | CAN_CMD_MOTOR_ENABLE;	         // id赋值
    TxHeader.IDE = 0;                     //标准帧
    TxHeader.RTR = 0;                     //
    TxHeader.DLC = 0;

    //等一个空の邮箱呢
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //发送成功了吗？失败就卡住了捏
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, &txdata.data_8[0], &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
