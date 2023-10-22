#include "can_rec.h"
#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "laser.h"
#include "Omni_Classis_task.h"

/**
  * @brief  使用can时必须初始化它的过滤器，cubemx不会帮忙初始化这个，同时这个函数会启动这个can的can通信
  *         需要根据自己的收发策略来更改这个函数，默认的接收策略是无条件全部接收can总线上的所有信息
  *         这个函数放在can的初始化程序后面
  * @param  *hcan 要初始化的目标can
  */
void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan)
{
    //can1 &can2 use same filter config
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    if (_hcan == &hcan1)
    {
        CAN_FilterConfigStructure.FilterBank = 0;
    }
    else if (_hcan == &hcan2)
    {
        CAN_FilterConfigStructure.FilterBank = 14;
    }

    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }
    /* Start the CAN peripheral */
    if (HAL_CAN_Start(_hcan) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    /* Activate CAN TX notification */
    if (HAL_CAN_ActivateNotification(_hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}


/**
 * @brief 使用can给下层板子发送指令
 * @param command 无符号整数，8bit
 */
void CANSendData(CAN_HandleTypeDef *_hcan, uint32_t ID, uint8_t length, uint8_t *data)    //向底盘板子发送指令
{
    static CAN_TxHeaderTypeDef TxHeader;    //发送报文结构体定义
    static uint8_t TxData[8];
    static uint32_t mbox;

    TxHeader.StdId = ID;           // id赋值
    TxHeader.IDE=0;                     // 标准帧
    TxHeader.RTR=0;                     //
    TxHeader.DLC=length;                // 1字节数据帧

    //数据赋值
    for(int i = 0; i < length; i++)
    {
        TxData[i] = *(data + i);
    }

    //等一个空邮箱
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //判断是否发送成功
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}


void SendShootSpeed(int speed)
{
    CANSendData(&hcan1, COMMAND_SHOOT_SPD_ID, 4, (uint8_t*)(&speed));
}

void SendShootSiganl(void)
{
    CANSendData(&hcan1, COMMAND_SHOOT_SIGAL_ID, 0, NULL);
}

void SendClawPostion(int position)
{
    CANSendData(&hcan1, COMMAND_CLAW_POS_ID, 4, (uint8_t*)(&position));
}


/*声明一些queue和task*/
extern osMessageQId ResShootSpeedQueueHandle;
extern osMessageQId ResShootSignalQueueHandle;
extern osMessageQId ResClawPosQueueHandle;


/**
  * @brief  这是fifo0的回调函数，作用相当于can的rx0中断
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //接收回调函数
{
    static HAL_StatusTypeDef HAL_RetVal;
    static CAN_RxHeaderTypeDef RxHeader;
    static uint8_t RxData[8] = {0};

    if(hcan == &hcan2)
    {
        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);      //从CAN2接收数据，通过过滤器后放入FIFO0,存入RxMessage数据帧

        if(HAL_RetVal == HAL_OK)
        {
            switch (RxHeader.StdId) {
                case laser_01:
                    for (int i = 0; i < 8; ++i) {
                       laser_data_01.uint_8[i] = RxData[i];
                    }
                    break;
                case laser_23:
                    for (int i = 0; i < 8; ++i) {
                        laser_data_23.uint_8[i] = RxData[i];
                    }
                    break;
                default:
                    break;
            }

        }
        __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);   //清一下，不然就卡住了
    }
    else if(hcan == &hcan1)
    {
        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
        if(HAL_RetVal == HAL_OK)
        {
            switch (RxHeader.StdId)
            {
                case RESPOND_ID:
                    switch (RxData[0])
                    {
                        case RESPOND_SHOOT_SPD:
                            xQueueOverwriteFromISR(ResShootSpeedQueueHandle,RxData,0);
                            break;
                        case RESPOND_SHOOT_SIGAL:
                            xQueueOverwriteFromISR(ResShootSignalQueueHandle,RxData,0);
                            break;
                        case RESPOND_CLAW_POS:
                            xQueueOverwriteFromISR(ResClawPosQueueHandle,RxData,0);
                            break;
                        default:
                            break;
                    }
                    break;
                case STATE_ID:
                    break;

            }
        }
        __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

}