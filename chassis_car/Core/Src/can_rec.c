#include "can_rec.h"
#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "retarget.h"
moto_info_t motor_info[MOTOR_MAX_NUM];       //rm电机返回的数据数组

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
 * @brief can1过滤器初始化，只接运动学解算的三个数据
 */
void can1_filter_init()
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    //can1的过滤器从0开始
    CAN_FilterConfigStructure.FilterBank = 0;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = ((FW<<21)&0xffff0000)>>16;
    CAN_FilterConfigStructure.FilterIdLow = ((FW<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0xffff;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0xffff;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }

    CAN_FilterConfigStructure.FilterBank = 1;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = ((LW<<21)&0xffff0000)>>16;
    CAN_FilterConfigStructure.FilterIdLow = ((LW<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0xffff;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0xffff;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }

    CAN_FilterConfigStructure.FilterBank = 2;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = ((RW<<21)&0xffff0000)>>16;
    CAN_FilterConfigStructure.FilterIdLow = ((RW<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0xffff;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0xffff;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;



    if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }
    CAN_FilterConfigStructure.FilterBank = 3;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = ((BW<<21)&0xffff0000)>>16;
    CAN_FilterConfigStructure.FilterIdLow = ((BW<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0xffff;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0xffff;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }
    /* Start the CAN peripheral */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    /* Activate CAN TX notification */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}
/**
 * @brief can2过滤器初始化，是全部接受
 */
void can2_filter_init()
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterBank = 14;

    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the CAN peripheral */
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    /* Activate CAN TX notification */
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}
/**
  * @brief  使用can发送四个rm电机的电流(堵塞式)
  * @param  id_range 发送id 前四个电机为200，后四个为1FF
  * @param  currentx 该组中第x个电机的电流
  */
void set_current(CAN_HandleTypeDef *_hcan, int16_t id_range, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t mbox;         //发送使用到的can邮箱

    TxHeader.StdId = id_range;	         // id赋值
    TxHeader.IDE = 0;                     // 标准帧
    TxHeader.RTR = 0;                     //
    TxHeader.DLC = 8;                     // 8字节数据帧

    /*数据赋值*/
    TxData[0] = (current1 >> 8) & 0xff;
    TxData[1] = (current1) & 0xff;
    TxData[2] = (current2 >> 8) & 0xff;
    TxData[3] = (current2) & 0xff;
    TxData[4] = (current3 >> 8) & 0xff;
    TxData[5] = (current3) & 0xff;
    TxData[6] = (current4 >> 8) & 0xff;
    TxData[7] = (current4) & 0xff;

    //等一个空の邮箱呢
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //发送成功了吗？失败就卡住了捏
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
/**
 * @brief 对3508的反馈报文进行解码
 * @param ptr 目标电机
 * @param data 8字节can报文
 */
void motor_info_record(moto_info_t *ptr, uint8_t *data)
{
    /*首次上电时获取电机转子的初始位置*/
    if(ptr->msg_cnt == 0)
        ptr->offset_angle = ptr->angle = ((data[0] << 8) | data[1]);    //转子位置
    else
    {
        /*转子信息获取*/
        ptr->last_angle = ptr->angle;
        ptr->angle = ((data[0] << 8) | data[1]);    //转子位置
        ptr->speed = ((data[2] << 8) | data[3]);    //转子速度
        ptr->torque_current = ((data[4] << 8) | data[5]);    //转子扭矩
        ptr->temp = data[6];   //电机温度

        /*判断圈数是否加减*/
        if(ptr->angle - ptr->last_angle > 4095)
            ptr->total_cnt--;
        else if(ptr->angle - ptr->last_angle < -4095)
            ptr->total_cnt++;

        /*计算转过的总角度*/
        ptr->total_angle = ptr->total_cnt * 8192 + ptr->angle - ptr->offset_angle;
    }

    ptr->msg_cnt++;     //消息计数+1
}

/**
  * @brief  这是fifo0的回调函数，作用相当于can的rx0中断
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //接收回调函数
{
    HAL_StatusTypeDef HAL_RetVal;
    CAN_RxHeaderTypeDef RxHeader;
    union_64 rxdata;
    /*电机号记录*/
    static uint8_t index;

    if(hcan == &hcan2)
    {
//        printf("222");
        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxdata.data_8);      //从CAN1接收数据，通过过滤器后放入FIFO0,存入RxMessage数据帧

        if(HAL_RetVal == HAL_OK)
        {

            if(RxHeader.StdId >= 0x201 && RxHeader.StdId <= 0x208)
            {
                index = RxHeader.StdId - 0x201;   //结构体数组0-7对应电机ID1-8
                motor_info_record(&motor_info[index], rxdata.data_8);   //解包
            }
            __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);   //清一下，不然就卡住了
        }
    }
    // demo版本不需要主控

    else if(hcan == &hcan1)  // 收上层板子的发来的数据的
    {
        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxdata.data_8);
        if(HAL_RetVal == HAL_OK)
        {
            if(RxHeader.StdId == FW || RxHeader.StdId == LW || RxHeader.StdId == RW || RxHeader.StdId == BW)
                heartbeat_fresh();
            switch (RxHeader.StdId)
            {
                case FW:
                    speed_front = rxdata.data_f[0];
                    angle_front_target = rxdata.data_f[1];
                    usart_printf("%.2f,%.2f,%.2f,%.2f\n",speed_front,speed_left,speed_right,speed_back);
                    break;
                case LW:
                    speed_left = rxdata.data_f[0];
                    angle_left_target = rxdata.data_f[1];
//                    usart_printf("%.2f,%.2f,%.2f,%.2f\n",speed_front,speed_left,speed_right,speed_back);
                    break;
                case RW:
                    speed_right = rxdata.data_f[0];
                    angle_right_target = rxdata.data_f[1];
                    break;
                case BW:
                    speed_back = rxdata.data_f[0];
                    angle_back_target = rxdata.data_f[1];
                    break;
            }
        }
        __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

}