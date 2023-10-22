/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
    uint16_t crc_val;
    if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)==SET)
    {
        //清标志
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        //关dma
        HAL_UART_DMAStop(&huart2);
        //收数据
        HAL_UART_Receive_DMA(&huart2, encoder_data_u2, ENCODER_DATA_SIZE);
        //算crc校验
        crc_val = GetModbusCRC16_Tab(encoder_data_u2, 7);
//        算对了就存
        if(crc_val == ((encoder_data_u2[7] << 8) | (encoder_data_u2[8])))
        {
            //数据传递
            angle_feedback[encoder_left].angle_last = angle_feedback[encoder_left].angle_now;
            //原始数据
            angle_feedback[encoder_left].angle_now = (float)((encoder_data_u2[3] << 24) | (encoder_data_u2[4] << 16) | (encoder_data_u2[5] << 8) | encoder_data_u2[6]);
            //转换为0~360的角度值
            angle_feedback[encoder_left].angle_now = -(angle_feedback[encoder_left].angle_now / 0x40000 * 360 - zero_left);
            //判断圈数是否增加
            if(angle_feedback[encoder_left].msg_cnt)
            {
                if(angle_feedback[encoder_left].angle_now - angle_feedback[encoder_left].angle_last > 180.0f)
                    angle_feedback[encoder_left].round--;
                if(angle_feedback[encoder_left].angle_now - angle_feedback[encoder_left].angle_last < -180.0f)
                    angle_feedback[encoder_left].round++;
            }
            angle_feedback[encoder_left].msg_cnt++;
            //计算总角度
            angle_feedback[encoder_left].total_angle = angle_feedback[encoder_left].angle_now + (float)(angle_feedback[encoder_left].round * 360);
            //发送到队列中
            xQueueOverwriteFromISR(angle_leftHandle, &angle_feedback[encoder_left].total_angle, 0);
        }
    }
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
    uint16_t crc_val;
    if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)==SET)
    {
        //清标志
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        //关dma
        HAL_UART_DMAStop(&huart3);
        //收数据
        HAL_UART_Receive_DMA(&huart3, encoder_data_u3, ENCODER_DATA_SIZE);
        //算crc校验
        crc_val= GetModbusCRC16_Tab(encoder_data_u3, 7);
        //算对了就存
        if(crc_val == ((encoder_data_u3[7] << 8) | (encoder_data_u3[8])))
        {
            //数据传递
            angle_feedback[1].angle_last = angle_feedback[1].angle_now;
            //原始数据
            angle_feedback[1].angle_now = (float)((encoder_data_u3[3] << 24) | (encoder_data_u3[4] << 16) | (encoder_data_u3[5] << 8) | encoder_data_u3[6]);
            //转换为0~360的角度值
            angle_feedback[1].angle_now = -(angle_feedback[1].angle_now / 0x40000 * 360 - zero_front);
            //判断圈数是否增加
            if(angle_feedback[1].msg_cnt)
            {
                if(angle_feedback[1].angle_now - angle_feedback[1].angle_last > 180.0f)
                    angle_feedback[1].round--;
                if(angle_feedback[1].angle_now - angle_feedback[1].angle_last < -180.0f)
                    angle_feedback[1].round++;
            }
            angle_feedback[1].msg_cnt++;
            //计算总角度
            angle_feedback[1].total_angle = angle_feedback[1].angle_now + (float)(angle_feedback[1].round * 360);
            //发送到队列中
            xQueueOverwriteFromISR(angle_frontHandle, &angle_feedback[1].total_angle, 0);
        }
    }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
    uint16_t crc_val;
    if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)==SET)
    {
        //清标志
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        //关dma
        HAL_UART_DMAStop(&huart6);
        //收数据
        HAL_UART_Receive_DMA(&huart6, encoder_data_u6, ENCODER_DATA_SIZE);
        //算crc校验
        crc_val = GetModbusCRC16_Tab(encoder_data_u6, 7);
        //算对了就存
         if(crc_val == ((encoder_data_u6[7] << 8) | (encoder_data_u6[8])))
        {
            //数据传递
            angle_feedback[2].angle_last = angle_feedback[2].angle_now;
            //原始数据
            angle_feedback[2].angle_now = (float)((encoder_data_u6[3] << 24) | (encoder_data_u6[4] << 16) | (encoder_data_u6[5] << 8) | encoder_data_u6[6]);
            //转换为0~360的角度值
            angle_feedback[2].angle_now = -(angle_feedback[2].angle_now / 0x40000 * 360 - zero_right);
            //判断圈数是否增加
            if(angle_feedback[2].msg_cnt)
            {
                if(angle_feedback[2].angle_now - angle_feedback[2].angle_last > 180.0f)
                    angle_feedback[2].round--;
                if(angle_feedback[2].angle_now - angle_feedback[2].angle_last < -180.0f)
                    angle_feedback[2].round++;
            }
            angle_feedback[2].msg_cnt++;
            //计算总角度
            angle_feedback[2].total_angle = angle_feedback[2].angle_now + (float)(angle_feedback[2].round * 360);
            //发送到队列中
            xQueueOverwriteFromISR(angle_rightHandle, &angle_feedback[2].total_angle, 0);
        }
    }
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
