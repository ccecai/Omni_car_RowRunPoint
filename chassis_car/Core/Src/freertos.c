/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "steering.h"
#include "retarget.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId DebugTaskHandle;
osThreadId AngleTaskHandle;
osThreadId SpeedTaskHandle;
osThreadId HeartbeatTaskHandle;
osMessageQId angle_frontHandle;
osMessageQId angle_leftHandle;
osMessageQId angle_rightHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Debug(void const * argument);
void Angle(void const * argument);
void Speed(void const * argument);
void Heartbeat(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of angle_front */
  osMessageQDef(angle_front, 1, float);
  angle_frontHandle = osMessageCreate(osMessageQ(angle_front), NULL);

  /* definition and creation of angle_left */
  osMessageQDef(angle_left, 1, float);
  angle_leftHandle = osMessageCreate(osMessageQ(angle_left), NULL);

  /* definition and creation of angle_right */
  osMessageQDef(angle_right, 1, float);
  angle_rightHandle = osMessageCreate(osMessageQ(angle_right), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DebugTask */
  osThreadDef(DebugTask, Debug, osPriorityLow, 0, 1024);
  DebugTaskHandle = osThreadCreate(osThread(DebugTask), NULL);

  /* definition and creation of AngleTask */
  osThreadDef(AngleTask, Angle, osPriorityNormal, 0, 1024);
  AngleTaskHandle = osThreadCreate(osThread(AngleTask), NULL);

  /* definition and creation of SpeedTask */
  osThreadDef(SpeedTask, Speed, osPriorityNormal, 0, 1024);
  SpeedTaskHandle = osThreadCreate(osThread(SpeedTask), NULL);

  /* definition and creation of HeartbeatTask */
  osThreadDef(HeartbeatTask, Heartbeat, osPriorityHigh, 0, 1024);
  HeartbeatTaskHandle = osThreadCreate(osThread(HeartbeatTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    vTaskSuspend(SpeedTaskHandle);
    vTaskSuspend(AngleTaskHandle);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Debug */
/**
  * @brief  Function implementing the DebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Debug */
void Debug(void const * argument)
{
  /* USER CODE BEGIN Debug */
//    osDelay(1000);
//    osDelay(1000);
//    osDelay(1000);
//    DGM_CAN_CMD_MOTOR_ENABLE(&hcan2, DGM_FRONT_ID);
//    DGM_CAN_CMD_MOTOR_ENABLE(&hcan2, DGM_LEFT_ID);
//    DGM_CAN_CMD_MOTOR_ENABLE(&hcan2, DGM_RIGHT_ID);
//    osDelay(10);
//    DGM_CAN_CMD_MOTOR_ENABLE(&hcan2, DGM_FRONT_ID);
//    DGM_CAN_CMD_MOTOR_ENABLE(&hcan2, DGM_LEFT_ID);
//    DGM_CAN_CMD_MOTOR_ENABLE(&hcan2, DGM_RIGHT_ID);
//    osDelay(10);
//    DGM_CAN_CMD_MOTOR_ENABLE(&hcan2, DGM_FRONT_ID);
//    DGM_CAN_CMD_MOTOR_ENABLE(&hcan2, DGM_LEFT_ID);
//    DGM_CAN_CMD_MOTOR_ENABLE(&hcan2, DGM_RIGHT_ID);
//    osDelay(10);

//    //demo版本 固定写每个轮子的角度值和速度值
//    speed_front = 1;
////    speed_left = 1;
////    speed_right = 1;
//    angle_front_target = 90 ;
////    angle_left_target =90 ;
////    angle_right_target = 90 ;

    vTaskResume(AngleTaskHandle);
    vTaskResume(SpeedTaskHandle);
    vTaskSuspend(HeartbeatTaskHandle);
//    vTaskSuspend(DebugTaskHandle);
  /* Infinite loop */
  for(;;)
  {
//      usart_printf("%f,%f,%f\r\n",angle_feedback[0].total_angle,angle_feedback[1].total_angle,angle_feedback[2].total_angle);
//      usart_printf("%d\r\n",motor_info[0].speed);
      osDelay(100);
  }
  /* USER CODE END Debug */
}

/* USER CODE BEGIN Header_Angle */
/**
* @brief Function implementing the AngleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Angle */
void Angle(void const * argument)
{
  /* USER CODE BEGIN Angle */
    float angle_front, angle_left, angle_right,angle_back;
    float angle_out_front, angle_out_left, angle_out_right,angle_out_back;
    int16_t speed_out_front , speed_out_left, speed_out_right,speed_out_back;

    PID_Set(&PID_SPD_FRONT,3.0f,0.03f,0,30000,0);
    PID_Set(&PID_ANGLE_FRONT,300.0f,0,0,0,0);

    PID_Set(&PID_SPD_LEFT,3.0f,0.03f,0,30000,0);
    PID_Set(&PID_ANGLE_LEFT,300.0f,0,0,0,0);

    PID_Set(&PID_SPD_RIGHT,3.0f,0.03f,0,30000,0);
    PID_Set(&PID_ANGLE_RIGHT,300.0f,0.0f,0,0,0);

    PID_Set(&PID_SPD_BACK,3.0f,0.03f,0,30000,0);
    PID_Set(&PID_ANGLE_BACK,300.0f,0.0f,0,0,0);
  /* Infinite loop */
  for(;;)
  {

//      if(xQueueReceive(angle_frontHandle,&angle_front,0) == pdTRUE)
//      {
//          angle_out_front = PID_realise(&PID_ANGLE_FRONT,angle_front_target,angle_front,30000);  // 目标值来自can接收主控的  实际值是编码器通过串口读取
//          speed_out_front = (int16_t)PID_realise(&PID_SPD_FRONT,1000,motor_info[0].speed,M2006_CURRENT_MAX);
//      }
//      if(xQueueReceive(angle_leftHandle,&angle_left,0) == pdTRUE)
//      {
//          angle_out_left = PID_realise(&PID_ANGLE_LEFT,angle_left_target,angle_left,30000);
//          speed_out_left = (int16_t)PID_realise(&PID_SPD_LEFT,1000,motor_info[1].speed,M2006_CURRENT_MAX);
//      }
//
//      if(xQueueReceive(angle_rightHandle,&angle_right,0) == pdTRUE)
//      {
//          angle_out_right = PID_realise(&PID_ANGLE_RIGHT,angle_right_target,angle_right,30000);
//          speed_out_right = (int16_t)PID_realise(&PID_SPD_RIGHT,1000,motor_info[2].speed,M2006_CURRENT_MAX);
//      }

      speed_out_front = (int16_t)PID_realise(&PID_SPD_FRONT,speed_front,motor_info[0].speed,M2006_CURRENT_MAX);
      speed_out_left = (int16_t)PID_realise(&PID_SPD_LEFT,speed_left,motor_info[1].speed,M2006_CURRENT_MAX);
      speed_out_right = (int16_t)PID_realise(&PID_SPD_RIGHT,speed_right,motor_info[2].speed,M2006_CURRENT_MAX);
      speed_out_back = (int16_t)PID_realise(&PID_SPD_BACK,speed_back,motor_info[3].speed,M2006_CURRENT_MAX);

//      speed_out_front = (int16_t)PID_realise(&PID_SPD_FRONT,0,motor_info[0].speed,M2006_CURRENT_MAX);
//      speed_out_left = (int16_t)PID_realise(&PID_SPD_LEFT,-1000,motor_info[1].speed,M2006_CURRENT_MAX);
//      speed_out_right = (int16_t)PID_realise(&PID_SPD_RIGHT,0,motor_info[2].speed,M2006_CURRENT_MAX);
//      speed_out_back = (int16_t)PID_realise(&PID_SPD_BACK,0,motor_info[3].speed,M2006_CURRENT_MAX);

      set_current(&hcan2,0x200,speed_out_front,speed_out_left,speed_out_right,speed_out_back);
//      set_current(&hcan2,0x200,800,800,800,800);
      osDelay(5);
  }
  /* USER CODE END Angle */
}

/* USER CODE BEGIN Header_Speed */
/**
* @brief Function implementing the SpeedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Speed */
void Speed(void const * argument)
{
  /* USER CODE BEGIN Speed */
  /* Infinite loop */
  for(;;)
  {
//      DGM_SET_TARGET_VELOCITY(&hcan2, DGM_FRONT_ID, speed_front);  // speed只在can接收里被设置
//      DGM_SET_TARGET_VELOCITY(&hcan2, DGM_LEFT_ID, -speed_left);
//      DGM_SET_TARGET_VELOCITY(&hcan2, DGM_RIGHT_ID, speed_right);
      osDelay(5);
  }
  /* USER CODE END Speed */
}

/* USER CODE BEGIN Header_Heartbeat */
/**
* @brief Function implementing the HeartbeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Heartbeat */
void Heartbeat(void const * argument)
{
  /* USER CODE BEGIN Heartbeat */
  /* Infinite loop */
  for(;;)
  {
      if(heartbeat_check() == disable)
      {
          vTaskSuspend(SpeedTaskHandle);
//          DGM_SET_TARGET_VELOCITY(&hcan2, DGM_FRONT_ID, 0.0f);
//          DGM_SET_TARGET_VELOCITY(&hcan2, DGM_LEFT_ID, 0.0f);
//          DGM_SET_TARGET_VELOCITY(&hcan2, DGM_RIGHT_ID, 0.0f);
      }
      else
          vTaskResume(SpeedTaskHandle);
    osDelay(10);
  }
  /* USER CODE END Heartbeat */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
