/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usart.h"
#include "can_rec.h"
#include "gpio.h"
#include "printf.h"
#include "tim.h"
#include "pid.h"
#include "steering_wheel.h"
#include "laser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
OmniWheel FourWheel;
Target FourTarget;
FinalVelocity Fourv;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ABS(x)  ((x>0)? (x): -(x))

#define CAMERA_FACTOR  1

pointStruct target = {651,75.5f,0};

int32_t A1AutoFlag = 0;

int32_t AutoVisionFlag = 0;

int32_t clawHoldFlag = 0;

int8_t flag = 0;

float yawAngle = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint8_t null;

/* USER CODE END Variables */
osThreadId DebugTaskHandle;
osThreadId RiseAndYawTaskHandle;
osThreadId LEDTaskHandle;
osThreadId ClawTaskHandle;
osThreadId A1SPDLimitTasHandle;
osThreadId manualCrolTaskHandle;
osThreadId speedCtrlTaskHandle;
osThreadId heartBeatTaskHandle;
osThreadId laserLocTaskHandle;
osThreadId NRF_ReadHandle;
osMessageQId ResShootSpeedQueueHandle;
osMessageQId ResShootSignalQueueHandle;
osMessageQId ResClawPosQueueHandle;
osMessageQId A1ReadyQueueHandle;
osMessageQId NRF_RXQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebugTask(void const * argument);
void RiseAndYaw(void const * argument);
void LED(void const * argument);
void Claw(void const * argument);
void A1SpeedLimit(void const * argument);
void manualCrol(void const * argument);
void speedCtrl(void const * argument);
void heartBeat(void const * argument);
void laserLoc(void const * argument);
void NRFTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of ResShootSpeedQueue */
  osMessageQDef(ResShootSpeedQueue, 1, uint8_t);
  ResShootSpeedQueueHandle = osMessageCreate(osMessageQ(ResShootSpeedQueue), NULL);

  /* definition and creation of ResShootSignalQueue */
  osMessageQDef(ResShootSignalQueue, 1, uint8_t);
  ResShootSignalQueueHandle = osMessageCreate(osMessageQ(ResShootSignalQueue), NULL);

  /* definition and creation of ResClawPosQueue */
  osMessageQDef(ResClawPosQueue, 1, uint8_t);
  ResClawPosQueueHandle = osMessageCreate(osMessageQ(ResClawPosQueue), NULL);

  /* definition and creation of A1ReadyQueue */
  osMessageQDef(A1ReadyQueue, 1, uint8_t);
  A1ReadyQueueHandle = osMessageCreate(osMessageQ(A1ReadyQueue), NULL);

  /* definition and creation of NRF_RXQueue */
  osMessageQDef(NRF_RXQueue, 1, RemoteRXSturct);
  NRF_RXQueueHandle = osMessageCreate(osMessageQ(NRF_RXQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DebugTask */
  osThreadDef(DebugTask, StartDebugTask, osPriorityLow, 0, 256);
  DebugTaskHandle = osThreadCreate(osThread(DebugTask), NULL);

  /* definition and creation of RiseAndYawTask */
  osThreadDef(RiseAndYawTask, RiseAndYaw, osPriorityAboveNormal, 0, 512);
  RiseAndYawTaskHandle = osThreadCreate(osThread(RiseAndYawTask), NULL);

  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, LED, osPriorityLow, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of ClawTask */
  osThreadDef(ClawTask, Claw, osPriorityAboveNormal, 0, 512);
  ClawTaskHandle = osThreadCreate(osThread(ClawTask), NULL);

  /* definition and creation of A1SPDLimitTas */
  osThreadDef(A1SPDLimitTas, A1SpeedLimit, osPriorityHigh, 0, 256);
  A1SPDLimitTasHandle = osThreadCreate(osThread(A1SPDLimitTas), NULL);

  /* definition and creation of manualCrolTask */
  osThreadDef(manualCrolTask, manualCrol, osPriorityRealtime, 0, 512);
  manualCrolTaskHandle = osThreadCreate(osThread(manualCrolTask), NULL);

  /* definition and creation of speedCtrlTask */
  osThreadDef(speedCtrlTask, speedCtrl, osPriorityRealtime, 0, 1024);
  speedCtrlTaskHandle = osThreadCreate(osThread(speedCtrlTask), NULL);

  /* definition and creation of heartBeatTask */
  osThreadDef(heartBeatTask, heartBeat, osPriorityHigh, 0, 128);
  heartBeatTaskHandle = osThreadCreate(osThread(heartBeatTask), NULL);

  /* definition and creation of laserLocTask */
  osThreadDef(laserLocTask, laserLoc, osPriorityHigh, 0, 512);
  laserLocTaskHandle = osThreadCreate(osThread(laserLocTask), NULL);

  /* definition and creation of NRF_Read */
  osThreadDef(NRF_Read, NRFTask, osPriorityHigh, 0, 256);
  NRF_ReadHandle = osThreadCreate(osThread(NRF_Read), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

    vTaskSuspend(A1SPDLimitTasHandle);
    vTaskSuspend(manualCrolTaskHandle);
    vTaskSuspend(speedCtrlTaskHandle);
    vTaskSuspend(heartBeatTaskHandle);
    vTaskSuspend(ClawTaskHandle);
    vTaskSuspend(RiseAndYawTaskHandle);
//    vTaskSuspend(NRF_ReadHandle);

  /* add threads, ... */
  //挂起任务，等接收到nrf消息再进来
    vTaskSuspend(laserLocTaskHandle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDebugTask */
/**
  * @brief  Function implementing the DebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void const * argument)
{
  /* USER CODE BEGIN StartDebugTask */

  /*
   * 以下代码为使用遥控器对nrf进行初始化自检，只实验跑点时请注释
   */
    while(NRF24L01_Check())
    {
//        NRF24L01_RX_Mode();
    }
    NRF24L01_RX_Mode();
    usart_printf("NRF READY!\r\n");
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    CLAW_OPEN();

/*    while(NRF24L01_Check())
    {
        NRF24L01_RX_Mode();
    }
    usart_printf("NRF READY!\r\n");*/
    vTaskResume(speedCtrlTaskHandle);
    vTaskResume(heartBeatTaskHandle);
//    vTaskResume(NRF_ReadHandle);
    /* Infinite loop */
    for(;;)
    {
//        usart_printf("%f,%f,%f,%f\n",locater.pos_x,locater.pos_y,Fourv.v_r,locater.angle);
        osDelay(50);
        LED1_Flashing;
    }
  /* USER CODE END StartDebugTask */
}

/* USER CODE BEGIN Header_RiseAndYaw */
/**
* @brief Function implementing the RiseAndYawTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RiseAndYaw */
void RiseAndYaw(void const * argument)
{
  /* USER CODE BEGIN RiseAndYaw */

    /* Infinite loop */
    for(;;)
    {

        osDelay(1);
    }
  /* USER CODE END RiseAndYaw */
}

/* USER CODE BEGIN Header_LED */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED */
void LED(void const * argument)
{
  /* USER CODE BEGIN LED */
  /* Infinite loop */
  for(;;)
  {
      LED0_Flashing;
      osDelay(500);
  }
  /* USER CODE END LED */
}

/* USER CODE BEGIN Header_Claw */
/**
* @brief Function implementing the ClawTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Claw */
void Claw(void const * argument)
{
  /* USER CODE BEGIN Claw */

    /* Infinite loop */
  for(;;)
  {
      osDelay(1);
  }
  /* USER CODE END Claw */
}

/* USER CODE BEGIN Header_A1SpeedLimit */
/**
* @brief Function implementing the A1SPDLimitTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_A1SpeedLimit */
void A1SpeedLimit(void const * argument)
{
  /* USER CODE BEGIN A1SpeedLimit */
    /** 最大转速为1000*step/delay(plus/s) **/

    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
  /* USER CODE END A1SpeedLimit */
}

/* USER CODE BEGIN Header_manualCrol */
/**
* @brief Function implementing the manualCrolTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_manualCrol */
void manualCrol(void const * argument)
{
  /* USER CODE BEGIN manualCrol */

    /* Infinite loop */

    for(;;)
    {

        osDelay(10);
    }
  /* USER CODE END manualCrol */
}

/* USER CODE BEGIN Header_speedCtrl */
/**
* @brief Function implementing the speedCtrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_speedCtrl */
void speedCtrl(void const * argument)
{
  /* USER CODE BEGIN speedCtrl */
    RemoteRXSturct nrfRX;

    PID_TypeDef PID_x;
    PID_TypeDef PID_y;
    PID_TypeDef PID_r;

    pid_param_init(&PID_x,26,200,10,20,0,0,0.004f,0.0f,0.055f);
    pid_param_init(&PID_y,26,200,10,20,0,0,0.004f,0.0f,0.055f);
    pid_param_init(&PID_r,26,200,10,20,0,0,0.004f,0.0f,0.055f);

    /* Infinite loop */
    for(;;)
    {
//        xQueuePeek(NRF_RXQueueHandle, &nrfRX, 0);
        Run_Point(&PID_x,&PID_y,&PID_r,500.0f,500.0f,0.0f,360.0f,1);
//
        osDelay(10);
    }     /* USER CODE END speedCtrl */
}

/* USER CODE BEGIN Header_heartBeat */
/**
* @brief Function implementing the heartBeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_heartBeat */
void heartBeat(void const * argument)
{
  /* USER CODE BEGIN heartBeat */
  /* Infinite loop */
  for(;;)
  {
      if(NRFHeartBeat == 0)
      {
          //vTaskSuspend();
          //具体安全防范措施都在speedCtrl任务里执行，就他最危险
      }
      else if(NRFHeartBeat > 0)
      {
          NRFHeartBeat--;
      }
    osDelay(20);
  }
  /* USER CODE END heartBeat */
}

/* USER CODE BEGIN Header_laserLoc */
/**
* @brief Function implementing the laserLocTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_laserLoc */
void laserLoc(void const * argument)
{
  /* USER CODE BEGIN laserLoc */
    int32_t counting=0;
    int32_t tomerCounting=0;
    pid_param_init(&laser_loc_pid,8,0.5f,6,2,0,0,0.7f,0.15f,0);
  /* Infinite loop */
    for (;;)
    {
        //自动调整位置，正对墙面
        counting=0;
        tomerCounting = 0;
        do {
            tomerCounting++;
            Laser_Loc();
            osDelay(20);
            if(ABS(laser_loc_data[difference]) <= 0.5f)
                counting++;
            //usart_printf("%f,%f\n",laser_loc_data[difference],laser_loc_data[distance]);
        }while ( counting < 10 || tomerCounting > 250);

        laser_loc_pid.iout = 0;
        //检测是否靠墙角,两个位置需要不同的定位基准
        if (laser_data_23.float_16[0] <= 110.0f)
        {
            //小于50cm视为在墙角
            locater.pos_y_base += 600 - 24 - laser_data_23.float_16[0] - locater.pos_y;
            locater.pos_x_base += 750 + 24.5f + laser_loc_data[distance] - locater.pos_x;
        }
        else
        {
            locater.pos_y_base += 450 - laser_loc_data[distance] - locater.pos_y;
        }

        target.x = locater.pos_x;
        target.y = locater.pos_y;
        target.angle = locater.continuousAngle;
        //usart_printf("%f,%f,%f,%f\n",locater.pos_x_base,locater.pos_y_base,laser_loc_data[distance],laser_data_23.float_16[0]);
        vTaskResume(speedCtrlTaskHandle);
        vTaskSuspend(laserLocTaskHandle);
    }
  /* USER CODE END laserLoc */
}

/* USER CODE BEGIN Header_NRFTask */
/**
* @brief Function implementing the NRF_Read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NRFTask */
void NRFTask(void const * argument)
{
  /* USER CODE BEGIN NRFTask */
    static uint8_t rc_data[RX_PLOAD_WIDTH] = {0};     //遥控器控制数组

    rc_data[8] = 100;
    rc_data[9] = 127;
    /* Infinite loop */
  for(;;)
  {
      usart_printf("1\n");
      if(NRF24L01_RxPacket(rc_data) == 0)
      {
          usart_printf("2\n");
          static RemoteRXSturct RemoteRX;
          //获取摇杆电位值和按键值
          RemoteRX.lx = rc_data[0] - 128;
          RemoteRX.ly = rc_data[1] - 128;
          RemoteRX.rx = rc_data[2] - 128;
          RemoteRX.ry = rc_data[3] - 128;
          RemoteRX.command = rc_data[4];
          RemoteRX.status = rc_data[5];
          RemoteRX.speed = rc_data[6];
          RemoteRX.height = rc_data[7];
          RemoteRX.yaw = rc_data[8] - 100;
          RemoteRX.midValue = (uint16_t)rc_data[9] - 127;
          midValue = VISION_MIDVALUE + RemoteRX.midValue;

          if(ABS(RemoteRX.rx) < 10) RemoteRX.rx = 0;
          if(ABS(RemoteRX.ry) < 10) RemoteRX.ry = 0;
          if(ABS(RemoteRX.lx) < 10) RemoteRX.lx = 0;

          if(RemoteRX.rx > 100) RemoteRX.rx = 100;
          if(RemoteRX.ry > 100) RemoteRX.ry = 100;
          if(RemoteRX.lx > 100) RemoteRX.lx = 100;

          if(RemoteRX.rx < -100) RemoteRX.rx = -100;
          if(RemoteRX.ry < -100) RemoteRX.ry = -100;
          if(RemoteRX.lx < -100) RemoteRX.lx = -100;

          RemoteRX.lx = -RemoteRX.lx;

          NRFHeartBeat = 30;

          //把摇杆电位值发送到摇杆队列中
          xQueueOverwrite(NRF_RXQueueHandle, &RemoteRX);//向队列中发送遥控器摇杆电位值

          /*获取按键值以及判断按键值的变化，并发送到遥控器队列中*/
          //xQueueOverwriteFromISR(rc_keyHandle, &key_press, 0);     //向队列中发送遥控器按键值

          /*在中断中判断按键键值的变化，直接执行一些简单的操作*/

          /*使用can1向底盘的板子发送遥控器的数据8*/
          //send_rc_data_to_chassis(rc_data);

          //usart_printf("%d %d %d %d %d %d\n\r",RemoteRX.lx,RemoteRX.ly,RemoteRX.rx,RemoteRX.ry,RemoteRX.command,RemoteRX.status);

      }
      else
      {
          NRF24L01_RX_Mode();
      }
       osDelay(70);
  }
  /* USER CODE END NRFTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
