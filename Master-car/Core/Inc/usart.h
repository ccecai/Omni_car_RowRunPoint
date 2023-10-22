/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define A1_SEND_BUFSIZE     128 //发送不是很需要限制
#define A1_RECEIVE_BUFSIZE  78  //A1电机的反馈报文78字节

#define VISION_DATA_SIZE 5
#define VISION_STRUCT_NUM 3
#define VISION_MIDVALUE 328

typedef struct
{
    uint8_t ID;
    int16_t offest;
    uint16_t distance;
}VisionStructTypedef;

typedef union
{
    char str[3];
    uint16_t midValue;
}MidValueTransUnion;

extern uint8_t a1_send_buf[A1_SEND_BUFSIZE];   //向A1电机发送的数组
extern uint8_t a1_data_buf[A1_RECEIVE_BUFSIZE];    //A1电机反馈报文的存放数组

extern uint8_t vision_buf[VISION_DATA_SIZE];
extern VisionStructTypedef visionData[VISION_STRUCT_NUM];

#define LOCATER_DATA_SIE 15
extern uint8_t locater_data[LOCATER_DATA_SIE];



/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

