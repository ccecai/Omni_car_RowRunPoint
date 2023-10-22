/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "printf.h"
#include "gpio.h"
#include "dma.h"
#include "can.h"
#include "24l01.h"
#include "spi.h"
#include "crc32.h"
#include "A1_driver.h"
#include "pid.h"
#include "locater.h"
#include "Omni_Classis_task.h"
//#include "motor_rm.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define SPI1_IQRN_Pin GPIO_PIN_4
#define SPI1_IQRN_GPIO_Port GPIOC
#define SPI1_CE_Pin GPIO_PIN_0
#define SPI1_CE_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_14
#define LED0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOF
#define GATE0_Pin GPIO_PIN_0
#define GATE0_GPIO_Port GPIOG
#define GATE1_Pin GPIO_PIN_1
#define GATE1_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
