/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
extern uint8_t aRxBuffer,aRxBuffer1,aRxBuffer2,aRxBuffer3,flag2;
extern uint8_t buffer[30],buffer1[30],buffer2[20],buffer3[30];

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
#define IST_INT_Pin GPIO_PIN_3
#define IST_INT_GPIO_Port GPIOE
#define IST_RST_Pin GPIO_PIN_2
#define IST_RST_GPIO_Port GPIOE
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG
#define KEY1_Pin GPIO_PIN_7
#define KEY1_GPIO_Port GPIOD
#define XW_2_Pin GPIO_PIN_4
#define XW_2_GPIO_Port GPIOE
#define sda_Pin GPIO_PIN_7
#define sda_GPIO_Port GPIOI
#define scl_Pin GPIO_PIN_2
#define scl_GPIO_Port GPIOI
#define shejian_Pin GPIO_PIN_9
#define shejian_GPIO_Port GPIOI
#define XW_1_Pin GPIO_PIN_0
#define XW_1_GPIO_Port GPIOF
#define XW_0_Pin GPIO_PIN_1
#define XW_0_GPIO_Port GPIOF
#define XW_4_Pin GPIO_PIN_10
#define XW_4_GPIO_Port GPIOF
#define DIR_1_Pin GPIO_PIN_4
#define DIR_1_GPIO_Port GPIOA
#define DIR_2_Pin GPIO_PIN_5
#define DIR_2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOE
#define XW_3_Pin GPIO_PIN_12
#define XW_3_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
