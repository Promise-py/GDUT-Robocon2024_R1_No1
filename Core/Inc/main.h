/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define read_ppm_Pin GPIO_PIN_6
#define read_ppm_GPIO_Port GPIOE
#define read_ppm_EXTI_IRQn EXTI9_5_IRQn
#define Absorb_Pin GPIO_PIN_14
#define Absorb_GPIO_Port GPIOB
#define Claw_Pin GPIO_PIN_15
#define Claw_GPIO_Port GPIOB
#define Tripod_Pin GPIO_PIN_14
#define Tripod_GPIO_Port GPIOD
#define Red_Pin GPIO_PIN_6
#define Red_GPIO_Port GPIOC
#define Blue_Pin GPIO_PIN_7
#define Blue_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define x  		0
#define y 		1
#define w  		2
extern float target_current;
extern float Set_target_current;


extern uint8_t RxBuffer_for4[1];//串口接收缓冲
extern uint8_t DataBuff_for4[200];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
