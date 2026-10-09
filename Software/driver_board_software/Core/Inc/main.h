/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * LICENCED under the GNU GENERAL PUBLIC LICENSE
  *
  ******************************************************************************
  */
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
 * See LICENSE-SOFTWARE in the project root for details.
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
#include "SPIconfig.h"
#include "uartconfig.h"
#include "motion.h"
#include <math.h>
#include <string.h>
#include <stdio.h>



// Prototypes
//static void Step_Start(uint8_t);
//static void Step_Stop(uint8_t);
//static void Step_SetFreqHz(uint8_t , uint32_t );
//void UpdateAllRamps(float , uint8_t );
//float ComputeSyncedVmax(float , float , float );
//void StartSyncedMove(float , float , float , uint8_t );
//void StartMove(uint8_t , float , float , float );
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define axis 2


extern volatile uint8_t cmd_ready;  //in uartconfig.c/main.c
extern SPI_HandleTypeDef hspi2;     //in SPIconfig.c/main.c
extern float target_steps[];        //in uartconfig.c/main.c
//extern char rx_buffer[128];         //in uartconfig.c/main.c
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TMC1_CS_Pin GPIO_PIN_0
#define TMC1_CS_GPIO_Port GPIOC
#define TMC2_CS_Pin GPIO_PIN_1
#define TMC2_CS_GPIO_Port GPIOC
#define TMC3_CS_Pin GPIO_PIN_2
#define TMC3_CS_GPIO_Port GPIOC
#define TMC2_STEP_Pin GPIO_PIN_0
#define TMC2_STEP_GPIO_Port GPIOA
#define TMC4_CS_Pin GPIO_PIN_1
#define TMC4_CS_GPIO_Port GPIOA
#define TMC5_CS_Pin GPIO_PIN_4
#define TMC5_CS_GPIO_Port GPIOC
#define TMC6_CS_Pin GPIO_PIN_0
#define TMC6_CS_GPIO_Port GPIOB
#define TMC1_DIR_Pin GPIO_PIN_2
#define TMC1_DIR_GPIO_Port GPIOB
#define TMC5_STEP_Pin GPIO_PIN_6
#define TMC5_STEP_GPIO_Port GPIOC
#define TMC2_DIR_Pin GPIO_PIN_7
#define TMC2_DIR_GPIO_Port GPIOC
#define TMC3_DIR_Pin GPIO_PIN_8
#define TMC3_DIR_GPIO_Port GPIOC
#define TMC6_DIR_Pin GPIO_PIN_9
#define TMC6_DIR_GPIO_Port GPIOC
#define TMC1_STEP_Pin GPIO_PIN_8
#define TMC1_STEP_GPIO_Port GPIOA
#define TMC5_DIR_Pin GPIO_PIN_9
#define TMC5_DIR_GPIO_Port GPIOA
#define TMC4_DIR_Pin GPIO_PIN_10
#define TMC4_DIR_GPIO_Port GPIOA
#define TMC3_STEP_Pin GPIO_PIN_4
#define TMC3_STEP_GPIO_Port GPIOB
#define TMC4_STEP_Pin GPIO_PIN_6
#define TMC4_STEP_GPIO_Port GPIOB
#define TMC6_STEP_Pin GPIO_PIN_8
#define TMC6_STEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
