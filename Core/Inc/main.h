/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
void _Error_Handler(const char *file, size_t line);
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FENCE_6B_Pin GPIO_PIN_1
#define FENCE_6B_GPIO_Port GPIOC
#define FENCE_7_Pin GPIO_PIN_2
#define FENCE_7_GPIO_Port GPIOC
#define FENCE_8A_Pin GPIO_PIN_3
#define FENCE_8A_GPIO_Port GPIOC
#define B_PushButton_Pin GPIO_PIN_0
#define B_PushButton_GPIO_Port GPIOA
#define FENCE_8B_Pin GPIO_PIN_11
#define FENCE_8B_GPIO_Port GPIOB
#define FENCE_6A_Pin GPIO_PIN_13
#define FENCE_6A_GPIO_Port GPIOB
#define B_GreenLED_Pin GPIO_PIN_12
#define B_GreenLED_GPIO_Port GPIOD
#define B_OrangeLED_Pin GPIO_PIN_13
#define B_OrangeLED_GPIO_Port GPIOD
#define B_RedLED_Pin GPIO_PIN_14
#define B_RedLED_GPIO_Port GPIOD
#define B_BlueLED_Pin GPIO_PIN_15
#define B_BlueLED_GPIO_Port GPIOD

#define Rotate_Gerege_GPIO_Port GPIOA
#define Rotate_Gerege_Pin GPIO_PIN_15
#define Gerege_GPIO_Port GPIOB
#define Gerege_Pin GPIO_PIN_5
#define Grip_GPIO_Port GPIOB
#define Grip_Pin GPIO_PIN_7
#define Extend_GPIO_Port GPIOC
#define Extend_Pin GPIO_PIN_11
#define Shoot_GPIO_Port GPIOD
#define Shoot_Pin GPIO_PIN_4

#define Extend_Shoot_GPIO_Port GPIOD
#define Extend_Shoot_Pin GPIO_PIN_0

/*
        Grip 2 B7
        Shoot 1 D4
        Shoot_Extend 3 D0
        Extend_Platform 7 C11
        Gerege 6 B5
*/

#define THROWING_SWITCH_Pin GPIO_PIN_5
#define THROWING_SWITCH_GPIO_Port GPIOE
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
