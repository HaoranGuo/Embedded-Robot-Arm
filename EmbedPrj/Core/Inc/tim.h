/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#define SERVO0_MAX_ANGLE 270
#define SERVO1_MAX_ANGLE 270
#define SERVO2_MAX_ANGLE 270
#define SERVO3_MAX_ANGLE 270
#define SERVO4_MAX_ANGLE 270
#define SERVO5_MAX_ANGLE 270
#define SERVO6_MAX_ANGLE 270
#define SERVO7_MAX_ANGLE 270

#define USED_SERVO_NUM 6

#define SPECIAL_SERVO_NUM 5

// the more the value is, the slower the servo moves
#define GLOBAL_SERVO_SPEED 5

extern uint16_t servo_pulse[USED_SERVO_NUM];

extern uint16_t servo_pulse_last[USED_SERVO_NUM];

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN Private defines */

void servo_Move(uint8_t servo_num, uint16_t pulse);

void servo_Move2Goal(void);

void servo_Init(void);

/* USER CODE END Private defines */

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

