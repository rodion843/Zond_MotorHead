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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "tim.h"
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum
{
	CAN_Package_X_Positive	= 1 << 0,
	CAN_Package_X_Negative	= 1 << 1,
	CAN_Package_Y_Positive	= 1 << 2,
	CAN_Package_Y_Negative  = 1 << 3,
	CAN_Package_All_Stop 	= 1 << 4,
  New_Angle_X		= 1 << 5,
  New_Angle_Y		= 1 << 6,
} CAN_Package;
typedef enum Buttons
{
	LS_Up 		= 1 << 0,
	LS_Down		= 1 << 1,
} Buttons;
typedef enum Direction
{
	Direction_Stop,
	Direction_Positive,
	Direction_Negative
}Direction;

typedef enum {
	Axis_X,
	Axis_Y,
}Axis;

typedef struct _Step_Driver{
	Buttons Buttons;
	Direction Direction;
	Axis Axis;
	float Angle;
}Step_Driver;

Step_Driver Step_X;
Step_Driver Step_Y;

uint8_t UART_DATA[8];
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
Step_Driver Step_Driver_Init(Axis x);
bool HasFlag(uint32_t a, uint32_t b);
void Motor_Parser(CAN_Package *x);
//void Motor_Driver(Motor_CMD cmd);
void Send_Data(Step_Driver* SD);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIR_Y_Pin GPIO_PIN_13
#define DIR_Y_GPIO_Port GPIOC
#define DIAG_Y_Pin GPIO_PIN_14
#define DIAG_Y_GPIO_Port GPIOC
#define INDEX_Y_Pin GPIO_PIN_15
#define INDEX_Y_GPIO_Port GPIOC
#define INDEX_Y_EXTI_IRQn EXTI15_10_IRQn
#define STEP_Y_Pin GPIO_PIN_0
#define STEP_Y_GPIO_Port GPIOA
#define STEP_X_Pin GPIO_PIN_1
#define STEP_X_GPIO_Port GPIOA
#define DAC_Y_Pin GPIO_PIN_4
#define DAC_Y_GPIO_Port GPIOA
#define DAC_X_Pin GPIO_PIN_5
#define DAC_X_GPIO_Port GPIOA
#define Limit_SW_2_Pin GPIO_PIN_6
#define Limit_SW_2_GPIO_Port GPIOA
#define Limit_SW_2_EXTI_IRQn EXTI9_5_IRQn
#define Limit_SW_1_Pin GPIO_PIN_7
#define Limit_SW_1_GPIO_Port GPIOA
#define Limit_SW_1_EXTI_IRQn EXTI9_5_IRQn
#define SPREAD_X_Pin GPIO_PIN_14
#define SPREAD_X_GPIO_Port GPIOB
#define ENABLE_X_Pin GPIO_PIN_15
#define ENABLE_X_GPIO_Port GPIOB
#define DIR_X_Pin GPIO_PIN_8
#define DIR_X_GPIO_Port GPIOA
#define Motor_Power_CTRL_Pin GPIO_PIN_15
#define Motor_Power_CTRL_GPIO_Port GPIOA
#define DIAG_X_Pin GPIO_PIN_3
#define DIAG_X_GPIO_Port GPIOB
#define INDEX_X_Pin GPIO_PIN_4
#define INDEX_X_GPIO_Port GPIOB
#define INDEX_X_EXTI_IRQn EXTI4_IRQn
#define ENABLE_Y_Pin GPIO_PIN_5
#define ENABLE_Y_GPIO_Port GPIOB
#define SPREAD_Y_Pin GPIO_PIN_9
#define SPREAD_Y_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
//Step_X;
//Step_Y;
static inline void SetPrescaler(TIM_HandleTypeDef* htim, uint16_t x){
//	htim->Instance->PSC = (uint16_t)(1000 * (-pow(x, 3) / pow(255, 3) + 1));	// 100 * (-x^3 / 255^3 + 1)
	htim->Instance->PSC = x;	// 100 * (-x^3 / 255^3 + 1)
//	htim->Instance->PSC = (uint16_t)(-(x*x/65535.0 - 2*x + 65535));	// 100 * (-x^3 / 255^3 + 1)
}

static inline void Motor_X_Positive_Go(uint16_t x){
	HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_RESET);
	HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(ENABLE_X_GPIO_Port, ENABLE_X_Pin, GPIO_PIN_RESET);
	Step_X.Direction = Direction_Positive;
	SetPrescaler(&htim15, x);
}
static inline void Motor_X_Negative_Go(uint16_t x){
	HAL_GPIO_WritePin(DIR_X_GPIO_Port, DIR_X_Pin, GPIO_PIN_SET);
	HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(ENABLE_X_GPIO_Port, ENABLE_X_Pin, GPIO_PIN_RESET);
	Step_X.Direction = Direction_Negative;
	SetPrescaler(&htim15, x);
}
static inline void Motor_X_Stop(){
	HAL_GPIO_WritePin(ENABLE_X_GPIO_Port, ENABLE_X_Pin, GPIO_PIN_SET);
	HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1);
	Step_X.Direction = Direction_Stop;
}

static inline void Motor_Y_Positive_Go(uint16_t y){
	HAL_GPIO_WritePin(DIR_Y_GPIO_Port, DIR_Y_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(ENABLE_Y_GPIO_Port, ENABLE_Y_Pin, GPIO_PIN_RESET);
	Step_Y.Direction = Direction_Positive;
	SetPrescaler(&htim2, y);
}
static inline void Motor_Y_Negative_Go(uint16_t y){
	HAL_GPIO_WritePin(DIR_Y_GPIO_Port, DIR_Y_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(ENABLE_Y_GPIO_Port, ENABLE_Y_Pin, GPIO_PIN_RESET);
	Step_Y.Direction = Direction_Negative;
	SetPrescaler(&htim2, y);
}
static inline void Motor_Y_Stop(){
	HAL_GPIO_WritePin(ENABLE_Y_GPIO_Port, ENABLE_Y_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	Step_Y.Direction = Direction_Stop;
}
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
