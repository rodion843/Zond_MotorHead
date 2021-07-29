/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "crc.h"
#include "dac.h"
#include "lptim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
//#include <tgmath.h>
#include <math.h>
#include "TMC2209.h"
#include <string.h>
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

/* USER CODE BEGIN PV */
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_FilterTypeDef CAN_FILTER;

uint32_t System_Freq;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

int32_t IndexCounter_X = 0;
int32_t IndexCounter_Y = 0;
uint32_t Tim2_counter = 0;


volatile uint32_t counter = 0;
uint16_t Prescaler = 0;
uint8_t result;
uint8_t aData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void tmc2209_CRC8(uint8_t* datagram, uint8_t datagramLength);
void CAN_Init(void);
void stopping_sequence(TIM_HandleTypeDef htim);

void SendToTMC2209_X();
void SendToTMC2209_Y();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t debug_fl = 0;
int32_t tmcdata;

uint16_t Speed_x = 1000;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_DAC1_Init();
	MX_LPTIM1_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_TIM15_Init();
	MX_LPTIM2_Init();
	MX_TIM6_Init();
	MX_CRC_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(ENABLE_Y_GPIO_Port, ENABLE_Y_Pin, GPIO_PIN_RESET);
	Step_X = Step_Driver_Init(Axis_X);
	Step_Y = Step_Driver_Init(Axis_Y);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1500);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1500);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	CAN_Init();
	HAL_CAN_Start(&hcan1);
	//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim6);


	//	HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
	HAL_LPTIM_PWM_Start(&hlptim1, 6, 3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(ENABLE_X_GPIO_Port, ENABLE_X_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENABLE_Y_GPIO_Port, ENABLE_Y_Pin, GPIO_PIN_SET);
	TMC2209TypeDef TMC_Y;
	ConfigurationTypeDef TMC_Y_Conf;
	const int32_t reset_reg[128];
	tmc2209_init(&TMC_Y, TMC_UART_Y, 1, &TMC_Y_Conf, reset_reg);
	TMC2209TypeDef TMC_X;
	ConfigurationTypeDef TMC_X_Conf;
	tmc2209_init(&TMC_X, TMC_UART_X, 1, &TMC_X_Conf, reset_reg);

	tmc_fillCRC8Table(0x07, false, 0);

	int32_t tmcdata = 451;

	//	tmc2209_writeInt(&TMC_Y, TMC2209_GCONF, tmcdata);
	//	tmc2209_writeInt(&TMC_X, TMC2209_GCONF, tmcdata);
	while (1)
	{
		//		uint8_t package[8];
		//		memcpy(package, UART_DATA, 8);
		//		memset(UART_DATA, 0, 8);
		//
		if(debug_fl == 1){
			//			SendToTMC2209_X();

			Motor_X_Positive_Go(Speed_x);
			debug_fl = 0;
		}
		else if(debug_fl == 2){
			Motor_X_Negative_Go(Speed_x);
			debug_fl = 0;
		}
		else if(debug_fl == 3){
			Motor_Y_Positive_Go(Speed_x);
			debug_fl = 0;
		}
		else if(debug_fl == 4){
			Motor_Y_Negative_Go(Speed_x);
			debug_fl = 0;
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
			|RCC_PERIPHCLK_LPTIM1|RCC_PERIPHCLK_LPTIM2;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
	PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;
	PeriphClkInit.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_PCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
Step_Driver Step_Driver_Init(Axis a){
	Step_Driver result;
	result.Axis = a;
	return result;
}


uint32_t Motor_Stop;
uint32_t Motor_gogo;
void Motor_Parser(uint8_t *x){
	Motor_gogo++;
	uint16_t Speed_X = x[1] + (x[2] << 8);
	uint16_t Speed_Y = x[3] + (x[4] << 8);
	switch (x[0]){
	case CAN_Package_X_Positive:
		Motor_X_Positive_Go(Speed_X);
		Motor_Y_Stop();
		break;
	case CAN_Package_X_Negative:
		Motor_X_Negative_Go(Speed_X);
		Motor_Y_Stop();
		break;
	case CAN_Package_Y_Positive:
		Motor_Y_Positive_Go(Speed_Y);
		Motor_X_Stop();
		break;
	case CAN_Package_Y_Negative:
		Motor_Y_Negative_Go(Speed_Y);
		Motor_X_Stop();
		break;

	case (CAN_Package_X_Negative | CAN_Package_Y_Negative):
				Motor_X_Negative_Go(Speed_X);
	Motor_Y_Negative_Go(Speed_Y);
	break;
	case (CAN_Package_X_Positive | CAN_Package_Y_Negative):
				Motor_X_Positive_Go(Speed_X);
	Motor_Y_Negative_Go(Speed_Y);
	break;
	case (CAN_Package_X_Negative | CAN_Package_Y_Positive):
				Motor_X_Negative_Go(Speed_X);
	Motor_Y_Positive_Go(Speed_Y);
	break;
	case (CAN_Package_X_Positive | CAN_Package_Y_Positive):
				Motor_X_Positive_Go(Speed_X);
	Motor_Y_Positive_Go(Speed_Y);
	break;
	case 0:
	case CAN_Package_All_Stop:
		Motor_X_Stop();
		Motor_Y_Stop();
		Motor_Stop++;
		Motor_gogo = 0;
		break;
	case New_Angle_Y:
	case New_Angle_X:

		break;
	default:
		break;
	}
}

void stopping_sequence(TIM_HandleTypeDef htim){
	for(uint16_t current_speed = htim.Instance->PSC; current_speed != 0xffff; ++current_speed){
		htim.Instance->PSC = current_speed;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == Limit_SW_2_Pin){
		if (HAL_GPIO_ReadPin(Limit_SW_2_GPIO_Port, Limit_SW_2_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(ENABLE_Y_GPIO_Port, ENABLE_Y_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			Step_Y.Buttons |= LS_Down;
		}
		else {
			Step_Y.Buttons &= ~LS_Down;
		}
	}
	else if(GPIO_Pin == Limit_SW_1_Pin){
		if (HAL_GPIO_ReadPin(Limit_SW_1_GPIO_Port, Limit_SW_1_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(ENABLE_Y_GPIO_Port, ENABLE_Y_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			Step_Y.Buttons |= LS_Up;
		}
		else {
			Step_Y.Buttons &= ~LS_Up;
		}
	}
	else if(GPIO_Pin == INDEX_Y_Pin){
		if (Step_Y.Direction == Direction_Positive){
			IndexCounter_Y++;
		}else {
			IndexCounter_Y--;
		}
		Step_Y.Angle = fmodf(IndexCounter_Y * 7.2 / 63.0, 360.0);
		Send_Data(&Step_Y);
	}
	else if(GPIO_Pin == INDEX_X_Pin){
		if (Step_X.Direction == Direction_Positive){
			IndexCounter_X++;
		}else {
			IndexCounter_X--;
		}
		Step_X.Angle = fmodf(IndexCounter_X * 7.2 / 63.0, 360.0);
		Send_Data(&Step_X);
	}
	else{
		__NOP();
	}
}
void Send_Data(Step_Driver* SD)
{
	//	uint8_t *Package = &(SD->Axis);
	//	memcpy(&Package[1], &(SD->Angle), sizeof(SD->Angle));
	//
	//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Package, &TxMailbox);
}
bool HasFlag(uint32_t a, uint32_t b)
{
	return ((a & b) == b) ? true : false;
}

void tmc2209_CRC8(uint8_t* datagram, uint8_t datagramLength)
{
	tmc_CRC8(datagram, datagramLength, 0);
	//	int i,j;
	//	uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
	//	uint8_t currentByte;
	//	*crc = 0;
	//	for (i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
	//		currentByte = datagram[i]; // Retrieve a byte to be sent from Array
	//		for (j=0; j<8; j++) {
	//			if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
	//			{
	//				*crc = (*crc << 1) ^ 0x07;
	//			}
	//			else
	//			{
	//				*crc = (*crc << 1);
	//			}
	//			currentByte = currentByte >> 1;
	//		} // for CRC bit
	//	} // for message byte
}

#define SYNC 0x05
#define Y_ADDR 0x01
#define X_ADDR 0x00
#define WRITE_READ_BIT 0x80
#define reg0 0x00
#define INDEX_TOGGLE 0x20
#define PDN_UART 0x40
#define MSTEP 0x80
#define bit0 0x01
uint8_t data[4] = {INDEX_TOGGLE, 0, 0, bit0 | INDEX_TOGGLE | PDN_UART | MSTEP};
void SendToTMC2209_Y(){
	uint8_t package[] = {SYNC, Y_ADDR, WRITE_READ_BIT | reg0, data[3], data[2], data[1], data[0], 0};
	tmc2209_CRC8(package, sizeof(package));
	HAL_UART_Transmit(&huart1, package, sizeof(package), 0xffff);
}
void SendToTMC2209_X(){
	uint8_t package[] = {SYNC, X_ADDR, WRITE_READ_BIT | reg0, data[3], data[2], data[1], data[0], 0};
	tmc2209_CRC8(package, sizeof(package));
	HAL_UART_Transmit(&huart3, package, sizeof(package), 0xffff);
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
