/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "keypad_4x4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  Pulso_Max 550
#define  Pulso_Min 2400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char tecla, dato[500];
int PWM_MOTOR = 2400, ADC = 0, t = 0, angulo = 0, ag = 0;
float Potenciometro = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Configuraciones_Iniciales();
void keypad();
void condiciones();
void conversion_Pot_Ang();
void servo_angulo(uint8_t ang);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Configuraciones_Iniciales();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
	  keypad();
	  HAL_Delay(200);

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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Configuraciones_Iniciales(){
	sprintf(dato,"**********************El mejor control del mundo**********************\r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t*)dato, strlen(dato), HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, 1);
	  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
	  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	  HAL_TIM_Base_Start(&htim1);
	  HAL_TIM_Base_Start(&htim2);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  TIM1->CCR1 = 0;
	  TIM2->CCR2 = 0;
	  conversion_Pot_Ang();
	  t = (900/3.16)*Potenciometro;
	  TIM2->CCR2 = PWM_MOTOR;
	  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
	  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	  HAL_Delay(t);
	  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
	  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	  HAL_Delay(10);
}
void keypad()
{
	tecla = Keypad_Get_Char();
	if (tecla !=0){
		sprintf(dato,"Tecla: %c, ",tecla);
		HAL_UART_Transmit(&huart1, (uint8_t*)dato, strlen(dato), HAL_MAX_DELAY);
	}
	condiciones();
	conversion_Pot_Ang();
	servo_angulo(angulo);
}
void condiciones(){
	switch (tecla){
	case 'A':
		HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, 0);
		sprintf(dato,"Led encendido, ");
		HAL_UART_Transmit(&huart1, (uint8_t*)dato, strlen(dato), HAL_MAX_DELAY);
		break;
	case 'B':
		HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, 1);
		sprintf(dato,"Led apagado, ");
		HAL_UART_Transmit(&huart1, (uint8_t*)dato, strlen(dato), HAL_MAX_DELAY);
		break;
	case 'C':
	    t =(900/3.19)*3.19;
		TIM2->CCR2 = PWM_MOTOR;
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
		HAL_Delay(t);
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		HAL_Delay(10);
		break;
	case 'D':
		t =(900/3.19)*1.595;
		TIM2->CCR2 = PWM_MOTOR;
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
		HAL_Delay(t);
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		HAL_Delay(10);
		break;
	case '0':
		t =(900/3.19)*Potenciometro;
		TIM2->CCR2 = PWM_MOTOR;
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		HAL_Delay(t);
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		HAL_Delay(10);
		break;
	}
}
void conversion_Pot_Ang(){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADC = HAL_ADC_GetValue(&hadc1);
	Potenciometro = (float)((ADC*3.3)/4095.0);
	angulo = (180/3.19)*Potenciometro;
	sprintf(dato,"ADC: %i, Potenciometro: %0.2f, Angulo: %i ",ADC, Potenciometro, angulo);
	HAL_UART_Transmit(&huart1, (uint8_t*)dato, strlen(dato), HAL_MAX_DELAY);
}
void servo_angulo(uint8_t ang){
	uint16_t PWM_SERVO;
	PWM_SERVO =(uint16_t)((ang-0)*(Pulso_Max - Pulso_Min)/(180-0) + Pulso_Min);
	TIM1->CCR1 = PWM_SERVO;
	sprintf(dato,", PWM_SERVO: %i \r\n",PWM_SERVO);
	HAL_UART_Transmit(&huart1, (uint8_t*)dato, strlen(dato), HAL_MAX_DELAY);
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
