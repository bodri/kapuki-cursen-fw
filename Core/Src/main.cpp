/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "comp.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <string.h>

#include "jetiexhandler.h"

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

volatile uint16_t adc1Readings[4]; //ADC Readings
volatile uint16_t adc2Readings[4]; //ADC Readings
volatile bool busReleased = false;
volatile bool telemetryRequest = false;
volatile bool jetiboxRequest = false;
uint8_t serialData[1];
uint8_t state;
uint8_t length;
uint8_t packetId;

uint8_t textCounter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM4) {
//		shouldSendData = true;
//	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	if (serialData[0] == 0x3E && serialData[1] == 0x01)  {
//		shouldSendData = true;
//	}
}

#define POLY 0x07
unsigned char update_crc(unsigned char crc, unsigned char crc_seed) {
	unsigned char crc_u;
	unsigned char i;

	crc_u = crc;
	crc_u ^= crc_seed;

	for (i = 0; i < 8; i++) {
		crc_u = (crc_u & 0x80) ? POLY ^ (crc_u << 1) : (crc_u << 1);
	}
	return crc_u;
}

unsigned char calculateCrc8(unsigned char *crc, unsigned char crc_lenght) {
	unsigned char crc_up = 0;
	unsigned char c;

	for (c = 0; c < crc_lenght; c++) {
		crc_up = update_crc(crc[c], crc_up);
	}

	return crc_up;
}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_COMP1_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  // Calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  // Set REF to 1.6V
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2016);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1Readings, 2);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2Readings, 1);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  HAL_SYSCFG_EnableVREFBUF();

  JetiExHandler jetiProtocol;

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim4);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_UART_Receive(&huart1, serialData, sizeof(serialData), 1000) != HAL_OK) {
		  Error_Handler();
	  }

	  jetiProtocol.readByte(serialData[0]);

	  switch (state) {
	  case 0:
			if (serialData[0] == 0x3E || serialData[0] == 0x3D)  {
				state++;
			} else {
				state = 0;
			}
			break;
	  case 1:
		  if (serialData[0] == 0x01) {
			  state++;
		  } else {
			  state = 0;
		  }
		  break;
	  case 2:
		  length = serialData[0] - 3; // 2 (byte header) + 1 (zero based)
		  if (length >= 3) {
			  state++;
		  } else {
			  state = 0;
		  }
		  break;
	  case 3:
		  length--;
		  packetId = serialData[0];
		  state++;
		  break;
	  case 4:
		  length--;
		  telemetryRequest = serialData[0] == 0x3A ? true : false;
		  jetiboxRequest = serialData[0] == 0x3B ? true : false;
		  state++;
		  break;
	  case 5:
		  length--;
		  if (length == 0) {
			  for (int i = 0; i < 500; i++) { }
			  busReleased = true;
			  state = 0;
		  }
		  break;
	  }


	  if (busReleased) {
		  if (jetiboxRequest) {
			  uint8_t cucc[] = "\x43\x65\x6E\x74\x72\x61\x6C\x20\x42\x6F\x78\x20\x31\x30\x30\x3E\x20\x20\x20\x34\x2E\x38\x56\x20\x20\x31\x30\x34\x30\x6D\x41\x00";
			  uint8_t data[128];
			  data[0] = 0x3B;
			  data[1] = 0x01;
			  data[2] = sizeof(cucc) + 7; //len
			  data[3] = packetId; //0x08 packetId
			  data[4] = 0x3B;
			  data[5] = sizeof(cucc) - 1;

			  uint8_t crc8 = calculateCrc8(cucc + 1, sizeof(cucc) - 3);
			  cucc[sizeof(cucc) - 2] = crc8;

			  memcpy(&data[6], cucc, sizeof(cucc));
			  uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data, sizeof(cucc) + 5);
			  data[sizeof(cucc) + 5] = (uint8_t)crc;
			  data[sizeof(cucc) + 6] = (uint8_t)(crc >> 8);

			  if (HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(cucc) + 7, 1000) != HAL_OK) {
				 Error_Handler();
			  }
		  } else if (telemetryRequest) {
			  uint8_t cucc0[] = "\x9F\x10\xA1\xA4\x5D\x55\x00\x00\x40kapukiCS\x00"; // Sensor name
			  uint8_t cucc1[] = "\x9F\x10\xA1\xA4\x5D\x55\x00\x01\x39\x43\x75\x72\x72\x65\x6E\x74\x41\x00"; // Current
			  uint8_t cucc2[] = "\x9F\x10\xA1\xA4\x5D\x55\x00\x02\x39\x56\x6F\x6C\x74\x61\x67\x65\x56\x00"; // Voltage
//			  uint8_t cucc3[] = "\x9F\x13\xA1\xA4\x5D\x55\x00\x03\x43\x56\x61\x6C\x61\x63\x69\x74\x79mAh\x00"; // Capacity
			  uint8_t cucc3[] = "\x9F\x10\xA1\xA4\x5D\x55\x00\x04\x39Power  W\x00"; // Power

			  uint8_t cucc[sizeof(cucc1)];
			  memcpy(cucc,  textCounter == 0 ? cucc0 : (textCounter == 1 ? cucc1 : (textCounter == 2 ? cucc2 :cucc3)), sizeof(cucc1));
			  uint8_t data[128];
			  data[0] = 0x3B;
			  data[1] = 0x01;
			  data[2] = sizeof(cucc) + 7; //len
			  data[3] = packetId; //0x08 packetId
			  data[4] = 0x3A;
			  data[5] = sizeof(cucc) - 1;

			  textCounter >= 3 ? textCounter = 0 : textCounter++;

			  uint8_t crc8 = calculateCrc8(cucc + 1, sizeof(cucc) - 3);
			  cucc[sizeof(cucc) - 2] = crc8;

			  memcpy(&data[6], cucc, sizeof(cucc));
			  uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data, sizeof(cucc) + 5);
			  data[sizeof(cucc) + 5] = (uint8_t)crc;
			  data[sizeof(cucc) + 6] = (uint8_t)(crc >> 8);

			  if (HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(cucc) + 7, 1000) != HAL_OK) {
				 Error_Handler();
			  }
		  } else {
			  uint8_t cucc[] = "\x9F\x54\xA1\xA4\x5D\x55\x00\x11\xE8\x23\x21\x1A\x00\x31\x1A\x00\x41\x1A\x00\x00";
			  uint8_t data[128];
			  data[0] = 0x3B;
			  data[1] = 0x01;
			  data[2] = sizeof(cucc) + 7; //len
			  data[3] = packetId; //0x08 packetId
			  data[4] = 0x3A;
			  data[5] = sizeof(cucc) - 1;

			  // calculate current
			  float rawCurrent = ((3.3 * adc2Readings[0] / 4096) - 1.65) / 0.012; // (Vout - Vref) / (Rsense * Av)
			  uint16_t current = rawCurrent * 10;
			  cucc[8] = (uint8_t)current;
			  cucc[9] = (uint8_t)((current >> 8) & 0x1F) | 0x20;

			  // calculate voltage
			  float rawVoltage = (3.3 * adc1Readings[0] / 4096) / 0.0625;
			  uint16_t voltage = rawVoltage * 100;
			  cucc[11] = (uint8_t)voltage;
			  cucc[12] = (uint8_t)((voltage >> 8) & 0x1F) | 0x40;

			  // calculate power
			  uint16_t power = rawCurrent * rawVoltage * 100;
			  cucc[17] = (uint8_t)power;
			  cucc[18] = (uint8_t)(power >> 8) & 0x1F;

			  uint8_t crc8 = calculateCrc8(cucc + 1, sizeof(cucc) - 3);
			  cucc[sizeof(cucc) - 2] = crc8;

			  memcpy(&data[6], cucc, sizeof(cucc));
			  uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data, sizeof(cucc) + 5);
			  data[sizeof(cucc) + 5] = (uint8_t)crc;
			  data[sizeof(cucc) + 6] = (uint8_t)(crc >> 8);

			  if (HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(cucc) + 7, 1000) != HAL_OK) {
				 Error_Handler();
			  }
		  }


		  busReleased = false;
	  }
//
//	  HAL_Delay(5);
//	  if (UART_CheckIdleState(&huart1) == HAL_TIMEOUT) {
//		/* Timeout occurred */
//		return HAL_TIMEOUT;
//	  }
//	if (shouldSendData) {
//		uint8_t data[8];
//		data[0] = 0x69;
//		data[1] = adc2Readings[0];
//		data[2] = adc1Readings[0];
//		data[3] = ((adc2Readings[0] >> 4) & 0xf0)
//				| ((adc1Readings[0] >> 8) & 0x0f);
//		data[4] = adc1Readings[1];
//		data[5] = adc1Readings[2];
//		data[6] = ((adc1Readings[1] >> 4) & 0xf0)
//				| ((adc1Readings[2] >> 8) & 0x0f);
//
//		uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data, sizeof(data) - 1);
//		data[7] = (uint8_t) crc;
//		if (HAL_UART_Transmit_DMA(&huart1, data, sizeof(data)) != HAL_OK) {
//			Error_Handler();
//		}
//
//		shouldSendData = false;
//	}
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

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
