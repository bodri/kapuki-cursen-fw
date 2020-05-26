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
#include <string>
#include "jetiexprotocol.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct Settings {
	int16_t currentCalibrationValue;
	int16_t capacityResetChannel;

	Settings(int16_t currentCalibrationValue, int16_t capacityResetChannel) :
		currentCalibrationValue(currentCalibrationValue), capacityResetChannel(capacityResetChannel) { }
	operator uint64_t() const {
		return ((uint64_t)currentCalibrationValue << 16) | (uint64_t)capacityResetChannel;
	}
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SETTINGS_FLASH_ADDRESS 0x0801F800 // page 63

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint16_t adc1Readings[4]; //ADC Readings
volatile uint16_t adc2Readings[4]; //ADC Readings
volatile double measuredCurrent = 0;
volatile double measuredVoltage = 0;
volatile double measuredPower = 0;
volatile double measuredCapacity = 0;

uint8_t serialData[1];
bool jetiExBusInSync { false };
uint32_t numberOfCharsDidRead { 0 };
bool useExBusHighSpeed { true };
uint8_t currentScreen { 0 };
Settings settings(0, 0);

JetiExProtocol *jetiExProtocol;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		measuredCurrent = ((3.0 * adc2Readings[0] / 4096.0) - 1.5) / 0.012; // (Vout - Vref) / (Rsense * Av)
		measuredVoltage = (3.0 * adc1Readings[0] / 4096.0) / 0.058968058968059;
		measuredPower = measuredCurrent * measuredVoltage;
		measuredCapacity += measuredCurrent / 360.0; // mAh
	}
}

bool writeSettingsToFlash() {
    // Unlock flash
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTINGS_FLASH_ADDRESS, (uint64_t)settings) != HAL_OK) {
		HAL_FLASH_Lock();
		return false;
	}

    HAL_FLASH_Lock();
    return true;
}

void setReferenceForCurrentSenseAmplifier() {
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048 + settings.currentCalibrationValue);
}

void setCapacityResetChannelObserver() {
	  jetiExProtocol->addChannelObserver(settings.capacityResetChannel, [](uint16_t channelData) {
		  if (channelData == 8000) {
			  measuredCapacity = 0;
		  }
	  });
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

  // Load settings
  uint64_t loadedSettings = *(uint64_t *)SETTINGS_FLASH_ADDRESS;
  settings = Settings((loadedSettings >> 16) & 0xFFFF, loadedSettings & 0xFFFF);
//  if (settings.currentCalibrationValue == 0) {
//	  settings
//  }

  // Calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  // Set REF to MAX4081
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2031); // Calibrated to zero when no load
  setReferenceForCurrentSenseAmplifier();

  // Start Current and Voltage measurement
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1Readings, 1);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2Readings, 1);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  TelemetryData *sensor = new TelemetryData(0, "kapuki-CS", "", zero, 0);
  TelemetryData *current = new TelemetryData(1, "Current", "A", int14_t, 1);
  TelemetryData *voltage = new TelemetryData(2, "Voltage", "V", int14_t, 2);
  TelemetryData *capacity = new TelemetryData(3, "Capacity", "mAh", int14_t, 0);
  TelemetryData *power = new TelemetryData(4, "Power", "W", int14_t, 0);
  std::vector<TelemetryData *> telemetryDataArray = {
		  sensor,
		  current,
		  voltage,
		  capacity,
		  power
  };

  jetiExProtocol = new JetiExProtocol(0xA4A1, 0x555D, telemetryDataArray);
  jetiExProtocol->onPacketSend = [](const uint8_t *packet, size_t size) {
	  if (HAL_UART_Transmit(&huart1, (uint8_t *)packet, size, 1000) != HAL_OK) {
		 Error_Handler();
	  }
  };

  setCapacityResetChannelObserver();

  // JetiBox screens
  jetiExProtocol->onDisplayScreen = [](const uint8_t buttonStatus) {
	  switch (buttonStatus) {
		case 0xE0:
			if (currentScreen < 5) {
				currentScreen++;
			}
			break;
		case 0x70:
			if (currentScreen == 99) {
				currentScreen = 0;
			}
			if (currentScreen > 0) {
				currentScreen--;
			}
			break;
		case 0xD0:
			if (currentScreen == 3) {
				settings.currentCalibrationValue++;
				setReferenceForCurrentSenseAmplifier();
			}
			if (currentScreen == 4 && settings.capacityResetChannel < 24) {
				jetiExProtocol->removeChannelObserver(settings.capacityResetChannel);
				settings.capacityResetChannel++;
				setCapacityResetChannelObserver();
			}
			break;
		case 0xB0:
			if (currentScreen == 3) {
				settings.currentCalibrationValue--;
				setReferenceForCurrentSenseAmplifier();
			}
			if (currentScreen == 4 && settings.capacityResetChannel > 0) {
				jetiExProtocol->removeChannelObserver(settings.capacityResetChannel);
				settings.capacityResetChannel--;
				setCapacityResetChannelObserver();
			}
			break;
		case 0x90:
			writeSettingsToFlash(); // TODO: user feedback and error handling
			break;
		default:
			break;
	  }

	  switch (currentScreen) {
		case 0:
			return std::string("    kapuki-CS   "  " Current Sensor ");
		case 1: {
			char data[32];
			sprintf(data, "Current:%+4d.%01dA Voltage:  %2d.%02dV", (int)measuredCurrent, abs((int)(measuredCurrent * 100) % 100), (int)measuredVoltage, abs((int)(measuredVoltage * 100) % 100));
			std::string str(data);
			return str;
		}
		case 2: {
			char data[32];
			sprintf(data, "Capacity:%4dmAhPower:   %4dW", (int)measuredCapacity, abs((int)measuredPower));
			std::string str(data);
			return str;
		}
		case 3: {
			char data[32];
			sprintf(data, "Current:%+4d.%02dACalibration:%+4d", (int)measuredCurrent, abs((int)(measuredCurrent * 100) % 100), settings.currentCalibrationValue);
			return std::string(data);
		}
		case 4: {
			char data[32];
			sprintf(data, "Capacity reset  on channel: %d", settings.capacityResetChannel);
			return std::string(data);
		}
		case 5: {
			return std::string("Save changes    UpDown button");
		}
		default:
			return std::string();
	  }
  };

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

	  numberOfCharsDidRead++;
	  current->setValue((int16_t)(measuredCurrent * 10.0));
	  voltage->setValue((int16_t)(measuredVoltage * 100.0));
	  power->setValue((int16_t)(measuredPower));
	  capacity->setValue((int16_t)(measuredCapacity));

	  bool validPacket = jetiExProtocol->readByte(serialData[0]);
	  if (!jetiExBusInSync && !validPacket && numberOfCharsDidRead > 1000) {
		  // Toggle EX BUS speed: 125 kBaud (LS) or 250 kBaud (HS)
		  useExBusHighSpeed ? MX_USART1_UART_Init_low_speed() : MX_USART1_UART_Init();
		  useExBusHighSpeed = !useExBusHighSpeed;
		  numberOfCharsDidRead = 0;
	  }

	  if (validPacket) {
		  jetiExBusInSync = true;
	  }
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
