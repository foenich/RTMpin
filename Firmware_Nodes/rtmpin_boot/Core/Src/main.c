/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>

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
CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  /* wenn an adresse 0x0802 0000 der Wert 0x00 steht -> copy new programm code to 0x8004000
   * -> checken, ob das .bin file wirklich mit 0x00 anfängt (war bisher immer so) */
  if ( (*(uint8_t *)0x08020000) == 0x00 )
  {
	  /* suche letztes byte im Sektor, das den Wert 0x00 hat
	   * dann ist die Checksumme an dieser Adresse - 15
	   * das Programmfile-Ende (inkl. aufgefüllter Nullen für Frame) in Adresse - 16
	   */
	  uint32_t lastAddress = 0x0803FFFF; /* letzte Adresse in Sektor 5 */
	  while ( (*(uint8_t *)lastAddress) != 0x00 )
	  {
		  lastAddress--;
	  }
	  /* crc check: using hardware it's calculated like CRC32-MPEG2 with LSB first */
	  uint32_t crcIst;
	  uint32_t crcSoll;
	  __HAL_RCC_CRC_CLK_ENABLE();
	  crcIst = HAL_CRC_Calculate(&hcrc, (uint32_t*)0x08020000, (uint32_t)((lastAddress - 15 - 0x08020000)/4));
	  crcSoll = (*(uint32_t *)(lastAddress - 15));
/*		  uint8_t test[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	  	  crcIst = HAL_CRC_Calculate(&hcrc, (uint32_t*)test, (uint32_t)(4));
	  	  crcSoll = (*(uint32_t *)(lastAddress - 15));*/
	  if (crcIst != crcSoll)
	  {
		  HAL_FLASH_Unlock();
		  FLASH_Erase_Sector(5, FLASH_VOLTAGE_RANGE_3);

		  /* blink and wait for reset */
		  while (1)
		  {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			HAL_Delay(200);
		  }
	  }

	  HAL_FLASH_Unlock();
	  FLASH_Erase_Sector(1, FLASH_VOLTAGE_RANGE_3);
	  FLASH_Erase_Sector(2, FLASH_VOLTAGE_RANGE_3);
	  FLASH_Erase_Sector(3, FLASH_VOLTAGE_RANGE_3);
	  FLASH_Erase_Sector(4, FLASH_VOLTAGE_RANGE_3);

	  /* copy sector 5 to Sector 1,2,3 and 4 */
	  uint32_t readAddress =  0x08020000; /* begin of sector 5 */
	  uint32_t writeAddress = 0x08004000; /* begin of sector 1 */
	  while (readAddress < 0x0803FFFF) /* till the end of secotor 5 except last byte */
	  {
	      HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, writeAddress, (*(uint8_t *)readAddress) );
	      readAddress++;
	      writeAddress++;
	  }
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
	  FLASH_Erase_Sector(5, FLASH_VOLTAGE_RANGE_3);
	  HAL_FLASH_Lock();
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
	  HAL_Delay(1000);
}

  /* jump to applicationcode see also https://embetronicx.com/tutorials/microcontrollers/stm32/bootloader/simple-stm32-bootloader-implementation-bootloader-tutorial/ */
  void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*) (0x08004000 + 4U)));
  app_reset_handler();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

