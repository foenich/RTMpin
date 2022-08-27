/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t currentRx; /* current byte in serial buffer */
extern uint8_t dataArrived; /* new data byte arrived */

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
void led_update_sequence(uint8_t tc);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SOLFORCE25 4300
#define SOLFORCE50 7500
#define BlueLED_Pin LL_GPIO_PIN_13
#define BlueLED_GPIO_Port GPIOC
#define KeySwitch_Pin LL_GPIO_PIN_0
#define KeySwitch_GPIO_Port GPIOA
#define RS485_nWrite_Pin LL_GPIO_PIN_12
#define RS485_nWrite_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define NODE_ADDRESS 1 /* this nodes address */
#define NR_OF_NODES 3 /* total number of nodes in system */
#define NR_OF_LEDS 50 /* number of LEDs on the LED-string connected to this node */

#define BLACK       0x000000
#define MAROON      0x800000
#define DARKGREEN   0x008000
#define OLIVE       0x808000
#define NAVY        0x000080
#define PURPLE      0x800080
#define TEAL        0x008080
#define GREY        0x808080
#define RED         0xff0000
#define GREEN       0x00ff00
#define YELLOW      0xffff00
#define BLUE        0x0000ff
#define MAGENTA     0xff00ff
#define AQUA        0x00ffff
#define WHITE       0xffffff

#define ORANGE		0xff4000
#define PINK		0xff44CC


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
