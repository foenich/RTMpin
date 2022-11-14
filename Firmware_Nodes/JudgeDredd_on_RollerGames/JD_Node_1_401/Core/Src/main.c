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
uint8_t dataArrived = 0;
uint8_t currentRx = 0;

static uint32_t tmp_led_data[2*3*8];
static uint32_t current_led;        /*!< Current LED number we are sending */

uint32_t led[NR_OF_LEDS]; /* holds the color (0xWWRRGGBB) of all LEDs */

uint8_t rxBuffer[17]; /* addr&length + data (max 16) */
uint8_t rxBufferUpdated; /* new frame complete */
uint8_t nodeStatus; /* 	      * 0 -> standard (receiving) operation
                              * 1 -> send data to Pi
                              * 2 -> Software update of this node
                              * 3 -> Software update of other nodes */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Control of LED strips is  based on this idea: https://stm32f4-discovery.net/2018/06/tutorial-control-ws2812b-leds-stm32/ */
/**
 * \brief           Prepares data from memory for PWM output for timer
 * \note            Memory is in format R,G,B, while PWM must be configured in G,R,B[,W]
 * \param[in]       ledx: LED index to set the color
 * \param[out]      ptr: Output array with at least LED_CFG_RAW_BYTES_PER_LED-words of memory
 */
static uint8_t led_fill_led_pwm_data(uint8_t ledx, uint32_t* ptr) {
    uint8_t i;

    if (ledx < NR_OF_LEDS) {
        for (i = 0; i < 8; i++) {
            ptr[i] =        (led[ledx] & (1 << (15 - i))) ? (2 * TIM2->ARR / 3) : (TIM2->ARR / 3);
            ptr[8 + i] =    (led[ledx] & (1 << (23 - i))) ? (2 * TIM2->ARR / 3) : (TIM2->ARR / 3);
            ptr[16 + i] =   (led[ledx] & (1 << (7 - i))) ? (2 * TIM2->ARR / 3) : (TIM2->ARR / 3);
        }
        return 1;
    }
    return 0;
}




/**
 * \brief           Update sequence function, called on each DMA transfer complete or half-transfer complete events
 * \param[in]       tc: Transfer complete flag. Set to `1` on TC event, or `0` on HT event
 *
 * \note            TC = Transfer-Complete event, called at the end of block
 * \note            HT = Half-Transfer-Complete event, called in the middle of elements transfered by DMA
 *                  If block is 48 elements (our case),
 *                      HT is called when first LED_CFG_RAW_BYTES_PER_LED elements are transfered,
 *                      TC is called when second LED_CFG_RAW_BYTES_PER_LED elements are transfered.
 *
 * \note            This function must be called from DMA interrupt
 */
void led_update_sequence(uint8_t tc) {
    tc = !!tc;                                  /* Convert to 1 or 0 value only */

    if (current_led < NR_OF_LEDS) {

			current_led++;
			/*
			 *  if there was no TC event (it was HT):
			 *
			 *  - Prepare first part of array, because either there is no transfer
			 *      or second part (from HT to TC) is now in process for PWM transfer
			 *
			 * In other case (TC = 1)
			 */
			if (!tc) {
				led_fill_led_pwm_data(current_led, &tmp_led_data[0]);
			} else {
				led_fill_led_pwm_data(current_led, &tmp_led_data[3*8]);
			}

	/*
	 * When we reached all leds, we have to wait to transmit data for all leds before we can disable DMA and PWM:
	 *
	 *  - If TC event is enabled and we have EVEN number of LEDS (2, 4, 6, ...)
	 *  - If HT event is enabled and we have ODD number of LEDS (1, 3, 5, ...)
	 */
	} else if ((!tc && (NR_OF_LEDS & 0x01)) || (tc && !(NR_OF_LEDS & 0x01))) {
		LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH2); /* Disable channel */
		LL_TIM_DisableCounter(TIM2);
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
	}
}


void handleSerialData() {
	static uint32_t lastTick;
	static uint8_t rxLength; /* number of data bytes to be received. addr&length not counted;  */
	static uint8_t rxByteNr;  /* index of received byte (start with 0) */
	static uint8_t rxAddress; /* node to be addressed, read from first byte */
	static uint8_t nodesSending; /* mode when nodes are sending and Pi is quiet */

	if ( (uwTick - lastTick) > 5 ) /* resync if there was no data for longer than 5 ms */
	{
		rxLength = 0;
		rxByteNr = 0;
		rxAddress = 0;
		nodesSending = 0;
	}
	lastTick = uwTick;


	if ( nodesSending ) {
		rxByteNr++;
		if ( rxByteNr == 2*NODE_ADDRESS ) { /* this node is about to send except if its node 0 */
			nodeStatus = 1;
			rxByteNr += 2;
		}
		if ( rxByteNr == 2*NR_OF_NODES ) { /* all nodes finished sending */
			nodesSending = 0;
			rxByteNr = 0;
		}
	} else {
		if ( rxByteNr == 0 ) {
			rxByteNr++;
			rxLength = currentRx & 0xf; /* get length */
			rxAddress = currentRx>>4;  /* get address */

			if ( rxLength == 0xf ) /* Software update */
			{
				rxLength = 16; /* 16 bytes, that it is a multiple of 4 (good for CRC calculation) */
				if ( rxAddress == NODE_ADDRESS )
				{
					nodeStatus = 2; /* Software update of this node */
				} else
				{
					nodeStatus = 3;  /* Software update of other node */
				}
			}

			if ( rxAddress == NODE_ADDRESS ) {
				rxBuffer[0] = currentRx;
			}

			if ( (rxAddress == 14) && (nodeStatus != 2) && (nodeStatus != 3) ) {
				if (NODE_ADDRESS == 0) { /* start sending if this is node 0 */
					nodeStatus = 1;
					rxByteNr = 2;
				} else {
					rxByteNr = 0;
				}
				if (NR_OF_NODES > 1) {
					nodesSending = 1; /* more nodes are connected */
				} else {
					rxByteNr = 0;  /* this is the one and only node */
				}
			}
		} else {
			if ( rxAddress == NODE_ADDRESS ) {
				rxBuffer[rxByteNr] = currentRx;
			}
			if (rxByteNr == rxLength) {
				rxByteNr = 0;
				if ( rxAddress == NODE_ADDRESS ) {
					rxBufferUpdated = 1;
				}
			} else {
				rxByteNr++;
			}
		}
	}

}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	/* to be started form bootloader, see also
	 * https://embetronicx.com/tutorials/microcontrollers/stm32/bootloader/simple-stm32-bootloader-implementation-bootloader-tutorial/
	 * edit system_stm32f4xx.c as well to enter offset and #define USER_VECT_TAB_ADDRESS
	 * and edit linker script STM32F401CCUX_FLASH.ld */
	__disable_irq();
	SCB->VTOR = 0x8004000;
	__DSB();
	__enable_irq();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  rxBufferUpdated = 0;

  nodeStatus = 0;
  for (uint8_t bb = 0; bb < 17; bb++)
  {
	  rxBuffer[bb] = 0;
  }

  uint8_t txSwitchesByte1 = 0; /* all switches off, switches are low-active */
  uint8_t txSwitchesByte2 = 0;
  uint8_t prevTxSwitchByte1 = 0;
  uint8_t prevTxSwitchByte2 = 0;
  uint8_t txByte1_transmitted = 0;
  uint32_t stReload;
  uint32_t stNow, stOld;
  uint32_t stTicks = 0;
  uint32_t slingPulse = 0;
  uint32_t bumprightPulse = 0;
  uint32_t bumpmidPulse = 0;
  uint32_t bumpleftPulse = 0;
  uint32_t kickbackPulse = 0;
  uint32_t kickbackGrace = 0;
  uint32_t flipperPulse = 0;
  uint32_t readyToSendCounter = 0;
  uint32_t packetWatchDog = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  /* All outputs off */
  LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH4);
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_EnableCounter(TIM4);

  stOld = SysTick->VAL;
  stReload = SysTick->LOAD;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* count systicks. Only works that easy as the while loop is shorter than 1 ms!! Ohterwise make a systick interrupt */
	    stNow = SysTick->VAL;
		if( stNow != stOld )
		{
			if ( stNow < stOld )
			{
				stTicks = (stOld - stNow);
			}
			else
			{
				stTicks = (stReload - stNow + stOld);
			}
			stOld = stNow;
		}


		if(dataArrived)
		{
			dataArrived = 0;
			handleSerialData();
		}


		/* ---------- SW update  ----------- */

		if (rxBufferUpdated && (nodeStatus == 3) ) /* Software update of other node */
		{
			  NVIC_DisableIRQ(USART1_IRQn);
			  /* All outputs off */
			  LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH1);
			  LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH2);
			  LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH3);
			  LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH4);
			  LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH1);
			  LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH2);
			  LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH3);
			  LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH4);
			  LL_TIM_EnableCounter(TIM3);
			  LL_TIM_EnableCounter(TIM4);
			  txSwitchesByte1 = 0; /* all switches off */
			  txSwitchesByte2 = 0;
			  prevTxSwitchByte1 = 0;
			  prevTxSwitchByte2 = 0;
			  LL_GPIO_ResetOutputPin(BlueLED_GPIO_Port, BlueLED_Pin);

	  		  HAL_Delay(20000);
			  NVIC_SystemReset();
		}

		/* write new code to flash and reset */
		if (rxBufferUpdated && (nodeStatus == 2) ) /* Software update */
		{
			uint8_t ww = 0;
			uint32_t loopcounter = 0; /* timer */
			uint8_t blbl = 0; /* blink blink - so oft wird geblinkt */
			uint32_t writeAddress = 0x08020000; /* begin of sector 5 */
			HAL_FLASH_Unlock();

			while (1)
			{
				if(dataArrived)
				{
					dataArrived = 0;
					handleSerialData();
				}

				if (rxBufferUpdated && (nodeStatus == 2) )
				{
					rxBufferUpdated = 0;
					loopcounter = 0;
					LL_GPIO_TogglePin (GPIOC, LL_GPIO_PIN_13);
					for (ww = 0; ww < 16; ww++)
					{
						HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, writeAddress, ((uint8_t *)(rxBuffer+1))[ww] );
						writeAddress++;
					}
				}
				loopcounter++;
				if ( loopcounter > 3000000 ) /* ganz grob eine Sekunde */
				{
					LL_GPIO_TogglePin (GPIOC, LL_GPIO_PIN_13);
					loopcounter = 0;
					blbl++;
				}
				if (blbl == 6) NVIC_SystemReset();
			}
		}
		/* ---------- SW update end ----------- */


		if (rxBufferUpdated) {
		  LL_GPIO_TogglePin (GPIOC, LL_GPIO_PIN_13);
		  rxBufferUpdated = 0;
		  packetWatchDog = 0;

		  led[0] = ((rxBuffer[2]>>7)&1) ? BLUE : BLACK;
		  led[1] = ((rxBuffer[2]>>6)&1) ? RED : BLACK;
		  led[2] = ((rxBuffer[2]>>5)&1) ? ORANGE : BLACK;
		  led[3] = ((rxBuffer[2]>>4)&1) ? RED : BLACK;
		  led[4] = ((rxBuffer[2]>>3)&1) ? RED : BLACK;
		  led[5] = ((rxBuffer[2]>>2)&1) ? WHITE : BLACK;
		  led[6] = ((rxBuffer[2]>>1)&1) ? WHITE : BLACK;
		  led[7] = ((rxBuffer[2]>>0)&1) ? RED : BLACK;

		  led[8] = ((rxBuffer[3]>>7)&1) ? RED : BLACK;
		  led[9] = ((rxBuffer[3]>>6)&1) ? RED : BLACK;
		  led[10] = ((rxBuffer[3]>>5)&1) ? RED : BLACK;
		  led[11] = ((rxBuffer[3]>>4)&1) ? ORANGE : BLACK;
		  led[12] = ((rxBuffer[3]>>3)&1) ? RED : BLACK;
		  led[13] = ((rxBuffer[3]>>2)&1) ? ORANGE : BLACK;
		  led[14] = ((rxBuffer[3]>>1)&1) ? ORANGE : BLACK;
		  led[15] = ((rxBuffer[3]>>0)&1) ? RED : BLACK;

		  led[16] = ((rxBuffer[4]>>7)&1) ? RED : BLACK;
		  led[17] = ((rxBuffer[4]>>6)&1) ? WHITE : BLACK;
		  led[18] = ((rxBuffer[4]>>5)&1) ? RED : BLACK;
		  led[19] = ((rxBuffer[4]>>4)&1) ? YELLOW : BLACK;
		  led[20] = ((rxBuffer[4]>>3)&1) ? RED : BLACK;
		  led[21] = ((rxBuffer[4]>>2)&1) ? RED : BLACK;
		  led[22] = ((rxBuffer[4]>>1)&1) ? RED : BLACK;
		  led[23] = ((rxBuffer[4]>>0)&1) ? RED : BLACK;

		  led[24] = ((rxBuffer[5]>>7)&1) ? WHITE : BLACK;
		  led[25] = ((rxBuffer[5]>>6)&1) ? RED : BLACK;
		  led[26] = ((rxBuffer[5]>>5)&1) ? PINK : BLACK;
		  led[27] = ((rxBuffer[5]>>4)&1) ? WHITE : BLACK;
		  led[28] = ((rxBuffer[5]>>3)&1) ? WHITE : BLACK;
		  led[29] = ((rxBuffer[5]>>2)&1) ? WHITE : BLACK;
		  led[30] = ((rxBuffer[5]>>1)&1) ? RED : BLACK;
/*		  led[31] = ((rxBuffer[5]>>0)&1) ? WHITE : BLACK;*/
		  if ((rxBuffer[5]>>0)&1) led[31] = WHITE;
		  else if ((rxBuffer[9]>>7)&1) led[31] = RED;
		  else if ((rxBuffer[9]>>6)&1) led[31] = YELLOW;
		  else if ((rxBuffer[9]>>5)&1) led[31] = GREEN;
		  else led[31] = BLACK;

/*		  led[32] = ((rxBuffer[6]>>7)&1) ? WHITE : BLACK;*/
		  if ((rxBuffer[6]>>7)&1) led[32] = WHITE;
		  else if ((rxBuffer[8]>>5)&1) led[32] = RED;
		  else if ((rxBuffer[8]>>4)&1) led[32] = YELLOW;
		  else if ((rxBuffer[8]>>3)&1) led[32] = GREEN;
		  else led[32] = BLACK;
		  led[33] = ((rxBuffer[6]>>6)&1) ? RED : BLACK;
		  led[34] = ((rxBuffer[6]>>5)&1) ? RED : BLACK;
		  led[35] = ((rxBuffer[6]>>4)&1) ? RED : BLACK;
		  led[36] = ((rxBuffer[6]>>3)&1) ? RED : BLACK;
		  led[37] = ((rxBuffer[6]>>2)&1) ? WHITE : BLACK;
		  led[38] = ((rxBuffer[6]>>1)&1) ? WHITE : BLACK;
		  led[39] = ((rxBuffer[6]>>0)&1) ? WHITE : BLACK;

		  led[40] = ((rxBuffer[7]>>7)&1) ? ORANGE : BLACK;
		  led[41] = ((rxBuffer[7]>>6)&1) ? WHITE : BLACK;
		  led[42] = ((rxBuffer[7]>>5)&1) ? ORANGE : BLACK;
		  led[43] = ((rxBuffer[7]>>4)&1) ? ORANGE : BLACK;
		  led[44] = ((rxBuffer[7]>>3)&1) ? ORANGE : BLACK;
		  led[45] = ((rxBuffer[7]>>2)&1) ? WHITE : BLACK;
		  led[46] = ((rxBuffer[7]>>1)&1) ? WHITE : BLACK;
		  led[47] = ((rxBuffer[7]>>0)&1) ? ORANGE : BLACK;

/*		  led[48] = ((rxBuffer[8]>>7)&1) ? WHITE : BLACK;*/
		  if ((rxBuffer[8]>>7)&1) led[48] = WHITE;
		  else if ((rxBuffer[9]>>4)&1) led[48] = RED;
		  else if ((rxBuffer[9]>>3)&1) led[48] = YELLOW;
		  else if ((rxBuffer[9]>>2)&1) led[48] = GREEN;
		  else led[48] = BLACK;
/*		  led[49] = ((rxBuffer[8]>>6)&1) ? WHITE : BLACK;*/
		  if ((rxBuffer[8]>>6)&1) led[49] = WHITE;
		  else if ((rxBuffer[8]>>2)&1) led[49] = RED;
		  else if ((rxBuffer[8]>>1)&1) led[49] = YELLOW;
		  else if ((rxBuffer[8]>>0)&1) led[49] = GREEN;
		  else led[49] = BLACK;
		  led[50] = ((rxBuffer[8]>>5)&1) ? RED : BLACK;  /* Dummy LED */
		  led[51] = ((rxBuffer[8]>>4)&1) ? YELLOW : BLACK;  /* Dummy LED */
		  led[52] = ((rxBuffer[8]>>3)&1) ? GREEN : BLACK;  /* Dummy LED */
		  led[53] = ((rxBuffer[8]>>2)&1) ? RED : BLACK;  /* Dummy LED */
		  led[54] = ((rxBuffer[8]>>1)&1) ? YELLOW : BLACK;  /* Dummy LED */
		  led[55] = ((rxBuffer[8]>>0)&1) ? GREEN : BLACK;  /* Dummy LED */

		  led[56] = ((rxBuffer[9]>>7)&1) ? RED : BLACK;  /* Dummy LED */
		  led[57] = ((rxBuffer[9]>>6)&1) ? YELLOW : BLACK;  /* Dummy LED */
		  led[58] = ((rxBuffer[9]>>5)&1) ? GREEN : BLACK;  /* Dummy LED */
		  led[59] = ((rxBuffer[9]>>4)&1) ? RED : BLACK;  /* Dummy LED */
		  led[60] = ((rxBuffer[9]>>3)&1) ? YELLOW : BLACK;  /* Dummy LED */
		  led[61] = ((rxBuffer[9]>>2)&1) ? GREEN : BLACK;  /* Dummy LED */

		  current_led = 0;
		  led_fill_led_pwm_data(current_led, &tmp_led_data[0]); /* first LED */
		  current_led++;
		  led_fill_led_pwm_data(current_led, &tmp_led_data[3*8]); /* second LED */

	      LL_DMA_ClearFlag_TC6(DMA1);
	      LL_DMA_ClearFlag_HT6(DMA1);
	      LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
	      LL_TIM_SetCounter (TIM2, 0);
	      LL_TIM_EnableCounter(TIM2);
	      LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);

			/* Set and reset outputs (solenoids, magnets, flashers) */
			/* OB8 */
			/* OB8 -> left flipper controlled by flipper button. Pi only enables flipper generally (e.g. not tilt)
			((rxBuffer[1]>>7)&1) ?
				LL_TIM_CC_EnableChannel (TIM4, LL_TIM_CHANNEL_CH3) : LL_TIM_CC_EnableChannel (TIM4, LL_TIM_CHANNEL_CH3);*/
			/* OB9 */
			((rxBuffer[1]>>6)&1) ?
				LL_TIM_CC_EnableChannel (TIM4, LL_TIM_CHANNEL_CH4) : LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH4);
			/* OB1 */
			/* OB1 -> left sling shot controlled by switch -> see later in code. Pi only enables sling generally */
			/*((rxBuffer[1]>>5)&1) ?
				LL_TIM_CC_EnableChannel (TIM3, LL_TIM_CHANNEL_CH4) : LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH4);*/
			/* OB6 */
			((rxBuffer[1]>>4)&1) ?
				LL_TIM_CC_EnableChannel (TIM4, LL_TIM_CHANNEL_CH1) : LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH1);
			/* OB7 */
			((rxBuffer[1]>>3)&1) ?
				LL_TIM_CC_EnableChannel (TIM4, LL_TIM_CHANNEL_CH2) : LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH2);
			/* OA6 */
			/* OA6 -> Bumper controlled by switch -> see later in code. Pi only enables bumper generally */
			/*((rxBuffer[1]>>2)&1) ?
				LL_TIM_CC_EnableChannel (TIM3, LL_TIM_CHANNEL_CH1) : LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH1);*/
			/* OA7 */
			/* OA6 -> Bumper controlled by switch -> see later in code. Pi only enables bumper generally */
			/*((rxBuffer[1]>>1)&1) ?
				LL_TIM_CC_EnableChannel (TIM3, LL_TIM_CHANNEL_CH2) : LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH2);*/
			/* OB0 */
			/* OA6 -> Bumper controlled by switch -> see later in code. Pi only enables bumper generally */
			/*((rxBuffer[1]>>0)&1) ?
				LL_TIM_CC_EnableChannel (TIM3, LL_TIM_CHANNEL_CH3) : LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH3);*/

		}

	    /* Poll switches and remember if they where activated until they are reset to 0
	     * after sent to Pi */
		txSwitchesByte1 |= (
				LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_8)<<7 |
				LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_5)<<6 |
				LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_4)<<5 |
				LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_3)<<4 |
				LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_2)<<3 |
				LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_1)<<2 |
				LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_13)<<1 |
				LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_12));
		txSwitchesByte2 |= (
				LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_10)<<7 |
				LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_5)<<6 |
				LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_4)<<5 |
				LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_15)<<4 |
				LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_15)<<3 |
				LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_14)<<2 |
				LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_15)<<1 |
				LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_14));

		/* set fast solenoids (controlled solenoids by stwitch) */

		/* right flipper Button pressed and enabled, don't use
		 * txSwitchesByte1 here (as they stay 1 until transferred to Pi) */
		if ( (LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_15)) && ((rxBuffer[1]>>7)&1) )
		{
			LL_TIM_CC_EnableChannel (TIM4, LL_TIM_CHANNEL_CH3);
		}
		else
		{
			if (flipperPulse > 420000) /* minimum on time is 5 ms to debounce flipper button (if it bounces the flipper doesn't get full power) */
			{
				LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH3);
				flipperPulse = 0;
			}
		}
		if ( LL_TIM_CC_IsEnabledChannel(TIM4, LL_TIM_CHANNEL_CH3) ) flipperPulse += stTicks;

		/* rising edge of left sling shot switch */
		if ( ((txSwitchesByte2>>1)&1) && !((prevTxSwitchByte2>>1)&1) && ((rxBuffer[1]>>5)&1) && !slingPulse )
		{
			slingPulse = 1;
			LL_TIM_CC_EnableChannel (TIM3, LL_TIM_CHANNEL_CH4);
		}
		if ( slingPulse ) slingPulse += stTicks;
		if ( slingPulse > 3360000 ) LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH4); /* 40 ms @ 84 MHz */
		if ( slingPulse > 33600000 ) slingPulse = 0; /* anti bouncing 400 ms @ 84 MHz */

		/* rising edge of right bumper switch */
		if ( ((txSwitchesByte1>>7)&1) && !((prevTxSwitchByte1>>7)&1) && ((rxBuffer[1]>>0)&1) &&!bumprightPulse )
		{
			bumprightPulse = 1;
			LL_TIM_CC_EnableChannel (TIM3, LL_TIM_CHANNEL_CH3);
		}
		if ( bumprightPulse ) bumprightPulse += stTicks;
		if ( bumprightPulse > 3360000 ) LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH3); /* 40 ms @ 84 MHz */
		if ( bumprightPulse > 33600000 ) bumprightPulse = 0; /* anti bouncing 400 ms @ 84 MHz */

		/* rising edge of middle bumper switch */
		if ( ((txSwitchesByte1>>6)&1) && !((prevTxSwitchByte1>>6)&1) && ((rxBuffer[1]>>1)&1) && !bumpmidPulse )
		{
			bumpmidPulse = 1;
			LL_TIM_CC_EnableChannel (TIM3, LL_TIM_CHANNEL_CH2);
		}
		if ( bumpmidPulse ) bumpmidPulse += stTicks;
		if ( bumpmidPulse > 3360000 ) LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH2); /* 40 ms @ 84 MHz */
		if ( bumpmidPulse > 33600000 ) bumpmidPulse = 0; /* anti bouncing 400 ms @ 84 MHz */

		/* rising edge of left bumper switch */
		if ( ((txSwitchesByte1>>5)&1) && !((prevTxSwitchByte1>>5)&1) && ((rxBuffer[1]>>2)&1) && !bumpleftPulse )
		{
			bumpleftPulse = 1;
			LL_TIM_CC_EnableChannel (TIM3, LL_TIM_CHANNEL_CH1);
		}
		if ( bumpleftPulse ) bumpleftPulse += stTicks;
		if ( bumpleftPulse > 3360000 ) LL_TIM_CC_DisableChannel (TIM3, LL_TIM_CHANNEL_CH1); /* 40 ms @ 84 MHz */
		if ( bumpleftPulse > 33600000 ) bumpleftPulse = 0; /* anti bouncing 400 ms @ 84 MHz */

		/* rising edge of kickback switch */
		if ( ((txSwitchesByte2>>2)&1) && !((prevTxSwitchByte2>>2)&1) && (((rxBuffer[1]>>6)&1) | kickbackGrace) && !kickbackPulse )
		{
			kickbackPulse = 1;
			LL_TIM_CC_EnableChannel (TIM4, LL_TIM_CHANNEL_CH4);
			kickbackGrace = 1;
		}
		if ( kickbackPulse ) kickbackPulse += stTicks;
		if ( kickbackPulse > 3360000 ) LL_TIM_CC_DisableChannel (TIM4, LL_TIM_CHANNEL_CH4); /* 40 ms @ 84 MHz */
		if ( kickbackPulse > 8400000 ) kickbackPulse = 0; /* anti bouncing 100 ms @ 84 MHz */
		if ( kickbackGrace ) kickbackGrace += stTicks;
		if ( kickbackGrace > 420000000 ) kickbackGrace = 0; /* grace period 4 s @ 84 MHz */


		/* send status to Pi, currently only switch states */
		if ( nodeStatus == 1 )
		{
			/* wait a little (minimum half a bit with) to be sure that TransmitComplete of previous node is done and RS485 bus is free */
			if ( readyToSendCounter > 4200 ) /* 50 µs @ 84 MHz */
			{
				if ( LL_USART_IsActiveFlag_TXE(USART1) && (!txByte1_transmitted) )
				{
					LL_GPIO_SetOutputPin (GPIOA, RS485_nWrite_Pin);
					LL_USART_TransmitData8 (USART1, txSwitchesByte1);
					txByte1_transmitted = 1;
				}
				if ( LL_USART_IsActiveFlag_TXE(USART1) && txByte1_transmitted ) {
						LL_USART_TransmitData8 (USART1, txSwitchesByte2);
						nodeStatus = 0;
						txByte1_transmitted = 0;
						readyToSendCounter = 0;
						txSwitchesByte1 = 0;
						txSwitchesByte2 = 0;
				}
			}
			readyToSendCounter += stTicks;

			 /* Timeout sendStatus after 5 ms @ 84 MHz */
			if ( readyToSendCounter > 420000 )
			{
				nodeStatus = 0;
				readyToSendCounter = 0;
				txByte1_transmitted = 0;
			}
		}

		prevTxSwitchByte1 = txSwitchesByte1;
		prevTxSwitchByte2 = txSwitchesByte2;

		/* Paketintervall-Überwachung*/
		packetWatchDog += stTicks;
		/* toggle LED after 4 seconds */
		if ( packetWatchDog > 336000000 ) LL_GPIO_TogglePin (GPIOC, LL_GPIO_PIN_13);
		/* Timeout WatchDog: 5 s @ 84 MHz */
		if ( packetWatchDog > 420000000 )
		{
			NVIC_SystemReset();
		}

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(84000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA11   ------> TIM1_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 DMA Init */

  /* TIM2_CH2_CH4 Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_3);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  /* USER CODE BEGIN TIM2_Init 1 */
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)&TIM2->CCR2);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)tmp_led_data);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, 2*3*8);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_6);


  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 104;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */
  LL_TIM_EnableDMAReq_CC2(TIM2);              /* Enable DMA requests */


  /* USER CODE END TIM2_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM2 GPIO Configuration
  PB3   ------> TIM2_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 16;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 10000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = SOLFORCE25;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PA6   ------> TIM3_CH1
  PA7   ------> TIM3_CH2
  PB0   ------> TIM3_CH3
  PB1   ------> TIM3_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 16;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 10000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = SOLFORCE50;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.CompareValue = 9000;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.CompareValue = 10000;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.CompareValue = SOLFORCE50;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration
  PB6   ------> TIM4_CH1
  PB7   ------> TIM4_CH2
  PB8   ------> TIM4_CH3
  PB9   ------> TIM4_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  LL_USART_EnableIT_RXNE (USART1);
  LL_USART_EnableIT_TC (USART1);


  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(BlueLED_GPIO_Port, BlueLED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(RS485_nWrite_GPIO_Port, RS485_nWrite_Pin);

  /**/
  GPIO_InitStruct.Pin = BlueLED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BlueLED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = KeySwitch_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(KeySwitch_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_4
                          |LL_GPIO_PIN_5|LL_GPIO_PIN_8|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14
                          |LL_GPIO_PIN_15|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RS485_nWrite_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RS485_nWrite_GPIO_Port, &GPIO_InitStruct);

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

