/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "ssd1306.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

uint16_t counter, Xpos, Ypos;

volatile uint8_t flagADC = 0;
volatile uint16_t adc[10] = { 0, }; // у нас два канала поэтому массив из двух элементов

uint8_t Tsp1 = 0;
uint8_t Tsp2 = 0;

uint16_t TP1 = 0;
uint16_t TP2 = 0;

uint8_t flag_xpos_press = 1;
uint8_t flag_xpos_wait = 1;
uint32_t time_xpos_press = 0;

uint8_t flag_ypos_press = 1;
uint8_t flag_ypos_wait = 1;
uint32_t time_ypos_press = 0;

// CAN private variables
char trans_str[128] = { 0, };

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = { 0, };
uint8_t RxData[8] = { 0, };
uint32_t TxMailbox = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		flagADC = 1;
//		counterDMA++;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);

		if (RxHeader.StdId == 0x0200) {
			TP1 = (RxData[1] << 8) | RxData[0];
			TP2 = (RxData[3] << 8) | RxData[2];
		} else if (RxHeader.StdId == 0x0126) {
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
		}
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	uint32_t er = HAL_CAN_GetError(hcan);
	sprintf(trans_str, "ER CAN %lu %08lX", er, er);

	ssd1306_SetCursor(0, 54);
	ssd1306_WriteString(trans_str, Font_6x8, White);
	ssd1306_UpdateScreen();

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_CAN_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(50);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_Delay(50);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc, 10); // стартуем АЦП
	TxHeader.StdId = 0x0378;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = 0;

	ssd1306_Init();

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Tsp1   Tsp2", Font_11x18, White);
	ssd1306_SetCursor(0, 18);
	sprintf(trans_str, "%03d%%   %03d%%", Tsp1, Tsp2);
	ssd1306_WriteString(trans_str, Font_11x18, White);
	ssd1306_UpdateScreen();

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	HAL_Delay(100);

	HAL_CAN_Start(&hcan);

	HAL_CAN_ActivateNotification(&hcan,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF
					| CAN_IT_LAST_ERROR_CODE);

	int i;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		//TxHeader.StdId = 0x0378;
		//TxData[0] = 90;

		if (flagADC) {
			Xpos = 0;
			Ypos = 0;
			for (i = 0; i < 10; i = i + 2) {
				Xpos = Xpos + adc[i];
				Ypos = Ypos + adc[i + 1];
			}
			Xpos /= 50;
			Ypos /= 50;
			//HAL_Delay(500);

			flagADC = 0;
		}

		if ((Xpos < 150 || Xpos > 250) && flag_xpos_press) {
			flag_xpos_press = 0;
			flag_xpos_wait = 0;
			time_xpos_press = HAL_GetTick();
		}

		if (!flag_xpos_wait && (HAL_GetTick() - time_xpos_press) > 50) {
			if (Xpos < 10) {
				// действие на нажатие
				if (Tsp1 > 5)
					Tsp1 -= 5;
				else
					Tsp1 = 0;
				flag_xpos_wait = 1;
			} else if (Xpos < 150 && Xpos > 10) {
				// действие на нажатие
				if (Tsp1 > 1)
					Tsp1 -= 1;
				else
					Tsp1 = 0;
				flag_xpos_wait = 1;
			} else if (Xpos > 390) {
				// действие на нажатие
				if (Tsp1 < 95)
					Tsp1 += 5;
				else
					Tsp1 = 100;
				flag_xpos_wait = 1;
			} else if (Xpos > 250) {
				// действие на нажатие
				if (Tsp1 < 99)
					Tsp1 += 1;
				else
					Tsp1 = 100;
				flag_xpos_wait = 1;
			} else {
				flag_xpos_wait = 1;
				flag_xpos_press = 1;
			}
		}
		if ((Xpos < 50 || Xpos > 350) && !flag_xpos_press
				&& (HAL_GetTick() - time_xpos_press) > 100)
			flag_xpos_press = 1;
		if ((Xpos < 100 || Xpos > 300) && !flag_xpos_press
				&& (HAL_GetTick() - time_xpos_press) > 250)
			flag_xpos_press = 1;
		if (!flag_xpos_press && (HAL_GetTick() - time_xpos_press) > 500)
			flag_xpos_press = 1;

		if ((Ypos < 150 || Ypos > 250) && flag_ypos_press) {
			flag_ypos_press = 0;
			flag_ypos_wait = 0;
			time_ypos_press = HAL_GetTick();
		}

		if (!flag_ypos_wait && (HAL_GetTick() - time_ypos_press) > 50) {
			if (Ypos <= 3) {
				// действие на нажатие
				if (Tsp2 > 5)
					Tsp2 -= 5;
				else
					Tsp2 = 0;
				flag_ypos_wait = 1;
			} else if (Ypos < 150 && Ypos > 3) {
				// действие на нажатие
				if (Tsp2 > 1)
					Tsp2 -= 1;
				else
					Tsp2 = 0;
				flag_ypos_wait = 1;
			} else if (Ypos > 405) {
				// действие на нажатие
				if (Tsp2 < 95)
					Tsp2 += 5;
				else
					Tsp2 = 100;
				flag_ypos_wait = 1;
			} else if (Ypos > 250) {
				// действие на нажатие
				if (Tsp2 < 99)
					Tsp2 += 1;
				else
					Tsp2 = 100;
				flag_ypos_wait = 1;
			} else {
				flag_ypos_wait = 1;
				flag_ypos_press = 1;
			}
		}
		if ((Ypos < 50 || Ypos > 350) && !flag_ypos_press
				&& (HAL_GetTick() - time_ypos_press) > 100)
			flag_ypos_press = 1;
		if ((Ypos < 100 || Ypos > 300) && !flag_ypos_press
				&& (HAL_GetTick() - time_ypos_press) > 250)
			flag_ypos_press = 1;
		if (!flag_ypos_press && (HAL_GetTick() - time_ypos_press) > 500)
			flag_ypos_press = 1;

		ssd1306_SetCursor(0, 18);
		sprintf(trans_str, "%03d%%   %03d%%", Tsp1, Tsp2);
		ssd1306_WriteString(trans_str, Font_11x18, White);
		ssd1306_UpdateScreen();
		ssd1306_SetCursor(0, 36);

		sprintf(trans_str, "%04d   %04d", TP1, TP2);
		ssd1306_WriteString(trans_str, Font_11x18, White);
		ssd1306_UpdateScreen();

		TxHeader.StdId = 0x0100;

		TxData[0] = Tsp1;
		TxData[1] = Tsp2;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
//		while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
//			;
//
//		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox)
//				!= HAL_OK) {
//			ssd1306_SetCursor(0, 36);
//					ssd1306_WriteString("CAN ERROR", Font_6x8, White);
//					ssd1306_UpdateScreen();
//		}

		//while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);

		//if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		//	HAL_UART_Transmit(&huart1, (uint8_t*) "ER SEND\n", 8, 100);
		//}

		//HAL_Delay(500);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef sFilterConfig;
	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 4;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = ENABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	//sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB3 PB4 PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure peripheral I/O remapping */
	__HAL_AFIO_REMAP_SPI1_ENABLE();

	/*Configure peripheral I/O remapping */
	__HAL_AFIO_REMAP_USART1_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
