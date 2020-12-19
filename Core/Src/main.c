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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDR (0x27 << 1)

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int timerRTC = 0;
int inc, ouc = 0;
char flagADC = 0;
char strTest[17] = { 0, };

int ain1, ain2, ainT;

RTC_TimeTypeDef sTime = { 0 };
RTC_DateTypeDef DateToUpdate = { 0 };

char trans_str[64] = { 0, };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data,
		uint8_t flags) {
	HAL_StatusTypeDef res;
	for (;;) {
		res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1,
		HAL_MAX_DELAY);
		if (res == HAL_OK)
			break;
	}

	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	data_arr[0] = up | flags | BACKLIGHT | PIN_EN;
	data_arr[1] = up | flags | BACKLIGHT;
	data_arr[2] = lo | flags | BACKLIGHT | PIN_EN;
	data_arr[3] = lo | flags | BACKLIGHT;

	res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, sizeof(data_arr),
	HAL_MAX_DELAY);
	HAL_Delay(LCD_DELAY_MS);
	return res;
}

HAL_StatusTypeDef Read_From_PCF8591(uint8_t DevAddress, uint8_t data,
		uint8_t *pData, uint8_t len) {
	HAL_StatusTypeDef returnValue;
	uint8_t data_arr[1];
	/* We compute the MSB and LSB parts of the memory address */
	data_arr[0] = data;
	/* First we send the command byte where start reading data */
	returnValue = HAL_I2C_Master_Transmit(&hi2c1, DevAddress, data_arr,
			sizeof(data_arr), HAL_MAX_DELAY);
	if (returnValue != HAL_OK)
		return returnValue;
	/* Next we can retrieve the data from ADC */
	returnValue = HAL_I2C_Master_Receive(&hi2c1, DevAddress, pData, len,
	HAL_MAX_DELAY);
	return returnValue;
}
HAL_StatusTypeDef Write_To_PCF8591(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint8_t *pData, uint16_t len) {
	HAL_StatusTypeDef returnValue;
	uint8_t *data;
	/* First we allocate a temporary buffer to store the destination memory
	 * address and the data to store */
	data = (uint8_t*) malloc(sizeof(uint8_t) * (len + 2));
	/* We compute the MSB and LSB parts of the memory address */
	data[0] = (uint8_t) ((MemAddress & 0xFF00) >> 8);
	data[1] = (uint8_t) (MemAddress & 0xFF);
	/* And copy the content of the pData array in the temporary buffer */
	memcpy(data + 2, pData, len);
	/* We are now ready to transfer the buffer over the I2C bus */
	returnValue = HAL_I2C_Master_Transmit(hi2c, DevAddress, data, len + 2,
	HAL_MAX_DELAY);
	if (returnValue != HAL_OK)
		return returnValue;
	free(data);
	/* We wait until the EEPROM effectively stores data in memory */
	while (HAL_I2C_Master_Transmit(hi2c, DevAddress, 0, 0, HAL_MAX_DELAY)
			!= HAL_OK)
		;
	return HAL_OK;
}

HAL_StatusTypeDef PCF8591_SendInternal(uint8_t adc_addr, uint8_t data) {
	HAL_StatusTypeDef res;
	for (;;) {
		res = HAL_I2C_IsDeviceReady(&hi2c1, adc_addr, 1,
		HAL_MAX_DELAY);
		if (res == HAL_OK)
			break;
	}

	uint8_t data_arr[1];

	data_arr[0] = data;

	res = HAL_I2C_Master_Transmit(&hi2c1, adc_addr, data_arr, sizeof(data),
	HAL_MAX_DELAY);
	HAL_Delay(LCD_DELAY_MS);
	return res;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
	LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
	LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void PCF8591_SendCommand(uint8_t adc_addr, uint8_t cmd) {
	PCF8591_SendInternal(adc_addr, cmd);
}

void PCF8591_Read(uint8_t adc_addr, uint8_t cmd) {
	PCF8591_SendInternal(adc_addr, cmd);
}

int MAP_Read() {
	int tmp = 256;
	Read_From_PCF8591(ADC_ADDR, 0b00000010, (uint8_t*) &tmp, 1);
	return tmp;
}

void LCD_Init(uint8_t lcd_addr) {
	// 4-bit mode, 2 lines, 5x7 format
	LCD_SendCommand(lcd_addr, 0b00110000);
	// display & cursor home (keep this!)
	LCD_SendCommand(lcd_addr, 0b00000010);
	// display on, right shift, underline off, blink off

//#define	LCD_CONTROL	0x08
//#define	LCD_DISPLAY	0x04
//#define	LCD_CURSOR	0x02
//#define	LCD_BLINK	0x01

	LCD_SendCommand(lcd_addr, 0b00001100);
	// clear display (optional here)
	LCD_SendCommand(lcd_addr, 0b00000001);
}

void LCD_SendString(uint8_t lcd_addr, char *str) {
	while (*str) {
		LCD_SendData(lcd_addr, (uint8_t) (*str));
		str++;
	}
}

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc) {
	timerRTC++;
	//Read_From_PCF8591(ADC_ADDR, 0b00000010, (uint8_t*) &ain1, 1);
	//HAL_Delay(1);
	//Read_From_PCF8591(ADC_ADDR, 0b00000011, (uint8_t*) &ain2, 1);
	if (flagADC == 1) {
		snprintf(trans_str, 63, "%d %d\n", timerRTC, (2294*ain2-16381)/4247);
		HAL_UART_Transmit(&huart1, (uint8_t*) trans_str, strlen(trans_str),
				1000);
		flagADC = 0;
	}

	//snprintf(trans_str, 63, "%04d U1=%04dmV U2=%04dmV\r\n", timerRTC, (255 - ain1) * 5000 / 255, (255 - ain2) * 5000 / 255);

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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	//LCD_Init(LCD_ADDR);
	HAL_Delay(100);

	//LCD_SendCommand(LCD_ADDR, 0b11000000);
	//LCD_SendString(LCD_ADDR, "RS485 test...   ");
	HAL_RTCEx_SetSecond_IT(&hrtc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_UART_Transmit(&huart1, "Test UART\r\n\0", 11, 15);
		//USART_SendData(USART1, RXc);
		//HAL_Delay(1000);
		if (flagADC == 0) {
			ainT = 0;
			int i;
			for (i = 0; i < 100; i++) {
				Read_From_PCF8591(ADC_ADDR, 0b00000010, (uint8_t*) &ain1, 1);
				if (ain1 < 256) {
					ain2 = ain2 + ain1;
					ainT++;
				}
				HAL_Delay(2);
			}
			ain2 = ain2 / ainT;
			flagADC = 1;

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  DateToUpdate.Month = RTC_MONTH_OCTOBER;
  DateToUpdate.Date = 14;
  DateToUpdate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
