
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>

#include "nsio.h"

#include "usbd_core.h"
#include "rgboled.h"
#include "flash.h"
#include "logo.h"
#include "ble.h"
#include "mpu.h"
#include "max30102.h"

#define VERSION "dev180529"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
RGBOLED* dev;
Flash* flash;
uint8_t flashOK = 0;

BLE* ble;
MPUnit* mpu;
MAX30102* max30102;
uint32_t red, ir;

FATFS fileSystem;
FIL testFile;
uint8_t testBuffer[16] = "Hello Gensokyo!\0";
UINT testBytes;
FRESULT res;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RNG_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
float getBatV() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 5);
	uint16_t data = HAL_ADC_GetValue(&hadc1);
	return (float) data / 4095.0F * 3.3F * 2.0F;
}

void extTest() {
	if (HAL_GPIO_ReadPin(B13_GPIO_Port, B13_Pin) == GPIO_PIN_RESET) {
		if (HAL_GPIO_ReadPin(B15_GPIO_Port, B15_Pin) == GPIO_PIN_RESET) {
			print("Erase flash...\n");
			flash->eraseAll(flash->p);
			while (flash->busy(flash->p));
			print("Erase done.\n");
		} else {
			print("Test flash...\n");
			for (uint8_t n = 0; n < 32; n++) {
				print("Test cycle: %d\n", n + 1);
				uint8_t buf[512]; uint32_t addr = rand() & 0xFFFE00;
				print("  Addr: 0x%X\n", addr);
				memset(buf, 0x32, 512);
				flash->write512byte(flash->p, addr, buf);
				memset(buf, 0x00, 512);
				flash->read512byte(flash->p, addr, buf);
				uint16_t diff = 0;
				for (uint16_t i = 0; i < 512; i++)
					if (buf[i] != 0x32) diff++;
				print("  Diff: %d\n", diff);
				HAL_Delay(1000);
			}
		}
	}

	if (HAL_GPIO_ReadPin(B12_GPIO_Port, B12_Pin) == GPIO_PIN_RESET) {
		f_mount(&fileSystem, USERPath, 1);
		if(f_mount(&fileSystem, USERPath, 1) == FR_OK) {
			uint8_t path[13] = "NYA_GAME.TXT";
			path[12] = '\0';
			res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
			res = f_write(&testFile, testBuffer, 16, &testBytes);
			res = f_close(&testFile);

			print("FATFS OK...\n");
		} else {
			print("FATFS Error\n");
		}
	}

	if (HAL_GPIO_ReadPin(B14_GPIO_Port, B14_Pin) == GPIO_PIN_RESET) {
		print("Init USB...\n");
		USBD_Start(&hUsbDeviceFS);
		print("USB initialized\n");
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	USBD_Stop(&hUsbDeviceFS);
	HAL_Delay(1000);

	dev = SoftRGBInit(
			OLED_SDA_GPIO_Port, OLED_SDA_Pin,
			OLED_SCL_GPIO_Port, OLED_SCL_Pin,
			OLED_DC_GPIO_Port, OLED_DC_Pin,
			OLED_CS_GPIO_Port, OLED_CS_Pin,
			OLED_RST_GPIO_Port, OLED_RST_Pin);

	dev->reset(dev->p);
	dev->init(dev->p);

	dev->colorb(dev->p, 0xFFFFFF);
	dev->colorf(dev->p, 0x000000);
	dev->clear(dev->p);
	dev->bitmapsc(dev->p, 63, 63, 64, 64, getLogo());

	HAL_Delay(3000);
	dev->colorb(dev->p, 0x000000);
	dev->colorf(dev->p, 0xFFFFFF);
	dev->clear(dev->p);

	if (HAL_GPIO_ReadPin(B15_GPIO_Port, B15_Pin) == GPIO_PIN_RESET) {
		dev->bitmap(dev->p, 0 ,0 ,128, 128, getImage());
		HAL_Delay(3000);
		while (HAL_GPIO_ReadPin(B15_GPIO_Port, B15_Pin) == GPIO_PIN_SET);
		dev->clear(dev->p);
	}

	//HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);

	print("==============\n");
	print("PPGColle v1.0\n");
	print("by drzzm32\n");
	print("==============\n");
	HAL_Delay(1000);

	print("Init MAX30102...\n");
	max30102 = MAX30102Init(
			&hi2c1, MAX_30102_ADDR,
			MAX_INT_GPIO_Port, MAX_INT_Pin);
	max30102->reset(max30102->p);
	max30102->clrint(max30102->p);
	max30102->init(max30102->p);
	max30102->waitint(max30102->p);
	max30102->fifo(max30102->p, &red, &ir);
	print("  Red: %d\n", red);
	print("  IR: %d\n", ir);
	print("\n");
	HAL_Delay(3000);

	print("Init MPU...\n");
	mpu = MPUInit(&huart2, 9.8F);
	mpu->reset(mpu->p);
	HAL_Delay(1000);
	float x = 0.0F, y = 0.0F, z = 0.0F;
	while (x == 0.0F) mpu->acc(mpu->p, &x, &y, &z);
	float sum = sqrt(x * x + y * y + z * z);
	print("  AccX: %f\n", x);
	print("  AccY: %f\n", y);
	print("  AccZ: %f\n", z);
	print("  AccSum: %f\n", sum);
	print("  Temper: %f\n", mpu->p->temper);
	print("\n");
	HAL_Delay(3000);

	print("Init flash...\n");
	flash = FlashInit(&hspi2, FLASH_CS_GPIO_Port, FLASH_CS_Pin, W25Q128);
	flashOK = flash->begin(flash->p);
	print("  State: %d\n", flashOK);
	print("  PID: 0x%X\n", flash->readPartID(flash->p));
	print("  UID: 0x%X\n", flash->readUniqueID(flash->p));
	print("\n");
	HAL_Delay(3000);

	print("Init BLE...\n");
	ble = BLEInit(&huart1,
			BLE_RST_GPIO_Port, BLE_RST_Pin,
			BLE_STE_GPIO_Port, BLE_STE_Pin,
			BLE_LED_GPIO_Port, BLE_LED_Pin);
	ble->reset(ble->p);
	HAL_Delay(500);
	ble->write(ble->p, "AT");
	HAL_Delay(500);
	ble->write(ble->p, "AT+NAME");
	HAL_Delay(500);
	ble->write(ble->p, "AT+ROLE");
	print("\n");

	print("Start BLE Rx...\n\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	char rxBuf[32];
	char txBuf[32];
  while (1)
  {
	  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, ble->led(ble->p));
	  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, ble->state(ble->p));

	  dev->colorf(dev->p, 0xFF9800);
	  dev->printf(dev->p, 96, 0, "%1.2fV", getBatV());
	  dev->colorf(dev->p, 0xFFFFFF);

	  if (ble->state(ble->p)) {
		  max30102->waitint(max30102->p);
		  max30102->fifo(max30102->p, &red, &ir);
		  sprintf(txBuf, "%d,%d", red, ir);
		  HAL_UART_Transmit(&huart1, (uint8_t*) txBuf, strlen(txBuf), 100);

		  HAL_UART_Receive(&huart1, (uint8_t*) rxBuf, 32, 100);
		  if (strlen(rxBuf) > 0)
			  print("%s\n", rxBuf);

		  memset(rxBuf, 0, 32);
		  memset(txBuf, 0, 32);
	  } else {
		  HAL_Delay(100);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RNG init function */
static void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA4   ------> SharedAnalog_PA4
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OLED_CS_Pin|FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_A_Pin|LED_B_Pin|OLED_SCL_Pin|OLED_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OLED_RST_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLE_RST_Pin|BLE_MODE_Pin|BLE_WKUP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : OLED_CS_Pin FLASH_CS_Pin OLED_RST_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_CS_Pin|FLASH_CS_Pin|OLED_RST_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_A_Pin LED_B_Pin OLED_SCL_Pin OLED_SDA_Pin */
  GPIO_InitStruct.Pin = LED_A_Pin|LED_B_Pin|OLED_SCL_Pin|OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_ADC_Pin */
  GPIO_InitStruct.Pin = BAT_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BAT_ADC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX_INT_Pin */
  GPIO_InitStruct.Pin = MAX_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MAX_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_RST_Pin BLE_MODE_Pin BLE_WKUP_Pin */
  GPIO_InitStruct.Pin = BLE_RST_Pin|BLE_MODE_Pin|BLE_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B12_Pin B13_Pin B14_Pin B15_Pin */
  GPIO_InitStruct.Pin = B12_Pin|B13_Pin|B14_Pin|B15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_CE_Pin */
  GPIO_InitStruct.Pin = BAT_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BAT_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_STE_Pin BLE_LED_Pin */
  GPIO_InitStruct.Pin = BLE_STE_Pin|BLE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
