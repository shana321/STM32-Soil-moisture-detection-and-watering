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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

#include "hal_delay.h"
#include "Sofei2cMaster.h"
#include "oled.h"
#include "dht11.h"
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

OLED_t my_oled;
DHT11_t my_dht11;
uint8_t usart_sendbuf[128];
float humi,temp;
uint16_t AD_Value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t stm32_oled_cb(OLED_t *slf, OLED_CB_MSG msg, uint8_t data_byte);
//uint8_t stm32_i2c_cb(Sofei2cMaster *i2c, uint8_t msg);
uint8_t DHT11_STM32_cb(DHT11_t *dht, uint8_t msg);
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
	delay_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	delay_us(1000000);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	
	New_OLED(&my_oled, stm32_oled_cb);
	DHT11_Init(&my_dht11, DHT11_STM32_cb);
	OLED_Init(&my_oled);
	OLED_ShowString(&my_oled, 0, 4, "hello world!", 16);
	char *str = "RS232\r\n";
	uint16_t len = strlen(str);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)str, len);
//	fill_picture(&my_oled, 0xff);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		DHT11_Read(&my_dht11);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);//等待转换完成， 第二个参数表示超时时间，单位ms.
		if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))	
			AD_Value = HAL_ADC_GetValue(&hadc1);
		HAL_Delay(1000);
		DHT11_Fget(&my_dht11, &humi, &temp);
		snprintf((char*)usart_sendbuf, sizeof(usart_sendbuf), "Temp:%.1f, Humi:%.1f, AD_Value:%d\r\n", temp, humi, AD_Value);
		HAL_UART_Transmit_IT(&huart1, usart_sendbuf, strlen((char*)usart_sendbuf));
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
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
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_I2C_SCL_Pin|OLED_I2C_SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_I2C_SCL_Pin OLED_I2C_SDA_Pin */
  GPIO_InitStruct.Pin = OLED_I2C_SCL_Pin|OLED_I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//i2c底层操作回调
uint8_t stm32_i2c_cb(Sofei2cMaster *i2c, uint8_t msg)
{
	switch(msg)
	{
		case SI2C_MSG_INIT:					break;	//hal库已初始化IO
		case SI2C_MSG_SET_SDA:			OLED_I2C_SDA_GPIO_Port->ODR |= OLED_I2C_SDA_Pin;break;
		case SI2C_MSG_RESET_SDA:		OLED_I2C_SDA_GPIO_Port->ODR &= ~OLED_I2C_SDA_Pin;break;
		case SI2C_MSG_READ_SDA:			return OLED_I2C_SDA_GPIO_Port->IDR & OLED_I2C_SDA_Pin;
		case SI2C_MSG_OUTPUT_SDA:		break; 
		case SI2C_MSG_INPUT_SDA:		break;	

		case SI2C_MSG_OUTPUT_SCL:		break;
		case SI2C_MSG_SET_SCL:			OLED_I2C_SCL_GPIO_Port->ODR |= OLED_I2C_SCL_Pin;break;
		case SI2C_MSG_RESET_SCL:		OLED_I2C_SCL_GPIO_Port->ODR &= ~OLED_I2C_SCL_Pin;break;
//		case SI2C_MSG_BUF_DELAY:  	delay_ns(1300);break;	//停止信号与开始信号间的总线释放时间(SSD1306、TMP75为1.3us)
//		case SI2C_MSG_SUSTA_DELAY:	delay_ns(600);break;	//重复开始信号的建立时间(SCL=1的最短时间，SSD1306、TMP75为0.6us)
//		case SI2C_MSG_HDSTA_DELAY:	delay_ns(600);break;	//开始信号的保持时间(SDA拉低的最短时间，在这之后产生第一个时钟信号，SSD1306、TMP75为0.6us)
//		case SI2C_MSG_SUSTO_DELAY:	delay_ns(600);break;	//停止信号的建立时间（SDA拉高前SCL=1，SDA=0的保持时间，SSD1306、TMP75为0.6us)
//		case SI2C_MSG_HDDAT_DELAY:	break;								//数据保持时间(SCL=0后数据需要保持不变的时间，TMP75为4ns,很短,IO频率超过250M才需要?)
//		case SI2C_MSG_SUDAT_DELAY:	delay_ns(100);break;	//数据建立时间(SCL=1前SDA需要保持状态的时间，SSD1306、TMP75为100ns，在读写时序中被SCL延时覆盖，在ack中需使用，若频率低于10M则不需要?)
//		case SI2C_MSG_SCLL_DELAY:		delay_us(2);break;		//SCL低电平时间(读写时序中的主要延时，TMP75为1.3us min)
//		case SI2C_MSG_SCLH_DELAY:		delay_us(2);break;		//SCL高电平时间(读写时序中的主要延时, TMP75为0.6us min)
		default: break;//这里都不需要使用额外延时，2层回调 即使168M的STM32F4，调用回调的时间也已经够长了，当然不注释掉要使用延时也可以
		//对于有些外设，读的时候需要适当慢一点，要不然外设反应不过来
	}
	return 0;
}
uint8_t stm32_oled_cb(OLED_t *slf, OLED_CB_MSG msg, uint8_t data_byte)
{
	static Sofei2cMaster oled_i2c;
	switch (msg)
	{
		case OLED_MSG_I2C_INIT:
		{
			New_Sofei2cMaster(&oled_i2c, stm32_i2c_cb);
			Sofei2cMaster_init(&oled_i2c);
			break;
		}
		case OLED_MSG_I2C_START:
		{
			Sofei2cMaster_start(&oled_i2c);
			break;
		}
		case OLED_MSG_I2C_STOP:
		{
			Sofei2cMaster_stop(&oled_i2c);
			break;
		}
		case OLED_MSG_I2C_WAIT_ACK:
		{
			Sofei2cMaster_wait_ack(&oled_i2c);
			break;
		}
		case OLED_MSG_I2C_WRITE_BYTE:
		{
			Sofei2cMaster_send_byte(&oled_i2c, data_byte);
			break;
		}

		default: break;
	}
	return 0;
}
uint8_t DHT11_STM32_cb(DHT11_t *dht, uint8_t msg)
{
	//这里未使用DHT11对象指针，因为我这里只需要去驱动1个DHT11
	switch(msg)
	{
	case DHT11_MSG_2S_DELAY://用于上电初始化稳定DHT11
		delay_us(2000000);
	break;
	case DHT11_MSG_18MS_DELAY:
			delay_us(18000);
	break;
	case DHT11_MSG_1MS_DELAY:
		delay_us(1000);
	break;
	
	case DHT11_MSG_30US_DELAY:
		delay_us(30);
	break;
	case DHT11_MSG_READ_SDA: 
		DHT11_GPIO_Port->ODR |= DHT11_Pin;
		return HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin);
	case DHT11_MSG_SDA_HIGH:
				DHT11_GPIO_Port->ODR |= DHT11_Pin;
	break;
	case DHT11_MSG_SDA_LOW:
				DHT11_GPIO_Port->ODR &= ~DHT11_Pin;
	break;
	default:return 0xFF;
	}
	return 0;
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
