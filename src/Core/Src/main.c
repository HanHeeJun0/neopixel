/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdbool.h>

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

#define LED   21
#define LED_BRIGHTNESS 125 // 0~255
#define HIGH  40
#define LOW   19

bool is_init = false;

typedef struct
{
  uint16_t LED_CNT;
} ws2812_t;

static uint8_t led_buf[280 + 24*LED];

ws2812_t ws2812;
extern TIM_HandleTypeDef htim3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool ws2812Init(void)
{
  memset(led_buf, 0, sizeof(led_buf));

  is_init = true;


  return true;
}

void ws2812Begin(uint32_t led_cnt)
{
  ws2812.LED_CNT = led_cnt;


  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)led_buf, (280 + 24 * led_cnt) * 1);
}

void ws2812SetColor(uint32_t index, uint8_t red, uint8_t green, uint8_t blue)
{
  uint8_t r_bit[8];
  uint8_t g_bit[8];
  uint8_t b_bit[8];

  uint32_t offset;


  for (int i=0; i<8; i++)
  {
    if (red & (1<<7))
    {
      r_bit[i] = HIGH;
    }
    else
    {
      r_bit[i] = LOW;
    }
    red <<= 1;

    if (green & (1<<7))
    {
      g_bit[i] = HIGH;
    }
    else
    {
      g_bit[i] = LOW;
    }
    green <<= 1;

    if (blue & (1<<7))
    {
      b_bit[i] = HIGH;
    }
    else
    {
      b_bit[i] = LOW;
    }
    blue <<= 1;
  }

  offset = 280;

    memcpy(&led_buf[offset + index*24 + 8*0], g_bit, 8*1);
    memcpy(&led_buf[offset + index*24 + 8*1], r_bit, 8*1);
    memcpy(&led_buf[offset + index*24 + 8*2], b_bit, 8*1);
}

void offLed() {
	for(int i=0; i<21; i++)
	{
		ws2812SetColor(i, 0, 0, 0);
	}
}

void rgbLed() {
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<21; j++)
		{
			if(i==0) {
			  ws2812SetColor(j, 255, 0, 0);
			} else if(i==1) {
			  ws2812SetColor(j, 0, 255, 0);
			} else if(i==2) {
			  ws2812SetColor(j, 0, 0, 255);
			}
			HAL_Delay(20);
		}
	}
	offLed();
}

void sirenLed() {
	for(int n=0; n<10; n++)
	{
		for(int i=0; i<8; i++)
		{
			ws2812SetColor(i, 255, 0, 0);
		}
		for(int i=8; i<21; i++)
		{
			ws2812SetColor(i, 0, 0, 255);
		}
		HAL_Delay(100);
		for(int i=0; i<8; i++)
		{
			ws2812SetColor(i, 0, 0, 255);
		}
		for(int i=8; i<21; i++)
		{
			ws2812SetColor(i, 255, 0, 0);
		}
		HAL_Delay(100);
	}
	offLed();
}

void blinkLed() {
	for(int n=0; n<3; n++)
	{
		for(int i=0; i<21; i++)
		{
			ws2812SetColor(i, 255, 255, 255);
		}
		HAL_Delay(100);
		offLed();
		HAL_Delay(100);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  ws2812Init();
  ws2812Begin(21);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  blinkLed();
	  rgbLed();
	  sirenLed();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV128;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
