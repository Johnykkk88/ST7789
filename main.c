/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ST7789.h>
#include "fonts.h"
#include "stdio.h"
#include "analog_scale.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t  time = 0;
int val = 0;
int button_counter = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct {
	TIM_TypeDef *htim_bk;		//------- для подсветки с PWM:- таймер
	uint32_t channel_htim_bk;	//----------------------------- канал таймера

	GPIO_TypeDef *blk_port;		//просто для включения и выключения подсветки, если htim_bk = 0 (без PWM, определен порт вывода)
	uint16_t blk_pin;			//----------------------------------------------------------------------- пин порта

	uint8_t bk_percent;			//яркость подсветки для режима PWM, %
								//либо 0 - подсветка отключена, > 0 подсветка включена, если если htim_bk = 0 (без PWM, определен порт вывода)
} LCD_BackLight_data;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char trans_str[64];
uint16_t adc = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  ST7789_Init();
  /*LCD_BackLight_data bkl_data = { TIM1,
		  	  	  	  	  	  	  TIM_CHANNEL_1,
 								  0,
 								  0,
 								  50  };*/
 ST7789_BacklightON();
  /*ST7789_FillScreen( RGB565(0, 0, 0) );
  ST7789_rotation( 1 );
  if((HAL_GetTick() - time) > 1000) // интервал 1000мс = 1сек
          {
     	ST7789_print( 30, 120, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "Welcome to" );
     	  	ST7789_print( 50, 150, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "SkySurf" );
                  time = HAL_GetTick();
          }
 	  	ST7789_FillScreen( RGB565(0, 0, 0) );*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 /* HAL_ADC_Start(&hadc1); // запускаем преобразование сигнала АЦП
	           HAL_ADC_PollForConversion(&hadc1, 100); // ожидаем окончания преобразования
	           adc = HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
	           HAL_ADC_Stop(&hadc1); // останавливаем АЦП (не обязательно)
	           ST7789_print( 40, 40, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "ADC = ");
	           //0ST7789_printADC( 130, 40, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, adc);

	           HAL_Delay(1000);
*/
	           //HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
	           //HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //ST7789_BacklightON();

	  	 	ST7789_print( 40, 40, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "Mode" );
	  	 	ST7789_print( 40, 70, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "Parameters" );
	  	  	ST7789_print( 40, 100, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "Settings" );
	  	  	ST7789_print( 40, 130, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "FW Update" );

	  			  if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)==0)
	  			  {
	  				  val++;
	  				  //HAL_Delay(1000);
	  			  }

	  	  	switch (val) {
	  	  case 1:
	  		ST7789_print( 20, 40, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		ST7789_print( 20, 70, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, ">" );
	  		ST7789_print( 20, 100, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		ST7789_print( 20, 130, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  	    break;
	  	  case 2:
	  		ST7789_print( 20, 40, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		ST7789_print( 20, 70, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		ST7789_print( 20, 100, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, ">" );
	  		ST7789_print( 20, 130, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  	    break;
	  	  case 3:
	  		ST7789_print( 20, 40, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		ST7789_print( 20, 70, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		ST7789_print( 20, 100, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		ST7789_print( 20, 130, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, ">" );
	  		break;
	  	  default:
	  		ST7789_print( 20, 40, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, ">" );
	  		ST7789_print( 20, 70, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		ST7789_print( 20, 100, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		ST7789_print( 20, 130, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, " " );
	  		val = 0;
	  	    break;
	  	  }


	  	// установка ротации дисплея не обязательно так как по умолчанию стоит режим 1 ( всего 4 режима 1,2,3,4 )

	  // рисуем треугольник пустотелый
	  	//ST7789_DrawTriangle(20, 50, 100, 50, 50, 50, RGB565(0, 0, 0) );
/*
	  	ST7789_print( 20, 40, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, ">" );
	 	ST7789_print( 40, 40, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "Mode" );

	 	ST7789_print( 40, 70, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "Parameters" );
	  	ST7789_print( 40, 100, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "Settings" );
	  	ST7789_print( 40, 130, RGB565(255, 255, 255) , RGB565(0, 0, 0) , 1, &Font_16x26, 1, "FW Update" );
	  	ST7789_InversionMode(0);
*/
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
