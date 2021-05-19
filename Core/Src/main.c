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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/*Mise à jour le 25 janvier 2021*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stm32f429i_discovery_lcd.h"
#include "taskdef.h"
#include "myLib.h"
#include "const.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_B_TELECOM 0xFF065784
#define LCD_G_TELECOM 0xFF81B835
#define LCD_R_TELECOM 0xFF813C2F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t RTOS_RunTimeCounter;

void Calib_G(I2C_HandleTypeDef* hi2c3, double* p_g) {
	p_g[0] = 0;
	p_g[1] = 0;
	p_g[2] = 0;

	double p_bias_zero[3];
	p_bias_zero[0] = 0;
	p_bias_zero[1] = 0;
	p_bias_zero[2] = 0;

	double p_g_item[3];

	for (int i = 0; i < 100; ++i) {
		Measure_G(hi2c3, p_g_item, p_bias_zero);
		p_g[0] = p_g[0] + p_g_item[0];
		p_g[1] = p_g[1] + p_g_item[1];
		p_g[2] = p_g[2] +  p_g_item[2];
		HAL_Delay(10);
	}

	p_g[0] /= 100;
	p_g[1] /= 100;
	p_g[2] /= 100;
}

void RTOS_AppConfigureTimerForRuntimeStats(void)
{
  RTOS_RunTimeCounter = 0;
  HAL_TIM_Base_Start_IT(&htim7);
}

uint32_t RTOS_AppGetRuntimeCounterValueFromISR(void)
{
  return RTOS_RunTimeCounter;
}

/*Redirecting printf to USART1*/
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /*LEDs turned on*/
  	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
  	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);

 /*LCD screen initialization and test*/
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER,LCD_FRAME_BUFFER);
	BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER,LCD_FRAME_BUFFER);
	BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
	BSP_LCD_DisplayOn();
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_B_TELECOM);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"PROJET", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(2), (uint8_t *)"SYSTEMES", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(3), (uint8_t *)"TEMPS REEL", CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_G_TELECOM);
	BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"2020/2021", CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_R_TELECOM);
	BSP_LCD_DisplayStringAt(0, LINE(7), (uint8_t *)"TELECOM STE", CENTER_MODE);


	HAL_Delay(2000);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

/*LEDs turned off*/
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_RESET);

  /* USER CODE END 2 */

	char adresse[10];
	I2C_Scan(&hi2c3, adresse);
	Init_BMP280(&hi2c3);
	Init_MPU9250_AK8963(&hi2c3);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_B_TELECOM);
	I2C_Scan(&hi2c3, adresse); // On fait un deuxième scan pour détecter l'activation du magnétomètre
	uint8_t hal_mem = 0;
	double p_acc[3], p_mag[3], p_gyro[3], p_bias[3], p_scale[3], p_theta, p_press;
	long signed int p_Tfine;
	p_bias[0] = 0;
	p_bias[1] = 0;
	p_bias[2] = 0;

	p_scale[0] = 1;
	p_scale[0] = 1;
	p_scale[0] = 1;

	if(HAL_I2C_Mem_Read(&hi2c3, MPU_ADD, WHO_AM_I_MPU9250, sizeof(char), &hal_mem, sizeof(uint8_t), 1000) == HAL_ERROR) {
		Error_Handler();
	}

  /* Call init function for freertos objects (in freertos.c) */
 // MX_FREERTOS_Init();
  /* Start scheduler */
 // osKernelStart();
	Measure_G(&hi2c3, p_gyro, p_bias);
	printf("Vitesse angulaire avant calibration : Omega(%.2f, %.2f, %.2f)\r\n", p_gyro[0], p_gyro[1], p_gyro[2]);
	Calib_G(&hi2c3, p_bias);
	Measure_G(&hi2c3, p_gyro, p_bias);
	printf("Vitesse angulaire après calibration : Omega(%.2f, %.2f, %.2f)\r\n", p_gyro[0], p_gyro[1], p_gyro[2]);

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  Measure_A(&hi2c3, p_acc);
	  Measure_G(&hi2c3, p_gyro, p_bias);
	  Measure_M(&hi2c3, p_mag, p_bias, p_scale);
	  Measure_T_BMP280(&hi2c3, &p_theta, &p_Tfine);
	  Measure_T(&hi2c3, &p_theta);
	  Measure_P(&hi2c3, &p_press, &p_Tfine);

	  printf("A(%.2f; %.2f; %.2f) | Omega(%.2f; %.2f; %.2f) | B(%.2f; %.2f; %.2f) | Temperature : %.2f | Pression : %.2f  \r\n", p_acc[0], p_acc[1], p_acc[2], p_gyro[0], p_gyro[1], p_gyro[2], p_mag[0], p_mag[1], p_mag[2], p_theta, p_press);

	  HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 60;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void debugPrint(UART_HandleTypeDef *huart, char _out[])
//{
//	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
//}
//
//void debugPrintln(UART_HandleTypeDef *huart, char _out[])
//{
//	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
//	char newline[2] = "\r\n";
//	HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
//}
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
