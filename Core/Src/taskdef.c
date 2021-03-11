/*
 * taskdef.c
 *
 *  Created on: Jan 20, 2021
 *      Author: florentgoutailler
 */

#include "main.h"
#include "stm32f429i_discovery_lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include "taskdef.h"

UART_HandleTypeDef huart1;

/******************************************/
/*Tâche de test*/
void vTask0( void *pvParameters )
{

	while(1)
	{
		const char *mess = "Task 1 is running";

		BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)mess,CENTER_MODE);	/*print on the LCD screen*/

		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(1000));
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(1000));

		//debugPrintln(&huart1, "Hello World !"); 	/*print full line on a serial terminal*/
		printf("Hello World !\r\n");
	}
}

/******************************************/
/*Gyroscope calibration*/








