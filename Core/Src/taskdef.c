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

static void prvAutoReloadTimerCallback( TimerHandle_t xTimer )
{
		TickType_t xTimeNow;
		 /* Obtain the current tick count. */
		 xTimeNow = uxTaskGetTickCount();
		 /* Output a string to show the time at which the callback was executed. */
		 vPrintStringAndNumber( "Auto-reload timer callback executing", xTimeNow );
		 ulCallCount++;
}



/* La tâche 1A prend les données de l'accéléromètre et du gyromètre toutes les : 1000/100 = 10 ms */
#define timerAcqAG pdMS_TO_TICKS( 10 )
/* La tâche 1B prend les données du magnetomètre et du baromètre toutes les : 1000/50 = 20 ms */
#define timerAcqMB pdMS_TO_TICKS( 20 )

TimerHandle_t xAutoReloadTimer;
BaseType_t xTimer1Started, xTimer2Started;

/* Create the auto-reload timer, storing the handle to the created timer in xAutoReloadTimer. */
xAutoReloadTimerAcqAG = xTimerCreate(

		/* Text name for the software timer - not used by FreeRTOS. */
		"Timer Accel - Gyro",
		/* The software timer's period in ticks. */
		timerAcqAG,
		/* Setting uxAutoRealod to pdTRUE creates an auto-reload timer. */
		pdTRUE,
		/* This example does not use the timer id. */
		0,
		/* The callback function to be used by the software timer being created. */
		prvAutoReloadTimerCallback

);

/* Create the auto-reload timer, storing the handle to the created timer in xAutoReloadTimer. */
xAutoReloadTimerAcqMB = xTimerCreate(

		/* Text name for the software timer - not used by FreeRTOS. */
		"Timer Magne - Baro",
		/* The software timer's period in ticks. */
		timerAcqMB,
		/* Setting uxAutoRealod to pdTRUE creates an auto-reload timer. */
		pdTRUE,
		/* This example does not use the timer id. */
		0,
		/* The callback function to be used by the software timer being created. */
		prvAutoReloadTimerCallback

);

/* Sémaphore gérant l'accès au bus I2C */
SemaphoreHandle_t SemB0;

/* Queue pour échanger les données entre */
QueueHandle_t Queue;



/******************************************/
/*Tâche de test*/
void vTask0( void *pvParameters )
{
	QAngle = xQueueCreate(10, sizeof(double) * 3);
	if(QAngle == NULL) {
		printf("Erreur de création du message queue angle");
		exit(1);
	}

	QAltitude = xQueueCreate(10, sizeof(double));
	if(QAltitude == NULL) {
		printf("Erreur de création du message queue altitude");
		exit(1);
	}

	QAcceleration = xQueueCreate(20, sizeof(double));
	if(QAcceleration == NULL) {
		printf("Erreur de création du message queue accélération");
		exit(1);
	}

	QChampMagnetique = xQueueCreate(10, sizeof(double));
	if(QChampMagnetique == NULL) {
		printf("Erreur de création du message queue champ magnétique");
		exit(1);
	}

	QPression = xQueueCreate(10, sizeof(double));
	if(QPression == NULL) {
		printf("Erreur de création du message queue pression");
		exit(1);
	}

	QVitesseAngulaire = xQueueCreate(20, sizeof(double));
	if(QVitesseAngulaire == NULL) {
		printf("Erreur de création du message queue vitesse angulaire");
		exit(1);
	}

	xTaskCreate(vTask1a, "task 1A", 1000, NULL, 3, NULL);
	printf("task 1A created \r\n");

	xTaskCreate(vTask1b, "task 1B", 1000, NULL, 3, NULL);
	printf("task 1B created \r\n");
	/*
	xTaskCreate(vTask2a, "task 2A", 1000, NULL, 4, NULL);
	printf("task 2A created \r\n");

	xTaskCreate(vTask2b, "task 2B", 1000, NULL, 4, NULL);
	printf("task 2B created \r\n");

	xTaskCreate(vTask3, "task 3", 1000, NULL, 5, NULL);
	printf("task 3 created \r\n");*/

	vTaskDelete(NULL);


	/*
	while(1)
	{
		const char *mess = "Task 1 is running";

		BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)mess,CENTER_MODE);	/*print on the LCD screen*/

		/*
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(1000));
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(1000));

		//debugPrintln(&huart1, "Hello World !"); 	/*print full line on a serial terminal*/
		/*
		printf("Hello World !\r\n");
	}*/
}

void vTask1a( void *pvParameters )
{
	while(1)
		{
			if( ( xAutoReloadTimerAcqAG != NULL ) )
			 {
				/* Start the software timers, using a block time of 0 (no block time). The scheduler has
				 not been started yet so any block time specified here would be ignored anyway. */
				 xTimer1Started = xTimerStart( xAutoReloadTimer, 0 );

				 if( ( xTimer1Started == pdPASS ) )
				  {
					 printf("Task 1a created !\r\n");
					 /* Start the scheduler. */
					 vTaskStartScheduler();
				  }
			 }
		}
}

void vTask1b( void *pvParameters )
{

	while(1)
	{
			if( ( xAutoReloadTimerAcqMB != NULL ) )
			{
				/* Start the software timers, using a block time of 0 (no block time). The scheduler has
				not been started yet so any block time specified here would be ignored anyway. */
				xTimer2Started = xTimerStart( xAutoReloadTimer, 0 );

				if( ( xTimer1Started == pdPASS ) )
				{
					printf("Task 1b created !\r\n");
						/* Start the scheduler. */
						vTaskStartScheduler();
				}
			}
	}

}

void vTask2a( void *pvParameters )
{
	Donnees_LCD donnees_temp = Donnees_LCD();
	int i = 0;

	while(xQueueReceive(QChampMagnetique, &(donnees_temp.champ_magnetique), portMAX_DELAY) == pdTRUE) { donneesLCD.champ_magnetique += donnees_temp.champ_magnetique; i++; }
	donneesLCD.champ_magnetique /= i;

	while(xQueueReceive(QAcceleration, &(donnees_temp.acceleration), portMAX_DELAY) == pdTRUE) { donneesLCD.acceleration += donnees_temp.acceleration; i++; }
	donneesLCD.acceleration /= i;

	while(xQueueReceive(QVitesseAngulaire, &(donnees_temp.vitesse_angulaire), portMAX_DELAY) != pdTRUE) { donneesLCD.vitesse_angulaire += donnees_temp.vitesse_angulaire; i++; }
	donneesLCD.vitesse_angulaire /= i;

	donneesLCD.angle[0] = atan(donneesLCD.acceleration[0] / sqrt(pow(donneesLCD.acceleration[1],2) + pow(donneesLCD.acceleration[2],2) ) );
	donneesLCD.angle[1] = atan(donneesLCD.acceleration[1] / sqrt(pow(donneesLCD.acceleration[0],2) + pow(donneesLCD.acceleration[2],2) ) );
	donneesLCD.angle[2] = atan(donneesLCD.acceleration[2] / sqrt(pow(donneesLCD.acceleration[1],2) + pow(donneesLCD.acceleration[0],2) ) );

	xQueueSend(QAngle, &donneesLCD.angle, portMAX_DELAY);

}

void vTask2b( void *pvParameters )
{
	Donnees_LCD donnees_temp = Donnees_LCD();
	int i = 0;
	while(xQueueReceive(QPression, &donnees_temp.pression, portMAX_DELAY) == pdTRUE) { donneesLCD.pression += donnees_temp; i++; }
	donneesLCD.pression /= i;

	donneesLCD.altitude = 44330 * (1 - pow(donneesLCD.pression / 101325.0, 1.0/5.255)) ; // Formule de conversion pression vers altitude

	xQueueSend(QAltitude, &donnees_temp.altitude, portMAX_DELAY);

}

void vTask3( void *pvParameters )
{

	// On fait d'abord la moyenne de toutes les valeurs reçues

	Donnees_LCD donnees_temp = Donnees_LCD();



	while(xQueueReceive(QAltitude, &(donnees_temp.altitude), portMAX_DELAY) == pdTRUE) { donneesLCD.altitude += donnees_temp.altitude; }
	donneesLCD.altitude /= 10;

	while(xQueueReceive(QAngle, &(donnees_temp.angle), portMAX_DELAY) == pdTRUE) {
		donneesLCD.angle[0] += donnees_temp.angle[0];
		donneesLCD.angle[1] += donnees_temp.angle[1];
		donneesLCD.angle[2] += donnees_temp.angle[2];
	}
	donneesLCD.angle[0] /= 10;
	donneesLCD.angle[1] /= 10;
	donneesLCD.angle[2] /= 10;

	while(xQueueReceive(QAltitude, &(donnees_temp.altitude), portMAX_DELAY) == pdTRUE) { donneesLCD.altitude += donnees_temp.altitude;  }
	donneesLCD.altitude /= 10;

	while(xQueueReceive(QTemperature, &(donnees_temp.temperature), portMAX_DELAY) == pdTRUE) {
	   donneesLCD.temperature += donnees_temp.temperature;
	}
	donneesLCD.temperature /= 10;


	// Afficher sur le LCD
	GUI(donneesLCD.angle[0], donneesLCD.angle[1], donneesLCD.angle[2], donneesLCD.altitude, donneesLCD.temperature);

	vTaskDelete(NULL);

}

/******************************************/
/*Gyroscope calibration*/








