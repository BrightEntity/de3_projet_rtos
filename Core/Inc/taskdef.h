/*
 * taskdef.h
 *
 *  Created on: Jan 20, 2021
 *      Author: florentgoutailler
 */

#ifndef TASKDEF_H_
#define TASKDEF_H_

//extern variable
extern uint32_t RTOS_RunTimeCounter;

//Tasks
void vTask0( void *);
void vTask1a( void *);
void vTask1b( void *);
void vTask2a( void *);
void vTask2b( void *);
void vTask3( void *);

QueueHandle_t QAngle;
QueueHandle_t QAltitude;
QueueHandle_t QPression;
QueueHandle_t QChampMagnetique;
QueueHandle_t QAcceleration;
QueueHandle_t QVitesseAngulaire;
QueueHandle_t QTemperature;

typedef struct Donnees_LCD {
	double angle[3];
	double altitude;
	double pression;
	double champ_magnetique;
	double acceleration;
	double vitesse_angulaire;
	double temperature;
} Donnees_LCD;

Donnees_LCD donneesLCD;


#endif /* TASKDEF_H_ */
